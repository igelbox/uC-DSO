#include <optional>
#include <variant>

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#define SERIO_IMPLEMENTATION
#include ".config.hpp"
#include "assert.hpp"
#include "esp32-hal-adc-hack.hpp"
#include "protocol.hpp"
#include "serio.hpp"
#include "tcp.hpp"
#include "triggers.hpp"

#define MAX_CPP (4092u / SOC_ADC_DIGI_RESULT_BYTES)
#define SAMPLES_COUNT 20000
#define MAX_RATE SOC_ADC_SAMPLE_FREQ_THRES_HIGH
static const uint64_t FEMTOS = 1e15;

static uint16_t samples[SAMPLES_COUNT + MAX_CPP];
struct {
  uint16_t cursor = 0;
  uint16_t upper = SAMPLES_COUNT;
} loop_thread_vars;

static uint16_t depth = 500;
static triggers::Direction trig_dir = triggers::Direction::kRising;
static float trig_level = 0;
static uint8_t trig_sou = 0;

struct Config {
  static constexpr uint8_t PINS[] = {0, 1, 3, 4}; // pin 2 is pulled up whatever I do
  size_t rate = MAX_RATE;
  adc_attenuation_t atten = ADC_0db;
  uint8_t enabled = 0x01;
  bool is_enabled(uint8_t index) const { return enabled & (1 << index); }
  void set_enabled(uint8_t index, bool value) { enabled = value ? enabled | (1 << index) : enabled & ~(1 << index); }
  auto operator<=>(const Config &) const = default;
} config, nconfig = config;

static float ranges[sizeof(Config::PINS)] = {5.f};
static float offsets[sizeof(Config::PINS)] = {0.f};

using Range = std::pair<adc_attenuation_t, float>;
static std::array<Range, ADC_ATTENDB_MAX> RANGES{
    Range{ADC_0db, 0.f /*uninitialized*/},
    Range{ADC_2_5db, 0.f},
    Range{ADC_6db, 0.f},
    Range{ADC_11db, 0.f},
};
auto recalc_atten(const Config &config) {
  float top = 0.f;
  for (uint8_t i = 0; i < sizeof(Config::PINS); ++i) {
    if (config.is_enabled(i)) {
      top = std::max(top, ranges[i] / 2.f + offsets[i]);
    }
  }
  adc_attenuation_t result;
  for (const auto &[atten, max] : RANGES) {
    result = atten;
    if (max >= top) {
      break;
    }
  }
  return result;
}
struct {
  bool begin() {
    auto log = Serio << "att:";
    for (auto &[atten, max] : RANGES) {
      analogContinuousSetAtten(atten);
      uint16_t mvolts;
      if (!(CHECK(analogContinuous(Config::PINS, 1, 64, config.rate, nullptr)) &&
            CHECK(analogSampleToMVolts((1 << SOC_ADC_DIGI_MAX_BITWIDTH) - 1, mvolts)) //
            && CHECK(analogContinuousDeinit()) && CHECK(analogContinuousDeinitFix()))) {
        return false;
      }
      max = mvolts / 1e3;
      log << ' ' << atten << '=' << max;
    }
    return true;
  }
  bool start() {
    uint8_t pins[sizeof(Config::PINS)];
    size_t count = 0;
    for (uint8_t i = 0; i < sizeof(Config::PINS); ++i) {
      if (config.is_enabled(i)) {
        pins[count++] = Config::PINS[i];
      }
    }
    analogContinuousSetAtten(config.atten);
    const auto cpp = min((95 * config.rate) / 1000, MAX_CPP);
    Serio << "adc: rate=" << config.rate << " count=" << count << " cpp=" << cpp << " att=" << config.atten;
    return CHECK(analogContinuous(pins, count, cpp / count, config.rate, nullptr)) //
           && analogContinuousStart();
  }
  bool restart() {
    return CHECK(analogContinuousStop())      //
           && CHECK(analogContinuousDeinit()) //
           && CHECK(analogContinuousDeinitFix()) && start();
  }
} adc;

template <typename R> using CommandHandler = R (*)(const char *args);
std::initializer_list<std::pair<const char *, std::variant<                     //
                                                  CommandHandler<std::string>,  //
                                                  CommandHandler<const char *>, //
                                                  CommandHandler<void>          //
                                                  >>>
    COMMANDS{
        {"*IDN?\n", [](const auto) { return "igelbox,EScoPe,1,0.1"; }},
        {"RATES?\n",
         [](const auto) {
           std::string result;
           for (const auto interval : {30, 40, 50, 80, 100, 200, 250, 400, 500, 800, 1000, 2000, 2500, 4000}) {
             const auto rate = 5'000'000 / 2 / interval;
             result += std::to_string(rate) + ',';
           }
           return result;
         }},
        {"RATE ", [](const char *arg) { nconfig.rate = min((long long)MAX_RATE, atoll(arg)); }},
        {"DEPTHS?\n", [](const auto) { return "500,1000,2000,4000,5000,10000,20000,"; }},
        {"DEPTH ", [](const char *arg) { depth = atoll(arg); }},
        {"TRIG:LEV ", [](const char *arg) { trig_level = atoff(arg); }},
        {"TRIG:SOU CHAN", [](const char *arg) { trig_sou = *arg - '1'; }},
        {"TRIG:EDGE:DIR ",
         [](const char *arg) {
           if (strcmp(arg, "RISING") == 0) {
             trig_dir = triggers::Direction::kRising;
           } else if (strcmp(arg, "FALLING") == 0) {
             trig_dir = triggers::Direction::kFalling;
           } else if (strcmp(arg, "ANY") == 0) {
             trig_dir = triggers::Direction::kAny;
           } else {
             Serio << "TRIG:EDGE:DIR " << arg;
           }
         }},
        {":CHAN",
         [](const auto *arg) {
           auto c = arg;
           const auto index = *c++ - '1';
           if (!CHECK_VA(*c++ == ':', arg))
             return;
           if (const auto on = strcmp(c, "ON") == 0; on || (strcmp(c, "OFF") == 0)) {
             nconfig.set_enabled(index, on);
           } else if (strncmp(c, "RANGE ", 6) == 0) {
             ranges[index] = atoff(c + 6);
           } else if (strncmp(c, "OFFS ", 5) == 0) {
             offsets[index] = atoff(c + 5);
           } else {
             Serio << ":CHAN" << arg;
           }
           nconfig.atten = recalc_atten(nconfig);
         }},
    };

template <typename Writer> struct ScopeClient : tcp::Client<Writer> {
  using Base = tcp::Client<Writer>;
  std::string input;
  ScopeClient(Writer &writer) : Base(writer) {}
  void data(void *data, size_t size) {
    input.append((const char *)data, size);
    protocol::handle(input, [&]() { handle_acquire(); }, [&](const auto &cmd) { handle_command(cmd); });
  }

protected:
  uint32_t seqnum = 0;
  void handle_acquire();
  void handle_command(const char *command);
};

template <typename T> auto sacpy(T *dst, const T *src, size_t count, size_t stride) {
  for (; count; --count) {
    *dst++ = *src;
    src += stride;
  }
  return dst;
}

template <typename Writer> void ScopeClient<Writer>::handle_acquire() {
  uint8_t toffs = 0, nchan = 0;
  for (uint8_t i = 0; i < sizeof(Config::PINS); ++i) {
    if (config.is_enabled(i)) {
      if (i == trig_sou) {
        toffs = nchan;
      }
      ++nchan;
    }
  }
  // Serio << "nc=" << nchan << " to=" << toffs;
  Base::output.reserve(sizeof(protocol::AcquireHeader)           //
                       + nchan * sizeof(protocol::ChannelHeader) //
                       + depth * 2);
  const auto icap = Base::output.capacity();
  protocol::AcquireHeader header{seqnum++, nchan, FEMTOS * nchan / config.rate, 0, config.rate / 1e6 / nchan};
  Base::write((const char *)&header, sizeof(header));

  const auto [cursor, upper] = loop_thread_vars; // stable copy
  // Serio << depth << '/' << cursor << '/' << upper;
  const uint16_t cdepth = depth / nchan, tsize = cdepth * nchan;
  // Serio << cdepth << '/' << tsize;
  uint16_t *a, asize, *b, bsize;
  if (const auto prev = tsize - cursor; tsize > cursor) {
    a = &samples[upper - prev], asize = prev / nchan;
    b = &samples[0], bsize = cursor / nchan;
  } else {
    a = &samples[cursor - tsize], asize = tsize / nchan;
    bsize = 0;
  }
  // Serio << "as=" << asize << " bs=" << bsize;
  static const auto scale = 1e-3f;
  triggers::Edge<uint16_t, uint16_t> trigger(trig_dir, constrain(trig_level / scale, 0, 4095), *(a + toffs), nchan);
  if (!trigger.process(a + toffs, asize, 0))
    trigger.process(b + toffs, bsize, asize);
  // Serio << "tl=" << trigger.level << " ti=" << trigger.index;

  uint8_t coffset = 0;
  for (uint8_t i = 0; i < sizeof(Config::PINS); ++i) {
    if (config.is_enabled(i)) {
      protocol::ChannelHeader channel{i, cdepth, scale, 0.f, -1e15f * trigger.index / config.rate * nchan, false};
      Base::write((const char *)&channel, sizeof(channel));

      auto data = (uint16_t *)Base::advance(channel.memdepth * 2);
      // Serio << "oc=" << output.capacity() << " he=" << (ESP.getFreeHeap() / 1024) << 'k';
      data = sacpy(data, a + coffset, asize, nchan);
      data = sacpy(data, b + coffset, bsize, nchan);
      ++coffset;
    }
  }
  CHECK_VA(Base::output.capacity() == icap, "%d,%d", Base::output.capacity(), icap);
  Base::flush();
}

template <typename Writer> void ScopeClient<Writer>::handle_command(const char *command) {
  for (const auto &[prefix, handler] : COMMANDS) {
    if (const auto offset = scpi::match_args_offset(prefix, strlen(prefix), command)) {
      struct Visitor {
        const char *const args;
        ScopeClient &client;
        void operator()(const CommandHandler<void> &handler) const { handler(args); }
        void operator()(const CommandHandler<const char *> &handler) const { reponse(handler(args)); }
        void operator()(const CommandHandler<std::string> &handler) const { reponse(handler(args).c_str()); }
        void reponse(const char *response) const {
          client.write(response);
          static const char N{'\n'};
          client.write(&N, 1);
        }
      };
      std::visit(Visitor{command + offset, *this}, handler);
      return;
    }
  }

  Serio << "unknown: '" << command << '\'';
}

struct : public AsyncServer {
  using AsyncServer::_port;
} server(5555);
void setup() {
  setCpuFrequencyMhz(80); // reduce heating a bit
  delay(500);             // give a time to connect monitor
  Serial.begin(115200);

  ASSERT(adc.begin());
  ASSERT(adc.start());

  {
    auto log = Serio << "Connecting to " << WIFI_SSID << ": ";
    for (auto status = WiFi.begin(WIFI_SSID, WIFI_PASW); status != WL_CONNECTED; status = WiFi.status()) {
      log << status;
      delay(1000);
    }
    log << ' ' << WiFi.localIP();
    CHECK(WiFi.setTxPower(WIFI_POWER_2dBm));
    log << " P" << WiFi.getTxPower();
  }

  CHECK(MDNS.begin("escope"));
  Serio << "mDNS started";

  server.onClient(tcp::onClient<ScopeClient<AsyncClient>>, nullptr);
  server.setNoDelay(true);
  server.begin();
  Serio << "Server started :" << server._port;
}

struct SerialScopeClient : ScopeClient<decltype(Serial)> {
  SerialScopeClient(Writer &writer) : ScopeClient<Writer>(writer) {
    SerIO::prelude = [](SerIO &log) { log << "// "; };
    server.end();
    MDNS.end();
    CHECK(WiFi.disconnect(true, true));
  }
};
std::optional<SerialScopeClient> slient;
auto last_report = millis();
void loop() {
  if (Serial.available()) {
    auto &client = slient ? *slient : slient.emplace(Serial);
    char buffer[1024];
    client.data(buffer, Serial.readBytes(buffer, sizeof(buffer)));
  }
  const auto time = millis();
  if ((time - last_report) > 5000) {
    last_report = time;
    Serio << "heap=" << (100 * ESP.getFreeHeap() / ESP.getHeapSize()) << '%' << (ESP.getFreeHeap() / 1024) << 'k'
          << (ESP.getMinFreeHeap() / 1024);
  }

  if (config != nconfig) {
    config = nconfig;
    CHECK(adc.restart());
  }
  auto &[cursor, upper] = loop_thread_vars;
  if (cursor += analogContinuousReadSamples(samples + cursor, MAX_CPP, 100); cursor > SAMPLES_COUNT) {
    upper = cursor;
    cursor = 0;
  }
}
