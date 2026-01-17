#include <variant>

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

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
static uint16_t samples[SAMPLES_COUNT + MAX_CPP], cursor = 0, upper = SAMPLES_COUNT;
static uint16_t depth = 500;
static size_t rate = MAX_RATE, nrate = rate;
static float trig_level = 0;

struct {
  bool begin() {
    uint8_t pin = 0;
    const auto cpp = min((95 * rate) / 1000, MAX_CPP);
    Serio << "adc: rate=" << rate << " cpp=" << cpp;
    return CHECK(analogContinuous(&pin, 1, cpp, rate, nullptr)) //
           && analogContinuousStart();
  }
  bool restart() {
    return CHECK(analogContinuousStop())      //
           && CHECK(analogContinuousDeinit()) //
           && begin();
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
        {"RATE ", [](const char *arg) { nrate = min((long long)MAX_RATE, atoll(arg)); }},
        {"DEPTHS?\n", [](const auto) { return "500,1000,2000,4000,5000,10000,20000,"; }},
        {"DEPTH ", [](const char *arg) { depth = atoll(arg); }},
        {"TRIG:LEV ", [](const char *arg) { trig_level = atoff(arg); }},
    };

struct ScopeClient : tcp::Client {
  std::string input;
  ScopeClient(AsyncClient &async) : tcp::Client(async) {}
  void data(void *data, size_t size) {
    input.append((const char *)data, size);
    protocol::handle(input, [&]() { handle_acquire(); }, [&](const auto &cmd) { handle_command(cmd); });
  }

protected:
  void handle_acquire();
  void handle_command(const char *command);
};

void ScopeClient::handle_acquire() {
  protocol::AcquireHeader header{0, 1, (uint64_t)1e15 / rate, 0, rate / 1e6};
  write((const char *)&header, sizeof(header));

  const auto count = depth;
  const auto cur = cursor;
  uint16_t *a, as, *b, bs;
  if (const auto prev = count - cur; count > cur) {
    a = &samples[upper - prev], as = prev;
    b = &samples[0], bs = cur;
  } else {
    a = &samples[cur - count], as = count;
    bs = 0;
  }
  static const auto scale = 0.750f / 4095.f;
  triggers::Rising<uint16_t, uint16_t> trigger((int)constrain(trig_level / scale, 0, 4095), *a);
  if (!trigger.process(a, as, 0))
    trigger.process(b, bs, as);

  protocol::ChannelHeader channel{0, count, scale, 0.f, -1e15f * trigger.index / rate, false};
  write((const char *)&channel, sizeof(channel));
  write((const char *)a, 2 * as);
  if (bs)
    write((const char *)b, 2 * bs);
}

void ScopeClient::handle_command(const char *command) {
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

  analogContinuousSetAtten(ADC_0db);
  ASSERT(adc.begin());

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

  server.onClient(tcp::onClient<ScopeClient>, nullptr);
  server.setNoDelay(true);
  server.begin();
  Serio << "Server started :" << server._port;
}

auto last_report = millis();
void loop() {
  const auto time = millis();
  if ((time - last_report) > 5000) {
    last_report = time;
    Serio << "heap=" << (100 * ESP.getFreeHeap() / ESP.getHeapSize()) << '%' << (ESP.getFreeHeap() / 1024) << 'k'
          << (ESP.getMinFreeHeap() / 1024);
  }

  if (rate != nrate) {
    rate = nrate;
    CHECK(adc.restart());
  }
  if (cursor += analogContinuousReadSamples(samples + cursor, MAX_CPP, 100); cursor > SAMPLES_COUNT) {
    upper = cursor;
    cursor = 0;
  }
}
