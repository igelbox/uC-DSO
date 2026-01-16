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

template <typename R> using CommandHandler = R (*)(const char *args);
std::initializer_list<std::pair<const char *, std::variant<                     //
                                                  CommandHandler<const char *>, //
                                                  CommandHandler<void>          //
                                                  >>>
    COMMANDS{
        {"*IDN?\n", [](const auto) { return "igelbox,EScoPe,1,0.1"; }},
        {"RATES?\n", [](const auto) { return "83333,"; }},
        {"DEPTHS?\n", [](const auto) { return "1023,"; }},
    };

static size_t current = 0;
static adc_continuous_samples_t samples[4];

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
  const auto count = sizeof(samples) / 2;

  protocol::AcquireHeader header{0, 1, (uint64_t)1e15 / 83333, 0, 83333 / 1e6};
  write((const char *)&header, sizeof(header));

  protocol::ChannelHeader channel{0, count, 0.750f / 4095.f, 0.f, (0.f), false};
  write((const char *)&channel, sizeof(channel));
  write((const char *)samples, 2 * count);
}

void ScopeClient::handle_command(const char *command) {
  for (const auto &[prefix, handler] : COMMANDS) {
    if (const auto offset = scpi::match_args_offset(prefix, strlen(prefix), command)) {
      struct Visitor {
        const char *const args;
        ScopeClient &client;
        void operator()(const CommandHandler<void> &handler) const { handler(args); }
        void operator()(const CommandHandler<const char *> &handler) const {
          client.write(handler(args));
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

  uint8_t pin = 0;
  ASSERT(analogContinuous(&pin, 1, 4092 / SOC_ADC_DIGI_RESULT_BYTES, 83333, nullptr));
  ASSERT(analogContinuousStart());

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

void loop() {
  Serial.print(analogContinuousReadSamples(samples[current], 100) == 1023 ? '.' : '!');
  current = (current + 1) % 4;
}
