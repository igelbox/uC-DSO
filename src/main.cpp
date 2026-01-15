#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include ".config.hpp"
#include "assert.hpp"
#include "esp32-hal-adc-hack.hpp"
#include "protocol.hpp"
#include "serio.hpp"

#include <variant>

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

void client_write(AsyncClient &client, const char *data, size_t size) {
  for (;;) {
    const auto r = client.write(data, size);
    size -= r;
    if (!size)
      break;
    delay(10);
    Serial.printf(":%d", r);
    data += r;
  }
}

void handle_acquire(AsyncClient &client) {
  const auto count = sizeof(samples) / 2;

  protocol::AcquireHeader header{0, 1, (uint64_t)1e15 / 83333, 0, 83333 / 1e6};
  client_write(client, (const char *)&header, sizeof(header));

  protocol::ChannelHeader channel{0, count, 0.750f / 4095.f, 0.f, (0.f), false};
  client_write(client, (const char *)&channel, sizeof(channel));
  client_write(client, (const char *)samples, 2 * count);
}

void handle_command(AsyncClient &client, const char *command) {
  for (const auto &[prefix, handler] : COMMANDS) {
    if (const auto offset = scpi::match_args_offset(prefix, strlen(prefix), command)) {
      struct Visitor {
        const char *const args;
        AsyncClient &client;
        void operator()(const CommandHandler<void> &handler) const { handler(args); }
        void operator()(const CommandHandler<const char *> &handler) const {
          client.write(handler(args));
          static const char N{'\n'};
          client.write(&N, 1);
        }
      };
      std::visit(Visitor{command + offset, client}, handler);
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

  server.onClient(
      [](void *arg, AsyncClient *client) {
        Serio << "connected: " << (void *)client << ' ' << client->remoteIP() << ':' << client->remotePort();
        using Buffer = std::string;
        const auto buffer = new Buffer();
        try {
          client->onDisconnect(
              [](void *arg, AsyncClient *client) {
                Serio << "disconnected: " << (void *)client;
                delete (Buffer *)arg;
                delete client;
              },
              (void *)buffer);
        } catch (...) {
          delete buffer;
          throw;
        }
        client->onData(
            [](void *arg, AsyncClient *client, void *data, size_t len) {
              // Serio << "received " << (void *)client << ' ' << data << ' ' << len;
              const auto chars = (const char *)data;
              auto &buffer = *(Buffer *)arg;
              buffer.append(chars, len);

              protocol::handle(
                  buffer, [&]() { handle_acquire(*client); },
                  [&](const auto &command) { handle_command(*client, command); });
              if (!buffer.empty()) {
                Serio << "buffer: " << buffer;
              }
            },
            (void *)buffer);
        client->onError(
            [](void *arg, AsyncClient *client, int8_t error) { Serio << "errored " << (void *)client << ' ' << error; },
            nullptr);
      },
      nullptr);

  server.begin();
  Serio << "Server started :" << server._port;
}

void loop() {
  Serial.print(analogContinuousReadSamples(samples[current], 100) == 1023 ? '.' : '!');
  current = (current + 1) % 4;
}
