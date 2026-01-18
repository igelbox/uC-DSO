#pragma once

#include <AsyncTCP.h>

#include "serio.hpp"

namespace tcp {
namespace {
SerIO &operator<<(SerIO &serio, AsyncClient *value) { return serio << (void *)value; }
} // namespace

struct Client {
  Client(AsyncClient &async) : async(async) {}

protected:
  AsyncClient &async;
  std::string output;

  char *advance(size_t size) {
    const auto sz = output.size();
    output.resize(sz + size);
    return output.data() + sz;
  }
  void write(const char *data, size_t size) {
    if (!output.empty()) {
      output.append(data, size);
    } else if (const auto written = async.write(data, size); written != size) {
      output.append(data + written, size - written);
    }
  }
  void write(const char *data) { write(data, strlen(data)); }
  void flush() {
    if (!output.empty()) {
      if (const auto written = async.write(output.data(), output.size())) {
        output.erase(0, written);
      }
    }
  }

  template <typename Client> friend void onClient(void *, AsyncClient *);
};

template <typename Client> void onClient(void *, AsyncClient *async) {
  Serio << "connected: " << async << ' ' << async->remoteIP() << ':' << async->remotePort();
  const auto client = new Client(*async);
  try {
    async->onDisconnect(
        [](void *arg, AsyncClient *async) {
          const auto client = (Client *)arg;
          Serio << "disconnected: " << async;
          delete client;
          delete async;
        },
        (void *)client);
  } catch (...) {
    delete client;
    throw;
  }
  const auto arg = (void *)client;
  async->onData([](void *arg, auto, void *data, size_t len) { ((Client *)arg)->data(data, len); }, arg);
  async->onAck([](void *arg, auto, size_t, uint32_t) { ((Client *)arg)->flush(); }, arg);
  async->onError([](auto, AsyncClient *async, int8_t error) { Serio << "errored: " << async << ' ' << error; }, arg);
  async->onTimeout([](auto, AsyncClient *async, int32_t time) { Serio << "timeout: " << async << ' ' << time; }, arg);
}

} // namespace tcp
