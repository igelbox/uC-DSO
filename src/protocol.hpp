#pragma once

#include "scpi.hpp"

namespace protocol {

struct __attribute__((packed)) AcquireHeader {
  uint32_t seqnum;
  uint16_t num_channels;
  uint64_t fs_per_sample;
  uint64_t trigger_fs;
  double rate; // de-facto hardware capture rate.
};

struct __attribute__((packed)) ChannelHeader {
  uint8_t chnum;
  uint64_t memdepth;
  float scale, offset, trigphase;
  bool clipping;
};

template <typename AcquireHandler> void _acquires(std::string &buffer, const AcquireHandler &acquire) {
  while (buffer.starts_with('K')) {
    buffer.erase(0, 1);
    acquire();
  }
}

template <typename AcquireHandler, typename CommandHandler>
void handle(std::string &buffer, const AcquireHandler &acquire, const CommandHandler &command) {
  _acquires(buffer, acquire);
  while (scpi::handle(buffer, command)) {
    _acquires(buffer, acquire);
  }
}

} // namespace protocol
