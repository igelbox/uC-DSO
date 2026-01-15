#pragma once

#include <string>

namespace scpi {

template <typename Length> auto match_args_offset(const char *prefix, Length length, const char *command) {
  const auto is_full = length && (prefix[length - 1] == '\n');
  length -= is_full;
  // Serio << '"' << prefix << '"' << length << is_full << ':' << strncmp(command, prefix, length);
  return strncmp(command, prefix, length) || (is_full && (command[length] != '\0')) // mismatch
             ? 0
             : length;
}

template <typename CommandHandler> bool handle(std::string &buffer, const CommandHandler &command) {
  const auto i = buffer.find('\n');
  if (i == std::string::npos) {
    return false;
  }
  buffer[i] = 0;
  command(buffer.c_str());
  buffer.erase(0, i + 1);
  return true;
}

} // namespace scpi
