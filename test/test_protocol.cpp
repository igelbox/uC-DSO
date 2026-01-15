#include <gtest/gtest.h>

#include "protocol.hpp"

namespace {
static const std::string E;

auto handle(std::string buffer) {
  std::vector<std::string> commands;
  protocol::handle(buffer, [&]() { commands.push_back("ACQ"); }, [&](const char *cmd) { commands.push_back(cmd); });
  return std::pair{commands, buffer};
}
using Result = decltype(handle(E));
} // namespace

TEST(Protocol, handle) {
  EXPECT_EQ(handle(E), Result({}, "")) << "empty";

  EXPECT_EQ(handle(E + 'K' + 'K' + "CMK\n" + 'K' + "KMD\n" + 'K' + "suffix"),
            Result({"ACQ", "ACQ", "CMK", "ACQ", "ACQ", "MD", "ACQ"}, "suffix"));
}
