#include <gtest/gtest.h>

#include "scpi.hpp"

namespace {
static const std::string E;

auto handle(std::string buffer) {
  std::vector<std::string> commands;
  const auto result = scpi::handle(buffer, [&](const char *cmd) { commands.push_back(cmd); });
  return std::tuple{result, commands, buffer};
}
using Result = decltype(handle(E));

auto match_args_offset(const char *prefix, const char *command) {
  const auto offset = scpi::match_args_offset(prefix, strlen(prefix), command);
  return offset ? command + offset : nullptr;
}
} // namespace

TEST(Scpi, match_args_offset) {
  EXPECT_STREQ(match_args_offset("CMDA", "CMD"), nullptr) << "partial mismatch";

  EXPECT_STREQ(match_args_offset("CMD\n", "CMDA"), nullptr) << "full mismatch";

  EXPECT_STREQ(match_args_offset("CMD ", "CMD A,B"), "A,B") << "partial match";

  EXPECT_STREQ(match_args_offset("CMD\n", "CMD"), "") << "full match";
}

TEST(Scpi, handle) {
  EXPECT_EQ(handle(E), Result(false, {}, "")) << "none";

  EXPECT_EQ(handle(E + "\n" + "CMD\n"), Result(true, {""}, "CMD\n")) << "empty";

  EXPECT_EQ(handle(E + "CMD\n" + "suffix"), Result(true, {"CMD"}, "suffix")) << "command";
}
