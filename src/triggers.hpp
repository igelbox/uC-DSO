#pragma once

namespace triggers {

enum class Direction { kRising, kFalling, kAny };
template <typename Sample, typename Index> struct Edge {
  const Direction direction;
  const Sample level;
  const Index stride;
  Sample last;
  Index index = 0;

  Edge(Direction direction, Sample level, Sample last, Index stride)
      : direction(direction), level(level), stride(stride), last(last) {}
  bool process(const Sample *samples, Index count, Index offset) {
    for (uint16_t i = 0; i < count; ++i) {
      const auto s = *samples;
      samples += stride;
      if (((direction != Direction::kFalling) && (last < level) && (s >= level)) ||
          ((direction != Direction::kRising) && (last > level) && (s <= level))) {
        index = i + offset;
        return true;
      }
      last = s;
    }
    return true;
  }
};

} // namespace triggers
