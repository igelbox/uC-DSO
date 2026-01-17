#pragma once

namespace triggers {

template <typename Index, typename Sample> struct Rising {
  const Sample level;
  Sample last;
  Index index = 0;

  Rising(Sample level, Sample last) : level(level), last(last) {}
  bool process(const Sample *samples, Index count, Index offset) {
    for (uint16_t i = 0; i < count; ++i) {
      const auto s = *samples++;
      if ((last < level) && (s >= level)) {
        index = i + offset;
        return true;
      }
      last = s;
    }
    return true;
  }
};

} // namespace triggers
