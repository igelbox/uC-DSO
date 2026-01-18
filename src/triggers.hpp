#pragma once

namespace triggers {

template <typename Sample, typename Index> struct Rising {
  const Sample level;
  const Index stride;
  Sample last;
  Index index = 0;

  Rising(Sample level, Sample last, Index stride) : level(level), stride(stride), last(last) {}
  bool process(const Sample *samples, Index count, Index offset) {
    for (uint16_t i = 0; i < count; ++i) {
      const auto s = *samples;
      samples += stride;
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
