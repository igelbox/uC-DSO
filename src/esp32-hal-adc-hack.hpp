#pragma once

#include <cstdint>
#include <soc/soc_caps.h>

uint16_t analogContinuousReadSamples(uint16_t *samples, uint16_t count, uint32_t timeout_ms);
