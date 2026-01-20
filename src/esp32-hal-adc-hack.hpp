#pragma once

#include <cstdint>

uint16_t analogContinuousReadSamples(uint16_t *samples, uint16_t count, uint32_t timeout_ms);
bool analogSampleToMVolts(uint16_t sample, uint16_t &mvolts);
bool analogContinuousDeinitFix();
