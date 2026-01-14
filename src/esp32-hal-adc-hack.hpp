#pragma once

#include <cstdint>
#include <soc/soc_caps.h>

using adc_continuous_samples_t = uint16_t[4092 / SOC_ADC_DIGI_RESULT_BYTES];
uint16_t analogContinuousReadSamples(adc_continuous_samples_t &samples, uint32_t timeout_ms);
