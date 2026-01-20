#include "esp32-hal-adc-hack.hpp"

#include <esp32-hal-adc.h>
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_oneshot.h>

typedef struct {
  voidFuncPtr fn;
  void *arg;
} interrupt_config_t;

typedef struct {
  adc_oneshot_unit_handle_t _;
  adc_continuous_handle_t adc_continuous_handle;
  interrupt_config_t _;
  adc_cali_handle_t adc_cali_handle;
  uint32_t _;
  uint32_t conversion_frame_size;
} adc_handle_t;

extern adc_handle_t adc_handle;

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

uint16_t analogContinuousReadSamples(uint16_t *samples, uint16_t count, uint32_t timeout_ms) {
  if (!adc_handle.adc_continuous_handle) {
    log_e("ADC Continuous is not initialized!");
    return 0;
  }
  uint32_t bytes_read = 0;
  uint8_t adc_read[adc_handle.conversion_frame_size];
  memset(adc_read, 0xcc, sizeof(adc_read));

  if (auto err = adc_continuous_read(adc_handle.adc_continuous_handle, adc_read, adc_handle.conversion_frame_size,
                                     &bytes_read, timeout_ms);
      err != ESP_OK) {
    log_e("Reading data failed with error: %X", err);
    return 0;
  }

  if ((bytes_read / SOC_ADC_DIGI_RESULT_BYTES) > count) {
    log_e("Insufficient buffer");
    return 0;
  }

  size_t result = 0;
  for (int i = 0; i < bytes_read; i += SOC_ADC_DIGI_RESULT_BYTES) {
    const auto p = (adc_digi_output_data_t *)&adc_read[i];
    const auto chan_num = ADC_GET_CHANNEL(p);
    auto data = ADC_GET_DATA(p);

    if (chan_num >= SOC_ADC_CHANNEL_NUM(0)) {
      log_e("Invalid data [%d_%d]", chan_num, data);
      return 0;
    }

    if (data >= (1 << SOC_ADC_DIGI_MAX_BITWIDTH)) {
      log_e("Invalid data");
      data = 0xFFFF;
    }

    int mvolts;
    data = adc_cali_raw_to_voltage(adc_handle.adc_cali_handle, data, &mvolts) == ESP_OK ? mvolts : 0xFFFE;
    samples[result++] = data;
  }
  return result;
}

bool analogSampleToMVolts(uint16_t sample, uint16_t &mvolts) {
  int buffer;
  return adc_cali_raw_to_voltage(adc_handle.adc_cali_handle, sample, &buffer) == ESP_OK ? mvolts = buffer, true : false;
}

bool analogContinuousDeinitFix() {
  if (adc_handle.adc_cali_handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (adc_cali_delete_scheme_curve_fitting(adc_handle.adc_cali_handle) != ESP_OK) {
      return false;
    }
#elif (!defined(CONFIG_IDF_TARGET_ESP32H2) && !defined(CONFIG_IDF_TARGET_ESP32P4))
    if (adc_cali_delete_scheme_line_fitting(adc_handle.adc_cali_handle) != ESP_OK) {
      return false;
    }
#endif
  }
  adc_handle.adc_cali_handle = nullptr;
  return true;
}
