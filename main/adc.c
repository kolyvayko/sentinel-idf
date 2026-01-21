#include "adc.h"
#include <driver/adc.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <string.h>

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define EXAMPLE_READ_LEN 256

static adc_channel_t channel[2] = {ADC_CHANNEL_1,
                                   ADC_CHANNEL_2}; // GPIO32, GPIO33

static TaskHandle_t s_task_handle;
static const char *TAG = "ADC";

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data) {
  BaseType_t mustYield = pdFALSE;
  // Notify that ADC continuous driver has done enough number of conversions
  vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

  return (mustYield == pdTRUE);
}

adc_continuous_handle_t adc_init() {
  adc_continuous_handle_t handle = NULL;

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = EXAMPLE_READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

  adc_continuous_config_t dig_cfg = {
      .sample_freq_hz = 20 * 1000,
      .conv_mode = EXAMPLE_ADC_CONV_MODE,
  };

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
  dig_cfg.pattern_num = sizeof(channel) / sizeof(adc_channel_t);
  for (int i = 0; i < dig_cfg.pattern_num; i++) {
    adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
    adc_pattern[i].channel = channel[i] & 0x7;
    adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
    adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

    ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
    ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i,
             adc_pattern[i].channel);
    ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
  }
  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

  s_task_handle = xTaskGetCurrentTaskHandle();

  adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = s_conv_done_cb,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
  ESP_ERROR_CHECK(adc_continuous_start(handle));

  return handle;
}

void adc_read_values(adc_continuous_handle_t handle, int *adc1, int *adc2) {
  esp_err_t ret;
  uint32_t ret_num = 0;
  uint8_t result[EXAMPLE_READ_LEN] = {0};

  /**
   * This is to show you the way to use the ADC continuous mode driver event
   * callback. This `ulTaskNotifyTake` will block when the data processing in
   * the task is fast. However in this example, the data processing (print) is
   * slow, so you barely block here.
   *
   * Without using this event callback (to notify this task), you can still just
   * call `adc_continuous_read()` here in a loop, with/without a certain block
   * timeout.
   */
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
  if (ret == ESP_OK) {
    ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32 " bytes", ret, ret_num);

    adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
    uint32_t num_parsed_samples = 0;

    esp_err_t parse_ret = adc_continuous_parse_data(
        handle, result, ret_num, parsed_data, &num_parsed_samples);
    if (parse_ret == ESP_OK) {
      for (int i = 0; i < num_parsed_samples; i++) {
        if (parsed_data[i].valid) {
          ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %" PRIu32,
                   parsed_data[i].unit + 1, parsed_data[i].channel,
                   parsed_data[i].raw_data);
          if (parsed_data[i].channel == ADC_CHANNEL_1) {
            *adc1 = parsed_data[i].raw_data;
          } else if (parsed_data[i].channel == ADC_CHANNEL_2) {
            *adc2 = parsed_data[i].raw_data;
          }
        } else {
          ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%" PRIu32 "]",
                   parsed_data[i].unit + 1, parsed_data[i].channel,
                   parsed_data[i].raw_data);
        }
      }
    } else {
      ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
    }
  }
}