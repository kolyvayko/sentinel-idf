// main/config/config.c
#include "config.h"
#include "../app_config.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/FreeRTOS.h>

static const char *TAG = "CONFIG";
static const char *NVS_NS = "sentinel";

static const char *s_keys[CFG_PARAM_COUNT] = {
    [CFG_VPHS_ZERO_MV]    = "vphs_zero",
    [CFG_VPHS_ZERO2_MV]   = "vphs_zero2",
    [CFG_SIGNAL_THRESHOLD] = "sig_thr",
};

static const uint16_t s_defaults[CFG_PARAM_COUNT] = {
    [CFG_VPHS_ZERO_MV]    = SENTINEL_VPHS_ZERO_MV,
    [CFG_VPHS_ZERO2_MV]   = SENTINEL_VPHS_ZERO_MV,
    [CFG_SIGNAL_THRESHOLD] = SENTINEL_SIGNAL_THRESHOLD,
};

static uint16_t s_values[CFG_PARAM_COUNT];

void config_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_handle_t h;
    esp_err_t open_ret = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (open_ret == ESP_OK) {
        for (int i = 0; i < CFG_PARAM_COUNT; i++) {
            uint16_t v = s_defaults[i];
            nvs_get_u16(h, s_keys[i], &v);   // ignore ESP_ERR_NVS_NOT_FOUND
            s_values[i] = v;
            ESP_LOGI(TAG, "%s = %u", s_keys[i], v);
        }
        nvs_close(h);
    } else {
        // Namespace doesn't exist yet — use defaults
        for (int i = 0; i < CFG_PARAM_COUNT; i++) {
            s_values[i] = s_defaults[i];
            ESP_LOGI(TAG, "%s = %u (default)", s_keys[i], s_defaults[i]);
        }
    }
}

uint16_t config_get_u16(cfg_param_id_t id) {
    if (id >= CFG_PARAM_COUNT) {
        ESP_LOGW(TAG, "config_get_u16: invalid id %d", id);
        return 0;
    }
    return s_values[id];
}

void config_set_u16(cfg_param_id_t id, uint16_t value) {
    if (id >= CFG_PARAM_COUNT) {
        ESP_LOGW(TAG, "config_set_u16: invalid id %d", id);
        return;
    }
    s_values[id] = value;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        esp_err_t err = nvs_set_u16(h, s_keys[id], value);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS write failed for %s: %s", s_keys[id], esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "saved %s = %u", s_keys[id], value);
        }
    } else {
        ESP_LOGE(TAG, "NVS open failed for write");
    }
}

// Temporary local typedef — matches adc_sample_t in bearing.c; removed in Task 6
#ifndef ADC_SAMPLE_T_DEFINED
#define ADC_SAMPLE_T_DEFINED
typedef struct { int adc1; int adc2; int adc3; } adc_sample_t;
#endif

void config_auto_zero(int adc1_raw, int adc2_raw) {
    // Clamp to valid 12-bit ADC range
    if (adc1_raw < 0) adc1_raw = 0;
    if (adc1_raw > 4095) adc1_raw = 4095;
    // Convert raw ADC to mV: 3300 mV / 4095 counts
    uint16_t mv1 = (uint16_t)((adc1_raw * 3300) / 4095);
    config_set_u16(CFG_VPHS_ZERO_MV, mv1);
    ESP_LOGI(TAG, "auto-zero: AZ=%umV", mv1);

#if SENTINEL_ELEVATION_ENABLED
    if (adc2_raw < 0) adc2_raw = 0;
    if (adc2_raw > 4095) adc2_raw = 4095;
    uint16_t mv2 = (uint16_t)((adc2_raw * 3300) / 4095);
    config_set_u16(CFG_VPHS_ZERO2_MV, mv2);
    ESP_LOGI(TAG, "auto-zero: EL=%umV", mv2);
#endif
}

void config_btn_task(void *param) {
    QueueHandle_t adc_q = (QueueHandle_t)param;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << SENTINEL_CAL_BTN_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);

    for (;;) {
        if (gpio_get_level(SENTINEL_CAL_BTN_GPIO) == 0) {
            // Button pressed (active low) — start hold timer
            uint32_t held_ms = 0;
            while (gpio_get_level(SENTINEL_CAL_BTN_GPIO) == 0
                   && held_ms < SENTINEL_CAL_BTN_HOLD_MS) {
                vTaskDelay(pdMS_TO_TICKS(50));
                held_ms += 50;
            }
            if (held_ms >= SENTINEL_CAL_BTN_HOLD_MS) {
                // Average VPHS over 100 ms for a stable zero reading.
                // Signal must be present during calibration (spec requirement).
                int64_t sum1 = 0, sum2 = 0;
                int count = 0;
                TickType_t start = xTaskGetTickCount();
                adc_sample_t s = {0};
                while (((xTaskGetTickCount() - start) * portTICK_PERIOD_MS) < 100) {
                    if (xQueuePeek(adc_q, &s, pdMS_TO_TICKS(10)) == pdPASS) {
                        sum1 += s.adc1;
                        sum2 += s.adc2;
                        count++;
                    }
                }
                if (count > 0) {
                    config_auto_zero((int)(sum1 / count), (int)(sum2 / count));
                    ESP_LOGI(TAG, "auto-zero triggered (%d samples)", count);
                } else {
                    ESP_LOGW(TAG, "auto-zero skipped: no ADC signal");
                }
                // Wait for button release (debounce)
                while (gpio_get_level(SENTINEL_CAL_BTN_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
