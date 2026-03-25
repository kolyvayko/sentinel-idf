// main/config/config.c
#include "config.h"
#include "app_config.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_log.h>

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
