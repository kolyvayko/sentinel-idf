// main/config/config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

typedef enum {
    CFG_VPHS_ZERO_MV = 0,   // uint16, azimuth zero (mV)
    CFG_VPHS_ZERO2_MV,      // uint16, elevation zero (mV) — Phase 5
    CFG_SIGNAL_THRESHOLD,   // uint16, min ADC counts
    CFG_PARAM_COUNT
} cfg_param_id_t;

void     config_init(void);
uint16_t config_get_u16(cfg_param_id_t id);
void     config_set_u16(cfg_param_id_t id, uint16_t value);

// Auto-zero: reads current VPHS ADC values and saves as zero calibration.
// adc1_raw: current raw ADC reading for azimuth channel.
// adc2_raw: current raw ADC reading for elevation channel (ignored if ELEVATION_ENABLED=0).
void config_auto_zero(int adc1_raw, int adc2_raw);

// FreeRTOS task that monitors the calibration button (GPIO SENTINEL_CAL_BTN_GPIO).
// param: QueueHandle_t of adc_sample_t — reads latest ADC values for auto-zero.
void config_btn_task(void *param);

#endif // CONFIG_H
