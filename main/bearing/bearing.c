// main/bearing/bearing.c
#include "bearing.h"
#include "../config/config.h"
#include "../app_config.h"
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "adc/adc.h"

// Temporary — will be removed when adc_sample_t moves to adc/adc.h in Task 6
#ifndef ADC_SAMPLE_T_DEFINED
#define ADC_SAMPLE_T_DEFINED
typedef struct { int adc1; int adc2; int adc3; } adc_sample_t;
#endif

static const char *TAG = "BEARING";

// Queues populated by bearing_task. Created in main.c before tasks start.
extern QueueHandle_t g_bearing_nav_queue;
extern QueueHandle_t g_bearing_disp_queue;
// Queue from adc_task (defined in main.c)
extern QueueHandle_t g_adc_queue;

float vphs_to_angle(int adc_raw, float antenna_spacing_m,
                    float freq_hz, float vphs_zero_v) {
    float vphs      = adc_raw * 3.3f / 4095.0f;
    float delta_phi = (vphs - vphs_zero_v) / vphs_zero_v * M_PI;
    float lambda    = 3e8f / freq_hz;
    float sin_theta = delta_phi * lambda / (2.0f * M_PI * antenna_spacing_m);
    sin_theta = fmaxf(-1.0f, fminf(1.0f, sin_theta));
    return asinf(sin_theta) * (180.0f / M_PI);
}

void bearing_task(void *param) {
    adc_sample_t sample = {0};

    for (;;) {
        if (xQueueReceive(g_adc_queue, &sample, portMAX_DELAY) != pdPASS) continue;

        if (sample.adc1 < SENTINEL_SIGNAL_THRESHOLD) {
            ESP_LOGD(TAG, "signal below threshold, skipping");
            continue;
        }

        float vphs_zero_v = config_get_u16(CFG_VPHS_ZERO_MV) / 1000.0f;

        bearing_t b = {
            .adc1_raw = sample.adc1,
            .adc2_raw = sample.adc2,
            .azimuth_deg = vphs_to_angle(
                sample.adc1,
                SENTINEL_ANTENNA_SPACING_M,
                SENTINEL_FREQ_HZ,
                vphs_zero_v
            ),
#if SENTINEL_ELEVATION_ENABLED
            .elevation_deg = vphs_to_angle(
                sample.adc2,
                SENTINEL_ANTENNA_SPACING_M,
                SENTINEL_FREQ_HZ,
                config_get_u16(CFG_VPHS_ZERO2_MV) / 1000.0f
            ),
#else
            .elevation_deg = 0.0f,
#endif
        };

        ESP_LOGI(TAG, "AZ=%.1f EL=%.1f", b.azimuth_deg, b.elevation_deg);

        // Send to display queue (drop oldest if full)
        if (xQueueSend(g_bearing_disp_queue, &b, 0) != pdPASS) {
            bearing_t tmp;
            xQueueReceive(g_bearing_disp_queue, &tmp, 0);
            xQueueSend(g_bearing_disp_queue, &b, 0);
        }
        // Send to navigator queue (drop oldest if full)
        if (xQueueSend(g_bearing_nav_queue, &b, 0) != pdPASS) {
            bearing_t tmp;
            xQueueReceive(g_bearing_nav_queue, &tmp, 0);
            xQueueSend(g_bearing_nav_queue, &b, 0);
        }
    }
}
