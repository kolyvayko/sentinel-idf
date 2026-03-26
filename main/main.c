// SPDX-License-Identifier: MIT
#include "adc/adc.h"
#include "app_config.h"
#include "bearing/bearing.h"
#include "config/config.h"
#include "display/display.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char *TAG = "MAIN";

// Shared queues — extern'd by bearing.c
QueueHandle_t g_adc_queue          = NULL;
QueueHandle_t g_bearing_nav_queue  = NULL;
QueueHandle_t g_bearing_disp_queue = NULL;

static void adc_task(void *param) {
    adc_continuous_handle_t handle = adc_init();
    if (!handle) {
        ESP_LOGE(TAG, "ADC init failed");
        vTaskDelete(NULL);
        return;
    }

    adc_sample_t s = {0};
    uint32_t n = 0;
    while (1) {
        adc_read_values(handle, &s.adc1, &s.adc2);
        n++;
        if ((n % DISPLAY_UPDATE_INTERVAL) == 0) {
            if (xQueueSend(g_adc_queue, &s, 0) != pdPASS) {
                adc_sample_t tmp;
                xQueueReceive(g_adc_queue, &tmp, 0);
                xQueueSend(g_adc_queue, &s, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void display_task(void *param) {
    ssd1306_handle_t disp = init_display();
    if (!disp) {
        ESP_LOGE(TAG, "Display init failed");
        vTaskDelete(NULL);
        return;
    }

    bearing_t b = {0};
    for (;;) {
        if (xQueueReceive(g_bearing_disp_queue, &b, portMAX_DELAY) == pdPASS) {
            display_show_bearing(disp, &b, "SEARCHING");
        }
    }
}

void app_main(void) {
    config_init();

    g_adc_queue          = xQueueCreate(ADC_QUEUE_LENGTH, sizeof(adc_sample_t));
    g_bearing_nav_queue  = xQueueCreate(1,                sizeof(bearing_t));
    g_bearing_disp_queue = xQueueCreate(1,                sizeof(bearing_t));

    if (!g_adc_queue || !g_bearing_nav_queue || !g_bearing_disp_queue) {
        ESP_LOGE(TAG, "Queue creation failed");
        return;
    }

    xTaskCreate(adc_task,        "adc_task",     ADC_TASK_STACK_SIZE,     NULL, ADC_TASK_PRIORITY,     NULL);
    xTaskCreate(bearing_task,    "bearing_task", BEARING_TASK_STACK_SIZE, NULL, BEARING_TASK_PRIORITY, NULL);
    xTaskCreate(display_task,    "disp_task",    DISPLAY_TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIORITY, NULL);
    xTaskCreate(config_btn_task, "cal_btn_task", 2048, (void *)g_adc_queue, 3,  NULL);
}
