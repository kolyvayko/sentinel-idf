// SPDX-License-Identifier: MIT
/*
 * Simple I2C example for SSD1306 driver
 * Initializes display, draws some primitives, and flushes to screen.
 */
#include "adc/adc.h"
#include "app_config.h"
#include "display/display.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdint.h>

static const char *TAG = "MAIN";

typedef struct {
  int adc1;
  int adc2;
} adc_sample_t;

static QueueHandle_t s_sample_queue = NULL;

static void adc_task(void *param) {
  adc_continuous_handle_t adc_handle = adc_init();
  if (adc_handle == NULL) {
    ESP_LOGE(TAG, "Failed to initialize ADC");
    vTaskDelete(NULL);
    return;
  }
  adc_sample_t sample = {0};
  uint32_t sample_count = 0;
  while (1) {
    adc_read_values(adc_handle, &sample.adc1, &sample.adc2);
    sample_count++;

    ESP_LOGI(TAG, "ADC reading #%lu -> CH1=%d, CH2=%d",
             (unsigned long)sample_count, sample.adc1, sample.adc2);

    if ((sample_count % DISPLAY_UPDATE_INTERVAL) == 0) {
      if (xQueueSend(s_sample_queue, &sample, 0) != pdPASS) {
        // If full, drop the oldest and enqueue the latest
        (void)xQueueReceive(s_sample_queue, &sample, 0);
        (void)xQueueSend(s_sample_queue, &sample, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void display_task(void *param) {
  ssd1306_handle_t disp = init_display();
  if (disp == NULL) {
    ESP_LOGE(TAG, "Failed to initialize display");
    vTaskDelete(NULL);
    return;
  }

  adc_sample_t latest = {0};
  for (;;) {
    if (xQueueReceive(s_sample_queue, &latest, portMAX_DELAY) == pdPASS) {
      ESP_LOGE(TAG, "send to display.");
      display_show_data(disp, latest.adc1, latest.adc2);
    }
  }
}

void app_main(void) {
  s_sample_queue = xQueueCreate(ADC_QUEUE_LENGTH, sizeof(adc_sample_t));
  if (s_sample_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create ADC sample queue");
    return;
  }

  if (xTaskCreate(adc_task, "adc_task", ADC_TASK_STACK_SIZE, NULL,
                  ADC_TASK_PRIORITY, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create ADC task");
    vQueueDelete(s_sample_queue);
    return;
  }

  if (xTaskCreate(display_task, "display_task", DISPLAY_TASK_STACK_SIZE, NULL,
                  DISPLAY_TASK_PRIORITY, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create display task");
    vQueueDelete(s_sample_queue);
    return;
  }
}

// // ----- Draw pixels in corners of screen -----
// ESP_ERROR_CHECK(ssd1306_draw_pixel(d, 0, 0, true));
// ESP_ERROR_CHECK(ssd1306_draw_pixel(d, cfg.width - 1, 0, true));
// ESP_ERROR_CHECK(ssd1306_draw_pixel(d, 0, cfg.height - 1, true));
// ESP_ERROR_CHECK(ssd1306_draw_pixel(d, cfg.width - 1, cfg.height - 1, true));

// // ----- Draw rectangles -----
// ESP_ERROR_CHECK(ssd1306_draw_rect(d, 2, 2, 40, 20, false));
// ESP_ERROR_CHECK(ssd1306_draw_rect(d, 2, 24, 32, 16, true));

// // ----- Draw circles -----
// ESP_ERROR_CHECK(ssd1306_draw_circle(d, 32, 52, 8, true));
// ESP_ERROR_CHECK(ssd1306_draw_circle(d, 100, 52, 4, false));

// // ----- Draw lines -----
// ESP_ERROR_CHECK(ssd1306_draw_line(d, 2, 2, 40, 20, true));
// ESP_ERROR_CHECK(ssd1306_draw_line(d, 32, 52, 100, 52, true));

// ----- Draw text -----
// ESP_ERROR_CHECK(ssd1306_draw_text(d, 48, 2, "OK!", true));