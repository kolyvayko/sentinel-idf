// SPDX-License-Identifier: MIT
/*
 * Simple I2C example for SSD1306 driver
 * Initializes display, draws some primitives, and flushes to screen.
 */
#include "adc/adc.h"
#include "app_config.h"
#include "direction/direction.h"
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
static QueueHandle_t s_adc1_queue = NULL;

static void queue_send_latest_int(QueueHandle_t queue, int value) {
  if (queue == NULL) {
    return;
  }

  if (xQueueSend(queue, &value, 0) != pdPASS) {
    int discard = 0;
    (void)xQueueReceive(queue, &discard, 0);
    (void)xQueueSend(queue, &value, 0);
  }
}

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
    queue_send_latest_int(s_adc1_queue, sample.adc1);
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

static void adc1_direction_task(void *param) {
  direction_state_t state;
  direction_init(&state);
  for (;;) {
    int value = 0;
    if (xQueueReceive(s_adc1_queue, &value, portMAX_DELAY) != pdPASS) {
      continue;
    }
    int peak = 0;
    direction_t direction = DIRECTION_UNKNOWN;
    if (direction_update(&state, value, ADC1_THRESHOLD, ADC1_HYSTERESIS, &peak,
                         &direction)) {
      ESP_LOGI(TAG, "ADC1 peak=%d direction=%s", peak,
               (direction == DIRECTION_INCREASING) ? "increasing"
                                                   : "decreasing");
    }
  }
}

void app_main(void) {
  s_sample_queue = xQueueCreate(ADC_QUEUE_LENGTH, sizeof(adc_sample_t));
  s_adc1_queue = xQueueCreate(ADC1_QUEUE_LENGTH, sizeof(int));
  if (s_sample_queue == NULL || s_adc1_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create ADC sample queue");
    if (s_sample_queue != NULL) {
      vQueueDelete(s_sample_queue);
    }
    if (s_adc1_queue != NULL) {
      vQueueDelete(s_adc1_queue);
    }
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
    vQueueDelete(s_adc1_queue);
    return;
  }

  if (xTaskCreate(adc1_direction_task, "adc1_dir_task", ADC1_TASK_STACK_SIZE,
                  NULL, ADC1_TASK_PRIORITY, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create ADC1 direction task");
    vQueueDelete(s_sample_queue);
    vQueueDelete(s_adc1_queue);
    return;
  }
}
