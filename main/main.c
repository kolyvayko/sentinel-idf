// SPDX-License-Identifier: MIT
/*
 * Simple I2C example for SSD1306 driver
 * Initializes display, draws some primitives, and flushes to screen.
 */
#include "adc.h"
#include "display.h"
#include <esp_log.h>

static const char *TAG = "MAIN";

void app_main(void) {
  ssd1306_handle_t disp = init_display();
  if (disp == NULL) {
    ESP_LOGE(TAG, "Failed to initialize display");
    return;
  }

  adc_continuous_handle_t adc_handle = adc_init();

  int adc1_val = 0, adc2_val = 0;
  while (1) {
    adc_read_values(adc_handle, &adc1_val, &adc2_val);
    display_show_data(disp, adc1_val, adc2_val);
  }

  // Cleanup (not reached)
  adc_continuous_stop(adc_handle);
  adc_continuous_deinit(adc_handle);
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