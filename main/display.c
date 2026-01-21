#include "display.h"
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "DISPLAY";

// Init i2c
static i2c_master_bus_handle_t i2c_bus0_init(gpio_num_t sda, gpio_num_t scl,
                                             uint32_t hz) {
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 0,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t bus = NULL;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
  // NOTE: per-device speed is set when adding the device (in driver bind).
  return bus;
}

// Internal function to initialize SSD1306
static ssd1306_handle_t init_ssd1306() {
  i2c_master_bus_handle_t i2c_bus =
      i2c_bus0_init(GPIO_NUM_22, GPIO_NUM_23, 400000);

  ssd1306_config_t cfg = {
      .width = 128,
      .height = 32,
      .fb = NULL, // let driver allocate internally
      .fb_len = 0,
      .iface.i2c =
          {
              .port = I2C_NUM_0,
              .addr = 0x3C,            // typical SSD1306 I2C address
              .rst_gpio = GPIO_NUM_NC, // no reset pin
          },
  };

  ssd1306_handle_t d = NULL;
  ESP_ERROR_CHECK(ssd1306_new_i2c(&cfg, &d));
  return d;
}

ssd1306_handle_t init_display() {
  i2c_master_bus_handle_t i2c_bus =
      i2c_bus0_init(GPIO_NUM_22, GPIO_NUM_23, 400000);

  ssd1306_config_t cfg = {
      .width = 128,
      .height = 32,
      .fb = NULL, // let driver allocate internally
      .fb_len = 0,
      .iface.i2c =
          {
              .port = I2C_NUM_0,
              .addr = 0x3C,            // typical SSD1306 I2C address
              .rst_gpio = GPIO_NUM_NC, // no reset pin
          },
  };

  ssd1306_handle_t d = NULL;
  ESP_ERROR_CHECK(ssd1306_new_i2c(&cfg, &d));
  return d;
}

void display_show_data(ssd1306_handle_t d, int val1, int val2) {
  if (!d)
    return;

  // ----- Clear screen -----
  ESP_ERROR_CHECK(ssd1306_clear(d));

  char text[30];
  sprintf(text, "ADC1: %d\nADC2: %d", val1, val2);
  ESP_ERROR_CHECK(ssd1306_draw_text_scaled(d, 0, 0, text, true, 2));

  ESP_ERROR_CHECK(ssd1306_display(d));

  ESP_LOGI(TAG, "Display updated with ADC: %d, %d", val1, val2);
  vTaskDelay(pdMS_TO_TICKS(100)); // Delay 100ms between updates
}