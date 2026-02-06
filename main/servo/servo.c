// SPDX-License-Identifier: MIT
#include "servo.h"
#include "app_config.h"
#include <driver/ledc.h>
#include <esp_err.h>

void servo_init(void) {
  ledc_timer_config_t timer_cfg = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_16_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = SERVO_PWM_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

  ledc_channel_config_t channel_cfg = {
      .gpio_num = SERVO_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

void servo_set_angle(int angle_deg) {
  if (angle_deg < 0) {
    angle_deg = 0;
  }
  if (angle_deg > SERVO_MAX_ANGLE_DEG) {
    angle_deg = SERVO_MAX_ANGLE_DEG;
  }

  const int pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
  const int pulse_us =
      SERVO_MIN_PULSE_US + (pulse_range * angle_deg) / SERVO_MAX_ANGLE_DEG;

  const uint32_t duty_max = (1U << LEDC_TIMER_16_BIT) - 1;
  const uint32_t duty =
      (uint32_t)((pulse_us * (uint64_t)SERVO_PWM_HZ * duty_max) / 1000000ULL);

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}
