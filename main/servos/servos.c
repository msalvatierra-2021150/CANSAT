#include "driver/ledc.h"
#include "esp_err.h"
#include "servos.h"


#define SERVO_GPIO 17
#define SERVO_FREQ_HZ 50
#define SERVO_RES_BITS 14

#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_CHANNEL LEDC_CHANNEL_0

void servo_init(void) {
  ledc_timer_config_t timer = {.speed_mode = SERVO_MODE,
                               .timer_num = SERVO_TIMER,
                               .duty_resolution = LEDC_TIMER_14_BIT,
                               .freq_hz = SERVO_FREQ_HZ,
                               .clk_cfg = LEDC_AUTO_CLK};

  ESP_ERROR_CHECK(ledc_timer_config(&timer));

  ledc_channel_config_t channel = {.gpio_num = SERVO_GPIO,
                                   .speed_mode = SERVO_MODE,
                                   .channel = SERVO_CHANNEL,
                                   .timer_sel = SERVO_TIMER,
                                   .intr_type = LEDC_INTR_DISABLE,
                                   .duty = 0,
                                   .hpoint = 0};

  ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

void servo_set_90_once(void) {
  const uint32_t max_duty = (1 << SERVO_RES_BITS) - 1;
  const uint32_t duty = (1500 * max_duty) / 20000; // 1500us = 90 degrees

  ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
  ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));
}