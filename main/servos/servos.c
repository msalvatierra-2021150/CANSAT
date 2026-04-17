#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_SERVO = "SERVO";

// ===================== SERVO CONFIG =====================
#define SERVO_GPIO 7
#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_CHANNEL LEDC_CHANNEL_0
#define SERVO_FREQ_HZ 50
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_MAX_ANGLE 180

// ===================== SERVO HELPERS =====================
uint32_t servo_us_to_duty(uint32_t pulse_us) {
  uint32_t duty_max = (1 << 14) - 1; // 14-bit
  uint32_t period_us = 20000;        // 50 Hz = 20 ms
  return (pulse_us * duty_max) / period_us;
}

void servo_set_angle(uint32_t angle) {
  if (angle > SERVO_MAX_ANGLE)
    angle = SERVO_MAX_ANGLE;

  uint32_t pulse_us =
      SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / SERVO_MAX_ANGLE;

  uint32_t duty = servo_us_to_duty(pulse_us);

  ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
  ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);

  ESP_LOGI(TAG_SERVO, "Angle=%lu  Pulse=%lu us  Duty=%lu", (unsigned long)angle,
           (unsigned long)pulse_us, (unsigned long)duty);
}

// ===================== SERVO INIT =====================
void servo_init(void) {
  ledc_timer_config_t ledc_timer = {.speed_mode = SERVO_MODE,
                                    .duty_resolution = SERVO_RESOLUTION,
                                    .timer_num = SERVO_TIMER,
                                    .freq_hz = SERVO_FREQ_HZ,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {.gpio_num = SERVO_GPIO,
                                        .speed_mode = SERVO_MODE,
                                        .channel = SERVO_CHANNEL,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .timer_sel = SERVO_TIMER,
                                        .duty = 0,
                                        .hpoint = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
                                        .sleep_mode =
                                            LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
#endif
                                        .flags.output_invert = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ESP_LOGI(TAG_SERVO, "Servo PWM initialized on GPIO %d", SERVO_GPIO);
}