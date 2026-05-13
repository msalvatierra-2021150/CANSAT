#include "tasks.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "i2c/i2c_bus.c"
#include "esp_log.h"

// --- Sensor Includes ---
#include "camera/anaglyph_core.h"
#include "dps310/dps310.h"
#include "lsm9ds1/lsm9ds1_hal.h"
#include "servos/servos.h"

// ===================== LOGGING TAGS =====================

static const char *TAG_BARO  = "BARO";
static const char *TAG_GPS   = "GPS";
static const char *TAG_LORA  = "LORA";
static const char *TAG_IMU   = "IMU";
static const char *TAG_SERVO = "SERVO";

TelemetryF32V1 transmittedData;
SemaphoreHandle_t dataMutex;

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

//===================== BAROMETER TASK =====================
void baro_task(void *arg) {
  dps310_t dps = {};

  ESP_ERROR_CHECK(dps310_attach(&dps, bus_handle, 0x77));
  ESP_ERROR_CHECK(dps310_init(&dps));
  ESP_ERROR_CHECK(dps310_config(&dps, DPS310_RATE_4HZ, DPS310_OSR_16,
                                DPS310_RATE_4HZ, DPS310_OSR_16,
                                DPS310_MODE_CONT_PT));

  ESP_LOGI(TAG_BARO, "DPS310 configured.");

  while (1) {
    float t_c = 0.0f;
    float p_hpa = 0.0f;
    float velocity_mps = 0.0f;

    if (dps310_read(&dps, &t_c, &p_hpa, &velocity_mps) == ESP_OK) {
      float alt_m = 44330.0f * (1.0f - powf(p_hpa / 1013.25f, 0.1903f));
      if (alt_m <= 260) {
        xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, 8,
        NULL, 1);
      }
      ESP_LOGI(TAG_BARO,
               "Temp: %.2f C | Pressure: %.2f hPa | Velocity: %.2f m/s | Alt: %.2f m",
               t_c, p_hpa, velocity_mps, alt_m);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        transmittedData.temp = t_c;
        transmittedData.pressure = p_hpa;
        transmittedData.altitude = alt_m;
        xSemaphoreGive(dataMutex);
      }
    } else {
      ESP_LOGW(TAG_BARO, "Failed to read DPS310");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// // ===================== LSM9DS1 TASK =====================

void lsm9ds1_task(void *arg) {
    ESP_LOGI(TAG_IMU, "Starting LSM9DS1 task...");
    ESP_ERROR_CHECK(lsm9ds1_attach(bus_handle));
    ESP_ERROR_CHECK(lsm9ds1_init());
    while (1) {
        float ax, ay, az;
        float gx, gy, gz;

        if (lsm9ds1_read_accel(&ax, &ay, &az) == ESP_OK) {
            ESP_LOGI(TAG_IMU, "Accel (mg): X=%.2f Y=%.2f Z=%.2f", ax, ay, az);

            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
            {
                transmittedData.accelX = ax;
                transmittedData.accelY = ay;
                transmittedData.accelZ = az;
                //UNLOCK
                xSemaphoreGive(dataMutex);
            }
        }
        if (lsm9ds1_read_gyro(&gx, &gy, &gz) == ESP_OK) {
            ESP_LOGI(TAG_IMU, "Accel (deg/s): X=%.2f Y=%.2f Z=%.2f", gx, gy, gz);

            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
            {
                transmittedData.gyroX = gx;
                transmittedData.gyroY = gy;
                transmittedData.gyroZ = gz;
                //UNLOCK
                xSemaphoreGive(dataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===================== SERVO TASK =====================
void servo_task(void *arg) {
  servo_init();
  servo_set_90_once();
  // Give the servo a little time to move
  vTaskDelay(pdMS_TO_TICKS(500));

  // Do NOT return from a FreeRTOS task
  vTaskDelete(NULL);
}

//Camera Task
void camera_task(void *arg) {
  ptc06_t *cam = (ptc06_t *)arg;

  if (cam == NULL) {
    ESP_LOGE("CAMERA", "camera_task received NULL cam");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI("CAMERA", "Waiting before first capture...");
  vTaskDelay(pdMS_TO_TICKS(10000)); // 10 seconds

  while (1) {
    run_anaglyph_capture_cycle(cam, 1500);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}