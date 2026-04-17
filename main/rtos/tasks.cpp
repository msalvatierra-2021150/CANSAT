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
#include "gps_speed/2d_velocity.h"
#include "lsm9ds1/lsm9ds1_hal.h"
#include "neo6m/neo6m.h"
#include "servos/servos.h"

// ===================== LOGGING TAGS =====================

static const char *TAG_BARO  = "BARO";
static const char *TAG_GPS   = "GPS";
static const char *TAG_LORA  = "LORA";
static const char *TAG_IMU   = "IMU";
static const char *TAG_SERVO = "SERVO";
// static const char *TAG_TMP   = "TMP";

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

    if (dps310_read(&dps, &t_c, &p_hpa) == ESP_OK) {
      float alt_m = 44330.0f * (1.0f - powf(p_hpa / 1013.25f, 0.1903f));

      ESP_LOGI(TAG_BARO, "T=%.2f C  P=%.2f hPa  Alt≈%.1f m", t_c, p_hpa, alt_m);

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

// // ===================== GPS TASK =====================
void gps_task(void *arg) {
    ESP_LOGI(TAG_GPS, "Starting GPS task...");
    gps_start();

    while (1) {
        raw_nmea();
        float v_north = 0.0f;
        float v_east = 0.0f;

        if (gps_get_ground_velocity_ms(&v_north, &v_east)) {
            ESP_LOGI(TAG_GPS, "v_north=%.2f m/s  v_east=%.2f m/s", v_north, v_east);

            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
                {
                    transmittedData.velocityX = v_north;
                    transmittedData.velocityY = v_east;
                    //UNLOCK
                    xSemaphoreGive(dataMutex);
                }
        } else {
            ESP_LOGW(TAG_GPS, "Could not parse ground velocity");
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
  ESP_LOGI(TAG_SERVO, "Starting servo task...");
  servo_init();
  vTaskDelay(pdMS_TO_TICKS(5000));
  while (1) {
    for (int angle = 0; angle <= 180; angle += 10) {
      servo_set_angle(angle);
      vTaskDelay(pdMS_TO_TICKS(300));
    }

    for (int angle = 180; angle >= 0; angle -= 10) {
      servo_set_angle(angle);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }
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

  run_anaglyph_capture_cycle(cam, 1500);

  ESP_LOGI("CAMERA", "Single capture done, deleting camera task.");
  vTaskDelete(NULL);
}