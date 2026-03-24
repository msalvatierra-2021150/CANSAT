#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rtos/tasks.h"
#include "i2c/i2c_bus.h"
#include <stdio.h>

static const char *TAG_MAIN = "MAIN";

extern "C" {
void app_main(void) {

  // ---------------------------------------------------------
  // STEP 1: CRITICAL - Create the Mutex BEFORE anything else
  // ---------------------------------------------------------
  dataMutex = xSemaphoreCreateMutex();

  // Check if it failed (e.g., out of memory)
  if (dataMutex == NULL) {
    ESP_LOGE("MAIN", "CRITICAL ERROR: Could not create Mutex!");
    return; // Stop here, do not create tasks
  }

  // ---------------------------------------------------------
  // STEP 2: Initialize I2C (Shared Bus)
  // ---------------------------------------------------------
  i2c_init();

  ESP_LOGI(TAG_MAIN, "Creating tasks...");

  // // ---------------------------------------------------------
  // // STEP 3: Create Tasks
  // // ---------------------------------------------------------
  // // // GPS Task
  xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 1);

  // // // // IMU Task
  xTaskCreatePinnedToCore(lsm9ds1_task, "lsm9ds1_task", 4096, NULL, 8, NULL, 1);

  // // // Barometer Task
  xTaskCreatePinnedToCore(baro_task, "baro_task", 4096, NULL, 4, NULL, 1);

  BaseType_t ok;

  // // // LoRa Task
  // ok = xTaskCreatePinnedToCore(lora_task, "lora_task", 5120, NULL, 10, NULL, 1);
  // ESP_LOGI(TAG_MAIN, "lora_task create: %s", ok == pdPASS ? "OK" : "FAIL");

  xTaskCreatePinnedToCore(tmp117_task, "tpm117_task", 4096, NULL, 8, NULL, 1);
}
}