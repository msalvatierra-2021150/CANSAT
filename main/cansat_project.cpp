#include "anaglyph_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "fs/fs_handler.h"
#include "i2c/i2c_bus.h"
#include "ptc06.h"
#include "rfm95_task.h"
#include "rtos/tasks.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include <stdio.h>
#include <stdlib.h>

static const char *TAG_MAIN = "MAIN";
static const char *TAG_CAM_INIT = "CAM_INIT";

extern "C" {

static esp_err_t init_camera_for_project(ptc06_t **out_cam) {
  if (out_cam == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG_CAM_INIT, "Initializing camera at 38400 bps...");

  ptc06_config_t cfg = {
      .uart_num = UART_NUM_1,
      .tx_pin = GPIO_NUM_4, // ESP TX -> camera RX
      .rx_pin = GPIO_NUM_5, // ESP RX -> camera TX
      .baud_rate = 38400,
      .uart_rx_buf_size = 4096,
      .uart_tx_buf_size = 512,
      .cmd_timeout_ms = 3000,
  };

  ptc06_t *cam = NULL;
  if (ptc06_init(&cam, &cfg) != ESP_OK || cam == NULL) {
    ESP_LOGE(TAG_CAM_INIT, "Failed to initialize camera");
    return ESP_FAIL;
  }

  // Do NOT force resolution yet.
  // Your logs suggest the camera links, but this command times out on your
  // module. You can revisit it later once capture is stable.
  //
  // esp_err_t res_err = ptc06_set_resolution(cam, PTC06_RES_320x240);
  // if (res_err != ESP_OK) {
  //   ESP_LOGW(TAG_CAM_INIT, "ptc06_set_resolution timed out; continuing
  //   anyway");
  // }

  // Optional one-shot smoke test before launching the task

  *out_cam = cam;
  return ESP_OK;
}

void app_main(void) {
  mountFileSystem();
  loadImageIntoPSRAM();

  // 1) Mutex first
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    ESP_LOGE(TAG_MAIN, "CRITICAL ERROR: Could not create Mutex!");
    return;
  }

  // 2) Shared I2C bus
  i2c_init();

  // 3) Camera first, before other tasks
  ptc06_t *cam = NULL;
  if (init_camera_for_project(&cam) != ESP_OK) {
    ESP_LOGE(TAG_MAIN, "Camera init failed");
    return;
  }

  ESP_LOGI(TAG_MAIN, "Creating tasks...");

  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0));

  // Camera task first
  xTaskCreatePinnedToCore(camera_task, "camera_task", 32768, (void *)cam, 5,
                          NULL, 1);

  // Other tasks
  xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(lsm9ds1_task, "lsm9ds1_task", 4096, NULL, 8, NULL, 1);
  xTaskCreatePinnedToCore(baro_task, "baro_task", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(rfm95_task, "rfm95_task", 5120, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, 8, NULL, 1);
}

} // extern "C"