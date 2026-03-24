/**
 * @file lsm9ds1_hal.c
 * @brief High-level HAL for LSM9DS1 (IMU + Magnetometer)
 *
 * Uses the ESP-IDF "new" I2C master bus API:
 *   - one shared i2c_master_bus_handle_t from your i2c_bus module
 *   - one device handle for IMU/GYRO (0x6B)
 *   - one device handle for MAG (0x1E)
 *
 * Based on the STMicroelectronics lsm9ds1-pid driver callbacks.
 */

#include "lsm9ds1_hal.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lsm9ds1_reg.h"

#include <stdlib.h>
#include <string.h>

static const char *TAG = "LSM9DS1_HAL";

// Default I2C addresses
#define LSM9DS1_IMU_ADDR 0x6B // Accelerometer/Gyroscope
#define LSM9DS1_MAG_ADDR 0x1E // Magnetometer

#define I2C_TIMEOUT_MS 100

// ST driver contexts
static stmdev_ctx_t dev_ctx_imu;
static stmdev_ctx_t dev_ctx_mag;

// New-driver device handles
static i2c_master_dev_handle_t imu_dev = NULL;
static i2c_master_dev_handle_t mag_dev = NULL;

/* ---------- Platform I2C access functions ---------- */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len) {
  if (handle == NULL || bufp == NULL) {
    return -1;
  }

  i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)handle;
  if (dev == NULL) {
    return -1;
  }

  uint8_t *data = (uint8_t *)malloc(len + 1);
  if (data == NULL) {
    return -1;
  }

  data[0] = reg;
  memcpy(&data[1], bufp, len);

  esp_err_t ret = i2c_master_transmit(dev, data, len + 1, I2C_TIMEOUT_MS);

  free(data);
  return (ret == ESP_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len) {
  if (handle == NULL || bufp == NULL) {
    return -1;
  }

  i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)handle;
  if (dev == NULL) {
    return -1;
  }

  esp_err_t ret =
      i2c_master_transmit_receive(dev, &reg, 1, bufp, len, I2C_TIMEOUT_MS);

  return (ret == ESP_OK) ? 0 : -1;
}

/* ---------- Public attach/init ---------- */

esp_err_t lsm9ds1_attach(i2c_master_bus_handle_t bus_handle) {
  if (bus_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (imu_dev != NULL && mag_dev != NULL) {
    ESP_LOGW(TAG, "LSM9DS1 already attached");
    return ESP_OK;
  }

  i2c_device_config_t imu_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = LSM9DS1_IMU_ADDR,
      .scl_speed_hz = 400000,
  };

  i2c_device_config_t mag_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = LSM9DS1_MAG_ADDR,
      .scl_speed_hz = 400000,
  };

  esp_err_t err = i2c_master_bus_add_device(bus_handle, &imu_cfg, &imu_dev);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to attach IMU/GYRO device: %s", esp_err_to_name(err));
    return err;
  }

  err = i2c_master_bus_add_device(bus_handle, &mag_cfg, &mag_dev);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to attach MAG device: %s", esp_err_to_name(err));
    return err;
  }

  // Hook platform functions for IMU
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = &imu_dev;

  // Hook platform functions for Magnetometer
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  dev_ctx_mag.handle = &mag_dev;

  ESP_LOGI(TAG, "LSM9DS1 attached (IMU=0x%02X, MAG=0x%02X)", LSM9DS1_IMU_ADDR,
           LSM9DS1_MAG_ADDR);

  return ESP_OK;
}

esp_err_t lsm9ds1_init(void) {
  if (imu_dev == NULL || mag_dev == NULL) {
    ESP_LOGE(TAG, "LSM9DS1 not attached");
    return ESP_ERR_INVALID_STATE;
  }

  // Read device IDs
  lsm9ds1_id_t whoami;
  if (lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoami) != 0) {
    ESP_LOGE(TAG, "Failed to read LSM9DS1 device IDs");
    return ESP_FAIL;
  }

  if (whoami.imu != LSM9DS1_IMU_ID) {
    ESP_LOGE(TAG, "IMU not found! ID: 0x%02X (expected 0x%02X)", whoami.imu,
             LSM9DS1_IMU_ID);
    return ESP_FAIL;
  }

  if (whoami.mag != LSM9DS1_MAG_ID) {
    ESP_LOGE(TAG, "Mag not found! ID: 0x%02X (expected 0x%02X)", whoami.mag,
             LSM9DS1_MAG_ID);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "LSM9DS1 detected (IMU=0x%02X, MAG=0x%02X)", whoami.imu,
           whoami.mag);

  // Reset to default configuration
  uint8_t rst = 0;
  if (lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE) != 0) {
    ESP_LOGE(TAG, "Failed to request device reset");
    return ESP_FAIL;
  }

  do {
    if (lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst) != 0) {
      ESP_LOGE(TAG, "Failed while waiting for reset");
      return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  } while (rst);

  // Accelerometer: ±4 g, ~15 Hz
  if (lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g) != 0) {
    ESP_LOGE(TAG, "Failed to set accel full-scale");
    return ESP_FAIL;
  }

  if (lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_14Hz9) != 0) {
    ESP_LOGE(TAG, "Failed to set IMU data rate");
    return ESP_FAIL;
  }

  // Gyroscope: ±2000 dps
  if (lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps) != 0) {
    ESP_LOGE(TAG, "Failed to set gyro full-scale");
    return ESP_FAIL;
  }

  // Magnetometer: ±4 gauss, ~10 Hz
  if (lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_4Ga) != 0) {
    ESP_LOGE(TAG, "Failed to set mag full-scale");
    return ESP_FAIL;
  }

  if (lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_LP_10Hz) != 0) {
    ESP_LOGE(TAG, "Failed to set mag data rate");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "LSM9DS1 configured (XL/GY/MAG @ ~10 Hz)");
  return ESP_OK;
}

/* ---------- Public read functions ---------- */

esp_err_t lsm9ds1_read_accel(float *x, float *y, float *z) {
  if (x == NULL || y == NULL || z == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t status = 0;
  if (lsm9ds1_xl_flag_data_ready_get(&dev_ctx_imu, &status) != 0) {
    return ESP_FAIL;
  }

  if (status) {
    int16_t data_raw[3];
    if (lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw) != 0) {
      return ESP_FAIL;
    }

    *x = lsm9ds1_from_fs4g_to_mg(data_raw[0]);
    *y = lsm9ds1_from_fs4g_to_mg(data_raw[1]);
    *z = lsm9ds1_from_fs4g_to_mg(data_raw[2]);
    return ESP_OK;
  }

  return ESP_ERR_NOT_FOUND;
}

esp_err_t lsm9ds1_read_gyro(float *x, float *y, float *z) {
  if (x == NULL || y == NULL || z == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t status = 0;
  if (lsm9ds1_gy_flag_data_ready_get(&dev_ctx_imu, &status) != 0) {
    return ESP_FAIL;
  }

  if (status) {
    int16_t data_raw[3];
    if (lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw) != 0) {
      return ESP_FAIL;
    }

    *x = lsm9ds1_from_fs2000dps_to_mdps(data_raw[0]) / 1000.0f;
    *y = lsm9ds1_from_fs2000dps_to_mdps(data_raw[1]) / 1000.0f;
    *z = lsm9ds1_from_fs2000dps_to_mdps(data_raw[2]) / 1000.0f;
    return ESP_OK;
  }

  return ESP_ERR_NOT_FOUND;
}

esp_err_t lsm9ds1_read_mag(float *x, float *y, float *z) {
  if (x == NULL || y == NULL || z == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t status = 0;
  if (lsm9ds1_mag_flag_data_ready_get(&dev_ctx_mag, &status) != 0) {
    return ESP_FAIL;
  }

  if (status) {
    int16_t data_raw[3];
    if (lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw) != 0) {
      return ESP_FAIL;
    }

    *x = lsm9ds1_from_fs4gauss_to_mG(data_raw[0]) / 1000.0f;
    *y = lsm9ds1_from_fs4gauss_to_mG(data_raw[1]) / 1000.0f;
    *z = lsm9ds1_from_fs4gauss_to_mG(data_raw[2]) / 1000.0f;
    return ESP_OK;
  }

  return ESP_ERR_NOT_FOUND;
}