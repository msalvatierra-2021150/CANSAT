#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lsm9ds1_attach(i2c_master_bus_handle_t bus_handle);
esp_err_t lsm9ds1_init(void);

esp_err_t lsm9ds1_read_accel(float *x, float *y, float *z);
esp_err_t lsm9ds1_read_gyro(float *x, float *y, float *z);
esp_err_t lsm9ds1_read_mag(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif