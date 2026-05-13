#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DPS310_RATE_1HZ = 0,
  DPS310_RATE_2HZ = 1,
  DPS310_RATE_4HZ = 2,
  DPS310_RATE_8HZ = 3,
  DPS310_RATE_16HZ = 4,
  DPS310_RATE_32HZ = 5,
  DPS310_RATE_64HZ = 6,
  DPS310_RATE_128HZ = 7,
} dps310_rate_t;

typedef enum {
  DPS310_OSR_1 = 0,
  DPS310_OSR_2 = 1,
  DPS310_OSR_4 = 2,
  DPS310_OSR_8 = 3,
  DPS310_OSR_16 = 4,
  DPS310_OSR_32 = 5,
  DPS310_OSR_64 = 6,
  DPS310_OSR_128 = 7,
} dps310_osr_t;

typedef enum {
  DPS310_MODE_IDLE = 0x00,
  DPS310_MODE_CMD_P = 0x01,
  DPS310_MODE_CMD_T = 0x02,
  DPS310_MODE_CONT_P = 0x05,
  DPS310_MODE_CONT_T = 0x06,
  DPS310_MODE_CONT_PT = 0x07,
} dps310_mode_t;

typedef struct {
  i2c_master_dev_handle_t i2c_dev;
  uint8_t i2c_addr;
  bool temp_ext;

  dps310_osr_t osr_t;
  dps310_osr_t osr_p;
  int32_t kT;
  int32_t kP;

  int16_t c0, c1;
  int32_t c00, c10;
  int16_t c01, c11, c20, c21, c30;
} dps310_t;

esp_err_t dps310_attach(dps310_t *dev, i2c_master_bus_handle_t bus_handle,
                        uint8_t addr);
esp_err_t dps310_init(dps310_t *dev);
esp_err_t dps310_config(dps310_t *dev, dps310_rate_t rate_t, dps310_osr_t osr_t,
                        dps310_rate_t rate_p, dps310_osr_t osr_p,
                        dps310_mode_t mode);
esp_err_t dps310_read(dps310_t *dev, float *temperature_c, float *pressure_hpa,
                      float *velocity_mps);

#ifdef __cplusplus
}
#endif