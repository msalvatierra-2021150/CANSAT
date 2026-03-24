#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TMP117_ADDR 0x48
#define TMP117_TEMP_REG 0x00
#define TMP117_CONFIG_REG 0x01

esp_err_t tmp117_attach(i2c_master_bus_handle_t bus_handle);
esp_err_t tmp117_init(void);
esp_err_t tmp117_read_raw(int16_t *raw_temp);
float tmp117_compensate(int16_t raw_temp);

#ifdef __cplusplus
}
#endif