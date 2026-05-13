#include "dps310.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "DPS310"
#define I2C_TIMEOUT_MS 1000

// Registers
#define DPS310_REG_PRSB2 0x00
#define DPS310_REG_TMPB2 0x03
#define DPS310_REG_PRSCFG 0x06
#define DPS310_REG_TMPCFG 0x07
#define DPS310_REG_MEASCFG 0x08
#define DPS310_REG_CFGREG 0x09
#define DPS310_REG_RESET 0x0C
#define DPS310_REG_PRODREVID 0x0D
#define DPS310_REG_COEF 0x10 // 18 bytes
#define DPS310_REG_TMP_COEF_SRCE 0x28

// default I2C addr if none provided
#define DPS310_ADDR_DEFAULT 0x77

static bool first_reading = true;

// ---------- I2C helpers ----------
static esp_err_t wr8(dps310_t *dev, uint8_t reg, uint8_t val) {
  if (dev == NULL || dev->i2c_dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t buf[2] = {reg, val};
  return i2c_master_transmit(dev->i2c_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t rd(dps310_t *dev, uint8_t reg, uint8_t *buf, size_t n) {
  if (dev == NULL || dev->i2c_dev == NULL || buf == NULL || n == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buf, n,
                                     I2C_TIMEOUT_MS);
}

static int32_t sx24(int32_t x) {
  if (x & 0x800000)
    x -= 1 << 24;
  return x;
}
static int32_t sx20(int32_t x) {
  if (x & 0x80000)
    x -= 1 << 20;
  return x;
}
static int16_t sx12(int16_t x) {
  if (x & 0x0800)
    x -= 1 << 12;
  return x;
}
static int16_t sx16(int16_t x) {
  if (x & 0x8000)
    x -= 1 << 16;
  return x;
}

// Wait helper: poll MEASCFG until (mask bits) == expect
static esp_err_t wait_bits(dps310_t *dev, uint8_t mask, uint8_t expect,
                           int timeout_ms) {
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

  while (xTaskGetTickCount() < deadline) {
    uint8_t m = 0;
    esp_err_t e = rd(dev, DPS310_REG_MEASCFG, &m, 1);
    if (e != ESP_OK) {
      return e;
    }
    if ((m & mask) == expect) {
      return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  return ESP_ERR_TIMEOUT;
}

// Robust scale mapping (don’t assume enum is 0..7)
static int32_t scale_from_osr(dps310_osr_t osr) {
  switch (osr) {
  case DPS310_OSR_1:
    return 524288; // 1x
  case DPS310_OSR_2:
    return 1572864; // 2x
  case DPS310_OSR_4:
    return 3670016; // 4x
  case DPS310_OSR_8:
    return 7864320; // 8x
  case DPS310_OSR_16:
    return 253952; // 16x
  case DPS310_OSR_32:
    return 516096; // 32x
  case DPS310_OSR_64:
    return 1040384; // 64x
  case DPS310_OSR_128:
    return 2088960; // 128x
  default:
    return 524288;
  }
}

static esp_err_t read_coeffs(dps310_t *dev) {
  uint8_t b[18];
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_COEF, b, sizeof(b)), TAG, "coef read");

  int16_t c0 = sx12((int16_t)((b[0] << 4) | (b[1] >> 4)));
  int16_t c1 = sx12((int16_t)(((b[1] & 0x0F) << 8) | b[2]));
  int32_t c00 = sx20((int32_t)((b[3] << 12) | (b[4] << 4) | (b[5] >> 4)));
  int32_t c10 = sx20((int32_t)(((b[5] & 0x0F) << 16) | (b[6] << 8) | b[7]));
  int16_t c01 = sx16((int16_t)((b[8] << 8) | b[9]));
  int16_t c11 = sx16((int16_t)((b[10] << 8) | b[11]));
  int16_t c20 = sx16((int16_t)((b[12] << 8) | b[13]));
  int16_t c21 = sx16((int16_t)((b[14] << 8) | b[15]));
  int16_t c30 = sx16((int16_t)((b[16] << 8) | b[17]));

  dev->c0 = c0;
  dev->c1 = c1;
  dev->c00 = c00;
  dev->c10 = c10;
  dev->c01 = c01;
  dev->c11 = c11;
  dev->c20 = c20;
  dev->c21 = c21;
  dev->c30 = c30;

  ESP_LOGI(TAG, "coeff ok (c0=%d c1=%d c00=%ld c10=%ld)", c0, c1, (long)c00,
           (long)c10);

  return ESP_OK;
}

esp_err_t dps310_attach(dps310_t *dev, i2c_master_bus_handle_t bus_handle,
                        uint8_t addr) {
  if (dev == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (dev->i2c_dev != NULL) {
    ESP_LOGW(TAG, "DPS310 already attached");
    return ESP_OK;
  }

  dev->i2c_addr = addr ? addr : DPS310_ADDR_DEFAULT;

  i2c_device_config_t cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = dev->i2c_addr,
      .scl_speed_hz = 400000,
  };

  esp_err_t err = i2c_master_bus_add_device(bus_handle, &cfg, &dev->i2c_dev);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "DPS310 attached at 0x%02X", dev->i2c_addr);
  } else {
    ESP_LOGE(TAG, "Failed to attach DPS310: %s", esp_err_to_name(err));
  }

  return err;
}

esp_err_t dps310_init(dps310_t *dev) {
  if (dev == NULL || dev->i2c_dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // soft reset
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_RESET, 0x09), TAG, "reset");
  vTaskDelay(pdMS_TO_TICKS(10));

  // Wait for SENSOR_RDY (bit6) and COEF_RDY (bit7)
  ESP_RETURN_ON_ERROR(wait_bits(dev, 0xC0, 0xC0, 200), TAG,
                      "sensor/coef not ready");

  uint8_t id = 0;
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_PRODREVID, &id, 1), TAG, "prod id");
  ESP_LOGI(TAG, "prod/rev id=0x%02X (expect 0x10)", id);

  uint8_t src = 0;
  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_TMP_COEF_SRCE, &src, 1), TAG,
                      "coef src");
  dev->temp_ext = (src & 0x80) != 0;
  ESP_LOGI(TAG, "TMP coef source: %s", dev->temp_ext ? "EXTERNAL" : "INTERNAL");

  // read coeffs AFTER COEF_RDY
  ESP_RETURN_ON_ERROR(read_coeffs(dev), TAG, "coeff");

  // defaults until user configures
  dev->osr_t = DPS310_OSR_16;
  dev->osr_p = DPS310_OSR_16;
  dev->kT = scale_from_osr(dev->osr_t);
  dev->kP = scale_from_osr(dev->osr_p);

  return ESP_OK;
}

esp_err_t dps310_config(dps310_t *dev, dps310_rate_t rate_t, dps310_osr_t osr_t,
                        dps310_rate_t rate_p, dps310_osr_t osr_p,
                        dps310_mode_t mode) {
  if (dev == NULL || dev->i2c_dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  // PRSCFG: [7:4]=rate, [3:0]=osr
  ESP_RETURN_ON_ERROR(
      wr8(dev, DPS310_REG_PRSCFG, ((rate_p & 0x0F) << 4) | (osr_p & 0x0F)), TAG,
      "prscfg");

  // TMPCFG: [7]=TMP_EXT, [6:4]=rate, [3:0]=osr -> match coef source
  uint8_t tmpcfg = ((rate_t & 0x0F) << 4) | (osr_t & 0x0F);
  if (dev->temp_ext) {
    tmpcfg |= 0x80;
  }
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_TMPCFG, tmpcfg), TAG, "tmpcfg");

  // CFGREG: shift enable for OSR > 8
  uint8_t cfg = 0;
  if (osr_p > DPS310_OSR_8) {
    cfg |= 0x04; // P_SHIFT_EN
  }
  if (osr_t > DPS310_OSR_8) {
    cfg |= 0x08; // T_SHIFT_EN
  }
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_CFGREG, cfg), TAG, "cfgreg");

  // MEASCFG: mode
  ESP_RETURN_ON_ERROR(wr8(dev, DPS310_REG_MEASCFG, (uint8_t)mode), TAG,
                      "meascfg");

  dev->osr_t = osr_t;
  dev->osr_p = osr_p;
  dev->kT = scale_from_osr(dev->osr_t);
  dev->kP = scale_from_osr(dev->osr_p);

  return ESP_OK;
}

static esp_err_t read_raw_tp(dps310_t *dev, int32_t *raw_t, int32_t *raw_p) {
  uint8_t b[3];

  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_PRSB2, b, 3), TAG, "raw P");
  int32_t p = sx24(((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]);

  ESP_RETURN_ON_ERROR(rd(dev, DPS310_REG_TMPB2, b, 3), TAG, "raw T");
  int32_t t = sx24(((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]);

  *raw_p = p;
  *raw_t = t;
  return ESP_OK;
}

#include "esp_timer.h"
#include <math.h>

esp_err_t dps310_read(dps310_t *dev, float *temperature_c, float *pressure_hpa,
                      float *velocity_mps) {
  static float last_alt_m = 0.0f;
  static int64_t last_time_us = 0;
  static bool first_reading = true;

  if (dev == NULL || dev->i2c_dev == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_RETURN_ON_ERROR(wait_bits(dev, 0x30, 0x30, 200), TAG, "data not ready");

  int32_t raw_t = 0;
  int32_t raw_p = 0;

  ESP_RETURN_ON_ERROR(read_raw_tp(dev, &raw_t, &raw_p), TAG, "raw rd");

  float tr = (float)raw_t / (float)dev->kT;
  float pr = (float)raw_p / (float)dev->kP;

  float T = dev->c0 * 0.5f + dev->c1 * tr;

  float p_pa = dev->c00 + pr * (dev->c10 + pr * (dev->c20 + pr * dev->c30)) +
               tr * (dev->c01 + pr * (dev->c11 + pr * dev->c21));

  float p_hpa = p_pa / 100.0f;

  if (temperature_c) {
    *temperature_c = T;
  }

  if (pressure_hpa) {
    *pressure_hpa = p_hpa;
  }

  if (p_hpa <= 0.0f) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  float alt_m = 44330.0f * (1.0f - powf(p_hpa / 1013.25f, 0.1903f));

  int64_t now_us = esp_timer_get_time();

  if (velocity_mps) {
    if (!first_reading) {
      float dt = (now_us - last_time_us) / 1000000.0f; // seconds

      if (dt > 0.0f) {
        *velocity_mps = (alt_m - last_alt_m) / dt; // m/s
      } else {
        *velocity_mps = 0.0f;
      }
    } else {
      *velocity_mps = 0.0f;
      first_reading = false;
    }
  }

  last_alt_m = alt_m;
  last_time_us = now_us;

  return ESP_OK;
}