#include "i2c_bus.h"
#include "esp_log.h"


// I2C Pins (DPS310, SCD41, LSM9DS1)
#define I2C_PORT I2C_NUM_0
#define SDA_GPIO 8
#define SCL_GPIO 9

i2c_master_bus_handle_t bus_handle = NULL;

void i2c_init(void) {
  i2c_master_bus_config_t cfg = {};
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.i2c_port = I2C_NUM_0;
  cfg.scl_io_num = GPIO_NUM_9;
  cfg.sda_io_num = GPIO_NUM_8;
  cfg.glitch_ignore_cnt = 7;
  cfg.flags.enable_internal_pullup = true;

  ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus_handle));
  ESP_LOGI("I2C", "I2C bus initialized");
}