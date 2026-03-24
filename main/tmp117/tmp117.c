// tmp117.c
#include "tmp117.h"
#include "esp_log.h"

#define I2C_TIMEOUT_MS 1000

static const char *TAG = "TMP117";
static i2c_master_dev_handle_t tmp117_dev = NULL;

esp_err_t tmp117_attach(i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP117_ADDR,
        .scl_speed_hz = 100000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &tmp117_dev);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TMP117 attached");
    } else {
        ESP_LOGE(TAG, "Failed to attach TMP117: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t tmp117_init(void) {
    if (tmp117_dev == NULL) {
        ESP_LOGE(TAG, "TMP117 not attached");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t config_data[] = { TMP117_CONFIG_REG, 0x02, 0x20 };

    esp_err_t err = i2c_master_transmit(
        tmp117_dev,
        config_data,
        sizeof(config_data),
        I2C_TIMEOUT_MS
    );

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TMP117 initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to initialize TMP117: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t tmp117_read_raw(int16_t *raw_temp) {
    if (tmp117_dev == NULL || raw_temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = TMP117_TEMP_REG;
    uint8_t data[2] = {0};

    esp_err_t err = i2c_master_transmit_receive(
        tmp117_dev,
        &reg,
        1,
        data,
        2,
        I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(err));
        return err;
    }

    *raw_temp = (int16_t)((data[0] << 8) | data[1]);
    return ESP_OK;
}

float tmp117_compensate(int16_t raw_temp) {
    return raw_temp * 0.0078125f;
}