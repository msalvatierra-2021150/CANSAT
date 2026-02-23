#include "tasks.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"
#include "esp_log.h"

// --- RadioLib Includes ---
#include <RadioLib.h>
#include "EspHal/EspHal.h"

// // --- Sensor Includes ---
#include "gps_speed/2d_velocity.h"
#include "dps310/dps310.h"
#include "lsm9ds1/lsm9ds1_hal.h"
#include "neo6m/neo6m.h"
#include "tmp117/tmp117.h"

// ===================== PIN DEFINITIONS =====================

// I2C Pins (DPS310, SCD41, LSM9DS1)
#define I2C_PORT       I2C_NUM_0
#define SDA_GPIO       8
#define SCL_GPIO       9

// SX1276 Pins (SX1276) - ESP32-S3 
#define SX1276_SCK       12
#define SX1276_MISO      13
#define SX1276_MOSI      11
#define SX1276_CS        10
#define SX1276_RST       14
#define SX1276_DIO0      15
#define SX1276_DIO1      16

// ===================== LOGGING TAGS =====================

static const char *TAG_BARO  = "BARO";
static const char *TAG_GPS   = "GPS";
static const char *TAG_LORA  = "LORA";
static const char *TAG_IMU   = "IMU";
static const char *TAG_TMP   = "TMP";

// ===================== I2C INIT =====================

void i2c_init(void) {
    i2c_config_t c = {}; 
    c.mode = I2C_MODE_MASTER;
    c.sda_io_num = SDA_GPIO;
    c.scl_io_num = SCL_GPIO;
    c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    c.master.clk_speed = 400000;
    c.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &c));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, c.mode, 0, 0, 0));
}

TelemetryF32V1 transmittedData;
SemaphoreHandle_t dataMutex;

// ===================== LORA TASK =====================

void lora_task(void *arg) {
    ESP_LOGI(TAG_LORA, "Setting up LoRa Hardware...");

    // 1. Initialize HAL (SPI)
    EspHal* hal = new EspHal(SX1276_SCK, SX1276_MISO, SX1276_MOSI);

    // 2. Initialize Radio Module
    ESP_LOGI(TAG_LORA, "[SX1276] Initializing FSK...");

    Module* module = new Module(
        hal,
        SX1276_CS,
        SX1276_DIO0,
        SX1276_RST,
        SX1276_DIO1
    );

    SX1276* radio = new SX1276(module);

    int state = radio->beginFSK(915.0, 50.0, 25.0, 100.0, 17, 40);
    if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG_LORA, "FSK init failed: %d", state);
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG_LORA, "FSK init success");

    // Match RX packet settings:
    radio->setCRC(true);
    radio->fixedPacketLengthMode(sizeof(TelemetryF32V1));
    uint8_t syncWord[] = { 0x2D, 0xD4 };
    radio->setSyncWord(syncWord, 2);

    static uint16_t pktCounter = 0;
    // 4. Transmission Loop
    while (1) {
    // 1. Create a local copy of data to minimize mutex holding time
        TelemetryF32V1 localData;
        
        // Take Mutex
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            localData = transmittedData; // Copy global to local
            xSemaphoreGive(dataMutex);   // Release Mutex
        }

        ESP_LOGI(TAG_LORA, "Sending");

        // 3. Transmit
        // Transmit raw binary struct
        localData.magic1  = 0xCA;
        localData.magic2  = 0xFE;
        localData.version = 1;
        localData.count   = pktCounter++;

        state = radio->transmit((uint8_t*)&localData, sizeof(localData));


        if (state == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG_LORA, "TX success!");
        } else {
            ESP_LOGE(TAG_LORA, "TX failed, code %d", state);
        }

        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===================== BAROMETER TASK =====================

void baro_task(void *arg) {
    dps310_t dps;
    ESP_LOGI(TAG_BARO, "Initializing DPS310...");

    if (dps310_init(&dps, I2C_PORT, 0x77) != ESP_OK) {
        ESP_LOGE(TAG_BARO, "Failed to initialize DPS310");
        vTaskDelete(NULL);
    }

    if (dps310_config(&dps, DPS310_RATE_16HZ, DPS310_OSR_16, DPS310_RATE_16HZ,
                      DPS310_OSR_16, DPS310_MODE_CONT_PT) != ESP_OK) {
        ESP_LOGE(TAG_BARO, "Failed to configure DPS310");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG_BARO, "DPS310 configured.");

    while (1) {
        float t_c = 0.0f;
        float p_hpa = 0.0f;

        if (dps310_read(&dps, &t_c, &p_hpa) == ESP_OK) 
        {
            float alt_m = 44330.0f * (1.0f - powf(p_hpa / 1013.25f, 0.1903f));
            ESP_LOGI(TAG_BARO, "T=%.2f C  P=%.2f hPa  Alt≈%.1f m", t_c, p_hpa, alt_m);
                // transmittedData.temp     = t_c;

                if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
                {
                    transmittedData.pressure = p_hpa;
                    transmittedData.altitude = alt_m;
                    // UNLOCK
                    xSemaphoreGive(dataMutex);
                }
        } 
        
        else 
        {
            ESP_LOGW(TAG_BARO, "Failed to read DPS310");
        }
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

// ===================== GPS TASK =====================

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

// ===================== TMP117 TASK =====================
void tmp117_task(void *arg)
{
    float raw_temperature = 0.0f;

    ESP_LOGI(TAG_IMU, "Starting TMP117 task...");

    tmp117_init();

    while(1)
    {
        raw_temperature = tmp117_read_raw();
        raw_temperature = tmp117_compensate(raw_temperature);

        ESP_LOGI(TAG_TMP, "Temperature: %.2f ", raw_temperature);

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            transmittedData.temp = raw_temperature;
            //UNLOCK
            xSemaphoreGive(dataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// ===================== LSM9DS1 TASK =====================

void lsm9ds1_task(void *arg) {
    ESP_LOGI(TAG_IMU, "Starting LSM9DS1 task...");

    if (lsm9ds1_init(I2C_PORT) != ESP_OK) 
    {
        ESP_LOGE(TAG_IMU, "Failed to initialize LSM9DS1");
        vTaskDelete(NULL);
    }

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