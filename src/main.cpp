#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "cJSON.h"

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

struct Data 
{
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure;
  float temp;
  float velocityX, velocityY, velocityZ;
  float altitude;
};


// Mutex handle to protect the data
SemaphoreHandle_t dataMutex = NULL;
Data transmittedData; 


// ===================== PIN DEFINITIONS =====================

// I2C Pins (DPS310, SCD41, LSM9DS1)
#define I2C_PORT       I2C_NUM_0
#define SDA_GPIO       8
#define SCL_GPIO       9

// LoRa Pins (SX1276) - ESP32-S3 
#define LORA_SCK       12
#define LORA_MISO      13
#define LORA_MOSI      11
#define LORA_CS        10
#define LORA_RST       14
#define LORA_DIO0      15
#define LORA_DIO1      16

// ===================== LOGGING TAGS =====================

static const char *TAG_MAIN  = "MAIN";
static const char *TAG_BARO  = "BARO";
static const char *TAG_GPS   = "GPS";
static const char *TAG_LORA  = "LORA";
static const char *TAG_IMU   = "IMU";
static const char *TAG_TMP   = "TMP";

// ===================== I2C INIT =====================

static void i2c_init(void) {
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

// ===================== LORA TASK =====================

static void lora_task(void *arg) {
    ESP_LOGI(TAG_LORA, "Setting up LoRa Hardware...");

    // 1. Initialize HAL (SPI)
    EspHal* hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);

    // 2. Initialize Radio Module
    // Note: We use 'new' to allocate on the heap and pass the HAL
    SX1276* radio = new SX1276(new Module(hal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1));

    // 3. Begin Radio
    ESP_LOGI(TAG_LORA, "[SX1276] Initializing ...");
    hal->pinMode(LORA_CS, OUTPUT);
    hal->digitalWrite(LORA_CS, HIGH);

    auto sxRead = [&](uint8_t reg) -> uint8_t {
    hal->digitalWrite(LORA_CS, LOW);
    hal->spiTransferByte(reg & 0x7F);      // read
    uint8_t v = hal->spiTransferByte(0x00);
    hal->digitalWrite(LORA_CS, HIGH);
    return v;
    };

    uint8_t ver = sxRead(0x42);
    ESP_LOGI(TAG_LORA, "SX127x RegVersion=0x%02X (expected 0x12)", ver);

    int state = radio->begin();
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG_LORA, "radio->begin() failed, code %d", state);
        // If radio fails, we delete the task or loop indefinitely
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG_LORA, "radio->begin() success!");

    // Buffer for the JSON string
    char tx_buffer[256];
    // 4. Transmission Loop
    while (1) {
    // 1. Create a local copy of data to minimize mutex holding time
        Data localData;
        
        // Take Mutex
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            localData = transmittedData; // Copy global to local
            xSemaphoreGive(dataMutex);   // Release Mutex
        }

        // 2. Format as JSON (Matching your Receiver's keys!)
        // Keys: ax, ay, az, gx, gy, gz, press, alt, vx, vy
        snprintf(tx_buffer, sizeof(tx_buffer), 
    "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"press\":%.2f,\"alt\":%.2f,"
    "\"temp\":%.2f,"
    "\"vx\":%.2f,\"vy\":%.2f}",
    localData.accelX, localData.accelY, localData.accelZ,
    localData.gyroX, localData.gyroY, localData.gyroZ,
    localData.pressure, localData.altitude,
    localData.temp, localData.velocityX, localData.velocityY
);

        ESP_LOGI(TAG_LORA, "Sending: %s", tx_buffer);

        // 3. Transmit
        state = radio->transmit(tx_buffer);

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

static void baro_task(void *arg) {
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

static void gps_task(void *arg) {
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
static void tmp117_task(void *arg)
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

static void lsm9ds1_task(void *arg) {
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

// ===================== APP MAIN =====================

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

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

    // ---------------------------------------------------------
    // STEP 3: Create Tasks
    // ---------------------------------------------------------
    // // GPS Task
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 1);
    
    // // IMU Task
    xTaskCreatePinnedToCore(lsm9ds1_task, "lsm9ds1_task", 4096, NULL, 8, NULL, 1);
    
    // Barometer Task
    xTaskCreatePinnedToCore(baro_task, "baro_task", 4096, NULL, 4, NULL, 1);
    
    BaseType_t ok;

    // // LoRa Task
    ok = xTaskCreatePinnedToCore(lora_task, "lora_task", 5120, NULL, 10, NULL, 1);
    ESP_LOGI(TAG_MAIN, "lora_task create: %s", ok == pdPASS ? "OK" : "FAIL");

    // TMP117 Task
    xTaskCreatePinnedToCore(tmp117_task, "tpm117_task", 4096, NULL, 8, NULL, 1);
}