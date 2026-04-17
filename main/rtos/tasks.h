#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Remove padding
#pragma pack(push, 1)
typedef struct {
  uint8_t  magic1;   // 0xCA
  uint8_t  magic2;   // 0xFE
  uint8_t  version;  // 1
  uint8_t  count;

  float accelX, accelY, accelZ;
  float gyroX,  gyroY,  gyroZ;
  float pressure;
  float temp;
  float velocityX, velocityY, velocityZ;
  float altitude;

} TelemetryF32V1;
#pragma pack(pop)

// Shared global data
extern TelemetryF32V1 transmittedData;
extern SemaphoreHandle_t dataMutex;

// Entry tasks
void lora_task(void *arg);
void baro_task(void *arg);
void gps_task(void *arg);
// void tmp117_task(void *arg);
void servo_task(void *arg);
void lsm9ds1_task(void *arg);
void i2c_init(void);

#ifdef __cplusplus
}
#endif

#endif