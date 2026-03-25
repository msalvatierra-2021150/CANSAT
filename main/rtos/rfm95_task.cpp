// --- RadioLib Includes ---
#include "rfm95_task.h"
#include "EspHal/EspHal.h"
#include <RadioLib.h>
#include <string.h>
#include "tasks.h"

// SX1276 Pins (SX1276) - ESP32-S3
#define SX1276_SCK 12
#define SX1276_MISO 13
#define SX1276_MOSI 11
#define SX1276_CS 10
#define SX1276_RST 14
#define SX1276_DIO0 15
#define SX1276_DIO1 16

typedef enum { SEND_TELEMETRY, BURST_IMAGE, LISTEN_FOR_NACK } DeviceState_t;

DeviceState_t currentState = SEND_TELEMETRY;

static const char *TAG_RFM95 = "RFM95";

bool image_is_ready_to_send = true;

void rfm95_task(void *arg) {
  ESP_LOGI(TAG_RFM95, "Setting up RFM95 Hardware...");

  // 1. Initialize HAL (SPI)
  EspHal *hal = new EspHal(SX1276_SCK, SX1276_MISO, SX1276_MOSI);

  // 2. Initialize Radio Module
  ESP_LOGI(TAG_RFM95, "[SX1276] Initializing FSK...");

  Module *module =
      new Module(hal, SX1276_CS, SX1276_DIO0, SX1276_RST, SX1276_DIO1);

  SX1276 *radio = new SX1276(module);

  int state = radio->beginFSK(915.0, 50.0, 25.0, 125.0, 17, 40);

  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG_RFM95, "FSK init failed: %d", state);
    while (true)
      vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ESP_LOGI(TAG_RFM95, "FSK init success");

  // Match RX packet settings:
  radio->setCRC(true);
  radio->variablePacketLengthMode();

  uint8_t syncWord[] = {0x2D, 0xD4};

  radio->setSyncWord(syncWord, 2);

  static uint16_t pktCounter = 0;

  // 4. Transmission Loop
  while (1) {
    switch (currentState) {
    case SEND_TELEMETRY:
      // 1. Create a local copy of data to minimize mutex holding time
      TelemetryF32V1 localData;
      // 2.  Take Mutex
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        localData = transmittedData; // Copy global to local
        xSemaphoreGive(dataMutex);   // Release Mutex
      }

      ESP_LOGI(TAG_RFM95, "Sending");

      // 3. Transmit raw binary struct
      localData.magic1 = 0xCA;
      localData.magic2 = 0xFE;
      localData.version = 1;
      localData.count = pktCounter++;

      state = radio->transmit((uint8_t *)&localData, sizeof(localData));

      printf("State: %d\n", sizeof(localData));

      if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG_RFM95, "TX success!");
      } else {
        ESP_LOGE(TAG_RFM95, "TX failed, code %d", state);
      }

      // Wait for 10 milisecond
      vTaskDelay(pdMS_TO_TICKS(10));

      // After sending telemetry, check if a new image is ready in your buffer
      // image_is_ready_to_send = true;
      if (image_is_ready_to_send) {
        currentState = BURST_IMAGE;
        // currentChunkToSend = 0;
        current_chunk_index = 0;
      } else {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Normal telemetry delay
      }
      break;

    case BURST_IMAGE:
      // 1. Construct ImageChunkPacket for 'currentChunkToSend'
      // 2. Read 200 bytes from your JPEG buffer
      // 3. Transmit it: radio->transmit((uint8_t*)&chunk, sizeof(chunk));
      sendNextImageChunk(radio);

      // Check if the chunking function finished the image and reset the index
      if (current_chunk_index == 0) {
        ESP_LOGI(TAG_RFM95, "Image complete! Returning to telemetry.");
        image_is_ready_to_send = false;
        currentState = SEND_TELEMETRY;
      }

      vTaskDelay(pdMS_TO_TICKS(10));
      break;

    case LISTEN_FOR_NACK:
      // Wait briefly to see if Ground Station requests missing chunks
      // If timeout -> Assume success (or wait for Ground Station ACK) -> go
      // back to SEND_TELEMETRY If NACK received -> Parse the missing_chunks
      // array, resend those specific chunks
      break;
    }
  }
}

void sendNextImageChunk(SX1276 *radio) {
  if (image_buffer == NULL || image_size == 0)
    return;

  // Set to 50 to match our safe hardware limit
  uint16_t payload_size = 50;
  uint16_t total_chunks = (image_size + payload_size - 1) / payload_size;

  if (current_chunk_index >= total_chunks) {
    is_sending_image = false;
    current_chunk_index = 0;
    return;
  }

  ImageChunkPacket chunk;
  chunk.magic1 = 0xBE;
  chunk.magic2 = 0xEF;
  chunk.image_id = 1;
  chunk.total_chunks = total_chunks;
  chunk.chunk_index = current_chunk_index;

  uint16_t current_payload_size = payload_size;
  if (current_chunk_index == total_chunks - 1) {
    current_payload_size = image_size - (current_chunk_index * payload_size);
  }

  uint32_t buffer_offset = current_chunk_index * payload_size;
  memcpy(chunk.payload, image_buffer + buffer_offset, current_payload_size);

  size_t packet_size_to_send =
      sizeof(chunk) - payload_size + current_payload_size;

  // Transmit ONE 58-byte chunk (fits easily in the 64-byte FIFO)
  radio->transmit((uint8_t *)&chunk, packet_size_to_send);
  ESP_LOGI("RFM95", "Sent chunk %d of %d", chunk.chunk_index,
           chunk.total_chunks);

  current_chunk_index++;
}