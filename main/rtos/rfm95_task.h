#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <RadioLib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// 1. The Extern Bridge: Tell this file where the PSRAM data is
extern uint8_t *image_buffer;
extern size_t image_size;

// Global state trackers for the image transmission
static uint16_t current_chunk_index = 0;
static bool is_sending_image = false; // Set to true when you snap a photo

// 2. The Packed Struct: Prevent 32-bit memory padding corruption
struct __attribute__((packed)) ImageChunkPacket {
  uint8_t magic1;
  uint8_t magic2;
  uint16_t image_id;
  uint16_t total_chunks;
  uint16_t chunk_index;
  uint8_t payload[50]; // Sized specifically for the 64-byte hardware FIFO
};

// Only declare the task entry point
void rfm95_task(void *arg);
void sendNextImageChunk(SX1276 *radio);

#ifdef __cplusplus
}
#endif