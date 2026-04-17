#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <RadioLib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t *image_buffer;
extern size_t image_size;

extern uint16_t current_chunk_index;
extern bool is_sending_image;
extern bool image_is_ready_to_send;

struct __attribute__((packed)) ImageChunkPacket {
  uint8_t magic1;
  uint8_t magic2;
  uint16_t image_id;
  uint16_t total_chunks;
  uint16_t chunk_index;
  uint8_t payload[50];
};

void rfm95_task(void *arg);

#ifdef __cplusplus
}
#endif

void sendNextImageChunk(SX1276 *radio);