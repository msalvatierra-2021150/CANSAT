#include "fs/fs.h"

void chunkingImage(SX1276 *radio) {
  // Assuming 'image_buffer' is a pointer to your JPEG data in PSRAM
  // Assuming 'image_size' is 112640

  uint16_t payload_size = 200;
  uint16_t total_chunks =
      (image_size + payload_size - 1) / payload_size; // Ceiling division

  for (uint16_t i = 0; i < total_chunks; i++) {
    ImageChunkPacket chunk;
    chunk.magic1 = 0xBE;
    chunk.magic2 = 0xEF;
    chunk.image_id = current_image_id;
    chunk.total_chunks = total_chunks;
    chunk.chunk_index = i;

    // Calculate how many bytes to copy for THIS specific chunk
    uint16_t current_payload_size = payload_size;

    // If it's the very last chunk, it might be smaller than 200 bytes
    if (i == total_chunks - 1) {
      current_payload_size = image_size - (i * payload_size);
    }

    // Calculate the memory offset and copy directly from the source
    uint32_t buffer_offset = i * payload_size;
    memcpy(chunk.payload, image_buffer + buffer_offset, current_payload_size);

    // The total packet size shrinks on the final transmission
    size_t packet_size_to_send =
        sizeof(chunk) - payload_size + current_payload_size;

    // Transmit
    radio->transmit((uint8_t *)&chunk, packet_size_to_send);

    // Keep this delay extremely tight, or remove it entirely if the RFM95 FIFO
    // can keep up
    currentChunkToSend++;

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
