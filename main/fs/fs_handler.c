#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <stdio.h>


void mountFileSystem() {
  ESP_LOGI("FS", "Initializing SPIFFS");

  esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                                .partition_label =
                                    "storage", // Match the name in CSV
                                .max_files = 5,
                                .format_if_mount_failed = true};

  esp_err_t ret = esp_vfs_spiffs_register(&conf);

  if (ret != ESP_OK) {
    ESP_LOGE("FS", "Failed to mount or format filesystem");
    return;
  }
  ESP_LOGI("FS", "SPIFFS mounted successfully");
}

uint8_t *image_buffer = NULL;
size_t image_size = 0;

void loadImageIntoPSRAM() {
  // 1. Open the file in binary read mode ("rb")
  FILE *f = fopen("/spiffs/cansat.jpeg", "rb");
  if (f == NULL) {
    ESP_LOGE("FS", "Failed to open image file");
    return;
  }

  // 2. Measure the exact file size
  fseek(f, 0, SEEK_END);
  image_size = ftell(f);
  fseek(f, 0, SEEK_SET); // Reset pointer to start

  ESP_LOGI("FS", "Image size dynamically measured: %d bytes", image_size);

  // 3. Allocate PSRAM
  if (image_buffer != NULL) {
    heap_caps_free(image_buffer);
  }

  // MALLOC_CAP_SPIRAM forces the allocation into the R8 external RAM
  image_buffer = (uint8_t *)heap_caps_malloc(image_size, MALLOC_CAP_SPIRAM);

  if (image_buffer == NULL) {
    ESP_LOGE("FS", "Failed to allocate PSRAM!");
    fclose(f);
    return;
  }

  // 4. Read the bytes and close
  fread(image_buffer, 1, image_size, f);
  fclose(f);

  ESP_LOGI("FS", "Image loaded to PSRAM successfully.");
}

void replaceImageInPSRAM(const uint8_t *data, size_t len) {
  if (data == NULL || len == 0) {
    ESP_LOGE("FS", "replaceImageInPSRAM got invalid data");
    return;
  }

  if (image_buffer != NULL) {
    heap_caps_free(image_buffer);
    image_buffer = NULL;
    image_size = 0;
  }

  image_buffer =
      (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (image_buffer == NULL) {
    ESP_LOGE("FS", "Failed to allocate PSRAM for generated image");
    return;
  }

  memcpy(image_buffer, data, len);
  image_size = len;

  ESP_LOGI("FS", "Generated image copied to PSRAM: %u bytes",
           (unsigned)image_size);
}

/*
Memory partition explanation
Powered the Pins: We told the ESP32 to actually send electricity to the pins
connected to the RAM chip. By default, these are off to save power.

Synchronized the Clock: We set the communication protocol to Octal (OPI). This
ensures the CPU and the RAM are perfectly synced. If this was wrong, the RAM
would send "garbage" data, and the ESP32 would crash.

Mapped the Memory: We told the Heap Allocator (the part of the OS that manages
memory) that there is a new "territory" of 8MB available.

Before: When we called heap_caps_malloc(..., MALLOC_CAP_SPIRAM), the OS looked
at its map, saw 0MB of SPIRAM, and returned NULL.

After: The OS now sees the 8MB territory, marks your image_size as "taken," and
gives our code the starting address of that memory.
*/