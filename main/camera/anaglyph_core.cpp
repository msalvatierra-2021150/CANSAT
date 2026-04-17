#include "anaglyph_core.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fs/fs_handler.h"
#include "img_converters.h"
#include "ptc06.h"
#include "rfm95_task.h"


#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define CHANNELS 3

typedef struct {
  int x;
  int y;
} offset_2d_t;

static void scale_image_down(uint8_t *src, uint8_t *dst, float scale_factor);
static offset_2d_t calculate_alignment_offset(uint8_t *img_left,
                                              uint8_t *img_right);
static void apply_anaglyph_merge(uint8_t *img_target, uint8_t *img_right,
                                 offset_2d_t offset);
static uint8_t *decode_jpeg_to_rgb(uint8_t *jpeg_data, size_t jpeg_len);
static uint8_t *encode_rgb_to_jpeg(uint8_t *rgb_data, int quality,
                                   size_t *out_size);

static const char *TAG_CAM = "CAMERA";

esp_err_t run_anaglyph_capture_cycle(ptc06_t *cam,
                                     uint32_t delay_between_photos_ms) {
  esp_err_t ret = ESP_OK;

  if (cam == NULL) {
    ESP_LOGE(TAG_CAM, "Camera handle is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t *jpg1 = NULL, *jpg2 = NULL;
  size_t len1 = 0, len2 = 0;
  uint8_t *rgb1 = NULL, *rgb2 = NULL;
  uint8_t *rgb1_scaled = NULL;
  uint8_t *anaglyph_jpeg = NULL;
  size_t anaglyph_jpeg_size = 0;
  offset_2d_t offset = {0, 0};

  ESP_LOGI(TAG_CAM, "[1/7] Capturing photo 1...");
  if (ptc06_capture_jpeg_to_buffer(cam, &jpg1, &len1) != ESP_OK) {
    ESP_LOGE(TAG_CAM, "Failed to capture photo 1");
    ret = ESP_FAIL;
    goto cleanup;
  }

  vTaskDelay(pdMS_TO_TICKS(delay_between_photos_ms));

  ESP_LOGI(TAG_CAM, "[2/7] Capturing photo 2...");
  if (ptc06_capture_jpeg_to_buffer(cam, &jpg2, &len2) != ESP_OK) {
    ESP_LOGE(TAG_CAM, "Failed to capture photo 2");
    ret = ESP_FAIL;
    goto cleanup;
  }

  ESP_LOGI(TAG_CAM, "[3/7] Decoding JPEGs...");
  rgb1 = decode_jpeg_to_rgb(jpg1, len1);
  rgb2 = decode_jpeg_to_rgb(jpg2, len2);

  if (!(rgb1 && rgb2)) {
    ESP_LOGE(TAG_CAM, "JPEG decode failed");
    ret = ESP_FAIL;
    goto cleanup;
  }

  ESP_LOGI(TAG_CAM, "[4/7] Scaling photo 1...");
  rgb1_scaled =
      (uint8_t *)heap_caps_malloc(IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!rgb1_scaled) {
    ESP_LOGE(TAG_CAM, "Failed to allocate scaled image");
    ret = ESP_ERR_NO_MEM;
    goto cleanup;
  }

  scale_image_down(rgb1, rgb1_scaled, 0.909f);
  heap_caps_free(rgb1);
  rgb1 = rgb1_scaled;
  rgb1_scaled = NULL;

  ESP_LOGI(TAG_CAM, "[5/7] Calculating alignment...");
  offset = calculate_alignment_offset(rgb1, rgb2);
  ESP_LOGI(TAG_CAM, "Detected offset: X=%d Y=%d", offset.x, offset.y);

  ESP_LOGI(TAG_CAM, "[6/7] Building anaglyph...");
  apply_anaglyph_merge(rgb1, rgb2, offset);

  ESP_LOGI(TAG_CAM, "[7/7] Encoding JPEG...");
  anaglyph_jpeg = encode_rgb_to_jpeg(rgb1, 85, &anaglyph_jpeg_size);
  if (!anaglyph_jpeg) {
    ESP_LOGE(TAG_CAM, "Failed to encode anaglyph JPEG");
    ret = ESP_FAIL;
    goto cleanup;
  }

  replaceImageInPSRAM(anaglyph_jpeg, anaglyph_jpeg_size);
  image_is_ready_to_send = true;

  ESP_LOGI(TAG_CAM, "Anaglyph queued for radio TX (%d bytes)",
           (int)anaglyph_jpeg_size);

cleanup:
  if (anaglyph_jpeg) {
    free(anaglyph_jpeg);
  }
  if (rgb1) {
    heap_caps_free(rgb1);
  }
  if (rgb2) {
    heap_caps_free(rgb2);
  }
  if (jpg1) {
    free(jpg1);
  }
  if (jpg2) {
    free(jpg2);
  }

  return ret;
}

static uint8_t *decode_jpeg_to_rgb(uint8_t *jpeg_data, size_t jpeg_len) {
  if (!jpeg_data || jpeg_len == 0) {
    return NULL;
  }

  uint8_t *rgb =
      (uint8_t *)heap_caps_malloc(IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!rgb) {
    ESP_LOGE(TAG_CAM, "No memory for RGB decode buffer");
    return NULL;
  }

  if (!fmt2rgb888(jpeg_data, jpeg_len, PIXFORMAT_JPEG, rgb)) {
    ESP_LOGE(TAG_CAM, "fmt2rgb888 failed");
    heap_caps_free(rgb);
    return NULL;
  }

  return rgb;
}

static uint8_t *encode_rgb_to_jpeg(uint8_t *rgb_data, int quality,
                                   size_t *out_size) {
  if (!rgb_data || !out_size) {
    return NULL;
  }

  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;

  if (!fmt2jpg(rgb_data, IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS, IMAGE_WIDTH,
               IMAGE_HEIGHT, PIXFORMAT_RGB888, quality, &jpg_buf, &jpg_len)) {
    ESP_LOGE(TAG_CAM, "fmt2jpg failed");
    return NULL;
  }

  *out_size = jpg_len;
  return jpg_buf;
}

static void scale_image_down(uint8_t *src, uint8_t *dst, float scale_factor) {
  if (!src || !dst || scale_factor <= 0.0f) {
    return;
  }

  const float inv = 1.0f / scale_factor;

  for (int y = 0; y < IMAGE_HEIGHT; ++y) {
    for (int x = 0; x < IMAGE_WIDTH; ++x) {
      int sx = (int)(x * inv);
      int sy = (int)(y * inv);

      if (sx < 0)
        sx = 0;
      if (sy < 0)
        sy = 0;
      if (sx >= IMAGE_WIDTH)
        sx = IMAGE_WIDTH - 1;
      if (sy >= IMAGE_HEIGHT)
        sy = IMAGE_HEIGHT - 1;

      int dst_idx = (y * IMAGE_WIDTH + x) * CHANNELS;
      int src_idx = (sy * IMAGE_WIDTH + sx) * CHANNELS;

      dst[dst_idx + 0] = src[src_idx + 0];
      dst[dst_idx + 1] = src[src_idx + 1];
      dst[dst_idx + 2] = src[src_idx + 2];
    }
  }
}

static offset_2d_t calculate_alignment_offset(uint8_t *img_left,
                                              uint8_t *img_right) {
  offset_2d_t best = {0, 0};

  if (!img_left || !img_right) {
    return best;
  }

  const int max_shift = 12;
  long best_score = LONG_MAX;

  for (int dy = -max_shift; dy <= max_shift; ++dy) {
    for (int dx = -max_shift; dx <= max_shift; ++dx) {
      long score = 0;

      for (int y = max_shift; y < IMAGE_HEIGHT - max_shift; y += 4) {
        for (int x = max_shift; x < IMAGE_WIDTH - max_shift; x += 4) {
          int xr = x + dx;
          int yr = y + dy;

          if (xr < 0 || xr >= IMAGE_WIDTH || yr < 0 || yr >= IMAGE_HEIGHT) {
            continue;
          }

          int idx_l = (y * IMAGE_WIDTH + x) * CHANNELS;
          int idx_r = (yr * IMAGE_WIDTH + xr) * CHANNELS;

          int gl = (img_left[idx_l + 0] + img_left[idx_l + 1] +
                    img_left[idx_l + 2]) /
                   3;
          int gr = (img_right[idx_r + 0] + img_right[idx_r + 1] +
                    img_right[idx_r + 2]) /
                   3;

          int diff = gl - gr;
          score += (diff < 0) ? -diff : diff;
        }
      }

      if (score < best_score) {
        best_score = score;
        best.x = dx;
        best.y = dy;
      }
    }
  }

  return best;
}

static void apply_anaglyph_merge(uint8_t *img_target, uint8_t *img_right,
                                 offset_2d_t offset) {
  if (!img_target || !img_right) {
    return;
  }

  uint8_t *merged =
      (uint8_t *)heap_caps_malloc(IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS,
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!merged) {
    ESP_LOGE(TAG_CAM, "No memory for anaglyph merge");
    return;
  }

  memset(merged, 0, IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS);

  for (int y = 0; y < IMAGE_HEIGHT; ++y) {
    for (int x = 0; x < IMAGE_WIDTH; ++x) {
      int left_idx = (y * IMAGE_WIDTH + x) * CHANNELS;

      int xr = x + offset.x;
      int yr = y + offset.y;

      int dst_idx = left_idx;
      merged[dst_idx + 0] = img_target[left_idx + 0];

      if (xr >= 0 && xr < IMAGE_WIDTH && yr >= 0 && yr < IMAGE_HEIGHT) {
        int right_idx = (yr * IMAGE_WIDTH + xr) * CHANNELS;
        merged[dst_idx + 1] = img_right[right_idx + 1];
        merged[dst_idx + 2] = img_right[right_idx + 2];
      }
    }
  }

  memcpy(img_target, merged, IMAGE_WIDTH * IMAGE_HEIGHT * CHANNELS);
  heap_caps_free(merged);
}