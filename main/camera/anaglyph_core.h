#ifndef ANAGLYPH_CORE_H
#define ANAGLYPH_CORE_H

#include "esp_err.h"
#include "ptc06.h"
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

esp_err_t run_anaglyph_capture_cycle(ptc06_t *cam,
                                     uint32_t delay_between_photos_ms);

void camera_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // ANAGLYPH_CORE_H