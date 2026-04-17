#ifndef FS_HANDLER_H
#define FS_HANDLER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void mountFileSystem(void);
void loadImageIntoPSRAM(void);

extern uint8_t *image_buffer;
extern size_t image_size;

void replaceImageInPSRAM(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif