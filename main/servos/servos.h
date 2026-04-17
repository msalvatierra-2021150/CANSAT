#ifndef SERVOS_H
#define SERVOS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void);
void servo_set_angle(uint32_t angle);

#ifdef __cplusplus
}
#endif

#endif