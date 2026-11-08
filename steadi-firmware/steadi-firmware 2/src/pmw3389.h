#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t dx, dy;     // counts since last read
    bool moved;
} optical_motion_t;

bool pmw3389_check_id(void);
// srom may be NULL until the SROM image is added to the project
bool pmw3389_init(const uint8_t *srom, size_t srom_len);
optical_motion_t pmw3389_read_motion(void);
