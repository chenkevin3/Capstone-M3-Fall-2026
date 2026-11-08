#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "tremor_detect.h"

typedef struct {
    int16_t dx, dy;     // corrected movement to send as mouse input
} hid_delta_t;

void corrector_init(void);
// Called after each slow-path analysis (or when the bypass button changes)
void corrector_set_params(const tremor_params_t *p, bool bypass);
// Fast path: called once per sample
hid_delta_t corrector_process(float dx_counts, float dy_counts);
