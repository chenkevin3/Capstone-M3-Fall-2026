#pragma once
#include <stdbool.h>
#include <stdint.h>

// Output of the slow path, consumed by the fast-path corrector
typedef struct {
    bool  tremor;          // is this tremor? (frequency decision)
    float freq_hz;         // dominant tremor frequency
    float intensity_mm;    // how severe? (RMS tremor displacement)
    bool  resting;         // motion state: hand resting vs. moving the cursor
    float band_fraction;   // fused share of energy in the 3-12 Hz band
    uint32_t seq;          // increments on every completed analysis
} tremor_params_t;

void detector_init(void);
// Fast path: push one sample (optical in mm, accel in g)
void detector_push(float dx_mm, float dy_mm, float ax_g, float ay_g);
// Slow path: analyze the current window
void detector_analyze(tremor_params_t *out);
