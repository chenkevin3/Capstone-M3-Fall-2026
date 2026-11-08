#pragma once
#include "config.h"

// Two-stage tremor canceller for one axis:
//   Stage 1, WFLC (Weighted-frequency Fourier Linear Combiner): tracks the
//            tremor frequency on a 3-12 Hz band-passed copy of the signal.
//   Stage 2, FLC (Fourier Linear Combiner): builds a tremor estimate at
//            that frequency and fits it to the raw signal.
// Based on Riviere, Rader & Thakor (1998), IEEE Trans. Biomed. Eng.
typedef struct {
    float bp[2][2];                       // band-pass filter state (2 biquads)
    float w0;                             // frequency estimate, rad/sample
    float phase_f;                        // stage 1 phase
    float wf[2 * WFLC_HARMONICS];         // stage 1 weights
    float phase_c;                        // stage 2 phase
    float wc[2 * WFLC_HARMONICS + 1];     // stage 2 weights (+ bias term)
    float power;                          // running mean of x^2, for scaling
    unsigned n;                           // samples seen
} wflc_t;

void  wflc_init(wflc_t *w, float f_init_hz);
// Jump the frequency estimate to f_hz (used when the detector disagrees)
void  wflc_reseed(wflc_t *w, float f_hz);
float wflc_freq_hz(const wflc_t *w);
// Feed one sample; returns the estimated tremor in the same units as x
float wflc_process(wflc_t *w, float x);
