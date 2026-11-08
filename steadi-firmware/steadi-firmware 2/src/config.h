#pragma once

// Tunable constants for the Steadi firmware pipeline.
// Every value here is a starting point to be tuned on real hardware.

// ---- Firmware mode ----
// BRINGUP: sensor checks, on-chip timing test, then raw data streaming.
//          Start here on new hardware.
// PIPELINE: the full tremor-filtering loop.
#define MODE_PIPELINE         0
#define MODE_BRINGUP          1
#ifndef FIRMWARE_MODE
#define FIRMWARE_MODE         MODE_BRINGUP
#endif

// ---- Sampling (Sense) ----
#define SAMPLE_RATE_HZ        200
#define SAMPLE_PERIOD_US      (1000000 / SAMPLE_RATE_HZ)

// ---- Analysis window (Decide / slow path) ----
// 128 samples at 200 Hz = 640 ms window -> ~1.56 Hz frequency resolution.
// A long window does NOT add cursor latency: the fast path never waits on it.
#define WINDOW_SIZE           128
#define ANALYSIS_EVERY_N      25        // re-analyze every 25 samples (125 ms)

#define TREMOR_BAND_LO_HZ     3.0f
#define TREMOR_BAND_HI_HZ     12.0f

// Fraction of spectral energy that must fall in the tremor band
#define BAND_FRACTION_THRESH  0.5f
// Feature-level fusion weights: optical vs. IMU band fractions
#define W_OPTICAL             0.7f
#define W_IMU                 0.3f

// Intensity thresholds (RMS tremor displacement, mm) from our research
#define INTENSITY_MIN_MM      0.5f      // below this: not corrected
#define INTENSITY_SEVERE_MM   2.5f      // at/above this: full correction depth

// Net cursor speed below this counts as "resting" (mm/s)
#define RESTING_SPEED_MM_S    5.0f

// ---- Correction (fast path) ----
#define CORRECTION_NOTCH      0
#define CORRECTION_WFLC       1
#ifndef CORRECTION_METHOD
#define CORRECTION_METHOD     CORRECTION_WFLC   // switch to CORRECTION_NOTCH to compare
#endif

// Notch settings (used when CORRECTION_METHOD is NOTCH).
// Q = 2 tested best in simulation: more tremor removed than Q = 1,
// and half the delay on intentional movement (15 ms vs. 30 ms).
#define NOTCH_Q               2.0f

// WFLC settings (used when CORRECTION_METHOD is WFLC).
// Learning rates were tuned in simulation (filter_compare.py).
#define WFLC_HARMONICS        2         // tremor fundamental + 2nd harmonic
#define WFLC_MU_W             0.02f     // stage 1 weight learning rate
#define WFLC_MU_F             0.001f    // frequency learning rate
#define WFLC_MU_FLC           0.08f     // stage 2 learning rate
#define WFLC_INIT_HZ          6.0f      // starting frequency guess
#define WFLC_RESEED_HZ        1.0f      // reset to the detector's frequency if WFLC is this far off
// Only reset when the detector actually sees tremor-band energy; otherwise
// it is reporting the frequency of sensor noise
#define WFLC_SEED_MIN_FRACTION BAND_FRACTION_THRESH

// Shared correction settings
#define DEPTH_MIN             0.4f      // correction depth for mild tremor
#define DEPTH_RAMP            0.05f     // per-sample smoothing of depth changes

// ---- Optical sensor ----
// Must match the resolution the PMW3389 is actually configured to.
#define OPTICAL_CPI           1600
#define COUNTS_TO_MM          (25.4f / OPTICAL_CPI)
