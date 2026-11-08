#pragma once

// Bring-up test program for new hardware. Runs three stages in order:
//   1. Sensor checks: does each chip respond, and does the IMU read ~1 g at rest?
//   2. On-chip timing: runs the detector and correction on a simulated tremor
//      and measures how long each step takes on the RP2350. Needs no sensors.
//   3. Raw data streaming: prints sensor readings as CSV at 200 Hz, for
//      recording with tools/capture_serial.py.
// Enabled with FIRMWARE_MODE = MODE_BRINGUP in config.h.
void bringup_run(void);

// Stage 2 on its own (also used by the PC-side test)
void bringup_benchmark(void);
