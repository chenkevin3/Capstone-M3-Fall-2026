# Steadi Firmware

Firmware for the Team M3 anti-tremor mouse, built on the Raspberry Pi Pico SDK (RP2350 / Pico 2).

## Repository layout

| Path | What it is |
|---|---|
| `src/` | Firmware (C) |
| `tools/filter_compare.py` | Python simulation comparing tremor filters |
| `tools/mouse_logger.py` | Records real movement from any ordinary mouse |
| `tools/capture_serial.py` | Records the board's raw sensor stream (bring-up mode) |
| `tests/` | PC-side check that the C firmware matches the Python simulation |

## Firmware modules

| File | Stage | Purpose |
|---|---|---|
| `config.h` | all | Firmware mode, correction method, all tunable constants |
| `pins.h`, `board_init.c` | setup | Pin map and peripheral initialization |
| `icm20948.c` | Sense | IMU init and reads (accel in g, gyro in deg/s) |
| `pmw3389.c` | Sense | Optical sensor power-up, SROM upload, motion reads |
| `sampler.c` | Sense | Fixed 200 Hz sample timing |
| `tremor_detect.c` | Decide | Spectral detector: tremor yes/no, frequency, intensity, motion state |
| `correction.c` | Correct | Applies the chosen correction method, scaled by intensity |
| `wflc.c` | Correct | Detector-seeded WFLC tremor canceller (default method) |
| `bringup.c` | Test | Bring-up test program for new hardware |
| `main.c` | all | Runs bring-up mode or the full pipeline |

## Setup

1. Install the toolchain and SDK, and set `PICO_SDK_PATH` to your local `pico-sdk` clone.
2. Copy `pico_sdk_import.cmake` from `$PICO_SDK_PATH/external/` into the repo root.
3. Edit `src/pins.h` to match your wiring.

## Build and flash

```bash
mkdir build && cd build
cmake ..
make
```

Flash `build/steadi.uf2` by holding BOOTSEL while plugging the board in, then dragging the file onto the drive.

## Bring-up mode (default)

`FIRMWARE_MODE` in `config.h` is set to `MODE_BRINGUP`. Open a serial monitor and the board runs:

1. **Sensor checks:** PASS/FAIL for each chip, and an IMU reading of about 1 g at rest.
2. **On-chip timing:** runs the detector and correction on a simulated tremor and reports
   how long each step takes, against the 5000 us sample period. Needs no sensors.
3. **Raw data streaming:** press any key, or run `tools/capture_serial.py`, to stream
   sensor readings as CSV at 200 Hz.

Set `FIRMWARE_MODE` to `MODE_PIPELINE` for the full tremor-filtering loop.

## PC-side tests

From the repo root:

```bash
gcc -std=c11 -O2 -DCORRECTION_METHOD=CORRECTION_WFLC  -Isrc tests/host_harness.c src/tremor_detect.c src/correction.c src/wflc.c -lm -o tests/harness_WFLC
gcc -std=c11 -O2 -DCORRECTION_METHOD=CORRECTION_NOTCH -Isrc tests/host_harness.c src/tremor_detect.c src/correction.c src/wflc.c -lm -o tests/harness_NOTCH
python tests/compare_c_python.py
```

## Current status

- Not yet compiled with the real Pico SDK or run on hardware.
- Detection and correction tested on a PC: the C version matches the Python simulation to within 1%.
- Optical tracking needs the PMW3389 SROM image from PixArt, which is not included.
- Not yet implemented: USB HID mouse output, Bluetooth, calibration profiles, dual-core split.
