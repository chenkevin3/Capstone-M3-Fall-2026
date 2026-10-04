# Steadi Firmware

Firmware for the Team M3 anti-tremor mouse, built on the Raspberry Pi Pico SDK (RP2350 / Pico 2).

## Setup

1. Install the toolchain and SDK, and set `PICO_SDK_PATH` to your local `pico-sdk` clone.
2. Copy `pico_sdk_import.cmake` from `$PICO_SDK_PATH/external/` into the repo root
   (required by `CMakeLists.txt`; not included here so it always matches your SDK version).

## Build

```bash
mkdir build && cd build
cmake ..
make
```

Flash `build/steadi.uf2` by holding BOOTSEL while plugging the board in, then dragging the file onto the drive.

## Current status

- Board/peripheral init (I2C for IMU, SPI mode 3 for optical sensor, buttons, status LED)
- Heartbeat LED + USB serial message confirm the firmware is running

Pin assignments live in `src/pins.h` and are placeholders until the wiring is finalized.
