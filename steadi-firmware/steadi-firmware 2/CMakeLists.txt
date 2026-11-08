cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2)          # RP2350, not the original Pico
include(pico_sdk_import.cmake)

project(steadi_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(steadi
    src/main.c
    src/board_init.c
    src/icm20948.c
    src/pmw3389.c
    src/sampler.c
    src/tremor_detect.c
    src/correction.c
    src/wflc.c
    src/bringup.c
)

target_link_libraries(steadi
    pico_stdlib
    hardware_i2c
    hardware_spi
    pico_multicore      # for the dual-core split later
)

pico_enable_stdio_usb(steadi 1)    # printf over USB serial for debugging
pico_enable_stdio_uart(steadi 0)

pico_add_extra_outputs(steadi)     # generates the .uf2 you flash
