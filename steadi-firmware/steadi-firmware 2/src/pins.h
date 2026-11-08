#pragma once

// Pin map for the Steadi prototype.
// Change these to match the actual wiring -- nothing else should hardcode pins.

// IMU (ICM-20948) on I2C0
#define IMU_I2C        i2c0
#define IMU_SDA_PIN    4
#define IMU_SCL_PIN    5
#define IMU_I2C_HZ     (400 * 1000)

// Optical sensor (PMW3389) on SPI0
#define OPT_SPI        spi0
#define OPT_SCK_PIN    18
#define OPT_MOSI_PIN   19
#define OPT_MISO_PIN   16
#define OPT_CS_PIN     17
#define OPT_MOTION_PIN 20                  // optional: sensor's "motion detected" line
#define OPT_SPI_HZ     (2 * 1000 * 1000)   // PMW3389 max is 2 MHz

// User inputs
#define BTN_LEFT_PIN    10
#define BTN_RIGHT_PIN   11
#define BTN_BYPASS_PIN  12     // the "disable filtering" button

#define LED_PIN         25     // onboard LED as a status light
