#include "icm20948.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define ICM_ADDR               0x68     // AD0 low
#define REG_BANK_SEL           0x7F     // same address in every bank

// Bank 0
#define B0_WHO_AM_I            0x00
#define B0_PWR_MGMT_1          0x06
#define B0_PWR_MGMT_2          0x07
#define B0_ACCEL_XOUT_H        0x2D     // accel X..Z then gyro X..Z, 12 bytes

// Bank 2
#define B2_GYRO_SMPLRT_DIV     0x00
#define B2_GYRO_CONFIG_1       0x01
#define B2_ACCEL_SMPLRT_DIV_1  0x10
#define B2_ACCEL_SMPLRT_DIV_2  0x11
#define B2_ACCEL_CONFIG        0x14

#define WHO_AM_I_EXPECTED      0xEA

// Scale factors for the ranges configured below
#define ACCEL_LSB_PER_G        16384.0f   // +/-2 g
#define GYRO_LSB_PER_DPS       131.0f     // +/-250 dps

static void write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(IMU_I2C, ICM_ADDR, buf, 2, false);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t val = 0;
    i2c_write_blocking(IMU_I2C, ICM_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C, ICM_ADDR, &val, 1, false);
    return val;
}

static void select_bank(uint8_t bank) {
    write_reg(REG_BANK_SEL, (uint8_t)(bank << 4));   // bank in bits [5:4]
}

bool icm20948_check_id(void) {
    select_bank(0);
    uint8_t id = read_reg(B0_WHO_AM_I);
    printf("ICM-20948 WHO_AM_I: 0x%02X (expected 0x%02X)\n", id, WHO_AM_I_EXPECTED);
    return id == WHO_AM_I_EXPECTED;
}

void icm20948_init(void) {
    select_bank(0);
    write_reg(B0_PWR_MGMT_1, 0x80);   // device reset
    sleep_ms(50);
    write_reg(B0_PWR_MGMT_1, 0x01);   // wake from sleep, auto-select clock
    sleep_ms(10);
    write_reg(B0_PWR_MGMT_2, 0x00);   // enable all accel + gyro axes

    select_bank(2);
    // Output data rate = 1125 / (1 + div) Hz. div = 4 -> 225 Hz,
    // just above our 200 Hz sampling rate so every read gets fresh data.
    write_reg(B2_GYRO_CONFIG_1, 0x01);       // +/-250 dps, DLPF on
    write_reg(B2_GYRO_SMPLRT_DIV, 4);
    write_reg(B2_ACCEL_CONFIG, 0x01);        // +/-2 g, DLPF on
    write_reg(B2_ACCEL_SMPLRT_DIV_1, 0);     // 12-bit divider, high bits
    write_reg(B2_ACCEL_SMPLRT_DIV_2, 4);     // low bits

    select_bank(0);                          // data registers live in bank 0
}

imu_sample_t icm20948_read(void) {
    uint8_t raw[12];
    uint8_t reg = B0_ACCEL_XOUT_H;
    i2c_write_blocking(IMU_I2C, ICM_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C, ICM_ADDR, raw, 12, false);

    int16_t v[6];
    for (int i = 0; i < 6; i++) {
        v[i] = (int16_t)((raw[2 * i] << 8) | raw[2 * i + 1]);
    }

    imu_sample_t s = {
        .ax = v[0] / ACCEL_LSB_PER_G,
        .ay = v[1] / ACCEL_LSB_PER_G,
        .az = v[2] / ACCEL_LSB_PER_G,
        .gx = v[3] / GYRO_LSB_PER_DPS,
        .gy = v[4] / GYRO_LSB_PER_DPS,
        .gz = v[5] / GYRO_LSB_PER_DPS,
    };
    return s;
}
