#include "pmw3389.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>

// Register map (verify against the PMW3389 datasheet)
#define REG_PRODUCT_ID       0x00
#define REG_MOTION           0x02
#define REG_DELTA_X_L        0x03
#define REG_DELTA_X_H        0x04
#define REG_DELTA_Y_L        0x05
#define REG_DELTA_Y_H        0x06
#define REG_CONFIG2          0x10
#define REG_SROM_ENABLE      0x13
#define REG_SROM_ID          0x2A
#define REG_POWER_UP_RESET   0x3A
#define REG_SROM_LOAD_BURST  0x62

#define PRODUCT_ID_EXPECTED  0x47

// SPI timing in microseconds (verify against the datasheet timing table)
#define T_SRAD_US      160   // address -> data delay on a read
#define T_SCLK_NCS_W   35    // last clock -> CS high, after a write
#define T_SWX_US       180   // gap after a write before the next transaction
#define T_SRX_US       20    // gap after a read before the next transaction
#define T_SROM_BYTE    15    // gap between SROM burst bytes

static inline void cs_low(void)  { gpio_put(OPT_CS_PIN, 0); }
static inline void cs_high(void) { gpio_put(OPT_CS_PIN, 1); }

static uint8_t read_reg(uint8_t reg) {
    uint8_t addr = reg & 0x7F;           // MSB = 0 -> read
    uint8_t val = 0;
    cs_low();
    spi_write_blocking(OPT_SPI, &addr, 1);
    sleep_us(T_SRAD_US);
    spi_read_blocking(OPT_SPI, 0x00, &val, 1);
    sleep_us(1);
    cs_high();
    sleep_us(T_SRX_US);
    return val;
}

static void write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { (uint8_t)(reg | 0x80), val };   // MSB = 1 -> write
    cs_low();
    spi_write_blocking(OPT_SPI, buf, 2);
    sleep_us(T_SCLK_NCS_W);
    cs_high();
    sleep_us(T_SWX_US);
}

bool pmw3389_check_id(void) {
    uint8_t id = read_reg(REG_PRODUCT_ID);
    printf("PMW3389 Product_ID: 0x%02X (expected 0x%02X)\n", id, PRODUCT_ID_EXPECTED);
    return id == PRODUCT_ID_EXPECTED;
}

// The sensor needs a firmware image (SROM) uploaded at every power-up.
// The image is distributed by PixArt and is not included in this repo.
static bool upload_srom(const uint8_t *srom, size_t len) {
    write_reg(REG_CONFIG2, 0x00);        // disable rest mode during upload
    write_reg(REG_SROM_ENABLE, 0x1D);    // initialize SROM download
    sleep_ms(10);
    write_reg(REG_SROM_ENABLE, 0x18);    // start SROM download

    uint8_t cmd = REG_SROM_LOAD_BURST | 0x80;
    cs_low();
    spi_write_blocking(OPT_SPI, &cmd, 1);
    sleep_us(T_SROM_BYTE);
    for (size_t i = 0; i < len; i++) {
        spi_write_blocking(OPT_SPI, &srom[i], 1);
        sleep_us(T_SROM_BYTE);
    }
    cs_high();
    sleep_us(200);

    uint8_t srom_id = read_reg(REG_SROM_ID);
    printf("PMW3389 SROM_ID after upload: 0x%02X\n", srom_id);
    write_reg(REG_CONFIG2, 0x00);
    return srom_id != 0x00;
}

bool pmw3389_init(const uint8_t *srom, size_t srom_len) {
    cs_high(); cs_low(); cs_high();      // reset the sensor's SPI port

    write_reg(REG_POWER_UP_RESET, 0x5A);
    sleep_ms(50);

    // Datasheet power-up step: read the motion registers once to clear them
    for (uint8_t r = REG_MOTION; r <= REG_DELTA_Y_H; r++) {
        (void)read_reg(r);
    }

    if (srom == NULL || srom_len == 0) {
        printf("PMW3389: no SROM image provided, tracking may not work\n");
        return false;
    }
    bool ok = upload_srom(srom, srom_len);
    sleep_ms(10);
    // TODO: configure resolution so it matches OPTICAL_CPI in config.h
    return ok;
}

optical_motion_t pmw3389_read_motion(void) {
    optical_motion_t m = {0};

    write_reg(REG_MOTION, 0x01);         // any write latches the delta registers
    uint8_t motion = read_reg(REG_MOTION);
    uint8_t xl = read_reg(REG_DELTA_X_L);
    uint8_t xh = read_reg(REG_DELTA_X_H);
    uint8_t yl = read_reg(REG_DELTA_Y_L);
    uint8_t yh = read_reg(REG_DELTA_Y_H);

    m.moved = (motion & 0x80) != 0;
    m.dx = (int16_t)((xh << 8) | xl);
    m.dy = (int16_t)((yh << 8) | yl);
    return m;
}
