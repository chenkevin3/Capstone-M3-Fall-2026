#include "board_init.h"
#include "pins.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

static void init_imu_bus(void) {
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);     // internal pull-ups; fine for short wires
    gpio_pull_up(IMU_SCL_PIN);
}

static void init_optical_bus(void) {
    spi_init(OPT_SPI, OPT_SPI_HZ);
    // PMW3389 uses SPI mode 3: clock idles high, data sampled on rising edge
    spi_set_format(OPT_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(OPT_SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(OPT_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(OPT_MISO_PIN, GPIO_FUNC_SPI);

    // Chip-select is driven manually, because the PMW3389 needs
    // specific delays between CS edges and data transfers
    gpio_init(OPT_CS_PIN);
    gpio_set_dir(OPT_CS_PIN, GPIO_OUT);
    gpio_put(OPT_CS_PIN, 1);       // idle high (deselected)

    gpio_init(OPT_MOTION_PIN);
    gpio_set_dir(OPT_MOTION_PIN, GPIO_IN);
    gpio_pull_up(OPT_MOTION_PIN);
}

static void init_button(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);             // pressed = reads 0
}

void board_init(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    init_imu_bus();
    init_optical_bus();

    init_button(BTN_LEFT_PIN);
    init_button(BTN_RIGHT_PIN);
    init_button(BTN_BYPASS_PIN);
}
