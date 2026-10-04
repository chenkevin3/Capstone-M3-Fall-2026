#include <stdio.h>
#include "pico/stdlib.h"
#include "board_init.h"
#include "pins.h"

int main(void) {
    board_init();
    sleep_ms(2000);                // time to open the serial monitor

    printf("Steadi firmware: board init complete\n");

    // Next steps slot in here, in order:
    // icm20948_init();
    // pmw3389_init();             // power-up + SROM upload
    // start sampling timer
    // launch core 1 (output)

    while (true) {
        gpio_put(LED_PIN, 1); sleep_ms(250);
        gpio_put(LED_PIN, 0); sleep_ms(250);   // heartbeat: firmware is alive
    }
}
