#include "sampler.h"
#include "config.h"
#include <stddef.h>
#include "pico/stdlib.h"

// The timer interrupt only raises a flag. Sensor reads happen in the main
// loop, which keeps slow, blocking I2C/SPI transfers out of interrupt context.
static struct repeating_timer timer;
static volatile bool tick = false;
static volatile uint32_t overruns = 0;

static bool on_tick(struct repeating_timer *t) {
    (void)t;
    if (tick) overruns++;   // previous tick not consumed in time
    tick = true;
    return true;            // keep repeating
}

void sampler_start(void) {
    // Negative period = fixed spacing between callback starts
    add_repeating_timer_us(-SAMPLE_PERIOD_US, on_tick, NULL, &timer);
}

uint64_t sampler_wait(void) {
    while (!tick) {
        tight_loop_contents();
    }
    tick = false;
    return time_us_64();
}

uint32_t sampler_overruns(void) {
    return overruns;
}
