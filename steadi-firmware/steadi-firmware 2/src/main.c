#include <stdio.h>
#include "pico/stdlib.h"
#include "board_init.h"
#include "pins.h"
#include "config.h"
#include "icm20948.h"
#include "pmw3389.h"
#include "sampler.h"
#include "tremor_detect.h"
#include "correction.h"
#include "bringup.h"

int main(void) {
    board_init();
    sleep_ms(2000);                       // time to open the serial monitor
    printf("Steadi firmware starting\n");

#if FIRMWARE_MODE == MODE_BRINGUP
    bringup_run();                        // never returns
#endif

    // ---- Sensor bring-up ----
    if (!icm20948_check_id()) printf("IMU not detected, check wiring\n");
    icm20948_init();

    if (!pmw3389_check_id()) printf("Optical sensor not detected, check wiring\n");
    pmw3389_init(NULL, 0);                // TODO: pass SROM image once added

    // ---- Pipeline ----
    detector_init();
    corrector_init();
    sampler_start();

    tremor_params_t params = {0};
    int samples_since_analysis = 0;
    uint32_t loop = 0;

    while (true) {
        sampler_wait();                                   // 1. Sense
        imu_sample_t imu = icm20948_read();
        optical_motion_t opt = pmw3389_read_motion();

        detector_push(opt.dx * COUNTS_TO_MM,              // 2. Combine
                      opt.dy * COUNTS_TO_MM,
                      imu.ax, imu.ay);

        bool bypass = !gpio_get(BTN_BYPASS_PIN);          // button pressed = low

        if (++samples_since_analysis >= ANALYSIS_EVERY_N) {
            samples_since_analysis = 0;
            detector_analyze(&params);                    // 3. Decide (slow path)
        }
        corrector_set_params(&params, bypass);

        hid_delta_t out = corrector_process(opt.dx, opt.dy);   // 4. Correct (fast path)

        // 5. Output: TODO send `out` as a USB HID mouse report.
        // For now, print a status line twice a second.
        if (++loop % (SAMPLE_RATE_HZ / 2) == 0) {
            printf("tremor=%d f=%.1fHz I=%.2fmm frac=%.2f rest=%d out=(%d,%d) overruns=%lu\n",
                   params.tremor, params.freq_hz, params.intensity_mm,
                   params.band_fraction, params.resting, out.dx, out.dy,
                   (unsigned long)sampler_overruns());
        }
        gpio_put(LED_PIN, params.tremor);                 // LED on while correcting
    }
}
