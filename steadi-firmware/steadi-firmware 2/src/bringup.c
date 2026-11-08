#include "bringup.h"
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pins.h"
#include "config.h"
#include "icm20948.h"
#include "pmw3389.h"
#include "sampler.h"
#include "tremor_detect.h"
#include "correction.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Every line that isn't CSV data starts with '#', so the capture script
// can tell status messages from data.

static bool imu_ok = false;
static bool opt_ok = false;

static void stage_checks(void) {
    printf("#\n# === Stage 1: sensor checks ===\n");

    imu_ok = icm20948_check_id();
    printf("# IMU (ICM-20948) responds: %s\n", imu_ok ? "PASS" : "FAIL - check wiring and pins.h");
    if (imu_ok) {
        icm20948_init();
        sleep_ms(50);
        imu_sample_t s = icm20948_read();
        float g = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
        bool sane = g > 0.9f && g < 1.1f;
        printf("# IMU at rest: |accel| = %.2f g (expect about 1.00): %s\n",
               g, sane ? "PASS" : "CHECK - keep the board still, or check the range setting");
    }

    opt_ok = pmw3389_check_id();
    printf("# Optical sensor (PMW3389) responds: %s\n",
           opt_ok ? "PASS" : "FAIL - check wiring, pins.h, and SPI mode 3");
    if (opt_ok) {
        bool tracking = pmw3389_init(NULL, 0);   // TODO: pass SROM image once added
        if (!tracking) {
            printf("# Optical tracking: NOT AVAILABLE until the SROM image is added; dx/dy will read 0 or noise\n");
        }
    }
}

void bringup_benchmark(void) {
    printf("#\n# === Stage 2: on-chip timing (simulated tremor, no sensors needed) ===\n");
#if CORRECTION_METHOD == CORRECTION_WFLC
    printf("# Correction method: detector-seeded WFLC\n");
#else
    printf("# Correction method: notch filter\n");
#endif

    detector_init();
    corrector_init();
    tremor_params_t params = {0};

    const int n_samples = SAMPLE_RATE_HZ * 10;      // 10 s of simulated use
    const float fs = (float)SAMPLE_RATE_HZ;
    float phase = 0.0f, prev_mm = 0.0f;
    float pos_in = 0.0f, pos_out = 0.0f;
    double in_sum = 0, in_sq = 0, out_sum = 0, out_sq = 0;
    int n_scored = 0, since = 0;

    uint64_t fast_total = 0, slow_total = 0;
    uint32_t fast_max = 0, slow_max = 0, slow_count = 0;

    for (int n = 0; n < n_samples; n++) {
        // Tremor drifting between 4 and 6 Hz, 1.5 mm amplitude, hand otherwise still
        float t = n / fs;
        float f = 5.0f + sinf(2.0f * (float)M_PI * 0.12f * t);
        phase += 2.0f * (float)M_PI * f / fs;
        float trem_mm = 1.5f * sinf(phase);
        float dx_mm = trem_mm - prev_mm;
        prev_mm = trem_mm;
        float w = 2.0f * (float)M_PI * f;
        float ax_g = -w * w * trem_mm / 1000.0f / 9.81f;

        // Fast path: runs every sample
        uint64_t t0 = time_us_64();
        detector_push(dx_mm, 0.0f, ax_g, 0.0f);
        corrector_set_params(&params, false);
        hid_delta_t out = corrector_process(dx_mm / COUNTS_TO_MM, 0.0f);
        uint32_t fast = (uint32_t)(time_us_64() - t0);
        fast_total += fast;
        if (fast > fast_max) fast_max = fast;

        // Slow path: runs every ANALYSIS_EVERY_N samples
        if (++since >= ANALYSIS_EVERY_N) {
            since = 0;
            uint64_t t1 = time_us_64();
            detector_analyze(&params);
            uint32_t slow = (uint32_t)(time_us_64() - t1);
            slow_total += slow;
            slow_count++;
            if (slow > slow_max) slow_max = slow;
        }

        // Score the last 5 s: tremor size in vs. out (hand is still, so all motion is tremor)
        pos_in += dx_mm;
        pos_out += out.dx * COUNTS_TO_MM;
        if (n >= n_samples / 2) {
            in_sum += pos_in;   in_sq += (double)pos_in * pos_in;
            out_sum += pos_out; out_sq += (double)pos_out * pos_out;
            n_scored++;
        }
    }

    double in_rms = sqrt(in_sq / n_scored - (in_sum / n_scored) * (in_sum / n_scored));
    double out_rms = sqrt(out_sq / n_scored - (out_sum / n_scored) * (out_sum / n_scored));
    float fast_avg = (float)fast_total / n_samples;
    float slow_avg = slow_count ? (float)slow_total / slow_count : 0.0f;
    // Worst case for one sample period: fast path plus a slow-path analysis
    float budget_used = 100.0f * (fast_max + slow_max) / SAMPLE_PERIOD_US;

    printf("# Fast path (every sample):  average %.1f us, worst %lu us\n",
           fast_avg, (unsigned long)fast_max);
    printf("# Slow path (every %d samples): average %.1f us, worst %lu us\n",
           ANALYSIS_EVERY_N, slow_avg, (unsigned long)slow_max);
    printf("# Worst-case sample: %.1f%% of the %d us sample period: %s\n",
           budget_used, SAMPLE_PERIOD_US,
           budget_used < 50.0f ? "PASS" : "CHECK - little room left for sensor reads");
    printf("# Detector result: tremor=%d, frequency %.2f Hz, intensity %.2f mm\n",
           params.tremor, params.freq_hz, params.intensity_mm);
    printf("# Tremor size: %.2f mm RMS in, %.2f mm RMS out (%.0f%% removed, with intensity-based depth)\n",
           in_rms, out_rms, 100.0 * (1.0 - out_rms / in_rms));
}

static void stage_stream(void) {
    printf("#\n# === Stage 3: raw data streaming ===\n");
    printf("# Press any key (or start tools/capture_serial.py) to begin streaming.\n");
    while (getchar_timeout_us(250000) == PICO_ERROR_TIMEOUT) {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));        // blink while waiting
    }
    gpio_put(LED_PIN, 1);

    printf("t_us,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,dx_counts,dy_counts\n");
    sampler_start();
    uint32_t n = 0;
    while (true) {
        uint64_t t = sampler_wait();
        imu_sample_t imu = {0};
        optical_motion_t opt = {0};
        if (imu_ok) imu = icm20948_read();
        if (opt_ok) opt = pmw3389_read_motion();
        printf("%llu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%d\n",
               (unsigned long long)t, imu.ax, imu.ay, imu.az,
               imu.gx, imu.gy, imu.gz, opt.dx, opt.dy);
        if (++n % (SAMPLE_RATE_HZ * 5) == 0) {
            printf("# missed sample ticks so far: %lu\n", (unsigned long)sampler_overruns());
        }
    }
}

void bringup_run(void) {
    printf("# Steadi bring-up test\n");
    stage_checks();
    bringup_benchmark();
    stage_stream();               // runs until reset
}
