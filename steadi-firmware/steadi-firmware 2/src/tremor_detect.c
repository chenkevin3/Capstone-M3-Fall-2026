#include "tremor_detect.h"
#include "config.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Four input channels share one circular buffer layout
enum { CH_DX, CH_DY, CH_AX, CH_AY, NUM_CH };

static float ring[NUM_CH][WINDOW_SIZE];
static int head = 0;    // next write position
static int count = 0;   // samples stored, up to WINDOW_SIZE

static float hann[WINDOW_SIZE];

void detector_init(void) {
    memset(ring, 0, sizeof(ring));
    head = 0;
    count = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        hann[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (WINDOW_SIZE - 1));
    }
}

void detector_push(float dx_mm, float dy_mm, float ax_g, float ay_g) {
    ring[CH_DX][head] = dx_mm;
    ring[CH_DY][head] = dy_mm;
    ring[CH_AX][head] = ax_g;
    ring[CH_AY][head] = ay_g;
    head = (head + 1) % WINDOW_SIZE;
    if (count < WINDOW_SIZE) count++;
}

// Copy one channel out of the ring buffer, oldest sample first
static void unroll(int ch, float *dst) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        dst[i] = ring[ch][(head + i) % WINDOW_SIZE];
    }
}

// Power at DFT bin k via the Goertzel algorithm
static float goertzel_bin(const float *x, int k) {
    float coeff = 2.0f * cosf(2.0f * (float)M_PI * k / WINDOW_SIZE);
    float s1 = 0.0f, s2 = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float s0 = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

// Remove the mean (constant-velocity motion or gravity), apply a Hann
// window, then fill power[1..N/2]
static void spectrum(const float *x, float *power) {
    float tmp[WINDOW_SIZE];
    float mean = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) mean += x[i];
    mean /= WINDOW_SIZE;
    for (int i = 0; i < WINDOW_SIZE; i++) tmp[i] = (x[i] - mean) * hann[i];
    for (int k = 1; k <= WINDOW_SIZE / 2; k++) power[k] = goertzel_bin(tmp, k);
}

// RMS of displacement after removing the straight-line trend, so a steady
// drag does not count as tremor. Input is per-sample deltas in mm.
static float detrended_rms(const float *delta) {
    float pos[WINDOW_SIZE];
    float p = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) { p += delta[i]; pos[i] = p; }

    // Least-squares line fit: pos ~ a + b*i
    float n = WINDOW_SIZE, si = 0, sp = 0, sii = 0, sip = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        si += i; sp += pos[i]; sii += (float)i * i; sip += i * pos[i];
    }
    float b = (n * sip - si * sp) / (n * sii - si * si);
    float a = (sp - b * si) / n;

    float acc = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float r = pos[i] - (a + b * i);
        acc += r * r;
    }
    return sqrtf(acc / n);
}

void detector_analyze(tremor_params_t *out) {
    static uint32_t seq = 0;
    memset(out, 0, sizeof(*out));
    if (count < WINDOW_SIZE) return;     // not enough data yet (seq stays 0)
    out->seq = ++seq;

    static float x[NUM_CH][WINDOW_SIZE];
    static float pw[NUM_CH][WINDOW_SIZE / 2 + 1];
    for (int ch = 0; ch < NUM_CH; ch++) {
        unroll(ch, x[ch]);
        spectrum(x[ch], pw[ch]);
    }

    int k_lo = (int)ceilf(TREMOR_BAND_LO_HZ * WINDOW_SIZE / SAMPLE_RATE_HZ);
    int k_hi = (int)floorf(TREMOR_BAND_HI_HZ * WINDOW_SIZE / SAMPLE_RATE_HZ);

    float opt_band = 0, opt_total = 0, imu_band = 0, imu_total = 0;
    float best_power = 0;
    int best_k = k_lo;
    for (int k = 1; k <= WINDOW_SIZE / 2; k++) {
        float opt = pw[CH_DX][k] + pw[CH_DY][k];
        float imu = pw[CH_AX][k] + pw[CH_AY][k];
        opt_total += opt;
        imu_total += imu;
        if (k >= k_lo && k <= k_hi) {
            opt_band += opt;
            imu_band += imu;
            if (opt > best_power) { best_power = opt; best_k = k; }
        }
    }

    // Frequency decision: fuse optical and IMU band fractions
    float opt_frac = opt_total > 0 ? opt_band / opt_total : 0;
    float imu_frac = imu_total > 0 ? imu_band / imu_total : 0;
    out->band_fraction = W_OPTICAL * opt_frac + W_IMU * imu_frac;

    // Refine the peak between bins with a parabola through the neighbors,
    // so the notch lands on the real tremor frequency, not the nearest bin
    float k_est = (float)best_k;
    if (best_k > 1 && best_k < WINDOW_SIZE / 2) {
        float a = sqrtf(pw[CH_DX][best_k - 1] + pw[CH_DY][best_k - 1]);
        float b = sqrtf(pw[CH_DX][best_k]     + pw[CH_DY][best_k]);
        float c = sqrtf(pw[CH_DX][best_k + 1] + pw[CH_DY][best_k + 1]);
        float denom = a - 2.0f * b + c;
        if (denom < 0.0f) {
            float d = 0.5f * (a - c) / denom;
            if (d > 0.5f) d = 0.5f;
            if (d < -0.5f) d = -0.5f;
            k_est += d;
        }
    }
    out->freq_hz = k_est * SAMPLE_RATE_HZ / WINDOW_SIZE;

    // Intensity decision: detrended RMS displacement across both axes
    float rx = detrended_rms(x[CH_DX]);
    float ry = detrended_rms(x[CH_DY]);
    out->intensity_mm = sqrtf(rx * rx + ry * ry);

    // Motion state: net speed over the window (tremor oscillation cancels out)
    float net_x = 0, net_y = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) { net_x += x[CH_DX][i]; net_y += x[CH_DY][i]; }
    float window_s = (float)WINDOW_SIZE / SAMPLE_RATE_HZ;
    float speed = sqrtf(net_x * net_x + net_y * net_y) / window_s;
    out->resting = speed < RESTING_SPEED_MM_S;

    out->tremor = out->band_fraction > BAND_FRACTION_THRESH &&
                  out->intensity_mm >= INTENSITY_MIN_MM;
}
