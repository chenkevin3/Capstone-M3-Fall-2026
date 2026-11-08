#include "wflc.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI (2.0f * (float)M_PI)

#if SAMPLE_RATE_HZ != 200
#error "Band-pass coefficients below assume 200 Hz; regenerate them for a new rate"
#endif

// 4th-order Butterworth band-pass, 3-12 Hz at 200 Hz, as two biquads:
// {b0, b1, b2, a1, a2}. Generated with scipy:
//   signal.butter(2, [3, 12], btype="bandpass", fs=200, output="sos")
static const float BP[2][5] = {
    { 1.6581931669e-02f, 3.3163863339e-02f, 1.6581931669e-02f,
     -1.6442211333e+00f, 7.3956049369e-01f },
    { 1.0000000000e+00f, -2.0000000000e+00f, 1.0000000000e+00f,
     -1.8956415238e+00f, 9.0656262967e-01f },
};

static float hz_to_w(float hz) { return TWO_PI * hz / SAMPLE_RATE_HZ; }

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Keep phase in [0, 2*pi) so float precision doesn't degrade over time
static float wrap(float p) {
    while (p >= TWO_PI) p -= TWO_PI;
    return p;
}

// Transposed direct form II, the same structure scipy's sosfilt uses
static float bandpass(wflc_t *w, float x) {
    for (int s = 0; s < 2; s++) {
        const float *c = BP[s];
        float *z = w->bp[s];
        float y = c[0] * x + z[0];
        z[0] = c[1] * x - c[3] * y + z[1];
        z[1] = c[2] * x - c[4] * y;
        x = y;
    }
    return x;
}

void wflc_init(wflc_t *w, float f_init_hz) {
    memset(w, 0, sizeof(*w));
    w->w0 = hz_to_w(f_init_hz);
}

void wflc_reseed(wflc_t *w, float f_hz) {
    w->w0 = clampf(hz_to_w(f_hz), hz_to_w(TREMOR_BAND_LO_HZ), hz_to_w(TREMOR_BAND_HI_HZ));
}

float wflc_freq_hz(const wflc_t *w) {
    return w->w0 * SAMPLE_RATE_HZ / TWO_PI;
}

float wflc_process(wflc_t *w, float x) {
    const int H = WFLC_HARMONICS;

    // Running RMS so learning rates are independent of signal size:
    // plain average for the first second, then a slow average (~5 s)
    if (w->n < SAMPLE_RATE_HZ) {
        w->power += (x * x - w->power) / (float)(w->n + 1);
        w->n++;
    } else {
        w->power += (x * x - w->power) / 1000.0f;
    }
    float scale = sqrtf(w->power) + 1e-6f;
    float xs = x / scale;

    // ---- Stage 1: WFLC frequency tracking on the band-passed signal ----
    float xb = bandpass(w, xs);
    w->phase_f = wrap(w->phase_f + w->w0);

    float s_ref[WFLC_HARMONICS], c_ref[WFLC_HARMONICS];
    float est = 0.0f;
    for (int r = 0; r < H; r++) {
        s_ref[r] = sinf((r + 1) * w->phase_f);
        c_ref[r] = cosf((r + 1) * w->phase_f);
        est += w->wf[r] * s_ref[r] + w->wf[H + r] * c_ref[r];
    }
    float err = xb - est;

    // Frequency update uses the weights from before this step
    float grad = 0.0f;
    for (int r = 0; r < H; r++) {
        grad += (r + 1) * (w->wf[r] * c_ref[r] - w->wf[H + r] * s_ref[r]);
    }
    w->w0 += 2.0f * WFLC_MU_F * err * grad;
    w->w0 = clampf(w->w0, hz_to_w(TREMOR_BAND_LO_HZ), hz_to_w(TREMOR_BAND_HI_HZ));

    for (int r = 0; r < H; r++) {
        w->wf[r]     += 2.0f * WFLC_MU_W * err * s_ref[r];
        w->wf[H + r] += 2.0f * WFLC_MU_W * err * c_ref[r];
    }

    // ---- Stage 2: FLC cancellation on the raw signal ----
    w->phase_c = wrap(w->phase_c + w->w0);
    float sc[WFLC_HARMONICS], cc[WFLC_HARMONICS];
    float est_c = w->wc[2 * H];                 // bias absorbs steady drags
    for (int r = 0; r < H; r++) {
        sc[r] = sinf((r + 1) * w->phase_c);
        cc[r] = cosf((r + 1) * w->phase_c);
        est_c += w->wc[r] * sc[r] + w->wc[H + r] * cc[r];
    }
    float e = xs - est_c;
    for (int r = 0; r < H; r++) {
        w->wc[r]     += 2.0f * WFLC_MU_FLC * e * sc[r];
        w->wc[H + r] += 2.0f * WFLC_MU_FLC * e * cc[r];
    }
    w->wc[2 * H] += 2.0f * WFLC_MU_FLC * e;

    // Tremor estimate uses only the sine terms, never the bias, so
    // intentional movement is not subtracted
    float tremor = 0.0f;
    for (int r = 0; r < H; r++) {
        tremor += w->wc[r] * sc[r] + w->wc[H + r] * cc[r];
    }
    return tremor * scale;
}
