#include "correction.h"
#include "config.h"
#include "wflc.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Shared state
static float depth = 0.0f;          // current correction depth (0..1)
static float depth_target = 0.0f;
static float acc_x = 0.0f, acc_y = 0.0f;   // carries sub-count remainders

// Intensity -> how hard to correct (0 = none, 1 = full)
static float depth_from_intensity(float mm) {
    if (mm < INTENSITY_MIN_MM) return 0.0f;
    float t = (mm - INTENSITY_MIN_MM) / (INTENSITY_SEVERE_MM - INTENSITY_MIN_MM);
    if (t > 1.0f) t = 1.0f;
    return DEPTH_MIN + (1.0f - DEPTH_MIN) * t;
}

#if CORRECTION_METHOD == CORRECTION_NOTCH
// ===================================================== notch filter ===

// Biquad in direct form I, which handles coefficient updates mid-stream
// with fewer glitches than the transposed forms
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_t;

static biquad_t notch_x, notch_y;
static float notch_freq = 0.0f;

// Standard notch (band-stop) design at f0, from the Audio EQ Cookbook
static void notch_design(biquad_t *f, float f0) {
    float w0 = 2.0f * (float)M_PI * f0 / SAMPLE_RATE_HZ;
    float alpha = sinf(w0) / (2.0f * NOTCH_Q);
    float c = cosf(w0);
    float a0 = 1.0f + alpha;
    f->b0 = 1.0f / a0;
    f->b1 = -2.0f * c / a0;
    f->b2 = 1.0f / a0;
    f->a1 = -2.0f * c / a0;
    f->a2 = (1.0f - alpha) / a0;
    // filter state (x1..y2) is kept, so the output stays continuous
}

static float biquad_run(biquad_t *f, float x) {
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
            - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

static void method_init(void) {
    biquad_t zero = {0};
    notch_x = zero;
    notch_y = zero;
    notch_freq = 6.0f;                  // placeholder until the first analysis
    notch_design(&notch_x, notch_freq);
    notch_design(&notch_y, notch_freq);
}

static void method_new_analysis(const tremor_params_t *p) {
    if (p->tremor && fabsf(p->freq_hz - notch_freq) > 0.5f) {
        notch_freq = p->freq_hz;
        notch_design(&notch_x, notch_freq);
        notch_design(&notch_y, notch_freq);
    }
}

// Returns the estimated tremor component of each axis
static void method_tremor(float dx, float dy, float *tx, float *ty) {
    *tx = dx - biquad_run(&notch_x, dx);   // what the notch removes
    *ty = dy - biquad_run(&notch_y, dy);
}

#else
// ============================================ detector-seeded WFLC ===

static wflc_t wflc_x, wflc_y;

static void method_init(void) {
    wflc_init(&wflc_x, WFLC_INIT_HZ);
    wflc_init(&wflc_y, WFLC_INIT_HZ);
}

// The detector re-seeds WFLC whenever WFLC has lost the tremor frequency,
// but only when the detector sees tremor-band energy. Before tremor starts,
// the detector's "frequency" is just the peak of sensor noise, and following
// it would leave WFLC in a random spot when tremor begins.
static void method_new_analysis(const tremor_params_t *p) {
    if (p->band_fraction <= WFLC_SEED_MIN_FRACTION) return;
    if (p->freq_hz < TREMOR_BAND_LO_HZ || p->freq_hz > TREMOR_BAND_HI_HZ) return;
    if (fabsf(wflc_freq_hz(&wflc_x) - p->freq_hz) > WFLC_RESEED_HZ) wflc_reseed(&wflc_x, p->freq_hz);
    if (fabsf(wflc_freq_hz(&wflc_y) - p->freq_hz) > WFLC_RESEED_HZ) wflc_reseed(&wflc_y, p->freq_hz);
}

static void method_tremor(float dx, float dy, float *tx, float *ty) {
    *tx = wflc_process(&wflc_x, dx);
    *ty = wflc_process(&wflc_y, dy);
}

#endif
// ================================================== shared interface ===

static uint32_t last_seq = 0;

void corrector_init(void) {
    method_init();
    depth = depth_target = 0.0f;
    acc_x = acc_y = 0.0f;
    last_seq = 0;
}

void corrector_set_params(const tremor_params_t *p, bool bypass) {
    // Act on each detector result once, not on every sample it is reused
    if (p->seq != 0 && p->seq != last_seq) {
        last_seq = p->seq;
        method_new_analysis(p);
    }
    // Frequency decides WHERE to filter (above); intensity decides HOW MUCH
    depth_target = (bypass || !p->tremor) ? 0.0f : depth_from_intensity(p->intensity_mm);
}

hid_delta_t corrector_process(float dx_counts, float dy_counts) {
    // Ease depth toward its target so the cursor never jumps
    depth += DEPTH_RAMP * (depth_target - depth);

    // Always run the filter so its state stays warm
    float tx, ty;
    method_tremor(dx_counts, dy_counts, &tx, &ty);

    // Subtract the tremor estimate, scaled by depth
    acc_x += dx_counts - depth * tx;
    acc_y += dy_counts - depth * ty;

    // Mouse reports are whole counts; keep the remainder for next time
    hid_delta_t out;
    out.dx = (int16_t)roundf(acc_x);
    out.dy = (int16_t)roundf(acc_y);
    acc_x -= out.dx;
    acc_y -= out.dy;
    return out;
}
