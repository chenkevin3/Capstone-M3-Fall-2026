"""
Notch (current firmware) vs. WFLC on simulated tremor.

Everything runs on per-sample cursor deltas at 200 Hz, the same signal the
firmware sees. The notch path mirrors the firmware: a detector re-aims it
every 125 ms using the last 640 ms of data. The WFLC path is the standard
two-stage design: a WFLC tracks tremor frequency on a band-passed copy of
the signal, and an FLC at that frequency cancels tremor on the raw signal.
"""
import numpy as np
from scipy import signal

FS = 200.0
WINDOW = 128
ANALYZE_EVERY = 25
BAND = (3.0, 12.0)
CPI = 1600
SEED_MIN_FRACTION = 0.5   # matches BAND_FRACTION_THRESH in config.h
MM_PER_COUNT = 25.4 / CPI


# ---------------------------------------------------------------- signals
def make_motion(duration, tremor_hz_fn, tremor_amp_fn, seed=0, noise_mm=0.01):
    """Returns time, voluntary position, tremor position, measured deltas (mm)."""
    rng = np.random.default_rng(seed)
    t = np.arange(int(duration * FS)) / FS

    # Intentional movement: hold, drag right, hold, slow drag back
    vel = np.zeros_like(t)
    vel[(t >= 2) & (t < 4)] = 25.0
    vel[(t >= 6) & (t < 9)] = -12.0
    vel = np.convolve(vel, np.ones(40) / 40, mode="same")   # smooth starts/stops
    voluntary = np.cumsum(vel) / FS

    return _add_tremor(t, voluntary, tremor_hz_fn, tremor_amp_fn, rng, noise_mm)


def make_motion_from_recording(path, tremor_hz_fn, tremor_amp_fn, seed=0,
                               axis="x", noise_mm=0.01):
    """Like make_motion, but the intentional movement is a real mouse
    recording from mouse_logger.py (one axis) instead of a simulated path."""
    from mouse_logger import load_200hz
    rng = np.random.default_rng(seed)
    t, x_mm, y_mm = load_200hz(path)
    voluntary = (x_mm if axis == "x" else y_mm).astype(float)
    voluntary = voluntary - voluntary[0]
    return _add_tremor(t, voluntary, tremor_hz_fn, tremor_amp_fn, rng, noise_mm)


def _add_tremor(t, voluntary, tremor_hz_fn, tremor_amp_fn, rng, noise_mm):
    # Tremor with time-varying frequency: integrate frequency to get phase
    f = tremor_hz_fn(t, rng)
    phase = 2 * np.pi * np.cumsum(f) / FS
    amp = tremor_amp_fn(t, rng)
    tremor = amp * np.sin(phase) + 0.15 * amp * np.sin(2 * phase + 0.7)  # + harmonic

    pos = voluntary + tremor + rng.normal(0, noise_mm, t.size)
    counts = np.round(pos / MM_PER_COUNT)                 # sensor reports whole counts
    deltas = np.diff(counts, prepend=counts[0]) * MM_PER_COUNT
    return t, voluntary, tremor, deltas, f


def steady_freq(t, rng):
    return np.full_like(t, 5.0)


def drifting_freq(t, rng):
    walk = np.cumsum(rng.normal(0, 0.02, t.size))
    walk -= np.convolve(walk, np.ones(400) / 400, mode="same")  # keep it bounded
    return 5.0 + 1.0 * np.sin(2 * np.pi * 0.12 * t) + walk


def amp_modulated(t, rng):
    return 1.5 * (1 + 0.3 * np.sin(2 * np.pi * 0.2 * t + 1.0))


# ---------------------------------------------------------- notch (firmware)
def notch_coeffs(f0, q=1.0):
    w0 = 2 * np.pi * f0 / FS
    alpha = np.sin(w0) / (2 * q)
    c = np.cos(w0)
    a0 = 1 + alpha
    return np.array([1, -2 * c, 1]) / a0, np.array([1, -2 * c / a0, (1 - alpha) / a0])


def detect_freq(window, return_fraction=False):
    x = (window - window.mean()) * np.hanning(WINDOW)
    p = np.abs(np.fft.rfft(x)) ** 2
    k_lo = int(np.ceil(BAND[0] * WINDOW / FS))
    k_hi = int(np.floor(BAND[1] * WINDOW / FS))
    k = k_lo + int(np.argmax(p[k_lo:k_hi + 1]))
    frac = p[k_lo:k_hi + 1].sum() / (p[1:].sum() + 1e-12)   # share of energy in band
    a, b, c = np.sqrt(p[k - 1:k + 2])
    d = 0.0
    if a - 2 * b + c < 0:
        d = float(np.clip(0.5 * (a - c) / (a - 2 * b + c), -0.5, 0.5))
    f = (k + d) * FS / WINDOW
    return (f, frac) if return_fraction else f


def run_notch(deltas, q=1.0):
    out = np.zeros_like(deltas)
    f_track = np.full_like(deltas, np.nan)
    f0 = 6.0
    b, a = notch_coeffs(f0, q)
    x1 = x2 = y1 = y2 = 0.0
    for n, x in enumerate(deltas):
        if n >= WINDOW and n % ANALYZE_EVERY == 0:
            f_new = detect_freq(deltas[n - WINDOW:n])
            if abs(f_new - f0) > 0.5:
                f0 = f_new
                b, a = notch_coeffs(f0, q)
        y = b[0] * x + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
        x2, x1, y2, y1 = x1, x, y1, y
        out[n] = y
        f_track[n] = f0
    return out, f_track


# ------------------------------------------------------------------ WFLC
def run_wflc(deltas, mu_w=0.02, mu_f=0.001, mu_flc=0.08, harmonics=2, seeded=False):
    """Stage 1: WFLC on band-passed signal estimates frequency.
       Stage 2: FLC at that frequency subtracts tremor from the raw signal."""
    sos = signal.butter(2, BAND, btype="bandpass", fs=FS, output="sos")
    zi = np.zeros((sos.shape[0], 2))
    r = np.arange(1, harmonics + 1)

    w0 = 2 * np.pi * 6.0 / FS          # frequency estimate, rad/sample
    wf = np.zeros(2 * harmonics)        # WFLC weights
    phase_f = 0.0
    wc = np.zeros(2 * harmonics + 1)    # FLC weights (+ bias for steady drag)
    phase_c = 0.0

    out = np.zeros_like(deltas)
    f_track = np.zeros_like(deltas)
    # Running RMS of the input, so learning rates don't depend on the
    # signal's units or size. Plain average for the first second, then a
    # slow exponential average (time constant 5 s). Firmware does the same.
    power = 0.0

    for n, x in enumerate(deltas):
        if n < 200:
            power += (x * x - power) / (n + 1)
        else:
            power += (x * x - power) / 1000.0
        scale = np.sqrt(power) + 1e-6
        # ---- stage 1: frequency tracking
        xb, zi = signal.sosfilt(sos, [x / scale], zi=zi)
        xb = xb[0]
        # Optional: the firmware's spectral detector re-seeds the frequency
        # estimate if WFLC has drifted more than 1 Hz from it
        # Only trust the detector when the window actually looks like tremor
        # (most energy in the 3-12 Hz band); otherwise it is chasing noise.
        if seeded and n >= WINDOW and n % ANALYZE_EVERY == 0:
            f_det, frac = detect_freq(deltas[n - WINDOW:n], return_fraction=True)
            if frac > SEED_MIN_FRACTION and abs(f_det - w0 * FS / (2 * np.pi)) > 1.0:
                w0 = 2 * np.pi * f_det / FS
        phase_f += w0
        ref = np.concatenate([np.sin(r * phase_f), np.cos(r * phase_f)])
        err = xb - wf @ ref
        grad = np.sum(r * (wf[:harmonics] * ref[harmonics:] - wf[harmonics:] * ref[:harmonics]))
        w0 += 2 * mu_f * err * grad
        w0 = np.clip(w0, 2 * np.pi * BAND[0] / FS, 2 * np.pi * BAND[1] / FS)
        wf += 2 * mu_w * err * ref

        # ---- stage 2: cancellation on the raw signal
        phase_c += w0
        refc = np.concatenate([np.sin(r * phase_c), np.cos(r * phase_c), [1.0]])
        est = wc @ refc
        e = x / scale - est
        wc += 2 * mu_flc * e * refc
        tremor_est = (wc[:-1] @ refc[:-1]) * scale     # tremor part only, not bias
        out[n] = x - tremor_est
        f_track[n] = w0 * FS / (2 * np.pi)
    return out, f_track


# --------------------------------------------------------------- metrics
def evaluate(t, voluntary, tremor, out_deltas, start_s=2.0):
    pos = np.cumsum(out_deltas)
    err = pos - voluntary
    m = t >= start_s
    err = err[m] - np.mean(err[m])             # ignore one-time offset
    raw = tremor[m] - np.mean(tremor[m])
    return {
        "raw_rms": float(np.sqrt(np.mean(raw ** 2))),
        "out_rms": float(np.sqrt(np.mean(err ** 2))),
        "removed": float(100 * (1 - np.sqrt(np.mean(err ** 2)) / np.sqrt(np.mean(raw ** 2)))),
    }


def voluntary_delay_ms(method, q=1.0):
    """Delay added to intentional movement alone (no tremor)."""
    t = np.arange(int(4 * FS)) / FS
    vel = np.where((t >= 1) & (t < 3), 25.0, 0.0)
    vel = np.convolve(vel, np.ones(40) / 40, mode="same")
    d = vel / FS
    if method == "notch":
        b, a = notch_coeffs(5.0, q)
        y = signal.lfilter(b, a, d)
    else:
        y, _ = run_wflc(d + 1e-9 * np.random.default_rng(1).normal(size=d.size))
    pos_in, pos_out = np.cumsum(d), np.cumsum(y)
    lags = np.arange(0, 40)
    errs = [np.sum((pos_out[l:] - pos_in[:pos_in.size - l]) ** 2) for l in lags]
    return float(lags[int(np.argmin(errs))] / FS * 1000)


def strong_drift(t, rng):
    return 5.0 + 1.5 * np.sin(2 * np.pi * 0.25 * t) + 0.3 * np.sin(2 * np.pi * 0.9 * t)


def et_9hz(t, rng):
    return 9.0 + 0.8 * np.sin(2 * np.pi * 0.15 * t)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Compare notch and WFLC tremor filters.")
    ap.add_argument("--recording", help="a *_200hz.csv from mouse_logger.py to use as the "
                    "intentional movement (default: simulated movement)")
    ap.add_argument("--axis", choices=["x", "y"], default="x", help="which recorded axis to use")
    args = ap.parse_args()

    if args.recording:
        print(f"Intentional movement: {args.recording} ({args.axis} axis), simulated tremor added")
        def motion(f, seed):
            return make_motion_from_recording(args.recording, f, amp_modulated, seed=seed, axis=args.axis)
    else:
        print("Intentional movement: simulated")
        def motion(f, seed):
            return make_motion(12, f, amp_modulated, seed=seed)

    # Held-out comparison: settings were tuned on seeds 0-1, scored on 10-14
    scenarios = [("Steady 5 Hz", steady_freq), ("Drifting 4-6 Hz", drifting_freq),
                 ("Strong drift 3.5-6.5 Hz", strong_drift), ("ET-like 9 Hz", et_9hz)]
    for name, f in scenarios:
        n, w, ws = [], [], []
        for seed in range(10, 15):
            t, vol, trem, d, _ = motion(f, seed)
            n.append(evaluate(t, vol, trem, run_notch(d, q=2.0)[0])["removed"])
            w.append(evaluate(t, vol, trem, run_wflc(d)[0])["removed"])
            ws.append(evaluate(t, vol, trem, run_wflc(d, seeded=True)[0])["removed"])
        print(f"{name:24s} notch(Q=2) {np.mean(n):3.0f}%   WFLC {np.mean(w):3.0f}%   "
              f"seeded WFLC {np.mean(ws):3.0f}%")
    print("delay on intentional movement: notch Q=1 %.0f ms, Q=2 %.0f ms, WFLC %.0f ms"
          % (voluntary_delay_ms("notch", 1.0), voluntary_delay_ms("notch", 2.0),
             voluntary_delay_ms("wflc")))
