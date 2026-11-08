"""
Record real mouse movement for the Steadi filter tests.

Captures cursor movement from any ordinary mouse, converts it from pixels to
millimeters of mouse travel, and resamples it to an even 200 Hz so it plugs
straight into filter_compare.py.

Setup (once):
    pip install pynput numpy
    macOS: allow your terminal / VS Code under System Settings ->
    Privacy & Security -> Accessibility (and Input Monitoring).
    Turn off pointer acceleration in your mouse settings if your OS offers it,
    so cursor movement is proportional to physical mouse movement.

Usage:
    python mouse_logger.py calibrate --distance-mm 100
    python mouse_logger.py record --name drag_test --seconds 30

Output (in ./recordings):
    <name>_raw.csv    every movement event: time (s), x (px), y (px)
    <name>_200hz.csv  even 200 Hz samples: t_s, x_mm, y_mm, dx_mm, dy_mm
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

FS = 200.0              # output sample rate, matches the firmware
GAP_HOLD_S = 0.05       # gaps longer than this mean the mouse was still
CAL_FILE = "mouse_calibration.json"
OUT_DIR = "recordings"


# ------------------------------------------------------- pure functions
# (no mouse access, so they can be tested without a display)

def resample(times, xs, ys, fs=FS, gap_hold_s=GAP_HOLD_S):
    """Resample irregular movement events to an even rate.

    Between events that are close together, interpolate linearly. Across a
    long gap (no events because the mouse was still), hold the last position
    instead, so a pause isn't turned into a fake slow drift.
    """
    times = np.asarray(times, float)
    xs = np.asarray(xs, float)
    ys = np.asarray(ys, float)
    t_out = np.arange(times[0], times[-1], 1.0 / fs)
    i = np.searchsorted(times, t_out, side="right") - 1        # last event <= t
    i = np.clip(i, 0, len(times) - 2)
    t0, t1 = times[i], times[i + 1]
    span = t1 - t0
    frac = np.where(span > 0, (t_out - t0) / np.where(span > 0, span, 1), 0.0)
    frac = np.clip(frac, 0.0, 1.0)
    frac = np.where(span > gap_hold_s, 0.0, frac)               # hold during pauses
    x = xs[i] + frac * (xs[i + 1] - xs[i])
    y = ys[i] + frac * (ys[i + 1] - ys[i])
    return t_out - t_out[0], x, y


def to_mm(x_px, y_px, px_per_mm):
    x = (np.asarray(x_px) - x_px[0]) / px_per_mm
    y = (np.asarray(y_px) - y_px[0]) / px_per_mm
    return x, y


def write_200hz(path, t, x_mm, y_mm):
    dx = np.diff(x_mm, prepend=x_mm[0])
    dy = np.diff(y_mm, prepend=y_mm[0])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_mm", "y_mm", "dx_mm", "dy_mm"])
        for row in zip(t, x_mm, y_mm, dx, dy):
            w.writerow(["%.5f" % row[0]] + ["%.5f" % v for v in row[1:]])


def load_200hz(path):
    """Returns t, x_mm, y_mm arrays from a *_200hz.csv file."""
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data["t_s"], data["x_mm"], data["y_mm"]


# ------------------------------------------------------- mouse access

def _controller():
    try:
        from pynput import mouse
    except ImportError:
        sys.exit("pynput is not installed. Run: pip install pynput")
    return mouse


def calibrate(distance_mm, trials):
    mouse = _controller()
    ctl = mouse.Controller()
    print("Calibration: measures how many pixels the cursor moves per millimeter")
    print("of mouse travel. Lay a ruler on your desk and keep the mouse against it.\n")
    results = []
    for k in range(trials):
        input(f"Trial {k + 1}/{trials}: put the mouse at 0 mm, then press Enter")
        x0, y0 = ctl.position
        input(f"  Now slide it slowly and straight to {distance_mm:g} mm, then press Enter")
        x1, y1 = ctl.position
        px = math.hypot(x1 - x0, y1 - y0)
        if px < 10:
            print("  The cursor barely moved; skipping this trial.")
            continue
        results.append(px / distance_mm)
        print(f"  {px:.0f} px over {distance_mm:g} mm = {results[-1]:.2f} px/mm")
    if not results:
        sys.exit("No valid trials; calibration not saved.")
    px_per_mm = float(np.mean(results))
    spread = (max(results) - min(results)) / px_per_mm * 100
    with open(CAL_FILE, "w") as f:
        json.dump({"px_per_mm": px_per_mm, "trials": results}, f, indent=2)
    print(f"\nSaved {px_per_mm:.2f} px/mm to {CAL_FILE}")
    if spread > 10:
        print(f"Warning: trials differ by {spread:.0f}%. Pointer acceleration may be on,")
        print("or the slides weren't at a steady speed. Consider re-running.")


def record(name, seconds):
    if not os.path.exists(CAL_FILE):
        sys.exit(f"No {CAL_FILE} found. Run: python mouse_logger.py calibrate")
    px_per_mm = json.load(open(CAL_FILE))["px_per_mm"]
    mouse = _controller()

    events = []                                   # (time, x, y)
    def on_move(x, y):
        events.append((time.perf_counter(), x, y))

    print("Recording starts in 3 seconds. Move the mouse as you normally would.")
    print("Keep away from the screen edges. Press Ctrl+C to stop early.")
    for k in (3, 2, 1):
        print(f"  {k}...")
        time.sleep(1)
    start = time.perf_counter()
    events.append((start, *mouse.Controller().position))   # starting position
    print("Recording.")
    listener = mouse.Listener(on_move=on_move)
    listener.start()
    try:
        while time.perf_counter() - start < seconds:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    listener.stop()
    end = time.perf_counter()
    events.append((end, *mouse.Controller().position))     # ending position

    events.sort()
    t = np.array([e[0] for e in events]) - start
    x = np.array([e[1] for e in events], float)
    y = np.array([e[2] for e in events], float)
    moving = len(events) - 2
    print(f"Stopped after {end - start:.1f} s with {moving} movement events.")
    if moving < 50:
        sys.exit("Too few events to be useful; check the permissions listed at the top of this file.")

    os.makedirs(OUT_DIR, exist_ok=True)
    raw_path = os.path.join(OUT_DIR, f"{name}_raw.csv")
    with open(raw_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_px", "y_px"])
        w.writerows(zip(np.round(t, 5), x, y))

    ts, xs, ys = resample(t, x, y)
    xm, ym = to_mm(xs, ys, px_per_mm)
    out_path = os.path.join(OUT_DIR, f"{name}_200hz.csv")
    write_200hz(out_path, ts, xm, ym)

    rate = moving / max(t[-1] - t[0], 1e-9)
    print(f"Average event rate while recording: {rate:.0f} Hz")
    print(f"Total mouse travel: {np.sum(np.hypot(np.diff(xm), np.diff(ym))):.0f} mm")
    print(f"Saved {raw_path}\nSaved {out_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("calibrate", help="measure pixels per millimeter of mouse travel")
    c.add_argument("--distance-mm", type=float, default=100.0)
    c.add_argument("--trials", type=int, default=3)
    r = sub.add_parser("record", help="record mouse movement")
    r.add_argument("--name", required=True, help="file name for this recording")
    r.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()
    if args.cmd == "calibrate":
        calibrate(args.distance_mm, args.trials)
    else:
        record(args.name, args.seconds)
