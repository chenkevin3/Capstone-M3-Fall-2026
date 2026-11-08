"""
Capture the raw data stream from the board in bring-up mode (Stage 3).

Usage:
    pip install pyserial numpy
    python capture_serial.py --name desk_test --seconds 30
    (add --port /dev/cu.usbmodemXXXX if the board isn't found automatically)

Status lines from the board (starting with '#') are printed to the screen.
Data is saved to ./recordings:
    <name>_device.csv  every sensor reading, as sent by the board
    <name>_200hz.csv   optical movement in mm, same format as mouse_logger.py,
                       so it works with: python filter_compare.py --recording ...
"""
import argparse
import csv
import glob
import os
import sys
import time

import numpy as np

COUNTS_PER_INCH = 1600   # must match OPTICAL_CPI in config.h


def find_port():
    ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*")
    if not ports:
        sys.exit("No board found. Plug it in, or pass --port (on Windows, e.g. COM3).")
    return ports[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--port")
    args = ap.parse_args()
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is not installed. Run: pip install pyserial")

    port = args.port or find_port()
    print(f"Connecting to {port} ...")
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.5)
    ser.write(b"s")                         # any key starts Stage 3 streaming

    header, rows = None, []
    start = None
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        if line.startswith("#"):
            print(line)
            continue
        if line.startswith("t_us"):
            header = line.split(",")
            start = time.time()
            print(f"Streaming; recording for {args.seconds:g} s ...")
            continue
        if header is None:
            continue
        parts = line.split(",")
        if len(parts) == len(header):
            rows.append(parts)
        if time.time() - start >= args.seconds:
            break
    ser.close()

    os.makedirs("recordings", exist_ok=True)
    dev_path = os.path.join("recordings", f"{args.name}_device.csv")
    with open(dev_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    data = np.array(rows, dtype=float)
    t = (data[:, 0] - data[0, 0]) / 1e6
    mm_per_count = 25.4 / COUNTS_PER_INCH
    dx = data[:, header.index("dx_counts")] * mm_per_count
    dy = data[:, header.index("dy_counts")] * mm_per_count
    x, y = np.cumsum(dx), np.cumsum(dy)
    mm_path = os.path.join("recordings", f"{args.name}_200hz.csv")
    with open(mm_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x_mm", "y_mm", "dx_mm", "dy_mm"])
        for row in zip(t, x, y, dx, dy):
            w.writerow(["%.5f" % v for v in row])

    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0
    print(f"Saved {len(rows)} samples ({rate:.0f} Hz average) to {dev_path} and {mm_path}")
    if rate < 190:
        print("Warning: below 200 Hz. Check the board's 'missed sample ticks' messages.")


if __name__ == "__main__":
    main()
