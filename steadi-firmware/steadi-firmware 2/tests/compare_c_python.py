"""
Check that the C firmware's detector + correction match the Python simulation.

Builds nothing itself; compile the harness first (from the repo root):
    gcc -std=c11 -O2 -DCORRECTION_METHOD=CORRECTION_WFLC  -Isrc tests/host_harness.c \\
        src/tremor_detect.c src/correction.c src/wflc.c -lm -o tests/harness_WFLC
    gcc -std=c11 -O2 -DCORRECTION_METHOD=CORRECTION_NOTCH -Isrc tests/host_harness.c \\
        src/tremor_detect.c src/correction.c src/wflc.c -lm -o tests/harness_NOTCH
Then:
    python tests/compare_c_python.py
"""
import os, subprocess, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
from filter_compare import *


def run_c(binary, mode, d, trem):
    ax = np.gradient(np.gradient(trem)) * FS**2 / 1000.0 / 9.81    # tremor accel, g
    out = subprocess.run([os.path.join(HERE, binary), mode],
                         input="\n".join("%.9f %.9f" % (a, b) for a, b in zip(d, ax)),
                         capture_output=True, text=True, check=True).stdout
    return np.array([float(v) for v in out.split()])


if __name__ == "__main__":
    scen = [("Steady 5 Hz", steady_freq), ("Drifting 4-6 Hz", drifting_freq),
            ("Strong drift 3.5-6.5 Hz", strong_drift), ("ET-like 9 Hz", et_9hz)]
    print("%-24s %-22s %-22s %s" % ("", "Seeded WFLC: Py / C", "Notch Q=2: Py / C",
                                    "C with firmware depth: WFLC / notch"))
    for name, f in scen:
        r = {k: [] for k in ("pw", "cw", "pn", "cn", "rw", "rn")}
        for s in range(10, 15):
            t, vol, trem, d, _ = make_motion(12, f, amp_modulated, seed=s)
            ev = lambda out: evaluate(t, vol, trem, out)["removed"]
            r["pw"].append(ev(run_wflc(d, seeded=True)[0]))
            r["cw"].append(ev(run_c("harness_WFLC", "forced", d, trem)))
            r["pn"].append(ev(run_notch(d, 2.0)[0]))
            r["cn"].append(ev(run_c("harness_NOTCH", "forced", d, trem)))
            r["rw"].append(ev(run_c("harness_WFLC", "real", d, trem)))
            r["rn"].append(ev(run_c("harness_NOTCH", "real", d, trem)))
        m = {k: np.mean(v) for k, v in r.items()}
        print("%-24s %3.0f%% / %3.0f%%            %3.0f%% / %3.0f%%            %3.0f%% / %3.0f%%"
              % (name, m["pw"], m["cw"], m["pn"], m["cn"], m["rw"], m["rn"]))
