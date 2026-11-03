#!/usr/bin/env python3
"""Run up to three complete bottle pick/place cycles.

Entry point on Jetson:
    python3 /home/nvidia/calibration/run_three_cycles_v3.py

Each cycle gives grasp-photo bottle detection 30 seconds in total.  If no
bottle is found in that window, the runner exits cleanly without starting the
next cycle.  Any other cycle error also stops immediately.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


DEMO = Path(__file__).with_name("bottle_pick_place_demo_v3.py")


def main() -> int:
    parser = argparse.ArgumentParser(description="run three bottle pick/place cycles")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--no-bottle-timeout", type=float, default=30.0,
                        help="seconds per cycle to wait for a bottle")
    args = parser.parse_args()
    if args.cycles < 1 or args.no_bottle_timeout <= 0:
        parser.error("--cycles must be >= 1 and --no-bottle-timeout must be > 0")

    for cycle in range(1, args.cycles + 1):
        print(f"===== CYCLE {cycle}/{args.cycles} START =====", flush=True)
        proc = subprocess.Popen(
            [sys.executable, str(DEMO), "--detect-timeout-sec", str(args.no_bottle_timeout)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            print(line, end="", flush=True)
        rc = proc.wait()
        if rc == 3:
            print(f"[STOP] cycle {cycle}: no bottle within {args.no_bottle_timeout:.0f}s; exiting", flush=True)
            return 0
        if rc != 0:
            print(f"[ABORT] cycle {cycle} exited with code {rc}", flush=True)
            return rc
        print(f"===== CYCLE {cycle}/{args.cycles} DONE =====", flush=True)
    print(f"===== ALL {args.cycles} CYCLES DONE =====", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
