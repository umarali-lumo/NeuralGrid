#!/usr/bin/env python3
"""
Physics-informed synthetic sample generator for NeuralGrid development.

Starts from a small seed of laboratory-style operating points and expands
them with controlled perturbations (voltage sag, current ramps, PF drift,
thermal-index trajectories) to support offline pipeline testing when a
large labelled industrial corpus is not yet available.

This script does NOT replace real-world validation.
"""

from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def make_sample(
    v: float,
    i: float,
    pf: float,
    freq: float = 50.0,
    energy: float = 0.0,
    priority: float = 1.0,
) -> list[float]:
    """Build a rough 14-D feature vector consistent with the project docs."""
    nominal_v = 230.0
    ref_i = 15.0
    power = v * i * pf
    v_dev = (v - nominal_v) / nominal_v
    pf_dev = 1.0 - pf
    util = i / ref_i
    di_dt = random.uniform(-0.5, 0.5) * (i / max(ref_i, 1e-6))
    dv_dt = random.uniform(-0.2, 0.2)
    var_i = abs(di_dt) * random.uniform(0.1, 0.8)
    thermal = clamp((i / ref_i) ** 2 * random.uniform(0.5, 1.5), 0.0, 3.0)
    return [
        v, i, power, energy, freq, pf, priority,
        v_dev, pf_dev, util, di_dt, dv_dt, var_i, thermal,
    ]


def generate(n: int, seed: int) -> list[list[float]]:
    random.seed(seed)
    rows: list[list[float]] = []
    regimes = [
        (230.0, 2.0, 0.95),
        (228.0, 6.0, 0.90),
        (220.0, 10.0, 0.82),
        (215.0, 12.5, 0.75),
    ]
    for _ in range(n):
        base = random.choice(regimes)
        v = base[0] + random.uniform(-3.0, 3.0)
        i = max(0.1, base[1] + random.uniform(-1.5, 2.5))
        pf = clamp(base[2] + random.uniform(-0.05, 0.05), 0.6, 1.0)
        rows.append(make_sample(v, i, pf))
    return rows


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("-n", type=int, default=200, help="number of samples")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("-o", type=Path, default=Path("synthetic_features.csv"))
    args = p.parse_args()
    rows = generate(args.n, args.seed)
    header = [
        "V", "I", "P", "E", "f", "PF", "priority",
        "v_dev", "pf_dev", "util", "dI_dt", "dV_dt", "i_var", "thermal",
    ]
    with args.o.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f"Wrote {len(rows)} samples → {args.o}")


if __name__ == "__main__":
    main()
