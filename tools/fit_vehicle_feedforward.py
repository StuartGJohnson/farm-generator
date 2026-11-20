"""Fit a compact forward/reverse road-load and pedal model from open-loop CSV.

The output CSV is read directly by tractor_control_node. No ROS imports are
needed, so the same raw data can be refit repeatedly without Isaac Sim.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def fit(input_path: Path, output_path: Path):
    rows = list(csv.DictReader(input_path.open()))
    samples = []
    for index, row in enumerate(rows):
        if index < 3 or index + 3 >= len(rows):
            continue
        gear = int(float(row["gear"]))
        speed = gear * float(row["speed_mps"])
        if not 0.12 < speed < 1.8 or float(row["phase_s"]) < 0.30:
            continue
        # A local time regression reduces derivative noise. Reject windows
        # crossing a phase boundary or simulation-time gap.
        window = rows[index - 3:index + 4]
        if any(r["phase"] != row["phase"] for r in window):
            continue
        times = np.array([float(r["sim_s"]) for r in window])
        if times[-1] - times[0] > 0.65 or times[-1] - times[0] < 0.08:
            continue
        velocities = np.array([gear * float(r["speed_mps"]) for r in window])
        acceleration = np.polyfit(times - times[3], velocities, 1)[0]
        samples.append((gear, speed, float(row["accelerator"]), float(row["brake0"]), acceleration, row["phase"]))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    report = {"source": str(input_path), "model": "a_along_gear = coast0 + coast1*speed_abs + throttle_gain*accelerator - brake_gain*brake0",
              "gears": {}}
    with output_path.open("w", newline="") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(["gear", "coast0", "coast1", "throttle_gain", "brake_gain"])
        for gear in (1, -1):
            part = [s for s in samples if s[0] == gear]
            if len(part) < 30 or len({round(s[2], 2) for s in part if s[2] > 0}) < 2 or not any(s[3] > 0 for s in part):
                raise ValueError(f"Not enough throttle, coast, and brake samples for gear {gear}")
            x = np.array([[1, s[1], s[2], -s[3]] for s in part])
            y = np.array([s[4] for s in part])
            # Withhold one throttle level and one brake level to test whether
            # interpolation predicts trials absent from the fitting set.
            held_out = np.array([s[5] in ("throttle_0.12", "brake_0.06") for s in part])
            validation_rmse = None
            if held_out.any() and (~held_out).sum() > 30:
                training_coeff, *_ = np.linalg.lstsq(x[~held_out], y[~held_out], rcond=None)
                validation_rmse = float(np.sqrt(np.mean((y[held_out] - x[held_out] @ training_coeff) ** 2)))
            coeff, *_ = np.linalg.lstsq(x, y, rcond=None)
            if coeff[2] <= 0 or coeff[3] <= 0:
                raise ValueError(f"Nonphysical pedal gain for gear {gear}: {coeff}")
            writer.writerow([gear, *[f"{value:.9g}" for value in coeff]])
            rmse = float(np.sqrt(np.mean((y - x @ coeff) ** 2)))
            report["gears"][str(gear)] = {"samples": len(part), "rmse_mps2": rmse,
                "held_out_levels": ["throttle_0.12", "brake_0.06"],
                "held_out_samples": int(held_out.sum()), "held_out_rmse_mps2": validation_rmse,
                "coast0_mps2": float(coeff[0]), "coast1_per_s": float(coeff[1]),
                "throttle_gain_mps2": float(coeff[2]), "brake_gain_mps2": float(coeff[3]),
                "speed_range_mps": [float(min(s[1] for s in part)), float(max(s[1] for s in part))]}
    output_path.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("samples", type=Path)
    parser.add_argument("model", type=Path)
    args = parser.parse_args()
    print(json.dumps(fit(args.samples, args.model), indent=2))
