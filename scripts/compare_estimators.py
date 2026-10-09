"""Compare the linear Kalman filter (nav_kf) and the error-state EKF (ekf) against truth on identical simulated flights.

Both filters see the same sensor noise (same seed: the sensor random streams do not depend on the estimator, and the flight is
passive, so the truth is identical). "Truth" is the zero-error reference. The comparison is a simulation study only: no filter has
been run on real data.

    python scripts/compare_estimators.py [--seeds 8] [--out validation_results/estimator_comparison.md]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rocket_sim.config import apply_overrides, config_from_dict, read_mapping  # noqa: E402
from rocket_sim.simulation import Simulation  # noqa: E402

BASE = ROOT / "configs" / "example_g80.yaml"
# the simulator's default accelerometer with the full scale raised so the example motor's peak acceleration is not clipped
ACCEL = {
    "rate_hz": 400.0,
    "noise_std": 0.03,
    "bias_std": 0.05,
    "bias_walk_std": 1e-3,
    "scale_error_std": 0.002,
    "quantization": 157.0 / 32768.0,
    "saturation": 400.0,
}
COMMON = {
    "fidelity": 5,
    "controller.type": "none",
    "environment.wind.model": "none",
    "motor.ignition_delay_s": 2.0,
    "estimator.alignment_time_s": 1.5,
    "simulation.stop_after_apogee_s": 2.0,
    "sensors.accelerometer": ACCEL,
}
SCENARIOS: dict[str, tuple[str, dict]] = {
    "nominal": ("default sensors, no wind", {}),
    "wind 6 m/s": (
        "constant 6 m/s wind from 270 deg (the vehicle weathercocks and drifts)",
        {
            "environment.wind.model": "constant",
            "environment.wind.speed_ms": 6.0,
            "environment.wind.direction_from_deg": 270.0,
        },
    ),
    "baro dropout 90 %": (
        "90 % of the barometer samples lost at random",
        {"sensors.barometer.dropout_probability": 0.9},
    ),
    "GPS dropout 90 %": ("90 % of the GPS fixes lost at random", {"sensors.gps.dropout_probability": 0.9}),
    "no GPS": ("GPS not used by the filter (barometer and IMU only)", {"estimator.gps_enabled": False}),
    "vibration, filter not told": (
        "accelerometer noise 1.0 m/s^2 in the sensor (33x the configured 0.03), filter still assumes 0.03",
        {"sensors.accelerometer": {**ACCEL, "noise_std": 1.0}},
    ),
    "vibration, filter told": (
        "same sensor noise; EKF noise_scale 33, which inflates the assumed noise of ALL sensors (so barometer and GPS are down-weighted too); the linear filter has no such setting and is unchanged",
        {"sensors.accelerometer": {**ACCEL, "noise_std": 1.0}, "estimator.noise_scale": 33.0},
    ),
}


def run(estimator: str, overrides: dict, seed: int):
    raw = read_mapping(BASE)
    cfg = config_from_dict(
        apply_overrides(raw, {**COMMON, **overrides, "estimator.type": estimator}), base_dir=BASE.parent
    )
    return Simulation(cfg, seed=seed).run()


def errors(rec) -> tuple[float, float, float]:
    """RMS position [m], velocity [m/s] and attitude [deg] error between ignition and apogee."""
    t_up = rec.col("t")
    sel = (rec.col("phase") >= 2) & (rec.col("phase") <= 5)
    dp = np.linalg.norm(np.stack([rec.col(f"est_pos_{a}") - rec.col(f"pos_{a}") for a in "xyz"], 1), axis=1)
    dv = np.linalg.norm(np.stack([rec.col(f"est_vel_{a}") - rec.col(f"vel_{a}") for a in "xyz"], 1), axis=1)
    q = np.stack([rec.col(f"est_quat_{a}") for a in "wxyz"], 1)
    qt = np.stack([rec.col(f"quat_{a}") for a in "wxyz"], 1)
    ang = 2.0 * np.degrees(np.arccos(np.clip(np.abs(np.sum(q * qt, 1)), 0.0, 1.0)))
    assert t_up.size and sel.any()
    return tuple(float(np.sqrt(np.mean(x[sel] ** 2))) for x in (dp, dv, ang))  # type: ignore[return-value]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--out", help="write the markdown report to this file")
    a = ap.parse_args()
    lines = [
        "# Linear Kalman filter vs error-state EKF (simulation)",
        "",
        f"Example 29 mm vehicle (`configs/example_g80.yaml`), passive flight, {a.seeds} seeds per scenario, identical sensor noise for both",
        "filters. RMS error against truth from ignition to apogee, mean (standard deviation over seeds). Simulation only.",
        "",
        "| scenario | filter | position [m] | velocity [m/s] | attitude [deg] |",
        "|---|---|---|---|---|",
    ]
    for name, (_, sensors) in SCENARIOS.items():
        for est in ("nav_kf", "ekf"):
            res = np.array([errors(run(est, sensors, s)) for s in range(a.seeds)])
            m, sd = res.mean(axis=0), res.std(axis=0, ddof=1)
            lines.append(
                f"| {name} | {est} | " + " | ".join(f"{m[i]:.2f} ({sd[i]:.2f})" for i in range(3)) + " |"
            )
            print(lines[-1], flush=True)
    lines += ["", "Scenarios:", ""] + [f"* **{n}**: {d}" for n, (d, _) in SCENARIOS.items()]
    text = "\n".join(lines) + "\n"
    if a.out:
        Path(a.out).write_text(text, encoding="utf-8")
        print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
