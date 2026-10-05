"""Input-uncertainty Monte Carlo for real-flight validation.

A single deterministic comparison hides how much of the apogee error is explained by inputs nobody measured
(wind, air temperature, launch angle, as-flown mass, motor thrust). This module samples those inputs from documented
priors, re-simulates the flight, and reports where the REAL apogee falls inside the simulated distribution.

Interpretation (stated once, here): if the real apogee lies within the 90 % band, the model is *consistent with* the
flight given the input uncertainty (it does not prove the physics is right). If it lies far outside, the error is
not explained by these inputs and points at model-form error (or at an input convention problem).

Priors (1-sigma unless stated), applied around each flight's nominal configuration:
    wind            speed ~ |N(0, 3)| m/s, direction uniform, constant with altitude (replaces the nominal wind)
    air temperature offset from ISA: + N(0, 4) K
    launch elevation: + N(0, 1.5) deg (clipped to <= 90)
    as-flown dry mass: x N(1, 0.02)
    motor thrust scale: x N(1, 0.03)  (manufacturer total-impulse tolerance is typically +-5-10 %)
    [optional] drag multiplier: x N(1, 0.07)  -> reported separately as "inputs + drag"
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..config import read_mapping
from .compare import run_validation

INPUT_PRIORS = {
    "wind_speed_sigma_ms": 3.0,
    "temperature_offset_sigma_k": 4.0,
    "launch_elevation_sigma_deg": 1.5,
    "dry_mass_rel_sigma": 0.02,
    "thrust_scale_sigma": 0.03,
    "drag_scale_sigma": 0.07,
}


@dataclass
class McResult:
    flight: str
    n: int
    real_apogee_m: float
    nominal_apogee_m: float
    mean_m: float
    std_m: float
    p05_m: float
    p95_m: float
    z_score: float
    inside_90: bool
    with_drag: bool

    def to_dict(self) -> dict[str, Any]:
        return {k: (bool(v) if isinstance(v, (bool, np.bool_)) else v) for k, v in self.__dict__.items()}


def _nominal(flight_yaml: Path) -> dict[str, float]:
    d = read_mapping(flight_yaml)
    sim = read_mapping(flight_yaml.parent / d["sim_config"])
    return {
        "temp": float(sim.get("environment", {}).get("atmosphere", {}).get("temperature_offset_k", 0.0)),
        "elev": float(sim.get("launch", {}).get("elevation_deg", 90.0)),
        "mass": float(sim["rocket"]["dry_mass_kg"]),
        "thrust": float(sim.get("motor", {}).get("thrust_scale", 1.0)),
    }


def sample_overrides(nom: dict[str, float], rng: np.random.Generator, with_drag: bool) -> dict[str, Any]:
    p = INPUT_PRIORS
    o: dict[str, Any] = {
        "environment.wind.model": "constant",
        "environment.wind.speed_ms": abs(float(rng.normal(0.0, p["wind_speed_sigma_ms"]))),
        "environment.wind.direction_from_deg": float(rng.uniform(0.0, 360.0)),
        "environment.atmosphere.temperature_offset_k": nom["temp"]
        + float(rng.normal(0.0, p["temperature_offset_sigma_k"])),
        "launch.elevation_deg": min(
            89.9, nom["elev"] + float(rng.normal(0.0, p["launch_elevation_sigma_deg"]))
        ),
        "rocket.dry_mass_kg": nom["mass"] * float(rng.normal(1.0, p["dry_mass_rel_sigma"])),
        "motor.thrust_scale": nom["thrust"] * float(rng.normal(1.0, p["thrust_scale_sigma"])),
    }
    if with_drag:
        o["rocket.aero.drag_scale"] = float(rng.normal(1.0, p["drag_scale_sigma"]))
    return o


def _one(args: tuple[str, dict[str, Any]]) -> tuple[float, float]:
    yaml_path, ov = args
    r = run_validation(yaml_path, overrides=ov, plot=False)
    return r.metrics["apogee"]["sim"], r.metrics["apogee"]["real"]


def run_input_mc(
    flight_yaml: str | Path, n: int = 40, seed: int = 0, with_drag: bool = False, workers: int = 1
) -> McResult:
    fy = Path(flight_yaml)
    nom = _nominal(fy)
    nominal = run_validation(fy, plot=False).metrics["apogee"]
    rng = np.random.default_rng(seed)
    jobs = [(str(fy), sample_overrides(nom, rng, with_drag)) for _ in range(n)]
    if workers > 1:
        with ProcessPoolExecutor(workers) as ex:
            out = list(ex.map(_one, jobs))
    else:
        out = [_one(j) for j in jobs]
    sims = np.array([o[0] for o in out])
    real = float(nominal["real"])
    sd = float(sims.std(ddof=1))
    lo, hi = float(np.percentile(sims, 5)), float(np.percentile(sims, 95))
    return McResult(
        fy.stem,
        n,
        real,
        float(nominal["sim"]),
        float(sims.mean()),
        sd,
        lo,
        hi,
        float((real - sims.mean()) / sd) if sd > 0 else float("nan"),
        bool(lo <= real <= hi),
        with_drag,
    )
