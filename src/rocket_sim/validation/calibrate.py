"""Physically-bounded calibration with leave-one-out cross-validation.

Rules:
  * only parameters with a physical meaning and physical bounds may be fitted;
  * each parameter carries a written justification that is stored with the result;
  * a fit is only accepted if it improves the HELD-OUT flights (leave-one-out), otherwise the
    result is reported as "not supported by the data" and the model is left unchanged.

``fit_shared_parameter`` fits one MODEL parameter shared by all flights (e.g. a drag
multiplier). Per-flight input uncertainties (thrust tolerance, atmosphere, wind) are NOT fitted
here: fitting them to the flight they are evaluated on would hide model error.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import minimize_scalar

from .compare import run_validation


@dataclass
class ParamSpec:
    path: str
    lower: float
    upper: float
    justification: str
    nominal: float = 1.0


@dataclass
class LooResult:
    param: ParamSpec
    folds: list[dict[str, Any]] = field(default_factory=list)
    summary: dict[str, Any] = field(default_factory=dict)

    def text(self) -> str:
        lines = [
            f"Leave-one-out calibration of {self.param.path} in [{self.param.lower}, {self.param.upper}]",
            f"  justification: {self.param.justification}",
        ]
        for f in self.folds:
            lines.append(
                f"  held out {f['held_out']:<22} fitted on {', '.join(f['train'])}: value {f['value']:.3f}  "
                f"apogee error nominal {f['nominal_err_pct']:+6.2f} %  calibrated {f['calibrated_err_pct']:+6.2f} %"
            )
        s = self.summary
        lines.append(
            f"  RMS apogee error on held-out flights: nominal {s['rms_nominal_pct']:.2f} %  "
            f"calibrated {s['rms_calibrated_pct']:.2f} %"
        )
        lines.append("  verdict: " + s["verdict"])
        return "\n".join(lines)


def _apogee_err(flight: Path, path: str, value: float) -> float:
    r = run_validation(flight, overrides={path: value}, plot=False)
    return float(r.metrics["apogee"]["pct_error"])


def fit_shared_parameter(flights: list[Path], param: ParamSpec) -> LooResult:
    res = LooResult(param)
    names = [f.stem for f in flights]
    cache: dict[tuple[str, float], float] = {}

    def err(f: Path, v: float) -> float:
        k = (f.stem, round(v, 6))
        if k not in cache:
            cache[k] = _apogee_err(f, param.path, v)
        return cache[k]

    for i, held in enumerate(flights):
        train = [f for j, f in enumerate(flights) if j != i]
        opt = minimize_scalar(
            lambda v: sum(err(f, v) ** 2 for f in train),
            bounds=(param.lower, param.upper),
            method="bounded",
            options={"xatol": 0.01},
        )
        v = float(opt.x)
        res.folds.append(
            {
                "held_out": held.stem,
                "train": [f.stem for f in train],
                "value": v,
                "nominal_err_pct": err(held, param.nominal),
                "calibrated_err_pct": err(held, v),
            }
        )
    nom = np.array([f["nominal_err_pct"] for f in res.folds])
    cal = np.array([f["calibrated_err_pct"] for f in res.folds])
    rn, rc = float(np.sqrt(np.mean(nom**2))), float(np.sqrt(np.mean(cal**2)))
    verdict = (
        "calibration generalises: adopt"
        if rc < 0.8 * rn
        else "NOT supported: held-out flights do not improve materially; parameter left at nominal"
    )
    res.summary = {
        "rms_nominal_pct": rn,
        "rms_calibrated_pct": rc,
        "verdict": verdict,
        "n_flights": len(names),
    }
    return res
