"""Structural load screening: peak q*alpha and lateral aerodynamic load against configurable limits.

This is a screen, not a structural analysis: it says "this run loaded the airframe beyond the limit you configured",
not whether the airframe breaks. The limits are inputs (``structure.*`` in the config); the simulator ships none.
Derivation: docs/derivations/structural_load.md.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..config.schema import StructureCfg
from .record import FlightRecord

MIN_AIRSPEED = (
    20.0  # [m/s] below this the angle of attack is ill-defined (same cut-off as the aero range check)
)


def q_alpha(q_dyn: Any, alpha: Any) -> Any:
    """Dynamic pressure times the magnitude of the total angle of attack [Pa*rad] (scalars or arrays)."""
    return np.asarray(q_dyn) * np.abs(np.asarray(alpha))


def body_normal_force(lift: float, drag: float, alpha: float) -> float:
    """TODO(you): force component perpendicular to the body axis [N], from the wind-axis ``lift`` and ``drag`` [N] and the
    total angle of attack ``alpha`` [rad].

    Derive it in docs/derivations/structural_load.md (section 3): resolve the aerodynamic force, given along and normal to
    the relative wind, onto the body axes. Check it against the hand-calculated value in tests/test_structural_warning.py
    (``pytest --runxfail -m user_todo`` shows that test failing until this is implemented).
    """
    raise NotImplementedError("body_normal_force: see the TODO in rocket_sim/simulation/structure.py")


@dataclass
class StructuralResult:
    max_q_alpha_pa_rad: float | None = None
    max_q_alpha_time_s: float | None = None
    max_lateral_load_n: float | None = None
    max_lateral_load_time_s: float | None = None
    lateral_load_available: bool = False
    exceeded: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def summary(self, limits: StructureCfg) -> dict[str, Any]:
        return {
            "max_q_alpha_pa_rad": self.max_q_alpha_pa_rad,
            "max_q_alpha_time_s": self.max_q_alpha_time_s,
            "max_lateral_load_n": self.max_lateral_load_n,
            "structural_limits_set": limits.max_q_alpha_pa_rad is not None
            or limits.max_lateral_load_n is not None,
            "structural_limit_exceeded": bool(self.exceeded),
        }


def check_structure(rec: FlightRecord, limits: StructureCfg) -> StructuralResult:
    """Peak loads while the vehicle is flying under power or coasting (before apogee), against ``limits``."""
    res = StructuralResult()
    needed = ("t", "qdyn", "aoa", "airspeed", "phase")
    if not all(rec.has(c) for c in needed):
        return res
    t = rec.col("t")
    sel = (rec.col("phase") < 5) & (rec.col("airspeed") > MIN_AIRSPEED)
    if not sel.any():
        return res
    ts = t[sel]
    qa = q_alpha(rec.col("qdyn")[sel], rec.col("aoa")[sel])
    i = int(np.argmax(qa))
    res.max_q_alpha_pa_rad, res.max_q_alpha_time_s = float(qa[i]), float(ts[i])
    if limits.max_q_alpha_pa_rad is not None and res.max_q_alpha_pa_rad > limits.max_q_alpha_pa_rad:
        res.exceeded.append("q_alpha")
        res.warnings.append(
            f"STRUCTURAL: peak q*alpha {res.max_q_alpha_pa_rad:.0f} Pa*rad at t={res.max_q_alpha_time_s:.2f} s exceeds the configured "
            f"limit {limits.max_q_alpha_pa_rad:.0f} Pa*rad (angle of attack {math.degrees(float(rec.col('aoa')[sel][i])):.0f} deg); "
            "the airframe may have been overloaded"
        )
    if rec.has("lift") and rec.has("drag"):
        try:
            n = np.array(
                [
                    body_normal_force(lf, dr - ch, a)
                    for lf, dr, ch, a in zip(
                        rec.col("lift")[sel],
                        rec.col("drag")[sel],
                        rec.col("chute_drag")[sel] if rec.has("chute_drag") else np.zeros(sel.sum()),
                        rec.col("aoa")[sel],
                        strict=True,
                    )
                ]
            )
        except NotImplementedError:
            n = None
        if n is not None:
            j = int(np.argmax(np.abs(n)))
            res.lateral_load_available = True
            res.max_lateral_load_n, res.max_lateral_load_time_s = float(abs(n[j])), float(ts[j])
            if limits.max_lateral_load_n is not None and res.max_lateral_load_n > limits.max_lateral_load_n:
                res.exceeded.append("lateral_load")
                res.warnings.append(
                    f"STRUCTURAL: peak lateral aerodynamic load {res.max_lateral_load_n:.0f} N at t={res.max_lateral_load_time_s:.2f} s "
                    f"exceeds the configured limit {limits.max_lateral_load_n:.0f} N"
                )
    if limits.max_lateral_load_n is not None and not res.lateral_load_available:
        res.warnings.append(
            "structure.max_lateral_load_n is set but not checked: body_normal_force() is not implemented yet "
            "(rocket_sim/simulation/structure.py)"
        )
    return res


def apply_structure_check(rec: FlightRecord, limits: StructureCfg) -> StructuralResult:
    """Run the check and record its numbers in ``rec.summary`` and its warnings in ``rec.meta.warnings``."""
    res = check_structure(rec, limits)
    rec.summary.update(res.summary(limits))
    rec.meta.warnings.extend(res.warnings)
    return res
