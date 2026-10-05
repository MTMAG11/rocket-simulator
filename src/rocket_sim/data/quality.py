"""Quality gates: bad simulations never enter a dataset.

Checks (all configurable through ``QualityLimits``):
  * simulation raised an exception (config/numerical error)            -> rejected by the caller
  * non-finite values (NaN/inf) in any recorded column
  * run status other than "ok" (timeout / never left the pad)
  * speed, altitude or angular rate outside physical sanity bounds ("exploding states")
  * quaternion norm drift (invalid orientation)
  * required telemetry columns missing
  * no landing event recorded / apogee below a minimum
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..simulation.record import FlightRecord


@dataclass
class QualityLimits:
    max_speed_ms: float = 3000.0
    max_altitude_m: float = 150_000.0
    min_altitude_m: float = -1e-3
    max_rate_rad_s: float = 1000.0
    max_quat_norm_error: float = 1e-6
    min_apogee_m: float = 1.0
    require_landing: bool = True
    allowed_status: tuple[str, ...] = ("ok",)
    required_columns: tuple[str, ...] = field(default_factory=tuple)


def check_record(rec: FlightRecord, limits: QualityLimits | None = None) -> list[str]:
    """Return a list of problems (empty list = record passes)."""
    lim = limits or QualityLimits()
    problems: list[str] = []
    missing = [c for c in lim.required_columns if not rec.has(c)]
    if missing:
        problems.append(f"missing columns: {missing}")
    if rec.n_rows < 3:
        problems.append("fewer than 3 recorded rows")
        return problems
    if not np.isfinite(rec.data).all():
        bad = [c for c in rec.columns if not np.isfinite(rec.col(c)).all()]
        problems.append(f"non-finite values in columns {bad[:5]}")
        return problems  # further numeric checks are meaningless
    if rec.meta.status not in lim.allowed_status:
        problems.append(f"run status '{rec.meta.status}'")
    if lim.require_landing and rec.event("landing") is None:
        problems.append("no landing event")
    if float(np.max(rec.col("speed"))) > lim.max_speed_ms:
        problems.append(f"speed exceeds {lim.max_speed_ms} m/s")
    alt = rec.col("altitude")
    if float(alt.max()) > lim.max_altitude_m:
        problems.append(f"altitude exceeds {lim.max_altitude_m} m")
    if float(alt.min()) < lim.min_altitude_m:
        problems.append("vehicle went below ground level")
    if float(alt.max()) < lim.min_apogee_m:
        problems.append(f"apogee below {lim.min_apogee_m} m")
    rates = np.abs(np.stack([rec.col("omega_p"), rec.col("omega_q"), rec.col("omega_r")]))
    if float(rates.max()) > lim.max_rate_rad_s:
        problems.append(f"angular rate exceeds {lim.max_rate_rad_s} rad/s")
    qn = np.sqrt(sum(rec.col(f"quat_{a}") ** 2 for a in "wxyz"))
    if float(np.max(np.abs(qn - 1.0))) > lim.max_quat_norm_error:
        problems.append("quaternion norm drift (invalid orientation)")
    return problems
