"""Flight record container, phases, events and summary metrics."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any

import numpy as np

from ..constants import G0


class FlightPhase(IntEnum):
    PRELAUNCH = 0
    IGNITION = 1
    POWERED_ASCENT = 2
    BURNOUT = 3
    COAST = 4
    APOGEE = 5
    DESCENT = 6
    LANDED = 7


@dataclass
class FlightEvent:
    t: float
    name: str
    info: dict[str, float] = field(default_factory=dict)


@dataclass
class RunMetadata:
    simulation_id: str
    seed: int
    config_hash: str
    sim_version: str
    physics_version: str
    schema_version: str
    fidelity: int
    fast: bool
    dt: float
    integrator: str
    created_utc: str
    status: str = "ok"  # ok | timeout | no_liftoff
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)  # fidelity overrides actually applied
    config: dict[str, Any] = field(default_factory=dict)
    wall_time_s: float = 0.0
    input_files: dict[str, str] = field(default_factory=dict)  # external input file -> sha256
    n_steps: int = 0

    def to_dict(self) -> dict[str, Any]:
        return dict(self.__dict__)


@dataclass
class FlightRecord:
    columns: list[str]
    data: np.ndarray  # shape (n_rows, n_columns), float64
    events: list[FlightEvent]
    meta: RunMetadata
    summary: dict[str, float | str | bool | None] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self._index = {c: i for i, c in enumerate(self.columns)}

    def col(self, name: str) -> np.ndarray:
        return self.data[:, self._index[name]]

    def has(self, name: str) -> bool:
        return name in self._index

    @property
    def n_rows(self) -> int:
        return int(self.data.shape[0])

    def event(self, name: str) -> FlightEvent | None:
        for e in self.events:
            if e.name == name:
                return e
        return None

    def to_dict(self) -> dict[str, np.ndarray]:
        """Column name -> array (feed to pandas.DataFrame / pyarrow.table if desired)."""
        return {c: self.col(c) for c in self.columns}


class Recorder:
    """Growing float64 table; one row per recorded step."""

    def __init__(self, columns: list[str], capacity: int = 4096) -> None:
        self.columns = columns
        self._n = 0
        self._buf = np.empty((capacity, len(columns)))

    def append(self, row: dict[str, float]) -> None:
        if self._n == self._buf.shape[0]:
            self._buf = np.concatenate([self._buf, np.empty_like(self._buf)])
        self._buf[self._n] = [row[c] for c in self.columns]
        self._n += 1

    def finish(self) -> np.ndarray:
        return self._buf[: self._n].copy()


def compute_summary(
    rec: FlightRecord, g_ref: float = G0, landing_zone: tuple[float, float, float] | None = None
) -> dict[str, Any]:
    """Headline flight metrics from the record and its events (all SI)."""
    t = rec.col("t")
    alt = rec.col("altitude")
    speed = rec.col("speed")
    acc = np.stack([rec.col("acc_x"), rec.col("acc_y"), rec.col("acc_z")], axis=1)
    out: dict[str, Any] = {}
    ev = {e.name: e for e in rec.events}

    liftoff = ev.get("liftoff")
    t_lift = liftoff.t if liftoff else None
    flying = t >= (t_lift if t_lift is not None else math.inf)

    i_ap = int(np.argmax(alt))
    out["apogee_m"] = float(alt[i_ap])
    out["apogee_time_s"] = float(t[i_ap])
    out["max_velocity_ms"] = float(np.max(speed))
    out["max_velocity_time_s"] = float(t[int(np.argmax(speed))])
    out["liftoff_time_s"] = t_lift
    if flying.any():
        a_mag = np.linalg.norm(acc[flying], axis=1)
        out["max_acceleration_ms2"] = float(np.max(a_mag))
        sf = acc[flying] + np.array([0.0, 0.0, g_ref])
        out["max_load_factor_g"] = float(np.max(np.linalg.norm(sf, axis=1)) / g_ref)
    else:
        out["max_acceleration_ms2"] = 0.0
        out["max_load_factor_g"] = 0.0
    bo = ev.get("burnout")
    out["burnout_time_s"] = bo.t if bo else None
    out["burnout_altitude_m"] = bo.info.get("altitude") if bo else None
    out["burnout_velocity_ms"] = bo.info.get("speed") if bo else None
    rx = ev.get("rail_exit")
    out["rail_exit_velocity_ms"] = rx.info.get("speed") if rx else None
    ap = ev.get("apogee")
    out["apogee_event_time_s"] = ap.t if ap else None
    ch = ev.get("parachute_deploy")
    out["parachute_deploy_time_s"] = ch.t if ch else None
    out["parachute_deploy_altitude_m"] = ch.info.get("altitude") if ch else None
    out["max_mach"] = float(np.max(rec.col("mach")))
    out["max_qdyn_pa"] = float(np.max(rec.col("qdyn")))
    out["max_thrust_n"] = float(np.max(rec.col("thrust")))
    # AoA is only meaningful with enough airflow (near apogee/at rest it is ill-defined)
    ascent = flying & (t <= out["apogee_time_s"]) & (rec.col("airspeed") >= 15.0)
    out["max_aoa_ascent_deg"] = float(math.degrees(np.max(rec.col("aoa")[ascent]))) if ascent.any() else 0.0
    sm = rec.col("static_margin")
    out["static_margin_launch_cal"] = float(sm[0])
    out["min_static_margin_cal"] = float(np.min(sm[flying])) if flying.any() else float(sm[0])
    out["liftoff_mass_kg"] = float(rec.col("mass")[0])
    out["liftoff_thrust_to_weight"] = float(
        rec.col("thrust")[: max(len(t) // 20, 3)].max() / (rec.col("mass")[0] * g_ref)
    )
    imp = ev.get("landing")
    if imp is not None:
        out["impact_time_s"] = imp.t
        out["impact_speed_ms"] = imp.info["speed"]
        out["impact_vertical_speed_ms"] = imp.info["vz"]
        out["landing_x_m"] = imp.info["x"]
        out["landing_y_m"] = imp.info["y"]
        out["landing_distance_m"] = math.hypot(imp.info["x"], imp.info["y"])
        out["flight_time_s"] = imp.t - (t_lift if t_lift is not None else 0.0)
        if landing_zone is not None:
            cx, cy, rad = landing_zone
            out["landing_miss_distance_m"] = math.hypot(imp.info["x"] - cx, imp.info["y"] - cy)
            out["landing_in_zone"] = bool(out["landing_miss_distance_m"] <= rad)
    else:
        out["impact_time_s"] = None
    out["status"] = rec.meta.status
    return out
