"""Standardised telemetry: one representation for real and simulated flight-computer data.

A ``Telemetry`` holds named channels, each with its own time base (real flight computers
sample altitude and acceleration at different rates):

    altitude      m   height above ground level (AGL)
    velocity_z    m/s vertical velocity (up positive)
    accel_axial   m/s^2 specific force along the nose axis (what an axial accelerometer reads;
                  +g at rest on the pad)
    pressure      Pa  static pressure

plus metadata (source, rocket, motor, mass, atmosphere, sampling, known uncertainties).
``telemetry_from_record`` converts a simulated flight into the same structure, optionally
through the simulated sensors, so simulated and real data can be compared (and exported)
identically. ``load_telemetry`` imports CSV logs through a declarative column map with unit
scaling (see validation_data/*.yaml).
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from ..errors import ConfigError
from ..physics.math3d import quat_rotate_inv
from ..simulation.record import FlightRecord

STANDARD_CHANNELS = {
    "altitude": "m",
    "velocity_z": "m/s",
    "speed": "m/s",  # total (unsigned) speed
    "accel_axial": "m/s^2",
    "pressure": "Pa",
}


@dataclass
class Channel:
    t: np.ndarray
    y: np.ndarray
    unit: str
    uncertainty: float | None = None  # 1-sigma measurement uncertainty, same unit

    @property
    def sampling_hz(self) -> float:
        return float(1.0 / np.median(np.diff(self.t))) if len(self.t) > 2 else float("nan")


@dataclass
class Telemetry:
    channels: dict[str, Channel]
    metadata: dict[str, Any] = field(default_factory=dict)

    def __getitem__(self, name: str) -> Channel:
        return self.channels[name]

    def has(self, name: str) -> bool:
        return name in self.channels

    def to_csv(self, directory: str | Path, stem: str = "telemetry") -> list[Path]:
        """One CSV per channel (own time base): ``<stem>_<channel>.csv`` with time,value."""
        out = []
        d = Path(directory)
        d.mkdir(parents=True, exist_ok=True)
        for name, ch in self.channels.items():
            p = d / f"{stem}_{name}.csv"
            np.savetxt(
                p,
                np.column_stack([ch.t, ch.y]),
                delimiter=",",
                header=f"time_s,{name}_{ch.unit}",
                comments="",
                fmt="%.9g",
            )
            out.append(p)
        return out


def telemetry_from_record(rec: FlightRecord, use_sensors: bool = False) -> Telemetry:
    """Simulated telemetry. ``use_sensors`` uses meas_* columns (needs fidelity >= 4)."""
    t = rec.col("t")
    if use_sensors:
        if not rec.has("meas_accel_x"):
            raise ConfigError("use_sensors needs a record with sensors (fidelity >= 4)")
        ax = rec.col("meas_accel_x")
        alt = rec.col("meas_baro_altitude")
        pres = rec.col("meas_baro_pressure")
    else:
        sf = np.empty(len(t))
        for i in range(len(t)):
            q = (rec.col("quat_w")[i], rec.col("quat_x")[i], rec.col("quat_y")[i], rec.col("quat_z")[i])
            a = (rec.col("acc_x")[i], rec.col("acc_y")[i], rec.col("acc_z")[i] + rec.col("gravity")[i])
            sf[i] = quat_rotate_inv(q, a)[0]
        ax, alt, pres = sf, rec.col("altitude"), rec.col("pressure")
    ch = {
        "altitude": Channel(t, alt, "m"),
        "altitude_baro": Channel(t, _baro_equivalent_altitude(rec.col("pressure")), "m"),
        "velocity_z": Channel(t, rec.col("vel_z"), "m/s"),
        "speed": Channel(t, rec.col("speed"), "m/s"),
        "accel_axial": Channel(t, ax, "m/s^2"),
        "pressure": Channel(t, pres, "Pa"),
    }
    return Telemetry(
        ch,
        {
            "source": "simulation",
            "simulation_id": rec.meta.simulation_id,
            "fidelity": rec.meta.fidelity,
            "sensors": use_sensors,
        },
    )


def _baro_equivalent_altitude(pressure: np.ndarray) -> np.ndarray:
    """Altitude a barometric altimeter would report: standard-atmosphere inversion of the (simulated)
    static pressure relative to the pad pressure (first sample). Differs from geometric height when the real
    atmosphere is not ISA (temperature offset), which is exactly what a real altimeter experiences."""
    from ..environment import ISAAtmosphere

    isa = ISAAtmosphere()
    h = np.array([isa.pressure_to_altitude(float(p)) for p in pressure])
    return h - h[0]


def _read_table(path: Path, delimiter: str, comment_header: bool) -> tuple[list[str], list[list[str]]]:
    with path.open(newline="", encoding="utf-8", errors="replace") as f:
        rows = list(csv.reader(f, delimiter=delimiter))
    if not rows:
        raise ConfigError(f"{path}: empty file")
    header = [h.strip().lstrip("#") for h in rows[0]]
    return header, rows[1:]


def _col_index(spec: Any, header: list[str], where: str) -> int:
    if isinstance(spec, int):
        return spec
    names = [h.strip() for h in header]
    if spec.strip() not in names:
        raise ConfigError(f"{where}: column {spec!r} not in header {names}")
    return names.index(spec.strip())


def load_channel(path: Path, delimiter: str, spec: dict[str, Any], default_unit: str) -> Channel:
    header, rows = _read_table(path, delimiter, True)
    ti = _col_index(spec["time_column"], header, "time_column")
    yi = _col_index(spec["column"], header, "column")
    scale = float(spec.get("scale", 1.0))
    offset = float(spec.get("offset", 0.0))
    t, y = [], []
    for r in rows:
        try:
            t.append(float(r[ti]))
            y.append(float(r[yi]) * scale + offset)
        except (ValueError, IndexError):
            continue  # skip non-numeric / short rows (e.g. repeated headers)
    if len(t) < 3:
        raise ConfigError(f"{path.name}: channel {spec['column']!r} has fewer than 3 numeric rows")
    ta, ya = np.asarray(t), np.asarray(y)
    order = np.argsort(ta, kind="stable")
    ta, ya = ta[order], ya[order]
    keep = np.concatenate([[True], np.diff(ta) > 0])  # drop duplicate time stamps
    return Channel(ta[keep], ya[keep], default_unit, spec.get("uncertainty"))


def load_telemetry(
    path: str | Path,
    channel_specs: dict[str, dict[str, Any]],
    delimiter: str = ",",
    metadata: dict[str, Any] | None = None,
) -> Telemetry:
    p = Path(path)
    if not p.exists():
        raise ConfigError(f"telemetry file not found: {p}")
    ch = {}
    for name, spec in channel_specs.items():
        if name not in STANDARD_CHANNELS:
            raise ConfigError(f"unknown telemetry channel {name!r}; standard: {list(STANDARD_CHANNELS)}")
        ch[name] = load_channel(p, delimiter, spec, STANDARD_CHANNELS[name])
    return Telemetry(ch, dict(metadata or {}))
