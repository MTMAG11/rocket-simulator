"""Import a flight-computer log (CSV) into the standard telemetry used by ``rocketsim validate``.

The log layout is described in a small YAML file (``LogFormat``), so a different logger or a firmware change needs a new
format file, not new code. Handled: time base in s/ms/us with counter wraparound, duplicate and out-of-order rows, gaps and
dropped samples (reported, never filled), accelerometer units (g or m/s^2) and axis sign, saturation (full scale or a flag
column), gyro units, pressure units, launch detection from the acceleration, and conversion of pressure to altitude.

The pressure-to-altitude equation, ``pressure_to_altitude_m``, is left as a TODO for the project owner
(docs/derivations/baro_altitude.md); the importer calls it for every log with a pressure column.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.signal import savgol_filter

from ..constants import G0
from ..errors import ConfigError

TIME_UNITS = {"s": 1.0, "ms": 1e-3, "us": 1e-6}
ACCEL_UNITS = {"g": G0, "m/s2": 1.0}
GYRO_UNITS = {"rad/s": 1.0, "deg/s": math.pi / 180.0}
PRESSURE_UNITS = {"Pa": 1.0, "hPa": 100.0, "mbar": 100.0, "kPa": 1000.0}
MIN_PRESSURE_PA, MAX_PRESSURE_PA = 10_000.0, 120_000.0  # outside this a reading is not a static pressure


def pressure_to_altitude_m(pressure_pa: Any, reference_pressure_pa: float) -> Any:
    """TODO(you): height above the level where the pressure is ``reference_pressure_pa`` [m], from the static pressure
    ``pressure_pa`` [Pa] (a numpy array or a scalar), with the standard-atmosphere lapse rate.

    Derive it in docs/derivations/baro_altitude.md; check it against the hand-calculated value in tests/test_flightlog.py
    (``pytest --runxfail -m user_todo`` shows that test failing until this is implemented).
    """
    raise NotImplementedError("pressure_to_altitude_m: see the TODO in rocket_sim/validation/flightlog.py")


Column = str | int


@dataclass
class LogFormat:
    name: str
    time_column: Column
    time_unit: str = "s"
    time_wrap: float | None = (
        None  # counter modulus in raw time units (4294967296 for a 32-bit microsecond counter)
    )
    counter_column: Column | None = None  # running sample number: gaps in it are dropped samples
    accel_columns: list[Column] = field(default_factory=list)  # three body axes as they appear in the log
    accel_unit: str = "g"
    accel_axial: int = 0  # which of the three columns is along the nose
    accel_axial_sign: int = (
        1  # +1 or -1: the sign that makes the axial reading +1 g at rest on the pad, nose up
    )
    accel_full_scale: float | None = None  # saturation level in accel_unit (per axis)
    accel_saturation_flag: Column | None = None  # nonzero when the sensor reports saturation
    gyro_columns: list[Column] = field(default_factory=list)
    gyro_unit: str = "deg/s"
    pressure_column: Column | None = None
    pressure_unit: str = "Pa"
    pressure_repeats_are_holds: bool = (
        False  # a pressure equal to the previous row is a held value, not a new sample
    )
    delimiter: str = ","
    skip_rows: int = 0  # text lines before the header row
    pad_s: float = 2.0  # time at the start of the log assumed to be on the pad
    launch_threshold_g: float = 2.5
    launch_min_duration_s: float = 0.05
    gap_factor: float = 3.0  # a time step longer than this many median steps is a gap
    velocity_window_s: float = 0.5  # Savitzky-Golay window for the barometric vertical velocity

    def validate(self) -> None:
        if self.time_unit not in TIME_UNITS:
            raise ConfigError(f"log format: time_unit must be one of {list(TIME_UNITS)}")
        if (
            self.accel_unit not in ACCEL_UNITS
            or self.gyro_unit not in GYRO_UNITS
            or self.pressure_unit not in PRESSURE_UNITS
        ):
            raise ConfigError("log format: unknown accel_unit, gyro_unit or pressure_unit")
        if len(self.accel_columns) != 3:
            raise ConfigError("log format: accel_columns needs exactly three columns (x, y, z in log order)")
        if self.gyro_columns and len(self.gyro_columns) != 3:
            raise ConfigError("log format: gyro_columns needs three columns or none")
        if self.accel_axial not in (0, 1, 2) or self.accel_axial_sign not in (-1, 1):
            raise ConfigError("log format: accel_axial must be 0, 1 or 2 and accel_axial_sign +1 or -1")
        if self.time_wrap is not None and self.time_wrap <= 0:
            raise ConfigError("log format: time_wrap must be > 0")
        for key in (
            "pad_s",
            "launch_threshold_g",
            "launch_min_duration_s",
            "gap_factor",
            "velocity_window_s",
        ):
            if not (math.isfinite(getattr(self, key)) and getattr(self, key) > 0):
                raise ConfigError(f"log format: {key} must be > 0")


def load_log_format(path: str | Path) -> LogFormat:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ConfigError(f"{path}: a log format is a mapping of keys to values")
    known = set(LogFormat.__dataclass_fields__)
    unknown = sorted(set(raw) - known)
    if unknown:
        raise ConfigError(f"{path}: unknown log-format keys {unknown}; known: {sorted(known)}")
    for req in ("name", "time_column", "accel_columns"):
        if req not in raw:
            raise ConfigError(f"{path}: log format needs '{req}'")
    fmt = LogFormat(**raw)
    fmt.validate()
    return fmt


@dataclass
class ImportReport:
    n_rows_read: int = 0
    n_rows_used: int = 0
    n_bad_rows: int = 0  # non-numeric or short rows (repeated headers, truncated last line)
    n_duplicate_times: int = 0
    n_out_of_order: int = 0
    n_wraps: int = 0
    gaps: list[tuple[float, float, int]] = field(
        default_factory=list
    )  # (raw start time [s], duration [s], missing samples)
    n_dropped_by_counter: int = 0
    n_invalid_pressure: int = 0  # zero, negative or implausible pressures (sensor not ready, bus error)
    n_saturated: int = 0
    saturated_intervals: list[tuple[float, float]] = field(default_factory=list)  # launch-relative [s]
    median_dt_s: float = 0.0
    launch_time_s: float | None = None  # on the log's own (unwrapped, seconds) time base
    pad_accel_g: float | None = None
    pad_pressure_pa: float | None = None
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {k: getattr(self, k) for k in self.__dataclass_fields__}

    def text(self) -> str:
        lines = [
            f"rows read {self.n_rows_read}, used {self.n_rows_used} (bad {self.n_bad_rows}, duplicate times {self.n_duplicate_times}, "
            f"out of order {self.n_out_of_order}); median step {self.median_dt_s * 1000:.3f} ms; time-counter wraps {self.n_wraps}",
            f"gaps: {len(self.gaps)}"
            + (f" (longest {max(g[1] for g in self.gaps):.3f} s)" if self.gaps else "")
            + f"; dropped by counter: {self.n_dropped_by_counter}",
            f"saturated accelerometer samples: {self.n_saturated}",
            "launch at "
            + ("not detected" if self.launch_time_s is None else f"{self.launch_time_s:.3f} s (log time)")
            + (f"; pad |a| {self.pad_accel_g:.3f} g" if self.pad_accel_g is not None else "")
            + (f"; pad pressure {self.pad_pressure_pa:.1f} Pa" if self.pad_pressure_pa is not None else ""),
        ]
        lines.extend(f"warning: {w}" for w in self.warnings)
        return "\n".join(lines)


@dataclass
class ImportedLog:
    t: np.ndarray  # seconds since launch (since the first sample if no launch was found)
    accel_axial: np.ndarray  # m/s^2, +g at rest on the pad
    accel_axial_valid: np.ndarray  # same, NaN where saturated
    saturated: np.ndarray  # bool
    accel_g_magnitude: np.ndarray
    gyro: np.ndarray | None  # rad/s, n x 3
    pressure: np.ndarray | None  # Pa, NaN where not a new sample
    altitude: np.ndarray | None  # m above the pad, NaN where there is no pressure sample
    velocity_z: np.ndarray | None  # m/s from the smoothed altitude, NaN where there is no pressure sample
    report: ImportReport
    format: LogFormat

    def to_csv(self, path: str | Path) -> Path:
        cols: dict[str, np.ndarray] = {
            "time_s": self.t,
            "accel_axial_ms2": self.accel_axial,
            "accel_axial_valid_ms2": self.accel_axial_valid,
            "accel_saturated": self.saturated.astype(float),
        }
        if self.pressure is not None and self.altitude is not None and self.velocity_z is not None:
            cols.update(
                {"pressure_pa": self.pressure, "altitude_m": self.altitude, "velocity_z_ms": self.velocity_z}
            )
        if self.gyro is not None:
            for i, a in enumerate("xyz"):
                cols[f"gyro_{a}_rads"] = self.gyro[:, i]
        names = list(cols)
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(names)
            for i in range(len(self.t)):
                w.writerow(
                    [
                        "" if not math.isfinite(float(cols[n][i])) else f"{float(cols[n][i]):.9g}"
                        for n in names
                    ]
                )
        return p

    def flight_definition(self, csv_name: str) -> dict[str, Any]:
        """A skeleton flight definition for ``rocketsim validate``; ``sim_config`` and the source details are yours to fill in."""
        channels: dict[str, Any] = {
            "accel_axial": {"time_column": "time_s", "column": "accel_axial_valid_ms2"},
        }
        if self.altitude is not None:
            channels["altitude"] = {"time_column": "time_s", "column": "altitude_m"}
            channels["velocity_z"] = {"time_column": "time_s", "column": "velocity_z_ms"}
            channels["pressure"] = {"time_column": "time_s", "column": "pressure_pa"}
        return {
            "name": self.format.name,
            "description": "TODO: describe the flight",
            "sim_config": "TODO: path to a simulation config of this vehicle, relative to this file",
            "source": {
                "name": self.format.name,
                "quality": "TODO: describe the data quality",
                "known_uncertainties": "TODO",
            },
            "telemetry": {
                "file": csv_name,
                "launch_threshold_altitude_m": 15.0,
                "landing_altitude_m": 5.0,
                "channels": channels,
            },
        }


def _index(spec: Column, header: list[str], what: str) -> int:
    if isinstance(spec, int):
        return spec
    names = [h.strip() for h in header]
    if spec.strip() not in names:
        raise ConfigError(f"log: column {spec!r} ({what}) not in the header {names}")
    return names.index(spec.strip())


def import_flight_log(
    path: str | Path,
    fmt: LogFormat,
    pressure_to_altitude: Callable[[Any, float], Any] | None = None,
) -> ImportedLog:
    """Read ``path`` according to ``fmt`` and return clean, launch-aligned channels with a report of what was wrong with the log."""
    p = Path(path)
    if not p.exists():
        raise ConfigError(f"flight log not found: {p}")
    fmt.validate()
    rep = ImportReport()
    lines = p.read_text(encoding="utf-8", errors="replace").splitlines()[fmt.skip_rows :]
    rows = list(csv.reader(lines, delimiter=fmt.delimiter))
    if len(rows) < 2:
        raise ConfigError(f"{p.name}: no data rows")
    header = rows[0]
    idx_t = _index(fmt.time_column, header, "time")
    idx_a = [_index(c, header, "accelerometer") for c in fmt.accel_columns]
    idx_g = [_index(c, header, "gyroscope") for c in fmt.gyro_columns]
    idx_p = _index(fmt.pressure_column, header, "pressure") if fmt.pressure_column is not None else None
    idx_c = _index(fmt.counter_column, header, "counter") if fmt.counter_column is not None else None
    idx_s = (
        _index(fmt.accel_saturation_flag, header, "saturation flag")
        if fmt.accel_saturation_flag is not None
        else None
    )
    need = [idx_t, *idx_a, *idx_g] + [i for i in (idx_p, idx_c, idx_s) if i is not None]

    data: list[list[float]] = []
    for r in rows[1:]:
        rep.n_rows_read += 1
        try:
            data.append([float(r[i]) for i in need])
        except (ValueError, IndexError):
            rep.n_bad_rows += 1
    if len(data) < 20:
        raise ConfigError(f"{p.name}: fewer than 20 usable rows")
    col = {c: k for k, c in enumerate(need)}
    arr = np.array(data)

    # ---- time base: unwrap the counter, then keep strictly increasing times
    t_raw = arr[:, col[idx_t]]
    unwrapped = t_raw.copy()
    if fmt.time_wrap is not None:
        offset = 0.0
        for i in range(1, len(t_raw)):
            if t_raw[i] < t_raw[i - 1] - 0.5 * fmt.time_wrap:
                offset += fmt.time_wrap
                rep.n_wraps += 1
            unwrapped[i] = t_raw[i] + offset
    keep = np.ones(len(unwrapped), dtype=bool)
    last = -math.inf
    for i, tv in enumerate(unwrapped):
        if tv == last:
            keep[i] = False
            rep.n_duplicate_times += 1
        elif tv < last:
            keep[i] = False
            rep.n_out_of_order += 1
        else:
            last = tv
    arr, unwrapped = arr[keep], unwrapped[keep]
    t = unwrapped * TIME_UNITS[fmt.time_unit]
    rep.n_rows_used = len(t)
    dt = np.diff(t)
    med = float(np.median(dt))
    rep.median_dt_s = med
    for j in np.nonzero(dt > fmt.gap_factor * med)[0]:
        rep.gaps.append((float(t[j]), float(dt[j]), int(round(dt[j] / med)) - 1))
    if idx_c is not None:
        dc = np.diff(arr[:, col[idx_c]])
        rep.n_dropped_by_counter = int(np.sum(dc[dc > 1] - 1))
    if rep.gaps:
        rep.warnings.append(
            f"{len(rep.gaps)} gap(s) in the log (longest {max(g[1] for g in rep.gaps):.3f} s); the gaps are not filled"
        )

    # ---- accelerometer
    a_raw = arr[:, [col[i] for i in idx_a]]
    scale = ACCEL_UNITS[fmt.accel_unit]
    a_ms2 = a_raw * scale
    axial = a_ms2[:, fmt.accel_axial] * fmt.accel_axial_sign
    sat = np.zeros(len(t), dtype=bool)
    if fmt.accel_full_scale is not None:
        sat |= np.any(np.abs(a_raw) >= 0.995 * fmt.accel_full_scale, axis=1)
    if idx_s is not None:
        sat |= arr[:, col[idx_s]] != 0
    rep.n_saturated = int(sat.sum())
    mag_g = np.linalg.norm(a_ms2, axis=1) / G0

    # ---- launch detection: |a| above the threshold for a sustained time (saturation counts as above)
    pad_mask = t - t[0] <= fmt.pad_s
    rep.pad_accel_g = float(np.median(mag_g[pad_mask]))
    if not 0.8 <= rep.pad_accel_g <= 1.2:
        rep.warnings.append(
            f"the median |a| on the pad is {rep.pad_accel_g:.2f} g, not about 1 g: check accel_unit and the log's pad period"
        )
    exceed = (mag_g > fmt.launch_threshold_g) | sat
    t_launch = None
    start = None
    for i in range(len(t)):
        if exceed[i]:
            if start is None:
                start = i
            if t[i] - t[start] >= fmt.launch_min_duration_s:
                t_launch = float(t[start])
                break
        else:
            start = None
    rep.launch_time_s = t_launch
    if t_launch is None:
        rep.warnings.append("no launch found in the log (acceleration never stayed above the threshold)")
    t_rel = t - (t_launch if t_launch is not None else float(t[0]))
    if sat.any():
        edges = np.diff(np.concatenate([[0], sat.astype(int), [0]]))
        for s0, s1 in zip(np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0], strict=True):
            rep.saturated_intervals.append((float(t_rel[s0]), float(t_rel[s1 - 1])))
        rep.warnings.append(
            f"{rep.n_saturated} accelerometer sample(s) at full scale or flagged saturated: excluded from accel_axial_valid"
        )
    valid = np.where(sat, np.nan, axial)

    gyro = None
    if idx_g:
        gyro = arr[:, [col[i] for i in idx_g]] * GYRO_UNITS[fmt.gyro_unit]

    # ---- pressure -> altitude
    pressure = altitude = vel_z = None
    if idx_p is not None:
        pressure = arr[:, col[idx_p]] * PRESSURE_UNITS[fmt.pressure_unit]
        bad_p = ~((pressure > MIN_PRESSURE_PA) & (pressure < MAX_PRESSURE_PA))
        rep.n_invalid_pressure = int(bad_p.sum())
        if rep.n_invalid_pressure:
            rep.warnings.append(
                f"{rep.n_invalid_pressure} pressure sample(s) outside {MIN_PRESSURE_PA:.0f}-{MAX_PRESSURE_PA:.0f} Pa were ignored"
            )
        pressure = np.where(bad_p, np.nan, pressure)
        if fmt.pressure_repeats_are_holds:
            prev = np.concatenate([[np.nan], pressure[:-1]])
            pressure = np.where(pressure == prev, np.nan, pressure)  # equal to the previous row: a held value
        before = (t_rel < -0.1) if t_launch is not None else pad_mask
        pad_vals = pressure[before & np.isfinite(pressure)]
        if len(pad_vals) < 3:
            raise ConfigError("not enough pressure samples before launch to find the pad pressure")
        p_pad = float(np.mean(pad_vals))
        rep.pad_pressure_pa = p_pad
        fn = pressure_to_altitude or pressure_to_altitude_m
        ok = np.isfinite(pressure)
        altitude = np.full(len(t), np.nan)
        altitude[ok] = np.asarray(fn(pressure[ok], p_pad), dtype=float)
        vel_z = _vertical_velocity(t, altitude, fmt.velocity_window_s)
    return ImportedLog(t_rel, axial, valid, sat, mag_g, gyro, pressure, altitude, vel_z, rep, fmt)


def _vertical_velocity(t: np.ndarray, alt: np.ndarray, window_s: float) -> np.ndarray:
    ok = np.isfinite(alt)
    out = np.full(len(t), np.nan)
    if ok.sum() < 15:
        return out
    ts, hs = t[ok], alt[ok]
    dt = float(np.median(np.diff(ts)))
    tu = np.arange(ts[0], ts[-1] + 0.5 * dt, dt)
    hu = np.interp(tu, ts, hs)
    win = max(int(round(window_s / dt)), 7)
    win += 1 - win % 2
    if win >= len(tu):
        return out
    v = savgol_filter(hu, win, 3, deriv=1, delta=dt)
    out[ok] = np.interp(ts, tu, v)
    return out


def write_import(result: ImportedLog, out_dir: str | Path, stem: str = "flight") -> dict[str, Path]:
    """Write ``<stem>.csv`` (standard channels), ``<stem>_import.json`` (the report) and ``<stem>.flight.yaml`` (skeleton)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = result.to_csv(out / f"{stem}.csv")
    rep_path = out / f"{stem}_import.json"
    rep_path.write_text(json.dumps(result.report.to_dict(), indent=2), encoding="utf-8")
    yaml_path = out / f"{stem}.flight.yaml"
    yaml_path.write_text(
        yaml.safe_dump(result.flight_definition(csv_path.name), sort_keys=False), encoding="utf-8"
    )
    return {"csv": csv_path, "report": rep_path, "flight": yaml_path}
