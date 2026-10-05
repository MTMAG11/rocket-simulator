"""Turn a FlightRecord into ML-ready arrays according to a DatasetSection.

* ``tabular``  : one row per (resampled) timestep with the requested input and target columns.
* ``windowed`` : sliding windows X (N, history, n_inputs) with targets Y at the window end
                 ("current"), ``horizon_s`` later ("future") or the next K samples
                 ("trajectory", Y shape (N, K, n_targets)).

Everything is derived from the recorded telemetry; the simulator knows nothing about any
particular network. Inputs may be *measured* columns (``meas_*``, ``est_*``) or truth; the
labels are normally truth. Choosing deployable inputs (no truth leakage) is the user's decision
and is recorded in the manifest.

Derived features
----------------
time_since_ignition          t - ignition_delay (>= 0): known to a flight computer that commands ignition
time_since_launch_detected   seconds since the onset of the launch-detector exceedance (0 before
                             detection); requires fidelity >= 5 with an estimator
Both are computed on the native time axis and then resampled like any other column.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..errors import DataQualityError
from ..simulation.record import FlightRecord
from .spec import DatasetSection, FeatureSpec


@dataclass
class TabularData:
    t: np.ndarray  # (n,)
    inputs: np.ndarray  # (n, F)
    targets: np.ndarray  # (n, L)
    input_names: list[str]
    target_names: list[str]


@dataclass
class WindowedData:
    x: np.ndarray  # (N, H, F) float32
    y: np.ndarray  # (N, L) or (N, K, L) float32
    t_end: np.ndarray  # (N,) time of the last input sample
    input_names: list[str]
    target_names: list[str]


def required_columns(section: DatasetSection) -> tuple[str, ...]:
    cols = [f.column for f in section.inputs + section.targets if f.column]
    if any(f.derived == "time_since_launch_detected" for f in section.inputs):
        cols.append("launch_detected")
    return tuple(dict.fromkeys(cols))


def _native_values(rec: FlightRecord, f: FeatureSpec) -> np.ndarray:
    if f.column is not None:
        return rec.col(f.column)
    t = rec.col("t")
    if f.derived == "time_since_ignition":
        delay = float(rec.meta.config.get("motor", {}).get("ignition_delay_s", 0.0))
        return np.maximum(t - delay, 0.0)
    if f.derived == "time_since_launch_detected":
        flag = rec.col("launch_detected") > 0.5
        out = np.zeros_like(t)
        if flag.any():
            i0 = int(np.argmax(flag))
            out[i0:] = t[i0:] - t[i0]
        return out
    raise DataQualityError(f"unknown derived feature {f.derived!r}")


def flight_slice(rec: FlightRecord, trim: str) -> tuple[float, float]:
    t = rec.col("t")
    if trim == "none":
        return float(t[0]), float(t[-1])
    lift = rec.event("liftoff")
    land = rec.event("landing")
    if lift is None:
        raise DataQualityError("cannot trim to flight: no liftoff event")
    if land is None:
        if rec.meta.status != "truncated":
            raise DataQualityError("cannot trim to flight: no landing event")
        return lift.t, float(t[-1])  # run deliberately stopped after apogee
    return lift.t, land.t


def resample(
    rec: FlightRecord,
    values: list[np.ndarray],
    t0: float,
    t1: float,
    dt: float | None,
    causal_flags: list[bool] | None = None,
) -> tuple[np.ndarray, list[np.ndarray]]:
    """Resample columns onto a uniform grid [t0, t1] (linear), or slice natively when dt is None."""
    t = rec.col("t")
    if dt is None:
        m = (t >= t0 - 1e-12) & (t <= t1 + 1e-12)
        return t[m], [v[m] for v in values]
    n = int(np.floor((t1 - t0) / dt + 1e-9)) + 1
    grid = t0 + dt * np.arange(n)
    out = []
    for v, causal in zip(values, causal_flags or [False] * len(values)):
        if causal:  # inputs: zero-order hold (no interpolation toward the future)
            out.append(v[np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)])
        else:
            out.append(np.interp(grid, t, v))
    return grid, out


def _lagged(x: np.ndarray, lag: int) -> np.ndarray:
    if lag == 0:
        return x
    out = np.empty_like(x)
    out[lag:] = x[:-lag]
    out[:lag] = x[0]
    return out


def build_tabular(rec: FlightRecord, section: DatasetSection) -> TabularData:
    t0, t1 = flight_slice(rec, section.trim)
    cols_in = [_native_values(rec, f) for f in section.inputs]
    cols_out = [_native_values(rec, f) for f in section.targets]
    # EVERY input is zero-order-held (value of the latest native sample at or before the grid time): interpolating a
    # truth input would read the sample AFTER the grid time. Targets are labels and may be interpolated.
    causal = [True] * len(cols_in) + [False] * len(cols_out)
    grid, rs = resample(rec, cols_in + cols_out, t0, t1, section.sample_dt_s, causal)
    n_in = len(cols_in)
    xs = [_lagged(rs[i], section.inputs[i].lag) for i in range(n_in)]
    ys = rs[n_in:]
    if len(grid) < 2:
        raise DataQualityError("flight too short for the requested sampling")
    return TabularData(
        grid,
        np.column_stack(xs),
        np.column_stack(ys),
        [f.label for f in section.inputs],
        [f.label for f in section.targets],
    )


def build_windows(rec: FlightRecord, section: DatasetSection) -> WindowedData:
    tab = build_tabular(rec, section)
    w = section.window
    dt = section.sample_dt_s
    assert dt is not None
    n = len(tab.t)
    h = w.history
    if w.target == "current":
        k, last = 0, n - 1
    elif w.target == "future":
        k = max(int(round(w.horizon_s / dt)), 1)
        last = n - 1 - k
    else:
        k = w.trajectory_steps
        last = n - 1 - k
    ends = np.arange(h - 1, last + 1, w.stride)
    if len(ends) == 0:
        raise DataQualityError("flight shorter than one window")
    view = np.lib.stride_tricks.sliding_window_view(tab.inputs, h, axis=0)  # (n-h+1, F, h)
    x = np.transpose(view[ends - (h - 1)], (0, 2, 1)).astype(np.float32)
    if w.target in ("current", "future"):
        y = tab.targets[ends + k].astype(np.float32)
    else:
        idx = ends[:, None] + 1 + np.arange(k)[None, :]
        y = tab.targets[idx].astype(np.float32)  # (N, K, L)
    return WindowedData(x, y, tab.t[ends], tab.input_names, tab.target_names)
