"""Quantitative error metrics (never just "the graphs look similar")."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class SeriesError:
    n: int
    rmse: float
    mae: float
    max_abs: float
    bias: float  # mean(sim - real)
    nrmse_pct: float  # rmse / (max(real) - min(real)) * 100

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


def series_error(real: np.ndarray, sim: np.ndarray) -> SeriesError:
    d = sim - real
    span = float(np.max(real) - np.min(real)) or 1.0
    rmse = float(np.sqrt(np.mean(d * d)))
    return SeriesError(
        len(d),
        rmse,
        float(np.mean(np.abs(d))),
        float(np.max(np.abs(d))),
        float(np.mean(d)),
        100.0 * rmse / span,
    )


def scalar_error(real: float, sim: float) -> dict[str, float]:
    return {
        "real": real,
        "sim": sim,
        "abs_error": sim - real,
        "pct_error": 100.0 * (sim - real) / real if real else float("nan"),
    }


def crossing_time(
    t: np.ndarray, y: np.ndarray, level: float, rising: bool = True, start: int = 0
) -> float | None:
    """First time y crosses ``level`` (linear interpolation) after index ``start``."""
    for i in range(max(start, 1), len(y)):
        a, b = y[i - 1], y[i]
        hit = (a < level <= b) if rising else (a > level >= b)
        if hit:
            return float(t[i - 1] + (level - a) / (b - a) * (t[i] - t[i - 1]))
    return None


def smooth_derivative(t: np.ndarray, y: np.ndarray, window_s: float) -> np.ndarray:
    """Derivative of a locally fitted quadratic (Savitzky-Golay-like on non-uniform time)."""
    out = np.empty(len(t))
    half = window_s / 2.0
    j0 = 0
    for i in range(len(t)):
        while t[j0] < t[i] - half:
            j0 += 1
        j1 = j0
        while j1 + 1 < len(t) and t[j1 + 1] <= t[i] + half:
            j1 += 1
        if j1 - j0 < 3:
            out[i] = np.nan
            continue
        tt = t[j0 : j1 + 1] - t[i]
        c = np.polyfit(tt, y[j0 : j1 + 1], 2)
        out[i] = c[1]
    return out
