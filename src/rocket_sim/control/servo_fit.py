"""Fit the servo model (delay, time constant, rate limit, deadband, backlash) to bench step-response data.

The fit simulates the same ``ActuatorBank`` the flight simulation uses, so the fitted numbers can be dropped into
``tvc:`` without translation. Angles may be in any unit (degrees or radians) as long as command and measurement use the
same one; the fitted rate limit, deadband and backlash come out in that unit (per second for the rate).
Derivation and identifiability: docs/derivations/servo_model.md.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from ..errors import ConfigError
from .actuators import ActuatorBank

MAX_SUBSTEP = 0.002  # [s] the simulation never takes a longer step than this
PARAMS = ("delay", "tau", "rate", "deadband", "backlash")


@dataclass(frozen=True)
class ServoParams:
    delay: float = 0.0  # [s]
    tau: float = 0.0  # [s]
    rate: float = math.inf  # [angle/s]
    deadband: float = 0.0  # half-width [angle]
    backlash: float = 0.0  # total gap [angle]


def simulate_response(
    p: ServoParams, t: np.ndarray, cmd: np.ndarray, max_substep: float = MAX_SUBSTEP
) -> np.ndarray:
    """Output angle at the sample times ``t`` for the command ``cmd`` (zero-order hold between samples), starting at 0."""
    t = np.asarray(t, dtype=float)
    cmd = np.asarray(cmd, dtype=float)
    bank = ActuatorBank(
        [1e12],
        [p.rate if math.isfinite(p.rate) else 1e12],
        [p.tau],
        [0.0],
        deadband=[p.deadband],
        backlash=[p.backlash],
    )
    # The transport delay is applied by moving the instant at which each command reaches the bank: that keeps the
    # response continuous in the delay (a queue delay would only change in whole simulation steps).
    arrivals = t + p.delay
    out = np.empty_like(t)
    now = float(t[0])
    ia = 0  # next command not yet delivered
    for k in range(len(t)):
        target = float(t[k])
        while now < target - 1e-12:
            nxt = float(arrivals[ia]) if ia < len(t) else math.inf
            if nxt <= now + 1e-12:
                bank.command(now, [float(cmd[ia])])
                ia += 1
                continue
            h = min(target - now, max_substep, nxt - now)
            bank.step(now, h)
            now += h
        out[k] = bank.state[0]
    return out


@dataclass
class ServoFit:
    params: ServoParams
    stderr: dict[str, float]
    identifiable: dict[str, bool]
    rms_residual: float
    n_samples: int
    converged: bool
    starts_agree: bool
    free: tuple[str, ...]
    notes: list[str] = field(default_factory=list)

    def report(self, angle_unit: str = "deg") -> str:
        u = angle_unit
        lines = [
            f"servo fit: {self.n_samples} samples, residual rms {self.rms_residual:.4g} {u}, converged={self.converged}"
        ]
        rows = (
            ("delay", f"{self.params.delay * 1000:.2f} ms", self.stderr.get("delay", math.nan) * 1000, "ms"),
            ("tau", f"{self.params.tau * 1000:.2f} ms", self.stderr.get("tau", math.nan) * 1000, "ms"),
            ("rate", f"{self.params.rate:.4g} {u}/s", self.stderr.get("rate", math.nan), f"{u}/s"),
            ("deadband", f"{self.params.deadband:.4g} {u}", self.stderr.get("deadband", math.nan), u),
            ("backlash", f"{self.params.backlash:.4g} {u}", self.stderr.get("backlash", math.nan), u),
        )
        for name, value, se, unit in rows:
            if name not in self.free:
                lines.append(f"  {name:<9} {value:>16}   (fixed)")
            elif not self.identifiable.get(name, True):
                lines.append(f"  {name:<9} {'not identifiable':>16}   (the data never exercised it)")
            else:
                lines.append(f"  {name:<9} {value:>16}   +/- {se:.3g} {unit} (1 sigma)")
        lines.extend(f"  note: {n}" for n in self.notes)
        return "\n".join(lines)


def fit_servo(
    t: np.ndarray,
    cmd: np.ndarray,
    meas: np.ndarray,
    *,
    fit_backlash: bool = False,
    fit_rate: bool = True,
    fit_deadband: bool = True,
    max_delay: float = 0.25,
) -> ServoFit:
    """Least-squares fit of the servo parameters to (time, commanded angle, measured angle) bench data.

    Backlash is only fitted with ``fit_backlash=True`` (it needs ``actuators.play_operator``). The covariance is
    sigma^2 (J^T J)^-1 with sigma^2 = RSS / (n - p). A parameter whose Jacobian column is (numerically) zero is reported as
    not identifiable: for example the rate limit when no step was large enough to saturate it.
    """
    t, cmd, meas = (np.asarray(a, dtype=float) for a in (t, cmd, meas))
    if not (t.ndim == cmd.ndim == meas.ndim == 1 and len(t) == len(cmd) == len(meas)):
        raise ConfigError("fit_servo: time, command and measurement must be 1-D arrays of equal length")
    if len(t) < 20 or not (np.all(np.isfinite(t)) and np.all(np.isfinite(cmd)) and np.all(np.isfinite(meas))):
        raise ConfigError("fit_servo: need at least 20 finite samples")
    if np.any(np.diff(t) <= 0):
        raise ConfigError(
            "fit_servo: time must be strictly increasing (drop duplicate and out-of-order samples first)"
        )
    span = float(max(np.ptp(cmd), np.ptp(meas), 1e-9))
    dt = float(np.median(np.diff(t)))
    free = (
        ["delay", "tau"]
        + (["rate"] if fit_rate else [])
        + (["deadband"] if fit_deadband else [])
        + (["backlash"] if fit_backlash else [])
    )
    scale = {"delay": 0.02, "tau": 0.05, "rate": span / 0.2, "deadband": 0.02 * span, "backlash": 0.05 * span}
    lo = {"delay": 0.0, "tau": 1e-4, "rate": span / 50.0, "deadband": 0.0, "backlash": 0.0}
    hi = {"delay": max_delay, "tau": 1.0, "rate": span * 1e3, "deadband": 0.3 * span, "backlash": 0.3 * span}
    observed_rate = float(np.max(np.abs(np.diff(meas) / np.diff(t)))) if len(t) > 1 else span
    fixed = ServoParams(rate=math.inf)

    def to_params(x: np.ndarray) -> ServoParams:
        d = {n: float(v * scale[n]) for n, v in zip(free, x, strict=True)}
        return ServoParams(
            delay=d.get("delay", 0.0),
            tau=d.get("tau", 0.0),
            rate=d.get("rate", fixed.rate),
            deadband=d.get("deadband", 0.0),
            backlash=d.get("backlash", 0.0),
        )

    def residual(x: np.ndarray) -> np.ndarray:
        return simulate_response(to_params(x), t, cmd) - meas

    bounds = (np.array([lo[n] / scale[n] for n in free]), np.array([hi[n] / scale[n] for n in free]))
    starts = []
    for d0 in (0.0, 0.03, 0.08):
        for tau0 in (0.02, 0.08):
            guess = {
                "delay": d0,
                "tau": tau0,
                "rate": max(observed_rate, span / 5.0),
                "deadband": 0.01 * span,
                "backlash": 0.02 * span,
            }
            starts.append(np.clip([guess[n] / scale[n] for n in free], bounds[0] + 1e-9, bounds[1] - 1e-9))
    ranked = sorted(
        starts, key=lambda x0: float(np.sum(residual(x0) ** 2))
    )  # cheap screen, then refine the best two
    solutions = [least_squares(residual, x0, bounds=bounds, diff_step=1e-3, max_nfev=40) for x0 in ranked[:2]]
    best = min(solutions, key=lambda s: s.cost)
    n, p = len(t), len(free)
    rss = 2.0 * best.cost
    sigma2 = rss / max(n - p, 1)
    jac = best.jac
    col = np.linalg.norm(jac, axis=0)
    identifiable = {
        name: bool(col[i] > 1e-6 * float(col.max()) and col[i] > 1e-9) for i, name in enumerate(free)
    }
    cov = sigma2 * np.linalg.pinv(jac.T @ jac)
    stderr = {name: float(math.sqrt(max(cov[i, i], 0.0))) * scale[name] for i, name in enumerate(free)}
    good = [s for s in solutions if s.cost <= best.cost * 1.5 + 1e-12]
    spread = max((float(np.max(np.abs(s.x - best.x))) for s in good), default=0.0)
    notes = []
    params = to_params(best.x)
    if "rate" in free:
        # Is there any evidence for a rate limit? Compare with the same fit without one (chi-square difference, 4 ~ 2 sigma).
        unlimited = simulate_response(
            ServoParams(params.delay, params.tau, math.inf, params.deadband, params.backlash), t, cmd
        )
        d_chi2 = (float(np.sum((unlimited - meas) ** 2)) - rss) / max(sigma2, 1e-300)
        if not identifiable["rate"] or d_chi2 < 4.0:
            identifiable["rate"] = False
            params = ServoParams(params.delay, params.tau, math.inf, params.deadband, params.backlash)
            notes.append(
                "no rate limit is visible in the data (no step was fast enough to saturate it): use a larger step to measure it"
            )
    if best.x[free.index("tau")] * scale["tau"] < 2 * dt:
        notes.append(
            f"time constant is close to the sample period ({dt * 1000:.1f} ms): sample faster to resolve it"
        )
    return ServoFit(
        params=params,
        stderr=stderr,
        identifiable=identifiable,
        rms_residual=float(math.sqrt(rss / n)),
        n_samples=n,
        converged=bool(best.success),
        starts_agree=bool(spread < 0.05),
        free=tuple(free),
        notes=notes,
    )


def load_bench_csv(
    path: str | Path,
    time_col: str | int,
    cmd_col: str | int,
    meas_col: str | int,
    *,
    delimiter: str = ",",
    time_scale: float = 1.0,
    skip_rows: int = 0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read (time [s], command, measurement) from a delimited text file. Columns by header name or index; the time column
    is multiplied by ``time_scale`` (for example 1e-3 for milliseconds). Non-numeric rows are skipped."""
    rows = list(
        csv.reader(Path(path).read_text(encoding="utf-8").splitlines()[skip_rows:], delimiter=delimiter)
    )
    if not rows:
        raise ConfigError(f"{path}: empty file")
    header = [c.strip() for c in rows[0]]

    def index(col: str | int) -> int:
        if isinstance(col, int):
            return col
        if col not in header:
            raise ConfigError(f"{path}: column {col!r} not in header {header}")
        return header.index(col)

    ti, ci, mi = index(time_col), index(cmd_col), index(meas_col)
    data = []
    for r in rows:
        try:
            data.append((float(r[ti]), float(r[ci]), float(r[mi])))
        except (ValueError, IndexError):
            continue
    if len(data) < 20:
        raise ConfigError(f"{path}: fewer than 20 numeric rows")
    a = np.array(data)
    return a[:, 0] * time_scale, a[:, 1], a[:, 2]
