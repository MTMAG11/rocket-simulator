"""Drag coefficient versus Mach number from the coast phase (burnout to apogee) of a flight log.

Method and error sources: docs/derivations/drag_from_coast.md. The only equation left to the project owner is
``solve_cd``; everything around it (window selection, smoothing and differentiation, atmosphere, Mach binning, bootstrap
uncertainty) is implemented and tested.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from scipy.signal import savgol_filter

from ..constants import G0
from ..environment.atmosphere import ISAAtmosphere
from ..errors import ConfigError


def solve_cd(mass: Any, rho: Any, speed: Any, area: Any, drag_decel: Any) -> Any:
    """TODO(you): drag coefficient from the drag-induced deceleration.

    ``mass`` [kg], air density ``rho`` [kg/m^3], ``speed`` [m/s], reference ``area`` [m^2] and ``drag_decel`` [m/s^2] (the
    deceleration caused by drag alone, positive). Derive it in docs/derivations/drag_from_coast.md, section 1. It is called
    with numpy arrays, so plain arithmetic is enough. Check it against the hand-calculated value in
    tests/test_drag_fit.py (``pytest --runxfail -m user_todo`` shows that test failing until this is implemented).
    """
    raise NotImplementedError("solve_cd: see the TODO in rocket_sim/validation/drag_fit.py")


@dataclass
class CdBin:
    mach_low: float
    mach_high: float
    mach_mean: float
    cd: float
    se_random: float  # block-bootstrap standard error of the bin mean
    se_systematic: float | None  # from the relative mass/area/density uncertainties you supply, else None
    n: int

    @property
    def se_total(self) -> float | None:
        return None if self.se_systematic is None else math.hypot(self.se_random, self.se_systematic)


@dataclass
class CdResult:
    bins: list[CdBin]
    window: tuple[float, float]  # coast window [s]
    method: str
    n_samples: int
    notes: list[str] = field(default_factory=list)

    def combined(self) -> tuple[float, float]:
        """Inverse-variance weighted mean Cd over the bins and its random standard error.

        Slow samples (small q) have a much larger error than fast ones, so a plain average over samples is dominated by noise."""
        w = np.array([1.0 / max(b.se_random, 1e-12) ** 2 for b in self.bins])
        cd = np.array([b.cd for b in self.bins])
        return float(np.sum(w * cd) / np.sum(w)), float(1.0 / math.sqrt(np.sum(w)))

    def report(self) -> str:
        lines = [
            f"drag from coast ({self.method}): window {self.window[0]:.2f}-{self.window[1]:.2f} s, {self.n_samples} samples",
            "  Mach          Cd     random   systematic   n",
        ]
        for b in self.bins:
            sys_txt = "  n/a   " if b.se_systematic is None else f"{b.se_systematic:7.3f} "
            lines.append(
                f"  {b.mach_low:.2f}-{b.mach_high:.2f}  {b.cd:7.3f}  {b.se_random:7.3f}  {sys_txt}  {b.n:4d}"
            )
        if len(self.bins) > 1:
            cd, se = self.combined()
            lines.append(f"  all bins (inverse-variance weighted): Cd {cd:.3f} +/- {se:.3f} (random)")
        lines.extend(f"  note: {n}" for n in self.notes)
        return "\n".join(lines)


def _uniform(t: np.ndarray, y: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    tu = np.arange(float(t[0]), float(t[-1]) + 0.5 * dt, dt)
    return tu, np.interp(tu, t, y)


def _odd(n: int) -> int:
    return n if n % 2 == 1 else n + 1


def estimate_cd(
    t: np.ndarray,
    altitude: np.ndarray,
    *,
    mass_kg: float,
    ref_diameter_m: float,
    accel_axial: tuple[np.ndarray, np.ndarray] | None = None,
    site_elevation_m: float = 0.0,
    temperature_offset_k: float = 0.0,
    smooth_s: float = 0.6,
    skip_after_burnout_s: float = 0.4,
    min_speed: float = 15.0,
    mach_bin_width: float = 0.05,
    min_samples_per_bin: int = 20,
    rel_sigma_mass: float | None = None,
    rel_sigma_area: float | None = None,
    rel_sigma_rho: float | None = None,
    n_bootstrap: int = 400,
    seed: int = 0,
) -> CdResult:
    """Cd per Mach bin from barometer altitude (``t``, ``altitude`` [m above ground]) and optionally the axial specific force
    ``accel_axial = (t_acc, f_x)`` [m/s^2, along the nose, negative while decelerating].

    Without ``accel_axial`` the deceleration comes from the second derivative of the smoothed altitude (vertical-flight
    assumption); with it, a_D = -f_x and gravity drops out. The relative one-sigma uncertainties of the mass, reference area
    and air density turn into the systematic error; if not given, no systematic error is reported (it is not zero).
    """
    t = np.asarray(t, dtype=float)
    h = np.asarray(altitude, dtype=float)
    if not (t.ndim == h.ndim == 1 and len(t) == len(h)) or len(t) < 30:
        raise ConfigError("estimate_cd: need time and altitude arrays of equal length (at least 30 samples)")
    if np.any(np.diff(t) <= 0) or not (np.all(np.isfinite(t)) and np.all(np.isfinite(h))):
        raise ConfigError("estimate_cd: time must be strictly increasing and the data finite")
    if mass_kg <= 0 or ref_diameter_m <= 0:
        raise ConfigError("estimate_cd: mass_kg and ref_diameter_m must be > 0")
    notes: list[str] = []
    dt = float(np.median(np.diff(t)))
    if np.max(np.diff(t)) > 3 * dt:
        notes.append(
            f"the log has gaps up to {np.max(np.diff(t)):.2f} s; they were bridged by linear interpolation"
        )
    tu, hu = _uniform(t, h, dt)
    win = _odd(max(int(round(smooth_s / dt)), 7))
    if win >= len(tu):
        raise ConfigError("estimate_cd: the log is shorter than the smoothing window")
    v = savgol_filter(hu, win, 3, deriv=1, delta=dt)
    a_z = savgol_filter(hu, win, 3, deriv=2, delta=dt)
    hs = savgol_filter(hu, win, 3)

    i_peak = int(np.argmax(v))
    i_apogee = int(np.argmax(hs))
    if i_apogee <= i_peak + 5:
        raise ConfigError(
            "estimate_cd: no coast phase found (the speed peak is not followed by a climb to apogee)"
        )
    i0 = i_peak + int(math.ceil(skip_after_burnout_s / dt))
    above = np.nonzero(v[i0:i_apogee] >= min_speed)[0]
    if len(above) < 10:
        raise ConfigError("estimate_cd: fewer than 10 coast samples above the minimum speed")
    sel = np.zeros(len(tu), dtype=bool)
    sel[i0 + above[0] : i0 + above[-1] + 1] = True
    if accel_axial is not None:
        t_acc, f_x = (np.asarray(a, dtype=float) for a in accel_axial)
        dt_a = float(np.median(np.diff(t_acc)))
        win_a = _odd(max(int(round(smooth_s / dt_a)), 5))
        f_s = savgol_filter(f_x, min(win_a, len(f_x) - (1 - len(f_x) % 2)), 2)
        a_d = -np.interp(tu, t_acc, f_s)
        method = "accelerometer (a_D = -f_x)"
    else:
        a_d = -a_z - G0
        method = "barometer (vertical flight)"
    speed = np.abs(v)
    atm = ISAAtmosphere(temperature_offset_k)
    states = [atm.at(site_elevation_m + float(x)) for x in hs[sel]]
    rho = np.array([s.density for s in states])
    mach = speed[sel] / np.array([s.speed_of_sound for s in states])
    area = math.pi * ref_diameter_m**2 / 4.0
    cd = np.asarray(solve_cd(mass_kg, rho, speed[sel], area, a_d[sel]), dtype=float)
    if cd.shape != mach.shape:
        raise ConfigError("estimate_cd: solve_cd must return one value per sample")
    sys_rel = None
    if None not in (rel_sigma_mass, rel_sigma_area, rel_sigma_rho):
        assert rel_sigma_mass is not None and rel_sigma_area is not None and rel_sigma_rho is not None
        sys_rel = math.sqrt(rel_sigma_mass**2 + rel_sigma_area**2 + rel_sigma_rho**2)
    else:
        notes.append(
            "no systematic uncertainty reported: give rel_sigma_mass, rel_sigma_area and rel_sigma_rho"
        )

    rng = np.random.default_rng(seed)
    block = max(int(round(smooth_s / dt)), 1)  # neighbouring samples are correlated by the smoothing window
    edges = np.arange(
        math.floor(mach.min() / mach_bin_width) * mach_bin_width, mach.max() + mach_bin_width, mach_bin_width
    )
    bins: list[CdBin] = []
    for lo, hi in zip(edges[:-1], edges[1:], strict=True):
        m = (mach >= lo) & (mach < hi)
        if int(m.sum()) < min_samples_per_bin:
            if m.any():
                notes.append(f"Mach {lo:.2f}-{hi:.2f} has only {int(m.sum())} samples: not reported")
            continue
        c = cd[m]
        blk = max(
            min(block, len(c) // 5), 1
        )  # at least ~5 blocks per bin, otherwise the resampling has nothing to vary
        n_blocks = max(len(c) // blk, 1)
        means = np.empty(n_bootstrap)
        for b in range(n_bootstrap):
            starts = rng.integers(0, len(c) - blk + 1, size=n_blocks)
            means[b] = float(np.mean(np.concatenate([c[s : s + blk] for s in starts])))
        mean_cd = float(np.mean(c))
        bins.append(
            CdBin(
                float(lo),
                float(hi),
                float(np.mean(mach[m])),
                mean_cd,
                float(np.std(means, ddof=1)),
                None if sys_rel is None else abs(mean_cd) * sys_rel,
                int(m.sum()),
            )
        )
    if not bins:
        raise ConfigError(
            "estimate_cd: no Mach bin has enough samples; widen mach_bin_width or lower min_samples_per_bin"
        )
    if len(bins) == 1:
        notes.append(
            "the coast covers a single Mach bin: this is one drag value at that Mach number, not a curve"
        )
    return CdResult(bins, (float(tu[sel][0]), float(tu[sel][-1])), method, int(sel.sum()), notes)
