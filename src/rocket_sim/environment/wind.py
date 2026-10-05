"""Wind models. ``wind.at(t, z) -> (wx, wy, wz)`` is the velocity of the AIR in the launch
frame (ENU) [m/s] at time t [s] and altitude z above the launch site [m].

The vehicle's aerodynamic velocity is always ``v_vehicle - wind`` (see physics.dynamics).

Direction convention (meteorological): ``direction_from_deg`` is the compass bearing the wind
blows FROM, clockwise from North. A 5 m/s wind from the West (270 deg) blows toward +East:
wind = (+5, 0, 0).

Models
------
ConstantWind, ProfileWind (piecewise-linear in altitude on u/v components), PowerLawWind
(speed ~ (z/z_ref)^alpha), plus stochastic additions that are *pre-generated from a seed on a
fixed time grid* so results do not depend on integrator step size:
TurbulenceWind  first-order Gauss-Markov (Ornstein-Uhlenbeck) gusts per axis (simplified
                stand-in for Dryden; time-correlated, not spatially correlated),
GustWind        deterministic "1-cos" discrete gusts,
CompositeWind   sum of the above.
"""

from __future__ import annotations

import bisect
import math
from abc import ABC, abstractmethod

import numpy as np

from ..errors import ConfigError

WindVec = tuple[float, float, float]


def wind_vector(speed: float, direction_from_deg: float) -> WindVec:
    """ENU air-velocity vector for a wind of ``speed`` blowing from ``direction_from_deg``."""
    b = math.radians(direction_from_deg)
    return (-speed * math.sin(b), -speed * math.cos(b), 0.0)


class Wind(ABC):
    name = "wind"

    @abstractmethod
    def at(self, t: float, z: float) -> WindVec:
        """Air velocity in the launch frame at time t and height z above the launch site."""


class NoWind(Wind):
    name = "none"

    def at(self, t: float, z: float) -> WindVec:
        return (0.0, 0.0, 0.0)


class ConstantWind(Wind):
    name = "constant"

    def __init__(self, speed: float, direction_from_deg: float) -> None:
        if speed < 0:
            raise ConfigError("wind speed must be >= 0")
        self.vec = wind_vector(speed, direction_from_deg)

    def at(self, t: float, z: float) -> WindVec:
        return self.vec


class ProfileWind(Wind):
    """Rows of (altitude AGL [m], speed [m/s], direction_from [deg]); u/v linearly interpolated.

    Below the first row / above the last row the wind is held constant at the end values.
    """

    name = "profile"

    def __init__(self, rows: list[tuple[float, float, float]]) -> None:
        if len(rows) < 1:
            raise ConfigError("wind profile needs at least one row")
        rows = sorted(rows)
        self.z = [r[0] for r in rows]
        if any(b <= a for a, b in zip(self.z, self.z[1:], strict=False)):
            raise ConfigError("wind profile altitudes must be strictly increasing")
        vecs = [wind_vector(r[1], r[2]) for r in rows]
        self.u = [v[0] for v in vecs]
        self.v = [v[1] for v in vecs]
        if any(r[1] < 0 for r in rows):
            raise ConfigError("wind profile speeds must be >= 0")

    def at(self, t: float, z: float) -> WindVec:
        zs = self.z
        if z <= zs[0]:
            return (self.u[0], self.v[0], 0.0)
        if z >= zs[-1]:
            return (self.u[-1], self.v[-1], 0.0)
        i = bisect.bisect_right(zs, z) - 1
        f = (z - zs[i]) / (zs[i + 1] - zs[i])
        return (
            self.u[i] + f * (self.u[i + 1] - self.u[i]),
            self.v[i] + f * (self.v[i + 1] - self.v[i]),
            0.0,
        )


class PowerLawWind(Wind):
    """Boundary-layer power law: speed = ref_speed * (max(z, z_min)/ref_height)^alpha."""

    name = "power_law"

    def __init__(
        self,
        ref_speed: float,
        direction_from_deg: float,
        ref_height: float = 10.0,
        exponent: float = 1.0 / 7.0,
        z_min: float = 1.0,
    ) -> None:
        if ref_speed < 0 or ref_height <= 0 or z_min <= 0:
            raise ConfigError("power-law wind: speed >= 0, heights > 0 required")
        self.ref_speed, self.ref_height, self.alpha, self.z_min = (
            ref_speed,
            ref_height,
            exponent,
            z_min,
        )
        self.dir = direction_from_deg

    def at(self, t: float, z: float) -> WindVec:
        s = self.ref_speed * (max(z, self.z_min) / self.ref_height) ** self.alpha
        return wind_vector(s, self.dir)


class TurbulenceWind(Wind):
    """Gauss-Markov gusts: each axis an OU process with std ``sigma`` and time constant ``tau``.

    Pre-generated at ``grid_dt`` over ``horizon`` seconds from ``seed`` and linearly
    interpolated, so the field is identical for every integrator step size.
    Vertical component is scaled by ``vertical_ratio`` (default 0.5).
    """

    name = "turbulence"

    def __init__(
        self,
        sigma: float,
        tau: float,
        seed: int,
        horizon: float = 900.0,
        grid_dt: float = 0.05,
        vertical_ratio: float = 0.5,
    ) -> None:
        if sigma < 0 or tau <= 0 or horizon <= 0 or grid_dt <= 0:
            raise ConfigError("turbulence: sigma >= 0, tau/horizon/grid_dt > 0 required")
        rng = np.random.default_rng(seed)
        n = int(horizon / grid_dt) + 2
        a = math.exp(-grid_dt / tau)
        b = sigma * math.sqrt(1.0 - a * a)
        noise = rng.standard_normal((n, 3))
        series = np.empty((n, 3))
        series[0] = sigma * rng.standard_normal(3)
        for i in range(1, n):
            series[i] = a * series[i - 1] + b * noise[i]
        series[:, 2] *= vertical_ratio
        self._series = series
        self._dt = grid_dt
        self._n = n

    def at(self, t: float, z: float) -> WindVec:
        x = max(t, 0.0) / self._dt
        i = int(x)
        if i >= self._n - 1:
            i = self._n - 2
            x = float(i)
        f = x - i
        s0, s1 = self._series[i], self._series[i + 1]
        return (
            float(s0[0] + f * (s1[0] - s0[0])),
            float(s0[1] + f * (s1[1] - s0[1])),
            float(s0[2] + f * (s1[2] - s0[2])),
        )


class GustWind(Wind):
    """Discrete '1-cos' gust: amplitude vector ramps up and down over ``duration`` from ``start``."""

    name = "gust"

    def __init__(self, start: float, duration: float, speed: float, direction_from_deg: float) -> None:
        if duration <= 0 or speed < 0:
            raise ConfigError("gust: duration > 0 and speed >= 0 required")
        self.start, self.duration = start, duration
        self.vec = wind_vector(speed, direction_from_deg)

    def at(self, t: float, z: float) -> WindVec:
        if t <= self.start or t >= self.start + self.duration:
            return (0.0, 0.0, 0.0)
        s = 0.5 * (1.0 - math.cos(2.0 * math.pi * (t - self.start) / self.duration))
        return (self.vec[0] * s, self.vec[1] * s, 0.0)


class CompositeWind(Wind):
    name = "composite"

    def __init__(self, parts: list[Wind]) -> None:
        self.parts = parts

    def at(self, t: float, z: float) -> WindVec:
        wx = wy = wz = 0.0
        for p in self.parts:
            a, b, c = p.at(t, z)
            wx += a
            wy += b
            wz += c
        return (wx, wy, wz)
