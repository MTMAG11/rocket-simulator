"""Atmosphere models: temperature, pressure, density, speed of sound, viscosity vs altitude.

All models implement ``Atmosphere.at(h_msl) -> AtmState`` where ``h_msl`` is the geometric
altitude above mean sea level [m]. Air is treated as dry, calorically perfect gas.

Models
------
ISAAtmosphere          U.S. Standard Atmosphere 1976 layers to 84.852 km geopotential
                       (isothermal extrapolation above), with optional temperature offset
                       and sea-level pressure (day-specific conditions).
ExponentialAtmosphere  Simplified/fast: exponential density, linear-lapse temperature.
TableAtmosphere        Measured profile (e.g. radiosonde): T(h), p(h) interpolated; density
                       from the ideal-gas law (so the profile is thermodynamically consistent).

Source: NOAA/NASA/USAF, U.S. Standard Atmosphere 1976 (NASA-TM-X-74335).
Limitations: no humidity, no horizontal/temporal variation; ISA above 85 km is not realistic.
"""

from __future__ import annotations

import bisect
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from ..constants import (
    G0,
    GAMMA_AIR,
    P_SEA_LEVEL,
    R_AIR,
    SUTHERLAND_MU0,
    SUTHERLAND_S,
    T_SEA_LEVEL,
)
from ..errors import ConfigError

_R0_GEOPOTENTIAL = 6_356_766.0  # radius used to define geopotential height in the 1976 ISA


@dataclass(frozen=True)
class AtmState:
    temperature: float  # [K]
    pressure: float  # [Pa]
    density: float  # [kg/m^3]
    speed_of_sound: float  # [m/s]
    viscosity: float  # dynamic viscosity [Pa s]


def sutherland_viscosity(temperature: float) -> float:
    return SUTHERLAND_MU0 * temperature**1.5 / (temperature + SUTHERLAND_S)


def speed_of_sound(temperature: float) -> float:
    return math.sqrt(GAMMA_AIR * R_AIR * temperature)


def _state(temperature: float, pressure: float) -> AtmState:
    return AtmState(
        temperature,
        pressure,
        pressure / (R_AIR * temperature),
        speed_of_sound(temperature),
        sutherland_viscosity(temperature),
    )


class Atmosphere(ABC):
    name = "atmosphere"

    @abstractmethod
    def at(self, h_msl: float) -> AtmState:
        """Atmospheric state at geometric altitude h_msl [m above MSL]."""

    def pressure_to_altitude(self, pressure: float) -> float:
        """Invert p(h) by bisection (used by barometer altimetry)."""
        lo, hi = -2_000.0, 80_000.0
        if not self.at(hi).pressure <= pressure <= self.at(lo).pressure:
            raise ValueError(f"pressure {pressure} Pa outside atmosphere table range")
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if self.at(mid).pressure > pressure:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)


class ISAAtmosphere(Atmosphere):
    """1976 U.S. Standard Atmosphere with optional uniform temperature offset."""

    name = "isa"
    # (base geopotential height [m], lapse rate [K/m])
    _LAYERS = (
        (0.0, -0.0065),
        (11_000.0, 0.0),
        (20_000.0, 0.001),
        (32_000.0, 0.0028),
        (47_000.0, 0.0),
        (51_000.0, -0.0028),
        (71_000.0, -0.002),
    )

    def __init__(
        self,
        temperature_offset: float = 0.0,
        sea_level_pressure: float = P_SEA_LEVEL,
        sea_level_temperature: float = T_SEA_LEVEL,
    ) -> None:
        if sea_level_pressure <= 0 or sea_level_temperature + temperature_offset <= 0:
            raise ConfigError("atmosphere: pressure and temperature must be positive")
        self.dT = temperature_offset
        self.p0 = sea_level_pressure
        self.t0 = sea_level_temperature + temperature_offset
        # precompute base temperature / pressure of each layer
        self._base_t: list[float] = []
        self._base_p: list[float] = []
        t, p = self.t0, self.p0
        for i, (h_b, lapse) in enumerate(self._LAYERS):
            self._base_t.append(t)
            self._base_p.append(p)
            if i + 1 < len(self._LAYERS):
                dh = self._LAYERS[i + 1][0] - h_b
                p = p * self._p_ratio(t, lapse, dh)
                t = t + lapse * dh
        self._bases = [layer[0] for layer in self._LAYERS]

    @staticmethod
    def _p_ratio(t_base: float, lapse: float, dh: float) -> float:
        if abs(lapse) < 1e-12:
            return math.exp(-G0 * dh / (R_AIR * t_base))
        t = t_base + lapse * dh
        return (t / t_base) ** (-G0 / (R_AIR * lapse))

    def pressure_to_altitude(self, pressure: float) -> float:
        """Closed-form inverse of p(h) for the layered ISA (fast; used by baro altimetry)."""
        if pressure <= 0:
            raise ValueError("pressure must be positive")
        i = len(self._LAYERS) - 1
        while i > 0 and pressure > self._base_p[i]:
            i -= 1
        h_b, lapse = self._LAYERS[i]
        t_b, p_b = self._base_t[i], self._base_p[i]
        if abs(lapse) < 1e-12:
            hg = h_b - (R_AIR * t_b / G0) * math.log(pressure / p_b)
        else:
            hg = h_b + (t_b / lapse) * ((pressure / p_b) ** (-R_AIR * lapse / G0) - 1.0)
        return _R0_GEOPOTENTIAL * hg / (_R0_GEOPOTENTIAL - hg)

    def at(self, h_msl: float) -> AtmState:
        # geometric -> geopotential height
        hg = _R0_GEOPOTENTIAL * h_msl / (_R0_GEOPOTENTIAL + h_msl)
        i = max(0, bisect.bisect_right(self._bases, hg) - 1)
        h_b, lapse = self._LAYERS[i]
        dh = hg - h_b
        t = self._base_t[i] + lapse * dh
        p = self._base_p[i] * self._p_ratio(self._base_t[i], lapse, dh)
        return _state(t, p)


class ExponentialAtmosphere(Atmosphere):
    """Simplified atmosphere: rho = rho0 exp(-h/H); T from a 6.5 K/km lapse down to 216.65 K.

    Faster than ISA and adequate for fast-mode dataset generation (density error of a few
    percent below 11 km, growing above). Pressure is derived as p = rho R T.
    """

    name = "exponential"

    def __init__(self, rho0: float = 1.225, scale_height: float = 8_400.0, t0: float = T_SEA_LEVEL) -> None:
        if rho0 <= 0 or scale_height <= 0 or t0 <= 0:
            raise ConfigError("exponential atmosphere parameters must be positive")
        self.rho0, self.H, self.t0 = rho0, scale_height, t0

    def at(self, h_msl: float) -> AtmState:
        t = max(self.t0 - 0.0065 * h_msl, 216.65)
        rho = self.rho0 * math.exp(-h_msl / self.H)
        return AtmState(t, rho * R_AIR * t, rho, speed_of_sound(t), sutherland_viscosity(t))


class TableAtmosphere(Atmosphere):
    """Measured/custom profile: arrays of altitude [m], temperature [K], pressure [Pa].

    Temperature is interpolated linearly, ln(pressure) linearly in altitude (exact for an
    isothermal or constant-lapse layer between samples up to a small error). Queries outside
    the table are an error (never silently extrapolated).
    """

    name = "table"

    def __init__(self, altitude: list[float], temperature: list[float], pressure: list[float]) -> None:
        n = len(altitude)
        if n < 2 or len(temperature) != n or len(pressure) != n:
            raise ConfigError("atmosphere table needs >=2 rows with equal-length columns")
        if any(b <= a for a, b in zip(altitude, altitude[1:], strict=False)):
            raise ConfigError("atmosphere table altitudes must be strictly increasing")
        if min(temperature) <= 0 or min(pressure) <= 0:
            raise ConfigError("atmosphere table temperature/pressure must be positive")
        self.h = list(altitude)
        self.t = list(temperature)
        self.lnp = [math.log(p) for p in pressure]

    def at(self, h_msl: float) -> AtmState:
        if h_msl < self.h[0] - 1e-6 or h_msl > self.h[-1] + 1e-6:
            raise ValueError(
                f"altitude {h_msl:.0f} m outside measured atmosphere table "
                f"[{self.h[0]:.0f}, {self.h[-1]:.0f}] m"
            )
        i = min(max(bisect.bisect_right(self.h, h_msl) - 1, 0), len(self.h) - 2)
        f = (h_msl - self.h[i]) / (self.h[i + 1] - self.h[i])
        t = self.t[i] + f * (self.t[i + 1] - self.t[i])
        p = math.exp(self.lnp[i] + f * (self.lnp[i + 1] - self.lnp[i]))
        return _state(t, p)
