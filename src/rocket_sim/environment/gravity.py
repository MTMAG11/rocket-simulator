"""Gravity models. Gravity always acts along -z of the launch frame (flat-Earth assumption).

ConstantGravity       g = const (default 9.80665 m/s^2)
InverseSquareGravity  g(h) = GM / (R + h)^2, h = altitude above MSL
Neglected: latitude dependence (~0.5%), J2, centrifugal term, direction change over range.
For ranges < ~50 km the direction error is < 0.5 deg.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..constants import G0, GM_EARTH, R_EARTH
from ..errors import ConfigError


class Gravity(ABC):
    name = "gravity"

    @abstractmethod
    def g(self, h_msl: float) -> float:
        """Gravitational acceleration magnitude [m/s^2] at altitude h_msl [m]."""


class ConstantGravity(Gravity):
    name = "constant"

    def __init__(self, g0: float = G0) -> None:
        if g0 < 0:
            raise ConfigError("gravity: g must be >= 0")
        self.g0 = g0

    def g(self, h_msl: float) -> float:
        return self.g0


class InverseSquareGravity(Gravity):
    name = "inverse_square"

    def __init__(self, gm: float = GM_EARTH, radius: float = R_EARTH) -> None:
        if gm <= 0 or radius <= 0:
            raise ConfigError("gravity: GM and radius must be positive")
        self.gm = gm
        self.radius = radius

    def g(self, h_msl: float) -> float:
        r = self.radius + h_msl
        return self.gm / (r * r)
