"""Terrain: ground height relative to the launch-site elevation, as a function of (x, y).

FlatTerrain   constant height (0 = same elevation as the pad).
SlopeTerrain  planar slope.
GridTerrain   bilinear interpolation of a regular grid of heights (e.g. from a DEM).
Heights are in the launch frame (z up, pad at z=0). Queries outside a grid clamp to the edge
value (documented behaviour; keep the grid larger than the expected flight envelope).
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod

import numpy as np

from ..errors import ConfigError


class Terrain(ABC):
    name = "terrain"

    @abstractmethod
    def height(self, x: float, y: float) -> float:
        """Ground height [m] above the pad plane at launch-frame position (x, y)."""


class FlatTerrain(Terrain):
    name = "flat"

    def __init__(self, height: float = 0.0) -> None:
        self.h = height

    def height(self, x: float, y: float) -> float:
        return self.h


class SlopeTerrain(Terrain):
    name = "slope"

    def __init__(self, slope_deg: float, direction_deg: float) -> None:
        """Ground rises by tan(slope) per metre toward bearing ``direction_deg`` (cw from N)."""
        if not 0.0 <= slope_deg < 60.0:
            raise ConfigError("terrain slope must be in [0, 60) degrees")
        g = math.tan(math.radians(slope_deg))
        b = math.radians(direction_deg)
        self.gx, self.gy = g * math.sin(b), g * math.cos(b)

    def height(self, x: float, y: float) -> float:
        return self.gx * x + self.gy * y


class GridTerrain(Terrain):
    name = "grid"

    def __init__(self, x: list[float], y: list[float], heights: list[list[float]]) -> None:
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.z = np.asarray(heights, dtype=float)
        if self.z.shape != (len(self.y), len(self.x)):
            raise ConfigError("terrain grid heights must have shape (len(y), len(x))")
        if len(self.x) < 2 or len(self.y) < 2:
            raise ConfigError("terrain grid needs at least 2 points per axis")
        if np.any(np.diff(self.x) <= 0) or np.any(np.diff(self.y) <= 0):
            raise ConfigError("terrain grid axes must be strictly increasing")

    def height(self, x: float, y: float) -> float:
        xs, ys = self.x, self.y
        xc = min(max(x, xs[0]), xs[-1])
        yc = min(max(y, ys[0]), ys[-1])
        i = min(int(np.searchsorted(xs, xc, side="right")) - 1, len(xs) - 2)
        j = min(int(np.searchsorted(ys, yc, side="right")) - 1, len(ys) - 2)
        i, j = max(i, 0), max(j, 0)
        fx = (xc - xs[i]) / (xs[i + 1] - xs[i])
        fy = (yc - ys[j]) / (ys[j + 1] - ys[j])
        z = self.z
        return float(
            z[j, i] * (1 - fx) * (1 - fy)
            + z[j, i + 1] * fx * (1 - fy)
            + z[j + 1, i] * (1 - fx) * fy
            + z[j + 1, i + 1] * fx * fy
        )
