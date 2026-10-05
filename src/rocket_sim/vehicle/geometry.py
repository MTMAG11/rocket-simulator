"""Rocket geometry. Axial positions are measured AFT from the nose tip [m] (OpenRocket style).

Only axisymmetric body + nose + one trapezoidal fin set (+ optional parachute) is modelled.
Boat-tails, transitions, multiple fin sets, canards and launch lugs are NOT modelled; use
``extra_cd`` in the aero config to lump protuberance drag.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..errors import ConfigError

NOSE_SHAPES = ("cone", "ogive", "parabolic", "elliptical")

# Barrowman (1967) nose-cone centre of pressure, as a fraction of nose length from the tip.
# Tangent ogive 0.466, cone 2/3, parabola 1/2, ellipse 1/3.
NOSE_XCP_FRACTION = {"cone": 2.0 / 3.0, "ogive": 0.466, "parabolic": 0.5, "elliptical": 1.0 / 3.0}
# Wetted-area factors relative to a cone of the same length (approximate).
NOSE_WETTED_FACTOR = {"cone": 1.0, "ogive": 1.04, "parabolic": 1.03, "elliptical": 1.12}


@dataclass(frozen=True)
class NoseCone:
    shape: str
    length: float  # [m]

    def __post_init__(self) -> None:
        if self.shape not in NOSE_SHAPES:
            raise ConfigError(f"nose.shape must be one of {NOSE_SHAPES}, got {self.shape!r}")
        if self.length <= 0:
            raise ConfigError(f"nose.length_m must be > 0, got {self.length}")


@dataclass(frozen=True)
class FinSet:
    """Trapezoidal fin set. ``leading_edge_from_nose`` is the root-chord leading edge position."""

    count: int
    root_chord: float  # [m]
    tip_chord: float  # [m]
    span: float  # [m] radial height beyond the body surface
    sweep: float  # [m] axial offset of tip leading edge behind root leading edge
    thickness: float  # [m]
    leading_edge_from_nose: float  # [m]

    def __post_init__(self) -> None:
        if self.count not in (0, 2, 3, 4, 5, 6, 8):
            raise ConfigError(f"fins.count must be 0, 2, 3, 4, 5, 6 or 8, got {self.count}")
        if self.count == 0:
            return
        if self.root_chord <= 0 or self.tip_chord < 0 or self.span <= 0 or self.thickness < 0:
            raise ConfigError("fins: root_chord, span must be > 0; tip_chord, thickness >= 0")
        if self.sweep < 0:
            raise ConfigError("fins.sweep_m must be >= 0")

    @property
    def planform_area(self) -> float:
        """Area of ONE fin [m^2]."""
        return 0.5 * (self.root_chord + self.tip_chord) * self.span

    @property
    def mean_chord(self) -> float:
        return 0.5 * (self.root_chord + self.tip_chord)

    @property
    def leading_edge_sweep_angle(self) -> float:
        """Leading-edge sweep angle [rad] (0 = unswept)."""
        return math.atan2(self.sweep, self.span)

    @property
    def midchord_length(self) -> float:
        """Length of the mid-chord line (Barrowman's L_f) [m]."""
        dx = self.sweep + 0.5 * (self.tip_chord - self.root_chord)
        return math.hypot(dx, self.span)


@dataclass(frozen=True)
class BodyTube:
    diameter: float  # [m]
    length: float  # [m] total vehicle length (nose tip to base)

    def __post_init__(self) -> None:
        if self.diameter <= 0:
            raise ConfigError(f"body_diameter_m must be > 0, got {self.diameter}")
        if self.length <= self.diameter:
            raise ConfigError(f"body_length_m ({self.length}) must exceed body_diameter_m ({self.diameter})")

    @property
    def radius(self) -> float:
        return 0.5 * self.diameter

    @property
    def reference_area(self) -> float:
        return math.pi * self.radius**2


@dataclass(frozen=True)
class Parachute:
    """Single parachute. Drag area = cd * area. Deployment trigger evaluated by the simulator."""

    cd: float
    diameter: float  # [m] inflated canopy diameter
    trigger: str = "apogee"  # "apogee" | "altitude"
    altitude: float = 0.0  # [m AGL] when trigger == "altitude" (descending)
    delay: float = 0.0  # [s] after trigger (ejection delay)
    inflation_time: float = 0.5  # [s] linear ramp of drag area
    attach: float = 0.0  # [m] aft of nose tip: where the shock cord loads the airframe (6-DOF)

    def __post_init__(self) -> None:
        if self.cd <= 0 or self.diameter <= 0:
            raise ConfigError("parachute cd and diameter must be > 0")
        if self.trigger not in ("apogee", "altitude"):
            raise ConfigError("parachute trigger must be 'apogee' or 'altitude'")
        if self.delay < 0 or self.inflation_time < 0:
            raise ConfigError("parachute delay/inflation_time must be >= 0")

    @property
    def area(self) -> float:
        return math.pi * (0.5 * self.diameter) ** 2
