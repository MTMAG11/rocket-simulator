"""Component-based airframe geometry: sections, control surfaces and the ``Assembly``.

The rocket is a stack of axisymmetric sections (nose, body tubes, transitions / boat-tails) plus a fin set
and optional movable control-surface sets. Positions are measured AFT from the nose tip [m].

What the assembly provides to the aerodynamic models: wetted and planform areas, local radius, and the
Barrowman normal-force slopes / centres of pressure of the axisymmetric sections. Mass and inertia are
handled by ``vehicle.mass`` (each section can carry a mass; the CG and inertia tensor are COMPUTED from the
components, not typed in).

Limitations: axisymmetric sections only (no pods, strakes, launch lugs); fins are a single trapezoidal set
(plus control sets); section interiors are thin shells unless a measured inertia is supplied.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..errors import ConfigError
from .geometry import (
    NOSE_SHAPES,
    NOSE_WETTED_FACTOR,
    NOSE_XCP_FRACTION,
    BodyTube,
    FinSet,
    NoseCone,
)

SECTION_KINDS = ("nose", "body", "transition")


@dataclass(frozen=True)
class Section:
    """One axisymmetric station of the airframe, stacked from the nose tip aft.

    kind      nose | body | transition (a transition with d_aft < d_fore is a boat-tail)
    x         start position aft of the nose tip [m]
    length    axial length [m]
    d_fore    diameter at the forward end [m] (0 for a pointed nose)
    d_aft     diameter at the aft end [m]
    shape     nose profile (nose only)
    mass      structural mass of the section [kg] (0 = none; a lumped airframe mass can be given instead)
    cg_frac   CG position as a fraction of the length from the forward end (default 0.5)
    """

    kind: str
    x: float
    length: float
    d_fore: float
    d_aft: float
    shape: str = "ogive"
    mass: float = 0.0
    cg_frac: float = 0.5

    def __post_init__(self) -> None:
        if self.kind not in SECTION_KINDS:
            raise ConfigError(f"section kind must be one of {SECTION_KINDS}, got {self.kind!r}")
        if self.length <= 0 or self.x < 0:
            raise ConfigError(f"section {self.kind}: length must be > 0 and x >= 0")
        if self.d_fore < 0 or self.d_aft < 0 or (self.d_fore == 0 and self.d_aft == 0):
            raise ConfigError(f"section {self.kind}: diameters must be >= 0 and not both zero")
        if self.kind == "nose":
            if self.shape not in NOSE_SHAPES:
                raise ConfigError(f"nose shape must be one of {NOSE_SHAPES}, got {self.shape!r}")
            if self.d_fore != 0.0:
                raise ConfigError("a nose section must start from a point (d_fore = 0)")
        elif self.kind == "body" and abs(self.d_fore - self.d_aft) > 1e-9:
            raise ConfigError("a body section has a constant diameter (use a transition to change it)")
        elif self.kind == "transition" and abs(self.d_fore - self.d_aft) < 1e-9:
            raise ConfigError("a transition must change diameter")
        if self.mass < 0 or not 0.0 <= self.cg_frac <= 1.0:
            raise ConfigError(f"section {self.kind}: mass >= 0 and cg_frac in [0, 1] required")

    @property
    def x_end(self) -> float:
        return self.x + self.length

    @property
    def x_cg(self) -> float:
        return self.x + self.cg_frac * self.length

    @property
    def mean_radius(self) -> float:
        return 0.25 * (self.d_fore + self.d_aft)

    def wetted_area(self) -> float:
        r1, r2 = 0.5 * self.d_fore, 0.5 * self.d_aft
        if self.kind == "nose":
            cone = math.pi * r2 * math.sqrt(r2 * r2 + self.length**2)
            return cone * NOSE_WETTED_FACTOR[self.shape]
        slant = math.sqrt((r2 - r1) ** 2 + self.length**2)
        return math.pi * (r1 + r2) * slant

    def planform_area(self) -> float:
        """Projected side area [m^2] (used for crossflow drag)."""
        if self.kind == "nose":
            return 0.5 * self.d_aft * self.length  # ogive-like: about half the bounding rectangle
        return 0.5 * (self.d_fore + self.d_aft) * self.length

    def planform_centroid(self) -> float:
        if self.kind == "nose":
            return self.x + 0.4 * self.length
        d1, d2 = self.d_fore, self.d_aft
        return self.x + self.length * (d1 + 2.0 * d2) / (3.0 * (d1 + d2))  # trapezoid centroid

    @property
    def half_angle(self) -> float:
        """Half-angle of the surface relative to the axis [rad] (> 0 expanding aft, < 0 boat-tail)."""
        return math.atan2(0.5 * (self.d_aft - self.d_fore), self.length)


@dataclass(frozen=True)
class ControlSurfaceSet:
    """Movable fin set (canards or tail fins) with its actuator limits.

    Planform as a fin. ``roll_angle0`` is the angular position of the first fin about the nose axis,
    measured from +y_B toward +z_B; fins are evenly spaced. Positive deflection produces a force along
    n_i = (0, -sin(phi_i), cos(phi_i)) (normal to the fin plane).
    """

    count: int
    root_chord: float
    tip_chord: float
    span: float
    sweep: float
    thickness: float
    leading_edge_from_nose: float
    roll_angle0: float = 0.0
    max_deflection: float = math.radians(10.0)
    max_rate: float = math.radians(200.0)
    time_constant: float = 0.0
    delay: float = 0.0
    mass: float = 0.0  # total mass of the set [kg], located at the fin centroid

    def __post_init__(self) -> None:
        if self.count < 2:
            raise ConfigError("control_surfaces.count must be >= 2")
        if self.max_deflection <= 0 or self.max_rate <= 0 or self.time_constant < 0 or self.delay < 0:
            raise ConfigError("control-surface limits must be positive (lag/delay >= 0)")
        _ = self.fin  # validates the planform

    @property
    def fin(self) -> FinSet:
        n = self.count if self.count in (2, 3, 4, 5, 6, 8) else 4  # planform maths only
        return FinSet(
            n,
            self.root_chord,
            self.tip_chord,
            self.span,
            self.sweep,
            self.thickness,
            self.leading_edge_from_nose,
        )

    def fin_angles(self) -> list[float]:
        return [self.roll_angle0 + 2.0 * math.pi * i / self.count for i in range(self.count)]


class Assembly:
    """The airframe as an ordered stack of sections plus fins and control surfaces."""

    def __init__(
        self,
        sections: list[Section],
        fins: FinSet,
        controls: list[ControlSurfaceSet] | None = None,
        ref_diameter: float | None = None,
    ) -> None:
        if not sections:
            raise ConfigError("rocket needs at least one section (a nose and a body)")
        secs = sorted(sections, key=lambda s: s.x)
        if secs[0].kind != "nose" or secs[0].x != 0.0:
            raise ConfigError("the first section must be the nose, starting at x = 0")
        for a, b in zip(secs, secs[1:]):
            if abs(b.x - a.x_end) > 1e-6:
                raise ConfigError(
                    f"sections must be contiguous: {a.kind} ends at {a.x_end:.4f} m "
                    f"but {b.kind} starts at {b.x:.4f} m"
                )
            if abs(b.d_fore - a.d_aft) > 1e-6:
                raise ConfigError(
                    f"diameter discontinuity between {a.kind} (d_aft {a.d_aft}) and {b.kind} (d_fore {b.d_fore})"
                )
            if b.kind == "nose":
                raise ConfigError("only the first section can be a nose")
        self.sections = tuple(secs)
        self.fins = fins
        self.controls = tuple(controls or [])
        self.length = secs[-1].x_end
        self.max_diameter = max(max(s.d_fore, s.d_aft) for s in secs)
        self.ref_diameter = ref_diameter if ref_diameter is not None else self.max_diameter
        if self.ref_diameter <= 0:
            raise ConfigError("reference diameter must be > 0")
        self.ref_area = math.pi * (0.5 * self.ref_diameter) ** 2
        self.base_diameter = secs[-1].d_aft
        if fins.count:
            if fins.leading_edge_from_nose + fins.root_chord > self.length + 1e-9:
                raise ConfigError("fins: root chord extends beyond the end of the airframe")
            if fins.leading_edge_from_nose < secs[0].length - 1e-9:
                raise ConfigError("fins cannot start on the nose cone")
        for c in self.controls:
            if c.leading_edge_from_nose + c.root_chord > self.length + 1e-9:
                raise ConfigError("control surfaces: root chord extends beyond the end of the airframe")

    @property
    def nose(self) -> Section:
        return self.sections[0]

    def local_radius(self, x: float) -> float:
        """Body radius at axial station x (linear within each section)."""
        for s in self.sections:
            if x <= s.x_end + 1e-12:
                f = min(max((x - s.x) / s.length, 0.0), 1.0)
                return 0.5 * (s.d_fore + f * (s.d_aft - s.d_fore))
        return 0.5 * self.base_diameter

    def wetted_area(self) -> float:
        return sum(s.wetted_area() for s in self.sections)

    def planform(self) -> tuple[float, float]:
        """(projected side area [m^2], its centroid aft of the nose [m])."""
        a = sum(s.planform_area() for s in self.sections)
        c = sum(s.planform_area() * s.planform_centroid() for s in self.sections) / a
        return a, c

    def section_normal_force(self) -> list[tuple[float, float, str]]:
        """Barrowman normal-force slopes and CPs of the axisymmetric sections: (CNa [1/rad], x_cp, kind).

        nose: CNa = 2 (d_aft/d_ref)^2, CP at the shape-dependent fraction of its length.
        transition: CNa = 2[(d_a/d)^2 - (d_f/d)^2] at x_T + L/3 [1 + 1/(1 + d_f/d_a)] (negative for a
        boat-tail, i.e. destabilising). Body tubes carry no normal force in Barrowman's theory.
        """
        d = self.ref_diameter
        out: list[tuple[float, float, str]] = []
        for s in self.sections:
            if s.kind == "nose":
                out.append((2.0 * (s.d_aft / d) ** 2, s.x + NOSE_XCP_FRACTION[s.shape] * s.length, "nose"))
            elif s.kind == "transition":
                cn = 2.0 * ((s.d_aft / d) ** 2 - (s.d_fore / d) ** 2)
                r = s.d_fore / s.d_aft
                out.append((cn, s.x + s.length / 3.0 * (1.0 + 1.0 / (1.0 + r)), "transition"))
        return out


def legacy_assembly(body: BodyTube, nose: NoseCone, fins: FinSet) -> Assembly:
    """Single-diameter vehicle (V1 configuration) as an Assembly: nose + one body tube."""
    d = body.diameter
    return Assembly(
        [
            Section("nose", 0.0, nose.length, 0.0, d, nose.shape),
            Section("body", nose.length, body.length - nose.length, d, d),
        ],
        fins,
    )
