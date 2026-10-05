"""Mass, centre of mass and inertia tensor as a function of remaining propellant.

Axial positions are measured aft from the nose tip. Components are rigid bodies with their own inertia
tensor about their own CG; the vehicle CG and the FULL 3x3 inertia tensor about the CG are COMPUTED from the
components (parallel-axis theorem), not typed in:

    I_cg = sum_i [ I_i + m_i ( |d_i|^2 E - d_i d_i^T ) ],      d_i = r_i - r_cg   (body axes, x forward)

Body axes: x forward along the nose axis, y right, z down (docs/frames.md); a component at axial station
x_aft (aft of the nose) and lateral offsets (y, z) has body-frame position (-x_aft, y, z) relative to the nose.

Assumptions: the propellant is a solid uniform cylinder of the motor's diameter and length whose CG stays at the
motor centre while it burns (grain regression and CG shift are NOT modelled); products of inertia are zero for
axisymmetric components and non-zero only if components sit off-axis or the user supplies them. The dynamics
integrates Euler's equations with the full tensor; for the (default) axisymmetric case the tensor is diagonal
with Iyy = Izz and the solution reduces to the V1 equations exactly.
``thin_tube_inertia`` is an ESTIMATE for components of unknown inertia: replace it with measured/CAD values.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..errors import ConfigError


@dataclass(frozen=True)
class MassComponent:
    name: str
    mass: float  # [kg]
    x_cg: float  # [m] aft of nose tip
    ixx: float = 0.0  # [kg m^2] about own CG, roll axis
    iyy: float = 0.0  # [kg m^2] about own CG, y axis
    y: float = 0.0  # lateral offset of the CG from the axis, body y (right) [m]
    z: float = 0.0  # lateral offset of the CG from the axis, body z (down) [m]
    izz: float | None = None  # default: equal to iyy (axisymmetric)
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    def __post_init__(self) -> None:
        if self.mass < 0:
            raise ConfigError(f"mass component {self.name!r}: mass must be >= 0")
        if self.ixx < 0 or self.iyy < 0 or (self.izz is not None and self.izz < 0):
            raise ConfigError(f"mass component {self.name!r}: principal inertias must be >= 0")

    @property
    def izz_eff(self) -> float:
        return self.iyy if self.izz is None else self.izz


@dataclass(frozen=True)
class MassProps:
    """Instantaneous mass properties about the CG in body axes (full symmetric tensor)."""

    mass: float
    x_cg: float  # [m] aft of nose
    ixx: float
    iyy: float
    y_cg: float = 0.0  # CG lateral offsets from the nose axis (body y, z) [m]
    z_cg: float = 0.0
    izz: float | None = None  # None: axisymmetric (= iyy)
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    @property
    def izz_eff(self) -> float:
        return self.iyy if self.izz is None else self.izz

    @property
    def is_axisymmetric(self) -> bool:
        return (
            self.izz is None
            and self.ixy == 0.0
            and self.ixz == 0.0
            and self.iyz == 0.0
            and self.y_cg == 0.0
            and self.z_cg == 0.0
        )

    def tensor(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
        return (
            (self.ixx, -self.ixy, -self.ixz),
            (-self.ixy, self.iyy, -self.iyz),
            (-self.ixz, -self.iyz, self.izz_eff),
        )


def thin_tube_inertia(mass: float, radius: float, length: float) -> tuple[float, float]:
    """(Ixx, Iyy) of a thin-walled tube: Ixx = m r^2, Iyy = m (r^2/2 + L^2/12)."""
    return mass * radius**2, mass * (0.5 * radius**2 + length**2 / 12.0)


def solid_cylinder_inertia(mass: float, radius: float, length: float) -> tuple[float, float]:
    """(Ixx, Iyy) of a solid cylinder: Ixx = m r^2/2, Iyy = m (3 r^2 + L^2)/12."""
    return 0.5 * mass * radius**2, mass * (3.0 * radius**2 + length**2) / 12.0


def _shift_term(
    own: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    m: float,
    r: tuple[float, float, float],
) -> list[list[float]]:
    """I_own + m (|r|^2 E - r r^T): tensor of a body of mass m with own tensor ``own`` located at r."""
    r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    return [[own[i][j] + m * ((r2 if i == j else 0.0) - r[i] * r[j]) for j in range(3)] for i in range(3)]


def _component_tensor(c: MassComponent):
    return (
        (c.ixx, -c.ixy, -c.ixz),
        (-c.ixy, c.iyy, -c.iyz),
        (-c.ixz, -c.iyz, c.izz_eff),
    )


class MassModel:
    """Static components + a burning propellant cylinder (on the axis)."""

    def __init__(
        self,
        static: list[MassComponent],
        propellant_x: float,
        propellant_radius: float,
        propellant_length: float,
    ) -> None:
        if not static or sum(c.mass for c in static) <= 0:
            raise ConfigError("mass model needs at least one component with positive mass")
        self.static = list(static)
        self._m_s = sum(c.mass for c in static)
        self._sx = sum(c.mass * c.x_cg for c in static)  # first moments (x aft of the nose)
        self._sy = sum(c.mass * c.y for c in static)
        self._sz = sum(c.mass * c.z for c in static)
        self.px, self.pr, self.pl = propellant_x, propellant_radius, propellant_length
        self._axisymmetric = all(
            c.izz is None and c.ixy == 0 and c.ixz == 0 and c.iyz == 0 and c.y == 0 and c.z == 0
            for c in static
        )
        # scalar sums for the fast (axisymmetric) path: inertia about the nose-tip axis
        self._ixx_s = sum(c.ixx for c in static)
        self._iyy_o = sum(c.iyy + c.mass * c.x_cg**2 for c in static)
        # tensor of the static assembly about the nose tip (general path), body axes
        t = [[0.0] * 3 for _ in range(3)]
        for c in static:
            term = _shift_term(_component_tensor(c), c.mass, (-c.x_cg, c.y, c.z))
            for i in range(3):
                for j in range(3):
                    t[i][j] += term[i][j]
        self._t_static = t

    @property
    def static_mass(self) -> float:
        return self._m_s

    def at(self, propellant_mass: float) -> MassProps:
        mp = max(propellant_mass, 0.0)
        m = self._m_s + mp
        x_cg = (self._sx + mp * self.px) / m
        ixx_p, iyy_p = solid_cylinder_inertia(mp, self.pr, self.pl)
        if self._axisymmetric:  # fast path: identical to the V1 scalar formulas
            ixx = self._ixx_s + ixx_p
            iyy_o = self._iyy_o + iyy_p + mp * self.px**2
            return MassProps(m, x_cg, ixx, max(iyy_o - m * x_cg**2, 1e-12))
        y_cg, z_cg = self._sy / m, self._sz / m
        prop = _shift_term(
            ((ixx_p, 0.0, 0.0), (0.0, iyy_p, 0.0), (0.0, 0.0, iyy_p)), mp, (-self.px, 0.0, 0.0)
        )
        t_o = [[self._t_static[i][j] + prop[i][j] for j in range(3)] for i in range(3)]
        r_cg = (-x_cg, y_cg, z_cg)
        r2 = r_cg[0] ** 2 + r_cg[1] ** 2 + r_cg[2] ** 2
        i_cg = [
            [t_o[i][j] - m * ((r2 if i == j else 0.0) - r_cg[i] * r_cg[j]) for j in range(3)]
            for i in range(3)
        ]
        return MassProps(
            m,
            x_cg,
            max(i_cg[0][0], 1e-12),
            max(i_cg[1][1], 1e-12),
            y_cg,
            z_cg,
            max(i_cg[2][2], 1e-12),
            -i_cg[0][1],
            -i_cg[0][2],
            -i_cg[1][2],
        )


def inertia_tensor_about_cg(components: list[MassComponent]) -> tuple[float, float, list[list[float]]]:
    """Independent reference implementation by direct summation: (mass, x_cg aft of nose, 3x3 tensor).

    Used by the tests to verify ``MassModel`` against a separately written formula."""
    m = sum(c.mass for c in components)
    pos = [(-c.x_cg, c.y, c.z) for c in components]
    cg = [sum(c.mass * p[k] for c, p in zip(components, pos)) / m for k in range(3)]
    tensor = [[0.0] * 3 for _ in range(3)]
    for c, p in zip(components, pos):
        d = (p[0] - cg[0], p[1] - cg[1], p[2] - cg[2])
        term = _shift_term(_component_tensor(c), c.mass, d)
        for i in range(3):
            for j in range(3):
                tensor[i][j] += term[i][j]
    return m, -cg[0], tensor
