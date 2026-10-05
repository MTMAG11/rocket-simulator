"""Mass, centre of mass and moments of inertia as a function of remaining propellant.

Axial positions are measured aft from the nose tip. Inertias are about the instantaneous
centre of mass, expressed in body axes: Ixx (roll, about the nose axis) and Iyy = Izz
(pitch/yaw, axisymmetric vehicle; products of inertia are zero by symmetry).

Components are rigid bodies with their own inertia about their own CG. The propellant is
modelled as a solid uniform cylinder of the motor's diameter and length whose CG stays at the
motor centre while it burns (grain regression and CG shift are NOT modelled; this error is
small for the usual center-burning-grain hobby motors but is a limitation of the model).

Parallel-axis theorem: I_cg = sum(I_i + m_i d_i^2) - ... see MassModel.at().
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
    iyy: float = 0.0  # [kg m^2] about own CG, transverse axis

    def __post_init__(self) -> None:
        if self.mass < 0:
            raise ConfigError(f"mass component {self.name!r}: mass must be >= 0")
        if self.ixx < 0 or self.iyy < 0:
            raise ConfigError(f"mass component {self.name!r}: inertia must be >= 0")


@dataclass(frozen=True)
class MassProps:
    mass: float
    x_cg: float  # [m] aft of nose
    ixx: float
    iyy: float  # = Izz


def thin_tube_inertia(mass: float, radius: float, length: float) -> tuple[float, float]:
    """(Ixx, Iyy) of a thin-walled tube: Ixx = m r^2, Iyy = m (r^2/2 + L^2/12)."""
    return mass * radius**2, mass * (0.5 * radius**2 + length**2 / 12.0)


def solid_cylinder_inertia(mass: float, radius: float, length: float) -> tuple[float, float]:
    """(Ixx, Iyy) of a solid cylinder: Ixx = m r^2/2, Iyy = m (3 r^2 + L^2)/12."""
    return 0.5 * mass * radius**2, mass * (3.0 * radius**2 + length**2) / 12.0


class MassModel:
    """Static components + a burning propellant cylinder."""

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
        self._s_s = sum(c.mass * c.x_cg for c in static)
        self._ixx_s = sum(c.ixx for c in static)
        self._iyy_o = sum(c.iyy + c.mass * c.x_cg**2 for c in static)  # about the nose tip
        self.px = propellant_x
        self.pr = propellant_radius
        self.pl = propellant_length

    @property
    def static_mass(self) -> float:
        return self._m_s

    def at(self, propellant_mass: float) -> MassProps:
        mp = max(propellant_mass, 0.0)
        m = self._m_s + mp
        x_cg = (self._s_s + mp * self.px) / m
        ixx_p, iyy_p = solid_cylinder_inertia(mp, self.pr, self.pl)
        ixx = self._ixx_s + ixx_p
        iyy_o = self._iyy_o + iyy_p + mp * self.px**2
        iyy = iyy_o - m * x_cg**2  # shift from the nose-tip axis to the CG
        return MassProps(m, x_cg, ixx, max(iyy, 1e-12))
