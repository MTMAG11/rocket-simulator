"""Vehicle: geometry + aerodynamics + mass model + motor mounting + recovery."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..motor import Motor
from .aero import AerodynamicModel
from .assembly import Assembly
from .geometry import BodyTube, Parachute
from .mass import MassModel, MassProps


@dataclass
class Vehicle:
    name: str
    body: BodyTube
    aero: AerodynamicModel
    mass_model: MassModel
    motor: Motor
    nozzle_x: float  # [m] aft of nose tip: where thrust is applied (nozzle exit plane)
    thrust_misalignment: tuple[float, float] = (0.0, 0.0)  # (about y_B, about z_B) [rad]
    ignition_delay: float = 0.0  # [s] from simulation start to motor ignition
    parachutes: list[Parachute] = field(default_factory=list)
    assembly: Assembly | None = None  # component geometry (None for hand-built test vehicles)
    notes: list[str] = field(default_factory=list)  # build-time caveats (e.g. estimated inertia)

    def mass_props(self, t: float) -> MassProps:
        """Mass properties at simulation time t (propellant from the motor model)."""
        return self.mass_model.at(self.motor.propellant_at(t - self.ignition_delay))

    def static_margin(self, t: float, mach: float = 0.0) -> float:
        """Static margin in calibers: (x_cp - x_cg) / d_ref (d_ref = aerodynamic reference diameter,
        normally the largest body diameter). Positive = statically stable."""
        mp = self.mass_props(t)
        c = self.aero.coefficients(mach, 1e6, False)
        return (c.x_cp - mp.x_cg) / self.aero.ref_diameter

    def thrust_at(self, t: float) -> float:
        return self.motor.thrust_at(t - self.ignition_delay)

    @property
    def liftoff_mass(self) -> float:
        return self.mass_props(0.0).mass

    @property
    def burnout_time(self) -> float:
        return self.ignition_delay + self.motor.burn_time
