"""Solid rocket motor model: piecewise-linear thrust curve with impulse-proportional burn.

Model
-----
* Thrust T(t) is linearly interpolated between data points; it is 0 before ignition and after
  the final point. A (0, 0) point is prepended when the file starts later than t = 0
  (same convention as OpenRocket).
* Cumulative impulse I(t) = integral of T is computed in closed form per segment.
* Propellant mass is consumed in proportion to impulse:
      m_p(t) = m_p0 * (1 - I(t) / I_total)
  i.e. a constant effective exhaust velocity c = I_total / m_p0 over the whole burn. This is
  the standard simplification when only a thrust curve is available.
  Mass flow:  mdot = m_p0 * T(t) / I_total.
* Motor-time is measured from ignition; the simulator handles ignition delay.

Limitations: no pressure/altitude thrust correction (thrust curves are sea-level static tests),
no grain regression geometry, no nozzle erosion, no thrust vector misalignment (handled in the
vehicle model).
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

from ..constants import G0
from ..errors import MotorError

_TIME_TOL = 1e-9  # [s] slack when comparing accumulated simulation time to curve end points


@dataclass
class Motor:
    designation: str
    diameter: float  # [m]
    length: float  # [m]
    propellant_mass: float  # [kg]
    total_mass: float  # [kg] loaded (propellant + casing/hardware)
    times: list[float]  # [s], strictly increasing, starts at 0
    thrust: list[float]  # [N], same length
    manufacturer: str = ""
    delays: str = ""
    source: str = ""
    # derived (filled in __post_init__)
    cum_impulse: list[float] = field(init=False, repr=False)
    total_impulse: float = field(init=False)
    burn_time: float = field(init=False)
    max_thrust: float = field(init=False)

    def __post_init__(self) -> None:
        n = len(self.times)
        if n < 2 or len(self.thrust) != n:
            raise MotorError(f"{self.designation}: thrust curve needs >= 2 points")
        if self.times[0] != 0.0:
            raise MotorError(f"{self.designation}: curve must start at t = 0")
        for a, b in zip(self.times, self.times[1:], strict=False):
            if not b > a:
                raise MotorError(f"{self.designation}: thrust times must be strictly increasing")
        if min(self.thrust) < 0 or not all(math.isfinite(x) for x in self.thrust):
            raise MotorError(f"{self.designation}: thrust must be finite and >= 0")
        if self.propellant_mass <= 0:
            raise MotorError(f"{self.designation}: propellant mass must be > 0")
        if self.total_mass < self.propellant_mass:
            raise MotorError(
                f"{self.designation}: total mass {self.total_mass} kg < propellant {self.propellant_mass} kg"
            )
        if self.diameter <= 0 or self.length <= 0:
            raise MotorError(f"{self.designation}: diameter and length must be > 0")
        cum = [0.0]
        for i in range(n - 1):
            dt = self.times[i + 1] - self.times[i]
            cum.append(cum[-1] + 0.5 * (self.thrust[i] + self.thrust[i + 1]) * dt)
        self.cum_impulse = cum
        self.total_impulse = cum[-1]
        if self.total_impulse <= 0:
            raise MotorError(f"{self.designation}: total impulse is zero")
        self.burn_time = self.times[-1]
        self.max_thrust = max(self.thrust)

    # -- derived quantities -------------------------------------------------------------------
    @property
    def casing_mass(self) -> float:
        """Motor hardware mass after burnout [kg]."""
        return self.total_mass - self.propellant_mass

    @property
    def average_thrust(self) -> float:
        return self.total_impulse / self.burn_time

    @property
    def specific_impulse(self) -> float:
        """Effective Isp [s] = I_total / (m_p g0)."""
        return self.total_impulse / (self.propellant_mass * G0)

    @property
    def impulse_class(self) -> str:
        """NAR/TRA letter class from total impulse (A = 1.26-2.5 Ns, doubling each class)."""
        i = self.total_impulse
        if i <= 1.25:
            return "1/2A or less"
        cls = int(math.ceil(math.log2(i / 1.25)))
        return chr(ord("A") + cls - 1) if cls <= 26 else "?"

    # -- evaluation (t = seconds since ignition) ----------------------------------------------
    def thrust_at(self, t: float) -> float:
        """Thrust [N] at motor time t. Defined on the closed interval [0, burn_time]: the end
        points return the curve's end values (right-limit at 0, left-limit at burn_time) so that
        fixed-step integrators evaluating exactly at a step boundary see the correct thrust."""
        if t < -_TIME_TOL or t > self.burn_time + _TIME_TOL:
            return 0.0  # (tolerance absorbs floating-point drift of accumulated step times)
        if t >= self.burn_time:
            return self.thrust[-1]
        if t <= 0.0:
            return self.thrust[0]
        i = bisect.bisect_right(self.times, t) - 1
        t0, t1 = self.times[i], self.times[i + 1]
        f = (t - t0) / (t1 - t0)
        return self.thrust[i] + f * (self.thrust[i + 1] - self.thrust[i])

    def impulse_at(self, t: float) -> float:
        """Cumulative impulse delivered up to motor time t [N s]."""
        if t <= 0.0:
            return 0.0
        if t >= self.burn_time:
            return self.total_impulse
        i = bisect.bisect_right(self.times, t) - 1
        tau = t - self.times[i]
        dt = self.times[i + 1] - self.times[i]
        t0, t1 = self.thrust[i], self.thrust[i + 1]
        return self.cum_impulse[i] + t0 * tau + (t1 - t0) * tau * tau / (2.0 * dt)

    def propellant_at(self, t: float) -> float:
        """Remaining propellant mass [kg] at motor time t."""
        if t >= self.burn_time:
            return 0.0
        return self.propellant_mass * (1.0 - self.impulse_at(t) / self.total_impulse)

    def mass_flow_at(self, t: float) -> float:
        return self.propellant_mass * self.thrust_at(t) / self.total_impulse

    def scaled(self, thrust_scale: float = 1.0, time_scale: float = 1.0) -> Motor:
        """Return a copy with thrust and/or time axis scaled (propellant mass unchanged).

        Total impulse scales by thrust_scale * time_scale, so the effective Isp changes by the
        same factor; use this to represent manufacturing/ambient thrust variability.
        """
        if thrust_scale <= 0 or time_scale <= 0:
            raise MotorError("motor scale factors must be > 0")
        return Motor(
            self.designation,
            self.diameter,
            self.length,
            self.propellant_mass,
            self.total_mass,
            [t * time_scale for t in self.times],
            [f * thrust_scale for f in self.thrust],
            self.manufacturer,
            self.delays,
            self.source,
        )

    def sanity_warnings(self) -> list[str]:
        """Soft checks on data plausibility (returned, not raised)."""
        w = []
        isp = self.specific_impulse
        if not 40.0 <= isp <= 330.0:
            w.append(
                f"effective Isp {isp:.0f} s is outside the plausible 40-330 s range for solid "
                "motors; check propellant mass / thrust data"
            )
        if self.thrust[-1] != 0.0:
            w.append("thrust curve does not end at 0 N (thrust drops instantaneously)")
        return w
