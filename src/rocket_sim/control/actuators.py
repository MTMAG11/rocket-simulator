"""Reusable actuator model for TVC gimbals and aerodynamic control surfaces.

    command --(saturate)--> --(transport delay)--> --(dead zone)--> --(first-order lag)--> --(rate limit)-->
            --(acceleration limit)--> --(angle limit)--> shaft --(backlash)--> state

The PHYSICAL actuator state (``state``) is what the physics engine applies; the commanded value is only an
input. Delay is the communication/processing latency before the actuator sees the command; the rate limit and
lag are the actuator's own dynamics (so the actuator slews toward the *delayed* command). The lag is
integrated exactly per step (stable for any step size); the rate limit is applied to the resulting increment.

``ActuatorBank`` holds several independent channels with per-channel limits (N fins); ``TVCActuator`` is the
two-channel gimbal built on it.

Dead zone, backlash and the acceleration (torque) limit are optional and off by default, in which case the model is exactly
the original delay, lag, rate-limit and angle-limit chain. Derivation: docs/derivations/servo_model.md.

Not modelled: quantisation, load-dependent torque limits (the acceleration limit is a constant), position-dependent rate.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from ..config.schema import ActuatorCfg


@dataclass
class Command:
    """Controller output (radians). TVC: thrust deflection about body y and z. Control surfaces: pitch/yaw/roll
    deflection-equivalent commands that the vehicle's mixer converts to individual fin deflections."""

    tvc_y: float = 0.0
    tvc_z: float = 0.0
    fin_pitch: float = 0.0  # moment about +y_B
    fin_yaw: float = 0.0  # moment about +z_B
    fin_roll: float = 0.0  # moment about +x_B


def play_operator(shaft: float, output_prev: float, half_gap: float) -> float:
    """TODO(you): output angle of a gear train with play, given the motor-side ``shaft`` angle, the previous ``output_prev``
    angle and the half-gap ``half_gap`` (all in the same unit; ``half_gap`` >= 0).

    Derive it in docs/derivations/servo_model.md, section 5, and check it against the table there
    (``pytest --runxfail -m user_todo`` shows the test failing until this is implemented). Only called when a
    channel's backlash is non-zero.
    """
    raise NotImplementedError("play_operator: see the TODO in rocket_sim/control/actuators.py")


class ActuatorBank:
    def __init__(
        self,
        max_angle: list[float],
        max_rate: list[float],
        time_constant: list[float],
        delay: list[float],
        *,
        deadband: list[float] | None = None,
        backlash: list[float] | None = None,
        max_accel: list[float | None] | None = None,
    ) -> None:
        n = len(max_angle)
        if not (len(max_rate) == len(time_constant) == len(delay) == n) or n == 0:
            raise ValueError("actuator limit lists must have equal, non-zero length")
        self.n = n
        self.max_angle, self.max_rate, self.tau, self.delay = max_angle, max_rate, time_constant, delay
        self.deadband = (
            list(deadband) if deadband is not None else [0.0] * n
        )  # half-width of the dead zone [rad]
        self.backlash = list(backlash) if backlash is not None else [0.0] * n  # total play (gap) [rad]
        self.max_accel = (
            list(max_accel) if max_accel is not None else [None] * n
        )  # torque limit / inertia [rad/s^2]
        for name, vals in (("deadband", self.deadband), ("backlash", self.backlash)):
            if len(vals) != n or any(v < 0.0 or not math.isfinite(v) for v in vals):
                raise ValueError(f"actuator {name} must be {n} finite values >= 0")
        if len(self.max_accel) != n or any(a is not None and not a > 0.0 for a in self.max_accel):
            raise ValueError(f"actuator max_accel must be {n} values that are None or > 0")
        self.state = [0.0] * n  # physical actuator positions (the output, after backlash)
        self._shaft = [0.0] * n  # motor-side position, before backlash
        self._vel = [0.0] * n  # shaft angular velocity (used only with an acceleration limit)
        self.cmd = [0.0] * n  # latest (saturated) command issued
        self._effective = [0.0] * n
        self._queue: list[list[tuple[float, float]]] = [[] for _ in range(n)]  # kept sorted by due time

    def command(self, t: float, values: list[float]) -> None:
        for i, v in enumerate(values):
            a = self.max_angle[i]
            v = min(max(v, -a), a)
            self.cmd[i] = v
            bisect.insort(
                self._queue[i], (t + self.delay[i], v), key=lambda e: e[0]
            )  # stable: equal due times keep order

    def step(self, t: float, h: float) -> None:
        """Advance every channel from t to t + h."""
        for i in range(self.n):
            q = self._queue[i]
            while q and q[0][0] <= t + 1e-12:
                self._effective[i] = q.pop(0)[1]
            gain = 1.0 if self.tau[i] <= 0 else 1.0 - math.exp(-h / self.tau[i])
            lim = self.max_rate[i] * h
            e = self._effective[i] - self._shaft[i]
            db = self.deadband[i]
            if db > 0.0:
                e = 0.0 if abs(e) <= db else e - math.copysign(db, e)
            d = min(max(e * gain, -lim), lim)
            amax = self.max_accel[i]
            if amax is not None and h > 0.0:
                dv = amax * h
                v = min(max(d / h, self._vel[i] - dv), self._vel[i] + dv)
                d = v * h
                self._vel[i] = v
            self._shaft[i] = min(max(self._shaft[i] + d, -self.max_angle[i]), self.max_angle[i])
            if self.backlash[i] > 0.0:
                out = play_operator(self._shaft[i], self.state[i], 0.5 * self.backlash[i])
                self.state[i] = min(max(out, -self.max_angle[i]), self.max_angle[i])
            else:
                self.state[i] = self._shaft[i]


class TVCActuator(ActuatorBank):
    """Two-axis thrust-vector gimbal (channels: about y_B, about z_B)."""

    def __init__(self, cfg: ActuatorCfg) -> None:
        a = math.radians(cfg.max_angle_deg)
        r = math.radians(cfg.max_rate_deg_s)
        accel = None if cfg.max_accel_deg_s2 is None else math.radians(cfg.max_accel_deg_s2)
        super().__init__(
            [a, a],
            [r, r],
            [cfg.time_constant_s] * 2,
            [cfg.delay_s] * 2,
            deadband=[math.radians(cfg.deadband_deg)] * 2,
            backlash=[math.radians(cfg.backlash_deg)] * 2,
            max_accel=[accel] * 2,
        )

    def command(self, t: float, cmd: Command | list[float]) -> None:
        if isinstance(cmd, Command):
            cmd = [cmd.tvc_y, cmd.tvc_z]
        super().command(t, cmd)
