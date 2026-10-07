"""Reusable actuator model for TVC gimbals and aerodynamic control surfaces.

    command --(saturate)--> --(transport delay)--> --(first-order lag)--> --(rate limit)--> --(angle limit)--> state

The PHYSICAL actuator state (``state``) is what the physics engine applies; the commanded value is only an
input. Delay is the communication/processing latency before the actuator sees the command; the rate limit and
lag are the actuator's own dynamics (so the actuator slews toward the *delayed* command). The lag is
integrated exactly per step (stable for any step size); the rate limit is applied to the resulting increment.

``ActuatorBank`` holds several independent channels with per-channel limits (N fins); ``TVCActuator`` is the
two-channel gimbal built on it.

Not modelled: backlash, deadband, quantisation, load-dependent torque limits, position-dependent rate.
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


class ActuatorBank:
    def __init__(
        self,
        max_angle: list[float],
        max_rate: list[float],
        time_constant: list[float],
        delay: list[float],
    ) -> None:
        n = len(max_angle)
        if not (len(max_rate) == len(time_constant) == len(delay) == n) or n == 0:
            raise ValueError("actuator limit lists must have equal, non-zero length")
        self.n = n
        self.max_angle, self.max_rate, self.tau, self.delay = max_angle, max_rate, time_constant, delay
        self.state = [0.0] * n  # physical actuator positions
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
            d = (self._effective[i] - self.state[i]) * gain
            d = min(max(d, -lim), lim)
            self.state[i] = min(max(self.state[i] + d, -self.max_angle[i]), self.max_angle[i])


class TVCActuator(ActuatorBank):
    """Two-axis thrust-vector gimbal (channels: about y_B, about z_B)."""

    def __init__(self, cfg: ActuatorCfg) -> None:
        a = math.radians(cfg.max_angle_deg)
        r = math.radians(cfg.max_rate_deg_s)
        super().__init__([a, a], [r, r], [cfg.time_constant_s] * 2, [cfg.delay_s] * 2)

    def command(self, t: float, cmd: Command | list[float]) -> None:
        if isinstance(cmd, Command):
            cmd = [cmd.tvc_y, cmd.tvc_z]
        super().command(t, cmd)
