"""Thrust-vector-control actuator model (two axes, independent).

Command path:  controller -> clip to +-max_angle -> delay -> first-order lag -> rate limit ->
angle limit -> actual gimbal angle (what the physics sees).

Parameters (ActuatorCfg): max_angle_deg, max_rate_deg_s, time_constant_s (0 = ideal lag-free),
delay_s (pure transport delay). The lag is integrated exactly per step (stable for any step
size), rate limiting is applied to the resulting increment.

Not modelled: backlash, deadband, quantisation, load-dependent torque limits, actuator
dynamics beyond first order. Aerodynamic control surfaces (fins/canards) are not yet
implemented; the Command type is the extension point.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from ..config.schema import ActuatorCfg


@dataclass
class Command:
    """Controller output. Angles in radians: thrust deflection about body y and z."""

    tvc_y: float = 0.0
    tvc_z: float = 0.0


class TVCActuator:
    def __init__(self, cfg: ActuatorCfg) -> None:
        self.max_angle = math.radians(cfg.max_angle_deg)
        self.max_rate = math.radians(cfg.max_rate_deg_s)
        self.tau = cfg.time_constant_s
        self.delay = cfg.delay_s
        self.state = [0.0, 0.0]  # actual angles [rad]
        self.cmd = [0.0, 0.0]  # latest command issued (post-clip, pre-delay)
        self._effective = [0.0, 0.0]  # command after transport delay
        self._queue: deque[tuple[float, float, float]] = deque()

    def command(self, t: float, cmd: Command) -> None:
        a = self.max_angle
        y = min(max(cmd.tvc_y, -a), a)
        z = min(max(cmd.tvc_z, -a), a)
        self.cmd = [y, z]
        self._queue.append((t + self.delay, y, z))

    def step(self, t: float, h: float) -> None:
        """Advance the actuator state from t to t + h."""
        while self._queue and self._queue[0][0] <= t + 1e-12:
            _, y, z = self._queue.popleft()
            self._effective = [y, z]
        gain = 1.0 if self.tau <= 0 else 1.0 - math.exp(-h / self.tau)
        lim = self.max_rate * h
        for i in range(2):
            d = (self._effective[i] - self.state[i]) * gain
            d = min(max(d, -lim), lim)
            self.state[i] = min(max(self.state[i] + d, -self.max_angle), self.max_angle)
