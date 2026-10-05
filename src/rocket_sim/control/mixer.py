"""Control-surface mixer: body-axis commands -> individual fin deflections.

For fin i at angular position phi_i (from +y_B toward +z_B) with lift slope L_i (per rad, referenced to S_ref),
aerodynamic centre ahead of the CG by rx = x_cg - x_ac (negative for aft fins) and radius rho, a deflection
delta_i produces (per unit q S):

    M_x (roll)  =  rho L_i delta_i
    M_y (pitch) = -rx L_i cos(phi_i) delta_i
    M_z (yaw)   = -rx L_i sin(phi_i) delta_i

``B`` is the 3 x N effectiveness matrix of these. The mixer uses a matched filter: command channel k deflects fin i by
W_ik = B_ki / max_i |B_ki|, so a unit command (radians) means "the most effective fin moves 1 rad". For evenly spaced
fins (N >= 3) the pitch, yaw and roll channels are mutually orthogonal, so the cross-coupling is zero; for other layouts
the residual coupling is reported by ``coupling()``. ``gain(k)`` is the achieved moment per radian of command per unit
q S (needed by controllers to convert a desired angular acceleration into a command).
"""

from __future__ import annotations

from ..vehicle.aero import ControlFin
from .actuators import Command


class FinMixer:
    def __init__(
        self, fins: tuple[ControlFin, ...] | list[ControlFin], x_cg_ref: float, mach_factor: float = 1.0
    ) -> None:
        import math

        self.n = len(fins)
        self.B: list[list[float]] = [[0.0] * self.n for _ in range(3)]
        for i, f in enumerate(fins):
            rx = x_cg_ref - f.x_ac
            lift = f.cn_alpha_single * mach_factor
            self.B[0][i] = f.rho * lift  # roll
            self.B[1][i] = -rx * lift * math.cos(f.phi)  # pitch
            self.B[2][i] = -rx * lift * math.sin(f.phi)  # yaw
        self.W: list[list[float]] = []  # [fin][channel] with channels (roll, pitch, yaw)
        self._peak = [max((abs(v) for v in row), default=0.0) for row in self.B]
        for i in range(self.n):
            self.W.append(
                [(self.B[k][i] / self._peak[k] if self._peak[k] > 1e-12 else 0.0) for k in range(3)]
            )

    def mix(self, cmd: Command) -> list[float]:
        """Per-fin deflection commands [rad] for a body-axis command (roll, pitch, yaw channels)."""
        return [w[0] * cmd.fin_roll + w[1] * cmd.fin_pitch + w[2] * cmd.fin_yaw for w in self.W]

    def gain(self, channel: int) -> float:
        """Moment per radian of command per unit (q S) for channel 0 roll, 1 pitch, 2 yaw (sum B_ki W_ik)."""
        return sum(self.B[channel][i] * self.W[i][channel] for i in range(self.n))

    def coupling(self) -> float:
        """Largest off-axis moment as a fraction of the on-axis moment for unit commands (0 = decoupled)."""
        worst = 0.0
        for k in range(3):
            g = self.gain(k)
            if abs(g) < 1e-12:
                continue
            for j in range(3):
                if j != k:
                    cross = sum(self.B[j][i] * self.W[i][k] for i in range(self.n))
                    worst = max(worst, abs(cross / g))
        return worst
