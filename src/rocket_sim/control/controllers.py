"""Pluggable controllers. The physics engine never sees these: state in -> command out.

    class MyController(Controller):
        def update(self, inp: ControlInput) -> Command: ...

Register in YAML with ``controller: {type: python, params: {class: "package.module:MyController",
kwargs: {...}}}`` -- any object with ``reset(ctx)`` and ``update(inp)`` works, including a wrapper
around a neural network or a hardware-in-the-loop serial bridge.

Built in:
  NullController          zero command.
  ScheduleController      open-loop gimbal table [[t_s, tvc_y_deg, tvc_z_deg], ...] (linear
                          interpolation, time since ignition).
  TVCAttitudeController   PD pointing controller commanding angular acceleration
                          alpha = wn^2 * e - 2 zeta wn * omega, converted to a gimbal angle with the
                          known thrust/inertia/lever-arm (gain scheduling from the nominal motor
                          curve, as a real flight computer would). Roll is not controlled.
"""

from __future__ import annotations

import bisect
import importlib
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from ..errors import ConfigError
from ..physics.math3d import Quat, Vec, cross, norm, quat_rotate, quat_rotate_inv
from .actuators import Command


@dataclass
class ControlInput:
    """What a flight computer would know: ESTIMATED (or, if configured, true) state."""

    t: float  # simulation time [s]
    t_since_launch: float | None  # None until launch is detected
    valid: bool
    position: Vec
    velocity: Vec
    quaternion: Quat
    omega: Vec  # body rates [rad/s]
    phase: int
    sensors: Any = None  # latest raw SensorReadings (None below fidelity 4); used by HIL bridges


class Controller(ABC):
    name = "controller"

    def reset(self, ctx: dict[str, Any]) -> None:  # noqa: B027 - optional hook
        """Called once before the run. ``ctx`` has 'authority' and 'launch_axis' (see sim)."""

    @abstractmethod
    def update(self, inp: ControlInput) -> Command: ...


class NullController(Controller):
    name = "none"

    def update(self, inp: ControlInput) -> Command:
        return Command()


class ScheduleController(Controller):
    name = "schedule"

    def __init__(self, table: list[list[float]]) -> None:
        if not table or any(len(r) != 3 for r in table):
            raise ConfigError("controller.params.table: rows of [t_s, tvc_y_deg, tvc_z_deg]")
        table = sorted(table)
        self.t = [r[0] for r in table]
        self.y = [math.radians(r[1]) for r in table]
        self.z = [math.radians(r[2]) for r in table]

    def update(self, inp: ControlInput) -> Command:
        t = inp.t
        if t <= self.t[0]:
            return Command(self.y[0], self.z[0])
        if t >= self.t[-1]:
            return Command(self.y[-1], self.z[-1])
        i = bisect.bisect_right(self.t, t) - 1
        f = (t - self.t[i]) / (self.t[i + 1] - self.t[i])
        return Command(
            self.y[i] + f * (self.y[i + 1] - self.y[i]),
            self.z[i] + f * (self.z[i + 1] - self.z[i]),
        )


class TVCAttitudeController(Controller):
    """PD (+ optional integral) control of the nose pointing direction using TVC.

    Parameters: ``wn`` natural frequency [rad/s] (default 6), ``zeta`` damping ratio (default
    0.8), ``target`` = "vertical" | "initial" | [x, y, z] direction in the launch frame,
    ``ki`` integral gain on the pointing error (default 0), ``max_authority`` ratio cap.
    The gimbal-per-acceleration scheduling comes from ``ctx['authority'](t)``: a function
    returning I_yy / (T * |lever_arm|) at the nominal thrust (None when thrust ~ 0).
    """

    name = "tvc_attitude"

    def __init__(
        self, wn: float = 6.0, zeta: float = 0.8, target: Any = "vertical", ki: float = 0.0, **_: Any
    ) -> None:
        if wn <= 0 or zeta < 0 or ki < 0:
            raise ConfigError("tvc_attitude: wn > 0, zeta >= 0, ki >= 0 required")
        self.wn, self.zeta, self.ki = wn, zeta, ki
        self.target = target
        self._authority = None
        self._target_vec: Vec = (0.0, 0.0, 1.0)
        self._int = [0.0, 0.0]
        self._last_t: float | None = None

    def reset(self, ctx: dict[str, Any]) -> None:
        self._authority = ctx.get("authority")
        axis = ctx.get("launch_axis", (0.0, 0.0, 1.0))
        tgt = self.target
        if tgt == "vertical":
            self._target_vec = (0.0, 0.0, 1.0)
        elif tgt == "initial":
            self._target_vec = tuple(axis)
        elif isinstance(tgt, list | tuple) and len(tgt) == 3 and norm(tuple(tgt)) > 0:
            n = norm(tuple(tgt))
            self._target_vec = (tgt[0] / n, tgt[1] / n, tgt[2] / n)
        else:
            raise ConfigError(f"tvc_attitude: invalid target {tgt!r}")
        self._int = [0.0, 0.0]
        self._last_t = None

    def update(self, inp: ControlInput) -> Command:
        if not inp.valid or inp.t_since_launch is None or self._authority is None:
            return Command()
        gpa = self._authority(inp.t)
        if gpa is None:
            return Command()
        nose = quat_rotate(inp.quaternion, (1.0, 0.0, 0.0))
        err_l = cross(nose, self._target_vec)  # rotation vector (launch frame) nose -> target
        ey, ez = quat_rotate_inv(inp.quaternion, err_l)[1:]
        dt = 0.0 if self._last_t is None else inp.t - self._last_t
        self._last_t = inp.t
        if self.ki > 0 and dt > 0:
            self._int[0] += ey * dt
            self._int[1] += ez * dt
            lim = 0.3
            self._int = [min(max(v, -lim), lim) for v in self._int]
        kp, kd = self.wn**2, 2.0 * self.zeta * self.wn
        a_y = kp * ey + self.ki * self.wn**2 * self._int[0] - kd * inp.omega[1]
        a_z = kp * ez + self.ki * self.wn**2 * self._int[1] - kd * inp.omega[2]
        # thrust-offset torque: M_y = rn T sin(th_y), M_z = rn T sin(th_z), rn < 0  =>  th = -a * I/(|rn| T)
        return Command(-a_y * gpa, -a_z * gpa)


def build_controller(ctype: str, params: dict[str, Any]) -> Controller:
    if ctype == "none":
        return NullController()
    if ctype == "schedule":
        return ScheduleController(params.get("table", []))
    if ctype == "tvc_attitude":
        return TVCAttitudeController(**params)
    if ctype == "python":
        spec = params.get("class")
        if not isinstance(spec, str) or ":" not in spec:
            raise ConfigError("controller.params.class must look like 'package.module:ClassName'")
        mod_name, cls_name = spec.split(":", 1)
        try:
            cls = getattr(importlib.import_module(mod_name), cls_name)
        except (ImportError, AttributeError) as exc:
            raise ConfigError(f"cannot import controller '{spec}': {exc}") from exc
        return cls(**params.get("kwargs", {}))
    raise ConfigError(f"unknown controller type {ctype!r}")
