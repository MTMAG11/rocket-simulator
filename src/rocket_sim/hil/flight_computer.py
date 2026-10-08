"""The flight-computer side of the HIL boundary.

``FlightComputer`` is the contract a real flight computer (ESP32 firmware, a host build of it, or the Python reference below)
must fulfil: ``reset(init)`` once, then ``on_tick(tick) -> command dict`` at every control tick. It sees ONLY the protocol
messages (docs/hil.md): timestamped sensor samples and design data. ``handle_message``/``serve`` turn any FlightComputer into a
byte-line server (loopback, subprocess pipe, or a future serial/UDP link).

``ReferenceFlightComputer`` is a pure-Python implementation of the flight software loop (alignment, navigation filter, attitude
controller) that works from the packets alone: it is the executable specification of what the firmware has to do and the
test double for the whole HIL path. It is NOT validated flight software.
"""

from __future__ import annotations

import sys
from typing import IO, Any, Protocol

import numpy as np

from ..config.schema import EstimatorCfg, SensorsCfg
from ..control.controllers import ControlInput, TVCAttitudeController
from ..environment.gravity import ConstantGravity
from ..errors import SimulationError
from ..estimation import NavigationFilter
from ..sensors import SensorReadings
from .protocol import PROTOCOL_VERSION, SENSOR_NAMES, decode, encode


class FlightComputer(Protocol):
    def reset(self, init: dict[str, Any]) -> None: ...

    def on_tick(self, tick: dict[str, Any]) -> dict[str, float]: ...


def handle_message(fc: FlightComputer, msg: dict[str, Any]) -> dict[str, Any]:
    """One request message -> one reply message (the whole server logic; transports just move bytes)."""
    kind = msg.get("type")
    if kind == "init":
        fc.reset(msg)
        return {"v": PROTOCOL_VERSION, "type": "ready"}
    if kind == "tick":
        cmd = fc.on_tick(msg)
        return {"v": PROTOCOL_VERSION, "type": "command", "seq": msg.get("seq"), "cmd": cmd}
    raise SimulationError(f"flight computer: unknown message type {kind!r}")


def serve(fc: FlightComputer, stdin: IO[bytes] | None = None, stdout: IO[bytes] | None = None) -> None:
    """Blocking line server: read a request line, write the reply line (for PipeTransport)."""
    rd = stdin if stdin is not None else sys.stdin.buffer
    wr = stdout if stdout is not None else sys.stdout.buffer
    for line in iter(rd.readline, b""):
        if not line.strip():
            continue
        wr.write(encode(handle_message(fc, decode(line))))
        wr.flush()


class ReferenceFlightComputer:
    """Python flight software: estimator (``NavigationFilter``) + attitude controller, driven only by packets."""

    def __init__(self, wn: float = 8.0, zeta: float = 0.8, target: Any = "vertical", ki: float = 0.0) -> None:
        self.params = {"wn": wn, "zeta": zeta, "target": target, "ki": ki}
        self.nav: NavigationFilter | None = None
        self.ctrl: TVCAttitudeController | None = None
        self._held: dict[str, np.ndarray] = {}
        self._gps_fix = False
        self._table: list[list[Any]] = []
        self.n_ticks = 0

    def reset(self, init: dict[str, Any]) -> None:
        design = init["design"]
        sensors = SensorsCfg()
        for name, attr in (
            ("accel", "accelerometer"),
            ("gyro", "gyroscope"),
            ("baro", "barometer"),
            ("gps", "gps"),
            ("mag", "magnetometer"),
        ):
            spec = init["sensors"].get(name)
            if spec is not None:
                s = getattr(sensors, attr)
                s.rate_hz, s.noise_std, s.latency_s = spec["rate_hz"], spec["noise_std"], spec["latency_s"]
        sensors.magnetic_field_enu_t = list(design["magnetic_field_enu_t"])
        est = EstimatorCfg(type="nav_kf", alignment_time_s=float(design["alignment_time_s"]))
        self.nav = NavigationFilter(
            est, sensors, ConstantGravity(float(design["gravity_m_s2"])), float(design["site_elevation_m"])
        )
        self._table = design["authority_since_ignition"]
        self.ctrl = TVCAttitudeController(**self.params)
        self.ctrl.reset({"authority": self._authority, "launch_axis": tuple(design["launch_axis"])})
        self._held = {n: np.zeros(1 if n == "baro" else (6 if n == "gps" else 3)) for n in SENSOR_NAMES}
        self._launch_time: float | None = None

    def _authority(self, t_abs: float) -> float | None:
        """Gimbal per unit angular acceleration, from the DESIGN table indexed by time since the detected launch."""
        assert self.nav is not None
        lt = self.nav.detector.time
        if lt is None:
            return None
        tau = t_abs - lt
        tab = self._table
        if tau < tab[0][0] or tau > tab[-1][0]:
            return None
        lo, hi = 0, len(tab) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if tab[mid][0] <= tau:
                lo = mid
            else:
                hi = mid
        a, b = tab[lo], tab[hi]
        if a[1] is None or b[1] is None:
            return None
        f = 0.0 if b[0] == a[0] else (tau - a[0]) / (b[0] - a[0])
        return float(a[1]) + f * (float(b[1]) - float(a[1]))

    def on_tick(self, tick: dict[str, Any]) -> dict[str, float]:
        if self.nav is None or self.ctrl is None:
            raise SimulationError("flight computer used before init")
        self.n_ticks += 1
        est = None
        for s in tick["samples"]:
            name = s["sensor"]
            self._held[name] = np.asarray(s["value"], float)
            if name == "gps":
                self._gps_fix = True
            flags = {f"{n}_new": (n == name) for n in SENSOR_NAMES}
            r = SensorReadings(
                self._held["accel"],
                self._held["gyro"],
                float(self._held["baro"][0]),
                self._held["gps"],
                self._held["mag"],
                gps_has_fix=self._gps_fix,
                **flags,
            )
            est = self.nav.update(float(s["t_visible"]), r)
        if est is None:  # no new samples this tick: advance the filter state without a measurement
            est = self.nav.update(
                float(tick["t"]),
                SensorReadings(
                    self._held["accel"],
                    self._held["gyro"],
                    float(self._held["baro"][0]),
                    self._held["gps"],
                    self._held["mag"],
                    gps_has_fix=self._gps_fix,
                ),
            )
        t = float(tick["t"])
        ts = (t - est.launch_time) if est.launch_time is not None else None
        c = self.ctrl.update(
            ControlInput(t, ts, est.valid, est.position, est.velocity, est.quaternion, est.omega, 0)
        )
        return {"tvc_y": float(c.tvc_y), "tvc_z": float(c.tvc_z)}


def main() -> None:  # python -m rocket_sim.hil.flight_computer  (reference FC as a child process)
    serve(ReferenceFlightComputer())


if __name__ == "__main__":
    main()
