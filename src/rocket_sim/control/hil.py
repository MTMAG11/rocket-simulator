"""Hardware-in-the-loop (HIL) bridge: a controller that lives on the other side of a byte transport.

The simulator stays authoritative for physics; a *real flight computer* (or any external process) receives the
SIMULATED SENSOR readings, runs its own estimator/controller, and replies with actuator commands:

    SIMULATION -> sensors -> [ transport ] -> REAL FLIGHT COMPUTER -> [ transport ] -> command -> actuator -> SIMULATION

Wire protocol (newline-delimited JSON, one request/response per control tick):

    request : {"t": s, "phase": int, "accel": [3], "gyro": [3], "baro_pa": x, "gps": [6], "mag": [3],
               "new": {"accel": bool, "gyro": bool, "baro": bool, "gps": bool, "mag": bool}}
    response: {"tvc_y": rad, "tvc_z": rad}

``transport`` is any object with ``send(bytes)`` and ``recv() -> bytes`` (a serial port, socket or pipe wrapper).
This module implements the *plumbing* and a deterministic lock-step schedule (the simulation waits for the reply).
Real-time pacing, latency injection beyond the actuator ``delay_s`` and timeout handling are NOT implemented: for
non-real-time (lock-step) HIL this is sufficient and exactly reproducible; real-time HIL would add a pacing layer.
"""

from __future__ import annotations

import json
from typing import Any, Protocol

from ..errors import SimulationError
from .actuators import Command
from .controllers import ControlInput, Controller


class Transport(Protocol):
    def send(self, data: bytes) -> None: ...

    def recv(self) -> bytes: ...


class HilController(Controller):
    """Forward raw sensor readings to an external flight computer and return its commands."""

    name = "hil"

    def __init__(self, transport: Transport) -> None:
        self.transport = transport

    def update(self, inp: ControlInput) -> Command:
        rd = inp.sensors
        if rd is None:
            raise SimulationError("HIL controller needs sensors: run at fidelity >= 4")
        msg: dict[str, Any] = {
            "t": inp.t,
            "phase": inp.phase,
            "accel": [float(x) for x in rd.accel],
            "gyro": [float(x) for x in rd.gyro],
            "baro_pa": float(rd.baro_pressure),
            "gps": [float(x) for x in rd.gps],
            "mag": [float(x) for x in rd.mag],
            "new": {
                "accel": rd.accel_new,
                "gyro": rd.gyro_new,
                "baro": rd.baro_new,
                "gps": rd.gps_new,
                "mag": rd.mag_new,
            },
        }
        self.transport.send((json.dumps(msg) + "\n").encode())
        try:
            reply = json.loads(self.transport.recv().decode())
            return Command(float(reply["tvc_y"]), float(reply["tvc_z"]))
        except (ValueError, KeyError, TypeError) as exc:
            raise SimulationError(f"malformed reply from the flight computer: {exc}") from exc
