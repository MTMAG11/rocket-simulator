"""The simulator-side bridge: a ``Controller`` that forwards sensor SAMPLES to a flight computer and returns its commands.

    simulator tick -> [uplink latency] -> tick message (all new samples) -> flight computer -> command message
                   -> (compute time + downlink latency, applied by the simulator before the actuator sees it)

Consumes NO state: ``consumes_state = False``, so the simulator hands it a ControlInput with ``valid=False`` and zeroed
state fields (it cannot read truth even by mistake) and does not warn about a truth-fed controller. Timing is explicit:
the controller rate (``controller.rate_hz``) sets the tick period; ``controller.uplink_latency_s`` delays when a sample
becomes available to the flight computer; ``controller.compute_time_s + controller.downlink_latency_s`` delay when the command
reaches the actuator (see docs/hil.md for the full timing model).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from ..control.actuators import Command
from ..control.controllers import ControlInput, Controller
from ..errors import ConfigError, SimulationError
from .flight_computer import ReferenceFlightComputer, handle_message
from .protocol import PROTOCOL_VERSION, command_from_reply, decode, encode
from .transport import LoopbackTransport, PipeTransport, RecordingTransport, ReplayTransport, Transport


class HilBridgeController(Controller):
    name = "hil"
    wants_samples = True
    consumes_state = False

    def __init__(self, transport: Transport, uplink_latency_s: float | None = None) -> None:
        if uplink_latency_s is not None and not (math.isfinite(uplink_latency_s) and uplink_latency_s >= 0):
            raise ConfigError(f"uplink_latency_s must be a finite number >= 0, got {uplink_latency_s}")
        self.transport = transport
        # None: take controller.uplink_latency_s from the simulation config (the simulator fills it in at start)
        self.uplink_latency_s: float | None = uplink_latency_s
        self._pending: list[dict[str, Any]] = []
        self._seq = 0
        self.ticks: list[dict[str, Any]] = []  # per-tick log: seq, t, n_samples, command

    def reset(self, ctx: dict[str, Any]) -> None:
        init = ctx.get("hil_init")
        if init is None:
            raise SimulationError("HilBridgeController needs a simulation that provides the HIL init data")
        self.transport.send(encode({"v": PROTOCOL_VERSION, "type": "init", **init}))
        reply = decode(self.transport.recv())
        if reply.get("type") != "ready":
            raise SimulationError(f"flight computer did not acknowledge init: {reply}")
        self._pending, self._seq, self.ticks = [], 0, []

    def finish(self) -> None:
        """Post-run completeness check (replay transports verify that every recorded message was used)."""
        fin = getattr(self.transport, "finish", None)
        if fin is not None:
            fin()

    def close(self) -> None:
        close = getattr(self.transport, "close", None)
        if close is not None:
            close()

    def update(self, inp: ControlInput) -> Command:
        self._pending.extend(inp.samples)
        up = self.uplink_latency_s or 0.0
        due = [s for s in self._pending if s["t_visible"] + up <= inp.t + 1e-12]
        self._pending = [s for s in self._pending if s["t_visible"] + up > inp.t + 1e-12]
        self._seq += 1
        self.transport.send(
            encode({"v": PROTOCOL_VERSION, "type": "tick", "seq": self._seq, "t": inp.t, "samples": due})
        )
        cmd = command_from_reply(decode(self.transport.recv()), self._seq)
        self.ticks.append({"seq": self._seq, "t": inp.t, "n_samples": len(due), "cmd": cmd})
        return Command(cmd["tvc_y"], cmd["tvc_z"], cmd["fin_pitch"], cmd["fin_yaw"], cmd["fin_roll"])


def build_bridge(params: dict[str, Any]) -> HilBridgeController:
    """``controller: {type: hil, params: {fc: reference | command: [...] | replay: file.jsonl, record: file.jsonl}}``."""
    uplink = float(params["uplink_latency_s"]) if "uplink_latency_s" in params else None
    transport: Transport
    if "replay" in params:
        transport = ReplayTransport.from_jsonl(Path(params["replay"]))
    elif "command" in params:
        transport = PipeTransport([str(c) for c in params["command"]])
    elif params.get("fc", "reference") == "reference":
        fc = ReferenceFlightComputer(**params.get("fc_kwargs", {}))
        transport = LoopbackTransport(lambda b: encode(handle_message(fc, decode(b))))
    else:
        raise ConfigError(
            "controller.params for type 'hil': give fc: reference, command: [...] or replay: file"
        )
    if params.get("record"):
        transport = RecordingTransport(transport)
    return HilBridgeController(transport, uplink)
