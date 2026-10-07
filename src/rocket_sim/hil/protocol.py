"""Wire protocol v2: newline-delimited JSON, one message per line, deterministic encoding.

Messages (all carry ``"v": 2``)::

    simulator -> FC   {"type": "init", "controller_rate_hz": f, "sensors": {...nominal specs...}, "design": {...},
                       "estimator": {...}}
    FC -> simulator   {"type": "ready"}
    simulator -> FC   {"type": "tick", "seq": n, "t": s,
                       "samples": [{"sensor": "accel|gyro|baro|gps|mag", "t_sample": s, "t_visible": s, "value": [..]}]}
    FC -> simulator   {"type": "command", "seq": n,
                       "cmd": {"tvc_y": rad, "tvc_z": rad, "fin_pitch": rad, "fin_yaw": rad, "fin_roll": rad}}

* ``samples`` holds EVERY sample that became available since the previous tick (not only the latest), each with the time
  it was taken and the time it became visible (sensor latency is already included in ``t_visible``).
* Values: accel [m/s^2] and gyro [rad/s] in body axes; baro [Pa] as a 1-vector; gps [x, y, z, vx, vy, vz] in the launch
  frame (m, m/s); mag [T] in body axes. These are MEASUREMENTS (noise, bias, quantisation applied).
* ``design`` is DESIGN data the flight software legitimately has (nominal gimbal authority schedule vs time since ignition,
  gravity, site elevation, launch axis): never the true state.
* Deterministic encoding: sorted keys, no whitespace, NaN/Infinity rejected. The same message is the same bytes, which is
  what makes recorded sessions replayable bit-for-bit.
"""

from __future__ import annotations

import json
import math
from typing import Any

from ..errors import SimulationError

PROTOCOL_VERSION = 2
COMMAND_KEYS = ("tvc_y", "tvc_z", "fin_pitch", "fin_yaw", "fin_roll")
SENSOR_NAMES = ("accel", "gyro", "baro", "gps", "mag")


def encode(msg: dict[str, Any]) -> bytes:
    try:
        return (json.dumps(msg, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode(
            "utf-8"
        )
    except ValueError as exc:
        raise SimulationError(f"cannot encode HIL message (non-finite number?): {exc}") from exc


def decode(data: bytes) -> dict[str, Any]:
    try:
        msg = json.loads(data.decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as exc:
        raise SimulationError(f"malformed HIL message: {exc}") from exc
    if not isinstance(msg, dict):
        raise SimulationError("HIL message must be a JSON object")
    if msg.get("v") != PROTOCOL_VERSION:
        raise SimulationError(f"HIL protocol version {msg.get('v')!r} != {PROTOCOL_VERSION}")
    return msg


def command_from_reply(reply: dict[str, Any], seq: int) -> dict[str, float]:
    if reply.get("type") != "command" or reply.get("seq") != seq:
        raise SimulationError(
            f"expected a command reply for tick {seq}, got {reply.get('type')!r}/{reply.get('seq')!r}"
        )
    cmd = reply.get("cmd")
    if not isinstance(cmd, dict):
        raise SimulationError("command reply has no 'cmd' object")
    unknown = set(cmd) - set(COMMAND_KEYS)
    if unknown:
        raise SimulationError(f"unknown command field(s) {sorted(unknown)}")
    out = {k: 0.0 for k in COMMAND_KEYS}
    for k, v in cmd.items():
        if isinstance(v, bool) or not isinstance(v, int | float) or not math.isfinite(v):
            raise SimulationError(f"command field {k!r} must be a finite number, got {v!r}")
        out[k] = float(v)
    return out
