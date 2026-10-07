"""Hardware-in-the-loop foundation: a strict simulator <-> flight-computer boundary.

    SIMULATION (truth) -> sensor models -> [ protocol v2 over a Transport ] -> FLIGHT COMPUTER -> command -> actuator -> SIMULATION

Only SENSOR SAMPLES (with their timestamps) cross to the flight computer; only COMMANDS come back. No truth state crosses.
See docs/hil.md for the protocol, the timing model and what is and is not implemented.
"""

from .bridge import HilBridgeController, build_bridge
from .flight_computer import FlightComputer, ReferenceFlightComputer, handle_message, serve
from .protocol import PROTOCOL_VERSION, decode, encode
from .transport import LoopbackTransport, PipeTransport, RecordingTransport, ReplayTransport, Transport

__all__ = [
    "PROTOCOL_VERSION",
    "FlightComputer",
    "HilBridgeController",
    "LoopbackTransport",
    "PipeTransport",
    "RecordingTransport",
    "ReferenceFlightComputer",
    "ReplayTransport",
    "Transport",
    "build_bridge",
    "decode",
    "encode",
    "handle_message",
    "serve",
]
