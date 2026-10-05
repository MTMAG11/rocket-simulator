"""Flight-phase state machine (truth-based; the flight computer's own detections live in the
estimator). Every transition is logged as a ``phase_change`` event by the simulator.

PRELAUNCH --t >= ignition--> IGNITION --liftoff--> POWERED_ASCENT --motor end--> BURNOUT -> COAST
COAST/POWERED_ASCENT --vertical velocity <= 0--> APOGEE -> DESCENT;  any --ground contact--> LANDED

BURNOUT and APOGEE are single-row transient phases: they are held for the recorded row at the
event time and advance on the next row. Subclass ``PhaseMachine`` and override ``transitions`` to
change the logic (thresholds are configurable via ``phases:`` in the config).
"""

from __future__ import annotations

from dataclasses import dataclass

from .record import FlightPhase


@dataclass
class PhaseInputs:
    t: float
    ignited: bool
    liftoff: bool
    burnout: bool
    apogee: bool
    landed: bool


class PhaseMachine:
    def __init__(self) -> None:
        self.phase = FlightPhase.PRELAUNCH

    def update(self, p: PhaseInputs) -> FlightPhase | None:
        """Evaluate transitions; returns the new phase if it changed."""
        new = self.transitions(p)
        if new is not None and new != self.phase:
            self.phase = new
            return new
        return None

    def advance_transient(self) -> FlightPhase | None:
        """Called after a row is recorded: BURNOUT -> COAST, APOGEE -> DESCENT."""
        if self.phase == FlightPhase.BURNOUT:
            self.phase = FlightPhase.COAST
            return self.phase
        if self.phase == FlightPhase.APOGEE:
            self.phase = FlightPhase.DESCENT
            return self.phase
        return None

    def transitions(self, p: PhaseInputs) -> FlightPhase | None:
        ph = self.phase
        if ph == FlightPhase.LANDED:
            return None
        if p.landed:
            return FlightPhase.LANDED
        if ph == FlightPhase.PRELAUNCH and p.ignited:
            return FlightPhase.IGNITION
        if ph == FlightPhase.IGNITION:
            if p.liftoff:
                return FlightPhase.POWERED_ASCENT
            if p.burnout:
                return FlightPhase.BURNOUT
        if ph == FlightPhase.POWERED_ASCENT:
            if p.apogee:
                return FlightPhase.APOGEE
            if p.burnout:
                return FlightPhase.BURNOUT
        if ph == FlightPhase.COAST and p.apogee:
            return FlightPhase.APOGEE
        return None
