from dataclasses import dataclass


@dataclass
class SimulationConfig:
    motor: str
    rocket_dry_mass: float
    gravity: float
    dt: float