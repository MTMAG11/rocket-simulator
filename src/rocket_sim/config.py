from dataclasses import dataclass
from pathlib import Path


@dataclass
class SimulationConfig:
    motor: Path
    rocket_dry_mass: float
    gravity: float
    dt: float