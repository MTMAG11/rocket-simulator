from dataclasses import dataclass


@dataclass
class RocketState:
    x: float = 0.0
    y: float = 0.0

    vx: float = 0.0
    vy: float = 0.0

    ax: float = 0.0
    ay: float = 0.0