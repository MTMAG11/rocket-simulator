"""Environment models: atmosphere, gravity, wind, terrain."""

from dataclasses import dataclass

from .atmosphere import (
    Atmosphere,
    AtmState,
    ExponentialAtmosphere,
    ISAAtmosphere,
    TableAtmosphere,
)
from .gravity import ConstantGravity, Gravity, InverseSquareGravity
from .terrain import FlatTerrain, GridTerrain, SlopeTerrain, Terrain
from .wind import (
    CompositeWind,
    ConstantWind,
    GustWind,
    NoWind,
    PowerLawWind,
    ProfileWind,
    TurbulenceWind,
    Wind,
)


@dataclass
class Environment:
    """Bundle of environment models plus the launch-site elevation above MSL [m]."""

    atmosphere: Atmosphere
    gravity: Gravity
    wind: Wind
    terrain: Terrain
    site_elevation: float = 0.0


__all__ = [
    "AtmState",
    "Atmosphere",
    "CompositeWind",
    "ConstantGravity",
    "ConstantWind",
    "Environment",
    "ExponentialAtmosphere",
    "FlatTerrain",
    "Gravity",
    "GridTerrain",
    "GustWind",
    "ISAAtmosphere",
    "InverseSquareGravity",
    "NoWind",
    "PowerLawWind",
    "ProfileWind",
    "SlopeTerrain",
    "TableAtmosphere",
    "Terrain",
    "TurbulenceWind",
    "Wind",
]
