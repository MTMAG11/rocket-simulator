"""rocket_sim: physics-based rocket flight simulation and data-generation platform."""

from .version import PHYSICS_VERSION, SCHEMA_VERSION, SIM_VERSION

__version__ = SIM_VERSION
__all__ = ["PHYSICS_VERSION", "SCHEMA_VERSION", "SIM_VERSION", "__version__"]
