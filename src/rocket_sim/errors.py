"""Exception hierarchy. Every failure is explicit; nothing is silently clamped."""


class RocketSimError(Exception):
    """Base class for all simulator errors."""


class ConfigError(RocketSimError, ValueError):
    """Invalid or inconsistent configuration (message includes the config path)."""


class MotorError(RocketSimError, ValueError):
    """Malformed or physically implausible motor data."""


class SimulationError(RocketSimError, RuntimeError):
    """Numerical failure during a run (non-finite state, divergence)."""


class DataQualityError(RocketSimError, ValueError):
    """A simulation or dataset failed quality checks."""
