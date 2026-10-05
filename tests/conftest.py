"""Shared fixtures and helpers for the test-suite.

Analytic tests drive the dynamics objects directly so that the expected answers come from
closed-form physics, never from the simulator itself.
"""

from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import pytest

from rocket_sim.config import SimConfig, config_from_dict, load_config
from rocket_sim.environment import (
    Atmosphere,
    AtmState,
    ConstantGravity,
    Environment,
    FlatTerrain,
    NoWind,
)
from rocket_sim.motor import Motor
from rocket_sim.physics.dynamics import LaunchSetup, PointMass3DOF, RigidBody6DOF
from rocket_sim.physics.math3d import quat_from_pointing
from rocket_sim.vehicle import (
    BodyTube,
    ConstantAero,
    MassComponent,
    MassModel,
    Vehicle,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG_G80 = ROOT / "configs" / "example_g80.yaml"


class ConstantAtmosphere(Atmosphere):
    """Fixed-density atmosphere for analytic drag tests (T=288.15 K, rho given)."""

    name = "const"

    def __init__(self, rho: float = 1.225) -> None:
        self.rho = rho

    def at(self, h_msl: float) -> AtmState:
        return AtmState(288.15, self.rho * 287.05287 * 288.15, self.rho, 340.294, 1.789e-5)


def tiny_motor(thrust: float = 1e-9, burn: float = 1.0, prop: float = 1e-9, total: float = 2e-9) -> Motor:
    """Effectively inert motor: negligible thrust and propellant, so mass/inertia stay constant."""
    return Motor("TEST", 0.029, 0.1, prop, total, [0.0, burn], [thrust, thrust])


def make_vehicle(
    motor: Motor,
    dry_mass: float = 1.0,
    aero=None,
    length: float = 1.0,
    diameter: float = 0.05,
    cg: float = 0.5,
    ixx: float = 0.002,
    iyy: float = 0.08,
    nozzle_x: float | None = None,
) -> Vehicle:
    body = BodyTube(diameter, length)
    aero = aero or ConstantAero(diameter, 0.0)
    nozzle_x = length if nozzle_x is None else nozzle_x
    mx = nozzle_x - 0.5 * motor.length
    comps = [
        MassComponent("body", dry_mass, cg, ixx, iyy),
        MassComponent("casing", motor.casing_mass, mx),
    ]
    mm = MassModel(comps, mx, 0.5 * motor.diameter, motor.length)
    return Vehicle("test", body, aero, mm, motor, nozzle_x)


def make_env(rho: float | None = None, g: float = 9.81) -> Environment:
    from rocket_sim.environment import ISAAtmosphere

    atm = ConstantAtmosphere(rho) if rho is not None else ISAAtmosphere()
    return Environment(atm, ConstantGravity(g), NoWind(), FlatTerrain(0.0), 0.0)


def free_launch(z0: float = 0.0, elevation: float = np.pi / 2, azimuth: float = 0.0) -> LaunchSetup:
    return LaunchSetup((0.0, 0.0, z0), quat_from_pointing(elevation, azimuth), 0.0, on_rail=False)


def make_3dof(vehicle: Vehicle, env: Environment, launch: LaunchSetup, **kw) -> PointMass3DOF:
    return PointMass3DOF(vehicle, env, launch, **kw)


def make_6dof(vehicle: Vehicle, env: Environment, launch: LaunchSetup) -> RigidBody6DOF:
    return RigidBody6DOF(vehicle, env, launch)


@pytest.fixture(scope="session")
def base_config() -> SimConfig:
    return load_config(CONFIG_G80)


@pytest.fixture
def cfg(base_config: SimConfig) -> SimConfig:
    return copy.deepcopy(base_config)


def _deep_merge(base: dict, extra: dict) -> dict:
    out = dict(base)
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def cfg_from(**sections) -> SimConfig:
    """Build a validated config from the example by deep-merging ``sections`` over it."""
    from rocket_sim.config import read_mapping

    data = _deep_merge(read_mapping(CONFIG_G80), sections)
    return config_from_dict(data, base_dir=CONFIG_G80.parent)
