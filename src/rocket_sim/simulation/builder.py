"""Build vehicle / environment / dynamics objects from a validated SimConfig.

Also resolves the fidelity level (``resolve_fidelity``) into concrete feature flags and applies
the documented overrides, recording every override so datasets stay interpretable.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field

import numpy as np

from ..config import SimConfig, resolve_path
from ..config.schema import RocketCfg
from ..environment import (
    Atmosphere,
    CompositeWind,
    ConstantGravity,
    ConstantWind,
    Environment,
    ExponentialAtmosphere,
    FlatTerrain,
    GustWind,
    InverseSquareGravity,
    ISAAtmosphere,
    NoWind,
    PowerLawWind,
    ProfileWind,
    SlopeTerrain,
    TableAtmosphere,
    TurbulenceWind,
    Wind,
)
from ..errors import ConfigError
from ..motor import Motor, load_motor
from ..physics.dynamics import LaunchSetup, PointMass3DOF, RigidBody6DOF
from ..physics.math3d import quat_from_pointing
from ..vehicle import (
    BodyTube,
    BuildupAero,
    ConstantAero,
    FinSet,
    MassComponent,
    MassModel,
    NoseCone,
    Parachute,
    TableAero,
    Vehicle,
)
from ..vehicle.aero import AerodynamicModel
from ..vehicle.mass import solid_cylinder_inertia, thin_tube_inertia


@dataclass
class FidelityFlags:
    dof: int  # 1, 3 or 6
    aero: bool
    wind: bool
    sensors: bool
    estimator: bool
    controller: bool
    vertical_only: bool
    notes: list[str] = field(default_factory=list)


def resolve_fidelity(cfg: SimConfig) -> tuple[SimConfig, FidelityFlags]:
    """Apply fidelity-level rules. Returns (effective config copy, flags)."""
    c = copy.deepcopy(cfg)
    if hasattr(cfg, "_base_dir"):
        c._base_dir = cfg._base_dir  # type: ignore[attr-defined]
    lvl = cfg.fidelity
    notes: list[str] = []
    flags = FidelityFlags(
        dof=6 if lvl >= 3 else (1 if lvl == 0 else 3),
        aero=lvl >= 2,
        wind=lvl >= 2,
        sensors=lvl >= 4,
        estimator=lvl >= 5,
        controller=lvl >= 3,  # open-loop schedules/TVC allowed from level 3
        vertical_only=lvl == 0,
        notes=notes,
    )
    if lvl == 0:
        if c.launch.elevation_deg != 90.0:
            notes.append("fidelity 0 is vertical-only: launch.elevation_deg forced to 90")
            c.launch.elevation_deg = 90.0
        if c.environment.gravity.model != "constant":
            notes.append("fidelity 0 uses constant gravity")
            c.environment.gravity.model = "constant"
    if lvl < 5:
        if c.estimator.type != "none":
            notes.append(f"fidelity {lvl}: estimator disabled")
        c.estimator.type = "none"
    if lvl < 3 and c.controller.type != "none":
        notes.append(f"fidelity {lvl}: controller disabled")
        c.controller.type = "none"
    flags.estimator = lvl >= 5 and c.estimator.type != "none"
    flags.controller = lvl >= 3 and c.controller.type != "none"
    if cfg.fast:
        c.environment.atmosphere.model = "exponential"
        notes.append("fast mode: exponential atmosphere, Cd(Mach) lookup table (no Re dependence)")
    if lvl == 6:
        if c.simulation.dt_s > 0.005:
            notes.append("fidelity 6: dt reduced to 0.005 s")
            c.simulation.dt_s = 0.005
        if c.simulation.integrator != "rk4":
            notes.append("fidelity 6: integrator forced to rk4")
            c.simulation.integrator = "rk4"
        if c.environment.atmosphere.model != "isa":
            notes.append("fidelity 6: ISA atmosphere (table allowed)")
            if c.environment.atmosphere.model != "table":
                c.environment.atmosphere.model = "isa"
        c.environment.gravity.model = "inverse_square"
    return c, flags


def build_motor(cfg: SimConfig) -> Motor:
    m = cfg.motor
    path = resolve_path(cfg, m.file)
    motor = load_motor(path, m.designation)
    if m.thrust_scale != 1.0 or m.burn_time_scale != 1.0:
        motor = motor.scaled(m.thrust_scale, m.burn_time_scale)
    return motor


def _build_aero(
    r: RocketCfg, body: BodyTube, nose: NoseCone, fins: FinSet, motor: Motor, fast: bool
) -> AerodynamicModel:
    a = r.aero
    planform = body.diameter * (body.length - nose.length + 0.5 * nose.length)
    if a.model == "constant":
        assert a.cd is not None
        return ConstantAero(
            body.diameter,
            a.cd * a.drag_scale + a.extra_cd,
            a.cn_alpha or 0.0,
            a.x_cp_from_nose_m or 0.0,
            a.crossflow_cd,
            planform,
            0.5 * body.length,
        )
    if a.model == "table":
        assert a.cd_table is not None
        mach = [row[0] for row in a.cd_table]
        cd = [row[1] * a.drag_scale + a.extra_cd for row in a.cd_table]
        cdp = None
        if a.cd_powered_table:
            cdp = [row[1] * a.drag_scale + a.extra_cd for row in a.cd_powered_table]
        return TableAero(
            body.diameter,
            mach,
            cd,
            a.cn_alpha or 0.0,
            a.x_cp_from_nose_m or 0.0,
            cdp,
            a.crossflow_cd,
            planform,
            0.5 * body.length,
        )
    model = BuildupAero(
        body,
        nose,
        fins,
        nozzle_exit_diameter=a.nozzle_exit_ratio * motor.diameter,
        surface_roughness=a.surface_roughness_m,
        crossflow_cd=a.crossflow_cd,
        extra_cd=a.extra_cd,
        drag_scale=a.drag_scale,
    )
    if fast:
        # Mach-dependent Cd table evaluated once at a fixed representative air state (rho = 1.0 kg/m^3,
        # T = 270 K): no Reynolds/altitude dependence at run time, so evaluation is a cheap lookup.
        machs = [round(0.05 * i, 2) for i in range(0, 61)]
        cds = []
        for m in machs:
            v = max(m, 0.02) * math.sqrt(1.4 * 287.05 * 270.0)
            cds.append(model.coefficients(m, 1.0 * v * body.length / 1.75e-5, False).cd0)
        c0 = model.coefficients(0.3, 1e7, False)
        return TableAero(
            body.diameter,
            machs,
            cds,
            c0.cn_alpha,
            c0.x_cp,
            None,
            a.crossflow_cd,
            model.planform_area,
            model.x_crossflow,
        )
    return model


def build_vehicle(cfg: SimConfig, motor: Motor | None = None, fast: bool | None = None) -> Vehicle:
    r = cfg.rocket
    motor = motor if motor is not None else build_motor(cfg)
    fast = cfg.fast if fast is None else fast
    body = BodyTube(r.body_diameter_m, r.body_length_m)
    nose = NoseCone(r.nose.shape, r.nose.length_m)
    f = r.fins
    fins = FinSet(
        f.count, f.root_chord_m, f.tip_chord_m, f.span_m, f.sweep_m, f.thickness_m, f.position_from_nose_m
    )
    nozzle_x = r.motor_aft_from_nose_m if r.motor_aft_from_nose_m is not None else r.body_length_m
    if motor.diameter > body.diameter + 1e-9:
        raise ConfigError(
            f"motor {motor.designation} (diameter {motor.diameter * 1000:.0f} mm) does not fit in a "
            f"{body.diameter * 1000:.0f} mm body"
        )
    if motor.length > nozzle_x:
        raise ConfigError(
            f"motor length {motor.length * 1000:.0f} mm exceeds motor_aft_from_nose_m={nozzle_x} m"
        )
    mx = nozzle_x - 0.5 * motor.length  # motor centre
    if r.inertia is not None:
        ixx, iyy = r.inertia.ixx_kgm2, r.inertia.iyy_kgm2
    else:
        ixx, iyy = thin_tube_inertia(r.dry_mass_kg, body.radius, body.length)
    comps = [MassComponent("airframe", r.dry_mass_kg, r.cg_from_nose_m, ixx, iyy)]
    if r.payload is not None and r.payload.mass_kg > 0:
        comps.append(MassComponent("payload", r.payload.mass_kg, r.payload.position_from_nose_m))
    cixx, ciyy = thin_tube_inertia(motor.casing_mass, 0.5 * motor.diameter, motor.length)
    comps.append(MassComponent("motor_casing", motor.casing_mass, mx, cixx, ciyy))
    mass_model = MassModel(comps, mx, 0.5 * motor.diameter * 0.9, motor.length * 0.9)
    aero = _build_aero(r, body, nose, fins, motor, fast)
    chutes = []
    for p in r.parachutes:
        attach = p.attach_from_nose_m if p.attach_from_nose_m is not None else r.nose.length_m
        chutes.append(
            Parachute(p.cd, p.diameter_m, p.trigger, p.altitude_m, p.delay_s, p.inflation_time_s, attach)
        )
    mis = (math.radians(cfg.motor.misalignment_deg[0]), math.radians(cfg.motor.misalignment_deg[1]))
    return Vehicle(r.name, body, aero, mass_model, motor, nozzle_x, mis, cfg.motor.ignition_delay_s, chutes)


def build_wind(cfg: SimConfig, seed: np.random.SeedSequence) -> Wind:
    w = cfg.environment.wind
    parts: list[Wind] = []
    if w.model == "constant":
        parts.append(ConstantWind(w.speed_ms, w.direction_from_deg))
    elif w.model == "profile":
        assert w.profile is not None
        parts.append(ProfileWind([(r[0], r[1], r[2]) for r in w.profile]))
    elif w.model == "power_law":
        parts.append(PowerLawWind(w.speed_ms, w.direction_from_deg, w.ref_height_m, w.exponent))
    if w.turbulence_sigma_ms > 0:
        rng_seed = int(seed.generate_state(1)[0])
        parts.append(TurbulenceWind(w.turbulence_sigma_ms, w.turbulence_tau_s, rng_seed))
    for g in w.gusts:
        parts.append(GustWind(g.start_s, g.duration_s, g.speed_ms, g.direction_from_deg))
    if not parts:
        return NoWind()
    return parts[0] if len(parts) == 1 else CompositeWind(parts)


def build_environment(cfg: SimConfig, wind_seed: np.random.SeedSequence) -> Environment:
    e = cfg.environment
    a = e.atmosphere
    atm: Atmosphere
    if a.model == "isa":
        atm = ISAAtmosphere(a.temperature_offset_k, a.sea_level_pressure_pa)
    elif a.model == "exponential":
        atm = ExponentialAtmosphere(
            rho0=1.225 * (a.sea_level_pressure_pa / 101325.0) * (288.15 / (288.15 + a.temperature_offset_k)),
            scale_height=a.scale_height_m,
            t0=288.15 + a.temperature_offset_k,
        )
    else:
        assert a.table is not None
        atm = TableAtmosphere([r[0] for r in a.table], [r[1] for r in a.table], [r[2] for r in a.table])
    grav = InverseSquareGravity() if e.gravity.model == "inverse_square" else ConstantGravity(e.gravity.g_ms2)
    t = e.terrain
    terrain = (
        SlopeTerrain(t.slope_deg, t.slope_direction_deg) if t.model == "slope" else FlatTerrain(t.height_m)
    )
    return Environment(atm, grav, build_wind(cfg, wind_seed), terrain, e.site_elevation_msl_m)


def build_launch(cfg: SimConfig, terrain_height: float = 0.0) -> LaunchSetup:
    q = quat_from_pointing(
        math.radians(cfg.launch.elevation_deg),
        math.radians(cfg.launch.azimuth_deg),
        math.radians(cfg.launch.roll_deg),
    )
    return LaunchSetup((0.0, 0.0, terrain_height), q, cfg.launch.rail_length_m, True)


def build_dynamics(cfg: SimConfig, flags: FidelityFlags, vehicle: Vehicle, env: Environment):
    launch = build_launch(cfg, env.terrain.height(0.0, 0.0))
    if flags.dof == 6:
        return RigidBody6DOF(vehicle, env, launch)
    return PointMass3DOF(
        vehicle,
        env,
        launch,
        aero_enabled=flags.aero,
        wind_enabled=flags.wind,
        vertical_only=flags.vertical_only,
    )


def estimate_inertia_note(r: RocketCfg) -> str | None:
    if r.inertia is None:
        return "inertia not provided: thin-tube estimate used (provide measured/CAD values)"
    return None


__all__ = [
    "FidelityFlags",
    "build_dynamics",
    "build_environment",
    "build_launch",
    "build_motor",
    "build_vehicle",
    "build_wind",
    "estimate_inertia_note",
    "resolve_fidelity",
    "solid_cylinder_inertia",
]
