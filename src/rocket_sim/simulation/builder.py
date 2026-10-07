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
    ConstantAero,
    FinSet,
    MassComponent,
    MassModel,
    NoseCone,
    Parachute,
    TableAero,
    Vehicle,
)
from ..vehicle.aero import (
    AerodynamicModel,
    BarrowmanAero,
    EnhancedAero,
    SimplifiedAero,
    Table2DAero,
    default_provenance,
)
from ..vehicle.assembly import Assembly, ControlSurfaceSet, Section, legacy_assembly
from ..vehicle.mass import solid_cylinder_inertia, thin_tube_inertia
from ..vehicle.tables import load_table2d_csv


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
        if c.estimator.type not in ("none",) and not (c.estimator.type == "truth" and lvl >= 3):
            notes.append(f"fidelity {lvl}: estimator disabled")
        if not (c.estimator.type == "truth" and lvl >= 3):
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


def _build_assembly(cfg: SimConfig) -> Assembly:
    """Airframe geometry: component sections if given, else the V1 nose + one body tube."""
    r = cfg.rocket
    f = r.fins
    fins = FinSet(
        f.count, f.root_chord_m, f.tip_chord_m, f.span_m, f.sweep_m, f.thickness_m, f.position_from_nose_m
    )
    controls = []
    if r.control_surfaces is not None:
        c = r.control_surfaces
        controls.append(
            ControlSurfaceSet(
                c.count,
                c.root_chord_m,
                c.tip_chord_m,
                c.span_m,
                c.sweep_m,
                c.thickness_m,
                c.position_from_nose_m,
                math.radians(c.roll_angle0_deg),
                math.radians(c.max_deflection_deg),
                math.radians(c.max_rate_deg_s),
                c.time_constant_s,
                c.delay_s,
                c.mass_kg,
            )
        )
    ref = r.reference_diameter_m
    if not r.sections:
        return (
            legacy_assembly(
                BodyTube(r.body_diameter_m, r.body_length_m), NoseCone(r.nose.shape, r.nose.length_m), fins
            )
            if not controls and ref is None
            else _legacy_with(r, fins, controls, ref)
        )
    secs: list[Section] = []
    x = 0.0
    prev_d = 0.0
    for i, sc in enumerate(r.sections):
        kind = "transition" if sc.type == "boattail" else sc.type
        if kind == "nose":
            d_aft = sc.diameter_m if sc.diameter_m is not None else _next_diameter(r.sections, i)
            sec = Section("nose", x, sc.length_m, 0.0, d_aft, sc.shape, sc.mass_kg, sc.cg_fraction)
        elif kind == "body":
            d = sc.diameter_m
            assert d is not None
            sec = Section("body", x, sc.length_m, d, d, "ogive", sc.mass_kg, sc.cg_fraction)
        else:
            d_fore = sc.fore_diameter_m if sc.fore_diameter_m is not None else prev_d
            assert sc.aft_diameter_m is not None
            sec = Section(
                "transition", x, sc.length_m, d_fore, sc.aft_diameter_m, "ogive", sc.mass_kg, sc.cg_fraction
            )
        secs.append(sec)
        x += sc.length_m
        prev_d = sec.d_aft
    try:
        return Assembly(secs, fins, controls, ref)
    except ConfigError as exc:
        raise ConfigError(f"rocket.sections: {exc}") from exc


def _next_diameter(sections, i: int) -> float:
    for sc in sections[i + 1 :]:
        if sc.type == "body" and sc.diameter_m:
            return sc.diameter_m
        if sc.type in ("transition", "boattail") and sc.fore_diameter_m:
            return sc.fore_diameter_m
    raise ConfigError("rocket.sections[0] (nose): give diameter_m (no following body to take it from)")


def _legacy_with(r, fins, controls, ref) -> Assembly:
    base = legacy_assembly(
        BodyTube(r.body_diameter_m, r.body_length_m), NoseCone(r.nose.shape, r.nose.length_m), fins
    )
    return Assembly(list(base.sections), fins, controls, ref)


def _table_aero_from(
    model: BarrowmanAero, mach: list[float], cd: list[float], cdp, a, planform=True
) -> TableAero:
    """Lookup-table drag with the geometry-derived stability of ``model``."""
    c0 = model.coefficients(0.3, 1e7, False)
    return TableAero(
        model.ref_diameter,
        mach,
        cd,
        c0.cn_alpha,
        c0.x_cp,
        cdp,
        a.crossflow_cd,
        model.planform_area,
        model.x_crossflow,
        model.damping_surfaces,
        model.roll_damping_cn,
        model.roll_damping_radius,
        model.control_fins,
    )


def _build_aero(cfg: SimConfig, asm: Assembly, motor: Motor, fast: bool) -> AerodynamicModel:
    """Build the configured model and attach its PROVENANCE (what the numbers are, where they came from)."""
    a = cfg.rocket.aero
    m = _build_aero_model(cfg, asm, motor, fast)
    name = "barrowman" if a.model == "buildup" else a.model
    mach_range = None
    alpha_range = None
    if a.model == "table" and a.cd_table:
        ms = [row[0] for row in a.cd_table]
        mach_range = (min(ms), max(ms))
    elif a.model == "table2d":
        ms2 = getattr(m, "m", None)
        if ms2:
            mach_range = (float(min(ms2)), float(max(ms2)))
        als = getattr(m, "a", None)
        if als:
            alpha_range = (
                math.degrees(float(min(als))),
                math.degrees(float(max(als))),
            )  # beyond this the last row is HELD
    m.provenance = default_provenance(
        name,
        a.provenance,
        mach_range,
        alpha_range,
        stability_supplied=(a.cn_alpha is not None or a.x_cp_from_nose_m is not None),
    )
    return m


def _build_aero_model(cfg: SimConfig, asm: Assembly, motor: Motor, fast: bool) -> AerodynamicModel:
    a = cfg.rocket.aero
    noz = a.nozzle_exit_ratio * motor.diameter
    if noz >= asm.base_diameter:
        raise ConfigError(
            f"motor {motor.designation} nozzle (~{noz * 1000:.0f} mm) does not fit the base diameter "
            f"{asm.base_diameter * 1000:.0f} mm"
        )
    common = dict(
        surface_roughness=a.surface_roughness_m,
        crossflow_cd=a.crossflow_cd,
        extra_cd=a.extra_cd,
        drag_scale=a.drag_scale,
    )
    geo = BarrowmanAero(asm, noz, **common)  # geometry-derived stability for every model
    if a.model == "constant":
        assert a.cd is not None
        cn = a.cn_alpha if a.cn_alpha is not None else geo.cn_alpha_total
        xcp = a.x_cp_from_nose_m if a.x_cp_from_nose_m is not None else geo.x_cp_subsonic
        m = ConstantAero(
            asm.ref_diameter,
            a.cd * a.drag_scale + a.extra_cd,
            cn,
            xcp,
            a.crossflow_cd,
            geo.planform_area,
            geo.x_crossflow,
        )
        if a.cn_alpha is None:
            m.damping_surfaces, m.roll_damping_cn, m.roll_damping_radius = (
                geo.damping_surfaces,
                geo.roll_damping_cn,
                geo.roll_damping_radius,
            )
        m.control_fins = geo.control_fins
        return m
    if a.model == "table":
        assert a.cd_table is not None
        mach = [row[0] for row in a.cd_table]
        cd = [row[1] * a.drag_scale + a.extra_cd for row in a.cd_table]
        cdp = (
            [row[1] * a.drag_scale + a.extra_cd for row in a.cd_powered_table] if a.cd_powered_table else None
        )
        if a.cn_alpha is not None or a.x_cp_from_nose_m is not None:
            return TableAero(
                asm.ref_diameter,
                mach,
                cd,
                a.cn_alpha if a.cn_alpha is not None else geo.cn_alpha_total,
                a.x_cp_from_nose_m if a.x_cp_from_nose_m is not None else geo.x_cp_subsonic,
                cdp,
                a.crossflow_cd,
                geo.planform_area,
                geo.x_crossflow,
                geo.damping_surfaces,
                geo.roll_damping_cn,
                geo.roll_damping_radius,
                geo.control_fins,
            )
        return _table_aero_from(geo, mach, cd, cdp, a)
    if a.model == "table2d":
        assert a.table2d_file is not None
        t = load_table2d_csv(resolve_path(cfg, a.table2d_file))
        m2 = Table2DAero(
            asm.ref_diameter,
            t["mach"],
            t["alpha"],
            t["cd"],
            t["cl"],
            t["cm"],
            a.x_cm_ref_from_nose_m if a.x_cm_ref_from_nose_m is not None else geo.x_cp_subsonic,
            geo.x_cp_subsonic,
            geo.damping_surfaces,
            geo.roll_damping_cn,
            geo.roll_damping_radius,
        )
        m2.control_fins = geo.control_fins
        return m2
    if a.model == "simplified":
        assert a.cd is not None
        model: BarrowmanAero = SimplifiedAero(asm, a.cd, noz, **common)
    elif a.model == "enhanced":
        model = EnhancedAero(asm, noz, a.stall_angle_deg, **common)
    else:  # barrowman / buildup
        model = geo
    if fast:
        # Mach-dependent Cd table evaluated once at a fixed representative air state (rho = 1.0 kg/m^3,
        # T = 270 K): no Reynolds/altitude dependence at run time, so evaluation is a cheap lookup.
        machs = [round(0.05 * i, 2) for i in range(0, 61)]
        cds = []
        for m_ in machs:
            v = max(m_, 0.02) * math.sqrt(1.4 * 287.05 * 270.0)
            cds.append(model.coefficients(m_, 1.0 * v * asm.length / 1.75e-5, False).cd0)
        return _table_aero_from(model, machs, cds, None, a)
    return model


def _fin_set_inertia(mass: float, rho: float) -> tuple[float, float]:
    """(Ixx, Iyy) of a symmetric fin set of total mass m with centroid radius rho (thin plates)."""
    return mass * rho * rho, 0.5 * mass * rho * rho


def build_vehicle(cfg: SimConfig, motor: Motor | None = None, fast: bool | None = None) -> Vehicle:
    r = cfg.rocket
    motor = motor if motor is not None else build_motor(cfg)
    fast = cfg.fast if fast is None else fast
    asm = _build_assembly(cfg)
    length = asm.length
    body = BodyTube(asm.ref_diameter, length)
    notes: list[str] = []
    nozzle_x = r.motor_aft_from_nose_m if r.motor_aft_from_nose_m is not None else length
    if motor.diameter > asm.base_diameter + 1e-9 and motor.diameter > asm.ref_diameter + 1e-9:
        raise ConfigError(
            f"motor {motor.designation} (diameter {motor.diameter * 1000:.0f} mm) does not fit in a "
            f"{asm.ref_diameter * 1000:.0f} mm body"
        )
    if nozzle_x > length + 1e-9:
        raise ConfigError(f"motor_aft_from_nose_m={nozzle_x} m is beyond the airframe length {length} m")
    if motor.length > nozzle_x:
        raise ConfigError(
            f"motor length {motor.length * 1000:.0f} mm exceeds motor_aft_from_nose_m={nozzle_x} m"
        )
    mx = nozzle_x - 0.5 * motor.length  # motor centre
    comps: list[MassComponent] = []
    if r.sections:
        for sec, scfg in zip(asm.sections, r.sections, strict=True):
            if sec.mass > 0:
                ixx, iyy = thin_tube_inertia(sec.mass, sec.mean_radius, sec.length)
                comps.append(MassComponent(scfg.name or sec.kind, sec.mass, sec.x_cg, ixx, iyy))
                notes.append(f"inertia of the {sec.kind} section is a thin-shell ESTIMATE")
        if r.fins.mass_kg > 0 and r.fins.count:
            f_ = asm.fins
            xf = f_.leading_edge_from_nose + (
                f_.sweep / 3.0 * (f_.root_chord + 2 * f_.tip_chord) / (f_.root_chord + f_.tip_chord)
                + 0.5 * f_.mean_chord
            )
            rho = asm.local_radius(f_.leading_edge_from_nose) + 0.5 * f_.span
            ixx, iyy = _fin_set_inertia(r.fins.mass_kg, rho)
            comps.append(MassComponent("fins", r.fins.mass_kg, xf, ixx, iyy))
        for cs in asm.controls:
            if cs.mass > 0:
                rho = asm.local_radius(cs.leading_edge_from_nose) + 0.5 * cs.span
                ixx, iyy = _fin_set_inertia(cs.mass, rho)
                comps.append(
                    MassComponent(
                        "control_surfaces", cs.mass, cs.leading_edge_from_nose + 0.5 * cs.root_chord, ixx, iyy
                    )
                )
        if r.dry_mass_kg:
            assert r.cg_from_nose_m is not None
            if r.inertia is not None:
                ixx, iyy = r.inertia.ixx_kgm2, r.inertia.iyy_kgm2
            else:
                ixx, iyy = thin_tube_inertia(r.dry_mass_kg, 0.5 * asm.ref_diameter, length)
                notes.append("inertia of the lumped airframe mass is a thin-tube ESTIMATE")
            comps.append(MassComponent("airframe", r.dry_mass_kg, r.cg_from_nose_m, ixx, iyy))
    else:
        assert r.dry_mass_kg is not None and r.cg_from_nose_m is not None
        if r.inertia is not None:
            ixx, iyy = r.inertia.ixx_kgm2, r.inertia.iyy_kgm2
        else:
            ixx, iyy = thin_tube_inertia(r.dry_mass_kg, 0.5 * body.diameter, body.length)
            notes.append("inertia of the lumped airframe mass is a thin-tube ESTIMATE")
        comps.append(MassComponent("airframe", r.dry_mass_kg, r.cg_from_nose_m, ixx, iyy))
        if r.fins.mass_kg > 0 and r.fins.count:
            f_ = asm.fins
            xf = f_.leading_edge_from_nose + 0.5 * f_.mean_chord
            ixx, iyy = _fin_set_inertia(r.fins.mass_kg, asm.local_radius(xf) + 0.5 * f_.span)
            comps.append(MassComponent("fins", r.fins.mass_kg, xf, ixx, iyy))
        for cs in asm.controls:
            if cs.mass > 0:
                rho = asm.local_radius(cs.leading_edge_from_nose) + 0.5 * cs.span
                ixx, iyy = _fin_set_inertia(cs.mass, rho)
                comps.append(
                    MassComponent(
                        "control_surfaces", cs.mass, cs.leading_edge_from_nose + 0.5 * cs.root_chord, ixx, iyy
                    )
                )
    if r.payload is not None and r.payload.mass_kg > 0:
        comps.append(MassComponent("payload", r.payload.mass_kg, r.payload.position_from_nose_m))
    for item in r.masses:
        if item.mass_kg > 0:
            comps.append(
                MassComponent(
                    item.name,
                    item.mass_kg,
                    item.position_from_nose_m,
                    item.ixx_kgm2,
                    item.iyy_kgm2,
                    item.offset_y_m,
                    item.offset_z_m,
                    item.izz_kgm2,
                    item.ixy_kgm2,
                    item.ixz_kgm2,
                    item.iyz_kgm2,
                )
            )
    cixx, ciyy = thin_tube_inertia(motor.casing_mass, 0.5 * motor.diameter, motor.length)
    comps.append(MassComponent("motor_casing", motor.casing_mass, mx, cixx, ciyy))
    mass_model = MassModel(comps, mx, 0.5 * motor.diameter * 0.9, motor.length * 0.9)
    aero = _build_aero(cfg, asm, motor, fast)
    chutes = []
    nose_len = asm.nose.length
    for p in r.parachutes:
        attach = p.attach_from_nose_m if p.attach_from_nose_m is not None else nose_len
        chutes.append(
            Parachute(p.cd, p.diameter_m, p.trigger, p.altitude_m, p.delay_s, p.inflation_time_s, attach)
        )
    mis = (math.radians(cfg.motor.misalignment_deg[0]), math.radians(cfg.motor.misalignment_deg[1]))
    return Vehicle(
        r.name, body, aero, mass_model, motor, nozzle_x, mis, cfg.motor.ignition_delay_s, chutes, asm, notes
    )


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


__all__ = [
    "FidelityFlags",
    "build_dynamics",
    "build_environment",
    "build_launch",
    "build_motor",
    "build_vehicle",
    "build_wind",
    "resolve_fidelity",
    "solid_cylinder_inertia",
]
