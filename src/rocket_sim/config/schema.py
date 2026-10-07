"""Configuration schema: typed dataclasses, strict parsing and validation.

Conventions
-----------
* SI units everywhere. Every key carries its unit as a suffix (``_m``, ``_kg``, ``_s``,
  ``_ms`` = m/s, ``_pa``, ``_k``, ``_hz``, ``_deg`` for angles). Angles are given in DEGREES in
  config files (friendlier to write) and converted to radians internally.
* Unknown keys are errors (typo protection). Missing optional keys take the defaults below.
* Positions along the rocket are measured AFT from the nose tip.
* ``fidelity`` selects the physics level (see docs/physics.md, "Model hierarchy").

Every config section can be overridden per Monte-Carlo run by dotted path
(e.g. ``rocket.dry_mass_kg``), see rocket_sim.data.montecarlo.
"""

from __future__ import annotations

import difflib
import math
import types
import typing
from collections.abc import Mapping
from dataclasses import MISSING, dataclass, field, fields, is_dataclass
from typing import Any, Union

from ..errors import ConfigError
from ..version import CONFIG_VERSION

FIDELITY_LEVELS = {
    0: "1-D vertical, constant g, vacuum",
    1: "3-DOF point mass, g(h), vacuum",
    2: "3-DOF + atmosphere + wind + zero-AoA drag",
    3: "6-DOF rigid body + Barrowman aerodynamics + TVC",
    4: "6-DOF + simulated sensors",
    5: "6-DOF + sensors + estimator + controller",
    6: "level 5 with high-fidelity numerics (small dt, ISA, turbulence if configured)",
}


# ---------------------------------------------------------------------------------------------
# sections
# ---------------------------------------------------------------------------------------------
@dataclass
class SimulationSettings:
    dt_s: float = 0.01
    t_max_s: float = 900.0
    integrator: str = "rk4"  # euler | midpoint | rk4
    descent_dt_s: float | None = None  # step cap after apogee (smooth descent can use a larger step)
    stop_after_apogee_s: float | None = None  # end the run this long after apogee (status 'truncated')
    record_every: int = 1  # store every n-th step (events are always stored)
    seed: int | None = None  # master seed; None -> 0

    def validate(self, path: str) -> None:
        _check(self.dt_s > 0 and math.isfinite(self.dt_s), path, "dt_s must be > 0 and finite")
        _check(self.dt_s <= 1.0, path, "dt_s must be <= 1 s (integration would be meaningless)")
        _check(self.t_max_s > 0, path, "t_max_s must be > 0")
        _check(self.integrator in ("euler", "midpoint", "rk4"), path, "integrator must be euler|midpoint|rk4")
        _check(self.record_every >= 1, path, "record_every must be >= 1")
        if self.stop_after_apogee_s is not None:
            _check(self.stop_after_apogee_s >= 0, path, "stop_after_apogee_s must be >= 0")
        if self.descent_dt_s is not None:
            _check(0 < self.descent_dt_s <= 1.0, path, "descent_dt_s must be in (0, 1]")
        _check(self.t_max_s / self.dt_s < 5e7, path, "t_max_s/dt_s exceeds 5e7 steps")


@dataclass
class NoseCfg:
    shape: str = "ogive"
    length_m: float = 0.15


@dataclass
class FinsCfg:
    count: int = 4
    root_chord_m: float = 0.08
    tip_chord_m: float = 0.04
    span_m: float = 0.06
    sweep_m: float = 0.03
    thickness_m: float = 0.002
    position_from_nose_m: float = 0.0  # leading edge of root chord, aft of nose tip
    mass_kg: float = 0.0  # mass of the whole fin set (0 = included in the airframe mass)


@dataclass
class ParachuteCfg:
    cd: float = 1.5
    diameter_m: float = 0.6
    trigger: str = "apogee"
    altitude_m: float = 100.0
    delay_s: float = 0.0
    inflation_time_s: float = 0.5
    attach_from_nose_m: float | None = None  # shock-cord attachment; default: base of the nose cone


@dataclass
class AeroCfg:
    model: str = (
        "barrowman"  # simplified | barrowman | enhanced | table | table2d | constant  (buildup = barrowman)
    )
    cd: float | None = None  # constant model
    cd_table: list[list[float]] | None = None  # [[mach, cd], ...] (coast)
    cd_powered_table: list[list[float]] | None = None  # optional, motor burning
    cn_alpha: float | None = None  # constant/table models [1/rad]
    x_cp_from_nose_m: float | None = None  # constant/table models
    drag_scale: float = 1.0  # multiplier on component drag (calibration knob)
    extra_cd: float = 0.0  # lumped protuberance drag (lugs, rail buttons)
    surface_roughness_m: float = 60e-6
    crossflow_cd: float = 1.2
    nozzle_exit_ratio: float = 0.7  # nozzle exit diameter / motor diameter (base drag while burning)
    stall_angle_deg: float = 18.0  # enhanced model: fin lift stalls (saturates) near this angle of attack
    table2d_file: str | None = None  # table2d model: CSV with columns mach,alpha_deg,cd,cl[,cm]
    x_cm_ref_from_nose_m: float | None = None  # table2d: reference station of the cm column
    provenance: dict[str, Any] | None = (
        None  # user-declared provenance of table/table2d/constant data (kind, source, confidence, ...)
    )


@dataclass
class InertiaCfg:
    ixx_kgm2: float
    iyy_kgm2: float


@dataclass
class PayloadCfg:
    mass_kg: float
    position_from_nose_m: float


@dataclass
class SectionCfg:
    """One airframe section, listed from the nose tip aft (positions are computed by stacking)."""

    type: str = "body"  # nose | body | transition | boattail (a transition that narrows)
    length_m: float = 0.1
    diameter_m: float | None = None  # nose: base diameter; body: diameter
    fore_diameter_m: float | None = None  # transition: default = previous section's aft diameter
    aft_diameter_m: float | None = None  # transition
    shape: str = "ogive"  # nose profile
    mass_kg: float = 0.0
    cg_fraction: float = 0.5  # CG as a fraction of the length from the forward end
    name: str = ""  # optional label used in mass reports (default: the section type)


@dataclass
class MassItemCfg:
    """Additional rigid mass (payload, avionics, ballast ...). Inertia defaults to a point mass."""

    name: str = "payload"
    mass_kg: float = 0.0
    position_from_nose_m: float = 0.0
    offset_y_m: float = 0.0  # lateral offset from the axis (body y, right)
    offset_z_m: float = 0.0  # lateral offset from the axis (body z, down)
    ixx_kgm2: float = 0.0  # tensor about the item's own CG, body axes (x forward, y right, z down)
    iyy_kgm2: float = 0.0
    izz_kgm2: float | None = None  # default: = iyy (axisymmetric)
    ixy_kgm2: float = (
        0.0  # PRODUCTS of inertia (integral x y dm); the tensor off-diagonals are their negatives
    )
    ixz_kgm2: float = 0.0
    iyz_kgm2: float = 0.0


@dataclass
class ControlSurfaceCfg:
    """Movable fin set (canards or tail control fins)."""

    count: int = 4
    root_chord_m: float = 0.05
    tip_chord_m: float = 0.03
    span_m: float = 0.04
    sweep_m: float = 0.0
    thickness_m: float = 0.002
    position_from_nose_m: float = 0.0
    roll_angle0_deg: float = 0.0
    max_deflection_deg: float = 10.0
    max_rate_deg_s: float = 200.0
    time_constant_s: float = 0.0
    delay_s: float = 0.0
    mass_kg: float = 0.0


@dataclass
class RocketCfg:
    name: str = "rocket"
    body_diameter_m: float = 0.041
    body_length_m: float = 0.9
    nose: NoseCfg = field(default_factory=NoseCfg)
    fins: FinsCfg = field(default_factory=FinsCfg)
    sections: list[SectionCfg] | None = None  # component-based airframe (overrides body_*/nose)
    reference_diameter_m: float | None = None  # coefficient reference diameter (default: largest diameter)
    dry_mass_kg: float | None = (
        None  # lumped airframe mass (incl. recovery, WITHOUT motor); optional with sections
    )
    cg_from_nose_m: float | None = (
        None  # CG of that lumped mass (aft of nose tip); computed if only sections carry mass
    )
    masses: list[MassItemCfg] = field(default_factory=list)  # extra rigid masses
    control_surfaces: ControlSurfaceCfg | None = None
    inertia: InertiaCfg | None = None  # about dry CG; if absent a thin-tube estimate is used
    payload: PayloadCfg | None = None  # extra point mass (counted in addition to dry_mass_kg)
    motor_aft_from_nose_m: float | None = None  # nozzle exit plane; default = body_length_m
    aero: AeroCfg = field(default_factory=AeroCfg)
    parachutes: list[ParachuteCfg] = field(default_factory=list)
    vehicle_source: dict[str, Any] | None = (
        None  # {file, sha256, format_version, data_quality}: set when built from a vehicle file
    )


@dataclass
class MotorCfg:
    file: str = "data/motors/AeroTech_G80T.eng"
    designation: str | None = None
    thrust_scale: float = 1.0
    burn_time_scale: float = 1.0
    ignition_delay_s: float = 0.0
    misalignment_deg: list[float] = field(default_factory=lambda: [0.0, 0.0])  # about y_B, z_B


@dataclass
class AtmosphereCfg:
    model: str = "isa"  # isa | exponential | table
    temperature_offset_k: float = 0.0
    sea_level_pressure_pa: float = 101325.0
    table: list[list[float]] | None = None  # rows [altitude_m, temperature_k, pressure_pa]
    scale_height_m: float = 8400.0  # exponential model


@dataclass
class GravityCfg:
    model: str = "inverse_square"  # inverse_square | constant
    g_ms2: float = 9.80665


@dataclass
class GustCfg:
    start_s: float
    duration_s: float
    speed_ms: float
    direction_from_deg: float = 0.0


@dataclass
class WindCfg:
    model: str = "none"  # none | constant | profile | power_law
    speed_ms: float = 0.0
    direction_from_deg: float = 270.0  # bearing the wind comes FROM, clockwise from North
    profile: list[list[float]] | None = None  # rows [altitude_agl_m, speed_ms, direction_from_deg]
    ref_height_m: float = 10.0
    exponent: float = 1.0 / 7.0
    turbulence_sigma_ms: float = 0.0
    turbulence_tau_s: float = 2.0
    gusts: list[GustCfg] = field(default_factory=list)


@dataclass
class TerrainCfg:
    model: str = "flat"  # flat | slope
    height_m: float = 0.0
    slope_deg: float = 0.0
    slope_direction_deg: float = 0.0


@dataclass
class EnvironmentCfg:
    atmosphere: AtmosphereCfg = field(default_factory=AtmosphereCfg)
    gravity: GravityCfg = field(default_factory=GravityCfg)
    wind: WindCfg = field(default_factory=WindCfg)
    terrain: TerrainCfg = field(default_factory=TerrainCfg)
    site_elevation_msl_m: float = 0.0


@dataclass
class LaunchCfg:
    elevation_deg: float = 90.0  # nose above horizon
    azimuth_deg: float = 0.0  # compass bearing of the nose's horizontal projection
    roll_deg: float = 0.0
    rail_length_m: float = 1.0


@dataclass
class ActuatorCfg:
    max_angle_deg: float = 5.0
    max_rate_deg_s: float = 60.0
    time_constant_s: float = 0.03  # 0 -> ideal (no lag)
    delay_s: float = 0.0


@dataclass
class ControllerCfg:
    type: str = "none"  # none | tvc_attitude | schedule | python
    rate_hz: float = 50.0
    use_truth: bool = False  # feed the controller TRUE state (testing only; logged in metadata)
    state_source: str = (
        "auto"  # auto | estimate | truth. 'estimate' REFUSES to start without an estimator (no silent truth)
    )
    design_inertia_scale: float = 1.0  # flight computer's belief about I_yy/(T lever) relative to the as-built vehicle (1 = perfect knowledge)
    compute_time_s: float = (
        0.0  # flight-computer compute time: extra delay before a command reaches the actuator
    )
    uplink_latency_s: float = 0.0  # HIL bridge only: sensor -> flight-computer transport latency
    downlink_latency_s: float = 0.0  # command transport latency (adds to compute_time_s)
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class SensorCfg:
    enabled: bool = True
    rate_hz: float = 100.0
    noise_std: float = 0.0  # white noise 1-sigma [sensor units]
    bias_std: float = 0.0  # initial constant bias 1-sigma
    bias_walk_std: float = 0.0  # bias random walk [units / sqrt(s)]
    scale_error_std: float = 0.0  # fractional scale-factor error 1-sigma
    quantization: float = 0.0  # LSB [units] (0 = none)
    saturation: float = 0.0  # +- full-scale [units] (0 = none)
    latency_s: float = 0.0
    misalignment_std_deg: float = 0.0  # random small axis misalignment of 3-axis sensors (1-sigma, per run)
    dropout_probability: float = 0.0  # probability that a sample is lost (Bernoulli per sample)
    startup_delay_s: float = 0.0  # no samples before this time (e.g. GPS time-to-first-fix)
    velocity_noise_std: float | None = None  # GPS only: velocity 1-sigma [m/s] (default 0.05 x noise_std)


def _imu(rate: float, noise: float, bias: float, walk: float, fs: float) -> SensorCfg:
    return SensorCfg(True, rate, noise, bias, walk, 0.002, fs / 32768.0, fs, 0.0)


@dataclass
class SensorsCfg:
    """Representative MEMS-class sensor models. Defaults are *typical* orders of magnitude,
    not a specific datasheet; set them to your parts' values for hardware-specific work."""

    accelerometer: SensorCfg = field(default_factory=lambda: _imu(400.0, 0.03, 0.05, 1e-3, 157.0))
    gyroscope: SensorCfg = field(default_factory=lambda: _imu(400.0, 2.5e-3, 2e-3, 1e-5, 35.0))
    barometer: SensorCfg = field(
        default_factory=lambda: SensorCfg(True, 50.0, 3.0, 30.0, 0.05, 0.0, 1.0, 120000.0, 0.01)
    )
    gps: SensorCfg = field(default_factory=lambda: SensorCfg(True, 5.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2))
    magnetometer: SensorCfg = field(
        default_factory=lambda: SensorCfg(True, 50.0, 3e-7, 2e-7, 0.0, 0.01, 0.0, 0.0, 0.0)
    )
    magnetic_field_enu_t: list[float] = field(
        default_factory=lambda: [0.0, 2.0e-5, -4.0e-5]
    )  # Earth field, ENU [T]


@dataclass
class EstimatorCfg:
    type: str = "none"  # none | nav_kf | truth (perfect state: development/benchmark only)
    alignment_time_s: float = 1.0
    gps_enabled: bool = True
    estimate_gyro_bias: bool = True  # average the stationary pad gyro during alignment and subtract it


@dataclass
class LandingZoneCfg:
    center_m: list[float] = field(default_factory=lambda: [0.0, 0.0])
    radius_m: float = 50.0


@dataclass
class PhaseCfg:
    liftoff_distance_m: float = 0.01  # travel along rail/pad that counts as liftoff


@dataclass
class SimConfig:
    config_version: int = CONFIG_VERSION
    name: str = "simulation"
    fidelity: int = 3
    fast: bool = False  # fast mode: exponential atmosphere + Mach-independent Cd, fidelity <= 2
    simulation: SimulationSettings = field(default_factory=SimulationSettings)
    rocket: RocketCfg = field(default_factory=RocketCfg)
    motor: MotorCfg = field(default_factory=MotorCfg)
    environment: EnvironmentCfg = field(default_factory=EnvironmentCfg)
    launch: LaunchCfg = field(default_factory=LaunchCfg)
    tvc: ActuatorCfg = field(default_factory=ActuatorCfg)
    controller: ControllerCfg = field(default_factory=ControllerCfg)
    sensors: SensorsCfg = field(default_factory=SensorsCfg)
    estimator: EstimatorCfg = field(default_factory=EstimatorCfg)
    landing_zone: LandingZoneCfg | None = None
    phases: PhaseCfg = field(default_factory=PhaseCfg)

    def validate(self, path: str = "") -> None:
        _check(
            self.config_version == CONFIG_VERSION,
            path or "config_version",
            f"unsupported config_version {self.config_version} (this build reads {CONFIG_VERSION})",
        )
        _check(self.fidelity in FIDELITY_LEVELS, "fidelity", f"must be one of {sorted(FIDELITY_LEVELS)}")
        if self.fast and self.fidelity > 2:
            raise ConfigError("fast mode is only defined for fidelity <= 2")
        if (
            self.fidelity >= 5
            and self.controller.type not in ("none", "hil")
            and self.estimator.type == "none"
            and not self.controller.use_truth
        ):
            raise ConfigError(
                "controller needs state input: set estimator.type (e.g. nav_kf) or "
                "controller.use_truth: true for fidelity 5"
            )
        _validate_rocket(self.rocket)
        _validate_env(self.environment)
        l = self.launch
        _check(0.0 < l.elevation_deg <= 90.0, "launch.elevation_deg", "must be in (0, 90]")
        _check(l.rail_length_m >= 0.0, "launch.rail_length_m", "must be >= 0")
        m = self.motor
        _check(m.thrust_scale > 0 and m.burn_time_scale > 0, "motor", "scale factors must be > 0")
        _check(m.ignition_delay_s >= 0, "motor.ignition_delay_s", "must be >= 0")
        _check(len(m.misalignment_deg) == 2, "motor.misalignment_deg", "needs exactly 2 values")
        a = self.tvc
        _check(a.max_angle_deg >= 0 and a.max_rate_deg_s > 0, "tvc", "max_angle_deg >= 0, max_rate_deg_s > 0")
        _check(a.time_constant_s >= 0 and a.delay_s >= 0, "tvc", "time_constant_s/delay_s must be >= 0")
        _check(self.controller.rate_hz > 0, "controller.rate_hz", "must be > 0")
        c = self.controller
        _check(
            c.state_source in ("auto", "estimate", "truth"),
            "controller.state_source",
            "must be auto|estimate|truth",
        )
        _check(
            min(c.compute_time_s, c.uplink_latency_s, c.downlink_latency_s) >= 0,
            "controller",
            "compute_time_s/uplink_latency_s/downlink_latency_s must be >= 0",
        )
        _check(c.design_inertia_scale > 0, "controller.design_inertia_scale", "must be > 0")
        _check(
            not (c.use_truth and c.state_source == "estimate"),
            "controller.state_source",
            "use_truth: true contradicts state_source: estimate",
        )
        if c.type not in ("none", "hil") and c.state_source == "estimate":
            _check(
                self.estimator.type == "nav_kf" and self.fidelity >= 5,
                "controller.state_source",
                "'estimate' needs estimator.type: nav_kf at fidelity >= 5 (sensors + estimator in the loop)",
            )
        _check(
            self.controller.type in ("none", "tvc_attitude", "schedule", "python", "hil"),
            "controller.type",
            "must be none|tvc_attitude|schedule|python|hil",
        )
        _check(
            self.estimator.type in ("none", "nav_kf", "truth"), "estimator.type", "must be none|nav_kf|truth"
        )
        _check(
            len(self.sensors.magnetic_field_enu_t) == 3 and any(self.sensors.magnetic_field_enu_t),
            "sensors.magnetic_field_enu_t",
            "needs 3 components, not all zero",
        )
        if self.estimator.type == "nav_kf" and self.fidelity >= 5:
            _check(
                self.sensors.accelerometer.enabled and self.sensors.magnetometer.enabled,
                "estimator.type",
                "nav_kf aligns attitude from the accelerometer AND the magnetometer: both must be enabled",
            )
        if self.estimator.type != "none" and self.fidelity >= 5:
            _check(
                self.motor.ignition_delay_s >= self.estimator.alignment_time_s,
                "estimator.alignment_time_s",
                f"pad alignment ({self.estimator.alignment_time_s} s) must finish before ignition "
                f"(motor.ignition_delay_s = {self.motor.ignition_delay_s} s): set motor.ignition_delay_s >= alignment time",
            )
        for nm in ("accelerometer", "gyroscope", "barometer", "gps", "magnetometer"):
            s: SensorCfg = getattr(self.sensors, nm)
            p = f"sensors.{nm}"
            _check(s.rate_hz > 0, p, "rate_hz must be > 0")
            _check(
                min(
                    s.noise_std,
                    s.bias_std,
                    s.bias_walk_std,
                    s.scale_error_std,
                    s.quantization,
                    s.saturation,
                    s.latency_s,
                    s.misalignment_std_deg,
                    s.startup_delay_s,
                )
                >= 0
                and 0.0 <= s.dropout_probability <= 1.0,
                p,
                "noise/bias/limits must be >= 0",
            )
        if self.landing_zone is not None:
            _check(
                self.landing_zone.radius_m > 0 and len(self.landing_zone.center_m) == 2,
                "landing_zone",
                "radius_m > 0 and center_m of length 2 required",
            )
        self.simulation.validate("simulation")
        dt = self.simulation.dt_s
        if self.fidelity >= 3:
            _check(dt <= 0.05, "simulation.dt_s", "6-DOF needs dt_s <= 0.05 s for attitude stability")
            if self.simulation.descent_dt_s is not None:
                _check(
                    self.simulation.descent_dt_s <= 0.05,
                    "simulation.descent_dt_s",
                    "6-DOF needs descent_dt_s <= 0.05 s",
                )


# ---------------------------------------------------------------------------------------------
# validation helpers
# ---------------------------------------------------------------------------------------------
def _check(cond: bool, path: str, msg: str) -> None:
    if not cond:
        raise ConfigError(f"{path}: {msg}")


def airframe_length(r: RocketCfg) -> float:
    """Total airframe length [m] (sum of section lengths, or the legacy body_length_m)."""
    return sum(sec.length_m for sec in r.sections) if r.sections else r.body_length_m


def _validate_rocket(r: RocketCfg) -> None:
    p = "rocket"
    length = airframe_length(r)
    if r.sections:
        _check(r.sections[0].type == "nose", f"{p}.sections[0].type", "the first section must be the nose")
        for i, sec in enumerate(r.sections):
            sp = f"{p}.sections[{i}]"
            _check(
                sec.type in ("nose", "body", "transition", "boattail"),
                f"{sp}.type",
                "must be nose|body|transition|boattail",
            )
            _check(sec.length_m > 0, f"{sp}.length_m", "must be > 0")
            _check(
                sec.mass_kg >= 0 and 0.0 <= sec.cg_fraction <= 1.0,
                sp,
                "mass_kg >= 0 and cg_fraction in [0, 1]",
            )
            if sec.type == "body":
                _check(
                    sec.diameter_m is not None and sec.diameter_m > 0,
                    f"{sp}.diameter_m",
                    "required (> 0) for a body",
                )
            if sec.type in ("transition", "boattail"):
                _check(
                    sec.aft_diameter_m is not None and sec.aft_diameter_m > 0,
                    f"{sp}.aft_diameter_m",
                    "required (> 0)",
                )
        _check(
            sum(sec.mass_kg for sec in r.sections)
            + sum(m.mass_kg for m in r.masses)
            + (r.dry_mass_kg or 0.0)
            + r.fins.mass_kg
            > 0,
            f"{p}.sections",
            "no mass given: set section mass_kg, masses[] or a lumped dry_mass_kg",
        )
    else:
        _check(
            r.dry_mass_kg is not None and r.dry_mass_kg > 0,
            f"{p}.dry_mass_kg",
            f"must be > 0 (got {r.dry_mass_kg})",
        )
        _check(r.body_diameter_m > 0, f"{p}.body_diameter_m", f"must be > 0 (got {r.body_diameter_m})")
        _check(r.body_length_m > r.body_diameter_m, f"{p}.body_length_m", "must exceed body_diameter_m")
        _check(
            r.cg_from_nose_m is not None,
            f"{p}.cg_from_nose_m",
            "required with a lumped dry_mass_kg and no sections",
        )
        _check(0.0 < r.nose.length_m < r.body_length_m, f"{p}.nose.length_m", "must be in (0, body_length_m)")
    if r.dry_mass_kg is not None:
        _check(r.dry_mass_kg >= 0, f"{p}.dry_mass_kg", f"must be >= 0 (got {r.dry_mass_kg})")
        if r.dry_mass_kg > 0:
            _check(
                r.cg_from_nose_m is not None,
                f"{p}.cg_from_nose_m",
                "required when a lumped dry_mass_kg is given",
            )
    if r.cg_from_nose_m is not None:
        _check(
            0.0 < r.cg_from_nose_m < length,
            f"{p}.cg_from_nose_m",
            f"must lie inside the airframe (0, {length}) m",
        )
    nose_len = r.sections[0].length_m if r.sections else r.nose.length_m
    if r.fins.count > 0:
        f = r.fins
        _check(
            f.position_from_nose_m + f.root_chord_m <= length + 1e-9,
            f"{p}.fins",
            "fin root chord extends beyond the body base",
        )
        _check(
            f.position_from_nose_m >= nose_len - 1e-9,
            f"{p}.fins.position_from_nose_m",
            "fins cannot start on the nose cone",
        )
    for i, m in enumerate(r.masses):
        _check(m.mass_kg >= 0, f"{p}.masses[{i}].mass_kg", "must be >= 0")
        _check(
            0.0 <= m.position_from_nose_m <= length,
            f"{p}.masses[{i}].position_from_nose_m",
            "must be inside the airframe",
        )
    if r.control_surfaces is not None:
        c = r.control_surfaces
        _check(c.count >= 2, f"{p}.control_surfaces.count", "must be >= 2")
        _check(
            c.max_deflection_deg > 0 and c.max_rate_deg_s > 0 and c.time_constant_s >= 0 and c.delay_s >= 0,
            f"{p}.control_surfaces",
            "max_deflection_deg/max_rate_deg_s > 0, time_constant_s/delay_s >= 0",
        )
        _check(
            c.position_from_nose_m >= 0.0 and c.position_from_nose_m + c.root_chord_m <= length + 1e-9,
            f"{p}.control_surfaces.position_from_nose_m",
            "control surface must lie on the airframe",
        )
    if r.inertia is not None:
        _check(r.inertia.ixx_kgm2 > 0 and r.inertia.iyy_kgm2 > 0, f"{p}.inertia", "inertias must be > 0")
    if r.payload is not None:
        _check(r.payload.mass_kg >= 0, f"{p}.payload.mass_kg", "must be >= 0")
        _check(
            0.0 <= r.payload.position_from_nose_m <= length,
            f"{p}.payload.position_from_nose_m",
            "must be inside the body",
        )
    if r.motor_aft_from_nose_m is not None:
        _check(
            0.0 < r.motor_aft_from_nose_m <= length + 1e-9,
            f"{p}.motor_aft_from_nose_m",
            "must be within the body length",
        )
    a = r.aero
    _check(
        a.model in ("buildup", "simplified", "barrowman", "enhanced", "constant", "table", "table2d"),
        f"{p}.aero.model",
        "must be simplified|barrowman|enhanced|table|table2d|constant",
    )
    _check(a.drag_scale > 0, f"{p}.aero.drag_scale", "must be > 0")
    _check(0.0 <= a.nozzle_exit_ratio < 1.0, f"{p}.aero.nozzle_exit_ratio", "must be in [0, 1)")
    if a.model in ("constant", "simplified"):
        _check(
            a.cd is not None and a.cd >= 0,
            f"{p}.aero.cd",
            "required (>= 0) for the constant/simplified model",
        )
    if a.model == "table2d":
        _check(a.table2d_file is not None, f"{p}.aero.table2d_file", "required for the table2d model")
    if a.model == "table":
        _check(
            a.cd_table is not None and len(a.cd_table) >= 1 and all(len(row) == 2 for row in a.cd_table),
            f"{p}.aero.cd_table",
            "rows of [mach, cd] required for the table model",
        )


def _validate_env(e: EnvironmentCfg) -> None:
    a = e.atmosphere
    _check(
        a.model in ("isa", "exponential", "table"),
        "environment.atmosphere.model",
        "must be isa|exponential|table",
    )
    _check(a.sea_level_pressure_pa > 0, "environment.atmosphere.sea_level_pressure_pa", "must be > 0")
    _check(
        a.temperature_offset_k > -80.0 and a.temperature_offset_k < 80.0,
        "environment.atmosphere.temperature_offset_k",
        "implausible offset (|dT| must be < 80 K)",
    )
    if a.model == "table":
        _check(
            a.table is not None and len(a.table) >= 2 and all(len(r) == 3 for r in a.table),
            "environment.atmosphere.table",
            "rows of [altitude_m, temperature_k, pressure_pa] required",
        )
    _check(
        e.gravity.model in ("inverse_square", "constant"),
        "environment.gravity.model",
        "must be inverse_square|constant",
    )
    _check(e.gravity.g_ms2 > 0, "environment.gravity.g_ms2", "must be > 0")
    w = e.wind
    _check(
        w.model in ("none", "constant", "profile", "power_law"),
        "environment.wind.model",
        "must be none|constant|profile|power_law",
    )
    _check(w.speed_ms >= 0, "environment.wind.speed_ms", "must be >= 0")
    _check(
        w.turbulence_sigma_ms >= 0 and w.turbulence_tau_s > 0,
        "environment.wind",
        "turbulence sigma >= 0 and tau > 0",
    )
    if w.model == "profile":
        _check(
            w.profile is not None and len(w.profile) >= 1 and all(len(r) == 3 for r in w.profile),
            "environment.wind.profile",
            "rows of [altitude_agl_m, speed_ms, direction_from_deg] required",
        )
    _check(e.terrain.model in ("flat", "slope"), "environment.terrain.model", "must be flat|slope")
    _check(
        -500.0 <= e.site_elevation_msl_m <= 6000.0,
        "environment.site_elevation_msl_m",
        "must be in [-500, 6000] m",
    )


# ---------------------------------------------------------------------------------------------
# generic dict -> dataclass parser with precise error paths
# ---------------------------------------------------------------------------------------------
def from_dict(cls: type, data: Any, path: str = "") -> Any:
    """Build dataclass ``cls`` from a mapping. Raises ConfigError with a dotted path."""
    if not isinstance(data, Mapping):
        raise ConfigError(f"{path or 'config'}: expected a mapping, got {type(data).__name__}")
    hints = typing.get_type_hints(cls)
    known = {f.name: f for f in fields(cls)}
    for key in data:
        if key not in known:
            close = difflib.get_close_matches(str(key), list(known), n=1)
            hint = f" (did you mean '{close[0]}'?)" if close else ""
            raise ConfigError(f"{path + '.' if path else ''}{key}: unknown key{hint}")
    kwargs: dict[str, Any] = {}
    for name, f in known.items():
        sub = f"{path}.{name}" if path else name
        if name in data:
            kwargs[name] = _convert(hints[name], data[name], sub)
        elif f.default is MISSING and f.default_factory is MISSING:
            raise ConfigError(f"{sub}: required key is missing")
    obj = cls(**kwargs)
    return obj


def _convert(tp: Any, value: Any, path: str) -> Any:
    origin = typing.get_origin(tp)
    if origin in (Union, types.UnionType):
        members = [a for a in typing.get_args(tp) if a is not type(None)]
        if value is None:
            if len(members) < len(typing.get_args(tp)):
                return None
            raise ConfigError(f"{path}: null not allowed")
        return _convert(members[0], value, path)
    if isinstance(tp, type) and is_dataclass(tp):
        return from_dict(tp, value, path)
    if origin is list:
        if not isinstance(value, list | tuple):
            raise ConfigError(f"{path}: expected a list, got {type(value).__name__}")
        (inner,) = typing.get_args(tp)
        return [_convert(inner, v, f"{path}[{i}]") for i, v in enumerate(value)]
    if origin is dict or tp is dict:
        if not isinstance(value, Mapping):
            raise ConfigError(f"{path}: expected a mapping, got {type(value).__name__}")
        args = typing.get_args(tp)
        if len(args) == 2 and args[1] is not Any:
            return {str(k): _convert(args[1], v, f"{path}.{k}") for k, v in value.items()}
        return dict(value)
    if tp is Any:
        return value
    if tp is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{path}: expected true/false, got {value!r}")
        return value
    if tp is int:
        if isinstance(value, bool) or not isinstance(value, int):
            if isinstance(value, float) and value.is_integer():
                return int(value)
            raise ConfigError(f"{path}: expected an integer, got {value!r}")
        return value
    if tp is float:
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ConfigError(f"{path}: expected a number, got {value!r}")
        if not math.isfinite(value):
            raise ConfigError(f"{path}: must be finite, got {value!r}")
        return float(value)
    if tp is str:
        if not isinstance(value, str):
            raise ConfigError(f"{path}: expected a string, got {value!r}")
        return value
    return value


def validate_all(cfg: Any, path: str = "") -> None:
    """Run ``validate`` on the root config (nested checks are called from it)."""
    cfg.validate(path)
