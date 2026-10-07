"""Telemetry / dataset column schema. Versioned by ``rocket_sim.version.SCHEMA_VERSION``.

Rules
-----
* SI units; angles in radians; time in seconds. One row per recorded step.
* Every column has a ROLE (``role_of``): truth | measurement | estimate | command | actual. Vehicle-state columns
  (pos_*, vel_*, acc_*, quat_*, roll/pitch/yaw, omega_*, mass, CG, inertia, fsp_* ...) are GROUND
  TRUTH. Measured values are prefixed ``meas_``; estimator outputs ``est_``. Truth is never
  overwritten by measurements.
* Frames: pos/vel/acc/wind in the launch frame (x East, y North, z Up); omega/alpha/meas_accel/
  meas_gyro/meas_mag in the body frame (x nose, y right, z down). See docs/frames.md.
* Groups appear only when the fidelity level enables them (sensors >= 4, estimator >= 5), so
  every dataset is homogeneous; the manifest records the exact column list.
* Changing/removing/renaming a column or its meaning requires bumping SCHEMA_VERSION.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..version import SCHEMA_VERSION


@dataclass(frozen=True)
class Column:
    name: str
    unit: str
    description: str
    group: str
    dtype: str = "float64"
    min_fidelity: int = 0


def _c(name: str, unit: str, desc: str, group: str, dtype: str = "float64", fid: int = 0) -> Column:
    return Column(name, unit, desc, group, dtype, fid)


def _vec(
    prefix: str, axes: str, unit: str, desc: str, group: str, fid: int = 0, dtype: str = "float64"
) -> list[Column]:
    return [_c(f"{prefix}_{a}", unit, f"{desc} ({a})", group, dtype, fid) for a in axes]


COLUMNS: list[Column] = [
    _c("t", "s", "simulation time", "time"),
    _c("dt", "s", "step length that produced this row (0 for the first row)", "time"),
    _c("phase", "-", "flight phase code (see FlightPhase)", "flight", "int8"),
    *_vec("pos", "xyz", "m", "position, launch frame (E, N, U)", "state"),
    *_vec("vel", "xyz", "m/s", "velocity, launch frame", "state"),
    *_vec("acc", "xyz", "m/s^2", "coordinate acceleration, launch frame", "state"),
    *_vec("quat", "wxyz", "-", "attitude quaternion body->launch (Hamilton)", "state"),
    _c("roll", "rad", "roll about the nose axis (display Euler, 3-2-1 vs local NED)", "state"),
    _c("pitch", "rad", "nose elevation above the horizon (display Euler)", "state"),
    _c("yaw", "rad", "heading of the nose, clockwise from North (display Euler)", "state"),
    *_vec("omega", "pqr", "rad/s", "angular velocity, body frame", "state"),
    *_vec("alpha", "pqr", "rad/s^2", "angular acceleration, body frame", "state"),
    _c("mass", "kg", "total mass", "mass"),
    _c("prop_mass", "kg", "remaining propellant mass", "mass"),
    _c("dry_mass", "kg", "mass with zero propellant", "mass"),
    _c("cg", "m", "centre of gravity, aft of nose tip", "mass"),
    _c("cp", "m", "centre of pressure, aft of nose tip", "mass"),
    _c("static_margin", "cal", "(cp - cg)/diameter; > 0 is statically stable", "mass"),
    _c("ixx", "kg m^2", "roll moment of inertia about CG", "mass"),
    _c("iyy", "kg m^2", "pitch/yaw moment of inertia about CG", "mass"),
    _c(
        "izz",
        "kg m^2",
        "yaw moment of inertia about CG (full tensor; = iyy for an axisymmetric vehicle)",
        "mass",
        fid=3,
    ),
    _c(
        "ixy",
        "kg m^2",
        "product of inertia int(x y dm) about the CG; tensor off-diagonal is its negative",
        "mass",
        fid=3,
    ),
    _c("ixz", "kg m^2", "product of inertia int(x z dm) about the CG", "mass", fid=3),
    _c("iyz", "kg m^2", "product of inertia int(y z dm) about the CG", "mass", fid=3),
    _c("cg_y", "m", "lateral CG offset from the nose axis, body y (right)", "mass", fid=3),
    _c("cg_z", "m", "lateral CG offset from the nose axis, body z (down)", "mass", fid=3),
    *_vec(
        "fsp",
        "xyz",
        "m/s^2",
        "TRUE specific force at the CG, body frame (what an ideal accelerometer would read)",
        "state",
        3,
    ),
    _c("altitude", "m", "height above local ground (AGL)", "environment"),
    _c("altitude_msl", "m", "height above mean sea level", "environment"),
    _c("temperature", "K", "air temperature", "environment"),
    _c("pressure", "Pa", "static air pressure", "environment"),
    _c("density", "kg/m^3", "air density", "environment"),
    _c("speed_of_sound", "m/s", "speed of sound", "environment"),
    *_vec("wind", "xyz", "m/s", "air velocity, launch frame", "environment"),
    _c("airspeed", "m/s", "|v_vehicle - v_wind|", "flight"),
    _c("speed", "m/s", "|v_vehicle| (ground-relative)", "flight"),
    _c("gravity", "m/s^2", "gravitational acceleration magnitude", "flight"),
    _c("thrust", "N", "thrust magnitude", "flight"),
    _c("drag", "N", "aerodynamic drag incl. parachute (along -v_rel)", "flight"),
    _c("lift", "N", "aerodynamic force normal to the relative wind", "flight"),
    _c("chute_drag", "N", "parachute drag", "flight"),
    _c("mach", "-", "airspeed / speed of sound", "flight"),
    _c("aoa", "rad", "total angle of attack (nose to relative wind)", "flight"),
    _c("sideslip", "rad", "sideslip angle", "flight"),
    _c("qdyn", "Pa", "dynamic pressure", "flight"),
    _c("tvc_cmd_y", "rad", "commanded thrust deflection about y_B", "control", fid=3),
    _c("tvc_cmd_z", "rad", "commanded thrust deflection about z_B", "control", fid=3),
    _c("tvc_y", "rad", "actual thrust deflection about y_B", "control", fid=3),
    _c("tvc_z", "rad", "actual thrust deflection about z_B", "control", fid=3),
    _c(
        "fin_cmd_pitch",
        "rad",
        "commanded pitch deflection-equivalent (control surfaces)",
        "control_surfaces",
        fid=3,
    ),
    _c(
        "fin_cmd_yaw",
        "rad",
        "commanded yaw deflection-equivalent (control surfaces)",
        "control_surfaces",
        fid=3,
    ),
    _c(
        "fin_cmd_roll",
        "rad",
        "commanded roll deflection-equivalent (control surfaces)",
        "control_surfaces",
        fid=3,
    ),
    *[
        _c(
            f"fin_dcmd_{i}",
            "rad",
            f"commanded (post-mixer, saturated) deflection of control fin {i}",
            "control_surfaces",
            fid=3,
        )
        for i in range(8)
    ],
    *[
        _c(f"fin_{i}", "rad", f"actual deflection of control fin {i}", "control_surfaces", fid=3)
        for i in range(8)
    ],
    *_vec("meas_accel", "xyz", "m/s^2", "accelerometer specific force, body", "sensors", 4),
    _c(
        "meas_accel_new",
        "-",
        "1 when a new accelerometer sample became visible this row",
        "sensors",
        "int8",
        4,
    ),
    *_vec("meas_gyro", "xyz", "rad/s", "gyroscope rate, body", "sensors", 4),
    _c("meas_gyro_new", "-", "1 when a new gyroscope sample became visible this row", "sensors", "int8", 4),
    _c("meas_baro_pressure", "Pa", "barometer static pressure", "sensors", fid=4),
    _c(
        "meas_baro_altitude",
        "m",
        "ISA altitude from barometer relative to the first sample",
        "sensors",
        fid=4,
    ),
    _c("meas_baro_new", "-", "1 when a new barometer sample became visible this row", "sensors", "int8", 4),
    *_vec("meas_gps_pos", "xyz", "m", "GPS position, launch frame", "sensors", 4),
    *_vec("meas_gps_vel", "xyz", "m/s", "GPS velocity, launch frame", "sensors", 4),
    _c("meas_gps_new", "-", "1 when a new GPS sample became visible this row", "sensors", "int8", 4),
    *_vec("meas_mag", "xyz", "T", "magnetometer field, body", "sensors", 4),
    _c("meas_mag_new", "-", "1 when a new magnetometer sample became visible this row", "sensors", "int8", 4),
    _c("est_valid", "-", "1 when the estimator output is valid (aligned)", "estimator", "int8", 3),
    _c("launch_detected", "-", "1 once the flight computer has detected launch", "estimator", "int8", 3),
    *_vec("est_pos", "xyz", "m", "estimated position, launch frame", "estimator", 3),
    *_vec("est_vel", "xyz", "m/s", "estimated velocity, launch frame", "estimator", 3),
    *_vec("est_quat", "wxyz", "-", "estimated attitude quaternion", "estimator", 3),
    *_vec(
        "est_gyro_bias",
        "xyz",
        "rad/s",
        "estimated gyro bias (pad-estimated; 0 for the truth estimator)",
        "estimator",
        3,
    ),
]

_BY_NAME = {c.name: c for c in COLUMNS}


def columns_for(fidelity: int, estimator: bool = False, n_fins: int = 0) -> list[Column]:
    """Columns present for a fidelity level (estimator group only when an estimator runs; control-surface
    columns only for vehicles with ``n_fins`` movable fins)."""
    out = []
    for c in COLUMNS:
        if c.min_fidelity > fidelity:
            continue
        if c.group == "estimator" and not estimator:
            continue
        if c.group == "control_surfaces":
            if n_fins == 0:
                continue
            if c.name.startswith("fin_dcmd_") and int(c.name[9:]) >= n_fins:
                continue
            if c.name.startswith("fin_") and c.name[4:].isdigit() and int(c.name[4:]) >= n_fins:
                continue
        out.append(c)
    return out


def column(name: str) -> Column:
    return _BY_NAME[name]


ROLES = ("time", "truth", "measurement", "estimate", "command", "actual")


def role_of(c: Column) -> str:
    """The epistemic role of a column - what KIND of knowledge it carries (essential for ML feature selection):

    truth        the simulator's true state or environment (never available to a real flight computer)
    measurement  what a simulated sensor reported (``meas_*``)
    estimate     what the flight computer believes (``est_*``)
    command      what the controller requested (``tvc_cmd_*``, ``fin_cmd_*``, ``fin_dcmd_*``)
    actual       the physical actuator state that the physics used (``tvc_y/z``, ``fin_i``)
    """
    if c.group == "time":
        return "time"
    if c.group == "sensors":
        return "measurement"
    if c.group == "estimator":
        return "estimate"
    if c.group in ("control", "control_surfaces"):
        return "command" if ("cmd" in c.name) else "actual"
    return "truth"


def columns_by_role(role: str, fidelity: int = 6, estimator: bool = True, n_fins: int = 8) -> list[str]:
    """Names of all columns with a given role for a configuration (e.g. 'truth' to build labels, 'measurement' for inputs)."""
    return [c.name for c in columns_for(fidelity, estimator, n_fins) if role_of(c) == role]


def schema_dict() -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "columns": [
            {
                "name": c.name,
                "unit": c.unit,
                "dtype": c.dtype,
                "group": c.group,
                "role": role_of(c),
                "min_fidelity": c.min_fidelity,
                "description": c.description,
            }
            for c in COLUMNS
        ],
    }


def schema_markdown() -> str:
    lines = [
        f"# Telemetry schema v{SCHEMA_VERSION}",
        "",
        "| column | unit | dtype | group | role | min fidelity | description |",
        "|---|---|---|---|---|---|---|",
    ]
    for c in COLUMNS:
        lines.append(
            f"| `{c.name}` | {c.unit} | {c.dtype} | {c.group} | {role_of(c)} | {c.min_fidelity} | {c.description} |"
        )
    return "\n".join(lines) + "\n"
