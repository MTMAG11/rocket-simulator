"""Plot catalogue for the GUI: readable names, grouping and display units for the telemetry columns. No Qt here."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..data.schema import columns_for
from ..simulation.record import FlightRecord

DEG = 180.0 / math.pi
G0 = 9.80665


@dataclass(frozen=True)
class Series:
    column: str
    label: str
    scale: float = 1.0


@dataclass(frozen=True)
class Channel:
    id: str
    group: str
    title: str
    unit: str
    series: tuple[Series, ...]
    derived: str | None = None  # name of a derived quantity (see _derive)
    tip: str = ""

    def columns(self) -> list[str]:
        return [s.column for s in self.series]

    def available(self, rec: FlightRecord) -> bool:
        if self.derived == "load_factor":
            return all(rec.has(f"fsp_{a}") for a in "xyz")
        return all(rec.has(c) for c in self.columns())

    def values(self, rec: FlightRecord) -> list[tuple[str, np.ndarray]]:
        if self.derived == "load_factor":
            f = np.sqrt(sum(rec.col(f"fsp_{a}") ** 2 for a in "xyz"))
            return [("Load factor", f / G0)]
        return [(s.label, rec.col(s.column) * s.scale) for s in self.series]


def _ch(id_, group, title, unit, series, tip="", derived=None) -> Channel:
    return Channel(
        id_,
        group,
        title,
        unit,
        tuple(Series(*s) if isinstance(s, tuple) else s for s in series),
        derived,
        tip,
    )


def ENU(prefix: str, scale: float = 1.0) -> list[tuple[str, str, float]]:
    """East/North/Up series for columns ``<prefix>_x/_y/_z`` (launch frame)."""
    return [(f"{prefix}_x", "East", scale), (f"{prefix}_y", "North", scale), (f"{prefix}_z", "Up", scale)]


def BODY(prefix: str, scale: float = 1.0) -> list[tuple[str, str, float]]:
    """Body-axis series for columns ``<prefix>_x/_y/_z`` (x nose, y right, z down)."""
    return [
        (f"{prefix}_x", "X (nose)", scale),
        (f"{prefix}_y", "Y (right)", scale),
        (f"{prefix}_z", "Z (down)", scale),
    ]


CATALOG: list[Channel] = [
    # Trajectory
    _ch("altitude", "Trajectory", "Altitude above ground", "m", [("altitude", "Altitude")]),
    _ch("altitude_msl", "Trajectory", "Altitude above sea level", "m", [("altitude_msl", "Altitude MSL")]),
    _ch(
        "position",
        "Trajectory",
        "Position (launch frame)",
        "m",
        ENU("pos"),
        "East / North / Up from the launch point",
    ),
    # Velocity
    _ch(
        "speed",
        "Velocity",
        "Speed",
        "m/s",
        [("speed", "Ground speed"), ("airspeed", "Airspeed")],
        "Airspeed is relative to the wind",
    ),
    _ch("velocity", "Velocity", "Velocity (launch frame)", "m/s", ENU("vel")),
    _ch("mach", "Velocity", "Mach number", "", [("mach", "Mach")]),
    # Acceleration
    _ch(
        "load_factor",
        "Acceleration",
        "Load factor",
        "g",
        [("fsp_x", "Load factor")],
        "Magnitude of the specific force (what an accelerometer feels)",
        derived="load_factor",
    ),
    _ch(
        "specific_force",
        "Acceleration",
        "Specific force (body axes)",
        "m/s²",
        BODY("fsp"),
        "True accelerometer-equivalent acceleration",
    ),
    _ch(
        "acceleration",
        "Acceleration",
        "Acceleration (launch frame)",
        "m/s²",
        ENU("acc"),
        "Coordinate acceleration, gravity included",
    ),
    # Attitude
    _ch(
        "attitude",
        "Attitude",
        "Attitude",
        "deg",
        [
            ("pitch", "Pitch (above horizon)", DEG),
            ("yaw", "Heading (from North)", DEG),
            ("roll", "Roll", DEG),
        ],
    ),
    _ch(
        "aoa",
        "Attitude",
        "Angle of attack",
        "deg",
        [("aoa", "Angle of attack", DEG), ("sideslip", "Sideslip", DEG)],
    ),
    _ch(
        "rates",
        "Attitude",
        "Angular rate (body axes)",
        "deg/s",
        [("omega_p", "Roll rate p", DEG), ("omega_q", "Pitch rate q", DEG), ("omega_r", "Yaw rate r", DEG)],
    ),
    _ch(
        "ang_acc",
        "Attitude",
        "Angular acceleration (body axes)",
        "deg/s²",
        [("alpha_p", "p-dot", DEG), ("alpha_q", "q-dot", DEG), ("alpha_r", "r-dot", DEG)],
    ),
    # Forces
    _ch("thrust", "Forces", "Thrust", "N", [("thrust", "Thrust")]),
    _ch(
        "aero_forces",
        "Forces",
        "Aerodynamic forces",
        "N",
        [("drag", "Drag"), ("lift", "Lift"), ("chute_drag", "Parachute drag")],
    ),
    _ch("qdyn", "Forces", "Dynamic pressure", "Pa", [("qdyn", "Dynamic pressure")]),
    # Mass and stability
    _ch(
        "mass",
        "Mass and stability",
        "Mass",
        "kg",
        [("mass", "Total"), ("prop_mass", "Propellant"), ("dry_mass", "Dry")],
    ),
    _ch(
        "static_margin",
        "Mass and stability",
        "Static margin",
        "cal",
        [("static_margin", "Static margin")],
        "(CP - CG) / diameter; positive is stable",
    ),
    _ch(
        "cg_cp",
        "Mass and stability",
        "CG and CP (from nose tip)",
        "m",
        [("cg", "Centre of gravity"), ("cp", "Centre of pressure")],
    ),
    _ch(
        "inertia",
        "Mass and stability",
        "Moments of inertia",
        "kg m²",
        [("ixx", "Ixx (roll)"), ("iyy", "Iyy (pitch)"), ("izz", "Izz (yaw)")],
    ),
    # Atmosphere
    _ch("wind", "Atmosphere", "Wind (air velocity)", "m/s", ENU("wind")),
    _ch("temperature", "Atmosphere", "Air temperature", "K", [("temperature", "Temperature")]),
    _ch("pressure", "Atmosphere", "Air pressure", "Pa", [("pressure", "Static pressure")]),
    _ch("density", "Atmosphere", "Air density", "kg/m³", [("density", "Density")]),
    # Control
    _ch(
        "tvc_y",
        "Control",
        "TVC deflection about y",
        "deg",
        [("tvc_cmd_y", "Commanded", DEG), ("tvc_y", "Actual", DEG)],
    ),
    _ch(
        "tvc_z",
        "Control",
        "TVC deflection about z",
        "deg",
        [("tvc_cmd_z", "Commanded", DEG), ("tvc_z", "Actual", DEG)],
    ),
    # Sensors (measured)
    _ch("meas_accel", "Sensors", "Accelerometer (measured)", "m/s²", BODY("meas_accel")),
    _ch("meas_gyro", "Sensors", "Gyroscope (measured)", "deg/s", BODY("meas_gyro", DEG)),
    _ch("meas_baro", "Sensors", "Barometer altitude (measured)", "m", [("meas_baro_altitude", "Barometer")]),
    _ch(
        "meas_baro_p", "Sensors", "Barometer pressure (measured)", "Pa", [("meas_baro_pressure", "Pressure")]
    ),
    _ch(
        "meas_gps_pos",
        "Sensors",
        "GPS position (measured)",
        "m",
        [(f"meas_gps_pos_{a}", n) for a, n in zip("xyz", ("East", "North", "Up"))],
    ),
    _ch(
        "meas_gps_vel",
        "Sensors",
        "GPS velocity (measured)",
        "m/s",
        [(f"meas_gps_vel_{a}", n) for a, n in zip("xyz", ("East", "North", "Up"))],
    ),
    _ch("meas_mag", "Sensors", "Magnetometer (measured)", "µT", BODY("meas_mag", 1e6)),
    # Estimator and comparisons
    _ch(
        "nav_altitude",
        "Estimation",
        "Altitude: truth, sensors and estimate",
        "m",
        [
            ("pos_z", "Truth"),
            ("meas_baro_altitude", "Barometer"),
            ("meas_gps_pos_z", "GPS"),
            ("est_pos_z", "Estimate"),
        ],
    ),
    _ch(
        "nav_velocity",
        "Estimation",
        "Vertical velocity: truth, sensors and estimate",
        "m/s",
        [("vel_z", "Truth"), ("meas_gps_vel_z", "GPS"), ("est_vel_z", "Estimate")],
    ),
    _ch(
        "nav_axial",
        "Estimation",
        "Axial acceleration: truth and measured",
        "m/s²",
        [("fsp_x", "True specific force"), ("meas_accel_x", "Accelerometer")],
    ),
    _ch(
        "est_position",
        "Estimation",
        "Estimated position",
        "m",
        [(f"est_pos_{a}", n) for a, n in zip("xyz", ("East", "North", "Up"))],
    ),
    _ch(
        "est_velocity",
        "Estimation",
        "Estimated velocity",
        "m/s",
        [(f"est_vel_{a}", n) for a, n in zip("xyz", ("East", "North", "Up"))],
    ),
    _ch("est_bias", "Estimation", "Estimated gyro bias", "deg/s", BODY("est_gyro_bias", DEG)),
]

PRESETS: dict[str, list[str]] = {
    "Flight overview": ["altitude", "speed", "load_factor", "thrust"],
    "Motion": ["position", "velocity", "acceleration"],
    "Attitude": ["attitude", "aoa", "rates"],
    "Forces and mass": ["thrust", "aero_forces", "mass", "static_margin"],
    "Navigation (truth vs sensors)": ["nav_altitude", "nav_velocity", "nav_axial"],
    "Control": ["tvc_y", "tvc_z", "attitude"],
}


def available_channels(rec: FlightRecord) -> list[Channel]:
    """Catalogue channels whose columns exist in ``rec``, plus every remaining column as a raw channel."""
    out = [c for c in CATALOG if c.available(rec)]
    used = {col for c in out for col in c.columns()}
    used |= {f"{a}" for a in ("t", "dt", "phase")}
    desc = {c.name: c for c in columns_for(rec.meta.fidelity, estimator=rec.meta.fidelity >= 5)}
    for name in rec.columns:
        if name in used or name.endswith("_new") or name in ("est_valid", "launch_detected"):
            continue
        col = desc.get(name)
        unit = col.unit if col and col.unit != "-" else ""
        title = (col.description if col else name).split(";")[0].split("(")[0].strip()
        out.append(
            Channel(
                f"raw:{name}",
                "Other columns",
                f"{title} [{name}]",
                unit,
                (Series(name, name),),
                None,
                col.description if col else "",
            )
        )
    return out


def preset_ids(name: str, channels: list[Channel]) -> list[str]:
    have = {c.id for c in channels}
    return [i for i in PRESETS.get(name, []) if i in have]
