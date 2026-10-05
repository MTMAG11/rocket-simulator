"""Independent cross-check: this simulator vs RocketPy 1.x on an IDENTICAL simplified vehicle.

Same geometry, mass, motor thrust curve, constant Cd, standard atmosphere, variable gravity,
no wind, no parachute (flight ends at apogee). Disagreement between two independent codes is
investigated, not assumed to be an error of either. Run:  python validation_data/crosscheck_rocketpy.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import types  # noqa: E402

# netCDF4 (only needed for reanalysis/forecast atmospheres) may be blocked by OS application-control
# policies; this check uses the standard atmosphere, so a stub import is sufficient.
try:
    import netCDF4  # noqa: F401
except ImportError:
    sys.modules["netCDF4"] = types.ModuleType("netCDF4")

from rocketpy import Environment, Flight, GenericMotor, Rocket  # noqa: E402

from rocket_sim.config import config_from_dict  # noqa: E402
from rocket_sim.simulation import run_simulation  # noqa: E402

CD = 0.45
ENG = ROOT / "validation_data" / "raw" / "Cesaroni_7579M1520-P.eng"

# Prometheus-like vehicle (see prometheus_sim.yaml), tail_to_nose coordinates for RocketPy
L, D = 2.229, 0.1397
CG_FROM_NOSE = 1.2741
MASS = 13.93


def run_mine(elevation: float) -> dict:
    data = {
        "fidelity": 3,
        "simulation": {"dt_s": 0.005, "descent_dt_s": 0.05, "t_max_s": 120},
        "rocket": {
            "body_diameter_m": D,
            "body_length_m": L,
            "nose": {"shape": "ogive", "length_m": 0.742},
            "fins": {
                "count": 3,
                "root_chord_m": 0.268,
                "tip_chord_m": 0.136,
                "span_m": 0.130,
                "sweep_m": 0.066,
                "thickness_m": 0.003,
                "position_from_nose_m": 1.956,
            },
            "dry_mass_kg": MASS,
            "cg_from_nose_m": CG_FROM_NOSE,
            "inertia": {"ixx_kgm2": 0.05, "iyy_kgm2": 4.87},
            "motor_aft_from_nose_m": L,
            "aero": {"model": "constant", "cd": CD, "cn_alpha": None},
        },
        "motor": {"file": str(ENG)},
        "environment": {
            "atmosphere": {"model": "isa"},
            "gravity": {"model": "inverse_square"},
            "wind": {"model": "none"},
            "site_elevation_msl_m": 1401.0,
        },
        "launch": {"elevation_deg": elevation, "azimuth_deg": 75.0, "rail_length_m": 5.18},
    }
    # constant aero needs Barrowman CN/CP to be comparable: take them from the build-up model
    from rocket_sim.simulation import Simulation

    probe = Simulation(config_from_dict({**data, "rocket": {**data["rocket"], "aero": {"model": "buildup"}}}))
    c = probe.vehicle.aero.coefficients(0.3, 1e7, False)
    data["rocket"]["aero"] = {
        "model": "constant",
        "cd": CD,
        "cn_alpha": c.cn_alpha,
        "x_cp_from_nose_m": c.x_cp,
        "crossflow_cd": 1.2,
    }
    cfg = config_from_dict(data, base_dir=ROOT)
    r = run_simulation(cfg, seed=0)
    return {
        "apogee": r.summary["apogee_m"],
        "t_apogee": r.summary["apogee_time_s"],
        "vmax": r.summary["max_velocity_ms"],
        "x_apogee": r.event("apogee").info["x"],
        "y_apogee": r.event("apogee").info["y"],
        "rail_exit_v": r.summary["rail_exit_velocity_ms"],
        "cp": c.x_cp,
        "cn": c.cn_alpha,
    }


def run_rocketpy(elevation: float, cp_cn: tuple[float, float]) -> dict:
    env = Environment(latitude=32.939377, longitude=-106.911986, elevation=1401)
    env.set_atmospheric_model(type="standard_atmosphere")
    motor = GenericMotor(
        thrust_source=str(ENG),
        burn_time=4.897,
        propellant_initial_mass=3.737,
        dry_mass=2.981,
        chamber_radius=0.064,
        chamber_height=0.548,
        chamber_position=0.274,
        nozzle_radius=0.027,
    )
    rocket = Rocket(
        radius=D / 2,
        mass=MASS,
        inertia=(4.87, 4.87, 0.05),
        power_off_drag=CD,
        power_on_drag=CD,
        center_of_mass_without_motor=L - CG_FROM_NOSE,
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=0)
    rocket.add_nose(length=0.742, kind="tangent", position=L)
    rocket.add_trapezoidal_fins(
        n=3, span=0.13, root_chord=0.268, tip_chord=0.136, position=0.273, sweep_length=0.066
    )
    fl = Flight(
        rocket=rocket,
        environment=env,
        rail_length=5.18,
        inclination=elevation,
        heading=75,
        terminate_on_apogee=True,
    )
    return {
        "apogee": fl.apogee - fl.env.elevation,
        "t_apogee": fl.apogee_time,
        "vmax": fl.max_speed,
        "x_apogee": fl.x(fl.apogee_time),
        "y_apogee": fl.y(fl.apogee_time),
        "rail_exit_v": fl.out_of_rail_velocity,
        "static_margin_cal": rocket.static_margin(0),
    }


def main() -> None:
    for elev in (90.0, 80.0):
        mine = run_mine(elev)
        theirs = run_rocketpy(elev, (mine["cn"], mine["cp"]))
        print(f"\nlaunch elevation {elev:.0f} deg")
        print(f"{'':<22}{'this sim':>12}{'RocketPy':>12}{'diff %':>9}")
        for k in ("apogee", "t_apogee", "vmax", "rail_exit_v"):
            a, b = mine[k], theirs[k]
            print(f"{k:<22}{a:12.2f}{b:12.2f}{100 * (a - b) / b:9.2f}")
        horiz_a = float(np.hypot(mine["x_apogee"], mine["y_apogee"]))
        horiz_b = float(np.hypot(theirs["x_apogee"], theirs["y_apogee"]))
        print(f"{'horizontal @apogee':<22}{horiz_a:12.1f}{horiz_b:12.1f}")
        print(
            f"static margin: this sim CP {mine['cp']:.3f} m from nose; RocketPy {theirs['static_margin_cal']:.2f} cal"
        )


if __name__ == "__main__":
    main()
