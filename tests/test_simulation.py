import copy
import math

import numpy as np
import pytest

from rocket_sim.errors import ConfigError
from rocket_sim.simulation import FlightPhase, Simulation, run_simulation
from tests.conftest import CONFIG_G80, cfg_from


@pytest.fixture(scope="module")
def flight():
    return run_simulation(cfg_from(), seed=3)


def test_phase_sequence_and_event_log(flight):
    ph = flight.col("phase").astype(int)
    seen = [FlightPhase(p) for p in dict.fromkeys(ph.tolist())]
    assert seen == [
        FlightPhase.PRELAUNCH,
        FlightPhase.IGNITION,
        FlightPhase.POWERED_ASCENT,
        FlightPhase.BURNOUT,
        FlightPhase.COAST,
        FlightPhase.APOGEE,
        FlightPhase.DESCENT,
        FlightPhase.LANDED,
    ]
    names = [e.name for e in flight.events if e.name != "phase_change"]
    assert names == ["liftoff", "rail_exit", "burnout", "apogee", "parachute_deploy", "landing"]
    changes = [FlightPhase(int(e.info["phase"])) for e in flight.events if e.name == "phase_change"]
    assert changes == seen[1:] + [] or changes[: len(seen) - 1] == seen[1:]
    t = flight.col("t")
    assert np.all(np.diff(t) > 0)  # strictly increasing time
    ev = {e.name: e.t for e in flight.events}
    assert ev["liftoff"] < ev["rail_exit"] < ev["burnout"] < ev["apogee"] < ev["landing"]


def test_burnout_time_matches_motor(flight):
    assert flight.summary["burnout_time_s"] == pytest.approx(1.701, abs=1e-9)  # G80T curve end


def test_impact_detected_at_ground_and_never_underground(flight):
    alt = flight.col("altitude")
    assert alt.min() >= -1e-9
    ev = flight.event("landing")
    assert ev is not None and flight.col("t")[-1] == pytest.approx(ev.t)
    assert alt[-1] == pytest.approx(0.0, abs=1e-9)
    assert flight.summary["impact_speed_ms"] > 0
    assert flight.meta.status == "ok"
    # the run stops at impact: nothing recorded afterwards
    assert flight.col("phase")[-1] == int(FlightPhase.LANDED)


def test_impact_speed_of_ballistic_drop_matches_energy():
    """Vacuum, no motor thrust: drop from apogee-like state; impact speed = sqrt(2 g h)."""
    cfg = cfg_from(
        fidelity=1,
        rocket={"parachutes": []},
        environment={"gravity": {"model": "constant", "g_ms2": 9.81}},
        launch={"elevation_deg": 90, "rail_length_m": 1.0},
    )
    r = run_simulation(cfg, seed=0)
    apogee = r.summary["apogee_m"]
    # constant g, vacuum: v_impact(vertical) from energy conservation: v^2 = 2 g H (apogee at v=0)
    assert abs(r.summary["impact_vertical_speed_ms"]) == pytest.approx(math.sqrt(2 * 9.81 * apogee), rel=2e-3)


def test_vacuum_ballistic_flight_matches_rocket_equation():
    """Fidelity 0: burnout velocity/altitude vs the exact integral of the thrust curve."""
    cfg = cfg_from(
        fidelity=0, rocket={"parachutes": []}, simulation={"dt_s": 0.002}, launch={"rail_length_m": 0.0}
    )
    r = run_simulation(cfg, seed=0)
    # independent reference: integrate with scipy on the same thrust curve & impulse-proportional mass
    from scipy.integrate import solve_ivp

    sim = Simulation(cfg, seed=0)
    veh = sim.vehicle
    g = 9.80665

    def f(t, s):
        m = veh.mass_props(t).mass
        thr = veh.thrust_at(t)
        a = thr / m - g
        if s[1] <= 0 and a < 0 and t < veh.burnout_time and s[0] <= 0:
            return [0.0, 0.0]
        return [s[1], a]

    t_b = veh.burnout_time
    ts = np.linspace(0, t_b, 2001)
    sol = solve_ivp(f, (0, t_b), [0.0, 0.0], t_eval=ts, rtol=1e-11, atol=1e-11, max_step=0.002)
    bo = r.event("burnout")
    assert bo.info["speed"] == pytest.approx(sol.y[1, -1], rel=2e-4)
    assert bo.info["altitude"] == pytest.approx(sol.y[0, -1], rel=2e-3)
    # post-burnout coast to apogee: H = z_b + v_b^2/(2g)
    h_expect = sol.y[0, -1] + sol.y[1, -1] ** 2 / (2 * g)
    assert r.summary["apogee_m"] == pytest.approx(h_expect, rel=2e-3)


def test_mass_depletion_in_record(flight):
    m, mp, md = flight.col("mass"), flight.col("prop_mass"), flight.col("dry_mass")
    assert np.allclose(m, md + mp, atol=1e-12)
    assert mp[0] == pytest.approx(0.069, abs=1e-9)  # G80T propellant mass from the .eng header
    assert mp[-1] == 0.0 and np.all(np.diff(mp) <= 1e-12)
    t = flight.col("t")
    thrust = flight.col("thrust")
    # thrust only within the burn
    assert np.all(thrust[t > 1.701 + 1e-6] == 0.0)
    # impulse from the recording ~ total impulse of the motor
    imp = np.trapezoid(thrust, t)
    assert imp == pytest.approx(135.6, rel=0.01)


def test_mach_and_dynamic_pressure_logged(flight):
    mach = flight.col("mach")
    assert np.allclose(mach, flight.col("airspeed") / flight.col("speed_of_sound"), atol=1e-12)
    assert np.allclose(
        flight.col("qdyn"), 0.5 * flight.col("density") * flight.col("airspeed") ** 2, rtol=1e-12
    )
    assert 0.3 < flight.summary["max_mach"] < 0.9


def test_deterministic_same_seed_bitwise_identical():
    cfg = cfg_from(environment={"wind": {"model": "constant", "speed_ms": 4, "turbulence_sigma_ms": 1.5}})
    a = run_simulation(cfg, seed=5)
    b = run_simulation(copy.deepcopy(cfg), seed=5)
    c = run_simulation(cfg, seed=6)
    assert np.array_equal(a.data, b.data)
    assert not np.array_equal(
        a.data[: min(len(a.data), len(c.data))], c.data[: min(len(a.data), len(c.data))]
    )
    assert a.meta.simulation_id == b.meta.simulation_id


def test_never_lifts_off_when_thrust_below_weight():
    cfg = cfg_from(rocket={"dry_mass_kg": 20.0, "cg_from_nose_m": 0.55})
    r = run_simulation(cfg, seed=0)
    assert r.meta.status == "no_liftoff"
    assert r.event("liftoff") is None
    assert r.col("altitude").max() < 1e-6


def test_invalid_configs_give_useful_errors():
    with pytest.raises(ConfigError, match="dry_mass_kg"):
        cfg_from(rocket={"dry_mass_kg": -1.0})
    with pytest.raises(ConfigError, match="dt_s"):
        cfg_from(simulation={"dt_s": 0.0})
    with pytest.raises(ConfigError, match="unknown key"):
        cfg_from(rocket={"dry_masss_kg": 1.0})
    with pytest.raises(ConfigError, match="not found"):
        run_simulation(cfg_from(motor={"file": "data/motors/nonexistent.eng"}))
    with pytest.raises(ConfigError, match="cg_from_nose_m"):
        cfg_from(rocket={"cg_from_nose_m": 5.0})
    with pytest.raises(ConfigError, match="fit"):
        run_simulation(cfg_from(motor={"file": "data/motors/Cesaroni_40960O8000-P.eng"}))
    with pytest.raises(ConfigError, match="temperature_offset_k"):
        cfg_from(environment={"atmosphere": {"temperature_offset_k": 500}})


def test_fidelity_levels_run_and_agree_roughly():
    res = {}
    for lvl in (1, 2, 3):
        cfg = cfg_from(fidelity=lvl, rocket={"parachutes": []})
        res[lvl] = run_simulation(cfg, seed=0).summary["apogee_m"]
    assert res[1] > 2.5 * res[2]  # drag matters: vacuum apogee far higher
    assert res[3] == pytest.approx(res[2], rel=0.05)  # 3-DOF and 6-DOF agree within a few %


def test_fast_mode_runs_and_is_recorded():
    cfg = cfg_from(fidelity=2, fast=True, simulation={"dt_s": 0.02})
    r = run_simulation(cfg, seed=0)
    assert r.meta.fast and r.meta.fidelity == 2
    assert any("fast mode" in n for n in r.meta.notes)
    ref = run_simulation(cfg_from(fidelity=2), seed=0)
    assert r.summary["apogee_m"] == pytest.approx(ref.summary["apogee_m"], rel=0.04)


def test_sloped_terrain_ground_contact():
    cfg = cfg_from(
        environment={
            "terrain": {"model": "slope", "slope_deg": 5, "slope_direction_deg": 90},
            "wind": {"model": "constant", "speed_ms": 6, "direction_from_deg": 270},
        },
        rocket={"parachutes": [{"cd": 1.5, "diameter_m": 0.6}]},
    )
    r = run_simulation(cfg, seed=0)
    x, z = r.col("pos_x"), r.col("pos_z")
    assert z[-1] == pytest.approx(x[-1] * math.tan(math.radians(5)), abs=1e-6)  # landed ON the slope


def test_wind_effects_weathercocking_and_drift():
    """Rockets weathercock INTO the wind during boost (upwind drift, lower apogee); a parachute
    descent then carries them downwind."""
    wind = {"model": "constant", "speed_ms": 8, "direction_from_deg": 270}  # blows toward +x
    calm = run_simulation(
        cfg_from(environment={"wind": {"model": "none"}}, rocket={"parachutes": []}), seed=0
    )
    ballistic = run_simulation(cfg_from(environment={"wind": wind}, rocket={"parachutes": []}), seed=0)
    chute = run_simulation(cfg_from(environment={"wind": wind}), seed=0)
    assert abs(calm.summary["landing_x_m"]) < 1.0  # no wind -> no drift
    assert ballistic.summary["apogee_m"] < calm.summary["apogee_m"]  # weathercocking costs altitude
    assert ballistic.event("apogee").info["x"] < 0  # tilts/drifts upwind while powered
    assert chute.summary["landing_x_m"] > 100  # descent under canopy drifts downwind


def test_launch_rail_and_angle():
    r = run_simulation(
        cfg_from(
            launch={"elevation_deg": 80, "azimuth_deg": 90},
            environment={"wind": {"model": "none"}},
            rocket={"parachutes": []},
        ),
        seed=0,
    )
    assert r.summary["landing_x_m"] > 50  # launched toward the East
    assert abs(r.summary["landing_y_m"]) < 5
    # on the rail the attitude is exactly the launch attitude
    ex = r.event("rail_exit")
    t = r.col("t")
    i = np.searchsorted(t, ex.t)
    assert math.degrees(r.col("pitch")[0]) == pytest.approx(80.0, abs=1e-9)
    assert math.degrees(r.col("yaw")[0]) == pytest.approx(90.0, abs=1e-9)
    assert math.degrees(r.col("pitch")[i - 1]) == pytest.approx(80.0, abs=1e-6)


def test_record_every_decimates_but_keeps_events():
    full = run_simulation(cfg_from(rocket={"parachutes": []}), seed=0)
    dec = run_simulation(cfg_from(rocket={"parachutes": []}, simulation={"record_every": 10}), seed=0)
    assert dec.n_rows < full.n_rows / 5
    assert dec.summary["apogee_m"] == pytest.approx(full.summary["apogee_m"], rel=1e-9)
    assert dec.event("landing") is not None and dec.event("rail_exit") is not None


def test_uncontrolled_flight_is_stable_and_static_margin_reported(flight):
    sm = flight.col("static_margin")
    assert sm[0] > 1.0  # at least one caliber
    # relative-wind geometry only: 3 m/s crosswind over >= 15 m/s airspeed gives atan(3/15) = 11.3 deg
    assert flight.summary["max_aoa_ascent_deg"] < 15.0
    assert np.isfinite(flight.data).all()


def test_unstable_vehicle_is_flagged():
    cfg = cfg_from(rocket={"cg_from_nose_m": 0.95})
    sim = Simulation(cfg, seed=0)
    assert any("static margin" in w for w in sim.warnings)


def test_stop_after_apogee_truncates_run_and_trims_dataset():
    from rocket_sim.data.dataset import flight_slice

    full = run_simulation(cfg_from(), seed=0)
    cut = run_simulation(cfg_from(simulation={"stop_after_apogee_s": 2.0}), seed=0)
    assert cut.meta.status == "truncated" and cut.event("landing") is None
    assert cut.col("t")[-1] == pytest.approx(cut.summary["apogee_time_s"] + 2.0, abs=0.06)
    assert cut.summary["apogee_m"] == pytest.approx(full.summary["apogee_m"], rel=1e-9)  # same ascent
    t0, t1 = flight_slice(cut, "flight")
    assert t1 == pytest.approx(cut.col("t")[-1]) and t0 == pytest.approx(cut.event("liftoff").t)


def test_every_recorded_row_has_unit_quaternion_including_event_rows():
    r = run_simulation(cfg_from(), seed=1)
    qn = np.sqrt(sum(r.col(f"quat_{a}") ** 2 for a in "wxyz"))
    assert np.max(np.abs(qn - 1.0)) < 1e-12  # landing/apogee/rail-exit rows too


def test_input_file_hash_recorded():
    r = run_simulation(cfg_from(), seed=0)
    ((name, sha),) = r.meta.input_files.items()
    assert name.endswith("AeroTech_G80T.eng") and len(sha) == 64


def test_run_id_depends_on_motor_file_contents(tmp_path):
    base = cfg_from()
    shutil_text = (CONFIG_G80.parent.parent / "data/motors/AeroTech_G80T.eng").read_text()
    (tmp_path / "m.eng").write_text(shutil_text)
    (tmp_path / "m2.eng").write_text(shutil_text.replace("0.013 89.054", "0.013 99.054"))
    a = Simulation(cfg_from(motor={"file": str(tmp_path / "m.eng")}), seed=1)
    b = Simulation(cfg_from(motor={"file": str(tmp_path / "m2.eng")}), seed=1)
    c = Simulation(cfg_from(motor={"file": str(tmp_path / "m.eng")}), seed=1)
    assert a.simulation_id == c.simulation_id and a.simulation_id != b.simulation_id
    assert base.motor.file  # sanity
