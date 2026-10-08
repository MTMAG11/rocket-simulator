"""V1.2: strict separation of TRUTH, MEASUREMENT, ESTIMATE and COMMAND/ACTUAL.

The chain under test:  true state -> sensor models -> measurements -> estimator -> estimate -> controller -> command -> actuator.
"""

import math

import numpy as np
import pytest

from rocket_sim.config import config_from_dict
from rocket_sim.control import Command, Controller
from rocket_sim.data.schema import COLUMNS, columns_by_role, role_of
from rocket_sim.errors import ConfigError
from rocket_sim.estimation import NavigationFilter, TruthEstimator, build_estimator
from rocket_sim.estimation.estimators import Estimator
from rocket_sim.sensors import SensorReadings
from rocket_sim.simulation import Simulation, run_simulation
from tests.conftest import cfg_from


def est_cfg(**kw):
    kw.setdefault("sensors", {"accelerometer": {"saturation": 400.0}})
    return cfg_from(
        fidelity=5,
        estimator={"type": "nav_kf", "alignment_time_s": 1.0},
        motor={"ignition_delay_s": 2.0},
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
        launch={"elevation_deg": 90.0, "rail_length_m": 1.5},
        simulation={"t_max_s": 9.0},
        **kw,
    )


class Spy(Controller):
    """Records every ControlInput it is handed; commands nothing."""

    name = "spy"

    def __init__(self):
        self.inputs = []

    def update(self, inp):
        self.inputs.append(inp)
        return Command()


def test_navigation_filter_is_never_given_truth_and_truth_estimator_is_explicit(monkeypatch):
    seen = []
    orig = NavigationFilter.update

    def spy(self, t, r, truth=None):
        seen.append((truth, type(r).__name__))
        return orig(self, t, r, truth)

    monkeypatch.setattr(NavigationFilter, "update", spy)
    run_simulation(est_cfg(), seed=1)
    assert len(seen) > 100
    assert all(
        tr is None and name == "SensorReadings" for tr, name in seen
    )  # measurements in, no truth argument
    assert isinstance(
        build_estimator(cfg_from(estimator={"type": "truth"}).estimator, cfg_from().sensors, None, 0.0),
        TruthEstimator,
    )
    assert issubclass(NavigationFilter, Estimator) and not hasattr(NavigationFilter, "truth")


def test_estimates_are_a_pure_function_of_the_recorded_measurements():
    """Replay the logged measurements through a FRESH filter: it reproduces the logged estimates bit-for-bit, so nothing
    but the measurements (and the filter configuration) influenced it."""
    cfg = est_cfg()
    sim = Simulation(cfg, seed=7)
    rec = sim.run()
    nf = build_estimator(sim.cfg.estimator, sim.cfg.sensors, sim.env.gravity, sim.env.site_elevation)
    col = rec.col
    has_fix = False
    prev_t = -1.0
    n_checked = 0
    for i in range(rec.n_rows):
        t = float(col("t")[i])
        if t <= prev_t:  # event rows repeat a time; the estimator ran once for that time
            continue
        prev_t = t
        has_fix = has_fix or bool(col("meas_gps_new")[i])
        r = SensorReadings(
            np.array([col(f"meas_accel_{a}")[i] for a in "xyz"]),
            np.array([col(f"meas_gyro_{a}")[i] for a in "xyz"]),
            float(col("meas_baro_pressure")[i]),
            np.array([col(f"meas_gps_{k}_{a}")[i] for k in ("pos", "vel") for a in "xyz"]),
            np.array([col(f"meas_mag_{a}")[i] for a in "xyz"]),
            bool(col("meas_accel_new")[i]),
            bool(col("meas_gyro_new")[i]),
            bool(col("meas_baro_new")[i]),
            bool(col("meas_gps_new")[i]),
            bool(col("meas_mag_new")[i]),
            has_fix,
        )
        e = nf.update(t, r)
        assert e.valid == bool(col("est_valid")[i])
        if e.valid:
            n_checked += 1
            assert (e.position[0], e.position[1], e.position[2]) == (
                col("est_pos_x")[i],
                col("est_pos_y")[i],
                col("est_pos_z")[i],
            )
            assert e.velocity[2] == col("est_vel_z")[i]
            assert e.quaternion[0] == col("est_quat_w")[i]
    assert n_checked > 300


def test_estimate_differs_from_truth_and_tracks_it():
    rec = run_simulation(est_cfg(), seed=2)
    t = rec.col("t")
    m = (t > 4.0) & (t < 7.0)
    err = rec.col("est_pos_z")[m] - rec.col("pos_z")[m]
    assert np.max(np.abs(err)) > 1e-3  # a real estimate, not a copy of the truth
    assert np.sqrt(np.mean(err**2)) < 5.0  # but a useful one


def test_controller_input_equals_the_logged_estimate_not_the_truth():
    cfg = est_cfg(controller={"type": "none", "rate_hz": 50})
    spy = Spy()
    rec = Simulation(cfg, seed=3, controller=spy).run()
    assert spy.inputs and all(i.state_source == "estimate" for i in spy.inputs)
    assert rec.meta.controller_state_source == "estimate"
    t = rec.col("t")
    n_diff = 0
    for inp in spy.inputs:
        if not inp.valid:
            continue
        j = int(np.argmin(np.abs(t - inp.t)))
        assert inp.position == (rec.col("est_pos_x")[j], rec.col("est_pos_y")[j], rec.col("est_pos_z")[j])
        assert inp.velocity[2] == rec.col("est_vel_z")[j]
        if abs(inp.position[2] - rec.col("pos_z")[j]) > 1e-6:
            n_diff += 1
    assert n_diff > 20  # the input is not the true position (it only agrees with the ESTIMATE)


def test_truth_fed_controller_is_flagged_everywhere():
    cfg = est_cfg(controller={"type": "none", "rate_hz": 50, "use_truth": True})
    spy = Spy()
    rec = Simulation(cfg, seed=3, controller=spy).run()
    assert all(i.state_source == "truth" for i in spy.inputs)
    j = int(np.argmin(np.abs(rec.col("t") - spy.inputs[100].t)))
    assert spy.inputs[100].position[2] == rec.col("pos_z")[j]
    assert rec.meta.controller_state_source == "truth"
    assert any("TRUE state" in w for w in rec.meta.warnings)


def test_state_source_auto_falls_back_to_truth_loudly_and_estimate_refuses():
    low = cfg_from(
        fidelity=3,
        controller={"type": "schedule", "params": {"table": [[0, 0, 0]]}},
        simulation={"t_max_s": 3},
    )
    rec = run_simulation(low, seed=0)  # no sensors / estimator exist at fidelity 3
    assert rec.meta.controller_state_source == "truth" and any("TRUE state" in w for w in rec.meta.warnings)
    with pytest.raises(ConfigError, match="state_source"):
        cfg_from(
            fidelity=3,
            controller={"type": "schedule", "state_source": "estimate", "params": {"table": [[0, 0, 0]]}},
        )
    with pytest.raises(ConfigError, match="contradicts"):
        cfg_from(
            fidelity=5,
            estimator={"type": "nav_kf", "alignment_time_s": 1.0},
            motor={"ignition_delay_s": 2.0},
            controller={
                "type": "schedule",
                "use_truth": True,
                "state_source": "estimate",
                "params": {"table": [[0, 0, 0]]},
            },
        )
    ok = cfg_from(
        fidelity=5,
        estimator={"type": "nav_kf", "alignment_time_s": 1.0},
        motor={"ignition_delay_s": 2.0},
        controller={"type": "schedule", "state_source": "estimate", "params": {"table": [[0, 0, 0]]}},
        simulation={"t_max_s": 4},
    )
    assert run_simulation(ok, seed=0).meta.controller_state_source == "estimate"


def test_no_controller_means_state_source_none():
    assert (
        run_simulation(cfg_from(fidelity=3, simulation={"t_max_s": 2}), seed=0).meta.controller_state_source
        == "none"
    )


def perfect_sensors():
    z = {
        "noise_std": 0.0,
        "bias_std": 0.0,
        "bias_walk_std": 0.0,
        "scale_error_std": 0.0,
        "quantization": 0.0,
        "saturation": 0.0,
        "latency_s": 0.0,
    }
    return {k: dict(z) for k in ("accelerometer", "gyroscope", "barometer", "magnetometer")} | {
        "gps": dict(z)
    }


def test_measurement_equals_truth_only_when_every_error_term_is_zero():
    perfect = cfg_from(
        fidelity=4,
        sensors=perfect_sensors(),
        simulation={"t_max_s": 6},
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
    )
    r = run_simulation(perfect, seed=0)
    new = r.col("meas_accel_new") > 0.5
    k = new & (r.col("t") > 1.0)
    for a in "xyz":
        assert np.array_equal(
            r.col(f"meas_accel_{a}")[k], r.col(f"fsp_{a}")[k]
        )  # ideal sensor reads the TRUE specific force
    gnew = r.col("meas_gyro_new") > 0.5
    assert np.array_equal(r.col("meas_gyro_y")[gnew], r.col("omega_q")[gnew])
    # the same flight with the default (imperfect) sensors: measurements differ from truth
    noisy = run_simulation(
        cfg_from(
            fidelity=4,
            simulation={"t_max_s": 6},
            rocket={"parachutes": []},
            environment={"wind": {"model": "none"}},
        ),
        seed=0,
    )
    kn = (noisy.col("meas_accel_new") > 0.5) & (noisy.col("t") > 1.0)
    assert np.max(np.abs(noisy.col("meas_accel_z")[kn] - noisy.col("fsp_z")[kn])) > 0.05
    # and the truth itself is identical in both runs (sensors never alter the physics)
    # (the integration grid is cut at sensor sample times, so different sensor RATES shift the grid; the physics itself is unchanged)
    tt = np.linspace(0.5, 5.9, 50)
    assert (
        np.max(
            np.abs(
                np.interp(tt, r.col("t"), r.col("pos_z")) - np.interp(tt, noisy.col("t"), noisy.col("pos_z"))
            )
        )
        < 0.1
    )


def test_measurement_noise_statistics_match_the_configuration():
    s = perfect_sensors()
    s["accelerometer"]["noise_std"] = 0.2
    s["accelerometer"]["rate_hz"] = 400.0
    cfg = cfg_from(
        fidelity=4,
        sensors=s,
        motor={"ignition_delay_s": 4.0},
        simulation={"t_max_s": 4.0},
        rocket={"parachutes": []},
    )
    r = run_simulation(cfg, seed=5)
    new = r.col("meas_accel_new") > 0.5
    resid = r.col("meas_accel_z")[new] - r.col("fsp_z")[new]
    assert len(resid) > 1000
    assert np.std(resid) == pytest.approx(0.2, rel=0.08) and abs(np.mean(resid)) < 0.02


def test_saturation_clips_the_measurement_but_not_the_truth():
    s = perfect_sensors()
    s["accelerometer"]["saturation"] = 60.0
    cfg = cfg_from(
        fidelity=4,
        sensors=s,
        simulation={"t_max_s": 4},
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
    )
    r = run_simulation(cfg, seed=0)
    assert (
        np.max(np.abs(r.col("fsp_x"))) > 100.0
    )  # the true specific force of this motor exceeds the part's range
    assert np.max(np.abs(r.col("meas_accel_x"))) <= 60.0 + 1e-12


def test_barometer_altitude_is_derived_from_pressure_not_copied_from_truth():
    s = perfect_sensors()
    cfg = cfg_from(
        fidelity=4,
        sensors=s,
        simulation={"t_max_s": 12},
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
    )
    r = run_simulation(cfg, seed=0)
    k = (r.col("meas_baro_new") > 0.5) & (r.col("t") > 1.0)
    assert (
        np.max(np.abs(r.col("meas_baro_pressure")[k] - r.col("pressure")[k])) < 1e-6
    )  # ideal baro: true static pressure
    # ISA inversion recovers the true height (ISA atmosphere, so no model error): to well under a metre
    assert np.max(np.abs(r.col("meas_baro_altitude")[k] - r.col("altitude")[k])) < 1.0


def test_sensor_realisations_are_seeded_and_independent_of_the_truth():
    cfg = cfg_from(
        fidelity=4,
        simulation={"t_max_s": 5},
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
    )
    a, b, c = run_simulation(cfg, seed=11), run_simulation(cfg, seed=11), run_simulation(cfg, seed=12)
    assert np.array_equal(a.col("meas_accel_x"), b.col("meas_accel_x"))  # deterministic given the seed
    assert not np.array_equal(a.col("meas_accel_x"), c.col("meas_accel_x"))  # seed-dependent
    assert np.array_equal(a.col("pos_z"), c.col("pos_z"))  # truth does not depend on the sensor seed


def test_every_column_has_exactly_one_role_and_roles_partition_by_prefix():
    for c in COLUMNS:
        r = role_of(c)
        if c.name.startswith("meas_"):
            assert r == "measurement"
        elif c.name.startswith("est_") or c.name == "launch_detected":
            assert r == "estimate"
        elif c.name.startswith(("tvc_cmd", "fin_cmd", "fin_dcmd")):
            assert r == "command"
        elif c.name.startswith(("tvc_", "fin_")):
            assert r == "actual"
        else:
            assert r in ("truth", "time")
    truth = set(columns_by_role("truth"))
    assert {
        "pos_z",
        "vel_z",
        "acc_z",
        "quat_w",
        "omega_p",
        "mass",
        "cg",
        "ixx",
        "izz",
        "ixy",
        "cg_y",
        "fsp_x",
    } <= truth
    assert not (truth & set(columns_by_role("measurement")))
    assert {"est_pos_z", "est_gyro_bias_x"} <= set(columns_by_role("estimate"))
    assert {"tvc_cmd_y", "fin_dcmd_0"} <= set(columns_by_role("command"))
    assert {"tvc_y", "fin_0"} <= set(columns_by_role("actual"))


def test_output_carries_truth_measurement_estimate_and_control_groups_together():
    rec = run_simulation(
        est_cfg(
            controller={
                "type": "schedule",
                "rate_hz": 50,
                "use_truth": True,
                "params": {"table": [[0, 1.0, 0]]},
            }
        ),
        seed=4,
    )
    for need in (
        "pos_x",
        "vel_x",
        "acc_x",
        "quat_w",
        "omega_p",
        "mass",
        "cg",
        "izz",
        "fsp_x",  # truth
        "meas_accel_x",
        "meas_gyro_x",
        "meas_baro_pressure",
        "meas_gps_pos_x",
        "meas_mag_x",  # sensors
        "est_pos_x",
        "est_vel_x",
        "est_quat_w",
        "est_gyro_bias_x",  # estimate
        "tvc_cmd_y",
        "tvc_y",
    ):  # control
        assert rec.has(need), need
    # the actuator is not the command: with lag and rate limits they differ during the burn
    assert np.max(np.abs(rec.col("tvc_cmd_y") - rec.col("tvc_y"))) > 1e-4


def test_estimated_gyro_bias_is_logged_and_close_to_the_true_pad_bias():
    cfg = est_cfg(
        sensors={"accelerometer": {"saturation": 400.0}, "gyroscope": {"bias_std": 0.01, "noise_std": 0.002}}
    )
    sim = Simulation(cfg, seed=9)
    rec = sim.run()
    assert sim.sensors is not None and sim.sensors.gyro is not None
    true_bias = sim.sensors.gyro.bias
    t = rec.col("t")
    i = int(np.argmax(t > 6.0))
    est = np.array([rec.col(f"est_gyro_bias_{a}")[i] for a in "xyz"])
    assert np.allclose(est, true_bias, atol=3e-3)  # pad-estimated with 2 mrad/s noise: within a few mrad/s
    assert np.linalg.norm(est) > 1e-3 and math.isfinite(float(np.linalg.norm(est)))


def test_config_hash_includes_the_state_source_choice():
    from rocket_sim.config import config_hash

    a = cfg_from(controller={"type": "none"})
    b = cfg_from(controller={"type": "none", "state_source": "truth"})
    assert config_hash(a) != config_hash(b)
    assert config_from_dict is not None


def test_controller_receives_estimated_attitude_rates_and_phase_not_the_truth():
    cfg = est_cfg(controller={"type": "none", "rate_hz": 50})
    spy = Spy()
    rec = Simulation(cfg, seed=3, controller=spy).run()
    t = rec.col("t")
    n_q = n_w = 0
    for inp in spy.inputs:
        if not inp.valid:
            continue
        j = int(np.argmin(np.abs(t - inp.t)))
        assert inp.quaternion == tuple(rec.col(f"est_quat_{a}")[j] for a in "wxyz")  # the estimate, exactly
        tq = tuple(rec.col(f"quat_{a}")[j] for a in "wxyz")
        n_q += inp.quaternion != tq
        n_w += inp.omega != (rec.col("omega_p")[j], rec.col("omega_q")[j], rec.col("omega_r")[j])
        assert inp.phase in (0, 2)  # estimated phase only: never the true phase (apogee / descent codes)
    assert n_q > 20 and n_w > 20
    truth_phases = set(rec.col("phase").astype(int).tolist())
    assert truth_phases - {0, 2}  # the true phases do include others, so the check above is meaningful


def test_truth_estimator_means_a_truth_fed_controller_and_says_so():
    cfg = cfg_from(
        fidelity=3,
        estimator={"type": "truth"},
        controller={"type": "none", "rate_hz": 50},
        simulation={"t_max_s": 3},
    )
    spy = Spy()
    rec = Simulation(cfg, seed=0, controller=spy).run()
    assert rec.meta.controller_state_source == "truth" and any("TRUE state" in w for w in rec.meta.warnings)
    assert all(i.state_source == "truth" for i in spy.inputs)


def test_navigation_filter_requires_its_magnetometer_and_accelerometer():
    for off in ("magnetometer", "accelerometer"):
        with pytest.raises(ConfigError, match="both must be enabled"):
            est_cfg(sensors={off: {"enabled": False}})
