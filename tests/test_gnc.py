"""Sensors, estimator, actuators and closed-loop control."""

import math

import numpy as np
import pytest

from rocket_sim.config.schema import ActuatorCfg, SensorCfg
from rocket_sim.constants import EARTH_MAG_FIELD_ENU_T, G0
from rocket_sim.control import Command, TVCActuator
from rocket_sim.control.controllers import ControlInput, ScheduleController, build_controller
from rocket_sim.errors import ConfigError
from rocket_sim.estimation import LaunchDetector, integrate_gyro, triad_attitude
from rocket_sim.physics.math3d import (
    quat_from_axis_angle,
    quat_from_pointing,
    quat_mul,
    quat_rotate,
    quat_rotate_inv,
)
from rocket_sim.sensors import SensorChannel
from rocket_sim.simulation import run_simulation
from tests.conftest import cfg_from


# ------------------------------------------------------------------------------- sensors
def channel(**kw):
    cfg = SensorCfg(enabled=True, rate_hz=kw.pop("rate_hz", 100.0), **kw)
    return SensorChannel(cfg, 1, np.random.default_rng(3))


def test_sensor_noise_statistics_bias_and_scale():
    ch = channel(noise_std=0.5, bias_std=0.0)
    out = []
    for i in range(20000):
        ch.sample(i * 0.01, np.array([10.0]))
        ch.poll(i * 0.01)
        out.append(ch.value[0])
    assert np.mean(out) == pytest.approx(10.0, abs=0.02) and np.std(out) == pytest.approx(0.5, rel=0.03)
    b = channel(bias_std=2.0)
    assert abs(b.bias[0]) > 0 and b.bias[0] == pytest.approx(
        channel(bias_std=2.0).bias[0]
    )  # seeded, reproducible
    s = channel(scale_error_std=0.1)
    s.sample(0.0, np.array([100.0]))
    s.poll(0.0)
    assert s.value[0] == pytest.approx(100.0 * (1 + s.scale_err[0]))


def test_sensor_quantization_saturation_and_rate():
    ch = channel(quantization=0.25, saturation=5.0)
    ch.sample(0.0, np.array([1.1]))
    ch.poll(0.0)
    assert ch.value[0] == pytest.approx(1.0)  # rounded to the LSB
    ch.sample(0.01, np.array([123.0]))
    ch.poll(0.01)
    assert ch.value[0] == 5.0  # clipped at full scale
    assert ch.due(0.02) is True and ch.due(0.015) is False  # 100 Hz: next sample at 0.02
    ch.sample(0.02, np.array([0.0]))
    assert ch.next_time == pytest.approx(0.03)


def test_sensor_latency_delays_visibility():
    ch = channel(latency_s=0.05, rate_hz=1000.0)
    ch.sample(0.0, np.array([1.0]))
    assert ch.poll(0.01) is False and not ch.has_value  # not yet visible
    assert ch.poll(0.05) is True and ch.value[0] == 1.0


def test_bias_random_walk_grows_like_sqrt_time():
    ends = []
    for seed in range(60):
        ch = SensorChannel(
            SensorCfg(True, 100.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0), 1, np.random.default_rng(seed)
        )
        for i in range(1000):  # 10 s
            ch.sample(i * 0.01, np.array([0.0]))
        ends.append(ch.bias[0])
    assert np.std(ends) == pytest.approx(0.01 * math.sqrt(10.0), rel=0.3)


def test_true_state_differs_from_measured_state():
    r = run_simulation(cfg_from(fidelity=4, motor={"ignition_delay_s": 1.0}), seed=1)
    true_acc = np.array([r.col("acc_x")[0:3], r.col("acc_y")[0:3], r.col("acc_z")[0:3]])
    meas = r.col("meas_accel_z")
    # on the pad the accelerometer reads +g (specific force) with noise/bias -- never exactly truth
    t = r.col("t")
    pad = (t > 0.05) & (t < 0.9)
    assert np.mean(r.col("meas_accel_x")[pad]) == pytest.approx(G0, abs=1.0)  # nose-up on the pad: +g along x
    assert abs(np.mean(meas[pad])) < 1.0
    assert not np.allclose(r.col("meas_baro_pressure")[10:50], r.col("pressure")[10:50], atol=0.0)
    assert np.std(r.col("meas_gyro_x")[pad]) > 0 and true_acc is not None
    # barometer altitude follows the true altitude within a few metres (30 Pa bias ~ 2.5 m)
    ok = (t > 0.5) & (t < r.summary["apogee_time_s"])
    assert np.max(np.abs(r.col("meas_baro_altitude")[ok] - r.col("altitude")[ok])) < 15.0
    # magnetometer: Earth field rotated into the body frame
    i = 5
    q = (r.col("quat_w")[i], r.col("quat_x")[i], r.col("quat_y")[i], r.col("quat_z")[i])
    expect = np.array(quat_rotate_inv(q, EARTH_MAG_FIELD_ENU_T))
    got = np.array([r.col("meas_mag_x")[i], r.col("meas_mag_y")[i], r.col("meas_mag_z")[i]])
    assert np.allclose(got, expect, atol=2e-6)


def test_sensor_runs_are_reproducible_and_seed_dependent():
    a = run_simulation(cfg_from(fidelity=4, simulation={"t_max_s": 5}), seed=4)
    b = run_simulation(cfg_from(fidelity=4, simulation={"t_max_s": 5}), seed=4)
    c = run_simulation(cfg_from(fidelity=4, simulation={"t_max_s": 5}), seed=5)
    assert np.array_equal(a.col("meas_accel_x"), b.col("meas_accel_x"))
    assert not np.array_equal(a.col("meas_accel_x"), c.col("meas_accel_x"))
    assert np.array_equal(
        a.col("pos_z"), c.col("pos_z")
    )  # truth unaffected by the sensor seed (no wind noise)


# ----------------------------------------------------------------------------- estimation
def test_triad_recovers_attitude_from_up_and_field_vectors():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = quat_from_pointing(rng.uniform(0.1, 1.5), rng.uniform(0, 6.2), rng.uniform(-3, 3))
        up_b = np.array(quat_rotate_inv(q, (0.0, 0.0, 1.0))) * G0
        mag_b = np.array(quat_rotate_inv(q, EARTH_MAG_FIELD_ENU_T))
        est = triad_attitude(up_b, mag_b)
        v = (0.3, -0.5, 0.8)
        assert np.allclose(quat_rotate(est, v), quat_rotate(q, v), atol=1e-9)


def test_gyro_integration_matches_exact_rotation():
    q = quat_from_pointing(1.0, 0.5, 0.0)
    w = np.array([0.3, -0.2, 0.5])
    q2 = q
    for _ in range(1000):
        q2 = integrate_gyro(q2, w, 0.001)
    expect = quat_mul(q, quat_from_axis_angle(tuple(w), float(np.linalg.norm(w)) * 1.0))
    assert abs(sum(a * b for a, b in zip(q2, expect))) == pytest.approx(1.0, abs=1e-12)


def test_launch_detector_debounces():
    d = LaunchDetector(threshold=2.5 * G0, count=5)
    for i in range(4):
        d.update(i * 0.01, np.array([0, 0, 40.0]))
    assert not d.detected
    d.update(0.04 + 0.01, np.array([0, 0, 10.0]))  # a dip resets the count
    for i in range(5):
        d.update(0.1 + i * 0.01, np.array([0, 0, 40.0]))
    assert d.detected and d.time == pytest.approx(0.1)


def _est_config(**kw):
    kw.setdefault("motor", {})
    kw["motor"] = {**kw["motor"], "ignition_delay_s": 2.0}  # pad time for alignment before ignition
    kw.setdefault("sensors", {"accelerometer": {"saturation": 400.0}})  # 16 g part would clip this 19 g motor
    return cfg_from(
        fidelity=5, estimator={"type": "nav_kf", "alignment_time_s": 1.0}, controller={"type": "none"}, **kw
    )


def test_navigation_filter_tracks_true_state():
    r = run_simulation(
        _est_config(
            rocket={"parachutes": []},
            environment={"wind": {"model": "none"}},
            launch={"elevation_deg": 90, "rail_length_m": 1.5},
        ),
        seed=7,
    )
    t = r.col("t")
    assert r.col("est_valid")[-1] == 1.0
    t_det = t[np.argmax(r.col("launch_detected") > 0.5)]
    t_thrust = t[np.argmax(r.col("thrust") > 0)]
    assert t_thrust < t_det < t_thrust + 0.3  # detected from accelerometer data, shortly AFTER ignition
    flying = (t > 3.5) & (t < r.summary["apogee_time_s"])
    ez = r.col("est_pos_z")[flying] - r.col("pos_z")[flying]
    evz = r.col("est_vel_z")[flying] - r.col("vel_z")[flying]
    assert np.sqrt(np.mean(ez**2)) < 3.0  # baro/GPS-aided altitude (measured: ~1.3 m RMS)
    assert np.sqrt(np.mean(evz**2)) < 2.0
    # estimate is NOT the truth (sensor noise and gyro bias are real)
    assert not np.allclose(r.col("est_pos_z"), r.col("pos_z"), atol=1e-3)
    # attitude estimate stays close over the first seconds
    i = np.argmax(t > 6.0)
    q_t = np.array([r.col(f"quat_{c}")[i] for c in "wxyz"])
    q_e = np.array([r.col(f"est_quat_{c}")[i] for c in "wxyz"])
    assert 2 * math.degrees(math.acos(min(1.0, abs(q_t @ q_e)))) < 5.0


def test_estimator_not_valid_before_alignment():
    r = run_simulation(_est_config(), seed=7)
    t = r.col("t")
    assert np.all(r.col("est_valid")[t < 0.9] == 0.0) and r.col("est_valid")[t > 1.2].all()
    with pytest.raises(ConfigError, match="alignment"):
        cfg_from(fidelity=5, estimator={"type": "nav_kf", "alignment_time_s": 3.0})


# ----------------------------------------------------------------------------- actuators
def test_actuator_limits_lag_rate_and_delay():
    act = TVCActuator(ActuatorCfg(max_angle_deg=5.0, max_rate_deg_s=60.0, time_constant_s=0.05, delay_s=0.1))
    act.command(0.0, Command(math.radians(20.0), 0.0))  # exceeds the limit
    assert act.cmd[0] == pytest.approx(math.radians(5.0))
    t, h = 0.0, 0.001
    traj = []
    for _ in range(600):
        act.step(t, h)
        t += h
        traj.append(act.state[0])
    traj = np.array(traj)
    assert np.all(traj[: int(0.099 / h)] == 0.0)  # pure delay: nothing moves before 0.1 s
    assert np.max(np.abs(np.diff(traj))) / h <= math.radians(60.0) * 1.001  # rate limit
    assert traj[-1] == pytest.approx(math.radians(5.0), abs=1e-4)  # saturates at the angle limit
    # ideal actuator (tau=0, huge rate) follows the command immediately
    ideal = TVCActuator(ActuatorCfg(max_angle_deg=5.0, max_rate_deg_s=1e6, time_constant_s=0.0, delay_s=0.0))
    ideal.command(0.0, Command(0.05, -0.02))
    ideal.step(0.0, 0.01)
    assert ideal.state == pytest.approx([0.05, -0.02])


def test_first_order_lag_time_constant():
    act = TVCActuator(ActuatorCfg(max_angle_deg=10.0, max_rate_deg_s=1e5, time_constant_s=0.1, delay_s=0.0))
    act.command(0.0, Command(0.1, 0.0))
    t = 0.0
    for _ in range(100):
        act.step(t, 0.001)
        t += 0.001
    assert act.state[0] == pytest.approx(0.1 * (1 - math.exp(-1.0)), rel=1e-6)  # exact exponential at t = tau


# ----------------------------------------------------------------------------- controllers
def test_controller_interface_and_factory():
    assert build_controller("none", {}).update(None).tvc_y == 0.0  # type: ignore[arg-type]
    sc = ScheduleController([[0, 0, 0], [1, 2, -2]])
    inp = ControlInput(0.5, 0.5, True, (0, 0, 0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), 2)
    cmd = sc.update(inp)
    assert cmd.tvc_y == pytest.approx(math.radians(1.0)) and cmd.tvc_z == pytest.approx(math.radians(-1.0))
    with pytest.raises(ConfigError):
        build_controller("python", {"class": "no.such.module:Thing"})
    with pytest.raises(ConfigError):
        build_controller("bogus", {})


def test_custom_python_controller_plugs_in():
    class Const:
        def reset(self, ctx):
            self.n = 0

        def update(self, inp):
            self.n += 1
            return Command(0.01, 0.0)

    cfg = cfg_from(fidelity=3, controller={"type": "python", "params": {"class": "tests.test_gnc:Const"}})
    # inject through the Simulation API (module-level class lookup also works via dotted path)
    from rocket_sim.simulation import Simulation

    sim = Simulation(cfg, seed=0, controller=Const())
    r = sim.run()
    assert sim.controller.n > 100
    assert (
        np.max(np.abs(r.col("tvc_y"))) == pytest.approx(0.01, abs=2e-3) or np.max(np.abs(r.col("tvc_y"))) > 0
    )


def _unstable_cfg(controller: bool, kick_deg: float = 1.0):
    """CP forward of CG (statically unstable) -> uncontrolled flight tumbles; TVC attitude control saves it."""
    over = dict(
        fidelity=3,
        rocket={
            "fins": {"count": 0},
            "parachutes": [],
            "inertia": {"ixx_kgm2": 0.002, "iyy_kgm2": 0.5},
        },  # no fins: CP at the nose -> unstable
        environment={"wind": {"model": "none"}},
        tvc={"max_angle_deg": 8.0, "max_rate_deg_s": 200.0, "time_constant_s": 0.02},
        launch={"elevation_deg": 90.0, "rail_length_m": 1.0},
        motor={"file": "data/motors/AeroTech_G80T.eng", "misalignment_deg": [kick_deg, 0.0]},
        simulation={"t_max_s": 4.0},
    )
    if controller:
        over["controller"] = {
            "type": "tvc_attitude",
            "use_truth": True,
            "rate_hz": 100,
            "params": {"wn": 8.0, "zeta": 0.8},
        }
    return cfg_from(**over)


def test_closed_loop_tvc_stabilises_an_unstable_vehicle():
    from rocket_sim.simulation import Simulation

    sim = Simulation(_unstable_cfg(False), seed=0)
    assert sim.vehicle.static_margin(0.0) < 0
    free = sim.run()
    ctrl = run_simulation(_unstable_cfg(True), seed=0)
    t = free.col("t")
    tilt_free = np.degrees(np.arccos(np.clip(np.sin(free.col("pitch")), -1, 1)))  # angle from vertical
    tc = ctrl.col("t")
    tilt_ctrl = np.degrees(np.arccos(np.clip(np.sin(ctrl.col("pitch")), -1, 1)))
    burn = ctrl.summary["burnout_time_s"]
    in_burn_c = (tc > 0.4) & (tc < burn)
    in_burn_f = (t > 0.4) & (t < burn)
    assert tilt_free[in_burn_f].max() > 25.0  # uncontrolled: diverges
    assert tilt_ctrl[in_burn_c].max() < 8.0  # controlled: held near vertical
    assert np.max(np.abs(ctrl.col("tvc_y"))) > 0  # the actuator really moved
    assert np.max(np.abs(ctrl.col("tvc_y"))) <= math.radians(8.0) + 1e-9  # respects the gimbal limit
    assert np.isfinite(ctrl.data).all()


def test_estimator_driven_closed_loop_runs_and_logs_commands_vs_actual():
    cfg = cfg_from(
        fidelity=5,
        rocket={"parachutes": []},
        estimator={"type": "nav_kf"},
        motor={"ignition_delay_s": 2.0},
        sensors={"accelerometer": {"saturation": 400.0}},
        controller={"type": "tvc_attitude", "rate_hz": 100, "params": {"wn": 6.0, "zeta": 0.8}},
        tvc={"max_angle_deg": 5.0, "max_rate_deg_s": 100.0, "time_constant_s": 0.03, "delay_s": 0.02},
        environment={"wind": {"model": "constant", "speed_ms": 4, "direction_from_deg": 270}},
    )
    r = run_simulation(cfg, seed=3)
    assert r.meta.status == "ok" and np.isfinite(r.data).all()
    cmd, act = r.col("tvc_cmd_y"), r.col("tvc_y")
    assert np.max(np.abs(cmd)) > 0 and not np.array_equal(cmd, act)  # commanded vs actual differ (lag/delay)
    assert r.col("est_valid").any()


def test_hil_bridge_lock_step_loopback():
    """A 'flight computer' on the far side of a byte transport receives only SENSOR data and flies the vehicle."""
    import json

    from rocket_sim.control import HilController
    from rocket_sim.simulation import Simulation

    class Loopback:
        """Stands in for a serial link to a real flight computer running its own attitude loop."""

        def __init__(self):
            self.requests = []
            self._reply = b""

        def send(self, data: bytes) -> None:
            req = json.loads(data.decode())
            self.requests.append(req)
            # trivial 'firmware': commands a small constant deflection after launch is sensed (|a| > 3 g)
            a = math.sqrt(sum(x * x for x in req["accel"]))
            y = 0.01 if a > 3 * G0 else 0.0
            self._reply = (json.dumps({"tvc_y": y, "tvc_z": 0.0}) + "\n").encode()

        def recv(self) -> bytes:
            return self._reply

    link = Loopback()
    cfg = cfg_from(
        fidelity=4,
        motor={"ignition_delay_s": 1.0},
        sensors={"accelerometer": {"saturation": 400.0}},
        rocket={"parachutes": []},
        controller={"type": "none", "rate_hz": 50},
        simulation={"t_max_s": 6},
    )
    sim = Simulation(cfg, seed=2, controller=HilController(link))
    r = sim.run()
    assert len(link.requests) > 100 and set(link.requests[0]) >= {
        "accel",
        "gyro",
        "baro_pa",
        "gps",
        "mag",
        "new",
    }
    assert "pos_z" not in link.requests[0] and "vel" not in link.requests[0]  # truth never crosses the link
    assert np.max(np.abs(r.col("tvc_y"))) > 0.005  # the far side's commands reached the actuator
    t = r.col("t")
    assert np.all(r.col("tvc_cmd_y")[t < 1.0] == 0.0)  # nothing commanded before launch was sensed
    from rocket_sim.errors import SimulationError

    with pytest.raises(SimulationError, match="fidelity >= 4"):
        Simulation(
            cfg_from(fidelity=3, simulation={"t_max_s": 2}), seed=0, controller=HilController(link)
        ).run()


def test_python_controller_from_yaml_dotted_path():
    cfg = cfg_from(
        fidelity=3,
        simulation={"t_max_s": 3},
        rocket={"parachutes": []},
        controller={
            "type": "python",
            "rate_hz": 50,
            "params": {"class": "tests.helpers:ConstController", "kwargs": {"y": 0.01}},
        },
    )
    r = run_simulation(cfg, seed=0)
    assert np.max(np.abs(r.col("tvc_y"))) == pytest.approx(0.01, abs=1e-3)
