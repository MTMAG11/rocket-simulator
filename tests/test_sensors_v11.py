"""V1.1 sensors and estimation: dropout, startup delay, GPS velocity noise, misalignment, async rates,
latency, truth estimator, gyro-bias estimation, TRIAD with declination."""

import math

import numpy as np
import pytest

from rocket_sim.config.schema import EstimatorCfg, SensorCfg, SensorsCfg
from rocket_sim.environment.gravity import ConstantGravity
from rocket_sim.estimation import (
    NavigationFilter,
    TruthEstimator,
    TruthState,
    build_estimator,
    triad_attitude,
)
from rocket_sim.estimation.estimators import EstimatedState
from rocket_sim.physics.math3d import quat_from_pointing, quat_rotate, quat_rotate_inv
from rocket_sim.sensors import SensorChannel, SensorSuite
from rocket_sim.simulation import run_simulation
from tests.conftest import cfg_from


def ch(dim=1, seed=1, **kw):
    kw.setdefault("rate_hz", 100.0)
    return SensorChannel(SensorCfg(enabled=True, **kw), dim, np.random.default_rng(seed))


def drive(c, t_end, truth=None, h=0.001):
    t, vis = 0.0, []
    truth = np.ones(c.dim) if truth is None else truth
    while t < t_end - 1e-12:
        if c.due(t):
            c.sample(t, truth)
        if c.poll(t):
            vis.append((t, c.value.copy()))
        t += h
    return vis


def test_dropout_probability_matches_configuration():
    c = ch(rate_hz=1000.0, dropout_probability=0.3)
    vis = drive(c, 20.0)
    total = len(vis) + c.dropped
    assert total == pytest.approx(20000, abs=2)
    assert c.dropped / total == pytest.approx(0.3, abs=0.02)  # binomial sigma ~0.003


def test_zero_dropout_loses_nothing_and_full_dropout_loses_everything():
    assert (
        ch(rate_hz=200.0).dropped == 0
        and drive(ch(rate_hz=200.0), 1.0)
        and len(drive(ch(rate_hz=200.0), 1.0)) == 200
    )
    c = ch(dropout_probability=1.0)
    assert drive(c, 1.0) == [] and c.dropped == 100 and not c.has_value


def test_startup_delay_suppresses_early_samples():
    c = ch(startup_delay_s=0.5)
    vis = drive(c, 1.0)
    assert vis[0][0] >= 0.5 - 1e-9 and c.dropped == 50 and len(vis) == 50


def test_latency_and_rate_are_independent_across_channels():
    cfg = SensorsCfg()
    suite = SensorSuite(cfg, [np.random.SeedSequence(i) for i in range(5)])
    t, h = 0.0, 0.0005
    counts = {"a": 0, "g": 0, "b": 0, "gps": 0, "m": 0}
    while t < 5.0:
        r = suite.update(t, (0, 0, 9.81), (0, 0, 0), 101325.0, (0, 0, 0), (0, 0, 0), (1, 0, 0, 0))
        counts["a"] += r.accel_new
        counts["g"] += r.gyro_new
        counts["b"] += r.baro_new
        counts["gps"] += r.gps_new
        counts["m"] += r.mag_new
        t += h
    # asynchronous: each channel delivers at its own configured rate (within one sample)
    for key, c in zip(counts, (cfg.accelerometer, cfg.gyroscope, cfg.barometer, cfg.gps, cfg.magnetometer)):
        assert abs(counts[key] - 5.0 * c.rate_hz) <= 2, key
    rates = {c.rate_hz for c in (cfg.accelerometer, cfg.barometer, cfg.gps, cfg.magnetometer)}
    assert len(rates) > 1  # defaults really are mixed-rate


def test_gps_velocity_noise_is_independent_of_position_noise():
    cfg = SensorsCfg()
    cfg.gps.noise_std = 2.0
    cfg.gps.velocity_noise_std = 0.1
    cfg.gps.bias_std = 0.0
    cfg.gps.rate_hz = 1000.0
    cfg.gps.latency_s = 0.0
    cfg.gps.quantization = 0.0
    suite = SensorSuite(cfg, [np.random.SeedSequence(i) for i in range(5)])
    c = suite.gps
    vis = drive(c, 20.0, truth=np.zeros(6))
    x = np.array([v for _, v in vis])
    assert x[:, :3].std() == pytest.approx(2.0, rel=0.03)
    assert x[:, 3:].std() == pytest.approx(0.1, rel=0.03)


def test_gps_default_velocity_noise_follows_ratio():
    cfg = SensorsCfg()
    cfg.gps.noise_std = 3.0
    cfg.gps.velocity_noise_std = None
    cfg.gps.bias_std = 0.0
    cfg.gps.rate_hz = 1000.0
    cfg.gps.latency_s = 0.0
    cfg.gps.quantization = 0.0
    suite = SensorSuite(cfg, [np.random.SeedSequence(i) for i in range(5)])
    x = np.array([v for _, v in drive(suite.gps, 20.0, truth=np.zeros(6))])
    assert x[:, 3:].std() == pytest.approx(0.05 * 3.0, rel=0.04)


def test_gps_fix_flag():
    cfg = SensorsCfg()
    cfg.gps.startup_delay_s = 0.5
    suite = SensorSuite(cfg, [np.random.SeedSequence(i) for i in range(5)])
    args = ((0, 0, 9.81), (0, 0, 0), 101325.0, (0, 0, 0), (0, 0, 0), (1, 0, 0, 0))
    assert not suite.update(0.1, *args).gps_has_fix
    t = 0.1
    while t < 2.0:
        r = suite.update(t, *args)
        t += 0.01
    assert r.gps_has_fix


def test_misalignment_rotates_the_measured_vector_without_changing_its_norm():
    c = ch(3, misalignment_std_deg=2.0, rate_hz=100.0)
    assert c.misalign is not None
    assert np.allclose(c.misalign @ c.misalign.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(c.misalign) == pytest.approx(1.0)
    ang = math.degrees(math.acos(np.clip((np.trace(c.misalign) - 1) / 2, -1, 1)))
    assert 0.0 < ang < 10.0
    v = np.array([0.0, 0.0, 9.81])
    c.sample(0.0, v)
    c.poll(0.0)
    assert np.linalg.norm(c.value) == pytest.approx(9.81, rel=1e-9)
    assert not np.allclose(c.value, v)


def test_misalignment_statistics():
    angs = []
    for s in range(400):
        c = ch(3, seed=s, misalignment_std_deg=1.0)
        angs.append(math.acos(np.clip((np.trace(c.misalign) - 1) / 2, -1, 1)))
    # rotation-vector magnitude of 3 iid N(0, sigma): Maxwell, mean = 2 sqrt(2/pi) sigma
    assert np.mean(angs) == pytest.approx(2 * math.sqrt(2 / math.pi) * math.radians(1.0), rel=0.08)


def test_sensor_seed_reproducibility_with_dropout():
    a = drive(ch(seed=4, dropout_probability=0.2, noise_std=0.1), 1.0)
    b = drive(ch(seed=4, dropout_probability=0.2, noise_std=0.1), 1.0)
    assert [t for t, _ in a] == [t for t, _ in b] and np.allclose([v for _, v in a], [v for _, v in b])


def test_truth_estimator_passes_state_and_requires_truth():
    est = build_estimator(EstimatorCfg(type="truth"), SensorsCfg(), ConstantGravity(), 0.0)
    assert isinstance(est, TruthEstimator)
    assert est.update(0.0, None).valid is False  # no truth, no estimate
    ts = TruthState((1, 2, 3), (4, 5, 6), (1, 0, 0, 0), (0.1, 0.2, 0.3), 0.5)
    e = est.update(1.0, None, ts)
    assert (
        e.valid
        and e.position == (1, 2, 3)
        and e.velocity == (4, 5, 6)
        and e.launch_time == 0.5
        and e.launch_detected
    )


def test_truth_estimator_in_simulation_has_zero_error():
    r = run_simulation(
        cfg_from(
            fidelity=3,
            estimator={"type": "truth"},
            rocket={"parachutes": []},
            environment={"wind": {"model": "none"}},
            simulation={"t_max_s": 4.0},
        ),
        seed=0,
    )
    assert r.has("est_pos_z")
    assert np.allclose(r.col("est_pos_z")[1:], r.col("pos_z")[1:], atol=1e-9)
    assert np.allclose(r.col("est_vel_z")[1:], r.col("vel_z")[1:], atol=1e-9)


def test_triad_with_declination_recovers_attitude():
    # field with a +east component (declination ~ 15 deg): the simple construction would be biased
    b_l = np.array([0.25, 0.9, -0.4]) * 5e-5
    q = quat_from_pointing(1.2, 0.7, 0.3)
    up_b = np.array(quat_rotate_inv(q, (0.0, 0.0, 1.0))) * 9.81
    mag_b = np.array(quat_rotate_inv(q, tuple(b_l)))
    est = triad_attitude(up_b, mag_b, b_l)
    v = (0.3, -0.5, 0.8)
    assert np.allclose(quat_rotate(est, v), quat_rotate(q, v), atol=1e-9)
    naive = triad_attitude(up_b, mag_b)  # ignoring the east component
    assert not np.allclose(quat_rotate(naive, v), quat_rotate(q, v), atol=1e-3)


def _pad_filter(gyro_bias, estimate=True, seed=2):
    s = SensorsCfg()
    s.gyroscope.bias_std = 0.0
    s.gyroscope.noise_std = 0.002
    s.gyroscope.rate_hz = 200.0
    ecfg = EstimatorCfg(type="nav_kf", alignment_time_s=1.0, estimate_gyro_bias=estimate)
    nf = NavigationFilter(ecfg, s, ConstantGravity(), 0.0)
    suite = SensorSuite(s, [np.random.SeedSequence(seed + i) for i in range(5)])
    suite.gyro.bias = np.asarray(gyro_bias, float)
    return nf, suite, s


def _run_pad(nf, suite, t_end=3.0, h=0.005):
    t = 0.0
    q = (1.0, 0.0, 0.0, 0.0)
    while t < t_end:
        r = suite.update(
            t, tuple(quat_rotate_inv(q, (0, 0, 9.81))), (0, 0, 0), 101325.0, (0, 0, 0), (0, 0, 0), q
        )
        e = nf.update(t, r)
        t += h
    return e


def test_gyro_bias_estimated_on_pad_and_removed():
    bias = [0.01, -0.02, 0.015]
    nf, suite, _ = _pad_filter(bias)
    e = _run_pad(nf, suite)
    assert e.valid
    assert np.allclose(nf.gyro_bias, bias, atol=2e-3)  # noise 0.002 rad/s averaged over ~200 samples
    assert (
        np.linalg.norm(e.omega) < 0.006
    )  # bias-corrected rate is near zero on the pad (raw would be ~0.027)


def test_gyro_bias_not_removed_when_disabled():
    bias = [0.01, -0.02, 0.015]
    nf, suite, _ = _pad_filter(bias, estimate=False)
    e = _run_pad(nf, suite)
    assert np.linalg.norm(e.omega) > 0.02  # raw biased rate is exposed to the controller


def test_estimator_state_dataclass_defaults_invalid():
    assert EstimatedState().valid is False


def test_simulation_with_dropout_and_startup_delay_still_runs_and_estimates():
    r = run_simulation(
        cfg_from(
            fidelity=5,
            estimator={"type": "nav_kf", "alignment_time_s": 1.0},
            controller={"type": "none"},
            motor={"ignition_delay_s": 2.0},
            rocket={"parachutes": []},
            environment={"wind": {"model": "none"}},
            sensors={
                "accelerometer": {"saturation": 400.0},
                "gps": {"dropout_probability": 0.3, "startup_delay_s": 1.0, "velocity_noise_std": 0.2},
            },
        ),
        seed=3,
    )
    t = r.col("t")
    assert r.col("est_valid")[-1] == 1.0
    fl = (t > 3.5) & (t < r.summary["apogee_time_s"])
    assert np.sqrt(np.mean((r.col("est_pos_z")[fl] - r.col("pos_z")[fl]) ** 2)) < 5.0
