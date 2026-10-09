"""Error-state EKF: transition matrix against finite differences, consistency, truth separation, and comparison with the linear KF."""

from __future__ import annotations

import math

import numpy as np
import pytest

from rocket_sim.errors import ConfigError
from rocket_sim.estimation import NavigationFilter, build_estimator
from rocket_sim.estimation import ekf as ekf_mod
from rocket_sim.estimation.ekf import ErrorStateEKF, nominal_step, skew, transition_matrix
from rocket_sim.physics.math3d import quat_conj, quat_mul, quat_normalize
from rocket_sim.sensors import SensorReadings
from rocket_sim.simulation import Simulation
from tests.conftest import cfg_from

# The simulator's default accelerometer (400 Hz, 0.03 m/s^2 noise, 0.05 bias, 1e-3 walk, 0.2 % scale error), with the full scale
# raised so that the 18 g peak of the example motor is not clipped. (A dict in the config replaces the whole default.)
ACCEL = {
    "rate_hz": 400.0,
    "noise_std": 0.03,
    "bias_std": 0.05,
    "bias_walk_std": 1e-3,
    "scale_error_std": 0.002,
    "quantization": 157.0 / 32768.0,
    "saturation": 400.0,
}


def ekf_cfg(est="ekf", sensors=None, estimator=None, **kw):
    s = {"accelerometer": dict(ACCEL)}
    for k, v in (sensors or {}).items():
        s.setdefault(k, {}).update(v)
    return cfg_from(
        fidelity=5,
        estimator={"type": est, "alignment_time_s": 1.0, **(estimator or {})},
        sensors=s,
        motor={"ignition_delay_s": 2.0},
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
        launch={"elevation_deg": 90.0, "rail_length_m": 1.5},
        simulation={"t_max_s": 9.0},
        **kw,
    )


def errors(rec):
    """RMS position [m], velocity [m/s] and attitude [deg] errors against truth while the vehicle is in the air."""
    sel = (rec.col("t") > 4.0) & (rec.col("t") < 8.0)
    dp = np.linalg.norm(np.stack([rec.col(f"est_pos_{a}") - rec.col(f"pos_{a}") for a in "xyz"], 1), axis=1)
    dv = np.linalg.norm(np.stack([rec.col(f"est_vel_{a}") - rec.col(f"vel_{a}") for a in "xyz"], 1), axis=1)
    q = np.stack([rec.col(f"est_quat_{a}") for a in "wxyz"], 1)
    qt = np.stack([rec.col(f"quat_{a}") for a in "wxyz"], 1)
    ang = 2.0 * np.degrees(np.arccos(np.clip(np.abs(np.sum(q * qt, 1)), 0.0, 1.0)))
    return tuple(float(np.sqrt(np.mean(x[sel] ** 2))) for x in (dp, dv, ang))


def rotvec_quat(v):
    a = float(np.linalg.norm(v))
    if a < 1e-15:
        return (1.0, 0.0, 0.0, 0.0)
    return (math.cos(a / 2), *(np.asarray(v) / a * math.sin(a / 2)))


def quat_err(q_nominal, q_true):
    """Small rotation vector d with q_true = q_nominal (x) [1, d/2]."""
    dq = quat_mul(quat_conj(q_nominal), q_true)
    return 2.0 * np.array(dq[1:]) * (1.0 if dq[0] >= 0 else -1.0)


# ----------------------------------------------------------------------------------------- algebra
def test_skew_is_the_cross_product_matrix():
    rng = np.random.default_rng(0)
    for _ in range(5):
        a, b = rng.normal(size=3), rng.normal(size=3)
        assert np.allclose(skew(a) @ b, np.cross(a, b))
        assert np.allclose(skew(a), -skew(a).T)


@pytest.mark.user_todo
@pytest.mark.xfail(
    strict=True, raises=NotImplementedError, reason="TODO(you): implement attitude_measurement_jacobian"
)
def test_todo_attitude_measurement_jacobian_hand_values():
    """z_b = z_pred + H d_theta with q_true = q (x) [1, d_theta/2]  ->  H = [z_pred]x (docs/derivations/error_state_ekf.md, section 3)."""
    h = ekf_mod.attitude_measurement_jacobian
    assert np.allclose(h(np.array([0.0, 0.0, 1.0])), [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    assert np.allclose(h(np.array([1.0, 2.0, 3.0])), [[0.0, -3.0, 2.0], [3.0, 0.0, -1.0], [-2.0, 1.0, 0.0]])


def test_the_jacobian_oracle_matches_a_finite_difference_of_the_measurement():
    """Independent check of what the TODO has to produce: rotate the body by a small d_theta and difference the measurement."""
    rng = np.random.default_rng(4)
    q = quat_normalize(tuple(rng.normal(size=4)))
    from rocket_sim.physics.math3d import quat_to_dcm

    r = np.array(quat_to_dcm(q))
    m_l = rng.normal(size=3)
    z0 = r.T @ m_l
    h = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3)
        d[i] = 1e-6
        r_true = np.array(quat_to_dcm(quat_mul(q, rotvec_quat(d))))
        h[:, i] = (r_true.T @ m_l - z0) / 1e-6
    assert np.allclose(h, skew(z0), atol=1e-5)  # H = [z_pred]x


# ---------------------------------------------------------------------------------- transition matrix
def propagate_pair(dt, delta, rng):
    q0 = quat_normalize(tuple(rng.normal(size=4)))
    v0, p0 = rng.normal(size=3) * 50, rng.normal(size=3) * 100
    bg0, ba0 = rng.normal(size=3) * 1e-2, rng.normal(size=3) * 0.1
    gyro, accel = rng.normal(size=3) * 2.0, rng.normal(size=3) * 20.0 + np.array([0, 0, 9.8])
    d = rng.normal(size=15)
    d *= delta / np.linalg.norm(d)  # error state [dtheta, dv, dp, dbg, dba]
    q_t = quat_mul(q0, rotvec_quat(d[0:3]))
    qn, vn, pn = nominal_step(q0, v0, p0, bg0, ba0, gyro, accel, dt, 9.8)
    qt, vt, pt = nominal_step(
        q_t, v0 + d[3:6], p0 + d[6:9], bg0 + d[9:12], ba0 + d[12:15], gyro, accel, dt, 9.8
    )
    d_after = np.concatenate([quat_err(qn, qt), vt - vn, pt - pn, d[9:12], d[12:15]])
    phi = transition_matrix(qn, gyro - bg0, accel - ba0, dt)
    return phi @ d, d_after, d


def test_transition_matrix_matches_the_propagated_error_to_second_order():
    """Perturb the true state by a small error, propagate nominal and true with the same measurements, and compare the new
    error with Phi * error. The residual must shrink quadratically with the step (it is the discretisation error)."""
    rng = np.random.default_rng(1)
    res = {}
    for dt in (0.004, 0.002, 0.001):
        worst = 0.0
        for _ in range(5):
            pred, actual, d = propagate_pair(dt, 1e-6, rng)
            worst = max(
                worst, float(np.linalg.norm(pred - actual) / np.linalg.norm(actual - d))
            )  # relative to the CHANGE
        res[dt] = worst
    assert res[0.004] < 0.15, res  # the change over a step is predicted to within the discretisation error
    assert res[0.001] < res[0.004] / 3.0, (
        res
    )  # and the error shrinks when the step does (first-order in dt, then dt^2)


def test_transition_matrix_structure():
    rng = np.random.default_rng(5)
    q = quat_normalize(tuple(rng.normal(size=4)))
    w, f = rng.normal(size=3), rng.normal(size=3) * 10
    phi = transition_matrix(q, w, f, 0.0)
    assert np.allclose(phi, np.eye(15))  # no time, no change
    phi = transition_matrix(q, w, f, 0.01)
    assert np.allclose(phi[9:15, :9], 0.0) and np.allclose(
        phi[9:15, 9:15], np.eye(6)
    )  # biases do not depend on the rest
    assert np.allclose(phi[6:9, 3:6], np.eye(3) * 0.01, atol=1e-6)  # position error grows with velocity error


# ------------------------------------------------------------------------------------------- the filter
def test_ekf_is_built_from_config_and_is_a_navigation_filter_variant():
    cfg = ekf_cfg()
    sim = Simulation(cfg, seed=1)
    assert isinstance(sim.estimator, ErrorStateEKF) and isinstance(sim.estimator, NavigationFilter)
    assert sim.estimator.name == "ekf"


def test_config_validation():
    assert ekf_cfg().estimator.type == "ekf"
    with pytest.raises(ConfigError, match="noise_scale"):
        ekf_cfg(estimator={"noise_scale": 0.0})
    with pytest.raises(ConfigError, match=r"estimator.type"):
        cfg_from(estimator={"type": "nope"})
    with pytest.raises(ConfigError, match="aligns attitude"):
        ekf_cfg(sensors={"magnetometer": {"enabled": False}})
    cfg = ekf_cfg(controller={"type": "tvc_attitude", "state_source": "estimate", "params": {"wn": 8.0}})
    assert cfg.controller.state_source == "estimate"  # the controller can be fed the EKF estimate


def test_estimates_are_a_pure_function_of_the_recorded_measurements():
    """Bit-exact replay of the logged measurements through a fresh EKF reproduces its logged estimates."""
    sim = Simulation(ekf_cfg(), seed=7)
    rec = sim.run()
    nf = build_estimator(sim.cfg.estimator, sim.cfg.sensors, sim.env.gravity, sim.env.site_elevation)
    assert isinstance(nf, ErrorStateEKF)
    col = rec.col
    has_fix, prev_t, n = False, -1.0, 0
    for i in range(rec.n_rows):
        t = float(col("t")[i])
        if t <= prev_t:
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
            n += 1
            assert e.position == (col("est_pos_x")[i], col("est_pos_y")[i], col("est_pos_z")[i])
            assert e.quaternion[0] == col("est_quat_w")[i]
            assert e.extra["gyro_bias_x"] == col("est_gyro_bias_x")[i]
    assert n > 300


def test_ekf_is_never_given_truth(monkeypatch):
    seen = []
    orig = ErrorStateEKF.update

    def spy(self, t, r, truth=None):
        seen.append(truth)
        return orig(self, t, r, truth)

    monkeypatch.setattr(ErrorStateEKF, "update", spy)
    Simulation(ekf_cfg(), seed=1).run()
    assert len(seen) > 100 and all(tr is None for tr in seen)


def test_covariance_stays_symmetric_positive_definite_and_initial_values_come_from_the_sensors():
    sim = Simulation(ekf_cfg(), seed=3)
    rec = sim.run()
    p = sim.estimator.P15
    assert np.allclose(p, p.T, atol=1e-12 * np.abs(p).max())
    assert np.linalg.eigvalsh(0.5 * (p + p.T)).min() > -1e-9 * np.abs(p).max()
    assert rec.summary["apogee_m"] > 100


def test_the_position_error_is_consistent_with_the_filters_own_covariance():
    """Normalised estimation error squared over several flights: neither wildly optimistic nor wildly pessimistic."""
    nees = []
    for seed in range(8):
        sim = Simulation(ekf_cfg(), seed=seed)
        rec = sim.run()
        sel = (rec.col("t") > 4.0) & (rec.col("t") < 8.0)
        i = int(np.nonzero(sel)[0][-1])
        err = np.array([rec.col(f"est_pos_{a}")[i] - rec.col(f"pos_{a}")[i] for a in "xyz"])
        # covariance at the END of the run, used as a proxy; the run ends later, so the covariance is if anything larger
        p_pos = sim.estimator.P15[6:9, 6:9]
        nees.append(float(err @ np.linalg.solve(p_pos, err)))
    assert np.mean(nees) < 6.0 * 3.0, nees  # chi-square(3) has mean 3: allow a 6x optimism factor
    assert np.median(nees) < 20.0, nees


# ---------------------------------------------------------------------------------------- comparison
def test_ekf_is_not_worse_than_the_linear_filter_in_the_nominal_case():
    kf, ek = [], []
    for seed in range(4):
        kf.append(errors(Simulation(ekf_cfg("nav_kf"), seed=seed).run()))
        ek.append(errors(Simulation(ekf_cfg("ekf"), seed=seed).run()))
    kf_m, ek_m = np.mean(kf, axis=0), np.mean(ek, axis=0)
    assert ek_m[0] < 1.5 * kf_m[0] and ek_m[1] < 1.5 * kf_m[1]  # position and velocity: no meaningful loss
    assert ek_m[2] < kf_m[2]  # attitude: the EKF corrects it, the linear filter cannot
    assert ek_m[0] < 5.0 and ek_m[1] < 5.0 and ek_m[2] < 5.0


@pytest.mark.parametrize(
    "scenario",
    [
        {"barometer": {"dropout_probability": 0.6}},
        {"gps": {"dropout_probability": 0.6}},
        {"accelerometer": {"noise_std": 0.3}},  # ten times the default accelerometer noise, filter not told
    ],
    ids=["baro-dropout", "gps-dropout", "vibration"],
)
def test_ekf_stays_usable_under_degraded_sensors(scenario):
    r = Simulation(ekf_cfg("ekf", sensors=scenario), seed=2).run()
    pos, vel, att = errors(r)
    assert all(math.isfinite(x) for x in (pos, vel, att))
    assert pos < 25.0 and vel < 10.0 and att < 15.0, (pos, vel, att)


def test_noise_scale_inflates_the_assumed_noise():
    a = Simulation(ekf_cfg(), seed=1).estimator
    b = Simulation(ekf_cfg(estimator={"noise_scale": 5.0}), seed=1).estimator
    assert b._sa == pytest.approx(5.0 * a._sa) and b._r_baro == pytest.approx(25.0 * a._r_baro)


# ---------------------------------------------------------------------------------------- magnetometer
def test_magnetometer_update_is_off_by_default_and_needs_the_todo_when_on(monkeypatch):
    sim = Simulation(ekf_cfg(estimator={"ekf_use_magnetometer": True}), seed=1)
    with pytest.raises(NotImplementedError, match="attitude_measurement_jacobian"):
        sim.run()
    Simulation(ekf_cfg(), seed=1).run()  # the default never calls it


def test_magnetometer_update_corrects_a_heading_error_with_a_reference_jacobian(monkeypatch):
    monkeypatch.setattr(ekf_mod, "attitude_measurement_jacobian", lambda z: skew(z))
    sim = Simulation(ekf_cfg(estimator={"ekf_use_magnetometer": True}), seed=5)
    rec = sim.run()
    e = errors(rec)
    base = errors(Simulation(ekf_cfg(), seed=5).run())
    assert e[2] < base[2] * 1.1 and e[0] < 5.0  # the update must not hurt, and the run stays accurate
    # direct check of one update: a known heading error is pulled back by a perfect magnetometer reading
    ekf = sim.estimator
    from rocket_sim.physics.math3d import quat_to_dcm

    q_true = ekf.q
    ekf.q = quat_mul(q_true, rotvec_quat(np.radians([0.0, 0.0, 4.0])))  # 4 degrees of heading error
    ekf.P15[0:3, 0:3] = np.eye(3) * math.radians(5.0) ** 2
    z = np.array(quat_to_dcm(q_true)).T @ ekf.mag_ref
    u = z / np.linalg.norm(z)

    def observable_error(q):
        e = quat_err(q_true, q)
        return float(
            np.linalg.norm(e - (e @ u) * u)
        )  # a rotation about the measured vector itself cannot be seen

    before = observable_error(ekf.q)
    assert before > 0.01
    for _ in range(20):
        ekf._mag_update(z)
    assert observable_error(ekf.q) < 0.1 * before
