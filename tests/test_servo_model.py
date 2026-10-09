"""Servo model extensions (deadband, acceleration limit, backlash) and the bench-data fit."""

from __future__ import annotations

import math

import numpy as np
import pytest

from rocket_sim.config import apply_overrides, config_from_dict, read_mapping
from rocket_sim.control import actuators
from rocket_sim.control.actuators import ActuatorBank, TVCActuator
from rocket_sim.control.servo_fit import ServoParams, fit_servo, load_bench_csv, simulate_response
from rocket_sim.errors import ConfigError
from rocket_sim.resources import default_config


# --------------------------------------------------------------------------------------- oracles
def reference_play(shaft, output_prev, half_gap):
    """Test-local play operator (NOT the owner's TODO): the output sits inside the gap until the shaft pushes it."""
    if shaft - output_prev > half_gap:
        return shaft - half_gap
    if shaft - output_prev < -half_gap:
        return shaft + half_gap
    return output_prev


def legacy_step(state, eff, h, tau, rate, amax_angle):
    """The original delay-free chain: lag, rate limit, angle limit."""
    gain = 1.0 if tau <= 0 else 1.0 - math.exp(-h / tau)
    d = min(max((eff - state) * gain, -rate * h), rate * h)
    return min(max(state + d, -amax_angle), amax_angle)


def run_bank(bank, t_end, h, cmd_fn):
    t, out = 0.0, []
    while t < t_end - 1e-12:
        bank.command(t, [cmd_fn(t)])
        bank.step(t, h)
        t += h
        out.append(bank.state[0])
    return np.array(out)


# ----------------------------------------------------------------------------- defaults unchanged
def test_defaults_reproduce_the_original_chain_exactly():
    rng = np.random.default_rng(3)
    bank = ActuatorBank([0.1], [2.0], [0.05], [0.0])
    state = 0.0
    for k in range(400):
        c = float(rng.uniform(-0.2, 0.2))
        bank.command(k * 0.01, [c])
        bank.step(k * 0.01, 0.01)
        state = legacy_step(state, min(max(c, -0.1), 0.1), 0.01, 0.05, 2.0, 0.1)
        assert bank.state[0] == state  # bit-identical, not just close


def test_default_config_fields_are_off():
    cfg = config_from_dict(read_mapping(default_config()), base_dir=default_config().parent)
    a = cfg.tvc
    assert (a.deadband_deg, a.backlash_deg, a.max_accel_deg_s2) == (0.0, 0.0, None)


# -------------------------------------------------------------------------------------- deadband
def test_deadband_stops_the_servo_within_the_dead_zone_of_the_command():
    bank = ActuatorBank([1.0], [100.0], [0.05], [0.0], deadband=[0.02])
    out = run_bank(bank, 2.0, 0.005, lambda t: 0.30)
    assert out[-1] == pytest.approx(0.30 - 0.02, abs=1e-4)  # stops at the edge of the dead zone
    out = run_bank(bank, 2.0, 0.005, lambda t: 0.29)  # a change smaller than the dead zone does nothing
    assert out[-1] == pytest.approx(0.28, abs=1e-4)
    out = run_bank(bank, 2.0, 0.005, lambda t: 0.0)
    assert out[-1] == pytest.approx(0.02, abs=1e-4)  # approaches from above, again stopping at the edge


def test_deadband_exact_first_step():
    bank = ActuatorBank([1.0], [100.0], [0.1], [0.0], deadband=[0.05])
    bank.command(0.0, [0.25])
    bank.step(0.0, 0.1)
    gain = 1.0 - math.exp(-1.0)
    assert bank.state[0] == pytest.approx((0.25 - 0.05) * gain, rel=1e-12)


# --------------------------------------------------------------------------------- acceleration limit
def test_acceleration_limit_bounds_the_angular_acceleration():
    bank = ActuatorBank([10.0], [1000.0], [0.0], [0.0], max_accel=[50.0])  # ideal lag: only the limits act
    h, out = 0.001, []
    for k in range(300):
        bank.command(k * h, [5.0])
        bank.step(k * h, h)
        out.append(bank.state[0])
    out = np.array(out)
    vel = np.diff(np.concatenate([[0.0], out])) / h
    acc = np.diff(np.concatenate([[0.0], vel])) / h
    assert np.max(np.abs(acc)) <= 50.0 * (1 + 1e-9)
    # the velocity ramps by a*h per step, so after n steps the angle is a h^2 n (n + 1) / 2 (the discrete form of a t^2 / 2)
    n = 20
    assert out[n - 1] == pytest.approx(0.5 * 50.0 * h**2 * n * (n + 1), rel=1e-9)


def test_acceleration_limit_off_is_the_same_as_no_limit():
    a = ActuatorBank([1.0], [3.0], [0.04], [0.0])
    b = ActuatorBank([1.0], [3.0], [0.04], [0.0], max_accel=[None])
    cmd = lambda t: 0.8 * math.sin(7 * t)  # noqa: E731
    assert np.array_equal(run_bank(a, 1.0, 0.005, cmd), run_bank(b, 1.0, 0.005, cmd))


# --------------------------------------------------------------------------------------- backlash
def test_backlash_with_a_reference_play_operator(monkeypatch):
    monkeypatch.setattr(actuators, "play_operator", reference_play)
    bank = ActuatorBank([1.0], [1e3], [0.0], [0.0], backlash=[0.01])  # gap 0.01 rad -> half 0.005
    seq = []
    for c in (0.20, 0.20, 0.19, 0.18, 0.10, 0.10):
        bank.command(0.0, [c])
        bank.step(0.0, 0.1)
        seq.append(bank.state[0])
    # shaft follows the command exactly (ideal lag); the output trails by the half gap on each side
    assert seq[0] == pytest.approx(0.195)
    assert seq[1] == pytest.approx(0.195)
    assert seq[2] == pytest.approx(0.195)  # reversal of 0.01 stays inside the 0.01 gap
    assert seq[3] == pytest.approx(0.185)  # shaft 0.18 pushes the output from the other side
    assert seq[4] == pytest.approx(0.105) and seq[5] == pytest.approx(0.105)


def test_backlash_is_off_without_calling_the_todo():
    bank = ActuatorBank([1.0], [10.0], [0.01], [0.0], backlash=[0.0])
    bank.command(0.0, [0.5])
    bank.step(0.0, 0.1)  # would raise NotImplementedError if play_operator were called


def test_backlash_needs_the_todo_until_it_is_implemented():
    bank = ActuatorBank([1.0], [10.0], [0.01], [0.0], backlash=[0.01])
    bank.command(0.0, [0.5])
    with pytest.raises(NotImplementedError, match="play_operator"):
        bank.step(0.0, 0.1)


@pytest.mark.user_todo
@pytest.mark.xfail(
    strict=True, raises=NotImplementedError, reason="TODO(you): implement actuators.play_operator"
)
def test_todo_play_operator_hand_values():
    """Half gap b = 0.5 deg (docs/derivations/servo_model.md, section 5)."""
    p = actuators.play_operator
    assert p(2.0, 0.0, 0.5) == pytest.approx(1.5)
    assert p(1.8, 1.5, 0.5) == pytest.approx(1.5)  # inside the gap
    assert p(0.5, 1.5, 0.5) == pytest.approx(1.0)
    assert p(-3.0, 1.0, 0.5) == pytest.approx(-2.5)
    assert p(0.2, 0.0, 0.5) == pytest.approx(0.0)  # a small move inside the gap leaves the output alone


def test_reference_play_matches_the_derivation_table():
    assert [
        reference_play(*a) for a in ((2.0, 0.0, 0.5), (1.8, 1.5, 0.5), (0.5, 1.5, 0.5), (-3.0, 1.0, 0.5))
    ] == [1.5, 1.5, 1.0, -2.5]


# ----------------------------------------------------------------------------------------- config
def test_tvc_actuator_takes_the_new_parameters_from_the_config():
    raw = read_mapping(default_config())
    cfg = config_from_dict(
        apply_overrides(
            raw, {"tvc.deadband_deg": 0.2, "tvc.backlash_deg": 0.6, "tvc.max_accel_deg_s2": 5000.0}
        ),
        base_dir=default_config().parent,
    )
    a = TVCActuator(cfg.tvc)
    assert a.deadband == [math.radians(0.2)] * 2
    assert a.backlash == [math.radians(0.6)] * 2
    assert a.max_accel == [math.radians(5000.0)] * 2


@pytest.mark.parametrize(
    ("key", "bad"),
    [
        ("tvc.deadband_deg", -0.1),
        ("tvc.backlash_deg", float("nan")),
        ("tvc.max_accel_deg_s2", 0.0),
        ("tvc.max_accel_deg_s2", -5.0),
    ],
)
def test_invalid_servo_parameters_are_rejected(key, bad):
    with pytest.raises(ConfigError, match="tvc"):
        config_from_dict(
            apply_overrides(read_mapping(default_config()), {key: bad}), base_dir=default_config().parent
        )


# ------------------------------------------------------------------------------------------ fit
def independent_model(t, cmd, delay, tau, rate, deadband, dt=1e-4):
    """Servo response by explicit Euler at a 0.1 ms step with the delay as a pure time shift: a different integrator from
    the one the fit uses, so the recovery test is not circular."""
    n = int(round((t[-1] - t[0]) / dt)) + 1
    tt = t[0] + dt * np.arange(n)
    idx = np.clip(np.searchsorted(t, tt - delay, side="right") - 1, 0, len(t) - 1)
    c = np.where(tt - delay >= t[0], cmd[idx], 0.0)
    y = np.empty(n)
    x = 0.0
    for k in range(n):
        y[k] = x
        e = c[k] - x
        e = 0.0 if abs(e) <= deadband else e - math.copysign(deadband, e)
        x += float(np.clip(e / tau, -rate, rate)) * dt
    return np.interp(t, tt, y)


def step_sequence(
    fs=500.0, amplitudes=((0.2, 10), (0.7, -10), (1.2, 3), (1.7, -3), (2.2, 20), (2.7, 0)), duration=3.2
):
    t = np.arange(0.0, duration, 1.0 / fs)
    cmd = np.zeros_like(t)
    for t0, a in amplitudes:
        cmd[t >= t0] = a
    return t, cmd


def test_fit_recovers_known_parameters_from_independent_noisy_data():
    t, cmd = step_sequence()
    truth = dict(delay=0.020, tau=0.040, rate=300.0, deadband=0.30)
    meas = independent_model(t, cmd, **truth) + np.random.default_rng(11).normal(0.0, 0.05, len(t))
    fit = fit_servo(t, cmd, meas)
    p = fit.params
    assert fit.converged and fit.starts_agree
    assert p.delay == pytest.approx(truth["delay"], abs=0.003)
    assert p.tau == pytest.approx(truth["tau"], rel=0.08)
    assert p.rate == pytest.approx(truth["rate"], rel=0.08)
    assert p.deadband == pytest.approx(truth["deadband"], abs=0.1)
    assert fit.rms_residual == pytest.approx(0.05, rel=0.3)
    assert all(fit.identifiable[n] for n in fit.free)


def test_fit_standard_errors_are_consistent_with_the_actual_error():
    """Over several noise draws the scatter of the estimates must be close to the reported standard errors."""
    t, cmd = step_sequence()
    clean = simulate_response(ServoParams(delay=0.02, tau=0.04, rate=300.0, deadband=0.3), t, cmd)
    taus, ses = [], []
    for seed in range(8):
        meas = clean + np.random.default_rng(seed).normal(0.0, 0.1, len(t))
        f = fit_servo(t, cmd, meas)
        taus.append(f.params.tau)
        ses.append(f.stderr["tau"])
    assert 0.3 < np.std(taus, ddof=1) / np.mean(ses) < 3.0
    assert np.mean(taus) == pytest.approx(0.04, abs=3 * np.mean(ses) / math.sqrt(8) + 1e-4)


def test_rate_limit_is_reported_not_identifiable_when_never_reached():
    t, cmd = step_sequence(amplitudes=((0.2, 2), (0.7, -2), (1.2, 1), (1.7, -1), (2.2, 2), (2.7, 0)))
    meas = simulate_response(ServoParams(delay=0.02, tau=0.05, rate=1000.0, deadband=0.0), t, cmd)
    meas = meas + np.random.default_rng(5).normal(0.0, 0.01, len(t))
    fit = fit_servo(t, cmd, meas)
    assert fit.identifiable["rate"] is False
    assert "no rate limit is visible" in " ".join(fit.notes)
    assert fit.params.rate == float("inf")  # reported as no limit, not as an arbitrary large number
    assert "not identifiable" in fit.report()
    assert fit.params.tau == pytest.approx(0.05, rel=0.1)


def test_fit_backlash_with_a_reference_play_operator(monkeypatch):
    monkeypatch.setattr(actuators, "play_operator", reference_play)
    t, cmd = step_sequence(amplitudes=((0.2, 10), (0.7, -10), (1.2, 4), (1.7, -4), (2.2, 8), (2.7, 0)))
    truth = ServoParams(delay=0.015, tau=0.03, rate=400.0, deadband=0.0, backlash=1.0)
    meas = simulate_response(truth, t, cmd) + np.random.default_rng(2).normal(0.0, 0.03, len(t))
    fit = fit_servo(t, cmd, meas, fit_backlash=True, fit_deadband=False)
    assert fit.params.backlash == pytest.approx(1.0, abs=0.15)
    assert fit.params.tau == pytest.approx(0.03, rel=0.1)
    assert fit.params.delay == pytest.approx(0.015, abs=0.004)


def test_fit_with_backlash_requested_needs_the_todo():
    t, cmd = step_sequence()
    with pytest.raises(NotImplementedError, match="play_operator"):
        fit_servo(
            t, cmd, np.zeros_like(t) + 0.01 * np.random.default_rng(1).normal(size=len(t)), fit_backlash=True
        )


def test_fit_rejects_bad_input():
    t, cmd = step_sequence()
    with pytest.raises(ConfigError, match="equal length"):
        fit_servo(t, cmd[:-1], cmd)
    with pytest.raises(ConfigError, match="strictly increasing"):
        fit_servo(np.r_[t[:10], t[9], t[10:-1]], cmd, cmd)
    with pytest.raises(ConfigError, match="at least 20"):
        fit_servo(t[:5], cmd[:5], cmd[:5])
    bad = cmd.copy()
    bad[3] = np.nan
    with pytest.raises(ConfigError, match="finite"):
        fit_servo(t, cmd, bad)


def test_load_bench_csv(tmp_path):
    t, cmd = step_sequence(fs=100.0)
    meas = simulate_response(ServoParams(tau=0.03), t, cmd)
    p = tmp_path / "bench.csv"
    p.write_text(
        "t_ms,cmd_deg,meas_deg\n"
        + "\n".join(f"{ti * 1000:.3f},{c},{m:.5f}" for ti, c, m in zip(t, cmd, meas, strict=True))
        + "\n",
        encoding="utf-8",
    )
    t2, c2, m2 = load_bench_csv(p, "t_ms", "cmd_deg", "meas_deg", time_scale=1e-3)
    assert np.allclose(t2, t, atol=1e-6) and np.allclose(c2, cmd) and np.allclose(m2, meas, atol=1e-5)
    with pytest.raises(ConfigError, match="not in header"):
        load_bench_csv(p, "time", "cmd_deg", "meas_deg")
    q = tmp_path / "short.csv"
    q.write_text("t,c,m\n1,2,3\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="fewer than 20"):
        load_bench_csv(q, "t", "c", "m")


def test_simulate_response_matches_the_bank_for_whole_step_delays():
    """The fit's simulation and ActuatorBank agree when the delay is a whole number of steps (queue delay vs shifted commands)."""
    t, cmd = step_sequence(fs=500.0)
    p = ServoParams(delay=0.02, tau=0.04, rate=300.0, deadband=0.0)
    fit_out = simulate_response(p, t, cmd, max_substep=0.002)
    bank = ActuatorBank([1e12], [300.0], [0.04], [0.02])
    out = []
    for k in range(len(t)):
        out.append(bank.state[0])
        bank.command(float(t[k]), [float(cmd[k])])
        bank.step(float(t[k]), 0.002)
    assert np.max(np.abs(fit_out - np.array(out))) < 1e-6


def test_cli_fit_servo(tmp_path, capsys):
    from rocket_sim.cli import main

    t, cmd = step_sequence()
    meas = simulate_response(ServoParams(delay=0.02, tau=0.04, rate=300.0, deadband=0.3), t, cmd)
    meas = meas + np.random.default_rng(4).normal(0.0, 0.05, len(t))
    p = tmp_path / "bench.csv"
    p.write_text(
        "t_ms,cmd,meas\n"
        + "\n".join(f"{ti * 1000:.2f},{c},{m:.5f}" for ti, c, m in zip(t, cmd, meas, strict=True)),
        encoding="utf-8",
    )
    assert (
        main(
            [
                "fit-servo",
                str(p),
                "--time-col",
                "t_ms",
                "--cmd-col",
                "cmd",
                "--meas-col",
                "2",
                "--time-scale",
                "0.001",
            ]
        )
        == 0
    )
    out = capsys.readouterr().out
    assert "servo fit: 1600 samples" in out and "delay" in out and "(1 sigma)" in out
    assert main(["fit-servo", str(p), "--time-col", "nope", "--cmd-col", "cmd", "--meas-col", "meas"]) == 1
