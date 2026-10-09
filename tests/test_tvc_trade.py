"""Q-01 trade-study machinery, exercised on the illustrative example vehicle with TEST-ONLY numbers (not your vehicle)."""

from __future__ import annotations

import math
import subprocess
import sys

import numpy as np
import pytest
import yaml

from rocket_sim.errors import ConfigError
from rocket_sim.studies import tvc_trade
from rocket_sim.studies.report import comparison_sentence, write_report
from rocket_sim.studies.tvc_trade import (
    MotorResult,
    base_dict,
    configure,
    disturb,
    evaluate_motor,
    load_study,
    run_cfg,
    success,
    wilson,
)
from tests.conftest import ROOT

TEMPLATE = ROOT / "studies" / "q01_template.yaml"
EXAMPLE = ROOT / "configs" / "example_tvc_closed_loop.yaml"


def oracle_torque(thrust, lever, delta):
    """Test-local T sin(delta) l (NOT the owner's TODO)."""
    return np.asarray(thrust) * math.sin(delta) * np.asarray(lever)


@pytest.fixture
def with_oracle(monkeypatch):
    monkeypatch.setattr(tvc_trade, "max_correctable_torque", oracle_torque)


def make_study(tmp_path, **changes):
    """A study file for the example vehicle. TEST-ONLY numbers."""
    d = {
        "base_config": str(EXAMPLE),
        "motors": [str(ROOT / "data/motors/AeroTech_G80T.eng"), str(ROOT / "data/motors/Estes_E16.eng")],
        "added_masses": [{"name": "test_gimbal", "mass_kg": 0.05, "position_from_nose_m": 0.5}],
        "tvc": {"max_angle_deg": 6.0, "max_rate_deg_s": 120.0, "time_constant_s": 0.03, "delay_s": 0.01},
        "disturbances": {
            "wind_speed_mean_ms": 2.0,
            "wind_speed_std_ms": 1.0,
            "thrust_misalignment_deg": 0.3,
            "cg_error_m": 0.005,
            "cg_shift_item": "test_gimbal",
        },
        "success": {"max_tilt_deg": 20.0, "min_rail_exit_speed_ms": 5.0},
        "monte_carlo": {"runs": 3, "seed": 5},
        "reference_disturbance_torque_nm": 0.01,
    }
    d.update(changes)
    p = tmp_path / "study.yaml"
    p.write_text(yaml.safe_dump(d), encoding="utf-8")
    return p


# ------------------------------------------------------------------------------------------ TODO
@pytest.mark.user_todo
@pytest.mark.xfail(
    strict=True, raises=NotImplementedError, reason="TODO(you): implement max_correctable_torque"
)
def test_todo_max_correctable_torque_hand_value():
    """T = 40 N, lever arm 0.30 m, delta_max = 6 deg -> 40 sin(6 deg) 0.30 = 1.2543 N m (docs/derivations/tvc_authority.md)."""
    f = tvc_trade.max_correctable_torque
    assert f(40.0, 0.30, math.radians(6.0)) == pytest.approx(1.2543, abs=1e-3)
    assert f(0.0, 0.30, math.radians(6.0)) == pytest.approx(0.0)  # no thrust, no torque
    assert f(40.0, 0.30, 0.0) == pytest.approx(0.0)  # no deflection, no torque
    assert f(np.array([40.0, 20.0]), np.array([0.3, 0.3]), math.radians(6.0)) == pytest.approx(
        [1.2543, 0.6272], abs=1e-3
    )


def test_oracle_matches_the_hand_value():
    assert float(oracle_torque(40.0, 0.30, math.radians(6.0))) == pytest.approx(1.2543, abs=1e-3)


# ------------------------------------------------------------------------------------ study file
def test_the_template_refuses_to_run_until_every_required_number_is_filled_in():
    with pytest.raises(ConfigError) as e:
        load_study(TEMPLATE)
    msg = str(e.value)
    for needed in (
        "base_config",
        "added_masses[0].mass_kg",
        "tvc.max_angle_deg",
        "tvc.time_constant_s",
        "disturbances.cg_error_m",
        "success.max_tilt_deg",
    ):
        assert needed in msg, msg
    assert "none is guessed" in msg


def test_unknown_keys_and_bad_values_are_errors(tmp_path):
    p = make_study(tmp_path, surprise=1)
    with pytest.raises(ConfigError, match="unknown study keys"):
        load_study(p)
    with pytest.raises(ConfigError, match="estimator"):
        load_study(make_study(tmp_path, estimator="magic"))
    with pytest.raises(ConfigError, match="runs"):
        load_study(make_study(tmp_path, monte_carlo={"runs": 0, "seed": 1}))
    assert load_study(make_study(tmp_path)).runs == 3


# -------------------------------------------------------------------------------- building runs
def test_configure_adds_masses_servo_estimator_and_controller(tmp_path):
    spec = load_study(make_study(tmp_path))
    base, _ = base_dict(spec)
    d = configure(spec, base, spec.motors[0])
    assert d["rocket"]["masses"][-1]["name"] == "test_gimbal" and d["fidelity"] == 5
    assert d["controller"]["type"] == "tvc_attitude" and d["estimator"]["type"] == "nav_kf"
    assert d["motor"]["file"] == spec.motors[0]
    assert d["tvc"]["max_angle_deg"] == 6.0
    assert configure(spec, base, spec.motors[0], control=False)["controller"]["type"] == "none"
    assert (
        len(base["rocket"].get("masses", [])) == len(d["rocket"]["masses"]) - 1
    )  # the base dict is not mutated


def test_cg_error_moves_the_vehicle_cg_by_exactly_the_stated_amount(tmp_path):
    spec = load_study(make_study(tmp_path))
    base, bdir = base_dict(spec)
    d = configure(spec, base, spec.motors[0])
    d["environment"]["wind"] = {"model": "none"}
    _, rec0 = run_cfg(d, bdir, 0, t_end=3.0)
    m0 = float(rec0.col("mass")[0])
    for seed in range(4):
        rng = np.random.default_rng(seed)
        dd, draw = disturb(spec, d, rng, m0)
        _, rec = run_cfg(dd, bdir, 0, t_end=3.0)
        assert abs(draw["cg_shift_m"]) == pytest.approx(0.005)
        assert float(rec.col("cg")[0]) - float(rec0.col("cg")[0]) == pytest.approx(
            draw["cg_shift_m"], abs=2e-6
        )
        assert float(rec.col("mass")[0]) == pytest.approx(m0)  # moving a mass does not change the mass


def test_disturbance_draws_have_the_stated_statistics(tmp_path):
    spec = load_study(
        make_study(
            tmp_path,
            disturbances={
                **yaml.safe_load(make_study(tmp_path).read_text())["disturbances"],
                "wind_speed_std_ms": 1.0,
            },
        )
    )
    base, _ = base_dict(spec)
    d = configure(spec, base, spec.motors[0])
    rng = np.random.default_rng(0)
    draws = [disturb(spec, d, rng, 0.6)[1] for _ in range(400)]
    wind = np.array([x["wind_ms"] for x in draws])
    assert (
        wind.min() >= 0.0
        and wind.mean() == pytest.approx(2.0, abs=0.15)
        and wind.std() == pytest.approx(1.0, abs=0.15)
    )
    assert all(x["misalignment_deg"] == 0.3 for x in draws)
    assert np.mean([x["cg_shift_m"] > 0 for x in draws]) == pytest.approx(0.5, abs=0.1)
    out, _ = disturb(spec, d, np.random.default_rng(1), 0.6)
    assert math.hypot(*out["motor"]["misalignment_deg"]) == pytest.approx(0.3)


def test_unknown_cg_shift_item_is_an_error(tmp_path):
    spec = load_study(make_study(tmp_path))
    spec.disturbances["cg_shift_item"] = "nonexistent"
    base, _ = base_dict(spec)
    with pytest.raises(ConfigError, match="not a mass"):
        disturb(spec, configure(spec, base, spec.motors[0]), np.random.default_rng(0), 0.6)


# ----------------------------------------------------------------------------------------- stats
def test_wilson_interval_hand_values():
    lo, hi = wilson(28, 30)
    assert (lo, hi) == pytest.approx((0.7868, 0.9816), abs=2e-3)
    assert wilson(0, 10)[0] == 0.0 and wilson(0, 10)[1] == pytest.approx(3.8416 / 13.8416, abs=1e-3)
    assert wilson(10, 10)[1] == 1.0 and wilson(5, 0) == (0.0, 1.0)
    assert wilson(50, 100)[0] < 0.5 < wilson(50, 100)[1]


def make_result(name, k, n):
    z = np.zeros(2)
    return MotorResult(
        name,
        20.0,
        5.0,
        2.0,
        1.8,
        z,
        z,
        z,
        None,
        None,
        None,
        None,
        None,
        k,
        n,
        0,
        {},
        np.ones(n),
        np.ones(n),
        np.ones(n),
        np.ones(n, bool),
    )


def test_comparison_sentence_claims_only_what_the_intervals_support():
    a, b = make_result("A", 29, 30), make_result("B", 10, 30)
    assert "outperforms" in comparison_sentence([a, b]) and "A (97%)" in comparison_sentence([a, b])
    c, d = make_result("C", 28, 30), make_result("D", 25, 30)
    s = comparison_sentence([c, d])
    assert "not distinguishable" in s and "outperforms" not in s
    assert comparison_sentence([a]) == ""


# ---------------------------------------------------------------------------------------- a run
def test_success_criteria(tmp_path):
    spec = load_study(make_study(tmp_path))
    base, bdir = base_dict(spec)
    d = configure(spec, base, spec.motors[0])
    d["environment"]["wind"] = {"model": "none"}
    _, rec = run_cfg(d, bdir, 0, t_end=7.0)
    ok = success(spec, rec)
    assert ok["ok"] and ok["max_tilt_deg"] < 20.0 and ok["rail_exit_ms"] > 5.0
    spec.success["max_tilt_deg"] = 1e-4
    bad = success(spec, rec)
    assert not bad["ok"] and bad["reason"] == "tilt"
    spec.success.update({"max_tilt_deg": 20.0, "min_rail_exit_speed_ms": 1e6})
    assert success(spec, rec)["reason"] == "rail_exit_speed"
    unstable = configure(spec, base, spec.motors[0], control=False)
    unstable["environment"]["wind"] = {"model": "none"}
    unstable["motor"]["misalignment_deg"] = [
        0.3,
        0.0,
    ]  # a perfectly symmetric unstable vehicle would balance on the pad forever
    _, passive = run_cfg(unstable, bdir, 0, t_end=7.0)
    assert (
        success(spec, passive)["max_tilt_deg"] > 90.0
    )  # the example vehicle is statically unstable: without TVC it tumbles


def test_evaluate_motor_end_to_end_with_a_reference_torque(tmp_path, with_oracle):
    spec = load_study(make_study(tmp_path))
    base, bdir = base_dict(spec)
    r = evaluate_motor(spec, base, bdir, spec.motors[0])
    assert r.runs == 3 and 0 <= r.successes <= 3 and len(r.tilts) == 3 and len(r.winds) == 3
    assert r.successes > r.passive_successes  # the gimbal helps on this unstable vehicle
    assert r.thrust_to_weight > 1.0 and r.rail_exit_speed_ms and r.rail_exit_speed_ms > 5.0
    _, rec = run_cfg(
        {**configure(spec, base, spec.motors[0]), "environment": {"wind": {"model": "none"}}},
        bdir,
        0,
        t_end=4.0,
    )
    assert r.thrust_to_weight == pytest.approx(
        float(rec.col("thrust").max()) / (float(rec.col("mass")[0]) * 9.80665), rel=1e-6
    )
    assert r.authority_time_s == pytest.approx(r.t[-1] - r.t[0], abs=0.1)  # rail exit to burnout
    assert r.max_torque is not None and np.allclose(
        r.max_torque, oracle_torque(r.thrust, r.lever_arm, math.radians(6.0))
    )
    useful = r.thrust >= 0.1 * r.thrust.max()  # the minimum is taken while the motor still thrusts
    assert r.torque_min_nm == pytest.approx(float(r.max_torque[useful].min()))
    assert r.torque_min_nm > 0.0 and r.torque_at_rail_exit_nm == pytest.approx(float(r.max_torque[0]))
    assert r.max_ang_accel is not None and r.max_ang_accel > 0
    assert r.time_with_margin_s is not None and 0 <= r.time_with_margin_s <= r.authority_time_s + 0.1
    assert np.all(r.lever_arm > 0)


def test_the_torque_rows_are_na_until_the_todo_is_implemented(tmp_path):
    spec = load_study(make_study(tmp_path, monte_carlo={"runs": 1, "seed": 2}))
    base, bdir = base_dict(spec)
    r = evaluate_motor(spec, base, bdir, spec.motors[0])
    assert r.max_torque is None and r.torque_min_nm is None and r.time_with_margin_s is None
    assert r.thrust_to_weight > 1.0  # the rest of the study does not depend on it


def test_report_files(tmp_path, with_oracle):
    spec = load_study(make_study(tmp_path, monte_carlo={"runs": 2, "seed": 3}))
    res = tvc_trade.run_study(spec)
    p = write_report(spec, res, tmp_path / "out")
    text = p.read_text(encoding="utf-8")
    assert (tmp_path / "out" / "authority.png").stat().st_size > 5000 and (
        tmp_path / "out" / "monte_carlo.png"
    ).stat().st_size > 5000
    for needed in (
        "AeroTech_G80T",
        "Estes_E16",
        "rail exit speed",
        "Wilson",
        "no controller",
        "What this does not tell you",
        "test_gimbal",
    ):
        assert needed in text, needed
    row = next(ln for ln in text.splitlines() if ln.startswith("| smallest max gimbal torque"))
    assert "n/a" not in row
    assert "not implemented yet" not in text


def test_report_marks_missing_torque_function(tmp_path):
    spec = load_study(
        make_study(tmp_path, monte_carlo={"runs": 1, "seed": 3}, reference_disturbance_torque_nm=None)
    )
    p = write_report(spec, tvc_trade.run_study(spec), tmp_path / "out")
    assert "max_correctable_torque()" in p.read_text(encoding="utf-8") and "n/a" in p.read_text(
        encoding="utf-8"
    )


def test_script_refuses_the_template(tmp_path):
    r = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "q01_trade_study.py"),
            "--study",
            str(TEMPLATE),
            "--out",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert r.returncode == 1 and "none is guessed" in r.stderr and "Traceback" not in r.stderr


def test_a_cg_error_that_would_push_the_mass_out_of_the_airframe_is_explained(tmp_path):
    spec = load_study(make_study(tmp_path, monte_carlo={"runs": 1, "seed": 1}))
    spec.disturbances["cg_error_m"] = 0.05  # 50 mm of CG shift from a 50 g mass: a 0.6 m move
    base, bdir = base_dict(spec)
    with pytest.raises(ConfigError, match="heavier cg_shift_item"):
        evaluate_motor(spec, base, bdir, spec.motors[0])
