"""Structural load screen: peak q*alpha and lateral load against configured limits."""

from __future__ import annotations

import math

import numpy as np
import pytest

from rocket_sim.config import apply_overrides, config_from_dict, read_mapping
from rocket_sim.errors import ConfigError
from rocket_sim.reporting import summary_text
from rocket_sim.resources import default_config
from rocket_sim.simulation import Simulation, structure
from tests.conftest import ROOT


def run(overrides=None, path=None, seed=0):
    p = path or default_config()
    raw = read_mapping(p)
    return Simulation(
        config_from_dict(apply_overrides(raw, overrides or {}), base_dir=p.parent), seed=seed
    ).run()


@pytest.fixture(scope="module")
def stable():
    return run()


def window(rec):
    return (rec.col("phase") < 5) & (rec.col("airspeed") > structure.MIN_AIRSPEED)


# ------------------------------------------------------------------------------------------- q * alpha
def test_q_alpha_hand_values():
    assert structure.q_alpha(10_000.0, 0.2) == pytest.approx(2000.0)
    assert structure.q_alpha(10_000.0, -0.2) == pytest.approx(2000.0)  # magnitude of alpha
    assert np.allclose(structure.q_alpha(np.array([1.0, 2.0]), np.array([0.5, -0.25])), [0.5, 0.5])


def test_peak_q_alpha_matches_an_independent_calculation(stable):
    sel = window(stable)
    ref = float(np.max(stable.col("qdyn")[sel] * np.abs(stable.col("aoa")[sel])))
    assert stable.summary["max_q_alpha_pa_rad"] == pytest.approx(ref)
    t_ref = float(stable.col("t")[sel][np.argmax(stable.col("qdyn")[sel] * np.abs(stable.col("aoa")[sel]))])
    assert stable.summary["max_q_alpha_time_s"] == pytest.approx(t_ref)
    assert ref > 0.0


def test_no_limit_means_a_reported_number_but_no_flag(stable):
    s = stable.summary
    assert s["structural_limits_set"] is False and s["structural_limit_exceeded"] is False
    assert not any("STRUCTURAL" in w for w in stable.meta.warnings)
    text = summary_text(stable)
    assert "Peak q*alpha" in text and "no limit configured" in text


def test_limit_exceeded_is_flagged_in_warnings_summary_and_text(stable):
    peak = stable.summary["max_q_alpha_pa_rad"]
    rec = run({"structure.max_q_alpha_pa_rad": peak * 0.5})
    assert rec.summary["structural_limit_exceeded"] is True and rec.summary["structural_limits_set"] is True
    (w,) = [w for w in rec.meta.warnings if "STRUCTURAL" in w]
    assert "q*alpha" in w and f"{peak * 0.5:.0f}" in w
    assert "EXCEEDS limit" in summary_text(rec)


def test_limit_not_exceeded_is_reported_as_within(stable):
    peak = stable.summary["max_q_alpha_pa_rad"]
    rec = run({"structure.max_q_alpha_pa_rad": peak * 2.0})
    assert rec.summary["structural_limit_exceeded"] is False
    assert not any("STRUCTURAL" in w for w in rec.meta.warnings)
    assert "within limit" in summary_text(rec)


def test_the_tumbling_example_run_is_flagged():
    """The closed-loop TVC example tumbles (angle of attack ~180 deg); it used to be reported as a normal flight."""
    rec = run(path=ROOT / "configs" / "example_tvc_closed_loop.yaml", seed=3)
    assert math.degrees(float(rec.col("aoa")[window(rec)].max())) > 90.0
    assert rec.summary["structural_limit_exceeded"] is True  # placeholder limit in the example config
    assert any("STRUCTURAL" in w and "q*alpha" in w for w in rec.meta.warnings)


def cfg_with(structure):
    return config_from_dict(
        apply_overrides(read_mapping(default_config()), {"structure": structure}),
        base_dir=default_config().parent,
    )


def test_the_limit_is_configuration_validated():
    for bad in (-1.0, 0.0, float("nan"), float("inf")):
        with pytest.raises(ConfigError, match=r"structure\.max_q_alpha_pa_rad"):
            cfg_with({"max_q_alpha_pa_rad": bad})
    cfg = cfg_with({"max_lateral_load_n": 800.0})
    assert cfg.structure.max_lateral_load_n == 800.0 and cfg.structure.max_q_alpha_pa_rad is None
    assert cfg_with({}).structure.max_q_alpha_pa_rad is None  # no shipped default limit


def test_the_screen_stored_in_the_run_metadata_config(stable):
    assert stable.meta.config["structure"] == {"max_q_alpha_pa_rad": None, "max_lateral_load_n": None}


# ------------------------------------------------------------------------------------- lateral load
def test_lateral_load_is_unavailable_until_the_todo_is_implemented(stable):
    assert stable.summary["max_lateral_load_n"] is None
    rec = run({"structure.max_lateral_load_n": 100.0})
    assert any("not checked" in w and "body_normal_force" in w for w in rec.meta.warnings)
    assert (
        rec.summary["structural_limit_exceeded"] is False
    )  # an unchecked limit is never reported as passed or failed


def reference_normal_force(lift, drag, alpha):
    """Test-local oracle (the inverse of C_D = C_A cos a + C_N sin a, C_L = C_N cos a - C_A sin a), not the user's TODO."""
    return lift * math.cos(alpha) + drag * math.sin(alpha)


def test_lateral_pipeline_with_a_reference_implementation(stable, monkeypatch):
    monkeypatch.setattr(structure, "body_normal_force", reference_normal_force)
    sel = window(stable)
    ref = np.abs(
        [
            reference_normal_force(lf, dr - ch, a)
            for lf, dr, ch, a in zip(
                stable.col("lift")[sel],
                stable.col("drag")[sel],
                stable.col("chute_drag")[sel],
                stable.col("aoa")[sel],
                strict=True,
            )
        ]
    )
    res = structure.check_structure(stable, structure.StructureCfg(max_lateral_load_n=float(ref.max()) * 0.5))
    assert res.lateral_load_available and res.max_lateral_load_n == pytest.approx(float(ref.max()))
    assert res.exceeded == ["lateral_load"] and any("lateral aerodynamic load" in w for w in res.warnings)
    ok = structure.check_structure(stable, structure.StructureCfg(max_lateral_load_n=float(ref.max()) * 2.0))
    assert ok.exceeded == [] and ok.warnings == []


def test_reference_oracle_preserves_force_magnitude():
    n = reference_normal_force(50.0, 100.0, math.radians(10.0))
    a = 100.0 * math.cos(math.radians(10.0)) - 50.0 * math.sin(math.radians(10.0))
    assert n == pytest.approx(66.6052, abs=1e-4)
    assert math.hypot(n, a) == pytest.approx(math.hypot(50.0, 100.0))


# ----------------------------------------------------------------------------------------- user TODO
@pytest.mark.user_todo
@pytest.mark.xfail(
    strict=True, raises=NotImplementedError, reason="TODO(you): implement structure.body_normal_force"
)
def test_todo_body_normal_force_hand_value():
    """L = 50 N, D = 100 N, alpha = 10 deg  ->  N = 66.6052 N (docs/derivations/structural_load.md, section 3)."""
    assert structure.body_normal_force(50.0, 100.0, math.radians(10.0)) == pytest.approx(66.6052, abs=1e-3)
    assert structure.body_normal_force(0.0, 100.0, 0.0) == pytest.approx(
        0.0, abs=1e-12
    )  # no angle of attack, no normal force
    assert structure.body_normal_force(50.0, 100.0, math.radians(90.0)) == pytest.approx(
        100.0
    )  # lift axis is the drag axis
