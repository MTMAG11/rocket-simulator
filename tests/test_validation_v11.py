"""V1.1 validation protocol: registry splits, holdout log / staleness, fingerprint, input-uncertainty sampling.

These tests deliberately do NOT simulate the HOLDOUT flights of the real registry (that is what the protocol guards);
holdout mechanics are tested on a throw-away registry that points at a development flight.
"""

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from rocket_sim.errors import ConfigError
from rocket_sim.validation import registry as R
from rocket_sim.validation.compare import run_validation
from rocket_sim.validation.input_mc import INPUT_PRIORS, sample_overrides

VD = Path(__file__).resolve().parents[1] / "validation_data"


# ---------------------------------------------------------------------------------- fingerprint
def test_fingerprint_ignores_formatting_and_comments_but_not_code(tmp_path):
    root = tmp_path / "pkg"
    (root / "physics").mkdir(parents=True)
    f = root / "physics" / "a.py"
    f.write_text("def g(x):\n    return x*2+1\n", encoding="utf-8")
    h0 = R.physics_fingerprint(root)
    f.write_text("# a comment\ndef g( x ):\n\n    return x * 2 + 1   # trailing\n", encoding="utf-8")
    assert R.physics_fingerprint(root) == h0  # formatter-safe
    f.write_text("def g(x):\n    return x*2+1.0000001\n", encoding="utf-8")
    assert R.physics_fingerprint(root) != h0  # any executable change shows
    f.write_text("def g(x):\n    return x*2+1\n", encoding="utf-8")
    assert R.physics_fingerprint(root) == h0
    (root / "physics" / "b.py").write_text("X = 1\n", encoding="utf-8")
    assert R.physics_fingerprint(root) != h0  # a new file changes it


def test_fingerprint_of_the_real_package_is_stable_and_nonempty():
    a, b = R.physics_fingerprint(), R.physics_fingerprint()
    assert a == b and len(a) == 64


# ---------------------------------------------------------------------------- real registry file
def test_real_registry_is_consistent_and_split_is_as_preregistered():
    reg = R.load_registry(VD / "registry.yaml")
    ids = [f.id for f in reg.flights]
    assert len(ids) == len(set(ids)) == 7
    for f in reg.flights:
        assert f.definition.exists() and f.split in R.SPLITS and f.tier in (1, 2, 3)
        d = yaml.safe_load(f.definition.read_text(encoding="utf-8"))
        assert (f.definition.parent / d["sim_config"]).exists() and (
            f.definition.parent / d["telemetry"]["file"]
        ).exists()
    split = {s: sorted(f.id for f in reg.by_split(s)) for s in R.SPLITS}
    # the pre-registered assignment (written before the calibration/holdout flights were simulated)
    assert split["development"] == ["bella_lui", "ndrt_2020", "prometheus"]
    assert split["calibration"] == ["astra", "genesis"]
    assert split["holdout"] == ["cavour", "erebus11"]
    assert reg.frozen_physics_version
    assert any(
        "Estes" in str(e.get("id", "")) or "estes" in str(e.get("id", "")) for e in reg.excluded
    )  # the small-rocket gap is recorded


def test_calibration_record_exists_and_is_not_adopted():
    rec = yaml.safe_load((VD / "calibration_records.yaml").read_text(encoding="utf-8"))["records"][0]
    for k in (
        "id",
        "parameter",
        "old_value",
        "proposed_value",
        "why",
        "fit_data",
        "evidence",
        "effect",
        "decision",
        "status",
    ):
        assert k in rec, k
    assert rec["fit_data"] == ["genesis", "astra"]  # fitted on calibration flights only
    assert str(rec["status"]).startswith("NOT ADOPTED")
    # the default configs still use the nominal drag scale (calibration not silently applied)
    for name in (
        "genesis_sim",
        "astra_sim",
        "erebus11_sim",
        "cavour_sim",
        "bella_lui_sim",
        "ndrt_2020_sim",
        "prometheus_sim",
    ):
        cfg = yaml.safe_load((VD / f"{name}.yaml").read_text(encoding="utf-8"))
        assert cfg["rocket"].get("aero", {}).get("drag_scale", 1.0) == 1.0


def test_logged_holdout_results_match_the_documented_numbers():
    """The holdout log is the system of record. It must hold exactly one first evaluation per flight."""
    log = R.read_log(Path(__file__).resolve().parents[1] / "validation_results" / "holdout_log.jsonl")
    ev = [e for e in log if e.get("type") != "migration"]
    first = {e["flight"]: e for e in ev if e["calibration_id"] is None}
    assert set(first) == {"erebus11", "cavour"}
    assert all(e["status"] == "first holdout evaluation" for e in first.values())
    assert first["erebus11"]["apogee_err_pct"] == pytest.approx(-6.40, abs=0.01)
    assert first["cavour"]["apogee_err_pct"] == pytest.approx(4.52, abs=0.01)
    cal = {e["flight"]: e for e in ev if e["calibration_id"] == "cal-001"}
    assert cal["erebus11"]["apogee_err_pct"] == pytest.approx(-4.48, abs=0.01)
    assert cal["cavour"]["apogee_err_pct"] == pytest.approx(6.65, abs=0.01)
    for e in ev:
        assert len(e["physics_fingerprint"]) == 64 and e["physics_version"] in ("1.2.0", "1.2.1")


# --------------------------------------------------------------------------- holdout mechanics
@pytest.fixture
def toy_registry(tmp_path):
    """A registry whose 'holdout' flight is actually the cheap development flight bella_lui."""
    reg = {
        "protocol_version": 1,
        "frozen_physics_version": R.PHYSICS_VERSION,
        "flights": [
            {"id": "toy_dev", "definition": str(VD / "bella_lui.yaml"), "tier": 2, "split": "development"},
            {"id": "toy_hold", "definition": str(VD / "bella_lui.yaml"), "tier": 2, "split": "holdout"},
        ],
    }
    p = tmp_path / "registry.yaml"
    p.write_text(yaml.safe_dump(reg), encoding="utf-8")
    return R.load_registry(p), tmp_path


def test_holdout_is_skipped_without_confirmation_and_nothing_is_logged(toy_registry):
    reg, tmp = toy_registry
    rows = R.run_registry(reg, ("holdout",), tmp / "o", confirm_frozen=False, log_path=tmp / "log.jsonl")
    assert rows[0].apogee_err_pct is None and "SKIPPED" in rows[0].status
    assert not (tmp / "log.jsonl").exists()


def test_holdout_run_is_logged_and_labelled_and_goes_stale_when_physics_changes(toy_registry, monkeypatch):
    reg, tmp = toy_registry
    log = tmp / "log.jsonl"
    r1 = R.run_registry(reg, ("holdout",), tmp / "o", confirm_frozen=True, log_path=log)[0]
    assert r1.label == "HOLDOUT" and r1.status == "first holdout evaluation"
    e = R.read_log(log)
    assert (
        len(e) == 1
        and e[0]["flight"] == "toy_hold"
        and e[0]["physics_fingerprint"] == R.physics_fingerprint()
    )
    assert R.holdout_status("toy_hold", e, R.physics_fingerprint()) == "current"
    # re-running unchanged physics is labelled a re-evaluation, not a new test
    r2 = R.run_registry(reg, ("holdout",), tmp / "o", confirm_frozen=True, log_path=log)[0]
    assert "RE-EVALUATION with unchanged physics" in r2.status
    # physics changes -> the logged evaluation is stale; a further run says so
    monkeypatch.setattr(R, "physics_fingerprint", lambda *a, **k: "f" * 64)
    assert R.holdout_status("toy_hold", R.read_log(log), "f" * 64) == "stale"
    r3 = R.run_registry(reg, ("holdout",), tmp / "o", confirm_frozen=True, log_path=log)[0]
    assert "no longer a clean holdout" in r3.status
    assert R.holdout_status("never_run", [], "x") == "unevaluated"


def test_development_flights_are_always_labelled_in_sample(toy_registry):
    reg, tmp = toy_registry
    r = R.run_registry(reg, ("development",), tmp / "o")[0]
    assert r.label == "in-sample (development)" and r.status == "in-sample"
    text = R.summarize([r])
    assert "in-sample" in text and "RMS apogee error" in text


def test_migration_needs_reproduction_evidence(toy_registry, monkeypatch):
    reg, tmp = toy_registry
    log = tmp / "log.jsonl"
    R.run_registry(reg, ("holdout",), tmp / "o", confirm_frozen=True, log_path=log)
    real_fp = R.physics_fingerprint()
    new_fp = "a" * 64
    monkeypatch.setattr(R, "physics_fingerprint", lambda *a, **k: new_fp)
    assert R.holdout_status("toy_hold", R.read_log(log), new_fp) == "stale"
    # tampered log value: does not reproduce -> no migration written, still stale
    entries = R.read_log(log)
    entries[0]["apogee_err_pct"] += 1.0
    bad = tmp / "bad.jsonl"
    bad.write_text("\n".join(json.dumps(x) for x in entries) + "\n", encoding="utf-8")
    out = R.migrate_fingerprint(reg, bad)
    assert not out["ok"] and not any(e.get("type") == "migration" for e in R.read_log(bad))
    assert R.holdout_status("toy_hold", R.read_log(bad), new_fp) == "stale"
    # honest log: reproduces -> migration record -> current
    out = R.migrate_fingerprint(reg, log)
    assert out["ok"] and out["migrating_from"] == [real_fp]
    m = [e for e in R.read_log(log) if e.get("type") == "migration"]
    assert len(m) == 1 and m[0]["from_fingerprint"] == real_fp and m[0]["to_fingerprint"] == new_fp
    assert R.holdout_status("toy_hold", R.read_log(log), new_fp) == "current"


def test_registry_rejects_bad_files(tmp_path):
    p = tmp_path / "r.yaml"
    p.write_text(
        yaml.safe_dump({"flights": [{"id": "a", "definition": "x.yaml", "tier": 2, "split": "validation"}]}),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="unknown split"):
        R.load_registry(p)
    p.write_text(
        yaml.safe_dump({"flights": [{"id": "a", "definition": "x.yaml", "tier": 2, "split": "holdout"}] * 2}),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="duplicate"):
        R.load_registry(p)


# ------------------------------------------------------------------------- calibration flights
@pytest.mark.parametrize("name,expected", [("genesis", -2.54), ("astra", -0.93)])
def test_calibration_flight_regression(name, expected):
    r = run_validation(VD / f"{name}.yaml", plot=False)
    assert r.metrics["apogee"]["pct_error"] == pytest.approx(expected, abs=0.05)


# --------------------------------------------------------------------------- input uncertainty
def test_input_uncertainty_sampler_matches_declared_priors():
    nom = {"temp": 8.0, "elev": 84.0, "mass": 6.0, "thrust": 1.0}
    rng = np.random.default_rng(0)
    s = [sample_overrides(nom, rng, with_drag=True) for _ in range(4000)]
    col = lambda k: np.array([x[k] for x in s])  # noqa: E731
    assert col("rocket.dry_mass_kg").mean() == pytest.approx(6.0, rel=0.003)
    assert col("rocket.dry_mass_kg").std() / 6.0 == pytest.approx(
        INPUT_PRIORS["dry_mass_rel_sigma"], rel=0.06
    )
    assert col("environment.atmosphere.temperature_offset_k").std() == pytest.approx(
        INPUT_PRIORS["temperature_offset_sigma_k"], rel=0.06
    )
    assert col("motor.thrust_scale").std() == pytest.approx(INPUT_PRIORS["thrust_scale_sigma"], rel=0.06)
    assert col("rocket.aero.drag_scale").std() == pytest.approx(INPUT_PRIORS["drag_scale_sigma"], rel=0.06)
    assert col("launch.elevation_deg").max() <= 89.9
    w = col("environment.wind.speed_ms")
    assert w.min() >= 0 and np.sqrt(np.mean(w**2)) == pytest.approx(
        INPUT_PRIORS["wind_speed_sigma_ms"], rel=0.06
    )  # |N(0, s)|: rms = s
    assert (
        col("environment.wind.direction_from_deg").min() >= 0
        and col("environment.wind.direction_from_deg").max() <= 360
    )
    assert "rocket.aero.drag_scale" not in sample_overrides(nom, rng, with_drag=False)
    a = sample_overrides(nom, np.random.default_rng(3), True)
    assert a == sample_overrides(nom, np.random.default_rng(3), True)  # reproducible


def test_recorded_input_mc_results_are_consistent():
    p = Path(__file__).resolve().parents[1] / "validation_results" / "input_uncertainty_mc.json"
    rows = json.loads(p.read_text(encoding="utf-8"))
    assert len(rows) == 14 and {r["flight"] for r in rows} == {
        f.id for f in R.load_registry(VD / "registry.yaml").flights
    }
    for r in rows:
        assert r["p05_m"] < r["mean_m"] < r["p95_m"] and r["std_m"] > 0 and r["n"] == 150
        assert r["inside_90"] == (r["p05_m"] <= r["real_apogee_m"] <= r["p95_m"])


# --------------------------------------------------------------------- protocol hardening (critic 1)
def test_log_hash_chain_detects_tampering(tmp_path):
    log = tmp_path / "l.jsonl"
    for i in range(3):
        R.append_log(log, {"flight": f"f{i}", "physics_fingerprint": "x"})
    assert R.verify_log(log) == []
    lines = log.read_text(encoding="utf-8").splitlines()
    log.write_text("\n".join([lines[0], lines[1].replace("f1", "fX"), lines[2]]) + "\n", encoding="utf-8")
    assert R.verify_log(log)  # editing an earlier line breaks the next link
    log.write_text("\n".join([lines[0], lines[2]]) + "\n", encoding="utf-8")
    assert R.verify_log(log)  # deleting a line too


def test_holdout_needs_a_log_and_a_matching_freeze(toy_registry, monkeypatch):
    reg, tmp = toy_registry
    with pytest.raises(ConfigError, match="needs a log"):
        R.run_registry(reg, ("holdout",), None, confirm_frozen=True)
    monkeypatch.setattr(R, "PHYSICS_VERSION", "9.9.9")
    with pytest.raises(ConfigError, match="frozen for physics"):
        R.run_registry(reg, ("holdout",), tmp / "o", confirm_frozen=True, log_path=tmp / "l.jsonl")


def test_validate_command_refuses_holdout_flights(capsys):
    from rocket_sim.cli import main

    rc = main(["validate", str(VD / "erebus11.yaml"), "--no-plot"])
    assert rc != 0 and "HOLDOUT" in capsys.readouterr().err
