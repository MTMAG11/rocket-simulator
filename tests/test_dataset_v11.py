"""V1.1 datasets: correlated / linked sampling, leakage checks, statistics, quality gates, traceability."""

import json
from dataclasses import replace

import numpy as np
import pytest
from scipy import stats

from rocket_sim.config import config_to_dict, from_dict
from rocket_sim.data.batch import read_manifest, read_runs, run_batch
from rocket_sim.data.dataset import build_windows
from rocket_sim.data.leakage import (
    check_run_disjoint,
    near_duplicate_report,
    parameter_distribution_checks,
    temporal_causality_check,
)
from rocket_sim.data.montecarlo import CorrelationSpec, ParamSpec, derive_seeds, sample_run
from rocket_sim.data.quality import QualityLimits, check_record
from rocket_sim.data.spec import BatchSpec, DatasetSection, FeatureSpec, WindowSpec
from rocket_sim.errors import ConfigError
from rocket_sim.simulation import run_simulation
from rocket_sim.version import DATASET_VERSION
from tests.conftest import cfg_from

BASE = config_to_dict(cfg_from())
MASS, G = "rocket.dry_mass_kg", "environment.wind.speed_ms"


def draw(params, corr, n=3000, seed=5):
    out = [sample_run(params, BASE, derive_seeds(seed, 0, i), corr) for i in range(n)]
    return {p.path: np.array([o[p.path] for o in out]) for p in params}


# ---------------------------------------------------------------------------------- correlation
def test_copula_preserves_marginals_and_imposes_correlation():
    params = [
        ParamSpec(path=MASS, dist="normal", rel_std=0.05),
        ParamSpec(path=G, dist="uniform", low=0.0, high=8.0),
        ParamSpec(
            path="environment.atmosphere.temperature_offset_k", dist="lognormal", median=5.0, sigma=0.2
        ),
    ]
    corr = [
        CorrelationSpec(
            paths=[MASS, G, "environment.atmosphere.temperature_offset_k"],
            matrix=[[1.0, 0.8, 0.0], [0.8, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
    ]
    for c in corr:
        c.validate(params)
    d = draw(params, corr)
    m0 = BASE["rocket"]["dry_mass_kg"]
    # marginals unchanged: KS against the declared distributions
    assert stats.kstest(d[MASS], stats.norm(m0, 0.05 * m0).cdf).pvalue > 1e-3
    assert stats.kstest(d[G], stats.uniform(0, 8).cdf).pvalue > 1e-3
    assert (
        stats.kstest(
            d["environment.atmosphere.temperature_offset_k"], stats.lognorm(s=0.2, scale=5.0).cdf
        ).pvalue
        > 1e-3
    )
    # dependence: Pearson of (normal, uniform) ~ 0.8 * sqrt(3/pi) ... check rank correlation instead (copula-invariant)
    rho_s = stats.spearmanr(d[MASS], d[G]).statistic
    assert rho_s == pytest.approx(6 / np.pi * np.arcsin(0.8 / 2), abs=0.03)  # Spearman of a Gaussian copula
    assert abs(stats.spearmanr(d[MASS], d["environment.atmosphere.temperature_offset_k"]).statistic) < 0.05


def test_independent_parameters_unchanged_by_correlation_feature():
    params = [
        ParamSpec(path=MASS, dist="normal", rel_std=0.05),
        ParamSpec(path=G, dist="uniform", low=0.0, high=8.0),
    ]
    a = sample_run(params, BASE, derive_seeds(1, 0, 3))
    b = sample_run(params, BASE, derive_seeds(1, 0, 3), [])
    assert a == b  # existing specs reproduce bit-for-bit


def test_truncnormal_in_copula_respects_bounds():
    params = [
        ParamSpec(path=MASS, dist="truncnormal", mean=1.0, std=0.2, low=0.9, high=1.2),
        ParamSpec(path=G, dist="normal", mean=3.0, std=1.0),
    ]
    corr = [CorrelationSpec(paths=[MASS, G], matrix=[[1, -0.7], [-0.7, 1]])]
    d = draw(params, corr, 2000)
    assert d[MASS].min() >= 0.9 and d[MASS].max() <= 1.2
    assert stats.spearmanr(d[MASS], d[G]).statistic < -0.4


def test_correlation_validation():
    params = [
        ParamSpec(path=MASS, dist="normal", rel_std=0.05),
        ParamSpec(path=G, dist="choice", values=[1.0, 2.0]),
    ]
    with pytest.raises(ConfigError, match="positive definite"):
        CorrelationSpec([MASS, MASS + "2"], [[1, 1], [1, 1]]).validate(params)
    with pytest.raises(ConfigError, match="not a sampled"):
        CorrelationSpec([MASS, "x.y"], [[1, 0.1], [0.1, 1]]).validate(params)
    with pytest.raises(ConfigError, match="only"):
        CorrelationSpec([MASS, G], [[1, 0.1], [0.1, 1]]).validate(params)
    with pytest.raises(ConfigError, match="symmetric"):
        CorrelationSpec([MASS, G], [[1, 0.3], [0.1, 1]]).validate(params)


def test_linked_inertia_follows_sampled_mass():
    iyy = "rocket.inertia.iyy_kgm2"
    base = {**BASE, "rocket": {**BASE["rocket"], "inertia": {"ixx_kgm2": 0.001, "iyy_kgm2": 0.05}}}
    params = [
        ParamSpec(path=MASS, dist="normal", rel_std=0.1),
        ParamSpec(path=iyy, dist="linked", ref=MASS, exponent=1.0),
    ]
    for p in params:
        p.validate()
    for i in range(20):
        o = sample_run(params, base, derive_seeds(2, 0, i))
        assert o[iyy] / 0.05 == pytest.approx(o[MASS] / BASE["rocket"]["dry_mass_kg"], rel=1e-12)
    bad = [ParamSpec(path=iyy, dist="linked", ref=MASS), ParamSpec(path=MASS, dist="normal", rel_std=0.1)]
    with pytest.raises(ConfigError, match="earlier"):
        sample_run(bad, base, derive_seeds(2, 0, 0))
    with pytest.raises(ConfigError, match="ref"):
        ParamSpec(path=iyy, dist="linked").validate()


def test_gps_dropout_probability_can_be_randomised_per_run():
    p = ParamSpec(path="sensors.gps.dropout_probability", dist="uniform", low=0.0, high=0.4)
    base = {
        **BASE,
        "sensors": {
            **BASE.get("sensors", {}),
            "gps": {**BASE.get("sensors", {}).get("gps", {}), "dropout_probability": 0.0},
        },
    }
    v = np.array([sample_run([p], base, derive_seeds(3, 0, i))[p.path] for i in range(500)])
    assert v.min() >= 0.0 and v.max() <= 0.4 and abs(v.mean() - 0.2) < 0.02


# ------------------------------------------------------------------------------------- leakage
def _rows(train, test, eps_shift=0.0, rng=None):
    rng = rng or np.random.default_rng(0)
    rows = []
    for i, x in enumerate(train):
        rows.append(
            {
                "split": "train",
                "accepted": True,
                "simulation_id": f"t{i}",
                "random_seed": i,
                "param.a": x[0],
                "param.b": x[1],
            }
        )
    for i, x in enumerate(test):
        rows.append(
            {
                "split": "test",
                "accepted": True,
                "simulation_id": f"s{i}",
                "random_seed": 10_000 + i,
                "param.a": x[0] + eps_shift,
                "param.b": x[1],
            }
        )
    return rows


def test_near_duplicate_detection_catches_what_exact_matching_misses():  # (2-D, tiny perturbations)
    rng = np.random.default_rng(1)
    train = rng.normal(size=(300, 2))
    test_indep = rng.normal(size=(100, 2))
    clean = near_duplicate_report(_rows(train, test_indep), eps=1e-3)
    assert clean["splits"]["test"]["n_near_duplicates"] == 0
    leaked = np.vstack(
        [train[:30] + 1e-6 * rng.normal(size=(30, 2)), test_indep[:70]]
    )  # 30 tiny perturbations of train rows
    rep = near_duplicate_report(_rows(train, leaked), eps=1e-3)
    assert rep["splits"]["test"]["n_near_duplicates"] == 30
    # exact-vector matching (the V1 check) would have reported none of them
    keys = {(r["param.a"], r["param.b"]) for r in _rows(train, leaked) if r["split"] == "train"}
    assert not any((r["param.a"], r["param.b"]) in keys for r in _rows(train, leaked) if r["split"] == "test")
    assert rep["reference_within_median_nn"] > 10 * 1e-3  # eps is far below the natural spacing


def test_run_disjointness_check_flags_shared_seeds():
    rows = [
        {"simulation_id": "a", "split": "train", "random_seed": 1},
        {"simulation_id": "b", "split": "test", "random_seed": 2},
    ]
    assert check_run_disjoint(rows) == {"ids_in_multiple_splits": 0, "seeds_in_multiple_splits": 0}
    rows.append({"simulation_id": "c", "split": "val", "random_seed": 2})
    assert check_run_disjoint(rows)["seeds_in_multiple_splits"] == 1


def _section(inputs, dt=0.05, hist=10):
    return DatasetSection(
        kind="windowed",
        sample_dt_s=dt,
        inputs=inputs,
        targets=[FeatureSpec(column="vel_z")],
        window=WindowSpec(history=hist, stride=2, target="future", horizon_s=0.25),
    )


@pytest.fixture(scope="module")
def rec():
    return run_simulation(cfg_from(fidelity=4, motor={"ignition_delay_s": 0.5}), seed=1)


def test_temporal_causality_of_truth_and_measured_inputs(rec):
    sec = _section(
        [
            FeatureSpec(column="pos_z"),
            FeatureSpec(column="vel_z", lag=2),
            FeatureSpec(column="meas_baro_pressure"),
            FeatureSpec(derived="time_since_ignition"),
        ],
        dt=0.013,
    )  # off-grid dt exercises the hold
    r = temporal_causality_check(rec, sec)
    assert r["causal"] and r["rows_compared"] > 1000 and r["max_abs_difference"] == 0.0


def test_temporal_check_detects_a_leaky_interpolating_pipeline(rec, monkeypatch):
    """Negative control: with LINEAR interpolation of inputs (the old truth-input behaviour) the detector must fire."""
    import rocket_sim.data.dataset as ds

    orig = ds.resample

    def leaky(rec_, values, t0, t1, dt, causal_flags=None):
        return orig(rec_, values, t0, t1, dt, [False] * len(values))

    monkeypatch.setattr(ds, "resample", leaky)
    sec = _section([FeatureSpec(column="pos_z")], dt=0.013)
    r = temporal_causality_check(rec, sec)
    assert not r["causal"] and r["max_abs_difference"] > 0.0


def test_windows_never_span_two_flights(rec):
    w = build_windows(rec, _section([FeatureSpec(column="pos_z")]))
    # a window is built from one record only: every t_end lies inside that record's flight
    lift, land = rec.event("liftoff").t, rec.event("landing").t
    assert w.t_end.min() >= lift and w.t_end.max() <= land


# ------------------------------------------------------------------- batch: report, KS, traceability
def _spec(tmp_path, runs=240, name="v11", corr=False):
    d = {
        "name": name,
        "config": config_to_dict(
            cfg_from(fidelity=2, fast=True, simulation={"dt_s": 0.05, "descent_dt_s": 0.2})
        ),
        "master_seed": 21,
        "shard_runs": 40,
        "workers": 1,
        "output_dir": str(tmp_path / name),
        "splits": {"train": {"runs": runs}, "test": {"runs": 20}},
        "parameters": [
            {"path": "rocket.dry_mass_kg", "dist": "normal", "rel_std": 0.03},
            {"path": "environment.wind.model", "dist": "choice", "values": ["constant"]},
            {"path": "environment.wind.speed_ms", "dist": "uniform", "low": 0, "high": 6},
        ],
        "dataset": {"kind": "telemetry"},
    }
    if corr:
        d["correlations"] = [
            {"paths": ["rocket.dry_mass_kg", "environment.wind.speed_ms"], "matrix": [[1, 0.5], [0.5, 1]]}
        ]
    s = from_dict(BatchSpec, d)
    s.validate()
    return s


def test_batch_writes_statistics_leakage_report_and_traceability(tmp_path):
    spec = _spec(tmp_path, runs=240)
    m = read_manifest(run_batch(spec, tmp_path))
    assert m["dataset_version"] == DATASET_VERSION and m["versions"]["dataset"] == DATASET_VERSION
    assert m["leakage_checks"]["run_disjointness"] == {
        "ids_in_multiple_splits": 0,
        "seeds_in_multiple_splits": 0,
    }
    nd = m["leakage_checks"]["near_duplicates"]
    assert nd["splits"]["test"]["n_near_duplicates"] == 0 and nd["splits"]["test"]["n"] == 20
    assert "zero-order hold" in m["leakage_checks"]["input_resampling"]
    st = m["statistics"]["splits"]["train"]
    assert (
        st["accepted"] == 240
        and "apogee_m" in st["results"]
        and st["parameters"]["rocket.dry_mass_kg"]["std"] > 0
    )
    # sampler really draws the declared marginals (KS over 240 runs); none flagged
    assert m["sampler_checks"] and not any(c["flag"] for c in m["sampler_checks"])
    assert (
        (tmp_path / "v11" / "dataset_report.md")
        .read_text(encoding="utf-8")
        .startswith("# Dataset statistics")
    )
    runs = read_runs(tmp_path / "v11").to_pylist()
    assert {r["dataset_id"] for r in runs} == {m["dataset_id"]} and all(
        r["dataset_version"] == DATASET_VERSION for r in runs
    )
    assert all(len(r["config_hash"]) >= 8 for r in runs if r["accepted"])
    assert m["warnings"] == []


def test_batch_with_correlated_parameters_is_reproducible(tmp_path):
    m1 = run_batch(_spec(tmp_path, runs=30, name="c1", corr=True), tmp_path)
    run_batch(_spec(tmp_path, runs=30, name="c2", corr=True), tmp_path)
    r1, r2 = read_runs(tmp_path / "c1").to_pylist(), read_runs(tmp_path / "c2").to_pylist()
    assert [r["param.rocket.dry_mass_kg"] for r in r1] == [r["param.rocket.dry_mass_kg"] for r in r2]
    m = np.array([r["param.rocket.dry_mass_kg"] for r in r1])
    w = np.array([r["param.environment.wind.speed_ms"] for r in r1])
    assert stats.spearmanr(m, w).statistic > 0.1  # positive dependence is present (n = 30: loose)
    assert read_manifest(m1)["spec"]["correlations"][0]["matrix"][0][1] == 0.5


def test_ks_check_flags_a_wrong_sampler():
    base = BASE
    p = ParamSpec(path=MASS, dist="normal", rel_std=0.05)
    m0 = base["rocket"]["dry_mass_kg"]
    rng = np.random.default_rng(0)
    good = [{"accepted": True, f"param.{MASS}": float(rng.normal(m0, 0.05 * m0))} for _ in range(500)]
    bad = [{"accepted": True, f"param.{MASS}": float(rng.normal(m0 * 1.03, 0.05 * m0))} for _ in range(500)]
    assert not parameter_distribution_checks(good, [p], base)[0]["flag"]
    assert parameter_distribution_checks(bad, [p], base)[0]["flag"]


# ---------------------------------------------------------------------------------- quality gates
def test_quality_gate_rejects_each_failure_class(rec):
    assert check_record(rec, QualityLimits(require_landing=True)) == []
    cases = {
        "non-finite": lambda r: r.data.__setitem__((3, r.columns.index("pos_x")), np.nan),
        "speed exceeds": lambda r: r.data.__setitem__((slice(None), r.columns.index("speed")), 1e5),
        "below ground": lambda r: r.data.__setitem__((4, r.columns.index("altitude")), -9.0),
        "angular rate": lambda r: r.data.__setitem__((5, r.columns.index("omega_q")), 1e5),
        "quaternion": lambda r: r.data.__setitem__((slice(None), r.columns.index("quat_w")), 2.0),
    }
    import copy

    for needle, mutate in cases.items():
        bad = copy.deepcopy(rec)
        mutate(bad)
        problems = check_record(bad)
        assert any(needle in p for p in problems), (needle, problems)
    missing = check_record(rec, QualityLimits(required_columns=("fin_0",)))
    assert any("missing columns" in p for p in missing)
    no_land = replace(rec, events=[e for e in rec.events if e.name != "landing"])
    assert any("no landing" in p for p in check_record(no_land))


# --------------------------------------------------------------------------------- experiments
def test_experiment_is_versioned_and_reproducible(tmp_path):
    from rocket_sim.experiments import list_experiments, run_experiment, verify_experiment

    spec = _spec(tmp_path, runs=6, name="expbatch")
    spec.splits = {"train": spec.splits["train"]}
    bp = tmp_path / "batch.yaml"
    import dataclasses

    import yaml

    bp.write_text(
        yaml.safe_dump({k: v for k, v in dataclasses.asdict(spec).items() if k != "output_dir"}),
        encoding="utf-8",
    )  # (JSON is not safe: YAML 1.1 reads 6e-05 as a string)
    ef = tmp_path / "exp.yaml"
    ef.write_text("name: demo\ndescription: test\nbatch: batch.yaml\nmaster_seed: 5\n", encoding="utf-8")
    root = tmp_path / "experiments"
    out = run_experiment(ef, root, workers=1)
    rec = json.loads((out / "experiment.json").read_text(encoding="utf-8"))
    assert rec["experiment_id"].startswith("demo-") and rec["master_seed"] == 5
    for k in (
        "git",
        "versions",
        "spec_hash",
        "dataset_id",
        "base_config_hash",
        "environment",
        "outputs",
        "created_utc",
    ):
        assert k in rec
    assert rec["versions"]["physics"] and rec["counts"]["simulations_accepted"] == 6
    assert (out / "spec.yaml").exists() and (out / "dataset" / "manifest.json").exists()
    assert [e["experiment_id"] for e in list_experiments(root)] == [rec["experiment_id"]]
    v = verify_experiment(out)
    assert v["identical"] and v["shards"] >= 1  # regenerated data is bit-identical
    # a different seed gives a different spec hash and different data
    ef2 = tmp_path / "exp2.yaml"
    ef2.write_text("name: demo\nbatch: batch.yaml\nmaster_seed: 6\n", encoding="utf-8")
    rec2 = json.loads((run_experiment(ef2, root, workers=1) / "experiment.json").read_text(encoding="utf-8"))
    assert rec2["spec_hash"] != rec["spec_hash"] and rec2["outputs"]["shards"] != rec["outputs"]["shards"]


def test_near_duplicate_detector_works_at_realistic_dimension_and_perturbation():
    """20 parameters, test runs perturbed by 0.02 sigma from training runs (invisible to a fixed eps of 1e-3)."""
    rng = np.random.default_rng(4)
    d, n = 20, 400
    train = rng.normal(size=(n, d))

    def rows(test):
        out = []
        for i, x in enumerate(train):
            out.append(
                {
                    "split": "train",
                    "accepted": True,
                    "simulation_id": f"t{i}",
                    "random_seed": i,
                    **{f"param.p{k}": x[k] for k in range(d)},
                }
            )
        for i, x in enumerate(test):
            out.append(
                {
                    "split": "test",
                    "accepted": True,
                    "simulation_id": f"s{i}",
                    "random_seed": 9000 + i,
                    **{f"param.p{k}": x[k] for k in range(d)},
                }
            )
        return out

    leaked = train[:40] + 0.02 * rng.normal(size=(40, d))
    fresh = rng.normal(size=(40, d))
    assert near_duplicate_report(rows(leaked))["splits"]["test"]["n_near_duplicates"] == 40
    assert near_duplicate_report(rows(fresh))["splits"]["test"]["n_near_duplicates"] == 0


def test_temporal_check_covers_the_windowed_path(rec):
    sec = _section([FeatureSpec(column="pos_z"), FeatureSpec(column="meas_baro_pressure")], dt=0.013)
    assert temporal_causality_check(rec, sec)["causal"]


def test_split_that_drops_global_parameters_raises_a_warning(tmp_path):
    spec = _spec(tmp_path, runs=6, name="dropwarn")
    spec.splits["test"].parameters = spec.parameters[
        :1
    ]  # replaces the list: the wind parameters become nominal
    m = read_manifest(run_batch(spec, tmp_path))
    assert any("SPLIT 'test'" in w and "stay nominal" in w for w in m["warnings"])
    assert m["leakage_checks"]["near_duplicates"]["parameters_not_compared"] is not None
