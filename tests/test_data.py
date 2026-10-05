import json
import shutil

import numpy as np
import pyarrow.parquet as pq
import pytest

from rocket_sim.config import config_to_dict, from_dict
from rocket_sim.data.batch import load_spec, read_manifest, read_runs, run_batch
from rocket_sim.data.browser import describe_dataset, describe_run, load_telemetry, reproduce_run
from rocket_sim.data.dataset import build_tabular, build_windows
from rocket_sim.data.export import (
    export_record,
    read_record_csv,
    read_record_npz,
    read_record_parquet,
)
from rocket_sim.data.montecarlo import (
    ParamSpec,
    check_seed_disjointness,
    derive_seeds,
    sample_run,
)
from rocket_sim.data.quality import check_record
from rocket_sim.data.schema import COLUMNS, columns_for, schema_dict, schema_markdown
from rocket_sim.data.spec import BatchSpec, DatasetSection, FeatureSpec, WindowSpec
from rocket_sim.errors import ConfigError, RocketSimError
from rocket_sim.simulation import run_simulation
from rocket_sim.version import PHYSICS_VERSION, SCHEMA_VERSION
from tests.conftest import CONFIG_G80, cfg_from


@pytest.fixture(scope="module")
def rec():
    return run_simulation(cfg_from(fidelity=3), seed=2)


# ------------------------------------------------------------------------------------ export
@pytest.mark.parametrize(
    "fmt,reader", [("parquet", read_record_parquet), ("npz", read_record_npz), ("csv", read_record_csv)]
)
def test_export_round_trip(tmp_path, rec, fmt, reader):
    (p,) = export_record(rec, tmp_path, "run", (fmt,))
    back = reader(p)
    assert back.columns == rec.columns
    tol = 1e-6 if fmt == "csv" else 0.0  # csv is 9 significant digits
    assert np.allclose(back.data, rec.data, rtol=tol, atol=tol if fmt == "csv" else 0.0)
    assert back.meta.simulation_id == rec.meta.simulation_id and back.meta.seed == rec.meta.seed
    assert back.meta.physics_version == PHYSICS_VERSION and back.meta.schema_version == SCHEMA_VERSION
    assert back.summary["apogee_m"] == pytest.approx(rec.summary["apogee_m"])
    assert [e.name for e in back.events] == [e.name for e in rec.events]


def test_json_export_contains_everything(tmp_path, rec):
    (p,) = export_record(rec, tmp_path, "run", ("json",))
    doc = json.loads(p.read_text())
    assert doc["metadata"]["seed"] == 2 and doc["schema"]["schema_version"] == SCHEMA_VERSION
    assert len(doc["data"]["t"]) == rec.n_rows
    assert doc["metadata"]["config"]["rocket"]["dry_mass_kg"] == pytest.approx(0.45)  # full config stored


def test_parquet_has_typed_columns_and_metadata(tmp_path, rec):
    (p,) = export_record(rec, tmp_path, "run", ("parquet",))
    t = pq.read_table(p)
    assert str(t.schema.field("phase").type) == "int8" and str(t.schema.field("t").type) == "double"
    assert b"rocket_sim.meta" in t.schema.metadata


def test_unknown_export_format_rejected(tmp_path, rec):
    with pytest.raises(RocketSimError, match="unknown export format"):
        export_record(rec, tmp_path, "x", ("xlsx",))


def test_schema_is_documented_and_consistent(rec):
    names = [c.name for c in COLUMNS]
    assert len(names) == len(set(names))
    assert all(c.unit and c.description for c in COLUMNS)
    assert rec.columns == [c.name for c in columns_for(3)]
    assert set(rec.columns) <= set(names)
    assert schema_dict()["schema_version"] == SCHEMA_VERSION
    assert f"v{SCHEMA_VERSION}" in schema_markdown()
    # sensors/estimator groups only appear at their fidelity levels
    assert not any(c.group == "sensors" for c in columns_for(3))
    assert any(c.group == "sensors" for c in columns_for(4))
    assert not any(c.group == "estimator" for c in columns_for(4, estimator=False))
    assert any(c.group == "estimator" for c in columns_for(5, estimator=True))


def test_every_schema_column_has_finite_data_in_truth_group(rec):
    for c in columns_for(3):
        assert np.isfinite(rec.col(c.name)).all(), c.name


# ----------------------------------------------------------------------------------- quality
def test_quality_gate_flags_bad_records(rec):
    import copy

    assert check_record(rec) == []
    bad = copy.deepcopy(rec)
    bad.data[10, bad.columns.index("pos_z")] = np.nan
    assert any("non-finite" in p for p in check_record(bad))
    bad = copy.deepcopy(rec)
    bad.data[:, bad.columns.index("speed")] *= 100.0
    assert any("speed exceeds" in p for p in check_record(bad))
    bad = copy.deepcopy(rec)
    bad.data[5, bad.columns.index("altitude")] = -5.0
    assert any("below ground" in p for p in check_record(bad))
    bad = copy.deepcopy(rec)
    bad.data[:, bad.columns.index("quat_w")] += 0.1
    assert any("quaternion" in p for p in check_record(bad))
    bad = copy.deepcopy(rec)
    bad.meta.status = "timeout"
    assert any("status" in p for p in check_record(bad))
    nolift = run_simulation(cfg_from(rocket={"dry_mass_kg": 20.0}), seed=0)
    assert check_record(nolift)  # never left the pad -> rejected


# --------------------------------------------------------------------------------- Monte Carlo
def test_seed_derivation_is_deterministic_and_order_independent():
    a = derive_seeds(7, 0, 41)
    b = derive_seeds(7, 0, 41)
    assert a.sim_seed == b.sim_seed
    assert derive_seeds(7, 1, 41).sim_seed != a.sim_seed  # split namespace
    assert derive_seeds(7, 0, 42).sim_seed != a.sim_seed
    assert derive_seeds(8, 0, 41).sim_seed != a.sim_seed
    check_seed_disjointness(7, {"train": (0, 300), "val": (1, 300), "test": (2, 300)})
    # same draws no matter what was generated before
    base = config_to_dict(cfg_from())
    specs = [ParamSpec(path="rocket.dry_mass_kg", dist="normal", rel_std=0.05)]
    d1 = sample_run(specs, base, derive_seeds(7, 0, 5))
    _ = [sample_run(specs, base, derive_seeds(7, 0, i)) for i in range(5)]
    assert d1 == sample_run(specs, base, derive_seeds(7, 0, 5))


@pytest.mark.parametrize(
    "spec,check",
    [
        (
            dict(dist="normal", rel_std=0.1),
            lambda v: abs(np.mean(v) - 0.45) < 0.01 and abs(np.std(v) - 0.045) < 0.01,
        ),
        (
            dict(dist="uniform", low=0.4, high=0.5),
            lambda v: min(v) >= 0.4 and max(v) <= 0.5 and abs(np.mean(v) - 0.45) < 0.01,
        ),
        (
            dict(dist="truncnormal", mean=0.45, std=0.1, low=0.4, high=0.5),
            lambda v: min(v) >= 0.4 and max(v) <= 0.5,
        ),
        (dict(dist="lognormal", sigma=0.1), lambda v: abs(np.median(v) - 0.45) < 0.01 and min(v) > 0),
        (dict(dist="uniform", rel_range=0.2), lambda v: min(v) >= 0.36 and max(v) <= 0.54),
    ],
)
def test_distributions(spec, check):
    base = config_to_dict(cfg_from())
    p = ParamSpec(path="rocket.dry_mass_kg", **spec)
    p.validate()
    vals = [sample_run([p], base, derive_seeds(1, 0, i))["rocket.dry_mass_kg"] for i in range(2000)]
    assert check(vals)


def test_invalid_parameter_specs():
    for kw in (
        dict(dist="normal"),
        dict(dist="bogus"),
        dict(dist="uniform", low=2, high=1),
        dict(dist="choice"),
        dict(dist="normal", std=1, rel_std=1),
        dict(dist="lognormal"),
    ):
        with pytest.raises(ConfigError):
            ParamSpec(path="rocket.dry_mass_kg", **kw).validate()
    base = config_to_dict(cfg_from())
    with pytest.raises(ConfigError, match="does not exist"):
        sample_run(
            [ParamSpec(path="rocket.nope", dist="normal", std=1.0, mean=0.0)], base, derive_seeds(1, 0, 0)
        )


# --------------------------------------------------------------------------------- windowing
def _section(**kw):
    base = dict(
        kind="windowed",
        sample_dt_s=0.1,
        inputs=[FeatureSpec(column="pos_z"), FeatureSpec(column="vel_z", lag=2)],
        targets=[FeatureSpec(column="vel_z")],
        window=WindowSpec(history=10, stride=3, target="future", horizon_s=0.5),
    )
    base.update(kw)
    return DatasetSection(**base)


def test_tabular_features_resampled_and_lagged(rec):
    tab = build_tabular(rec, _section(kind="tabular"))
    t = rec.col("t")
    lift, land = rec.event("liftoff").t, rec.event("landing").t
    assert tab.t[0] == pytest.approx(lift) and tab.t[-1] <= land + 1e-9
    assert np.allclose(np.diff(tab.t), 0.1)
    # resampled values equal direct interpolation of the record
    assert np.allclose(tab.inputs[:, 0], np.interp(tab.t, t, rec.col("pos_z")))
    # lag-2 column is the same signal delayed by two resampled steps
    vz = np.interp(tab.t, t, rec.col("vel_z"))
    assert np.allclose(tab.inputs[2:, 1], vz[:-2])
    assert np.allclose(tab.targets[:, 0], vz)


@pytest.mark.parametrize(
    "target,extra", [("current", {}), ("future", {"horizon_s": 0.5}), ("trajectory", {"trajectory_steps": 4})]
)
def test_windows_shapes_and_alignment(rec, target, extra):
    sec = _section(window=WindowSpec(history=10, stride=3, target=target, **extra))
    w = build_windows(rec, sec)
    tab = build_tabular(rec, _section(kind="tabular"))
    n = len(w.t_end)
    assert w.x.shape == (n, 10, 2) and w.x.dtype == np.float32
    for k in (0, n // 2, n - 1):
        end = int(round((w.t_end[k] - tab.t[0]) / 0.1))
        assert np.allclose(w.x[k], tab.inputs[end - 9 : end + 1], atol=1e-3)  # float32 storage
        if target == "current":
            assert np.allclose(w.y[k], tab.targets[end], atol=1e-3)
        elif target == "future":
            assert np.allclose(w.y[k], tab.targets[end + 5], atol=1e-3)
        else:
            assert w.y.shape == (n, 4, 1) and np.allclose(w.y[k], tab.targets[end + 1 : end + 5], atol=1e-3)
    # no window reaches past the end of the flight
    if target == "future":
        assert w.t_end.max() + 0.5 <= tab.t[-1] + 1e-9


def test_dataset_spec_validation():
    with pytest.raises(ConfigError, match="sample_dt_s"):
        BatchSpec(config={}, runs=1, dataset=_section(sample_dt_s=None)).validate()
    with pytest.raises(ConfigError, match="inputs and targets"):
        BatchSpec(config={}, runs=1, dataset=DatasetSection(kind="tabular")).validate()
    with pytest.raises(ConfigError, match="exactly one"):
        BatchSpec(config={}, base_config="x", runs=1).validate()
    with pytest.raises(ConfigError, match="derived"):
        BatchSpec(config={}, runs=1, dataset=_section(inputs=[FeatureSpec(derived="bogus")])).validate()


# ---------------------------------------------------------------------------- batch generation
def _batch_spec(tmp_path, name="t", kind="telemetry", workers=1, splits=None, runs=6, shard_runs=3):
    d = {
        "name": name,
        "config": config_to_dict(
            cfg_from(fidelity=2, fast=True, simulation={"dt_s": 0.02, "descent_dt_s": 0.1})
        ),
        "master_seed": 11,
        "shard_runs": shard_runs,
        "workers": workers,
        "output_dir": str(tmp_path / name),
        "parameters": [
            {"path": "rocket.dry_mass_kg", "dist": "normal", "rel_std": 0.05},
            {"path": "environment.wind.model", "dist": "choice", "values": ["constant"]},
            {"path": "environment.wind.speed_ms", "dist": "uniform", "low": 0, "high": 6},
        ],
        "dataset": {"kind": kind},
    }
    if splits:
        d["splits"] = splits
    else:
        d["runs"] = runs
    if kind == "windowed":
        d["dataset"] = {
            "kind": "windowed",
            "sample_dt_s": 0.1,
            "inputs": [{"column": "pos_z"}, {"column": "vel_z"}],
            "targets": [{"column": "pos_z"}],
            "window": {"history": 10, "stride": 5, "target": "future", "horizon_s": 0.5},
        }
    spec = from_dict(BatchSpec, d)
    spec.validate()
    return spec


def test_batch_telemetry_dataset_end_to_end(tmp_path):
    spec = _batch_spec(tmp_path)
    mp = run_batch(spec, tmp_path)
    m = read_manifest(mp)
    assert m["counts"]["simulations_accepted"] == 6 and m["counts"]["simulations_rejected"] == 0
    assert m["versions"]["physics"] == PHYSICS_VERSION and m["versions"]["schema"] == SCHEMA_VERSION
    assert m["fidelity_level"] == 2 and m["fast_mode"] is True
    assert m["seeds"]["master_seed"] == 11 and "SeedSequence" in m["seeds"]["derivation"]
    assert m["telemetry_columns"] and m["base_config"]["rocket"]["dry_mass_kg"] > 0
    assert len(m["files"]) == 2 and all(len(f["sha256"]) == 64 for f in m["files"])
    runs = read_runs(tmp_path / "t")
    assert runs.num_rows == 6 and len(set(runs["random_seed"].to_pylist())) == 6
    assert all(runs["accepted"].to_pylist())
    # data browser: metadata without telemetry; one run's telemetry; exact reproduction
    assert "6 accepted / 6 requested" in describe_dataset(tmp_path / "t")
    sid = runs["simulation_id"].to_pylist()[4]
    detail = describe_run(tmp_path / "t", sid)
    assert "rocket.dry_mass_kg" in detail and "apogee_m" in detail
    tel = load_telemetry(tmp_path / "t", sid)
    again = reproduce_run(tmp_path / "t", sid)
    assert np.array_equal(tel["pos_z"], again.col("pos_z")) and np.array_equal(
        tel["thrust"], again.col("thrust")
    )
    with pytest.raises(RocketSimError, match="not found"):
        load_telemetry(tmp_path / "t", "nope")


def test_batch_is_independent_of_worker_count_and_resumable(tmp_path):
    s1 = _batch_spec(tmp_path, "serial", workers=1)
    s2 = _batch_spec(tmp_path, "parallel", workers=2)
    m1 = run_batch(s1, tmp_path)
    m2 = run_batch(s2, tmp_path)
    r1, r2 = read_runs(tmp_path / "serial").to_pylist(), read_runs(tmp_path / "parallel").to_pylist()
    key = lambda r: r["run_index"]  # noqa: E731
    for a, b in zip(sorted(r1, key=key), sorted(r2, key=key)):
        assert a["random_seed"] == b["random_seed"] and a["res.apogee_m"] == b["res.apogee_m"]
        assert a["param.rocket.dry_mass_kg"] == b["param.rocket.dry_mass_kg"]
    f1 = {f["path"]: f["sha256"] for f in read_manifest(m1)["files"]}
    f2 = {f["path"]: f["sha256"] for f in read_manifest(m2)["files"]}
    assert f1.keys() == f2.keys()
    # simulate a crash after the first chunk: delete the second chunk's marker + shard, then resume
    out = tmp_path / "serial"
    marker = sorted((out / "chunks").glob("*.json"))[-1]
    shard = out / json.loads(marker.read_text())["shard"]
    marker.unlink()
    shard.unlink()
    first = sorted((out / "chunks").glob("*.json"))[0]
    mtime = first.stat().st_mtime_ns
    m3 = run_batch(s1, tmp_path)
    assert first.stat().st_mtime_ns == mtime  # completed chunk was NOT recomputed
    assert read_manifest(m3)["counts"]["simulations_accepted"] == 6
    assert {f["path"]: f["sha256"] for f in read_manifest(m3)["files"]} == f1  # identical after resume
    # a changed spec must not silently reuse the directory
    s_bad = _batch_spec(tmp_path, "serial", runs=8)
    with pytest.raises(ConfigError, match="different spec"):
        run_batch(s_bad, tmp_path)


def test_batch_rejects_bad_runs_and_logs_them(tmp_path):
    spec = _batch_spec(tmp_path, "bad")
    spec.parameters.append(
        from_dict(ParamSpec, {"path": "rocket.dry_mass_kg", "dist": "uniform", "low": 10.0, "high": 20.0})
    )
    # all runs now have a 10-20 kg airframe -> never lift off -> every run is rejected, none enters shards
    mp = run_batch(spec, tmp_path)
    m = read_manifest(mp)
    assert m["counts"]["simulations_accepted"] == 0 and m["counts"]["simulations_rejected"] == 6
    assert m["files"] == []
    lines = (tmp_path / "bad" / "rejected.jsonl").read_text().strip().splitlines()
    assert len(lines) == 6 and "status 'no_liftoff'" in json.loads(lines[0])["reason"]
    assert not list((tmp_path / "bad").rglob("*.parquet")) or all(
        "runs" in p.name for p in (tmp_path / "bad").rglob("*.parquet")
    )


def test_batch_crashing_runs_are_rejected_not_fatal(tmp_path):
    spec = _batch_spec(tmp_path, "crash")
    spec.parameters = [
        from_dict(ParamSpec, {"path": "launch.elevation_deg", "dist": "uniform", "low": 120.0, "high": 130.0})
    ]
    m = read_manifest(run_batch(spec, tmp_path))
    assert m["counts"]["simulations_rejected"] == 6
    assert (
        "elevation_deg"
        in json.loads((tmp_path / "crash" / "rejected.jsonl").read_text().splitlines()[0])["reason"]
    )


def test_splits_use_disjoint_seeds_and_support_holdout_distributions(tmp_path):
    splits = {
        "train": {"runs": 4},
        "val": {"runs": 2},
        "test": {
            "runs": 2,
            "parameters": [
                {"path": "environment.wind.model", "dist": "choice", "values": ["constant"]},
                {"path": "environment.wind.speed_ms", "dist": "uniform", "low": 8, "high": 12},
            ],
        },
    }
    spec = _batch_spec(tmp_path, "splits", splits=splits)
    m = read_manifest(run_batch(spec, tmp_path))
    runs = read_runs(tmp_path / "splits").to_pylist()
    seeds = [r["random_seed"] for r in runs]
    assert len(set(seeds)) == len(seeds)  # no seed shared across train/val/test
    wind = {
        s: [r["param.environment.wind.speed_ms"] for r in runs if r["split"] == s]
        for s in ("train", "val", "test")
    }
    assert max(wind["train"]) <= 6 and min(wind["test"]) >= 8  # test is an out-of-distribution hold-out
    assert m["leakage_checks"]["duplicate_parameter_vectors_across_splits"] == 0
    assert set(m["seeds"]["splits"]) == {"train", "val", "test"}
    assert len({v["namespace"] for v in m["seeds"]["splits"].values()}) == 3


def test_windowed_dataset_shards(tmp_path):
    spec = _batch_spec(tmp_path, "win", kind="windowed")
    m = read_manifest(run_batch(spec, tmp_path))
    assert m["kind"] == "windowed" and m["window"]["history"] == 10
    assert [f["name"] for f in m["feature_schema"]] == ["pos_z", "vel_z"] and m["label_schema"][0][
        "name"
    ] == "pos_z"
    z = np.load(tmp_path / "win" / m["files"][0]["path"])
    assert z["x"].shape[1:] == (10, 2) and z["y"].shape[0] == z["x"].shape[0] == len(z["t_end"])
    assert set(z["simulation_ids"].tolist()) <= {
        r["simulation_id"] for r in read_runs(tmp_path / "win").to_pylist()
    }
    assert m["counts"]["samples"] == sum(f["n_samples"] for f in m["files"])


def test_load_spec_from_yaml_example(tmp_path):
    spec, d = load_spec(CONFIG_G80.parent / "batch_example.yaml")
    assert spec.dataset.kind == "telemetry" and len(spec.parameters) >= 10
    shutil.rmtree(tmp_path, ignore_errors=True)


def test_example_batch_rejects_nothing_and_hash_ignores_output_dir(tmp_path):
    import copy

    from rocket_sim.data.batch import _base_dict, spec_hash

    spec, d = load_spec(CONFIG_G80.parent / "batch_example.yaml")
    spec.output_dir = str(tmp_path / "a")
    spec.runs = None
    spec.splits = {}
    spec.runs = 6
    spec.shard_runs = 3
    spec.workers = 1
    m = read_manifest(run_batch(spec, d))
    assert m["counts"]["simulations_rejected"] == 0 and m["warnings"] == []
    bf, _ = _base_dict(spec, d)
    s2 = copy.deepcopy(spec)
    s2.output_dir, s2.workers = str(tmp_path / "b"), 4
    assert spec_hash(spec, bf) == spec_hash(s2, bf)


def test_measured_inputs_are_resampled_causally():
    rec = run_simulation(
        cfg_from(fidelity=4, motor={"ignition_delay_s": 1.0}, simulation={"t_max_s": 5}), seed=0
    )
    sec = _section(
        kind="tabular",
        inputs=[FeatureSpec(column="meas_baro_pressure")],
        targets=[FeatureSpec(column="pos_z")],
        sample_dt_s=0.013,
        trim="none",
    )
    tab = build_tabular(rec, sec)
    t = rec.col("t")
    for k in (5, 50, 120):
        idx = np.searchsorted(t, tab.t[k], side="right") - 1
        assert tab.inputs[k, 0] == pytest.approx(
            rec.col("meas_baro_pressure")[idx], rel=1e-6
        )  # value from the past
