"""Headless batch simulation and dataset generation: parallel, streaming, checkpointed.

Workflow::

    spec (YAML) -> parameter draws (per-run seeds) -> simulate -> quality gate -> shard files
                -> chunk JSON (completion marker) -> merge -> runs.parquet + manifest.json

Output directory layout::

    manifest.json            dataset manifest (versions, config, seeds, schemas, counts, checksums)
    runs.parquet             one row per attempted run: id, split, seed, accepted, params, results
    rejected.jsonl           every rejected run with its reasons (never enters shards)
    shards/<split>/*.parquet|npz   data shards (one per chunk of ``shard_runs`` runs)
    chunks/*.json            per-chunk completion markers (the checkpoint)
    state.json               spec hash guarding against resuming with a changed spec

Checkpointing: a chunk is complete when its marker JSON exists (written last, atomically). On
restart completed chunks are skipped, so a crash at run 73,000 of 100,000 loses at most the
chunks in flight. Results are independent of worker count and chunking (per-run seeds).
"""

from __future__ import annotations

import concurrent.futures as cf
import datetime as _dt
import hashlib
import json
import os
import time
import uuid
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from ..config import config_from_dict, config_hash, config_to_dict, from_dict, read_mapping
from ..errors import ConfigError, RocketSimError
from ..simulation import Simulation
from ..version import CONFIG_VERSION, DATASET_VERSION, PHYSICS_VERSION, SCHEMA_VERSION, SIM_VERSION
from .dataset import build_tabular, build_windows, required_columns
from .export import _arrow_table
from .montecarlo import check_seed_disjointness, derive_seeds, sample_run
from .quality import check_record
from .schema import columns_for, schema_dict
from .spec import BatchSpec

Progress = Callable[[int, int, int, int], None]


def load_spec(path: str | Path, validate: bool = True) -> tuple[BatchSpec, Path]:
    """Parse a batch spec. ``validate=False`` lets the CLI apply --runs before validation
    (``run_batch`` always validates)."""
    p = Path(path)
    spec = from_dict(BatchSpec, read_mapping(p))
    if validate:
        spec.validate()
    return spec, p.parent


def _base_dict(spec: BatchSpec, spec_dir: Path) -> tuple[dict[str, Any], Path]:
    """Full (defaults-included) base config dict with spec-level overrides applied."""
    from ..config import apply_overrides, resolve_path

    if spec.base_config is not None:
        cand = Path(spec.base_config)
        if not cand.is_absolute() and not (spec_dir / cand).exists():
            from ..config.loader import PROJECT_ROOT

            cand = PROJECT_ROOT / cand if (PROJECT_ROOT / cand).exists() else cand
        else:
            cand = spec_dir / cand if not cand.is_absolute() else cand
        raw = read_mapping(cand)
        base_dir = cand.parent
    else:
        raw = dict(spec.config or {})
        base_dir = spec_dir
    raw = apply_overrides(raw, spec.overrides)
    cfg = config_from_dict(raw, base_dir=base_dir)
    _ = resolve_path  # (file existence is validated per run by the builders)
    return config_to_dict(cfg), base_dir


def spec_hash(spec: BatchSpec, base_full: dict[str, Any]) -> str:
    d = asdict(spec)
    d.pop("output_dir", None)  # where results go and how many processes compute them do not change results
    d.pop("workers", None)
    motor = base_full["motor"]["file"]
    try:
        from ..config import config_from_dict, resolve_path

        msha = hashlib.sha256(resolve_path(config_from_dict(base_full), motor).read_bytes()).hexdigest()
    except Exception:
        msha = ""
    blob = json.dumps(
        {
            "spec": d,
            "base": base_full,
            "motor_sha256": msha,
            "v": [SIM_VERSION, PHYSICS_VERSION, SCHEMA_VERSION],
        },
        sort_keys=True,
        default=str,
    )
    return hashlib.sha256(blob.encode()).hexdigest()[:16]


def _atomic_write_text(path: Path, text: str) -> None:
    tmp = path.with_suffix(path.suffix + f".tmp{os.getpid()}")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for blk in iter(lambda: f.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def _flatten(prefix: str, d: dict[str, Any]) -> dict[str, Any]:
    out = {}
    for k, v in d.items():
        if isinstance(v, bool | int | float | str) or v is None:
            out[f"{prefix}{k}"] = v
    return out


def execute_chunk(task: dict[str, Any]) -> dict[str, Any]:
    """Simulate runs [start, stop) of one split; write the shard + completion marker."""
    spec: BatchSpec = from_dict(BatchSpec, task["spec"])
    out_dir = Path(task["out_dir"])
    split, ns = task["split"], task["namespace"]
    start, stop, chunk_id = task["start"], task["stop"], task["chunk_id"]
    base_full: dict[str, Any] = task["base_full"]
    base_dir = Path(task["base_dir"])
    sp = spec.split_table()[split]
    params = sp.parameters if sp.parameters is not None else spec.parameters
    section = spec.dataset
    required = required_columns(section) if section.kind != "telemetry" else ()
    limits = spec.quality.limits(required)
    t0 = time.perf_counter()

    rows: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    tables: list[pa.Table] = []
    npz_x: list[np.ndarray] = []
    npz_y: list[np.ndarray] = []
    npz_t: list[np.ndarray] = []
    npz_ref: list[np.ndarray] = []
    npz_ids: list[str] = []
    n_samples = 0

    for idx in range(start, stop):
        sid = f"{spec.name}-{split}-{idx:08d}"
        seeds = derive_seeds(spec.master_seed, ns, idx)
        row: dict[str, Any] = {
            "simulation_id": sid,
            "split": split,
            "run_index": idx,
            "random_seed": seeds.sim_seed,
            "accepted": False,
            "reject_reason": "",
        }
        draws: dict[str, Any] = {}
        try:
            draws = sample_run(params, base_full, seeds, spec.correlations or None)
            from ..config import apply_overrides

            cfg = config_from_dict(apply_overrides(base_full, draws), base_dir=base_dir)
            rec = Simulation(cfg, seed=seeds.sim_seed, simulation_id=sid).run()
            problems = check_record(rec, limits)
            row.update(
                {
                    "fidelity": rec.meta.fidelity,
                    "fast": rec.meta.fast,
                    "dt": rec.meta.dt,
                    "n_steps": rec.meta.n_steps,
                    "wall_time_s": rec.meta.wall_time_s,
                    "config_hash": rec.meta.config_hash,
                    "aero_provenance_kind": rec.meta.aero_provenance.get("kind", ""),
                    "aero_model": rec.meta.aero_provenance.get("model", ""),
                    "controller_state_source": rec.meta.controller_state_source,
                }
            )
            row.update(_flatten("res.", rec.summary))
            if not problems and section.kind != "telemetry":
                if section.kind == "tabular":
                    tab = build_tabular(rec, section)
                    block = {"simulation_id": np.full(len(tab.t), sid), "t": tab.t}
                    for j, nm in enumerate(tab.input_names):
                        block[nm] = tab.inputs[:, j].astype(np.float32)
                    for j, nm in enumerate(tab.target_names):
                        block[nm] = tab.targets[:, j].astype(np.float32)
                    tables.append(pa.table(block))
                    row["n_samples"] = len(tab.t)
                    n_samples += len(tab.t)
                else:
                    win = build_windows(rec, section)
                    npz_x.append(win.x)
                    npz_y.append(win.y)
                    npz_t.append(win.t_end)
                    npz_ref.append(np.full(len(win.t_end), len(npz_ids), dtype=np.int32))
                    npz_ids.append(sid)
                    row["n_samples"] = len(win.t_end)
                    n_samples += len(win.t_end)
            elif not problems:
                tables.append(_arrow_table(rec, {"simulation_id": np.full(rec.n_rows, sid)}))
                row["n_samples"] = rec.n_rows
                n_samples += rec.n_rows
            if problems:
                row["reject_reason"] = "; ".join(problems)
        except RocketSimError as exc:
            row["reject_reason"] = f"{type(exc).__name__}: {exc}"
        except Exception as exc:
            row["reject_reason"] = f"unexpected {type(exc).__name__}: {exc}"
        row["accepted"] = row["reject_reason"] == ""
        for k, v in draws.items():
            row[f"param.{k}"] = v
        rows.append(row)
        if not row["accepted"]:
            rejected.append(
                {
                    "simulation_id": sid,
                    "random_seed": seeds.sim_seed,
                    "split": split,
                    "reason": row["reject_reason"],
                    "params": draws,
                }
            )

    shard_rel = ""
    if n_samples:
        shard_dir = out_dir / "shards" / split
        shard_dir.mkdir(parents=True, exist_ok=True)
        if section.kind == "windowed":
            shard_rel = f"shards/{split}/{chunk_id}.npz"
            tmp = shard_dir / f"{chunk_id}.tmp.npz"
            np.savez_compressed(
                tmp,
                x=np.concatenate(npz_x),
                y=np.concatenate(npz_y),
                t_end=np.concatenate(npz_t),
                run_ref=np.concatenate(npz_ref),
                simulation_ids=np.array(npz_ids),
            )
            os.replace(tmp, out_dir / shard_rel)
        else:
            shard_rel = f"shards/{split}/{chunk_id}.parquet"
            tmp = shard_dir / f"{chunk_id}.tmp.parquet"
            with pq.ParquetWriter(tmp, tables[0].schema, compression="zstd") as w:
                for tb in tables:  # one row group per run: id lookups skip unrelated row groups
                    w.write_table(tb)
            os.replace(tmp, out_dir / shard_rel)

    result = {
        "chunk_id": chunk_id,
        "split": split,
        "shard": shard_rel,
        "n_samples": n_samples,
        "rows": rows,
        "rejected": rejected,
        "wall_s": time.perf_counter() - t0,
    }
    (out_dir / "chunks").mkdir(parents=True, exist_ok=True)
    _atomic_write_text(out_dir / "chunks" / f"{chunk_id}.json", json.dumps(result, default=_json_default))
    return result


def _json_default(o: Any) -> Any:
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def _plan_chunks(spec: BatchSpec) -> list[dict[str, Any]]:
    chunks = []
    for split, sp in spec.split_table().items():
        for k, start in enumerate(range(0, sp.runs, spec.shard_runs)):
            chunks.append(
                {
                    "chunk_id": f"{split}-{k:05d}",
                    "split": split,
                    "namespace": sp.namespace,
                    "start": start,
                    "stop": min(start + spec.shard_runs, sp.runs),
                }
            )
    return chunks


def run_batch(
    spec: BatchSpec,
    spec_dir: str | Path = ".",
    output_dir: str | Path | None = None,
    workers: int | None = None,
    resume: bool = True,
    progress: Progress | None = None,
) -> Path:
    """Run (or resume) a batch/dataset generation. Returns the path of ``manifest.json``."""
    spec.validate()
    spec_dir = Path(spec_dir)
    out = Path(output_dir or spec.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    base_full, base_dir = _base_dict(spec, spec_dir)
    h = spec_hash(spec, base_full)
    state_path = out / "state.json"
    if state_path.exists():
        prev = json.loads(state_path.read_text(encoding="utf-8"))
        if prev.get("spec_hash") != h:
            raise ConfigError(
                f"{out} holds output of a different spec (hash {prev.get('spec_hash')} != {h}); "
                "use a new output directory"
            )
        if not resume:
            raise ConfigError(f"{out} already contains results; pass resume=True or choose a new directory")
    else:
        _atomic_write_text(state_path, json.dumps({"spec_hash": h, "created_utc": _now()}))

    splits = spec.split_table()
    if sum(s.runs for s in splits.values()) <= 200_000:  # O(N) check; skipped for huge datasets
        check_seed_disjointness(spec.master_seed, {n: (s.ns, s.runs) for n, s in splits.items()})

    plan = _plan_chunks(spec)
    (out / "chunks").mkdir(exist_ok=True)
    todo = []
    for c in plan:
        marker = out / "chunks" / f"{c['chunk_id']}.json"
        if marker.exists():
            try:
                d = json.loads(marker.read_text(encoding="utf-8"))
                if not d["shard"] or (out / d["shard"]).exists():
                    continue
            except (json.JSONDecodeError, KeyError):
                pass
        todo.append(c)

    n_workers = workers if workers is not None else spec.workers
    if n_workers == 0:
        n_workers = max(1, (os.cpu_count() or 2) - 1)
    n_workers = min(n_workers, max(len(todo), 1))
    tasks = [
        {
            **c,
            "spec": asdict(spec),
            "out_dir": str(out),
            "base_full": base_full,
            "base_dir": str(base_dir),
        }
        for c in todo
    ]

    done = len(plan) - len(todo)
    acc = rej = 0
    if progress:
        progress(done, len(plan), acc, rej)
    if n_workers <= 1:
        for t in tasks:
            r = execute_chunk(t)
            done += 1
            acc += sum(1 for x in r["rows"] if x["accepted"])
            rej += len(r["rejected"])
            if progress:
                progress(done, len(plan), acc, rej)
    else:
        with cf.ProcessPoolExecutor(max_workers=n_workers) as ex:
            for r in ex.map(execute_chunk, tasks):
                done += 1
                acc += sum(1 for x in r["rows"] if x["accepted"])
                rej += len(r["rejected"])
                if progress:
                    progress(done, len(plan), acc, rej)
    return _finalize(spec, out, plan, base_full, h, n_workers)


def _rows_to_table(rows: list[dict[str, Any]]) -> pa.Table:
    """List of heterogeneous row dicts -> Arrow table (union of keys, nulls for missing)."""
    keys: list[str] = []
    seen: set[str] = set()
    for r in rows:
        for k in r:
            if k not in seen:
                seen.add(k)
                keys.append(k)
    cols = {}
    for k in keys:
        vals = [r.get(k) for r in rows]
        try:
            cols[k] = pa.array(vals)
        except (pa.ArrowInvalid, pa.ArrowTypeError):
            cols[k] = pa.array([None if v is None else str(v) for v in vals])
    return pa.table(cols)


def read_runs(path: str | Path) -> pa.Table:
    """Per-run table of a dataset directory (cheap: no telemetry is read)."""
    p = Path(path)
    return pq.read_table(p / "runs.parquet" if p.is_dir() else p)


def _now() -> str:
    return _dt.datetime.now(_dt.UTC).isoformat(timespec="seconds")


def _finalize(
    spec: BatchSpec, out: Path, plan: list[dict[str, Any]], base_full: dict[str, Any], h: str, n_workers: int
) -> Path:
    rows: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    shards: list[dict[str, Any]] = []
    wall = 0.0
    for c in plan:
        d = json.loads((out / "chunks" / f"{c['chunk_id']}.json").read_text(encoding="utf-8"))
        rows.extend(d["rows"])
        rejected.extend(d["rejected"])
        wall += d["wall_s"]
        if d["shard"]:
            p = out / d["shard"]
            shards.append(
                {
                    "path": d["shard"],
                    "split": d["split"],
                    "n_samples": d["n_samples"],
                    "sha256": _sha256(p),
                    "bytes": p.stat().st_size,
                }
            )
    dataset_id = f"{spec.name}-{h}"
    for r in rows:
        r["sim_version"], r["physics_version"], r["schema_version"] = (
            SIM_VERSION,
            PHYSICS_VERSION,
            SCHEMA_VERSION,
        )
        r["dataset_id"], r["dataset_version"] = dataset_id, DATASET_VERSION  # per-row traceability
    pq.write_table(_rows_to_table(rows), out / "runs.parquet", compression="zstd")
    with (out / "rejected.jsonl").open("w", encoding="utf-8") as f:
        for r in rejected:
            f.write(json.dumps(r, default=_json_default) + "\n")

    # leakage report: identical parameter vectors appearing in more than one split
    pkeys = sorted({k for r in rows for k in r if k.startswith("param.")})
    leakage: dict[str, Any] = {"duplicate_parameter_vectors_across_splits": 0}
    if pkeys and len({r["split"] for r in rows}) > 1:
        seen: dict[tuple, set[str]] = {}
        for r in rows:
            key = tuple(round(v, 12) if isinstance(v, float) else v for v in (r.get(k) for k in pkeys))
            seen.setdefault(key, set()).add(r["split"])
        leakage["duplicate_parameter_vectors_across_splits"] = sum(1 for v in seen.values() if len(v) > 1)
    from .leakage import (
        check_run_disjoint,
        dataset_statistics,
        near_duplicate_report,
        parameter_distribution_checks,
        statistics_markdown,
    )

    leakage["near_duplicates"] = near_duplicate_report(rows)
    # a split whose own parameter list is shorter than the global one makes the other parameters NOMINAL there
    split_warnings: list[str] = []
    glob = {p.path for p in spec.parameters}
    for sname, sp in spec.split_table().items():
        if sp.parameters is not None and glob - {p.path for p in sp.parameters}:
            dropped = sorted(glob - {p.path for p in sp.parameters})
            split_warnings.append(
                f"SPLIT '{sname}' replaces the global parameter list and does not randomise {len(dropped)} global "
                f"parameter(s) ({', '.join(dropped[:4])}{'...' if len(dropped) > 4 else ''}): they stay nominal there, "
                "which confounds a distribution-shift comparison"
            )
    leakage["run_disjointness"] = check_run_disjoint(rows)
    leakage["input_resampling"] = "zero-order hold for every input column (no value from after the grid time)"
    splits = spec.split_table()
    seed_info = {
        "master_seed": spec.master_seed,
        "derivation": "SeedSequence([master_seed, split_namespace, run_index]); param_rng = spawn(1)[0]; "
        "sim_seed = generate_state(1, uint64)[0] >> 1",
        "splits": {
            n: {
                "namespace": s.namespace,
                "runs": s.runs,
                "first_sim_seed": derive_seeds(spec.master_seed, s.ns, 0).sim_seed,
                "last_sim_seed": derive_seeds(spec.master_seed, s.ns, s.runs - 1).sim_seed,
            }
            for n, s in splits.items()
        },
    }
    reasons: dict[str, int] = {}
    for r in rows:
        if not r["accepted"]:
            key = r["reject_reason"].split(";")[0][:80]
            reasons[key] = reasons.get(key, 0) + 1
    alarms: list[str] = list(split_warnings)
    n_rej = sum(reasons.values())
    if rows and n_rej / len(rows) > 0.05:
        alarms.append(
            f"HIGH REJECTION RATE: {n_rej}/{len(rows)} runs rejected ({100 * n_rej / len(rows):.0f} %); "
            "the accepted set may be biased - inspect rejection_reasons"
        )
    ds = spec.dataset
    fid = int(base_full["fidelity"])
    stats = dataset_statistics(rows)
    base_dict_for_checks = base_full
    dist_checks = parameter_distribution_checks(
        [r for r in rows if r["split"] == "train"] or rows, spec.parameters, base_dict_for_checks
    )
    for c in dist_checks:
        if c["flag"]:
            alarms.append(
                f"SAMPLER CHECK: {c['parameter']} deviates from its declared {c['dist']} distribution (KS p={c['ks_p_value']:.2g})"
            )
    nd = leakage["near_duplicates"].get("splits", {})
    for sname, v in nd.items():
        if v["n_near_duplicates"]:
            alarms.append(
                f"LEAKAGE: {v['n_near_duplicates']} run(s) in split '{sname}' are near-duplicates of training runs"
            )
    (out / "dataset_report.md").write_text(
        statistics_markdown(stats, leakage, dist_checks, reasons), encoding="utf-8"
    )
    est_on = (base_full["estimator"]["type"] == "truth" and fid >= 3) or (
        base_full["estimator"]["type"] == "nav_kf" and fid >= 5
    )
    n_fins = int((base_full.get("rocket", {}).get("control_surfaces") or {}).get("count", 0))
    manifest: dict[str, Any] = {
        "dataset_id": dataset_id,
        "dataset_version": DATASET_VERSION,
        "name": spec.name,
        "created_utc": _now(),
        "uuid": str(uuid.uuid4()),
        "kind": ds.kind,
        "versions": {
            "simulator": SIM_VERSION,
            "physics": PHYSICS_VERSION,
            "schema": SCHEMA_VERSION,
            "config": CONFIG_VERSION,
            "dataset": DATASET_VERSION,
        },
        "fidelity_level": fid,
        "fast_mode": bool(base_full.get("fast", False)),
        "base_config": base_full,
        "base_config_hash": config_hash(base_full),
        "spec": asdict(spec),
        "spec_hash": h,
        "counts": {
            "simulations_requested": int(sum(s.runs for s in splits.values())),
            "simulations_accepted": sum(1 for r in rows if r["accepted"]),
            "simulations_rejected": sum(1 for r in rows if not r["accepted"]),
            "samples": int(sum(s["n_samples"] for s in shards)),
            "per_split": {
                n: {
                    "requested": s.runs,
                    "accepted": sum(1 for r in rows if r["split"] == n and r["accepted"]),
                }
                for n, s in splits.items()
            },
        },
        "seeds": seed_info,
        "leakage_checks": leakage,
        "statistics": stats,
        "sampler_checks": dist_checks,
        "statistics_report": "dataset_report.md",
        "rejection_reasons": reasons,
        "warnings": alarms,
        "feature_schema": [
            {"name": f.label, "column": f.column, "derived": f.derived, "lag": f.lag} for f in ds.inputs
        ],
        "label_schema": [{"name": f.label, "column": f.column} for f in ds.targets],
        "window": asdict(ds.window) if ds.kind == "windowed" else None,
        "sample_dt_s": ds.sample_dt_s,
        "trim": ds.trim,
        "telemetry_columns": [c.name for c in columns_for(fid, est_on, n_fins)]
        if ds.kind == "telemetry"
        else None,
        "telemetry_schema": schema_dict() if ds.kind == "telemetry" else None,
        "quality": asdict(spec.quality),
        "files": shards,
        "runs_table": "runs.parquet",
        "rejected": "rejected.jsonl",
        "performance": {"workers": n_workers, "sum_chunk_wall_s": wall},
    }
    mp = out / "manifest.json"
    _atomic_write_text(mp, json.dumps(manifest, indent=2, default=_json_default))
    return mp


def read_manifest(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    if p.is_dir():
        p = p / "manifest.json"
    return json.loads(p.read_text(encoding="utf-8"))
