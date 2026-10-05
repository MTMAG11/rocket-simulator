"""Inspect datasets without loading telemetry: manifest + per-run table + single-run access.

``describe_dataset``  headline manifest facts and a table of runs (reads only runs.parquet).
``describe_run``      configuration draws and results of one run.
``load_telemetry``    read ONE run's rows from the shards (row-group statistics skip other runs).
``reproduce_run``     regenerate a run from (base config, parameter draws, seed): the proof that
                      every simulation in a dataset is reproducible without its stored telemetry.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.compute as pc
import pyarrow.parquet as pq

from ..config import apply_overrides, config_from_dict
from ..errors import RocketSimError
from ..simulation import Simulation
from ..simulation.record import FlightRecord
from .batch import read_manifest, read_runs


def _row(runs: pa.Table, sim_id: str) -> dict[str, Any]:
    sel = runs.filter(pc.equal(runs["simulation_id"], sim_id))
    if sel.num_rows == 0:
        raise RocketSimError(f"simulation '{sim_id}' not found in dataset")
    return sel.to_pylist()[0]


def describe_dataset(path: str | Path, limit: int = 15) -> str:
    p = Path(path)
    m = read_manifest(p)
    runs = read_runs(p)
    c = m["counts"]
    lines = [
        f"dataset      {m['dataset_id']}  (kind: {m['kind']})",
        f"created      {m['created_utc']}   simulator {m['versions']['simulator']}  "
        f"physics {m['versions']['physics']}  schema {m['versions']['schema']}",
        f"fidelity     level {m['fidelity_level']}{' (fast mode)' if m['fast_mode'] else ''}",
        f"simulations  {c['simulations_accepted']} accepted / {c['simulations_requested']} requested "
        f"({c['simulations_rejected']} rejected)   samples: {c['samples']}",
    ]
    for n, d in c["per_split"].items():
        lines.append(
            f"  split {n:<6} {d['accepted']}/{d['requested']}   namespace "
            f"{m['seeds']['splits'][n]['namespace']}"
        )
    lines.append(
        f"seeds        master {m['seeds']['master_seed']}; leakage check: "
        f"{m['leakage_checks']['duplicate_parameter_vectors_across_splits']} duplicate parameter vectors across splits"
    )
    if m["feature_schema"]:
        lines.append("inputs       " + ", ".join(f["name"] for f in m["feature_schema"]))
        lines.append("targets      " + ", ".join(f["name"] for f in m["label_schema"]))
    lines.append("")
    lines.append(f"{'simulation_id':<28}{'seed':>20}  {'ok':<3}{'apogee [m]':>11}{'vmax [m/s]':>11}  reason")
    names = runs.column_names
    rows = runs.slice(0, limit).to_pylist()
    for r in rows:
        lines.append(
            f"{r['simulation_id']:<28}{r['random_seed']:>20}  {'Y' if r['accepted'] else 'N':<3}"
            f"{_f(r.get('res.apogee_m')):>11}{_f(r.get('res.max_velocity_ms')):>11}  {r['reject_reason'][:50]}"
        )
    if runs.num_rows > limit:
        lines.append(f"... {runs.num_rows - limit} more (columns: {len(names)})")
    return "\n".join(lines)


def _f(v: Any) -> str:
    return "-" if v is None else f"{v:.1f}"


def describe_run(path: str | Path, sim_id: str) -> str:
    p = Path(path)
    r = _row(read_runs(p), sim_id)
    lines = [
        f"Simulation {sim_id}",
        f"  seed {r['random_seed']}  split {r['split']}  run_index {r['run_index']}",
        f"  accepted: {r['accepted']}  {r['reject_reason']}",
        "",
        "Configuration draws:",
    ]
    for k in sorted(k for k in r if k.startswith("param.")):
        v = r[k]
        lines.append(f"  {k[6:]:<45} {v:.6g}" if isinstance(v, float) else f"  {k[6:]:<45} {v}")
    lines += ["", "Results:"]
    for k in sorted(k for k in r if k.startswith("res.")):
        v = r[k]
        lines.append(f"  {k[4:]:<30} {v:.4g}" if isinstance(v, float) else f"  {k[4:]:<30} {v}")
    return "\n".join(lines)


def load_telemetry(path: str | Path, sim_id: str) -> dict[str, np.ndarray]:
    """One run's telemetry columns from a ``telemetry``-kind dataset (no other run is decoded)."""
    p = Path(path)
    m = read_manifest(p)
    if m["kind"] != "telemetry":
        raise RocketSimError(
            f"dataset kind is '{m['kind']}'; per-run telemetry is only stored for 'telemetry'"
        )
    r = _row(read_runs(p), sim_id)
    chunk = f"{r['split']}-{r['run_index'] // m['spec']['shard_runs']:05d}"
    shard = p / "shards" / r["split"] / f"{chunk}.parquet"
    if not shard.exists():
        raise RocketSimError(f"no telemetry shard for {sim_id} (rejected run?)")
    t = pq.read_table(shard, filters=[("simulation_id", "==", sim_id)])
    return {c: t.column(c).to_numpy() for c in t.column_names if c != "simulation_id"}


def reproduce_run(path: str | Path, sim_id: str) -> FlightRecord:
    """Re-simulate a run exactly from the manifest's base config, its draws and its seed."""
    p = Path(path)
    m = read_manifest(p)
    r = _row(read_runs(p), sim_id)
    draws = {k[6:]: v for k, v in r.items() if k.startswith("param.") and v is not None}
    cfg = config_from_dict(apply_overrides(m["base_config"], draws), base_dir=Path.cwd())
    return Simulation(cfg, seed=int(r["random_seed"]), simulation_id=sim_id).run()


def load_run(path: str | Path, sim_id: str | None) -> FlightRecord:
    if not sim_id:
        raise RocketSimError("--run SIMULATION_ID is required when exporting from a dataset")
    return reproduce_run(path, sim_id)
