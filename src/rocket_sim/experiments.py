"""Versioned experiment runner.

An *experiment* is a dataset-generation (batch) run wrapped with everything needed to reproduce and audit it later:

    experiments/<experiment_id>/
        experiment.json      id, date, git commit (+dirty flag), versions, seed, spec hash, base-config hash,
                             environment, command, outputs with SHA-256, summary counts
        spec.yaml            the batch spec exactly as run (after the seed override)
        dataset/             manifest.json, runs.parquet, shards, dataset_report.md
    experiments/index.jsonl  one line per experiment (append-only)

``experiment_id = <name>-<spec_hash[:8]>-<UTC timestamp>``. The ``spec_hash`` covers the full resolved base
configuration, the sampled-parameter distributions, correlations, seeds and dataset section, so two experiments with
the same hash generate bit-identical data (the simulator is deterministic); ``verify_experiment`` re-runs an experiment
into a scratch directory and compares every shard's SHA-256 to prove it.

Experiment file (YAML)::

    name: baseline-wind
    description: wind-robustness study
    batch: configs/batch_example.yaml      # path relative to this file
    master_seed: 7                         # optional override
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import tempfile
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .config import read_mapping
from .config.loader import PROJECT_ROOT
from .data.batch import _base_dict, load_spec, read_manifest, run_batch, spec_hash
from .errors import ConfigError
from .version import CONFIG_VERSION, DATASET_VERSION, PHYSICS_VERSION, SCHEMA_VERSION, SIM_VERSION


def _git(*args: str) -> str | None:
    try:
        r = subprocess.run(["git", *args], cwd=PROJECT_ROOT, capture_output=True, text=True, timeout=20)
        return r.stdout.strip() if r.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def git_state() -> dict[str, Any]:
    commit = _git("rev-parse", "HEAD")
    status = _git("status", "--porcelain")
    return {"commit": commit, "dirty": bool(status) if status is not None else None}


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _environment() -> dict[str, str]:
    import numpy
    import pyarrow

    return {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "numpy": numpy.__version__,
        "pyarrow": pyarrow.__version__,
    }


def run_experiment(
    experiment_file: str | Path, root: str | Path | None = None, workers: int | None = None
) -> Path:
    """Run an experiment file; returns the experiment directory."""
    ef = Path(experiment_file)
    d = read_mapping(ef)
    for k in ("name", "batch"):
        if k not in d:
            raise ConfigError(f"{ef.name}: missing key '{k}'")
    batch_path = (ef.parent / d["batch"]) if not Path(d["batch"]).is_absolute() else Path(d["batch"])
    spec, spec_dir = load_spec(batch_path)
    if "master_seed" in d:
        spec.master_seed = int(d["master_seed"])
    spec.validate()
    base_full, _ = _base_dict(spec, spec_dir)
    h = spec_hash(spec, base_full)
    stamp = _dt.datetime.now(_dt.UTC).strftime("%Y%m%dT%H%M%SZ")
    exp_id = f"{d['name']}-{h[:8]}-{stamp}"
    out = Path(root or PROJECT_ROOT / "experiments") / exp_id
    ds_dir = out / "dataset"
    ds_dir.mkdir(parents=True, exist_ok=True)
    spec.output_dir = str(ds_dir)
    mp = run_batch(spec, spec_dir, ds_dir, workers=workers)
    man = read_manifest(mp)
    spec_yaml = out / "spec.yaml"
    spec_yaml.write_text(
        json.dumps(asdict(spec), indent=2, default=str), encoding="utf-8"
    )  # JSON is valid YAML
    record = {
        "experiment_id": exp_id,
        "name": d["name"],
        "description": d.get("description", ""),
        "created_utc": _dt.datetime.now(_dt.UTC).isoformat(timespec="seconds"),
        "git": git_state(),
        "versions": {
            "simulator": SIM_VERSION,
            "physics": PHYSICS_VERSION,
            "schema": SCHEMA_VERSION,
            "config": CONFIG_VERSION,
            "dataset": DATASET_VERSION,
        },
        "master_seed": spec.master_seed,
        "spec_hash": h,
        "spec_dir": str(Path(spec_dir).resolve()),
        "dataset_id": man["dataset_id"],
        "base_config_hash": man["base_config_hash"],
        "environment": _environment(),
        "command": " ".join(sys.argv),
        "counts": man["counts"],
        "warnings": man["warnings"],
        "outputs": {
            "manifest": {"path": "dataset/manifest.json", "sha256": _sha256(mp)},
            "shards": {f["path"]: f["sha256"] for f in man["files"]},
        },
    }
    (out / "experiment.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    with (Path(root or PROJECT_ROOT / "experiments") / "index.jsonl").open("a", encoding="utf-8") as fh:
        fh.write(
            json.dumps(
                {
                    k: record[k]
                    for k in ("experiment_id", "name", "created_utc", "spec_hash", "master_seed", "counts")
                }
            )
            + "\n"
        )
    return out


def verify_experiment(exp_dir: str | Path, workers: int | None = 1) -> dict[str, Any]:
    """Re-generate the experiment's dataset in a scratch directory and compare every shard hash."""
    ed = Path(exp_dir)
    rec = json.loads((ed / "experiment.json").read_text(encoding="utf-8"))
    from .config import from_dict
    from .data.spec import BatchSpec

    spec = from_dict(BatchSpec, json.loads((ed / "spec.yaml").read_text(encoding="utf-8")))
    spec.validate()
    scratch = Path(tempfile.mkdtemp(prefix="rsverify_"))
    try:
        run_batch(spec, Path(rec["spec_dir"]), scratch, workers=workers)
        man = read_manifest(scratch)
        new = {f["path"]: f["sha256"] for f in man["files"]}
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
    old = rec["outputs"]["shards"]
    mismatched = sorted(k for k in set(old) | set(new) if old.get(k) != new.get(k))
    return {
        "experiment_id": rec["experiment_id"],
        "shards": len(old),
        "identical": bool(old) and not mismatched,  # an experiment with no shards verifies nothing
        "mismatched": mismatched,
    }


def list_experiments(root: str | Path | None = None) -> list[dict[str, Any]]:
    idx = Path(root or PROJECT_ROOT / "experiments") / "index.jsonl"
    if not idx.exists():
        return []
    return [json.loads(line) for line in idx.read_text(encoding="utf-8").splitlines() if line.strip()]
