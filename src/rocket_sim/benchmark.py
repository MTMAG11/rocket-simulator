"""Throughput benchmarks: single simulation, 100 / 1000 simulations, dataset generation.

Reports simulations/second, timesteps/second and peak memory. Results depend on the machine;
the numbers recorded in docs/performance.md name the hardware they were measured on.
"""

from __future__ import annotations

import json
import os
import platform
import shutil
import tempfile
import time
import tracemalloc
from pathlib import Path
from typing import Any

from .config import load_config
from .config.loader import PROJECT_ROOT
from .data.batch import run_batch
from .data.spec import BatchSpec, DatasetSection, FeatureSpec, WindowSpec
from .simulation import run_simulation

_EXAMPLE = PROJECT_ROOT / "configs" / "example_g80.yaml"


def _rss_mb() -> float | None:
    try:
        import psutil

        return psutil.Process().memory_info().rss / 1e6
    except ImportError:
        return None


def _spec(n: int, kind: str, fidelity: int, fast: bool, workers: int, out: Path) -> BatchSpec:
    from .config import config_to_dict
    from .data.montecarlo import ParamSpec

    cfg = load_config(_EXAMPLE)
    cfg.fidelity = fidelity
    cfg.fast = fast
    cfg.simulation.dt_s = 0.02 if fast else 0.01
    cfg.simulation.descent_dt_s = 0.1 if fidelity < 3 else 0.05
    ds = DatasetSection(kind="telemetry")
    if kind == "windowed":
        ds = DatasetSection(
            kind="windowed",
            sample_dt_s=0.05,
            inputs=[FeatureSpec(column="pos_z"), FeatureSpec(column="vel_z"), FeatureSpec(column="acc_z")],
            targets=[FeatureSpec(column="pos_z")],
            window=WindowSpec(history=20, stride=4, target="future", horizon_s=0.5),
        )
    return BatchSpec(
        name="bench",
        config=config_to_dict(cfg),
        master_seed=1,
        runs=n,
        shard_runs=max(1, min(50, n // 4 or 1)),
        workers=workers,
        parameters=[
            ParamSpec(path="rocket.dry_mass_kg", dist="normal", rel_std=0.03),
            ParamSpec(path="environment.wind.speed_ms", dist="uniform", low=0.0, high=8.0),
            ParamSpec(path="environment.wind.model", dist="choice", values=["constant"]),
        ],
        dataset=ds,
        output_dir=str(out),
    )


def run_benchmarks(quick: bool = False, workers: int = 0) -> str:
    results: dict[str, Any] = {
        "machine": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "cpu_count": os.cpu_count(),
        },
    }
    lines = [
        f"machine: {results['machine']['platform']}, {results['machine']['cpu_count']} logical cores, "
        f"python {results['machine']['python']}",
        "",
    ]
    cfg = load_config(_EXAMPLE)

    def single(label: str, fid: int, fast: bool, dt: float, ddt: float) -> None:
        c = load_config(_EXAMPLE)
        c.fidelity, c.fast = fid, fast
        c.simulation.dt_s, c.simulation.descent_dt_s = dt, ddt
        run_simulation(c, seed=0)  # warm-up (imports, caches)
        t0 = time.perf_counter()
        r = run_simulation(c, seed=0)
        wall = time.perf_counter() - t0
        tracemalloc.start()  # memory pass is separate: tracemalloc slows Python ~5-10x
        run_simulation(c, seed=0)
        _, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        results[label] = {
            "wall_s": wall,
            "steps": r.meta.n_steps,
            "steps_per_s": r.meta.n_steps / wall,
            "peak_alloc_mb": peak / 1e6,
        }
        lines.append(
            f"{label:<34} {wall * 1000:8.0f} ms  {r.meta.n_steps:6d} steps  "
            f"{r.meta.n_steps / wall:9.0f} steps/s  peak alloc {peak / 1e6:5.1f} MB"
        )

    single("single: L3 6-DOF, dt=0.01", 3, False, 0.01, 0.05)
    single("single: L3 6-DOF, dt=0.001 (ref)", 3, False, 0.001, 0.05)
    single("single: L2 3-DOF, dt=0.01", 2, False, 0.01, 0.1)
    single("single: L2 FAST, dt=0.02", 2, True, 0.02, 0.1)
    del cfg

    n_list = [20, 100] if quick else [100, 1000]
    for n in n_list:
        for fid, fast, label in ((3, False, "L3 6-DOF"), (2, True, "L2 FAST")):
            out = Path(tempfile.mkdtemp(prefix="rsbench_"))
            try:
                t0 = time.perf_counter()
                run_batch(_spec(n, "telemetry", fid, fast, workers, out), PROJECT_ROOT, out, workers=workers)
                wall = time.perf_counter() - t0
            finally:
                shutil.rmtree(out, ignore_errors=True)
            results[f"batch {n} {label}"] = {"wall_s": wall, "sims_per_s": n / wall}
            lines.append(
                f"batch {n:>5} sims, {label:<9} workers={workers or 'auto'}  {wall:7.1f} s  {n / wall:7.1f} sims/s"
            )
    nd = 100 if quick else 1000
    out = Path(tempfile.mkdtemp(prefix="rsbench_"))
    try:
        t0 = time.perf_counter()
        run_batch(_spec(nd, "windowed", 2, True, workers, out), PROJECT_ROOT, out, workers=workers)
        wall = time.perf_counter() - t0
        size = sum(f.stat().st_size for f in out.rglob("*") if f.is_file())
    finally:
        shutil.rmtree(out, ignore_errors=True)
    results["dataset windowed"] = {"wall_s": wall, "sims_per_s": nd / wall, "bytes": size}
    lines.append(
        f"dataset (windowed, L2 FAST) {nd:>5} sims  {wall:7.1f} s  {nd / wall:7.1f} sims/s  {size / 1e6:.1f} MB on disk"
    )
    rss = _rss_mb()
    if rss is not None:
        lines.append(f"\nprocess RSS at end: {rss:.0f} MB")
    out_dir = PROJECT_ROOT / "output" / "benchmarks"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "latest.json").write_text(json.dumps({**results, "text": lines}, indent=2), encoding="utf-8")
    return "\n".join(lines)
