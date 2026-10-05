"""Command-line interface: ``rocketsim <command>``.

simulate CONFIG                 run one flight, print a summary, optionally export telemetry
batch SPEC [--runs N]           Monte-Carlo batch (parallel, checkpointed, resumable)
generate-dataset SPEC           same engine, ML dataset kinds (tabular / windowed)
validate FLIGHT.yaml            compare the simulator against real telemetry
export SOURCE                   convert a saved record / pull one run out of a dataset
inspect DATASET [--run ID]      browse a dataset's manifest/runs without loading telemetry
benchmark                       throughput of single runs, small batches, dataset generation
check CONFIG                    validate a configuration file without simulating
schema [--json]                 print the versioned telemetry schema
motors                          list bundled motor files
gui                             start the graphical interface
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

from . import __version__
from .errors import RocketSimError


def _parse_set(items: list[str] | None) -> dict[str, Any]:
    out: dict[str, Any] = {}
    import yaml

    for it in items or []:
        if "=" not in it:
            raise RocketSimError(f"--set expects key=value, got {it!r}")
        k, v = it.split("=", 1)
        out[k.strip()] = yaml.safe_load(v)
    return out


def cmd_simulate(a: argparse.Namespace) -> int:
    from .config import load_config
    from .data.export import export_record
    from .reporting import summary_text
    from .simulation import run_simulation

    ov = _parse_set(a.set)
    if a.fidelity is not None:
        ov["fidelity"] = a.fidelity
    if a.dt is not None:
        ov["simulation.dt_s"] = a.dt
    cfg = load_config(a.config, overrides=ov or None)
    rec = run_simulation(cfg, seed=a.seed)
    print(summary_text(rec))
    if a.out:
        fmts = tuple(f.strip() for f in a.format.split(","))
        for p in export_record(rec, a.out, a.name, fmts):
            print(f"wrote {p}")
    if a.plot:
        from .plotting import plot_overview

        fig = plot_overview(rec)
        fig.savefig(a.plot, dpi=130)
        print(f"wrote {a.plot}")
    return 0 if rec.meta.status == "ok" else 2


def _progress(done: int, total: int, acc: int, rej: int) -> None:
    sys.stderr.write(f"\r  chunks {done}/{total}  accepted {acc}  rejected {rej}   ")
    sys.stderr.flush()
    if done == total:
        sys.stderr.write("\n")


def cmd_batch(a: argparse.Namespace) -> int:
    from .data.batch import load_spec, read_manifest, run_batch

    spec, d = load_spec(a.spec, validate=False)
    if a.runs is not None:
        if spec.splits:
            tot = sum(s.runs for s in spec.splits.values())
            for s in spec.splits.values():
                s.runs = max(1, round(s.runs * a.runs / tot))
        else:
            spec.runs = a.runs
    if a.out:
        spec.output_dir = a.out
    t0 = time.perf_counter()
    mp = run_batch(spec, d, workers=a.workers, resume=not a.no_resume, progress=_progress)
    m = read_manifest(mp)
    c = m["counts"]
    dt = time.perf_counter() - t0
    print(
        f"dataset {m['dataset_id']}: {c['simulations_accepted']}/{c['simulations_requested']} accepted, "
        f"{c['simulations_rejected']} rejected, {c['samples']} samples, {dt:.1f} s "
        f"({c['simulations_requested'] / dt:.1f} sims/s)"
    )
    for w in m.get("warnings", []):
        print(f"WARNING: {w}")
    print(f"manifest: {mp}")
    return 0


def cmd_validate(a: argparse.Namespace) -> int:
    from .validation.compare import run_validation

    res = run_validation(a.flight, out_dir=a.out, plot=not a.no_plot)
    print(res.report_text())
    return 0


def cmd_export(a: argparse.Namespace) -> int:
    from .data.export import (
        export_record,
        read_record_csv,
        read_record_npz,
        read_record_parquet,
    )

    src = Path(a.source)
    if src.is_dir():
        from .data.browser import load_run

        rec = load_run(src, a.run)
    elif src.suffix == ".parquet":
        rec = read_record_parquet(src)
    elif src.suffix == ".npz":
        rec = read_record_npz(src)
    elif src.suffix == ".csv":
        rec = read_record_csv(src)
    else:
        raise RocketSimError(f"unsupported source {src}")
    for p in export_record(rec, a.out, a.name, tuple(a.format.split(","))):
        print(f"wrote {p}")
    return 0


def cmd_inspect(a: argparse.Namespace) -> int:
    from .data.browser import describe_dataset, describe_run

    print(describe_run(a.dataset, a.run) if a.run else describe_dataset(a.dataset, a.limit))
    return 0


def cmd_benchmark(a: argparse.Namespace) -> int:
    from .benchmark import run_benchmarks

    print(run_benchmarks(quick=a.quick, workers=a.workers))
    return 0


def cmd_check(a: argparse.Namespace) -> int:
    from .config import load_config
    from .simulation import Simulation

    cfg = load_config(a.config)
    sim = Simulation(cfg)
    print(
        f"{a.config}: OK (fidelity {cfg.fidelity}, motor {sim.motor.designation}, "
        f"liftoff mass {sim.vehicle.liftoff_mass:.3f} kg, static margin {sim.vehicle.static_margin(0.0):.2f} cal)"
    )
    for w in sim.warnings:
        print(f"  WARNING: {w}")
    return 0


def cmd_schema(a: argparse.Namespace) -> int:
    from .data.schema import schema_dict, schema_markdown

    print(json.dumps(schema_dict(), indent=2) if a.json else schema_markdown())
    return 0


def cmd_motors(a: argparse.Namespace) -> int:
    from .config.loader import PROJECT_ROOT
    from .motor import available_motors, load_motor

    for p in available_motors(Path(a.dir) if a.dir else PROJECT_ROOT / "data" / "motors"):
        m = load_motor(p)
        print(
            f"{p.name:34s} {m.designation:8s} class {m.impulse_class}  I={m.total_impulse:8.1f} Ns  "
            f"burn={m.burn_time:5.2f} s  Tpeak={m.max_thrust:8.1f} N  mp={m.propellant_mass * 1000:7.1f} g"
        )
    return 0


def cmd_gui(a: argparse.Namespace) -> int:
    from .ui.main_window import main as gui_main

    gui_main()
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="rocketsim", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--version", action="version", version=f"rocketsim {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("simulate", help="run one flight")
    s.add_argument("config")
    s.add_argument("--seed", type=int)
    s.add_argument("--fidelity", type=int, choices=range(0, 7))
    s.add_argument("--dt", type=float, help="override simulation.dt_s")
    s.add_argument(
        "--set", action="append", metavar="KEY=VALUE", help="override a config value (dotted path)"
    )
    s.add_argument("--out", help="directory for exported telemetry")
    s.add_argument("--name", help="file stem for exports")
    s.add_argument("--format", default="csv", help="comma list of csv,json,npz,parquet")
    s.add_argument("--plot", metavar="PNG", help="save an overview plot")
    s.set_defaults(fn=cmd_simulate)

    b = sub.add_parser("batch", aliases=["generate-dataset"], help="Monte-Carlo batch / dataset generation")
    b.add_argument("spec")
    b.add_argument("--runs", type=int, help="total number of runs (scales split sizes proportionally)")
    b.add_argument("--workers", type=int, help="processes (0 = all cores - 1, 1 = serial)")
    b.add_argument("--out", help="output directory")
    b.add_argument("--no-resume", action="store_true", help="refuse to continue an existing output directory")
    b.set_defaults(fn=cmd_batch)

    v = sub.add_parser("validate", help="compare against real flight telemetry")
    v.add_argument("flight", help="flight definition YAML (see validation_data/)")
    v.add_argument("--out", help="output directory for report/plots")
    v.add_argument("--no-plot", action="store_true")
    v.set_defaults(fn=cmd_validate)

    e = sub.add_parser("export", help="convert a saved record or extract a run from a dataset")
    e.add_argument("source")
    e.add_argument("--run", help="simulation_id (when source is a dataset directory)")
    e.add_argument("--out", default=".")
    e.add_argument("--name")
    e.add_argument("--format", default="csv")
    e.set_defaults(fn=cmd_export)

    i = sub.add_parser("inspect", help="browse a dataset")
    i.add_argument("dataset")
    i.add_argument("--run")
    i.add_argument("--limit", type=int, default=15)
    i.set_defaults(fn=cmd_inspect)

    k = sub.add_parser("benchmark", help="measure simulation throughput")
    k.add_argument("--quick", action="store_true")
    k.add_argument("--workers", type=int, default=0)
    k.set_defaults(fn=cmd_benchmark)

    c = sub.add_parser("check", help="validate a config file")
    c.add_argument("config")
    c.set_defaults(fn=cmd_check)

    sc = sub.add_parser("schema", help="print the telemetry schema")
    sc.add_argument("--json", action="store_true")
    sc.set_defaults(fn=cmd_schema)

    m = sub.add_parser("motors", help="list motor files")
    m.add_argument("--dir")
    m.set_defaults(fn=cmd_motors)

    g = sub.add_parser("gui", help="start the GUI")
    g.set_defaults(fn=cmd_gui)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.fn(args))
    except RocketSimError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("interrupted (completed batch chunks are checkpointed; rerun to resume)", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
