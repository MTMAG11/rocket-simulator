"""Q-01: compare motors for a TVC vehicle. Fill in a study file (see studies/q01_template.yaml), then:

    python scripts/q01_trade_study.py --study studies/q01.yaml --out validation_results/q01

Writes report.md and two figures. Definitions and caveats: docs/derivations/tvc_authority.md.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rocket_sim.errors import RocketSimError
from rocket_sim.studies.report import write_report
from rocket_sim.studies.tvc_trade import load_study, run_study


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--study", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--runs", type=int, help="override monte_carlo.runs")
    a = ap.parse_args()
    try:
        spec = load_study(a.study)
        if a.runs:
            spec.monte_carlo["runs"] = a.runs
        results = run_study(spec, progress=lambda i, n: print(f"  run {i}/{n}", end="\r", flush=True))
    except RocketSimError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    path = write_report(spec, results, a.out)
    print(f"\nwrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
