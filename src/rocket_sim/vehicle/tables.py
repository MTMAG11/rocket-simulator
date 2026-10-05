"""Aerodynamic lookup-table files (CFD / wind-tunnel / other tool exports)."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

from ..errors import ConfigError


def load_table2d_csv(path: str | Path) -> dict[str, Any]:
    """CSV with a header containing mach, alpha_deg, cd, cl and optionally cm (one row per grid point).

    Returns {"mach": [...], "alpha": [rad ...], "cd": [[...]], "cl": [[...]], "cm": [[...]] | None}. The grid must
    be complete (every Mach x alpha combination exactly once)."""
    p = Path(path)
    try:
        with p.open(newline="", encoding="utf-8") as f:
            rd = csv.DictReader(f)
            names = [n.strip().lower() for n in (rd.fieldnames or [])]
            need = {"mach", "alpha_deg", "cd", "cl"}
            if not need <= set(names):
                raise ConfigError(
                    f"{p.name}: header must contain {sorted(need)} (+ optional cm), got {names}"
                )
            rows = [{k.strip().lower(): float(v) for k, v in r.items()} for r in rd]
    except (OSError, ValueError) as exc:
        raise ConfigError(f"cannot read aero table {p}: {exc}") from exc
    machs = sorted({r["mach"] for r in rows})
    alphas = sorted({r["alpha_deg"] for r in rows})
    if len(rows) != len(machs) * len(alphas):
        raise ConfigError(f"{p.name}: table must be a complete mach x alpha grid")
    idx = {(r["mach"], r["alpha_deg"]): r for r in rows}
    if len(idx) != len(rows):
        raise ConfigError(f"{p.name}: duplicate (mach, alpha) rows")
    has_cm = "cm" in names

    def grid(key: str) -> list[list[float]]:
        return [[idx[(m, a)][key] for a in alphas] for m in machs]

    import math

    return {
        "mach": machs,
        "alpha": [math.radians(a) for a in alphas],
        "cd": grid("cd"),
        "cl": grid("cl"),
        "cm": grid("cm") if has_cm else None,
    }
