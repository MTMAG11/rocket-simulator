"""Flight registry, DEVELOPMENT / CALIBRATION / HOLDOUT split and the holdout evaluation log.

The point of this module is to make it hard to fool oneself:

* every flight carries a split and a source tier (``validation_data/registry.yaml``);
* results are always labelled "in-sample" (development), "calibration" or "holdout";
* a holdout flight is only simulated with ``confirm_frozen=True`` and the result is appended to a log together with
  a SHA-256 fingerprint of the physics source files. If the physics source later changes, earlier holdout results are
  reported as STALE (the flight can no longer be treated as clean holdout for the new physics).
"""

from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ..config import read_mapping
from ..errors import ConfigError
from ..version import PHYSICS_VERSION
from .compare import ValidationResult, run_validation

SPLITS = ("development", "calibration", "holdout")
LABEL = {"development": "in-sample (development)", "calibration": "calibration", "holdout": "HOLDOUT"}
# Packages whose source determines simulated results. Documentation, tests, UI and data tooling are excluded.
PHYSICS_PACKAGES = ("physics", "vehicle", "environment", "motor", "simulation", "config", "constants.py")


def physics_fingerprint(package_root: Path | None = None) -> str:
    """SHA-256 over the (sorted) relative paths and the PARSED SYNTAX TREE of every physics-relevant source file.

    Hashing ``ast.dump`` (not the bytes) makes the fingerprint insensitive to formatting and comments, so running a
    code formatter does not invalidate holdout evaluations, while any change to the executable code does."""
    import ast

    root = package_root or Path(__file__).resolve().parents[1]
    files: list[Path] = []
    for name in PHYSICS_PACKAGES:
        p = root / name
        if p.is_dir():
            files += [f for f in p.rglob("*.py")]
        elif p.is_file():
            files.append(p)
    h = hashlib.sha256()
    for f in sorted(files, key=lambda x: x.relative_to(root).as_posix()):
        h.update(f.relative_to(root).as_posix().encode())
        h.update(b"|")
        h.update(ast.dump(ast.parse(f.read_bytes())).encode())
        h.update(b"|")
    return h.hexdigest()


@dataclass
class FlightEntry:
    id: str
    definition: Path
    tier: int
    split: str
    info: dict[str, Any] = field(default_factory=dict)


@dataclass
class Registry:
    path: Path
    flights: list[FlightEntry]
    excluded: list[dict[str, Any]]
    frozen_physics_version: str

    def by_split(self, split: str) -> list[FlightEntry]:
        return [f for f in self.flights if f.split == split]


def load_registry(path: str | Path) -> Registry:
    p = Path(path)
    d = read_mapping(p)
    flights = []
    for f in d.get("flights", []):
        if f["split"] not in SPLITS:
            raise ConfigError(f"{p.name}: flight {f['id']!r} has unknown split {f['split']!r}")
        flights.append(
            FlightEntry(
                f["id"],
                p.parent / f["definition"],
                int(f["tier"]),
                f["split"],
                {k: v for k, v in f.items() if k not in ("id", "definition", "tier", "split")},
            )
        )
    ids = [f.id for f in flights]
    if len(set(ids)) != len(ids):
        raise ConfigError(f"{p.name}: duplicate flight ids")
    return Registry(p, flights, d.get("excluded", []), str(d.get("frozen_physics_version", "")))


def read_log(log_path: Path) -> list[dict[str, Any]]:
    if not log_path.exists():
        return []
    return [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _line_hash(line: str) -> str:
    return hashlib.sha256(line.encode("utf-8")).hexdigest()


def input_fingerprints(entry: FlightEntry) -> dict[str, str]:
    """SHA-256 of everything besides the physics source that determines a flight's reported error: the flight definition,
    its sim config, the telemetry and motor files, and the comparison code."""
    d = read_mapping(entry.definition)
    base = entry.definition.parent
    files = {
        "definition": entry.definition,
        "sim_config": base / d["sim_config"],
        "telemetry": base / d["telemetry"]["file"],
    }
    sim = read_mapping(base / d["sim_config"])
    mf = sim.get("motor", {}).get("file")
    if mf:
        files["motor"] = (base / d["sim_config"]).parent / mf
    pkg = Path(__file__).resolve().parent
    for n in ("compare.py", "metrics.py", "telemetry.py"):
        files[f"code:{n}"] = pkg / n
    out = {}
    for k, p in files.items():
        out[k] = (
            hashlib.sha256(p.read_bytes().replace(b"\r\n", b"\n")).hexdigest() if p.exists() else "missing"
        )
    return out


def append_log(log_path: Path, entry: dict[str, Any]) -> None:
    """Append an entry carrying ``prev`` = SHA-256 of the previous log line (a hash chain: editing or deleting an earlier
    line breaks every later link, which ``verify_log`` reports). Older entries have no ``prev``."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    lines = (
        [x for x in log_path.read_text(encoding="utf-8").splitlines() if x.strip()]
        if log_path.exists()
        else []
    )
    entry = {**entry, "prev": _line_hash(lines[-1]) if lines else "GENESIS"}
    with log_path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(entry) + "\n")


def verify_log(log_path: Path) -> list[str]:
    """Problems found in the hash chain (empty = intact). Legacy entries without ``prev`` are skipped, not trusted."""
    if not log_path.exists():
        return []
    lines = [x for x in log_path.read_text(encoding="utf-8").splitlines() if x.strip()]
    out = []
    for i, line in enumerate(lines):
        e = json.loads(line)
        if "prev" in e and i > 0 and e["prev"] != _line_hash(lines[i - 1]):
            out.append(f"line {i + 1}: chain broken (previous line was altered or removed)")
    return out


def _equivalent(fp: str, log: list[dict[str, Any]]) -> set[str]:
    """Fingerprints declared equivalent to ``fp`` by migration records (evidence-backed, see ``migrate_fingerprint``)."""
    eq = {fp}
    changed = True
    while changed:
        changed = False
        for e in log:
            if e.get("type") == "migration" and e["to_fingerprint"] in eq and e["from_fingerprint"] not in eq:
                eq.add(e["from_fingerprint"])
                changed = True
    return eq


def holdout_status(flight_id: str, log: list[dict[str, Any]], fingerprint: str) -> str:
    """'unevaluated' | 'current' (evaluated with this exact physics, or with physics declared equivalent by a
    verified migration record) | 'stale' (evaluated with older physics)."""
    entries = [e for e in log if e.get("flight") == flight_id and e.get("type") != "migration"]
    if not entries:
        return "unevaluated"
    return "current" if entries[-1]["physics_fingerprint"] in _equivalent(fingerprint, log) else "stale"


def migrate_fingerprint(
    reg: Registry, log_path: str | Path, rel_tol: float = 1e-9, seed: int = 0
) -> dict[str, Any]:
    """Declare the CURRENT fingerprint equivalent to older logged ones, but only with evidence: every logged holdout
    entry is re-simulated with its recorded overrides and must reproduce the logged apogee error to ``rel_tol``.
    (Use after a behaviour-preserving change such as reformatting or changing the fingerprint definition.) Returns the
    verdict; a migration record is appended to the log only when all entries reproduce. The re-simulation is NOT a new
    holdout evaluation and is not logged as one."""
    lp = Path(log_path)
    log = read_log(lp)
    fp = physics_fingerprint()
    old_fps = sorted(
        {e["physics_fingerprint"] for e in log if e.get("type") != "migration"} - _equivalent(fp, log)
    )
    out: dict[str, Any] = {"current": fp, "migrating_from": old_fps, "checked": [], "ok": True}
    by_id = {f.id: f for f in reg.flights}
    for e in [x for x in log if x.get("type") != "migration" and x["physics_fingerprint"] in old_fps]:
        res = run_validation(
            by_id[e["flight"]].definition, overrides=e.get("overrides") or None, plot=False, seed=seed
        )
        now = res.metrics["apogee"]["pct_error"]
        ok = abs(now - e["apogee_err_pct"]) <= rel_tol * max(1.0, abs(e["apogee_err_pct"]))
        out["checked"].append(
            {"flight": e["flight"], "logged": e["apogee_err_pct"], "now": now, "reproduces": ok}
        )
        out["ok"] = out["ok"] and ok
    if out["ok"] and old_fps:
        for o in old_fps:
            append_log(
                lp,
                {
                    "type": "migration",
                    "time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "from_fingerprint": o,
                    "to_fingerprint": fp,
                    "evidence": "every logged holdout entry re-simulated; apogee error reproduced",
                    "checked": out["checked"],
                },
            )
    return out


@dataclass
class RegistryRow:
    flight: str
    split: str
    tier: int
    label: str
    apogee_err_pct: float | None = None
    apogee_time_err_pct: float | None = None
    max_velocity_err_pct: float | None = None
    altitude_rmse_m: float | None = None
    status: str = ""
    result: ValidationResult | None = None

    def to_dict(self) -> dict[str, Any]:
        d = {k: v for k, v in self.__dict__.items() if k != "result"}
        return d


def _row(entry: FlightEntry, res: ValidationResult, status: str) -> RegistryRow:
    m = res.metrics
    return RegistryRow(
        entry.id,
        entry.split,
        entry.tier,
        LABEL[entry.split],
        m["apogee"]["pct_error"],
        m["apogee_time"]["pct_error"],
        m.get("max_velocity", {}).get("pct_error"),
        m.get("altitude_ascent", m["altitude_all"])["rmse"],
        status,
        res,
    )


def run_registry(
    reg: Registry,
    splits: tuple[str, ...] = ("development", "calibration"),
    out_dir: str | Path | None = None,
    confirm_frozen: bool = False,
    overrides: dict[str, Any] | None = None,
    calibration_id: str | None = None,
    log_path: str | Path | None = None,
    seed: int = 0,
    only: tuple[str, ...] | None = None,
) -> list[RegistryRow]:
    """Run the requested splits. Holdout flights are skipped unless ``confirm_frozen`` is True; a holdout run is
    appended to the log (with ``calibration_id`` if overrides are a recorded calibration)."""
    out = Path(out_dir) if out_dir else None
    if "holdout" in splits and confirm_frozen:
        if log_path is None and out is None:
            raise ConfigError(
                "holdout evaluation needs a log: pass a log path (CLI: --log) so the result is recorded"
            )
        if reg.frozen_physics_version != PHYSICS_VERSION:
            raise ConfigError(
                f"registry is frozen for physics {reg.frozen_physics_version!r} but the code is {PHYSICS_VERSION!r}: "
                "update the registry deliberately (this declares a new freeze)"
            )
    log_p = Path(log_path) if log_path else (out / "holdout_log.jsonl" if out else None)
    fp = physics_fingerprint()
    log = read_log(log_p) if log_p else []
    rows: list[RegistryRow] = []
    for split in splits:
        for e in reg.by_split(split):
            if only and e.id not in only:
                continue
            if split == "holdout":
                if not confirm_frozen:
                    rows.append(
                        RegistryRow(
                            e.id, split, e.tier, LABEL[split], status="SKIPPED (model not confirmed frozen)"
                        )
                    )
                    continue
                prior = holdout_status(e.id, log, fp)
                status = {
                    "unevaluated": "first holdout evaluation",
                    "current": "RE-EVALUATION with unchanged physics (not a new test)",
                    "stale": "RE-EVALUATION after the physics changed: no longer a clean holdout",
                }[prior]
            else:
                status = "in-sample" if split == "development" else "calibration flight"
            res = run_validation(
                e.definition, overrides=overrides, out_dir=out, plot=out is not None, seed=seed
            )
            row = _row(e, res, status)
            rows.append(row)
            if split == "holdout" and log_p is not None:
                log_p.parent.mkdir(parents=True, exist_ok=True)
                entry = {
                    "flight": e.id,
                    "time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "physics_version": PHYSICS_VERSION,
                    "physics_fingerprint": fp,
                    "calibration_id": calibration_id,
                    "inputs_sha256": input_fingerprints(e),
                    "overrides": overrides or {},
                    "status": status,
                    "apogee_err_pct": row.apogee_err_pct,
                    "apogee_time_err_pct": row.apogee_time_err_pct,
                    "max_velocity_err_pct": row.max_velocity_err_pct,
                }
                append_log(log_p, entry)
    return rows


def summarize(rows: list[RegistryRow]) -> str:
    lines = [
        f"{'flight':<12}{'split':<13}{'tier':<5}{'apogee err %':>13}{'t_apogee %':>12}{'vmax %':>9}{'asc RMSE m':>12}  label / status"
    ]
    for r in rows:
        f = lambda v, w, p=2: f"{v:>{w}.{p}f}" if v is not None else " " * (w - 1) + "-"  # noqa: E731
        lines.append(
            f"{r.flight:<12}{r.split:<13}{r.tier:<5}{f(r.apogee_err_pct, 13)}{f(r.apogee_time_err_pct, 12)}"
            f"{f(r.max_velocity_err_pct, 9)}{f(r.altitude_rmse_m, 12, 1)}  {r.label}; {r.status}"
        )
    import numpy as np

    for split in SPLITS:
        e = [r.apogee_err_pct for r in rows if r.split == split and r.apogee_err_pct is not None]
        if e:
            lines.append(
                f"  {LABEL[split]:<24} n={len(e)}  RMS apogee error {float(np.sqrt(np.mean(np.square(e)))):.2f} %  "
                f"mean {float(np.mean(e)):+.2f} %  worst {max(e, key=abs):+.2f} %"
            )
    return "\n".join(lines)
