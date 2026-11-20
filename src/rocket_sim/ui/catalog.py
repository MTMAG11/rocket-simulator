"""What the run page offers: bundled rockets and engines, friendly descriptions, and where results are saved. No Qt here."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from ..constants import G0
from ..data.export import export_record
from ..errors import RocketSimError
from ..motor import Motor, available_motors, load_motor
from ..motor.thrustcurve import read_sidecar
from ..plotting import plot_overview
from ..reporting import summary_text
from ..resources import configs_dir, motors_dir, output_dir, resource_root, user_motors_dir, vehicles_dir
from ..simulation.record import FlightRecord

LEVELS = ("starter", "advanced", "other")
LEVEL_TITLES = {"starter": "Rockets", "advanced": "Advanced experiments", "other": "Other rockets"}
POWER_WORDS = {
    "A": "tiny",
    "B": "tiny",
    "C": "small",
    "D": "small",
    "E": "medium",
    "F": "medium",
    "G": "big",
    "H": "big",
    "I": "bigger",
    "J": "bigger",
    "K": "huge",
    "L": "huge",
}


def power_word(impulse_class: str) -> str:
    """A word for how much push an engine class has (each letter is about twice the one before)."""
    return POWER_WORDS.get(impulse_class.upper()[:1], "giant")


@dataclass(frozen=True)
class VehicleEntry:
    label: str
    path: Path  # a simulation config (.yaml) or a vehicle file (.json)
    description: str
    level: str = "other"  # starter | advanced | other

    @property
    def is_vehicle_file(self) -> bool:
        return self.path.suffix.lower() == ".json"


@dataclass(frozen=True)
class MotorEntry:
    label: str
    key: str  # config value for motor.file: path relative to the resource root (forward slashes) when inside it
    summary: str


def _leading_comment(text: str) -> str:
    lines = []
    for ln in text.splitlines():
        if not ln.startswith("#"):
            break
        lines.append(ln.lstrip("# ").rstrip())
    return " ".join(x for x in lines if x)


def _catalog_metadata() -> dict[str, dict[str, Any]]:
    """``configs/catalog.yaml``: friendly title, one-sentence blurb and level per config file (optional file)."""
    p = configs_dir() / "catalog.yaml"
    try:
        raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return {}
    items = raw.get("rockets", []) if isinstance(raw, dict) else []
    return {str(i["file"]): i for i in items if isinstance(i, dict) and "file" in i}


def list_vehicles() -> list[VehicleEntry]:
    """Simulation configs that describe a rocket, plus vehicle files no config already wraps, starters first.

    Batch, dataset and experiment specs are skipped: they are not a single vehicle."""
    meta = _catalog_metadata()
    out: list[VehicleEntry] = []
    wrapped: set[str] = set()
    for p in sorted(configs_dir().glob("*.yaml")):
        try:
            text = p.read_text(encoding="utf-8")
            raw = yaml.safe_load(text)
        except (OSError, yaml.YAMLError):
            continue
        if not isinstance(raw, dict) or not ("rocket" in raw or "vehicle_file" in raw):
            continue
        if "vehicle_file" in raw:
            wrapped.add(Path(str(raw["vehicle_file"])).name)
            name = raw.get("name") or p.stem
        else:
            name = (raw.get("rocket") or {}).get("name") or raw.get("name") or p.stem
        m = meta.get(p.name, {})
        level = m.get("level", "other") if m.get("level") in LEVELS else "other"
        out.append(VehicleEntry(str(m.get("title", name)), p, str(m.get("blurb") or _leading_comment(text)), level))
    for p in sorted(vehicles_dir().glob("*.json")):
        if p.name in wrapped or p.name.endswith(".schema.json"):
            continue
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if isinstance(d, dict) and d.get("format") == "rocket-sim-vehicle":
            md = d.get("metadata", {})
            out.append(VehicleEntry(str(md.get("name", p.stem)), p, str(md.get("description", "")), "other"))
    names = [v.label for v in out]
    out = [
        v if names.count(v.label) == 1 else VehicleEntry(f"{v.label} ({v.path.name})", v.path, v.description, v.level)
        for v in out
    ]
    if not out:
        raise RocketSimError(
            f"No vehicle configurations were found.\n\nExpected simulation configs (*.yaml) in:\n{configs_dir()}"
        )
    listed = list(meta)
    return sorted(
        out,
        key=lambda v: (LEVELS.index(v.level), listed.index(v.path.name) if v.path.name in listed else len(listed), v.label),
    )


def motor_key(path: Path) -> str:
    """The value stored in ``motor.file``: relative to the resource root when possible (portable), else absolute."""
    try:
        return path.resolve().relative_to(resource_root().resolve()).as_posix()
    except ValueError:
        return str(path)


def motor_path(key: str) -> Path:
    p = Path(key)
    return p if p.is_absolute() else resource_root() / p


def friendly_motor_text(impulse_class: str, burn_s: float, impulse_ns: float, diameter_mm: float, designation: str = "") -> str:
    name = f"{designation}: " if designation else ""
    return (
        f"{name}a {power_word(impulse_class)} engine (class {impulse_class.upper()[:1]}). It pushes for {burn_s:.1f} s, "
        f"{impulse_ns:.0f} N·s of push in total, and is {diameter_mm:.0f} mm wide."
    )


def _summary(p: Path) -> str | None:
    """Friendly one-sentence engine description; downloaded motors use their saved catalogue data (no file parsing)."""
    meta = read_sidecar(p)
    if meta.get("file_total_impulse_ns"):
        return friendly_motor_text(
            str(meta["impulse_class"]),
            float(meta["file_burn_time_s"]),
            float(meta["file_total_impulse_ns"]),
            float(meta["file_diameter_mm"]),
            str(meta["file_designation"]),
        )
    try:
        m = load_motor(p)
    except RocketSimError:
        return None  # a malformed file must not hide the others
    return friendly_motor_text(m.impulse_class, m.burn_time, m.total_impulse, m.diameter * 1000, m.designation)


def list_motors() -> list[MotorEntry]:
    out: list[MotorEntry] = []
    seen: set[str] = set()
    for d in (motors_dir(), user_motors_dir()):  # bundled first: a downloaded copy never shadows it
        for p in available_motors(d):
            if p.stem in seen:
                continue
            summary = _summary(p)
            if summary is None:
                continue
            seen.add(p.stem)
            out.append(MotorEntry(p.stem.replace("_", " "), motor_key(p), summary))
    if not out:
        raise RocketSimError(f"No motor files were found.\n\nExpected motor data (*.eng) in:\n{motors_dir()}")
    return out


# ---------------------------------------------------------------------------------------------- rocket facts
def rocket_facts(cfg: dict[str, Any]) -> dict[str, float | None]:
    """Height, width and empty weight of a rocket from its config dict (None where it cannot be told simply)."""
    r = cfg.get("rocket", {})
    secs = r.get("sections") or []
    if secs:
        length = sum(float(s.get("length_m", 0.0)) for s in secs)
        widths = [float(w) for s in secs for w in (s.get("diameter_m"), s.get("aft_diameter_m")) if w]
        width = min(widths) if widths else None  # the narrowest part is where the engine sits
        masses = [s.get("mass_kg") for s in secs] + [m.get("mass_kg") for m in r.get("masses", [])]
        mass = sum(float(x) for x in masses if x) if masses and all(x is not None for x in masses) else None
        fins = r.get("fins") or {}
        if mass is not None and fins.get("mass_kg"):
            mass += float(fins["mass_kg"])
    else:
        length = r.get("body_length_m")
        width = r.get("body_diameter_m")
        mass = r.get("dry_mass_kg")
        extra = sum(float(m.get("mass_kg", 0.0)) for m in r.get("masses", []))
        if mass is not None:
            mass = float(mass) + extra
    return {
        "height_m": None if length is None else float(length),
        "width_m": None if width is None else float(width),
        "empty_mass_kg": None if mass is None else float(mass),
    }


def rocket_text(cfg: dict[str, Any]) -> str:
    f = rocket_facts(cfg)
    parts = []
    if f["height_m"]:
        parts.append(f"{f['height_m']:.2f} m tall")
    if f["width_m"]:
        parts.append(f"{f['width_m'] * 1000:.0f} mm wide")
    if f["empty_mass_kg"]:
        parts.append(f"{f['empty_mass_kg'] * 1000:.0f} g without its engine")
    return ", ".join(parts)


def check_fit(cfg: dict[str, Any], motor: Motor) -> tuple[str, str]:
    """Will this engine fit in, and lift, this rocket? Returns (``good`` | ``warn`` | ``bad``, a short sentence)."""
    f = rocket_facts(cfg)
    if f["width_m"] is not None and motor.diameter >= f["width_m"]:
        return "bad", "This engine is too wide to fit inside this rocket. Pick a thinner engine or a wider rocket."
    if f["height_m"] is not None and motor.length >= 0.95 * f["height_m"]:
        return "bad", "This engine is too long for this rocket. Pick a shorter engine or a taller rocket."
    peak = max(motor.thrust)
    if f["empty_mass_kg"] is not None:
        weight = (f["empty_mass_kg"] + motor.total_mass) * G0
        if peak < weight:
            return "bad", "This engine is too weak: it cannot lift this rocket off the ground. Pick a stronger engine."
        if peak < 2.0 * weight:
            return "warn", "This engine can lift the rocket, but only just. It will leave the launch rail slowly."
        if peak > 25.0 * weight:
            return "warn", "This engine is very strong for this rocket. It will go fast and high."
    return "good", "This engine fits and has enough push."


def save_run_results(rec: FlightRecord, label: str, root: Path | None = None) -> Path:
    """Write telemetry (CSV), the text summary and an overview plot to a new timestamped folder; return it."""
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", label).strip("_") or "run"
    base = (root or output_dir()) / "gui_runs"
    d = base / f"{datetime.now():%Y%m%d_%H%M%S}_{safe}"
    n = 1
    while d.exists():  # two runs within one second
        n += 1
        d = base / f"{datetime.now():%Y%m%d_%H%M%S}_{safe}_{n}"
    export_record(rec, d, stem="telemetry", formats=("csv",))
    (d / "summary.txt").write_text(summary_text(rec) + "\n", encoding="utf-8")
    plot_overview(rec).savefig(d / "flight_overview.png", dpi=110)
    return d
