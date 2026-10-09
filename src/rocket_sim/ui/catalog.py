"""What the run page offers: bundled vehicles and motors, and where results are saved. No Qt here (unit-testable)."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from ..data.export import export_record
from ..errors import RocketSimError
from ..motor import available_motors, load_motor
from ..motor.thrustcurve import read_sidecar
from ..plotting import plot_overview
from ..reporting import summary_text
from ..resources import configs_dir, motors_dir, output_dir, resource_root, user_motors_dir, vehicles_dir
from ..simulation.record import FlightRecord


@dataclass(frozen=True)
class VehicleEntry:
    label: str
    path: Path  # a simulation config (.yaml) or a vehicle file (.json)
    description: str

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


def list_vehicles() -> list[VehicleEntry]:
    """Simulation configs that describe a rocket, plus vehicle files no config already wraps.

    Batch, dataset and experiment specs are skipped: they are not a single vehicle."""
    import yaml

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
        out.append(VehicleEntry(str(name), p, _leading_comment(text)))
    for p in sorted(vehicles_dir().glob("*.json")):
        if p.name in wrapped or p.name.endswith(".schema.json"):
            continue
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if isinstance(d, dict) and d.get("format") == "rocket-sim-vehicle":
            meta = d.get("metadata", {})
            out.append(VehicleEntry(str(meta.get("name", p.stem)), p, str(meta.get("description", ""))))
    names = [v.label for v in out]
    out = [
        v if names.count(v.label) == 1 else VehicleEntry(f"{v.label} ({v.path.name})", v.path, v.description)
        for v in out
    ]
    if not out:
        raise RocketSimError(
            f"No vehicle configurations were found.\n\nExpected simulation configs (*.yaml) in:\n{configs_dir()}"
        )
    return out


def motor_key(path: Path) -> str:
    """The value stored in ``motor.file``: relative to the resource root when possible (portable), else absolute."""
    try:
        return path.resolve().relative_to(resource_root().resolve()).as_posix()
    except ValueError:
        return str(path)


def _summary(p: Path) -> str | None:
    """One-line motor description; downloaded motors use their saved catalogue data (no file parsing)."""
    meta = read_sidecar(p)
    if meta.get("file_total_impulse_ns"):
        return (
            f"{meta['file_designation']} (class {meta['impulse_class']}): {meta['file_total_impulse_ns']:.0f} N·s, "
            f"burn {meta['file_burn_time_s']:.2f} s, {meta['file_propellant_g']:.0f} g propellant, {meta['file_diameter_mm']:.0f} mm"
        )
    try:
        m = load_motor(p)
    except RocketSimError:
        return None  # a malformed file must not hide the others
    return (
        f"{m.designation} (class {m.impulse_class}): {m.total_impulse:.0f} N·s, burn {m.burn_time:.2f} s, "
        f"{m.propellant_mass * 1000:.0f} g propellant, {m.diameter * 1000:.0f} mm"
    )


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
            out.append(MotorEntry(p.stem, motor_key(p), summary))
    if not out:
        raise RocketSimError(f"No motor files were found.\n\nExpected motor data (*.eng) in:\n{motors_dir()}")
    return out


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
