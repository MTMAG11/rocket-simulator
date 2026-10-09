"""ThrustCurve.org client: search motors and save their RASP (.eng) thrust curves into a local library.

API v1 (https://www.thrustcurve.org/info/api.html). Each saved motor gets a ``.json`` sidecar with the certified
data from the search result and the provenance of the file (data source, simfile id, URL, SHA-256), so a thrust curve
can always be traced back to where it came from. Nothing here is used by the simulation itself; it only fills the
directory the GUI and CLI list motors from.
"""

from __future__ import annotations

import base64
import hashlib
import json
import re
import urllib.error
import urllib.request
from collections.abc import Callable, Iterable
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from ..errors import MotorError
from ..version import SIM_VERSION
from .loaders import parse_eng

API = "https://www.thrustcurve.org/api/v1"
SOURCE_RANK = {
    "cert": 0,
    "mfr": 1,
    "user": 2,
}  # prefer certification-lab data, then the manufacturer's, then user uploads
BATCH = 40  # motors per download request


@dataclass(frozen=True)
class MotorInfo:
    """One search result (certified/catalogue data, not the thrust curve itself)."""

    motor_id: str
    manufacturer: str
    manufacturer_abbrev: str
    designation: str
    common_name: str
    impulse_class: str
    diameter_mm: float
    length_mm: float
    total_impulse_ns: float
    burn_time_s: float
    avg_thrust_n: float
    max_thrust_n: float
    prop_mass_g: float
    total_mass_g: float
    data_files: int
    availability: str
    info_url: str

    @property
    def stem(self) -> str:
        return _safe(f"{self.manufacturer_abbrev}_{self.designation}")

    @classmethod
    def from_api(cls, m: dict[str, Any]) -> MotorInfo:
        def f(key: str) -> float:
            v = m.get(key)
            return float(v) if v is not None else 0.0

        return cls(
            motor_id=str(m["motorId"]),
            manufacturer=str(m.get("manufacturer", "")),
            manufacturer_abbrev=str(m.get("manufacturerAbbrev") or m.get("manufacturer", "")),
            designation=str(m.get("designation", "")),
            common_name=str(m.get("commonName", "")),
            impulse_class=str(m.get("impulseClass", "")),
            diameter_mm=f("diameter"),
            length_mm=f("length"),
            total_impulse_ns=f("totImpulseNs"),
            burn_time_s=f("burnTimeS"),
            avg_thrust_n=f("avgThrustN"),
            max_thrust_n=f("maxThrustN"),
            prop_mass_g=f("propWeightG"),
            total_mass_g=f("totalWeightG"),
            data_files=int(m.get("dataFiles") or 0),
            availability=str(m.get("availability", "")),
            info_url=str(m.get("infoUrl", "")),
        )


def _safe(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "-", name).strip("-.") or "motor"


def _post(endpoint: str, payload: dict[str, Any], timeout: float = 60.0) -> dict[str, Any]:
    req = urllib.request.Request(
        f"{API}/{endpoint}",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "User-Agent": f"rocket-simulator/{SIM_VERSION}"},
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            out = json.load(r)
    except urllib.error.HTTPError as exc:
        raise MotorError(f"ThrustCurve.org answered {exc.code} {exc.reason} for {endpoint}") from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise MotorError(
            f"Could not reach ThrustCurve.org ({getattr(exc, 'reason', exc)}).\nCheck the internet connection; motors already saved locally still work."
        ) from exc
    except ValueError as exc:
        raise MotorError(f"ThrustCurve.org returned an unreadable response for {endpoint}") from exc
    if not isinstance(out, dict):
        raise MotorError(f"ThrustCurve.org returned an unexpected response for {endpoint}")
    if out.get("error"):
        raise MotorError(f"ThrustCurve.org: {out['error']}")
    return out


def search(
    *,
    name: str | None = None,
    manufacturer: str | None = None,
    diameter_mm: float | None = None,
    impulse_class: str | None = None,
    availability: str = "all",
    max_results: int = 5000,
    post: Callable[..., dict[str, Any]] | None = None,
) -> list[MotorInfo]:
    """Motors matching the criteria (all of them if none is given), only those that have a thrust-curve file."""
    req: dict[str, Any] = {"availability": availability, "maxResults": max_results}
    if name:
        req["commonName"] = name
    if manufacturer:
        req["manufacturer"] = manufacturer
    if diameter_mm:
        req["diameter"] = diameter_mm
    if impulse_class:
        req["impulseClass"] = impulse_class
    out = (post or _post)("search.json", req)
    infos = [MotorInfo.from_api(m) for m in out.get("results", [])]
    return [
        m for m in infos if m.data_files > 0
    ]  # the API's own hasDataFiles filter returns nothing, so filter here


def find_by_name(name: str, **kw: Any) -> list[MotorInfo]:
    """Motors whose designation or common name equals ``name`` (case-insensitive), e.g. ``G40W`` or ``F15``."""
    want = name.strip().lower()
    prefix = re.match(r"^[A-Oa-o]\d+", want)
    seen: dict[str, MotorInfo] = {}
    for query in dict.fromkeys([name.strip(), prefix.group(0).upper() if prefix else name.strip()]):
        for m in search(name=query, **kw):
            if want in (m.designation.lower(), m.common_name.lower()):
                seen[m.motor_id] = m
    return list(seen.values())


def download_files(
    motor_ids: list[str], post: Callable[..., dict[str, Any]] | None = None
) -> dict[str, list[dict[str, Any]]]:
    """RASP file records per motor id, best source first: ``{"data": text, "source", "simfileId", "license", ...}``."""
    files: dict[str, list[dict[str, Any]]] = {}
    for i in range(0, len(motor_ids), BATCH):
        out = (post or _post)(
            "download.json", {"motorIds": motor_ids[i : i + BATCH], "format": "RASP", "data": "file"}
        )
        for r in out.get("results", []):
            try:
                text = base64.b64decode(r["data"]).decode("utf-8", errors="replace")
            except (KeyError, ValueError):
                continue
            files.setdefault(str(r["motorId"]), []).append(
                {**{k: v for k, v in r.items() if k != "data"}, "text": text}
            )
    for recs in files.values():
        recs.sort(key=lambda r: SOURCE_RANK.get(str(r.get("source")), 9))
    return files


def save_motor(info: MotorInfo, records: list[dict[str, Any]], directory: Path) -> Path | None:
    """Write the best parseable file for ``info`` to ``directory``; returns its path, or None if no file parses."""
    for rec in records:
        try:
            motors = parse_eng(rec["text"], source=info.stem)
        except MotorError:
            continue
        if not motors:
            continue
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{info.stem}.eng"
        if path.exists() and _sidecar(path).get("motor_id") not in (None, info.motor_id):
            path = directory / f"{info.stem}-{info.motor_id[-6:]}.eng"  # same name, different catalogue entry
        content = rec["text"].replace("\r\n", "\n")  # the hash below is of the bytes on disk
        path.write_bytes(content.encode("utf-8"))
        meta = {
            **{k: v for k, v in asdict(info).items()},
            # what the saved thrust curve itself integrates to (the catalogue values above are the certified ones)
            "file_designation": motors[0].designation,
            "file_total_impulse_ns": motors[0].total_impulse,
            "file_burn_time_s": motors[0].burn_time,
            "file_max_thrust_n": motors[0].max_thrust,
            "file_propellant_g": motors[0].propellant_mass * 1000,
            "file_diameter_mm": motors[0].diameter * 1000,
            "data_source": rec.get("source"),
            "license": rec.get("license"),
            "simfile_id": rec.get("simfileId"),
            "data_url": rec.get("source_url") or rec.get("dataUrl"),
            "sha256": hashlib.sha256(content.encode("utf-8")).hexdigest(),
            "fetched": datetime.now(UTC).isoformat(timespec="seconds"),
            "fetched_by": f"rocket-simulator {SIM_VERSION}",
        }
        _sidecar_path(path).write_text(json.dumps(meta, indent=2), encoding="utf-8")
        return path
    return None


def _sidecar_path(eng: Path) -> Path:
    return eng.with_suffix(".json")


def _sidecar(eng: Path) -> dict[str, Any]:
    p = _sidecar_path(eng)
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def read_sidecar(eng: Path) -> dict[str, Any]:
    """Catalogue data saved next to a downloaded motor (empty dict for hand-made files)."""
    return _sidecar(eng)


def download_motors(
    infos: Iterable[MotorInfo],
    directory: Path,
    *,
    skip_existing: bool = True,
    progress: Callable[[int, int, str], None] | None = None,
    post: Callable[..., dict[str, Any]] | None = None,
) -> tuple[list[Path], list[tuple[MotorInfo, str]]]:
    """Download and save motors. Returns (written or already-present paths, [(motor, reason it was skipped)])."""
    todo = list(infos)
    saved: list[Path] = []
    failed: list[tuple[MotorInfo, str]] = []
    pending: list[MotorInfo] = []
    for m in todo:
        existing = directory / f"{m.stem}.eng"
        if skip_existing and existing.exists() and _sidecar(existing).get("motor_id") == m.motor_id:
            saved.append(existing)
        else:
            pending.append(m)
    done = len(saved)
    for i in range(0, len(pending), BATCH):
        chunk = pending[i : i + BATCH]
        files = download_files([m.motor_id for m in chunk], post=post)
        for m in chunk:
            done += 1
            if progress:
                progress(done, len(todo), m.stem)
            recs = files.get(m.motor_id)
            if not recs:
                failed.append((m, "no thrust-curve file available"))
                continue
            path = save_motor(m, recs, directory)
            if path is None:
                failed.append((m, "the file could not be parsed"))
            else:
                saved.append(path)
    return saved, failed
