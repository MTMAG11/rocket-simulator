"""Versioned vehicle description file (``vehicle.json``) -> simulator ``rocket`` configuration.

The file describes a rocket as a list of physically meaningful COMPONENTS. The simulator derives total mass, CG,
the inertia tensor, CP, static margin and the aerodynamic inputs from it; nothing derived is typed in.

Conventions (also in docs/vehicle_format.md)
--------------------------------------------
* SI units. ``x`` is the axial position measured AFT from the nose tip [m] (the nose tip is x = 0).
* Body axes for lateral offsets and inertia tensors: x forward (toward the nose), y right, z down. An off-axis
  component sits at ``offset_m = [y, z]`` from the rocket axis.
* ``inertia_kgm2`` is the 3x3 inertia TENSOR (matrix) about the component's OWN centre of gravity, in body axes:
  ``[[Ixx, -Pxy, -Pxz], [-Pxy, Iyy, -Pyz], [-Pxz, -Pyz, Izz]]`` with products of inertia ``Pxy = integral x y dm``.
  (CAD systems often report products rather than the matrix; check the sign when copying.)
* The format is deterministic: loading, compiling and hashing the same file gives the same result.
* Unknown keys are errors (typos must not silently disappear).

This module is the ONLY place that knows about the vehicle file. It emits the ordinary ``rocket`` / ``motor`` config
sections, so the physics engine is independent of the file format (a future OpenRocket or CAD importer only has to
write this JSON).
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ..errors import ConfigError
from .assembly import Section

FORMAT = "rocket-sim-vehicle"
FORMAT_VERSION = 1

GEOMETRY_TYPES = ("nose", "body_tube", "transition")
MASS_TYPES = ("motor_mount", "avionics", "battery", "payload", "recovery", "ballast", "structure", "other")
FIN_TYPES = ("fin_set", "control_surface_set")
ALL_TYPES = GEOMETRY_TYPES + FIN_TYPES + MASS_TYPES
SHAPES = ("box", "cylinder", "point")

_COMMON = {"id", "type", "name", "mass_kg", "material", "notes"}
_PLACED = {
    "cg_x_m",
    "offset_m",
    "inertia_kgm2",
}  # CG / offset / tensor: only for components that carry them into the mass model
_ALLOWED: dict[str, set[str]] = {
    "nose": _COMMON | _PLACED | {"shape", "length_m", "base_diameter_m", "x_start_m", "wall_thickness_m"},
    "body_tube": _COMMON | _PLACED | {"length_m", "diameter_m", "x_start_m", "wall_thickness_m"},
    "transition": _COMMON
    | _PLACED
    | {"length_m", "fore_diameter_m", "aft_diameter_m", "x_start_m", "wall_thickness_m"},
    "fin_set": _COMMON
    | {"count", "root_chord_m", "tip_chord_m", "span_m", "sweep_m", "thickness_m", "x_root_le_m"},
    "control_surface_set": _COMMON
    | {
        "count",
        "root_chord_m",
        "tip_chord_m",
        "span_m",
        "sweep_m",
        "thickness_m",
        "x_root_le_m",
        "roll_angle0_deg",
        "max_deflection_deg",
        "max_rate_deg_s",
        "time_constant_s",
        "delay_s",
    },
}
for _t in MASS_TYPES:
    _ALLOWED[_t] = _COMMON | _PLACED | {"shape", "dimensions_m"}
_TOP = {
    "format",
    "format_version",
    "metadata",
    "materials",
    "components",
    "motor",
    "parachutes",
    "aero",
    "geometry",
}
_MOTOR = {"file", "designation", "aft_position_m"}
_META = {"name", "description", "author", "created", "source", "data_quality", "notes"}
DATA_QUALITY = ("design", "measured", "estimated", "placeholder")


@dataclass
class CompiledComponent:
    """One row of the component table (what the simulator will actually use)."""

    id: str
    type: str
    mass_kg: float
    x_cg_m: float
    offset_m: tuple[float, float]
    mass_source: str  # specified | derived_shell | derived_plate
    inertia_source: str  # specified | shape | simulator-estimate (thin shell) | point
    notes: str = ""


@dataclass
class CompiledVehicle:
    name: str
    rocket: dict[str, Any]  # the `rocket:` config section
    motor: dict[str, Any]  # `file` / `designation` (+ nothing else)
    components: list[CompiledComponent]
    sha256: str
    source: str
    data_quality: str
    warnings: list[str] = field(default_factory=list)


# ------------------------------------------------------------------------------------------ helpers
def _fail(cid: str, msg: str) -> ConfigError:
    return ConfigError(f"vehicle component {cid!r}: {msg}")


def _num(c: dict[str, Any], key: str, cid: str, positive: bool = True) -> float:
    if key not in c:
        raise _fail(cid, f"missing required field '{key}'")
    v = c[key]
    if isinstance(v, bool) or not isinstance(v, int | float) or not math.isfinite(v):
        raise _fail(cid, f"'{key}' must be a finite number, got {v!r}")
    if positive and v <= 0:
        raise _fail(cid, f"'{key}' must be > 0, got {v}")
    return float(v)


def _density(c: dict[str, Any], materials: dict[str, float], cid: str) -> float:
    m = c.get("material")
    if m is None:
        raise _fail(cid, "needs 'mass_kg' or a 'material' (with a wall/plate thickness) to derive its mass")
    if isinstance(m, str):
        if m not in materials:
            raise _fail(cid, f"material {m!r} is not in the top-level 'materials' table")
        return materials[m]
    if isinstance(m, dict) and "density_kgm3" in m:
        rho = float(m["density_kgm3"])
        if not math.isfinite(rho) or rho <= 0:
            raise _fail(cid, "material density_kgm3 must be a finite number > 0")
        return rho
    raise _fail(cid, "material must be a name from 'materials' or {'name':..., 'density_kgm3':...}")


def _tensor_to_products(t: Any, cid: str) -> dict[str, float]:
    try:
        ixx, iyy, izz = float(t[0][0]), float(t[1][1]), float(t[2][2])
        o = {"ixy": -float(t[0][1]), "ixz": -float(t[0][2]), "iyz": -float(t[1][2])}
        sym = abs(t[0][1] - t[1][0]) + abs(t[0][2] - t[2][0]) + abs(t[1][2] - t[2][1])
    except (TypeError, IndexError, ValueError, KeyError) as exc:
        raise _fail(cid, "inertia_kgm2 must be a 3x3 matrix of numbers") from exc
    if sym > 1e-9 * max(abs(ixx) + abs(iyy) + abs(izz), 1e-30):
        raise _fail(cid, "inertia_kgm2 must be symmetric")
    if min(ixx, iyy, izz) < 0:
        raise _fail(cid, "inertia_kgm2 diagonal entries must be >= 0")
    import numpy as np

    full = np.array(t, dtype=float)
    if np.linalg.eigvalsh(full).min() < -1e-9 * max(float(np.abs(full).max()), 1e-30):
        raise _fail(
            cid,
            "inertia_kgm2 must be positive semi-definite (a physical rigid body); check the sign of the off-diagonal products",
        )
    # a physical tensor satisfies the triangle inequalities of its principal moments
    if ixx + iyy < izz - 1e-12 or ixx + izz < iyy - 1e-12 or iyy + izz < ixx - 1e-12:
        raise _fail(cid, "inertia_kgm2 violates the triangle inequality of a physical rigid body")
    return {"ixx_kgm2": ixx, "iyy_kgm2": iyy, "izz_kgm2": izz, **{f"{k}_kgm2": v for k, v in o.items()}}


def shape_inertia(mass: float, shape: str, dims: dict[str, Any], cid: str) -> tuple[float, float, float]:
    """Principal (Ixx, Iyy, Izz) of a uniform solid about its own CG, body axes (x = axial).

    box       dimensions_m = [length_x (axial), width_y, height_z]
    cylinder  dimensions_m = [length (axial), diameter]
    point     zero inertia about its own CG
    """
    if shape == "point":
        return 0.0, 0.0, 0.0
    try:
        dvals = [float(v) for v in dims]
    except (TypeError, ValueError) as exc:
        raise _fail(cid, "dimensions_m must be a list of numbers") from exc
    need = {"box": 3, "cylinder": 2, "point": 0}.get(shape)
    if need is None:
        raise _fail(cid, f"shape must be one of {SHAPES}")
    if len(dvals) != need or any(not math.isfinite(v) or v <= 0 for v in dvals):
        raise _fail(cid, f"shape {shape!r} needs {need} positive dimensions_m, got {dims!r}")
    if shape == "box":
        lx, ly, lz = dvals
        return (
            mass / 12.0 * (ly * ly + lz * lz),
            mass / 12.0 * (lx * lx + lz * lz),
            mass / 12.0 * (lx * lx + ly * ly),
        )
    if shape == "cylinder":
        length, d = dvals
        r = 0.5 * d
        ixx = 0.5 * mass * r * r
        iyy = mass * (3 * r * r + length * length) / 12.0
        return ixx, iyy, iyy
    raise _fail(cid, f"shape must be one of {SHAPES}")


def _fin_cg(f: dict[str, Any]) -> float:
    """Axial CG of a fin set as the builder places it: leading edge + sweep-centroid + half the mean chord."""
    r, t, sw = f["root_chord_m"], f["tip_chord_m"], f["sweep_m"]
    return f["position_from_nose_m"] + sw / 3.0 * (r + 2 * t) / (r + t) + 0.5 * (0.5 * (r + t))


def _offset(c: dict[str, Any], cid: str, radius: float) -> list[float]:
    off = c.get("offset_m", [0.0, 0.0])
    if not (
        isinstance(off, list)
        and len(off) == 2
        and all(isinstance(v, int | float) and not isinstance(v, bool) and math.isfinite(v) for v in off)
    ):
        raise _fail(cid, "offset_m must be [y, z] (two finite numbers)")
    if math.hypot(off[0], off[1]) > radius + 1e-9:
        raise _fail(cid, f"offset_m {off} puts the component outside the airframe (radius {radius:.4f} m)")
    return [float(off[0]), float(off[1])]


def _check_keys(c: dict[str, Any], cid: str) -> None:
    t = c.get("type")
    if t not in ALL_TYPES:
        raise _fail(cid, f"type must be one of {ALL_TYPES}, got {t!r}")
    extra = set(c) - _ALLOWED[t]
    if extra:
        raise _fail(cid, f"unknown field(s) {sorted(extra)} for type {t!r}")


# --------------------------------------------------------------------------------------- compile
def compile_vehicle(data: dict[str, Any], base_dir: str | Path = ".", sha256: str = "") -> CompiledVehicle:
    data = copy.deepcopy(data)  # never mutate the caller's dictionary
    extra = set(data) - _TOP
    if extra:
        raise ConfigError(f"vehicle file: unknown top-level key(s) {sorted(extra)}")
    if data.get("format") != FORMAT:
        raise ConfigError(f"vehicle file: 'format' must be {FORMAT!r}")
    ver = data.get("format_version")
    if ver != FORMAT_VERSION:
        raise ConfigError(
            f"vehicle file: format_version {ver!r} is not supported (this simulator reads {FORMAT_VERSION})"
        )
    meta = data.get("metadata", {})
    if set(meta) - _META:
        raise ConfigError(f"vehicle file: unknown metadata key(s) {sorted(set(meta) - _META)}")
    name = str(meta.get("name", "vehicle"))
    quality = str(meta.get("data_quality", "design"))
    if quality not in DATA_QUALITY:
        raise ConfigError(f"vehicle file: metadata.data_quality must be one of {DATA_QUALITY}")
    materials = {k: float(v) for k, v in data.get("materials", {}).items()}
    for mk, mv in materials.items():
        if not math.isfinite(mv) or mv <= 0:
            raise ConfigError(f"vehicle file: material {mk!r} density must be a finite number > 0, got {mv}")
    comps = data.get("components")
    if not isinstance(comps, list) or not comps:
        raise ConfigError("vehicle file: 'components' must be a non-empty list")
    ids = [c.get("id") for c in comps]
    if any(not isinstance(i, str) or not i for i in ids) or len(set(ids)) != len(ids):
        raise ConfigError("vehicle file: every component needs a unique non-empty string 'id'")

    warnings: list[str] = []
    table: list[CompiledComponent] = []
    sections: list[dict[str, Any]] = []
    items: list[dict[str, Any]] = []
    fin_cfg: dict[str, Any] | None = None
    control_cfg: dict[str, Any] | None = None

    geo = [c for c in comps if c.get("type") in GEOMETRY_TYPES]
    for c in comps:
        _check_keys(c, c["id"])
    for c in geo:
        c.setdefault("x_start_m", None)
    if not geo or sum(1 for c in geo if c["type"] == "nose") != 1:
        raise ConfigError("vehicle file: exactly one 'nose' component is required")

    def x0(c: dict[str, Any]) -> float:
        return float(c["x_start_m"]) if c.get("x_start_m") is not None else math.nan

    # airframe stack: explicit x_start_m (or sequential if omitted), checked for contiguity
    geo_sorted = sorted(geo, key=lambda c: x0(c) if not math.isnan(x0(c)) else math.inf)
    if any(math.isnan(x0(c)) for c in geo):
        geo_sorted = list(geo)  # no positions: file order is the stacking order
    x_next = 0.0
    prev_d_aft = 0.0
    for k, c in enumerate(geo_sorted):
        cid, t = c["id"], c["type"]
        length = _num(c, "length_m", cid)
        xs = x0(c)
        if math.isnan(xs):
            xs = x_next
        if abs(xs - x_next) > 1e-6:
            raise _fail(
                cid, f"x_start_m={xs} leaves a gap/overlap in the airframe stack (expected {x_next:.6f})"
            )
        if k == 0 and t != "nose":
            raise _fail(cid, "the first airframe component (x = 0) must be the nose")
        if t == "nose":
            d_aft = _num(c, "base_diameter_m", cid)
            sec: dict[str, Any] = {
                "name": cid,
                "type": "nose",
                "shape": c.get("shape", "ogive"),
                "length_m": length,
                "diameter_m": d_aft,
            }
            d_fore = 0.0
        elif t == "body_tube":
            d_aft = d_fore = _num(c, "diameter_m", cid)
            sec = {"name": cid, "type": "body", "length_m": length, "diameter_m": d_aft}
        else:
            d_fore = _num(c, "fore_diameter_m", cid)
            d_aft = _num(c, "aft_diameter_m", cid)
            sec = {
                "name": cid,
                "type": "transition",
                "length_m": length,
                "fore_diameter_m": d_fore,
                "aft_diameter_m": d_aft,
            }
        if t != "nose" and abs(d_fore - prev_d_aft) > 1e-6:
            raise _fail(
                cid,
                f"forward diameter {d_fore} m does not match the previous section's aft diameter {prev_d_aft} m",
            )
        # mass
        geom_section = Section(
            "nose" if t == "nose" else ("body" if t == "body_tube" else "transition"),
            xs,
            length,
            d_fore,
            d_aft,
            sec.get("shape", "ogive"),
        )
        if "mass_kg" in c:
            mass, msrc = _num(c, "mass_kg", cid, positive=False), "specified"
            if mass < 0:
                raise _fail(cid, "mass_kg must be >= 0")
        else:
            rho = _density(c, materials, cid)
            th = _num(c, "wall_thickness_m", cid)
            if th >= 0.5 * min(x for x in (d_fore, d_aft) if x > 0):
                raise _fail(cid, "wall_thickness_m must be smaller than the smallest radius of the component")
            if t == "body_tube":
                ro = 0.5 * d_aft
                if th >= ro:
                    raise _fail(cid, "wall_thickness_m must be smaller than the radius")
                mass = math.pi * (ro * ro - (ro - th) ** 2) * length * rho
            else:
                mass = geom_section.wetted_area() * th * rho  # thin shell on the outer surface
            msrc = "derived_shell"
        cg_x = float(c["cg_x_m"]) if "cg_x_m" in c else xs + 0.5 * length
        if not xs - 1e-9 <= cg_x <= xs + length + 1e-9:
            raise _fail(cid, "cg_x_m must lie within the component")
        explicit = "inertia_kgm2" in c or "offset_m" in c
        if explicit and mass > 0:
            # geometry carries no mass; the (CAD-supplied) tensor / offset rides on a mass item at the CG
            sec["mass_kg"] = 0.0
            item = {"name": cid, "mass_kg": mass, "position_from_nose_m": cg_x}
            off = _offset(c, cid, 0.5 * max(d_fore, d_aft))
            item["offset_y_m"], item["offset_z_m"] = off
            if "inertia_kgm2" in c:
                item.update(_tensor_to_products(c["inertia_kgm2"], cid))
                isrc = "specified"
            else:
                warnings.append(f"{cid}: inertia not given: thin-shell estimate in the simulator")
                isrc = "simulator-estimate (thin shell)"
                from .mass import thin_tube_inertia

                ixx, iyy = thin_tube_inertia(mass, geom_section.mean_radius, length)
                item.update({"ixx_kgm2": ixx, "iyy_kgm2": iyy})
            items.append(item)
        else:
            sec["mass_kg"] = mass
            sec["cg_fraction"] = (cg_x - xs) / length
            isrc = "simulator-estimate (thin shell)" if mass > 0 else "none (massless section)"
        table.append(
            CompiledComponent(
                cid,
                t,
                mass,
                cg_x,
                (float(c["offset_m"][0]), float(c["offset_m"][1])) if "offset_m" in c else (0.0, 0.0),
                msrc,
                isrc,
                str(c.get("notes", "")),
            )
        )
        sections.append(sec)
        x_next = xs + length
        prev_d_aft = d_aft
    airframe_length = x_next
    max_diameter = max(sc.get("diameter_m", sc.get("fore_diameter_m", 0.0)) for sc in sections)
    max_diameter = max([max_diameter] + [sc.get("aft_diameter_m", 0.0) for sc in sections])

    def _count(c: dict[str, Any], cid: str) -> int:
        v = c.get("count")
        if isinstance(v, bool) or not isinstance(v, int) or v < 2:
            raise _fail(cid, f"'count' must be an integer >= 2, got {v!r}")
        return v

    def fin_geometry(c: dict[str, Any]) -> dict[str, Any]:
        cid = c["id"]
        return {
            "count": _count(c, cid),
            "root_chord_m": _num(c, "root_chord_m", cid),
            "tip_chord_m": _num(c, "tip_chord_m", cid),
            "span_m": _num(c, "span_m", cid),
            "sweep_m": _num(c, "sweep_m", cid, positive=False),
            "thickness_m": _num(c, "thickness_m", cid),
            "position_from_nose_m": _num(c, "x_root_le_m", cid, positive=False),
        }

    def plate_mass(c: dict[str, Any], g: dict[str, Any]) -> tuple[float, str]:
        cid = c["id"]
        if "mass_kg" in c:
            return _num(c, "mass_kg", cid, positive=False), "specified"
        rho = _density(c, materials, cid)
        area = 0.5 * (g["root_chord_m"] + g["tip_chord_m"]) * g["span_m"]
        return g["count"] * area * g["thickness_m"] * rho, "derived_plate"

    for c in comps:
        cid, t = c["id"], c["type"]
        if t == "fin_set":
            if fin_cfg is not None:
                raise _fail(cid, "only one fin_set is supported (use control_surface_set for movable fins)")
            fin_cfg = fin_geometry(c)
            if fin_cfg["span_m"] > 2.0 * max_diameter:
                warnings.append(
                    f"{cid}: fin span {fin_cfg['span_m']:.3f} m exceeds two body diameters ({2 * max_diameter:.3f} m): check the units"
                )
            mass, msrc = plate_mass(c, fin_cfg)
            fin_cfg["mass_kg"] = mass
            table.append(
                CompiledComponent(
                    cid,
                    t,
                    mass,
                    _fin_cg(fin_cfg),
                    (0.0, 0.0),
                    msrc,
                    "simulator-estimate (thin plates)",
                    str(c.get("notes", "")),
                )
            )
        elif t == "control_surface_set":
            if control_cfg is not None:
                raise _fail(cid, "only one control_surface_set is supported")
            control_cfg = fin_geometry(c)
            mass, msrc = plate_mass(c, control_cfg)
            control_cfg.update(
                {
                    "mass_kg": mass,
                    "roll_angle0_deg": float(c.get("roll_angle0_deg", 0.0)),
                    "max_deflection_deg": float(c.get("max_deflection_deg", 10.0)),
                    "max_rate_deg_s": float(c.get("max_rate_deg_s", 200.0)),
                    "time_constant_s": float(c.get("time_constant_s", 0.0)),
                    "delay_s": float(c.get("delay_s", 0.0)),
                }
            )
            table.append(
                CompiledComponent(
                    cid,
                    t,
                    mass,
                    control_cfg["position_from_nose_m"] + 0.5 * control_cfg["root_chord_m"],
                    (0.0, 0.0),
                    msrc,
                    "simulator-estimate (thin plates)",
                    str(c.get("notes", "")),
                )
            )
        elif t in MASS_TYPES:
            mass = _num(c, "mass_kg", cid, positive=False)
            if mass < 0:
                raise _fail(cid, "mass_kg must be >= 0")
            if "cg_x_m" not in c:
                raise _fail(
                    cid,
                    "missing required field 'cg_x_m' (axial position of its centre of gravity, aft of the nose)",
                )
            cg_x = _num(c, "cg_x_m", cid, positive=False)
            if not 0.0 <= cg_x <= airframe_length + 1e-9:
                raise _fail(cid, f"cg_x_m={cg_x} is outside the airframe (0, {airframe_length:.4f}) m")
            off = _offset(c, cid, 0.5 * max_diameter)
            item = {
                "name": cid,
                "mass_kg": mass,
                "position_from_nose_m": cg_x,
                "offset_y_m": float(off[0]),
                "offset_z_m": float(off[1]),
            }
            if "inertia_kgm2" in c and "shape" in c:
                raise _fail(cid, "give EITHER inertia_kgm2 OR shape + dimensions_m")
            if "inertia_kgm2" in c:
                item.update(_tensor_to_products(c["inertia_kgm2"], cid))
                isrc = "specified"
            elif "shape" in c:
                if "inertia_kgm2" in c:
                    raise _fail(cid, "give EITHER inertia_kgm2 OR shape + dimensions_m")
                dims = c.get("dimensions_m", [])
                ixx, iyy, izz = shape_inertia(mass, c["shape"], dims, cid)
                item.update({"ixx_kgm2": ixx, "iyy_kgm2": iyy, "izz_kgm2": izz})
                isrc = f"shape ({c['shape']})"
            else:
                isrc = "point"
                warnings.append(f"{cid}: no inertia or shape: treated as a point mass")
            items.append(item)
            table.append(
                CompiledComponent(
                    cid,
                    t,
                    mass,
                    cg_x,
                    (float(off[0]), float(off[1])),
                    "specified",
                    isrc,
                    str(c.get("notes", "")),
                )
            )

    # motor
    mot = data.get("motor")
    if not isinstance(mot, dict) or "file" not in mot:
        raise ConfigError("vehicle file: 'motor' with at least 'file' is required")
    if set(mot) - _MOTOR:
        raise ConfigError(f"vehicle file: unknown motor key(s) {sorted(set(mot) - _MOTOR)}")
    motor_aft = float(mot.get("aft_position_m", airframe_length))
    if not 0.0 < motor_aft <= airframe_length + 1e-9:
        raise ConfigError(
            f"vehicle file: motor.aft_position_m={motor_aft} must lie within the airframe (0, {airframe_length:.4f}) m"
        )
    motor_cfg: dict[str, Any] = {"file": str(mot["file"])}
    if mot.get("designation"):
        motor_cfg["designation"] = str(mot["designation"])

    rocket: dict[str, Any] = {
        "name": name,
        "sections": sections,
        "masses": items,
        "motor_aft_from_nose_m": motor_aft,
    }
    if fin_cfg is not None:
        rocket["fins"] = fin_cfg
    else:
        rocket["fins"] = {"count": 0, "mass_kg": 0.0}
    if control_cfg is not None:
        rocket["control_surfaces"] = control_cfg
    geo_block = data.get("geometry", {})
    if "reference_diameter_m" in geo_block:
        rocket["reference_diameter_m"] = float(geo_block["reference_diameter_m"])
    if data.get("parachutes"):
        rocket["parachutes"] = list(data["parachutes"])
    if data.get("aero"):
        rocket["aero"] = dict(data["aero"])
    return CompiledVehicle(
        name, rocket, motor_cfg, table, sha256, str(meta.get("source", "manual")), quality, warnings
    )


def load_vehicle(path: str | Path) -> CompiledVehicle:
    p = Path(path)
    raw = p.read_bytes()
    try:
        data = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as exc:
        raise ConfigError(f"{p.name}: not valid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"{p.name}: top level must be an object")
    sha = hashlib.sha256(raw.replace(b"\r\n", b"\n")).hexdigest()
    try:
        return compile_vehicle(data, p.parent, sha)
    except ConfigError as exc:
        raise ConfigError(f"{p.name}: {exc}") from exc
