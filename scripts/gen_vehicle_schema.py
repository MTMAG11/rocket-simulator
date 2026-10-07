"""Generate vehicles/vehicle.schema.json (JSON Schema 2020-12) from the compiler's own tables.

The schema is for editors, CAD exporters and external validators; the authoritative validation is
``rocket_sim.vehicle.vehicle_file.compile_vehicle`` (it checks physics: contiguity, tensor symmetry, triangle inequalities...).
Generating the schema from the same tables keeps the two from drifting (a test compares them).

Run:  python scripts/gen_vehicle_schema.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rocket_sim.vehicle import vehicle_file as vf  # noqa: E402

NUM = {"type": "number"}
POS = {"type": "number", "exclusiveMinimum": 0}
STR = {"type": "string"}
PROPS: dict[str, dict] = {
    "id": {"type": "string", "minLength": 1},
    "type": STR,
    "name": STR,
    "notes": STR,
    "mass_source": STR,
    "mass_kg": {
        "type": "number",
        "minimum": 0,
        "description": "component mass [kg]; omit to derive from material + thickness",
    },
    "cg_x_m": {**NUM, "description": "axial position of the component CG, aft of the nose tip [m]"},
    "offset_m": {
        "type": "array",
        "items": NUM,
        "minItems": 2,
        "maxItems": 2,
        "description": "[y, z] CG offset from the axis, body axes (y right, z down) [m]",
    },
    "inertia_kgm2": {
        "type": "array",
        "minItems": 3,
        "maxItems": 3,
        "items": {"type": "array", "items": NUM, "minItems": 3, "maxItems": 3},
        "description": "3x3 inertia TENSOR about the component's own CG, body axes (x forward, y right, z down); off-diagonals are minus the products of inertia",
    },
    "material": {
        "oneOf": [
            STR,
            {
                "type": "object",
                "required": ["density_kgm3"],
                "properties": {"name": STR, "density_kgm3": POS},
            },
        ]
    },
    "shape": STR,
    "dimensions_m": {
        "type": "array",
        "items": POS,
        "description": "box: [length_x, width_y, height_z]; cylinder: [length, diameter]",
    },
    "x_start_m": {**NUM, "minimum": 0, "description": "start of the section, aft of the nose tip [m]"},
    "length_m": POS,
    "base_diameter_m": POS,
    "diameter_m": POS,
    "fore_diameter_m": POS,
    "aft_diameter_m": POS,
    "wall_thickness_m": POS,
    "count": {"type": "integer", "minimum": 2},
    "root_chord_m": POS,
    "tip_chord_m": POS,
    "span_m": POS,
    "sweep_m": NUM,
    "thickness_m": POS,
    "x_root_le_m": {**NUM, "description": "leading edge of the root chord, aft of the nose tip [m]"},
    "roll_angle0_deg": NUM,
    "max_deflection_deg": POS,
    "max_rate_deg_s": POS,
    "time_constant_s": {"type": "number", "minimum": 0},
    "delay_s": {"type": "number", "minimum": 0},
}


def build() -> dict:
    comp_defs = {}
    one_of = []
    for t in vf.ALL_TYPES:
        props = {k: PROPS[k] for k in sorted(vf._ALLOWED[t])}
        props["type"] = {"const": t}
        comp_defs[t] = {
            "type": "object",
            "properties": props,
            "required": ["id", "type"],
            "additionalProperties": False,
        }
        one_of.append({"$ref": f"#/$defs/{t}"})
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://example.invalid/rocket-sim-vehicle-1.json",
        "title": "rocket-sim vehicle file",
        "description": "Version 1. Axial positions are metres AFT of the nose tip; body axes x forward, y right, z down. See docs/vehicle_format.md.",
        "type": "object",
        "required": ["format", "format_version", "components", "motor"],
        "additionalProperties": False,
        "properties": {
            "format": {"const": vf.FORMAT},
            "format_version": {"const": vf.FORMAT_VERSION},
            "metadata": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    k: ({"enum": list(vf.DATA_QUALITY)} if k == "data_quality" else STR)
                    for k in sorted(vf._META)
                },
            },
            "materials": {"type": "object", "additionalProperties": POS},
            "geometry": {
                "type": "object",
                "properties": {"reference_diameter_m": POS},
                "additionalProperties": False,
            },
            "components": {"type": "array", "minItems": 1, "items": {"oneOf": one_of}},
            "motor": {
                "type": "object",
                "required": ["file"],
                "additionalProperties": False,
                "properties": {k: (POS if k == "aft_position_m" else STR) for k in sorted(vf._MOTOR)},
            },
            "parachutes": {"type": "array", "items": {"type": "object"}},
            "aero": {"type": "object"},
        },
        "$defs": comp_defs,
    }


def main() -> None:
    out = ROOT / "vehicles" / "vehicle.schema.json"
    out.write_text(json.dumps(build(), indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
