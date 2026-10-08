"""Apogee sensitivity study and physics error budget.

Two complementary views (docs/error_budget.md):

1. ``run_sensitivity`` - one-at-a-time central differences of apogee with respect to each input, around a nominal
   configuration. Each input carries a 1-sigma *prior* (how well that quantity is typically known for an amateur
   flight, see ``DEFAULT_PARAMS``), so ``contribution = |d apogee / d x| * sigma_x`` ranks inputs by how much of the
   apogee uncertainty they cause. The first-order budget combines contributions in quadrature (assumes independence,
   linearity); it is an estimate, not a proof, and the Monte Carlo in ``input_mc`` checks it non-linearly.
2. ``model_form_study`` - apogee change when a MODEL choice is swapped (aero hierarchy level, 3-DOF vs 6-DOF, gravity
   model, time step, mass-convention). These are not input uncertainties; they indicate how much the answer depends on
   modelling decisions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ..config import apply_overrides, config_from_dict, config_to_dict, read_mapping
from ..simulation import Simulation, run_simulation


@dataclass
class SensParam:
    path: str
    label: str
    delta: float  # perturbation used for the finite difference (same units as the parameter, or relative)
    sigma: float  # 1-sigma prior (same units as delta)
    relative: bool = False  # delta/sigma are fractions of the nominal value
    one_sided: str | None = None  # "+" or "-" when the nominal sits on a bound (wind 0, extra_cd 0, 90 deg)
    note: str = ""


DEFAULT_PARAMS: list[SensParam] = [
    SensParam("rocket.dry_mass_kg", "airframe mass", 0.02, 0.02, True, note="as-built vs. as-reported"),
    SensParam(
        "motor.thrust_scale",
        "motor total impulse / thrust",
        0.03,
        0.03,
        True,
        note="manufacturer tolerance +-3-10 %",
    ),
    SensParam(
        "rocket.aero.drag_scale",
        "overall drag level",
        0.07,
        0.07,
        True,
        note="Barrowman-type build-up vs. real surface",
    ),
    SensParam(
        "rocket.aero.surface_roughness_m",
        "surface roughness",
        0.5,
        0.5,
        True,
        note="paint/finish: factor ~1.5",
    ),
    SensParam("rocket.fins.thickness_m", "fin thickness", 0.001, 0.001, note="usually assumed, +-1 mm"),
    SensParam(
        "rocket.aero.extra_cd",
        "protuberance drag (rail buttons, lugs)",
        0.02,
        0.02,
        one_sided="+",
        note="0 in the base model",
    ),
    SensParam(
        "rocket.aero.nozzle_exit_ratio",
        "nozzle exit / motor diameter",
        0.1,
        0.1,
        note="base drag while burning",
    ),
    SensParam(
        "rocket.cg_from_nose_m",
        "airframe CG position",
        0.01,
        0.01,
        note="stability, not apogee, in first order",
    ),
    SensParam(
        "environment.atmosphere.temperature_offset_k", "air temperature offset", 4.0, 4.0, note="density"
    ),
    SensParam("environment.atmosphere.sea_level_pressure_pa", "pressure", 600.0, 600.0, note="density"),
    SensParam(
        "environment.wind.speed_ms",
        "wind speed (constant)",
        2.0,
        3.0,
        one_sided="+",
        note="drift + weathercocking",
    ),
    SensParam(
        "launch.elevation_deg", "launch elevation", 1.5, 1.5, one_sided="-", note="rail tip-off not modelled"
    ),
    SensParam(
        "motor.misalignment_deg.0", "thrust misalignment", 0.2, 0.2, one_sided="+", note="lateral thrust loss"
    ),
]


def _get(d: dict[str, Any], path: str) -> Any:
    n: Any = d
    for s in path.split("."):
        n = n[int(s)] if isinstance(n, list) else n[s]
    return n


def _apogee(raw: dict[str, Any], base_dir: Path, ov: dict[str, Any], seed: int = 0) -> float:
    full = apply_overrides(raw, {"simulation.stop_after_apogee_s": 0.5, **ov})
    return float(
        run_simulation(config_from_dict(full, base_dir=base_dir), seed=seed).summary["apogee_m"] or 0.0
    )


def run_sensitivity(
    config_path: str | Path, params: list[SensParam] | None = None, overrides: dict[str, Any] | None = None
) -> dict[str, Any]:
    p = Path(config_path)
    raw = config_to_dict(config_from_dict(read_mapping(p), base_dir=p.parent))
    if (
        raw["environment"]["wind"]["model"] == "none"
    ):  # a wind-speed perturbation needs a wind model to act on
        raw = apply_overrides(raw, {"environment.wind.model": "constant", "environment.wind.speed_ms": 0.0})
    if overrides:
        raw = apply_overrides(raw, overrides)
    base_dir = p.parent
    base = _apogee(raw, base_dir, {})
    rows: list[dict[str, Any]] = []
    for sp in params or DEFAULT_PARAMS:
        try:
            x0 = float(_get(raw, sp.path))
        except (KeyError, IndexError, TypeError, ValueError):
            continue
        d = sp.delta * abs(x0) if sp.relative else sp.delta
        sig = sp.sigma * abs(x0) if sp.relative else sp.sigma
        if d == 0:
            continue
        up = _apogee(raw, base_dir, {sp.path: x0 + d}) if sp.one_sided != "-" else base
        dn = _apogee(raw, base_dir, {sp.path: x0 - d}) if sp.one_sided != "+" else base
        if sp.one_sided == "+":
            slope = (up - base) / d
        elif sp.one_sided == "-":
            slope = (base - dn) / d
        else:
            slope = (up - dn) / (2 * d)
        rows.append(
            {
                "path": sp.path,
                "label": sp.label,
                "nominal": x0,
                "delta": d,
                "sigma": sig,
                "d_apogee_per_delta_m": slope * d,
                "d_apogee_per_delta_pct": 100 * slope * d / base,
                "sigma_contribution_m": abs(slope) * sig,
                "sigma_contribution_pct": 100 * abs(slope) * sig / base,
                "signed_slope_m_per_unit": slope,
                "note": sp.note,
                "one_sided": sp.one_sided,
            }
        )
    rows.sort(key=lambda r: -r["sigma_contribution_m"])
    rss = math.sqrt(sum(r["sigma_contribution_m"] ** 2 for r in rows))
    return {
        "config": p.name,
        "nominal_apogee_m": base,
        "rows": rows,
        "first_order_rss_m": rss,
        "first_order_rss_pct": 100 * rss / base,
    }


def sensitivity_markdown(res: dict[str, Any]) -> str:
    L = [
        f"### {res['config']}: nominal apogee {res['nominal_apogee_m']:.0f} m",
        "",
        "| rank | input | nominal | 1-sigma prior | d(apogee) per sigma [m] | per sigma [%] | note |",
        "|---|---|---|---|---|---|---|",
    ]
    for i, r in enumerate(res["rows"], 1):
        L.append(
            f"| {i} | {r['label']} (`{r['path']}`) | {r['nominal']:.4g} | {r['sigma']:.3g} | "
            f"{r['d_apogee_per_delta_m'] / r['delta'] * r['sigma']:+.1f} | {r['sigma_contribution_pct']:.2f} | {r['note']} |"
        )
    L += [
        "",
        f"First-order combined (quadrature) apogee uncertainty: **{res['first_order_rss_m']:.0f} m ({res['first_order_rss_pct']:.1f} %)**.",
        "",
    ]
    return "\n".join(L)


def model_form_study(flight_sim_yaml: str | Path, casing_kg: float | None = None) -> dict[str, Any]:
    """Apogee under alternative MODEL choices, relative to the nominal configuration."""
    p = Path(flight_sim_yaml)
    raw = config_to_dict(config_from_dict(read_mapping(p), base_dir=p.parent))
    bd = p.parent
    base = _apogee(raw, bd, {})
    # a flat Cd of the kind a user without a drag model would pick: the coast average of this vehicle's own build-up
    veh = Simulation(config_from_dict(raw, base_dir=bd)).vehicle
    length = float(veh.body.length)
    cds = [veh.aero.cd(m, 0.0, m * 340.0 * length / 1.46e-5, False) for m in (0.2, 0.4, 0.6, 0.8)]
    flat_cd = sum(cds) / len(cds)
    rough = float(raw["rocket"]["aero"]["surface_roughness_m"])
    cases: dict[str, dict[str, Any]] = {
        f"aero: simplified (flat Cd = {flat_cd:.3f}, coast average M 0.2-0.8)": {
            "rocket.aero.model": "simplified",
            "rocket.aero.cd": flat_cd,
        },
        "aero: enhanced (Mach-dependent fin lift/stall)": {"rocket.aero.model": "enhanced"},
        "fidelity 2 (3-DOF point mass) instead of 6-DOF": {"fidelity": 2},
        "gravity: constant g0 instead of inverse-square": {"environment.gravity.model": "constant"},
        "time step 0.001 s instead of the run's dt": {"simulation.dt_s": 0.001},
        "roughness x2 (60 -> 120 um)": {"rocket.aero.surface_roughness_m": 2 * rough},
    }
    if casing_kg:
        cases[f"mass convention: casing ({casing_kg:.2f} kg) added on top of the reported mass"] = {
            "rocket.dry_mass_kg": float(raw["rocket"]["dry_mass_kg"]) + casing_kg
        }
    out: list[dict[str, Any]] = []
    for name, ov in cases.items():
        try:
            a = _apogee(raw, bd, ov)
        except Exception as exc:  # a model choice may be invalid for a vehicle; report instead of aborting
            out.append({"case": name, "error": f"{type(exc).__name__}: {exc}"})
            continue
        out.append({"case": name, "apogee_m": a, "delta_m": a - base, "delta_pct": 100 * (a - base) / base})
    return {"config": p.name, "nominal_apogee_m": base, "cases": out}
