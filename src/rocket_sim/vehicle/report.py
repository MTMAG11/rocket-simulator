"""Mass-properties and stability report: what the simulator DERIVED from the vehicle description.

Everything printed here is computed from the components (nothing is typed in): total/dry/propellant mass, CG and the full
inertia tensor about the CG at ignition and burnout, CG travel, CP, static margin, and the provenance of each number
(specified vs derived vs estimated). Conventions: ``docs/vehicle_format.md``.
"""

from __future__ import annotations

import math
from typing import Any

from .mass import MassProps
from .vehicle import Vehicle


def _props(mp: MassProps) -> dict[str, Any]:
    return {
        "mass_kg": mp.mass,
        "cg_x_m": mp.x_cg,
        "cg_offset_yz_m": [mp.y_cg, mp.z_cg],
        "inertia_tensor_kgm2": [[float(v) for v in row] for row in mp.tensor()],
        "ixx": mp.ixx,
        "iyy": mp.iyy,
        "izz": mp.izz_eff,
        "axisymmetric": mp.is_axisymmetric,
    }


def mass_properties_report(veh: Vehicle, n_burn_points: int = 5) -> dict[str, Any]:
    """Structured report. ``cg_travel`` samples the CG along the burn (propellant depletion)."""
    mm = veh.mass_model
    motor = veh.motor
    p0 = motor.propellant_mass
    t_burn_motor = motor.times[-1]  # burn duration of the thrust curve, measured from ignition
    start = mm.at(p0)
    end = mm.at(0.0)
    co = veh.aero.coefficients(0.05, 5e6, False)
    d_ref = veh.aero.ref_diameter
    comps = [
        {
            "name": c.name,
            "mass_kg": c.mass,
            "cg_x_m": c.x_cg,
            "offset_yz_m": [c.y, c.z],
            "own_inertia_diag_kgm2": [c.ixx, c.iyy, c.izz_eff],
            "own_products_kgm2": [c.ixy, c.ixz, c.iyz],
        }
        for c in mm.static
    ]
    comps.append(
        {"name": "propellant", "mass_kg": p0, "cg_x_m": mm.px, "offset_yz_m": [0.0, 0.0], "burns_off": True}
    )
    cg_travel = []
    for k in range(n_burn_points + 1):
        t = t_burn_motor * k / n_burn_points
        mp = mm.at(motor.propellant_at(t))
        cg_travel.append(
            {
                "t_from_ignition_s": t,
                "mass_kg": mp.mass,
                "cg_x_m": mp.x_cg,
                "static_margin_cal": (co.x_cp - mp.x_cg) / d_ref,
            }
        )
    prov = getattr(veh.aero, "provenance", None)
    return {
        "name": veh.name,
        "reference_diameter_m": d_ref,
        "length_m": veh.body.length,
        "dry_mass_kg": mm.static_mass,
        "propellant_mass_kg": p0,
        "liftoff_mass_kg": start.mass,
        "at_ignition": _props(start),
        "at_burnout": _props(end),
        "motor": {
            "designation": motor.designation,
            "diameter_m": motor.diameter,
            "length_m": motor.length,
            "propellant_mass_kg": p0,
            "casing_mass_kg": motor.casing_mass,
            "burn_time_s": t_burn_motor,
            "total_impulse_ns": motor.total_impulse,
            "mean_propellant_mass_flow_kg_s": p0 / t_burn_motor,
            "max_propellant_mass_flow_kg_s": motor.max_thrust
            * p0
            / motor.total_impulse,  # constant exhaust velocity: mdot = F m_p / I_total
            "nozzle_exit_x_m": veh.nozzle_x,
        },
        "components": comps,
        "cg_travel": cg_travel,
        "cp_x_m": co.x_cp,
        "cn_alpha_per_rad": co.cn_alpha,
        "static_margin_ignition_cal": (co.x_cp - start.x_cg) / d_ref,
        "static_margin_burnout_cal": (co.x_cp - end.x_cg) / d_ref,
        "aero_provenance": prov.to_dict() if prov is not None else None,
        "notes": list(dict.fromkeys(veh.notes)),
    }


def format_report(r: dict[str, Any]) -> str:
    L = [f"Vehicle {r['name']}", "-" * 72]
    L.append(f"reference diameter {r['reference_diameter_m'] * 1000:.1f} mm, length {r['length_m']:.3f} m")
    L.append(
        f"dry mass {r['dry_mass_kg']:.3f} kg + propellant {r['propellant_mass_kg']:.3f} kg = liftoff {r['liftoff_mass_kg']:.3f} kg"
    )
    m = r["motor"]
    L.append(
        f"motor {m['designation']}: {m['diameter_m'] * 1000:.0f} mm x {m['length_m'] * 1000:.0f} mm, burn {m['burn_time_s']:.2f} s, impulse {m['total_impulse_ns']:.0f} N s, "
        f"propellant flow mean {m['mean_propellant_mass_flow_kg_s'] * 1000:.0f} g/s, peak {m['max_propellant_mass_flow_kg_s'] * 1000:.0f} g/s"
    )
    L.append("")
    L.append(f"{'component':<26}{'mass [kg]':>10}{'x_cg [m]':>10}{'offset y,z [mm]':>20}")
    for c in r["components"]:
        oy, oz = c["offset_yz_m"]
        L.append(f"{c['name']:<26}{c['mass_kg']:10.4f}{c['cg_x_m']:10.4f}{oy * 1000:10.1f},{oz * 1000:7.1f}")
    for tag, key in (("ignition", "at_ignition"), ("burnout", "at_burnout")):
        p = r[key]
        L.append("")
        L.append(
            f"At {tag}: mass {p['mass_kg']:.4f} kg, CG x = {p['cg_x_m']:.4f} m, offset (y,z) = "
            f"({p['cg_offset_yz_m'][0] * 1000:.2f}, {p['cg_offset_yz_m'][1] * 1000:.2f}) mm"
        )
        L.append("  inertia tensor about the CG, body axes [kg m^2]:")
        for row in p["inertia_tensor_kgm2"]:
            L.append("    " + "  ".join(f"{v:12.6f}" for v in row))
    L.append("")
    L.append("CG travel during the burn:")
    for s in r["cg_travel"]:
        L.append(
            f"  t = {s['t_from_ignition_s']:5.2f} s   mass {s['mass_kg']:.4f} kg   CG {s['cg_x_m']:.4f} m   static margin {s['static_margin_cal']:.2f} cal"
        )
    L.append("")
    L.append(f"CP (subsonic, small alpha) x = {r['cp_x_m']:.4f} m, CNa = {r['cn_alpha_per_rad']:.3f} /rad")
    L.append(
        f"static margin: {r['static_margin_ignition_cal']:.2f} cal at ignition, {r['static_margin_burnout_cal']:.2f} cal at burnout"
    )
    if r.get("aero_provenance"):
        a = r["aero_provenance"]
        L.append(
            f"aerodynamics: model '{a['model']}', overall {a['kind'].upper()} (not measured)"
            if a["kind"] != "imported"
            else f"aerodynamics: model '{a['model']}', imported data"
        )
        for k in ("drag", "normal_force_cp", "damping"):
            L.append(f"  {k:<16}{a[k]['kind']}: {a[k]['source']} (confidence: {a[k]['confidence']})")
        L.append(f"  Reynolds dependence: {a['reynolds_dependence']}")
        if a["mach_range"]:
            L.append(f"  stated Mach range: {a['mach_range'][0]:g} - {a['mach_range'][1]:g}")
    for n in r["notes"]:
        L.append(f"note: {n}")
    return "\n".join(L)


def static_margin_ok(r: dict[str, Any]) -> bool:
    return math.isfinite(r["static_margin_ignition_cal"]) and r["static_margin_ignition_cal"] > 1.0
