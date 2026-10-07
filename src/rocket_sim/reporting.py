"""Human-readable flight summaries with uncertainty-aware number formatting."""

from __future__ import annotations

from .simulation.record import FlightRecord
from .uncertainty import format_value, provisional_note

# (label, summary key, unit)
SUMMARY_ROWS: list[tuple[str, str, str]] = [
    ("Apogee (AGL)", "apogee_m", "m"),
    ("Apogee time", "apogee_time_s", "s"),
    ("Max velocity", "max_velocity_ms", "m/s"),
    ("Max Mach", "max_mach", ""),
    ("Max acceleration", "max_acceleration_ms2", "m/s^2"),
    ("Max load factor", "max_load_factor_g", "g"),
    ("Burnout time", "burnout_time_s", "s"),
    ("Burnout altitude", "burnout_altitude_m", "m"),
    ("Burnout velocity", "burnout_velocity_ms", "m/s"),
    ("Rail-exit velocity", "rail_exit_velocity_ms", "m/s"),
    ("Impact speed", "impact_speed_ms", "m/s"),
    ("Impact vertical speed", "impact_vertical_speed_ms", "m/s"),
    ("Landing distance from pad", "landing_distance_m", "m"),
    ("Total flight time", "flight_time_s", "s"),
]


def summary_rows(rec: FlightRecord) -> list[tuple[str, str]]:
    s = rec.summary
    fid = rec.meta.fidelity
    out = outside_envelope(rec)
    rows = []
    for label, key, unit in SUMMARY_ROWS:
        v = s.get(key)
        rows.append(
            (label, format_value(float(v), key, unit, fid, out is not None) if v is not None else "n/a")
        )
    sm = s.get("static_margin_launch_cal")
    if sm is not None:
        rows.append(("Static margin at launch", f"{sm:.1f} cal"))
    rows.append(("Liftoff thrust/weight", f"{s.get('liftoff_thrust_to_weight', 0.0):.1f}"))
    return rows


def outside_envelope(rec: FlightRecord) -> str | None:
    """Why the stated model uncertainty does not apply to this flight (None: inside the compared envelope).

    The apogee/velocity bands come from 7 subsonic flights of 7-24 kg vehicles; a flight beyond the stated aerodynamic range,
    or a much lighter/heavier vehicle, is an extrapolation and its uncertainty is marked provisional."""
    reasons = []
    if any("stated Mach range" in w for w in rec.meta.warnings):
        reasons.append("flew beyond the stated Mach range of the aerodynamic model")
    m0 = float(rec.col("mass")[0]) if rec.has("mass") else None
    if m0 is not None and not 5.0 <= m0 <= 30.0:
        reasons.append(f"liftoff mass {m0:.2f} kg is outside the compared 7-24 kg class")
    return "; ".join(reasons) if reasons else None


def summary_text(rec: FlightRecord) -> str:
    m = rec.meta
    lines = [
        f"simulation {m.simulation_id}  seed={m.seed}  fidelity={m.fidelity}"
        f"{' (fast)' if m.fast else ''}  dt={m.dt:g}s  integrator={m.integrator}  status={m.status}",
        f"simulator {m.sim_version}  physics {m.physics_version}  schema {m.schema_version}  "
        f"({m.n_steps} steps, {m.wall_time_s:.2f} s wall)",
        "",
    ]
    width = max(len(a) for a, _ in summary_rows(rec))
    for label, val in summary_rows(rec):
        lines.append(f"  {label:<{width}}  {val}")
    if rec.meta.fidelity >= 2:
        lines.append("")
        lines.append("  " + provisional_note())
        lines.append(
            "  uncertainties are extrapolated from 7 compared 7-24 kg flights (3 in-sample) (docs/validation.md); "
            "this vehicle itself is not validated"
        )
    out = outside_envelope(rec)
    if out:
        lines.append(f"  * uncertainty bands are EXTRAPOLATED here: {out}")
    for w in m.warnings:
        lines.append(f"  WARNING: {w}")
    for n in m.notes:
        lines.append(f"  note: {n}")
    return "\n".join(lines)
