"""Simulation-vs-real-flight comparison with quantitative metrics, plots and a text report.

Procedure (see docs/validation.md):
 1. Load the real telemetry through the declarative column map of a flight definition YAML.
 2. Simulate the reconstructed flight (inputs documented in the YAML / sim config).
 3. Align time: the simulation is shifted so that it crosses ``launch_threshold_altitude_m``
    at the same instant as the real flight. This removes only the unknown launch-detect/ignition
    offset; it does NOT hide timing errors later in the flight.
 4. Metrics: RMSE / MAE / max-abs / bias of altitude (ascent, descent, all), velocity and axial
    acceleration series; percentage error of apogee, max velocity, max acceleration; timing error
    of apogee, burnout (zero crossing of axial acceleration) and landing.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from ..config import config_from_dict, read_mapping
from ..errors import ConfigError
from ..simulation import run_simulation
from ..simulation.record import FlightRecord
from .metrics import crossing_time, scalar_error, series_error, smooth_derivative
from .telemetry import Telemetry, load_telemetry, telemetry_from_record


@dataclass
class ValidationResult:
    name: str
    metrics: dict[str, Any]
    record: FlightRecord
    real: Telemetry
    sim: Telemetry
    shift_s: float
    definition: dict[str, Any]
    figure: Any = None
    notes: list[str] = field(default_factory=list)

    def report_text(self) -> str:
        m = self.metrics
        lines = [
            f"Validation: {self.name}",
            f"  source: {self.definition.get('source', {}).get('name', '?')}  "
            f"(quality: {self.definition.get('source', {}).get('quality', '?')})",
            f"  simulation fidelity {self.record.meta.fidelity}, dt {self.record.meta.dt:g} s, "
            f"time shift applied {self.shift_s:+.3f} s",
            "",
        ]
        for key, label, unit in (
            ("apogee", "Apogee (AGL)", "m"),
            ("apogee_time", "Apogee time", "s"),
            ("max_velocity", "Max velocity", "m/s"),
            ("max_accel", "Max axial accel", "m/s^2"),
            ("burnout_time", "Burnout (accel zero-cross)", "s"),
            ("landing_time", "Landing time", "s"),
        ):
            if key in m:
                e = m[key]
                lines.append(
                    f"  {label:<28} real {e['real']:9.2f}  sim {e['sim']:9.2f} {unit:<6} "
                    f"error {e['abs_error']:+8.2f} ({e['pct_error']:+6.2f} %)"
                )
        for key, label in (
            ("altitude_all", "altitude (all)"),
            ("altitude_ascent", "altitude (ascent)"),
            ("altitude_descent", "altitude (descent)"),
            ("velocity", "velocity"),
            ("accel_axial", "axial accel"),
        ):
            if key in m:
                e = m[key]
                lines.append(
                    f"  {label:<20} n={e['n']:5d}  RMSE {e['rmse']:8.3f}  MAE {e['mae']:8.3f}  "
                    f"max|e| {e['max_abs']:8.3f}  bias {e['bias']:+8.3f}  NRMSE {e['nrmse_pct']:5.2f} %"
                )
        for n in self.notes:
            lines.append(f"  note: {n}")
        return "\n".join(lines)


def _interp(t_src: np.ndarray, y_src: np.ndarray, t: np.ndarray) -> np.ndarray:
    return np.interp(t, t_src, y_src)


def compare_flight(real: Telemetry, rec: FlightRecord, definition: dict[str, Any]) -> ValidationResult:
    sim = telemetry_from_record(rec)
    tdef = definition.get("telemetry", {})
    notes: list[str] = []
    ra = real["altitude"]
    # real altimeters are barometric: compare with the barometer-equivalent altitude of the simulation
    # (default); "geometric" compares true height instead
    alt_ref = tdef.get("altitude_reference", "barometric")
    sa = sim["altitude_baro"] if alt_ref == "barometric" else sim["altitude"]
    sim.channels["altitude_used"] = sa
    notes.append(f"simulated altitude compared as: {alt_ref}")
    # time alignment channel: altitude by default; a velocity-type channel when the real altitude is a
    # lagging barometric estimate (see the Prometheus definition)
    ach = tdef.get("align_channel", "altitude")
    thr = float(tdef.get("align_threshold", tdef.get("launch_threshold_altitude_m", 15.0)))
    sim_ch = sa if ach == "altitude" else sim[ach]
    tr = crossing_time(real[ach].t, real[ach].y, thr, True)
    ts = crossing_time(sim_ch.t, sim_ch.y, thr, True)
    if tr is None or ts is None:
        raise ConfigError(f"{ach} never crosses the {thr} alignment threshold (real: {tr}, sim: {ts})")
    shift = tr - ts
    t_sim = sa.t + shift

    metrics: dict[str, Any] = {
        "alignment": {"channel": ach, "threshold": thr, "t_real": tr, "t_sim": ts, "shift_s": shift}
    }
    # --- apogee, landing -------------------------------------------------------------------
    i_r = int(np.argmax(ra.y))
    i_s = int(np.argmax(sa.y))
    metrics["apogee"] = scalar_error(float(ra.y[i_r]), float(sa.y[i_s]))
    metrics["apogee_time"] = scalar_error(
        float(ra.t[i_r] - tr), float(sa.t[i_s] - ts)
    )  # since threshold crossing
    ground = float(definition.get("telemetry", {}).get("landing_altitude_m", 3.0))
    t_land_r = crossing_time(ra.t, ra.y, ground, False, start=i_r)
    land = rec.event("landing")
    if t_land_r is not None and land is not None:
        metrics["landing_time"] = scalar_error(float(t_land_r - tr), float(land.t - ts))
    else:
        notes.append("real record ends before landing: landing time not compared")

    # --- altitude series -------------------------------------------------------------------
    t_end = min(ra.t[-1], t_sim[-1])
    m_all = (ra.t >= tr - 1.0) & (ra.t <= t_end)
    sim_on_r = _interp(t_sim, sa.y, ra.t)
    metrics["altitude_all"] = series_error(ra.y[m_all], sim_on_r[m_all]).to_dict()
    m_asc = m_all & (ra.t <= ra.t[i_r])
    m_des = m_all & (ra.t > ra.t[i_r])
    if m_asc.sum() > 3:
        metrics["altitude_ascent"] = series_error(ra.y[m_asc], sim_on_r[m_asc]).to_dict()
    if m_des.sum() > 3:
        metrics["altitude_descent"] = series_error(ra.y[m_des], sim_on_r[m_des]).to_dict()

    # --- velocity -----------------------------------------------------------------------------
    vname = "velocity_z" if real.has("velocity_z") else ("speed" if real.has("speed") else None)
    if vname is not None:
        rv = real[vname]
        rt, ry = rv.t, rv.y
    else:
        rt, ry = ra.t, smooth_derivative(ra.t, ra.y, 0.6)
        notes.append("real velocity derived from altitude (0.6 s local quadratic fit); not independent data")
    if vname == "speed":
        notes.append("real speed channel is TOTAL speed: compared with the simulated total speed")
    ok = np.isfinite(ry) & (rt >= tr - 1.0) & (rt <= t_end) & (rt <= ra.t[i_r])  # ascent
    if ok.sum() > 3:
        if vname is not None:
            sv = _interp(t_sim, sim[vname].y, rt)
            sim_vmax = float(np.max(sim[vname].y))
        else:  # apply the SAME derivative operator to the simulated altitude at the real sample times
            sv = smooth_derivative(rt, _interp(t_sim, sa.y, rt), 0.6)
            sim_vmax = float(np.nanmax(sv[ok]))
        metrics["velocity"] = series_error(ry[ok], sv[ok]).to_dict()
        metrics["max_velocity"] = scalar_error(float(np.max(ry[ok])), sim_vmax)

    # --- axial acceleration -------------------------------------------------------------------
    if real.has("accel_axial"):
        rac = real["accel_axial"]
        dt = float(np.median(np.diff(rac.t)))
        w = max(int(round(0.05 / dt)), 1)
        ker = np.ones(w) / w
        ys = np.convolve(rac.y, ker, mode="same")  # 50 ms boxcar (accelerometer is noisy)
        # axial accel time base is independent of the altimeter's: align with the same shift via
        # the first-motion time of the accelerometer (specific force exceeding 2 g)
        i0 = int(np.argmax(ys > 2.0 * 9.80665))
        t_r0 = float(rac.t[i0])
        sim_ac = sim["accel_axial"]
        j0 = int(np.argmax(sim_ac.y > 2.0 * 9.80665))
        t_s0 = float(sim_ac.t[j0])
        sh_a = t_r0 - t_s0
        win = (rac.t >= t_r0 - 0.2) & (rac.t <= t_r0 + float(rec.summary.get("burnout_time_s") or 5.0) + 2.0)
        sa_on = np.convolve(_interp(sim_ac.t + sh_a, sim_ac.y, rac.t), ker, mode="same")  # same 50 ms boxcar
        metrics["accel_axial"] = series_error(ys[win], sa_on[win]).to_dict()
        metrics["max_accel"] = scalar_error(float(ys[win].max()), float(sa_on[win].max()))
        # burnout: first falling zero crossing after the peak
        ipk = int(np.argmax(np.where(win, ys, -1e9)))
        tb_r = crossing_time(rac.t[ipk:], ys[ipk:], 0.0, False)
        jpk = int(np.argmax(np.where(win, sa_on, -1e9)))
        tb_s = crossing_time(rac.t[jpk:], sa_on[jpk:], 0.0, False)
        if tb_r is not None and tb_s is not None:
            metrics["burnout_time"] = scalar_error(float(tb_r - t_r0), float(tb_s - t_r0))
        metrics["accel_alignment_shift_s"] = sh_a
        notes.append(
            "axial-accel time base aligned independently (first >2 g sample); "
            f"differs from the altimeter alignment by {sh_a - shift:+.3f} s"
        )

    res = ValidationResult(
        definition.get("name", "flight"), metrics, rec, real, sim, shift, definition, None, notes
    )
    return res


def plot_comparison(res: ValidationResult):
    from matplotlib.figure import Figure

    real, sim, shift = res.real, res.sim, res.shift_s
    n = 3 if real.has("accel_axial") else 2
    fig = Figure(figsize=(11, 3.4 * n), layout="constrained")
    axs = fig.subplots(n, 1, sharex=False)
    ra, sa = real["altitude"], sim["altitude_used"]
    tr = res.metrics["alignment"]["t_real"]
    ax = axs[0]
    ax.plot(ra.t - tr, ra.y, ".", ms=2.5, color="#D55E00", label="real")
    ax.plot(sa.t + shift - tr, sa.y, color="#0072B2", lw=1.3, label="simulation")
    ax.set_ylabel("altitude AGL [m]")
    ax.legend(frameon=False)
    ax.grid(alpha=0.3)
    ax.set_title(res.name)
    ax = axs[1]
    vkey2 = "velocity_z" if real.has("velocity_z") or not real.has("speed") else "speed"
    if real.has("velocity_z") or real.has("speed"):
        rk = "velocity_z" if real.has("velocity_z") else "speed"
        ax.plot(real[rk].t - tr, real[rk].y, ".", ms=2.5, color="#D55E00", label="real")
    else:
        ax.plot(
            ra.t - tr,
            smooth_derivative(ra.t, ra.y, 0.6),
            ".",
            ms=2.5,
            color="#D55E00",
            label="real (derived)",
        )
    ax.plot(sim[vkey2].t + shift - tr, sim[vkey2].y, color="#0072B2", lw=1.3, label="simulation")
    ax.set_ylabel("vertical velocity [m/s]")
    ax.legend(frameon=False)
    ax.grid(alpha=0.3)
    if n == 3:
        ax = axs[2]
        rac = real["accel_axial"]
        sh = res.metrics["accel_alignment_shift_s"]
        ax.plot(rac.t - tr - (sh - shift), rac.y / 9.80665, color="#D55E00", lw=0.6, alpha=0.7, label="real")
        ax.plot(
            sim["accel_axial"].t + shift - tr,
            sim["accel_axial"].y / 9.80665,
            color="#0072B2",
            lw=1.3,
            label="simulation",
        )
        ax.set_xlim(-1, float(res.record.summary.get("burnout_time_s") or 5) + 4)
        ax.set_ylabel("axial specific force [g]")
        ax.legend(frameon=False)
        ax.grid(alpha=0.3)
    axs[-1].set_xlabel("time since altitude threshold crossing [s]")
    return fig


def load_flight_definition(path: str | Path) -> tuple[dict[str, Any], Path]:
    p = Path(path)
    d = read_mapping(p)
    for k in ("name", "sim_config", "telemetry"):
        if k not in d:
            raise ConfigError(f"{p.name}: missing key '{k}'")
    return d, p.parent


def run_validation(
    flight_yaml: str | Path,
    overrides: dict[str, Any] | None = None,
    out_dir: str | Path | None = None,
    plot: bool = True,
    seed: int = 0,
) -> ValidationResult:
    d, base = load_flight_definition(flight_yaml)
    tel = d["telemetry"]
    real = load_telemetry(
        base / tel["file"],
        tel["channels"],
        tel.get("delimiter", ","),
        {"source": d.get("source", {}), "name": d["name"]},
    )
    sim_path = base / d["sim_config"]
    raw = read_mapping(sim_path)
    from ..config import apply_overrides

    cfg = config_from_dict(
        apply_overrides(raw, {**d.get("sim_overrides", {}), **(overrides or {})}), base_dir=sim_path.parent
    )
    rec = run_simulation(cfg, seed=seed)
    res = compare_flight(real, rec, d)
    if plot:
        res.figure = plot_comparison(res)
    if out_dir:
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        (out / f"{d['name']}_metrics.json").write_text(json.dumps(res.metrics, indent=2), encoding="utf-8")
        (out / f"{d['name']}_report.txt").write_text(res.report_text(), encoding="utf-8")
        if res.figure is not None:
            res.figure.savefig(out / f"{d['name']}.png", dpi=130)
    return res
