"""Markdown report and plots for the Q-01 trade study."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml
from matplotlib.figure import Figure

from .tvc_trade import MotorResult, StudySpec

COLORS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]


def _fmt(v: float | None, spec: str = ".3g", unit: str = "") -> str:
    return "n/a" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{v:{spec}}{unit}"


def comparison_sentence(results: list[MotorResult]) -> str:
    """What the success rates support, and nothing more."""
    if len(results) < 2:
        return ""
    a, b = sorted(results, key=lambda r: r.rate, reverse=True)[:2]
    (alo, ahi), (blo, bhi) = a.interval, b.interval
    n = a.runs
    if alo > bhi:
        return (
            f"{a.motor} ({a.rate:.0%}) outperforms {b.motor} ({b.rate:.0%}) in this simulation: the 95 % intervals do not overlap "
            f"({alo:.0%}-{ahi:.0%} vs {blo:.0%}-{bhi:.0%}, {n} runs each), for the dispersions and the success definition above."
        )
    return (
        f"The success rates ({a.motor} {a.rate:.0%}, {b.motor} {b.rate:.0%}) are not distinguishable with {n} runs each: the 95 % "
        f"intervals overlap ({alo:.0%}-{ahi:.0%} and {blo:.0%}-{bhi:.0%}). More runs, or a harder dispersion set, would be needed to separate them."
    )


def figures(results: list[MotorResult], spec: StudySpec) -> dict[str, Figure]:
    out: dict[str, Figure] = {}
    fig = Figure(figsize=(7.5, 6.5), layout="constrained")
    ax1, ax2 = fig.subplots(2, 1, sharex=False)
    for i, r in enumerate(results):
        c = COLORS[i % len(COLORS)]
        ax1.plot(r.t - r.t[0], r.thrust, color=c, label=r.motor)
        if r.max_torque is not None:
            ax2.plot(r.t - r.t[0], r.max_torque, color=c, label=r.motor)
    ax1.set(
        xlabel="time since rail exit [s]", ylabel="thrust [N]", title="Thrust after rail exit (nominal run)"
    )
    ax2.set(
        xlabel="time since rail exit [s]",
        ylabel="max gimbal torque [N m]",
        title="Largest correctable torque at the gimbal limit",
    )
    if spec.reference_disturbance_torque_nm is not None:
        ax2.axhline(
            spec.reference_disturbance_torque_nm, color="k", ls="--", lw=1, label="reference disturbance"
        )
    if not any(r.max_torque is not None for r in results):
        ax2.text(
            0.5,
            0.5,
            "max_correctable_torque() is not implemented yet",
            ha="center",
            va="center",
            transform=ax2.transAxes,
        )
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        if ax.get_legend_handles_labels()[0]:
            ax.legend()
    out["authority.png"] = fig

    fig = Figure(figsize=(7.5, 6.5), layout="constrained")
    ax1, ax2 = fig.subplots(2, 1)
    bins = np.linspace(
        0.0,
        max(
            float(np.nanmax(np.concatenate([r.tilts for r in results] + [r.passive_tilts for r in results]))),
            1.0,
        ),
        24,
    )
    for i, r in enumerate(results):
        c = COLORS[i % len(COLORS)]
        ax1.hist(
            r.tilts[np.isfinite(r.tilts)], bins=bins, alpha=0.55, color=c, label=f"{r.motor} (controlled)"
        )
        ax1.hist(
            r.passive_tilts[np.isfinite(r.passive_tilts)],
            bins=bins,
            histtype="step",
            color=c,
            ls="--",
            label=f"{r.motor} (no controller)",
        )
    ax1.axvline(float(spec.success["max_tilt_deg"]), color="k", lw=1.2, label="success limit")
    ax1.set(
        xlabel="max tilt from vertical, rail exit to burnout + settle [deg]",
        ylabel="runs",
        title="Attitude excursion over the Monte Carlo runs",
    )
    ax1.legend(fontsize=8)
    for i, r in enumerate(results):
        c = COLORS[i % len(COLORS)]
        edges = np.linspace(0.0, max(float(r.winds.max()), 1.0) + 1e-9, 5)
        idx = np.clip(np.digitize(r.winds, edges) - 1, 0, 3)
        xs, ys = [], []
        for k in range(4):
            sel = idx == k
            if sel.sum() >= 1:
                xs.append(0.5 * (edges[k] + edges[k + 1]))
                ys.append(float(r.ok[sel].mean()))
        ax2.plot(xs, ys, "o-", color=c, label=r.motor)
    ax2.set(
        xlabel="sampled wind speed [m/s]",
        ylabel="success fraction",
        ylim=(-0.05, 1.05),
        title="Success vs wind (4 bins; few runs per bin)",
    )
    ax2.grid(alpha=0.3)
    ax2.legend()
    out["monte_carlo.png"] = fig
    return out


def write_report(
    spec: StudySpec, results: list[MotorResult], out_dir: str | Path, title: str = "Q-01: motor trade study"
) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for name, fig in figures(results, spec).items():
        fig.savefig(out / name, dpi=120)
    d = spec.disturbances
    s = spec.success
    lines = [
        f"# {title}",
        "",
        "Simulation study. Every physical input below was supplied in the study file; the simulator invented none of them. "
        "The comparison holds only for these inputs and the success definition.",
        "",
        "## Deterministic results (nominal run, no wind)",
        "",
        "| | " + " | ".join(r.motor for r in results) + " |",
        "|---|" + "---|" * len(results),
        "| rail exit speed [m/s] | " + " | ".join(_fmt(r.rail_exit_speed_ms, ".1f") for r in results) + " |",
        "| peak thrust / liftoff weight | "
        + " | ".join(_fmt(r.thrust_to_weight, ".1f") for r in results)
        + " |",
        "| burn time [s] | " + " | ".join(_fmt(r.burn_time_s, ".2f") for r in results) + " |",
        "| TVC authority time, rail exit to burnout [s] | "
        + " | ".join(_fmt(r.authority_time_s, ".2f") for r in results)
        + " |",
        "| smallest max gimbal torque after rail exit, thrust above 10 % of peak [N m] | "
        + " | ".join(_fmt(r.torque_min_nm, ".3f") for r in results)
        + " |",
        "| max gimbal torque at rail exit [N m] | "
        + " | ".join(_fmt(r.torque_at_rail_exit_nm, ".3f") for r in results)
        + " |",
        "| smallest max angular acceleration [rad/s^2] | "
        + " | ".join(_fmt(r.max_ang_accel, ".1f") for r in results)
        + " |",
    ]
    if spec.reference_disturbance_torque_nm is not None:
        lines.append(
            f"| time with gimbal torque >= {spec.reference_disturbance_torque_nm:g} N m [s] | "
            + " | ".join(_fmt(r.time_with_margin_s, ".2f") for r in results)
            + " |"
        )
    if all(r.max_torque is None for r in results):
        lines += [
            "",
            "*The torque rows are n/a: `max_correctable_torque()` (the project owner's TODO) is not implemented yet.*",
        ]
    lines += [
        "",
        f"## Closed-loop Monte Carlo ({spec.runs} runs per motor, seed {spec.monte_carlo['seed']})",
        "",
        f"Success: nose within {s['max_tilt_deg']:g} deg of vertical from rail exit to {spec.settle_s:g} s after burnout"
        + (
            f" and rail exit speed >= {s['min_rail_exit_speed_ms']:g} m/s"
            if s.get("min_rail_exit_speed_ms") is not None
            else ""
        )
        + ". Each run draws: wind speed from a normal distribution "
        + f"(mean {d['wind_speed_mean_ms']:g}, sd {d['wind_speed_std_ms']:g} m/s, clipped at 0) with a random direction; a thrust misalignment of "
        + f"{d['thrust_misalignment_deg']:g} deg in a random direction; a CG shift of {d['cg_error_m'] * 1000:g} mm with random sign, made by moving "
        + f"'{d['cg_shift_item']}'; sensor noise from the seed. Estimator: {spec.estimator}.",
        "",
        "| | " + " | ".join(r.motor for r in results) + " |",
        "|---|" + "---|" * len(results),
        "| successes (controlled) | " + " | ".join(f"{r.successes}/{r.runs}" for r in results) + " |",
        "| success rate, 95 % Wilson interval | "
        + " | ".join(f"{r.rate:.0%} ({r.interval[0]:.0%}-{r.interval[1]:.0%})" for r in results)
        + " |",
        "| successes with no controller, same disturbances | "
        + " | ".join(f"{r.passive_successes}/{r.runs}" for r in results)
        + " |",
        "| failure reasons (controlled) | "
        + " | ".join(", ".join(f"{k}: {v}" for k, v in r.failures.items()) or "none" for r in results)
        + " |",
        "| median max tilt [deg] | "
        + " | ".join(_fmt(float(np.nanmedian(r.tilts)), ".1f") for r in results)
        + " |",
        "",
        comparison_sentence(results),
        "",
        "![authority](authority.png)",
        "",
        "![monte carlo](monte_carlo.png)",
        "",
        "## Inputs used",
        "",
        "```yaml",
        yaml.safe_dump(
            {
                "base_config": spec.base_config,
                "motors": spec.motors,
                "added_masses": spec.added_masses,
                "tvc": spec.tvc,
                "controller": spec.controller,
                "estimator": spec.estimator,
                "disturbances": spec.disturbances,
                "success": spec.success,
                "settle_s": spec.settle_s,
                "reference_disturbance_torque_nm": spec.reference_disturbance_torque_nm,
            },
            sort_keys=False,
        ).rstrip(),
        "```",
        "",
        "## What this does not tell you",
        "",
        "* Anything outside the listed dispersions (a jammed servo, a gust beyond the sampled wind, structural failure).",
        "* With few runs the intervals are wide (see the table); a small difference is not a result.",
        "* The servo model, the added masses and the success limits are inputs; with placeholders the study says nothing about the real vehicle.",
        "* Method and definitions: docs/derivations/tvc_authority.md.",
        "",
    ]
    p = out / "report.md"
    p.write_text("\n".join(lines), encoding="utf-8")
    return p
