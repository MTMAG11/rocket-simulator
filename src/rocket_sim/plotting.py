"""Matplotlib figures for flight records (used by the CLI, validation reports and the GUI)."""

from __future__ import annotations

import numpy as np
from matplotlib.figure import Figure

from .constants import G0
from .simulation.record import FlightRecord

# Okabe-Ito colour-blind-safe palette
COLORS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9", "#000000"]

EVENT_STYLE = {
    "rail_exit": ("rail exit", "#999999"),
    "burnout": ("burnout", "#D55E00"),
    "apogee": ("apogee", "#009E73"),
    "parachute_deploy": ("chute", "#CC79A7"),
    "landing": ("landing", "#000000"),
}


def mark_events(ax, rec: FlightRecord, labels: bool = False) -> None:
    ymax = ax.get_ylim()[1]
    for e in rec.events:
        if e.name in EVENT_STYLE:
            text, col = EVENT_STYLE[e.name]
            ax.axvline(e.t, color=col, lw=0.8, ls=":", alpha=0.8)
            if labels:
                ax.text(e.t, ymax, text, rotation=90, va="top", ha="right", fontsize=7, color=col)


def plot_overview(rec: FlightRecord) -> Figure:
    t = rec.col("t")
    fig = Figure(figsize=(13, 9), layout="constrained")
    axs = fig.subplots(3, 3)
    a = axs.ravel()

    def panel(i, ys, title, ylabel):
        ax = a[i]
        for lab, y, col in ys:
            ax.plot(t, y, color=col, lw=1.2, label=lab)
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylabel)
        ax.set_xlabel("time [s]")
        ax.grid(alpha=0.3)
        if len(ys) > 1:
            ax.legend(fontsize=7, frameon=False)
        mark_events(ax, rec)

    panel(0, [("altitude AGL", rec.col("altitude"), COLORS[0])], "Altitude", "m")
    panel(
        1,
        [("speed", rec.col("speed"), COLORS[0]), ("vertical", rec.col("vel_z"), COLORS[1])],
        "Velocity",
        "m/s",
    )
    sf = np.linalg.norm(np.stack([rec.col("acc_x"), rec.col("acc_y"), rec.col("acc_z") + G0]), axis=0) / G0
    panel(2, [("|specific force|", sf, COLORS[0])], "Load factor", "g")
    panel(3, [("thrust", rec.col("thrust"), COLORS[1])], "Thrust", "N")
    panel(4, [("mass", rec.col("mass"), COLORS[2])], "Mass", "kg")
    panel(
        5,
        [("drag", rec.col("drag"), COLORS[1]), ("lift", rec.col("lift"), COLORS[0])],
        "Aerodynamic forces",
        "N",
    )
    panel(6, [("Mach", rec.col("mach"), COLORS[3])], "Mach number", "-")
    panel(
        7,
        [
            ("AoA", np.degrees(rec.col("aoa")), COLORS[4]),
            ("static margin", rec.col("static_margin"), COLORS[5]),
        ],
        "Angle of attack [deg] / static margin [cal]",
        "",
    )
    ax = a[8]
    ax.plot(rec.col("pos_x"), rec.col("pos_z"), color=COLORS[0], lw=1.2, label="x-z (E-up)")
    ax.plot(rec.col("pos_y"), rec.col("pos_z"), color=COLORS[1], lw=1.2, label="y-z (N-up)")
    ax.set_title("Flight path", fontsize=10)
    ax.set_xlabel("horizontal displacement [m]")
    ax.set_ylabel("height [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, frameon=False)
    fig.suptitle(
        f"{rec.meta.simulation_id}  (fidelity {rec.meta.fidelity}, dt {rec.meta.dt:g} s)", fontsize=11
    )
    return fig
