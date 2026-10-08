"""Graphs tab: pick named channels from a grouped list, stacked plots with units, events and a time cursor."""

from __future__ import annotations

from typing import Any

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT
from matplotlib.figure import Figure
from PySide6 import QtWidgets
from PySide6.QtCore import Qt

from ..plotting import EVENT_STYLE
from ..simulation.record import FlightRecord
from .channels import PRESETS, Channel, available_channels, preset_ids

BG, PANEL, FG, GRID = "#1e1e1e", "#252526", "#e0e0e0", "#4a4a4a"
LINE_COLORS = ["#4fc1ff", "#ff8c42", "#6adf8a", "#d78bf0", "#f2c94c", "#8fd3f4"]
TIME_RANGES = ("Full flight", "Until apogee", "Powered flight")
DEFAULT_PRESET = "Flight overview"


class GraphPanel(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.rec: FlightRecord | None = None
        self.channels: dict[str, Channel] = {}
        self.checked: list[str] = []
        self._cursor: list[Any] = []
        self._dots: list[list[Any]] = []
        self._series_t: np.ndarray | None = None
        self._building = False

        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        side = QtWidgets.QVBoxLayout()
        self.preset_box = QtWidgets.QComboBox()
        self.preset_box.addItem("Presets…")
        self.preset_box.addItems(list(PRESETS))
        self.preset_box.activated.connect(self._on_preset)
        self.search = QtWidgets.QLineEdit()
        self.search.setPlaceholderText("Search channels")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self._filter)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.setIndentation(14)
        self.tree.itemChanged.connect(self._on_item_changed)
        self.clear_button = QtWidgets.QPushButton("Clear selection")
        self.clear_button.clicked.connect(lambda: self._set_checked([]))
        side.addWidget(self.preset_box)
        side.addWidget(self.search)
        side.addWidget(self.tree, 1)
        side.addWidget(self.clear_button)
        left = QtWidgets.QWidget()
        left.setLayout(side)
        left.setFixedWidth(270)
        side.setContentsMargins(0, 0, 0, 0)
        root.addWidget(left)

        right = QtWidgets.QVBoxLayout()
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Time range:"))
        self.range_box = QtWidgets.QComboBox()
        self.range_box.addItems(TIME_RANGES)
        self.range_box.currentIndexChanged.connect(self.redraw)
        bar.addWidget(self.range_box)
        bar.addStretch(1)
        self.hint = QtWidgets.QLabel("Select channels on the left to plot them.")
        bar.addWidget(self.hint)
        right.addLayout(bar)
        self.figure = Figure(facecolor=BG, layout="constrained")
        self.canvas = FigureCanvas(self.figure)
        toolbar = NavigationToolbar2QT(self.canvas, self)
        right.addWidget(toolbar)
        right.addWidget(self.canvas, 1)
        root.addLayout(right, 1)

    # ----------------------------------------------------------------------------- data
    def set_record(self, rec: FlightRecord) -> None:
        self.rec = rec
        chans = available_channels(rec)
        self.channels = {c.id: c for c in chans}
        keep = [i for i in self.checked if i in self.channels]
        self._building = True
        self.tree.clear()
        groups: dict[str, QtWidgets.QTreeWidgetItem] = {}
        for c in chans:
            g = groups.get(c.group)
            if g is None:
                g = QtWidgets.QTreeWidgetItem([c.group])
                g.setFlags(Qt.ItemFlag.ItemIsEnabled)
                f = g.font(0)
                f.setBold(True)
                g.setFont(0, f)
                self.tree.addTopLevelItem(g)
                groups[c.group] = g
            label = c.title + (f"  ({c.unit})" if c.unit else "")
            it = QtWidgets.QTreeWidgetItem([label])
            it.setData(0, Qt.ItemDataRole.UserRole, c.id)
            it.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsUserCheckable)
            it.setCheckState(0, Qt.CheckState.Unchecked)
            if c.tip:
                it.setToolTip(0, c.tip)
            g.addChild(it)
        for name, g in groups.items():
            g.setExpanded(name in ("Trajectory", "Velocity", "Acceleration", "Attitude", "Forces"))
        self._building = False
        self._set_checked(keep or preset_ids(DEFAULT_PRESET, chans))

    def _groups(self) -> list[QtWidgets.QTreeWidgetItem]:
        return [
            g for i in range(self.tree.topLevelItemCount()) if (g := self.tree.topLevelItem(i)) is not None
        ]

    @staticmethod
    def _children(g: QtWidgets.QTreeWidgetItem) -> list[QtWidgets.QTreeWidgetItem]:
        return [c for j in range(g.childCount()) if (c := g.child(j)) is not None]

    def _items(self) -> list[QtWidgets.QTreeWidgetItem]:
        return [c for g in self._groups() for c in self._children(g)]

    def _set_checked(self, ids: list[str]) -> None:
        self._building = True
        for it in self._items():
            it.setCheckState(
                0,
                Qt.CheckState.Checked
                if it.data(0, Qt.ItemDataRole.UserRole) in ids
                else Qt.CheckState.Unchecked,
            )
        self._building = False
        self.checked = list(ids)
        self.redraw()

    def _on_item_changed(self, item: QtWidgets.QTreeWidgetItem, _col: int) -> None:
        if self._building:
            return
        cid = item.data(0, Qt.ItemDataRole.UserRole)
        if item.checkState(0) == Qt.CheckState.Checked:
            if cid not in self.checked:
                self.checked.append(cid)
        elif cid in self.checked:
            self.checked.remove(cid)
        self.redraw()

    def _on_preset(self, index: int) -> None:
        if index > 0:
            self._set_checked(preset_ids(self.preset_box.currentText(), list(self.channels.values())))
        self.preset_box.setCurrentIndex(0)

    def _filter(self, text: str) -> None:
        text = text.strip().lower()
        for g in self._groups():
            shown = 0
            for c in self._children(g):
                hit = not text or text in c.text(0).lower() or text in g.text(0).lower()
                c.setHidden(not hit)
                shown += hit
            g.setHidden(shown == 0)
            if text and shown:
                g.setExpanded(True)

    # ---------------------------------------------------------------------------- drawing
    def _time_limit(self) -> float | None:
        rec = self.rec
        assert rec is not None
        ev = {e.name: e.t for e in rec.events}
        mode = self.range_box.currentText()
        if mode == "Until apogee" and "apogee" in ev:
            return ev["apogee"] * 1.05
        if mode == "Powered flight" and "burnout" in ev:
            return max(ev["burnout"] * 1.5, ev["burnout"] + 1.0)
        return None

    def redraw(self) -> None:
        fig = self.figure
        fig.clear()
        self._cursor, self._dots = [], []
        rec = self.rec
        chans = [self.channels[i] for i in self.checked if i in self.channels]
        if rec is None or not chans:
            self.hint.setText(
                "Select channels on the left to plot them." if rec else "Run a simulation to see graphs."
            )
            self.canvas.draw_idle()
            return
        self.hint.setText("")
        t = rec.col("t")
        tmax = self._time_limit()
        sel = t <= tmax if tmax is not None else np.ones_like(t, dtype=bool)
        self._series_t = t
        axs = np.atleast_1d(fig.subplots(len(chans), 1, sharex=True))
        events = [
            (e.t, e.name) for e in rec.events if e.name in EVENT_STYLE and (tmax is None or e.t <= tmax)
        ]
        for k, (ax, ch) in enumerate(zip(axs, chans)):
            self._style_axes(ax)
            dots = []
            vals = ch.values(rec)
            for n, (label, y) in enumerate(vals):
                color = LINE_COLORS[n % len(LINE_COLORS)]
                ax.plot(t[sel], y[sel], color=color, lw=1.5, label=label)
                (dot,) = ax.plot([t[0]], [y[0]], "o", color=color, ms=5, zorder=5)
                dots.append((dot, y))
            ax.set_title(ch.title, loc="left", fontsize=10, color=FG, fontweight="bold", pad=4)
            ax.set_ylabel(ch.unit, color=FG, fontsize=9)
            if len(vals) > 1:
                leg = ax.legend(loc="upper right", fontsize=8, ncol=min(len(vals), 4), frameon=True)
                leg.get_frame().set_facecolor(PANEL)
                leg.get_frame().set_edgecolor(GRID)
                for txt in leg.get_texts():
                    txt.set_color(FG)
            for te, name in events:
                ax.axvline(te, color=EVENT_STYLE[name][1], lw=0.9, ls=":", alpha=0.9)
                if k == 0:
                    ax.annotate(
                        EVENT_STYLE[name][0],
                        (te, 0.96),
                        xycoords=("data", "axes fraction"),
                        rotation=90,
                        va="top",
                        ha="right",
                        fontsize=7,
                        color=EVENT_STYLE[name][1],
                        annotation_clip=False,
                    )
            self._cursor.append(ax.axvline(t[0], color="#ffffff", lw=1.0, alpha=0.8))
            self._dots.append(dots)
        axs[-1].set_xlabel("Time (s)", color=FG, fontsize=9)
        if tmax is not None:
            axs[-1].set_xlim(t[0], tmax)
        self.canvas.draw_idle()

    @staticmethod
    def _style_axes(ax: Any) -> None:
        ax.set_facecolor(PANEL)
        ax.grid(True, color=GRID, alpha=0.6, lw=0.6)
        ax.tick_params(colors=FG, labelsize=8)
        for s in ax.spines.values():
            s.set_color(GRID)

    def set_index(self, i: int) -> None:
        """Move the time cursor and its markers to sample ``i``."""
        t = self._series_t
        if t is None or not self._cursor:
            return
        for ln in self._cursor:
            ln.set_xdata([t[i], t[i]])
        for dots in self._dots:
            for dot, y in dots:
                dot.set_data([t[i]], [y[i]])
        self.canvas.draw_idle()

    def current_state(self) -> dict[str, Any]:
        return {"channels": list(self.checked), "range": self.range_box.currentText()}
