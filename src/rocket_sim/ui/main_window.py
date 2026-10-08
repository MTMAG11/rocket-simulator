"""Graphical front end (PySide6 + Matplotlib).

The GUI is a thin client of the simulation engine: it builds a config dict, runs the simulation
in a background thread, and visualises the resulting FlightRecord. Large dataset generation is
headless-only (``rocketsim batch``); the dataset browser here reads metadata and reproduces single
runs on demand, so it never loads every timestep.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path
from typing import Any

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT
from matplotlib.figure import Figure
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt

from ..config import apply_overrides, config_from_dict, config_to_dict, load_config
from ..data.export import export_record
from ..data.schema import column, columns_for
from ..errors import RocketSimError
from ..plotting import COLORS, EVENT_STYLE
from ..reporting import summary_text
from ..resources import (
    APP_NAME,
    app_version,
    configs_dir,
    default_config,
    launch_description,
    output_dir,
    resource_root,
)
from ..simulation import FlightPhase, Simulation
from ..simulation.record import FlightRecord
from .catalog import VehicleEntry, list_motors, list_vehicles, save_run_results
from .styles import MAIN_STYLE

DEFAULT_PLOTS = ["altitude", "speed", "thrust"]
PHASE_COLORS = {
    0: "#999999",
    1: "#E69F00",
    2: "#D55E00",
    3: "#CC79A7",
    4: "#0072B2",
    5: "#009E73",
    6: "#56B4E9",
    7: "#000000",
}


class SimWorker(QtCore.QObject):
    finished = QtCore.Signal(object)
    failed = QtCore.Signal(str)

    def __init__(self, cfg_dict: dict[str, Any], base_dir: Path, seed: int) -> None:
        super().__init__()
        self.cfg_dict, self.base_dir, self.seed = cfg_dict, base_dir, seed

    @QtCore.Slot()
    def run(self) -> None:
        try:
            cfg = config_from_dict(self.cfg_dict, base_dir=self.base_dir)
            self.finished.emit(Simulation(cfg, seed=self.seed).run())
        except RocketSimError as exc:
            self.failed.emit(f"{type(exc).__name__}: {exc}")
        except Exception as exc:
            self.failed.emit(f"unexpected {type(exc).__name__}: {exc}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(f"{APP_NAME}  v{app_version()}")
        self.resize(1500, 920)
        self.record: FlightRecord | None = None
        self.results_dir: Path | None = None
        self.base_path = default_config()
        self.base_dict: dict[str, Any] = {}
        self.base_dir = self.base_path.parent
        self._thread: QtCore.QThread | None = None
        self._worker: SimWorker | None = None
        self._build_ui()
        self._select_default_vehicle()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        left = QtWidgets.QWidget()
        left.setMaximumWidth(360)
        lv = QtWidgets.QVBoxLayout(left)

        g = QtWidgets.QGroupBox("1  Vehicle")
        gl = QtWidgets.QVBoxLayout(g)
        self.vehicle_box = QtWidgets.QComboBox()
        for v in list_vehicles():
            self.vehicle_box.addItem(v.label, v)
        self.vehicle_box.currentIndexChanged.connect(self._on_vehicle_chosen)
        self.open_button = QtWidgets.QPushButton("Browse…")
        self.open_button.clicked.connect(self.open_config)
        self.vehicle_info = QtWidgets.QLabel()
        self.vehicle_info.setWordWrap(True)
        gl.addWidget(self.vehicle_box)
        gl.addWidget(self.vehicle_info)
        gl.addWidget(self.open_button)
        lv.addWidget(g)

        g = QtWidgets.QGroupBox("2  Motor")
        gl = QtWidgets.QVBoxLayout(g)
        self.motor_box = QtWidgets.QComboBox()
        self.motor_info: dict[str, str] = {}
        for m in list_motors():
            self.motor_box.addItem(m.label, m.key)
            self.motor_info[m.key] = m.summary
        self.motor_box.currentIndexChanged.connect(self._on_motor_chosen)
        self.motor_label = QtWidgets.QLabel()
        self.motor_label.setWordWrap(True)
        gl.addWidget(self.motor_box)
        gl.addWidget(self.motor_label)
        lv.addWidget(g)

        g = QtWidgets.QGroupBox("3  Conditions")
        f = QtWidgets.QFormLayout(g)
        self.elevation = self._spin(5.0, 90.0, 1, " deg", 1.0)
        self.wind_speed = self._spin(0.0, 40.0, 1, " m/s", 0.5)
        self.wind_dir = self._spin(0.0, 360.0, 0, " deg (from)", 10.0)
        f.addRow("Launch elevation:", self.elevation)
        f.addRow("Wind speed:", self.wind_speed)
        f.addRow("Wind from:", self.wind_dir)
        lv.addWidget(g)

        self.run_button = QtWidgets.QPushButton("4  Run simulation")
        self.run_button.setObjectName("run_button")
        self.run_button.setMinimumHeight(44)
        self.run_button.clicked.connect(self.run_simulation)
        lv.addWidget(self.run_button)

        g = QtWidgets.QGroupBox("5  Results")
        gl = QtWidgets.QVBoxLayout(g)
        self.results_label = QtWidgets.QPlainTextEdit()  # selectable, wraps long paths anywhere
        self.results_label.setReadOnly(True)
        self.results_label.setWordWrapMode(QtGui.QTextOption.WrapMode.WrapAnywhere)
        self.results_label.setMaximumHeight(96)
        self.results_label.setPlainText(
            "No run yet. Each run is saved in a new folder under:\n" + str(output_dir() / "gui_runs")
        )
        self.folder_button = QtWidgets.QPushButton("Open results folder")
        self.folder_button.clicked.connect(self.open_results_folder)
        self.export_button = QtWidgets.QPushButton("Export telemetry…")
        self.export_button.clicked.connect(self.export_record)
        self.export_button.setEnabled(False)
        gl.addWidget(self.results_label)
        gl.addWidget(self.folder_button)
        gl.addWidget(self.export_button)
        lv.addWidget(g)

        # advanced: everything else, collapsed by default
        self.adv_box = QtWidgets.QGroupBox("Advanced settings")
        self.adv_box.setCheckable(True)
        self.adv_box.setChecked(False)
        av = QtWidgets.QVBoxLayout(self.adv_box)
        self.adv_inner = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(self.adv_inner)
        f.setContentsMargins(0, 0, 0, 0)
        self.dry_mass = self._spin(0.001, 500.0, 3, " kg", 0.01)
        self.thrust_scale = self._spin(0.5, 1.5, 3, "", 0.01)
        self.azimuth = self._spin(0.0, 360.0, 1, " deg", 5.0)
        self.temp_offset = self._spin(-40.0, 40.0, 1, " K", 1.0)
        self.site_elev = self._spin(-400.0, 5000.0, 0, " m", 50.0)
        self.fidelity = QtWidgets.QComboBox()
        for i, name in (
            (0, "0  1-D vacuum"),
            (1, "1  3-DOF vacuum"),
            (2, "2  3-DOF + aero"),
            (3, "3  6-DOF"),
            (4, "4  6-DOF + sensors"),
            (5, "5  + estimator/controller"),
            (6, "6  high-fidelity"),
        ):
            self.fidelity.addItem(name, i)
        self.dt = self._spin(0.0005, 0.1, 4, " s", 0.001)
        self.integrator = QtWidgets.QComboBox()
        self.integrator.addItems(["rk4", "midpoint", "euler"])
        self.seed = QtWidgets.QSpinBox()
        self.seed.setRange(0, 2**30)
        f.addRow("Dry mass:", self.dry_mass)
        f.addRow("Thrust scale:", self.thrust_scale)
        f.addRow("Azimuth:", self.azimuth)
        f.addRow("Temp. offset (ISA):", self.temp_offset)
        f.addRow("Site elevation MSL:", self.site_elev)
        f.addRow("Fidelity:", self.fidelity)
        f.addRow("Timestep:", self.dt)
        f.addRow("Integrator:", self.integrator)
        f.addRow("Seed:", self.seed)
        av.addWidget(self.adv_inner)
        self.adv_inner.setVisible(False)
        self.adv_box.toggled.connect(self.adv_inner.setVisible)
        lv.addWidget(self.adv_box)
        lv.addStretch()
        root.addWidget(left)

        right = QtWidgets.QVBoxLayout()
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_start_tab(), "Getting started")
        self.tabs.addTab(self._build_graph_tab(), "Graphs")
        self.tabs.addTab(self._build_3d_tab(), "3D view")
        self.summary_text = QtWidgets.QPlainTextEdit()
        self.summary_text.setReadOnly(True)
        self.summary_text.setStyleSheet("font-family: Consolas, monospace;")
        self.tabs.addTab(self.summary_text, "Summary")
        self.tabs.addTab(self._build_browser_tab(), "Data browser")
        right.addWidget(self.tabs, 1)

        tl = QtWidgets.QHBoxLayout()
        tl.addWidget(QtWidgets.QLabel("Time:"))
        self.slider = QtWidgets.QSlider(Qt.Orientation.Horizontal)
        self.slider.valueChanged.connect(self._on_scrub)
        tl.addWidget(self.slider, 1)
        self.time_label = QtWidgets.QLabel("-")
        self.time_label.setMinimumWidth(160)
        tl.addWidget(self.time_label)
        right.addLayout(tl)
        self.readout = QtWidgets.QLabel("Run a simulation to begin.")
        self.readout.setWordWrap(True)
        right.addWidget(self.readout)
        self.events_label = QtWidgets.QLabel()
        self.events_label.setWordWrap(True)
        right.addWidget(self.events_label)
        root.addLayout(right, 1)
        self.statusBar().showMessage("Ready")
        self.statusBar().addPermanentWidget(QtWidgets.QLabel(f"{APP_NAME} v{app_version()}"))

    def _spin(self, lo: float, hi: float, dec: int, suffix: str, step: float) -> QtWidgets.QDoubleSpinBox:
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(dec)
        s.setSuffix(suffix)
        s.setSingleStep(step)
        return s

    def _build_start_tab(self) -> QtWidgets.QWidget:
        self.start_page = QtWidgets.QTextBrowser()
        self.start_page.setOpenExternalLinks(False)
        self._refresh_start_page()
        return self.start_page

    def _refresh_start_page(self, last_run: str = "") -> None:
        last = f"<h3>Last run</h3><pre>{last_run}</pre>" if last_run else ""
        self.start_page.setHtml(
            f"""
            <h1>{APP_NAME}</h1>
            <p>Version {app_version()} ({launch_description()})</p>
            <ol>
              <li>Vehicle: choose a bundled vehicle or browse for a config / <code>vehicle.json</code>.</li>
              <li>Motor: the vehicle's own motor is preselected.</li>
              <li>Conditions: launch elevation and wind. Other settings are under Advanced.</li>
              <li>Run simulation.</li>
            </ol>
            <p>Data: <code>{resource_root()}</code><br>
               Results: <code>{output_dir() / "gui_runs"}</code></p>
            <p>The bundled vehicles are placeholders, not measured rockets. The model is not validated for small model
               rockets, supersonic flight or sensors on real data.</p>
            {last}
            """
        )

    def _select_default_vehicle(self) -> None:
        """Start on the default example config (so a first run needs no choices), else on the first vehicle."""
        want = default_config().name
        for i in range(self.vehicle_box.count()):
            if self.vehicle_box.itemData(i).path.name == want:
                self.vehicle_box.setCurrentIndex(i)
                break
        self._on_vehicle_chosen(self.vehicle_box.currentIndex())

    def _load_vehicle(self, path: Path) -> None:
        """Load a simulation config or a vehicle file into the controls; invalid files leave the old selection."""
        try:
            if path.suffix.lower() == ".json":
                cfg = config_from_dict(
                    {"config_version": 1, "fidelity": 3, "vehicle_file": path.name}, base_dir=path.parent
                )
            else:
                cfg = load_config(path)
        except (RocketSimError, OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(
                self,
                "Could not load vehicle",
                f"Could not load:\n{path}\n\n{exc}\n\nThe previous vehicle is still selected.",
            )
            return
        self.base_path, self.base_dir = path, path.parent
        self.base_dict = config_to_dict(cfg)
        self._load_controls_from_dict()

    def _on_vehicle_chosen(self, index: int) -> None:
        entry = self.vehicle_box.itemData(index)
        if isinstance(entry, VehicleEntry):
            self.vehicle_info.setText(entry.description)
            self._load_vehicle(entry.path)

    def _on_motor_chosen(self, _index: int = 0) -> None:
        self.motor_label.setText(self.motor_info.get(self.motor_box.currentData(), "custom motor file"))

    def _build_graph_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(w)
        self.var_list = QtWidgets.QListWidget()
        self.var_list.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.var_list.setMaximumWidth(250)
        self.var_list.itemSelectionChanged.connect(self.update_graph)
        h.addWidget(self.var_list)
        v = QtWidgets.QVBoxLayout()
        self.figure = Figure(layout="constrained")
        self.canvas = FigureCanvas(self.figure)
        v.addWidget(NavigationToolbar2QT(self.canvas, w))  # zoom / pan / home
        v.addWidget(self.canvas, 1)
        h.addLayout(v, 1)
        self.cursor_lines: list[Any] = []
        return w

    def _build_3d_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.fig3d = Figure(layout="constrained")
        self.canvas3d = FigureCanvas(self.fig3d)
        v.addWidget(NavigationToolbar2QT(self.canvas3d, w))
        v.addWidget(self.canvas3d, 1)
        self.ax3d = self.fig3d.add_subplot(111, projection="3d")
        self._rocket_line = None
        self._wind_arrow = None
        return w

    def _build_browser_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        top = QtWidgets.QHBoxLayout()
        self.ds_button = QtWidgets.QPushButton("Open dataset directory…")
        self.ds_button.clicked.connect(self.open_dataset)
        self.ds_label = QtWidgets.QLabel("No dataset open")
        self.plot_run_button = QtWidgets.QPushButton("Reproduce and plot selected run")
        self.plot_run_button.clicked.connect(self.plot_selected_run)
        self.plot_run_button.setEnabled(False)
        top.addWidget(self.ds_button)
        top.addWidget(self.ds_label, 1)
        top.addWidget(self.plot_run_button)
        v.addLayout(top)
        split = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        self.runs_table = QtWidgets.QTableWidget()
        self.runs_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.runs_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.runs_table.itemSelectionChanged.connect(self._on_run_selected)
        self.run_detail = QtWidgets.QPlainTextEdit()
        self.run_detail.setReadOnly(True)
        self.run_detail.setStyleSheet("font-family: Consolas, monospace;")
        split.addWidget(self.runs_table)
        split.addWidget(self.run_detail)
        v.addWidget(split, 1)
        self.dataset_dir: Path | None = None
        return w

    def _load_controls_from_dict(self) -> None:
        d = self.base_dict
        self.statusBar().showMessage(f"Vehicle loaded: {d['name']} ({self.base_path.name})")
        file = d["motor"]["file"].replace("\\", "/")
        idx = self.motor_box.findData(file)
        if idx < 0:  # the vehicle's own motor file lies outside the bundled list: keep it selectable
            self.motor_box.addItem(Path(file).stem + " (from vehicle)", file)
            idx = self.motor_box.count() - 1
        self.motor_box.setCurrentIndex(idx)
        self._on_motor_chosen()
        dm = d["rocket"].get("dry_mass_kg")
        # component-based airframes carry their mass in the sections / mass items: nothing to edit here
        self.dry_mass.setEnabled(dm is not None)
        self.dry_mass.setToolTip(
            "" if dm is not None else "airframe mass is computed from the component masses in the config"
        )
        if dm is not None:
            self.dry_mass.setValue(dm)
        self.thrust_scale.setValue(d["motor"]["thrust_scale"])
        self.elevation.setValue(d["launch"]["elevation_deg"])
        self.azimuth.setValue(d["launch"]["azimuth_deg"])
        w = d["environment"]["wind"]
        self.wind_speed.setValue(w["speed_ms"] if w["model"] != "none" else 0.0)
        self.wind_dir.setValue(w["direction_from_deg"])
        self.temp_offset.setValue(d["environment"]["atmosphere"]["temperature_offset_k"])
        self.site_elev.setValue(d["environment"]["site_elevation_msl_m"])
        self.fidelity.setCurrentIndex(self.fidelity.findData(d["fidelity"]))
        self.dt.setValue(d["simulation"]["dt_s"])
        self.integrator.setCurrentText(d["simulation"]["integrator"])
        self.seed.setValue(d["simulation"]["seed"] or 0)
        self._populate_vars(d["fidelity"])

    def _populate_vars(self, fidelity: int) -> None:
        keep = {i.data(Qt.ItemDataRole.UserRole) for i in self.var_list.selectedItems()} or set(DEFAULT_PLOTS)
        self.var_list.blockSignals(True)
        self.var_list.clear()
        for c in columns_for(fidelity, estimator=fidelity >= 5):
            if c.name in ("t", "dt", "phase"):
                continue
            it = QtWidgets.QListWidgetItem(f"{c.name}  [{c.unit}]")
            it.setData(Qt.ItemDataRole.UserRole, c.name)
            it.setToolTip(c.description)
            self.var_list.addItem(it)
            if c.name in keep:
                it.setSelected(True)
        self.var_list.blockSignals(False)

    def current_dict(self) -> dict[str, Any]:
        d = copy.deepcopy(self.base_dict)
        ov: dict[str, Any] = {
            "motor.file": self.motor_box.currentData(),
            "motor.thrust_scale": self.thrust_scale.value(),
            "launch.elevation_deg": self.elevation.value(),
            "launch.azimuth_deg": self.azimuth.value(),
            "environment.atmosphere.temperature_offset_k": self.temp_offset.value(),
            "environment.site_elevation_msl_m": self.site_elev.value(),
            "fidelity": int(self.fidelity.currentData()),
            "simulation.dt_s": self.dt.value(),
            "simulation.integrator": self.integrator.currentText(),
        }
        if self.dry_mass.isEnabled():
            ov["rocket.dry_mass_kg"] = self.dry_mass.value()
        if self.wind_speed.value() > 0:
            ov.update(
                {
                    "environment.wind.model": "constant",
                    "environment.wind.speed_ms": self.wind_speed.value(),
                    "environment.wind.direction_from_deg": self.wind_dir.value(),
                }
            )
        else:
            ov["environment.wind.model"] = "none"
        return apply_overrides(d, ov)

    def open_config(self) -> None:
        p, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open configuration or vehicle file",
            str(configs_dir()),
            "Config or vehicle (*.yaml *.yml *.json *.toml)",
        )
        if not p:
            return
        path = Path(p)
        entry = VehicleEntry(f"{path.stem}  [{path.name}]", path, f"Loaded from {path}")
        self.vehicle_box.blockSignals(True)
        self.vehicle_box.addItem(entry.label, entry)
        self.vehicle_box.setCurrentIndex(self.vehicle_box.count() - 1)
        self.vehicle_box.blockSignals(False)
        self._on_vehicle_chosen(self.vehicle_box.currentIndex())

    def run_simulation(self, blocking: bool = False) -> None:
        if self._thread is not None and self._thread.isRunning():
            return
        self.run_button.setEnabled(False)
        self.statusBar().showMessage("Simulating…")
        try:
            cfg_dict = self.current_dict()
        except RocketSimError as exc:
            self._on_failed(str(exc))
            return
        self._worker = SimWorker(cfg_dict, self.base_dir, self.seed.value())
        self._worker.finished.connect(self._on_finished)
        self._worker.failed.connect(self._on_failed)
        if blocking:
            self._worker.run()
            return
        self._thread = QtCore.QThread()
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.finished.connect(self._thread.quit)
        self._worker.failed.connect(self._thread.quit)
        self._thread.start()

    def _on_failed(self, msg: str) -> None:
        self.run_button.setEnabled(True)
        self.statusBar().showMessage("Simulation failed")
        QtWidgets.QMessageBox.critical(self, "Simulation error", msg)

    def _on_finished(self, rec: FlightRecord) -> None:
        self.run_button.setEnabled(True)
        self.set_record(rec)
        saved = ""
        try:
            self.results_dir = save_run_results(rec, str(self.base_dict.get("name", "run")))
            saved = f"Saved to:\n{self.results_dir}"
        except OSError as exc:
            saved = f"Results could not be saved ({exc}).\nUse Export telemetry to choose another folder."
        self.results_label.setPlainText(saved)
        self._refresh_start_page(self.summary_text.toPlainText())
        self.tabs.setCurrentIndex(self.tabs.indexOf(self.summary_text))
        self.statusBar().showMessage(
            f"Done: {rec.meta.n_steps} steps in {rec.meta.wall_time_s:.2f} s, status {rec.meta.status}"
        )

    def set_record(self, rec: FlightRecord) -> None:
        self.record = rec
        self._populate_vars(rec.meta.fidelity)
        self.export_button.setEnabled(True)
        self.summary_text.setPlainText(summary_text(rec))
        self.slider.blockSignals(True)
        self.slider.setRange(0, rec.n_rows - 1)
        self.slider.setValue(0)
        self.slider.blockSignals(False)
        ev = [
            f"{EVENT_STYLE.get(e.name, (e.name,))[0]} {e.t:.2f} s"
            for e in rec.events
            if e.name in EVENT_STYLE
        ]
        self.events_label.setText("Events: " + "   |   ".join(ev))
        self.update_graph()
        self.draw_3d_static()
        self._on_scrub(0)

    def open_results_folder(self) -> None:
        """Open the last run's folder (or the folder where runs will be saved) in the file manager."""
        from PySide6.QtCore import QUrl
        from PySide6.QtGui import QDesktopServices

        target = self.results_dir or (output_dir() / "gui_runs")
        target.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(target)))

    def export_record(self) -> None:
        if self.record is None:
            return
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Export directory", str(output_dir()))
        if d:
            paths = export_record(self.record, d, formats=("csv", "parquet", "json"))
            self.statusBar().showMessage("Wrote " + ", ".join(p.name for p in paths))

    def selected_vars(self) -> list[str]:
        return [i.data(Qt.ItemDataRole.UserRole) for i in self.var_list.selectedItems()]

    def update_graph(self) -> None:
        self.figure.clear()
        self.cursor_lines = []
        rec = self.record
        if rec is None:
            self.canvas.draw_idle()
            return
        names = [n for n in self.selected_vars() if rec.has(n)] or ["altitude"]
        axs = self.figure.subplots(len(names), 1, sharex=True)
        axs = np.atleast_1d(axs)
        t = rec.col("t")
        for i, (ax, n) in enumerate(zip(axs, names)):
            ax.plot(t, rec.col(n), color=COLORS[i % len(COLORS)], lw=1.2)
            ax.set_ylabel(f"{n}\n[{column(n).unit}]", fontsize=8)
            ax.grid(alpha=0.3)
            for e in rec.events:
                if e.name in EVENT_STYLE:
                    ax.axvline(e.t, color=EVENT_STYLE[e.name][1], lw=0.8, ls=":")
            self.cursor_lines.append(ax.axvline(t[0], color="#ffffff", lw=1.0))
        axs[-1].set_xlabel("time [s]")
        self.canvas.draw_idle()

    def _on_scrub(self, i: int) -> None:
        rec = self.record
        if rec is None:
            return
        t = rec.col("t")[i]
        for ln in self.cursor_lines:
            ln.set_xdata([t, t])
        self.canvas.draw_idle()
        ph = FlightPhase(int(rec.col("phase")[i])).name
        self.time_label.setText(f"{t:8.3f} s   {ph}")
        self.readout.setText(
            f"alt {rec.col('altitude')[i]:.1f} m   speed {rec.col('speed')[i]:.1f} m/s   "
            f"Mach {rec.col('mach')[i]:.3f}   thrust {rec.col('thrust')[i]:.1f} N   mass {rec.col('mass')[i]:.3f} kg   "
            f"AoA {np.degrees(rec.col('aoa')[i]):.1f} deg   pitch {np.degrees(rec.col('pitch')[i]):.1f} deg   "
            f"static margin {rec.col('static_margin')[i]:.2f} cal"
        )
        self.update_3d_dynamic(i)

    def draw_3d_static(self) -> None:
        rec = self.record
        ax = self.ax3d
        ax.clear()
        if rec is None:
            return
        x, y, z = rec.col("pos_x"), rec.col("pos_y"), rec.col("pos_z")
        ph = rec.col("phase").astype(int)
        for p in np.unique(ph):
            m = ph == p
            ax.plot(
                np.where(m, x, np.nan),
                np.where(m, y, np.nan),
                np.where(m, z, np.nan),
                color=PHASE_COLORS[int(p)],
                lw=1.6,
                label=FlightPhase(int(p)).name.title(),
            )
        span = max(np.ptp(x), np.ptp(y), 1.0)
        cx, cy = 0.5 * (x.max() + x.min()), 0.5 * (y.max() + y.min())
        r = 0.6 * max(span, 0.3 * z.max())
        gx, gy = np.meshgrid([cx - r, cx + r], [cy - r, cy + r])
        ax.plot_surface(gx, gy, np.zeros_like(gx), alpha=0.15, color="#4a7a4a")
        ax.set_xlim(cx - r, cx + r)
        ax.set_ylim(cy - r, cy + r)
        ax.set_zlim(0, max(z.max(), 1.0) * 1.05)
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
        ax.set_zlabel("Up [m]")
        ax.legend(fontsize=7, loc="upper left")
        (self._rocket_line,) = ax.plot([], [], [], color="#ffffff", lw=3.0)
        self._wind_arrow = None
        self.canvas3d.draw_idle()

    def update_3d_dynamic(self, i: int) -> None:
        rec = self.record
        if rec is None or self._rocket_line is None:
            return
        from ..physics.math3d import quat_rotate

        q = (rec.col("quat_w")[i], rec.col("quat_x")[i], rec.col("quat_y")[i], rec.col("quat_z")[i])
        nose = np.array(quat_rotate(q, (1.0, 0.0, 0.0)))
        p = np.array([rec.col("pos_x")[i], rec.col("pos_y")[i], rec.col("pos_z")[i]])
        scale = 0.06 * max(self.ax3d.get_zlim()[1], 10.0)
        tail, head = p - nose * scale, p + nose * scale
        self._rocket_line.set_data_3d([tail[0], head[0]], [tail[1], head[1]], [tail[2], head[2]])
        if self._wind_arrow is not None:
            self._wind_arrow.remove()
        w = np.array([rec.col("wind_x")[i], rec.col("wind_y")[i], 0.0])
        if np.linalg.norm(w) > 1e-6:
            lim = self.ax3d.get_xlim()
            origin = np.array([lim[0], self.ax3d.get_ylim()[0], 0.0])
            self._wind_arrow = self.ax3d.quiver(
                *origin, *(w / np.linalg.norm(w) * scale * 2), color="#56B4E9"
            )
        self.canvas3d.draw_idle()

    def open_dataset(self, path: str | None = None) -> None:
        d = path or QtWidgets.QFileDialog.getExistingDirectory(self, "Dataset directory", str(output_dir()))
        if not d:
            return
        try:
            from ..data.batch import read_manifest, read_runs

            m = read_manifest(d)
            runs = read_runs(d)
        except (OSError, RocketSimError, KeyError) as exc:
            QtWidgets.QMessageBox.critical(self, "Not a dataset", str(exc))
            return
        self.dataset_dir = Path(d)
        c = m["counts"]
        self.ds_label.setText(
            f"{m['dataset_id']}: {c['simulations_accepted']}/{c['simulations_requested']} accepted, "
            f"{c['samples']} samples (fidelity {m['fidelity_level']}, physics {m['versions']['physics']})"
        )
        cols = ["simulation_id", "split", "accepted", "res.apogee_m", "res.max_velocity_ms", "reject_reason"]
        cols = [c_ for c_ in cols if c_ in runs.column_names]
        rows = runs.select(cols).slice(0, 1000).to_pylist()
        self.runs_table.setColumnCount(len(cols))
        self.runs_table.setHorizontalHeaderLabels(cols)
        self.runs_table.setRowCount(len(rows))
        for r, row in enumerate(rows):
            for c_, name in enumerate(cols):
                v = row[name]
                self.runs_table.setItem(
                    r, c_, QtWidgets.QTableWidgetItem(f"{v:.1f}" if isinstance(v, float) else str(v))
                )
        self.runs_table.resizeColumnsToContents()
        self.run_detail.setPlainText(
            f"{len(rows)} of {runs.num_rows} runs shown (metadata only; no telemetry loaded)."
        )

    def _selected_run_id(self) -> str | None:
        r = self.runs_table.currentRow()
        item = self.runs_table.item(r, 0) if r >= 0 else None
        return None if item is None else item.text()

    def _on_run_selected(self) -> None:
        sid = self._selected_run_id()
        if sid and self.dataset_dir:
            from ..data.browser import describe_run

            self.run_detail.setPlainText(describe_run(self.dataset_dir, sid))
            self.plot_run_button.setEnabled(True)

    def plot_selected_run(self) -> None:
        sid = self._selected_run_id()
        if not sid or not self.dataset_dir:
            return
        from ..data.browser import reproduce_run

        try:
            rec = reproduce_run(self.dataset_dir, sid)
        except RocketSimError as exc:
            QtWidgets.QMessageBox.critical(self, "Cannot reproduce run", str(exc))
            return
        self.set_record(rec)
        self.tabs.setCurrentIndex(0)


def main() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    assert isinstance(app, QtWidgets.QApplication)
    app.setStyleSheet(MAIN_STYLE)
    w = MainWindow()
    w.show()
    app.exec()


if __name__ == "__main__":
    main()
