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
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt

from ..config import apply_overrides, config_from_dict, config_to_dict, load_config
from ..data.export import export_record
from ..errors import RocketSimError
from ..plotting import EVENT_STYLE
from ..reporting import summary_rows, summary_text
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
from .graphs import GraphPanel
from .motor_browser import MotorBrowser
from .styles import apply_theme
from .view3d import Trajectory3DView


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
        root.setContentsMargins(14, 12, 14, 8)
        root.setSpacing(14)

        left = QtWidgets.QWidget()
        left.setObjectName("sidebar")
        left.setFixedWidth(340)
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(10)

        title = QtWidgets.QLabel(
            f"<span style='font-size:15pt;font-weight:600'>Rocket Simulator</span>"
            f"&nbsp;&nbsp;<span style='color:#7d8590'>v{app_version()}</span>"
        )
        title.setObjectName("brand")
        lv.addWidget(title)

        card, cl = self._card("Vehicle")
        self.vehicle_box = QtWidgets.QComboBox()
        for v in list_vehicles():
            self.vehicle_box.addItem(v.label, v)
        self.vehicle_box.currentIndexChanged.connect(self._on_vehicle_chosen)
        self.open_button = QtWidgets.QPushButton("Browse…")
        self.open_button.setObjectName("ghost")
        self.open_button.clicked.connect(self.open_config)
        self.vehicle_info = QtWidgets.QLabel()
        self.vehicle_info.setObjectName("muted")
        self.vehicle_info.setWordWrap(True)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.vehicle_box, 1)
        row.addWidget(self.open_button)
        cl.addLayout(row)
        cl.addWidget(self.vehicle_info)
        lv.addWidget(card)

        card, cl = self._card("Motor")
        self.motor_box = QtWidgets.QComboBox()
        self.motor_box.setEditable(True)
        self.motor_box.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
        completer = self.motor_box.completer()
        if completer is not None:
            completer.setFilterMode(Qt.MatchFlag.MatchContains)
            completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.motor_info: dict[str, str] = {}
        self.motor_label = QtWidgets.QLabel()
        self.motor_label.setObjectName("muted")
        self.motor_label.setWordWrap(True)
        self._fill_motors()
        self.motor_box.currentIndexChanged.connect(self._on_motor_chosen)
        self.find_motors_button = QtWidgets.QPushButton("Find more motors…")
        self.find_motors_button.setObjectName("ghost")
        self.find_motors_button.setToolTip("Search ThrustCurve.org and add motors to the local library")
        self.find_motors_button.clicked.connect(self.open_motor_browser)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.motor_box, 1)
        row.addWidget(self.find_motors_button)
        cl.addLayout(row)
        cl.addWidget(self.motor_label)
        lv.addWidget(card)

        card, cl = self._card("Conditions")
        f = QtWidgets.QFormLayout()
        f.setHorizontalSpacing(12)
        f.setVerticalSpacing(8)
        self.elevation = self._spin(5.0, 90.0, 1, " deg", 1.0)
        self.wind_speed = self._spin(0.0, 40.0, 1, " m/s", 0.5)
        self.wind_dir = self._spin(0.0, 360.0, 0, " deg", 10.0)
        f.addRow("Launch elevation", self.elevation)
        f.addRow("Wind speed", self.wind_speed)
        f.addRow("Wind from", self.wind_dir)
        cl.addLayout(f)
        lv.addWidget(card)

        self.run_button = QtWidgets.QPushButton("Run simulation")
        self.run_button.setObjectName("run_button")
        self.run_button.setMinimumHeight(46)
        self.run_button.clicked.connect(self.run_simulation)
        lv.addWidget(self.run_button)

        card, cl = self._card("Results")
        self.results_label = QtWidgets.QPlainTextEdit()  # selectable, wraps long paths anywhere
        self.results_label.setReadOnly(True)
        self.results_label.setObjectName("path")
        self.results_label.setWordWrapMode(QtGui.QTextOption.WrapMode.WrapAnywhere)
        self.results_label.setMaximumHeight(64)
        self.results_label.setPlainText("Runs are saved under:\n" + str(output_dir() / "gui_runs"))
        self.folder_button = QtWidgets.QPushButton("Open folder")
        self.folder_button.setObjectName("ghost")
        self.folder_button.clicked.connect(self.open_results_folder)
        self.export_button = QtWidgets.QPushButton("Export…")
        self.export_button.setObjectName("ghost")
        self.export_button.clicked.connect(self.export_record)
        self.export_button.setEnabled(False)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.folder_button)
        row.addWidget(self.export_button)
        cl.addWidget(self.results_label)
        cl.addLayout(row)
        lv.addWidget(card)

        self.adv_toggle = QtWidgets.QToolButton()
        self.adv_toggle.setObjectName("disclosure")
        self.adv_toggle.setText("Advanced settings")
        self.adv_toggle.setCheckable(True)
        self.adv_toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.adv_toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.adv_inner, af = self._card("Advanced")
        f = QtWidgets.QFormLayout()
        f.setHorizontalSpacing(12)
        f.setVerticalSpacing(8)
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
        f.addRow("Dry mass", self.dry_mass)
        f.addRow("Thrust scale", self.thrust_scale)
        f.addRow("Azimuth", self.azimuth)
        f.addRow("Temp. offset (ISA)", self.temp_offset)
        f.addRow("Site elevation MSL", self.site_elev)
        f.addRow("Fidelity", self.fidelity)
        f.addRow("Timestep", self.dt)
        f.addRow("Integrator", self.integrator)
        f.addRow("Seed", self.seed)
        af.addLayout(f)
        self.adv_inner.setVisible(False)
        self.adv_toggle.toggled.connect(self._toggle_advanced)
        lv.addWidget(self.adv_toggle)
        lv.addWidget(self.adv_inner)
        lv.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setObjectName("sidebar_scroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(left)
        scroll.setFixedWidth(364)
        root.addWidget(scroll)

        right = QtWidgets.QVBoxLayout()
        self.key_results = QtWidgets.QWidget()
        self.key_row = QtWidgets.QHBoxLayout(self.key_results)
        self.key_row.setContentsMargins(0, 0, 0, 0)
        self.key_row.setSpacing(10)
        self.key_results.setVisible(False)
        right.addWidget(self.key_results)
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
        self.play_button = QtWidgets.QPushButton("Play")
        self.play_button.setFixedWidth(70)
        self.play_button.clicked.connect(self._toggle_play)
        self.speed_box = QtWidgets.QComboBox()
        for label, factor in (("1x", 1.0), ("5x", 5.0), ("20x", 20.0), ("50x", 50.0)):
            self.speed_box.addItem(label, factor)
        self.speed_box.setCurrentIndex(1)
        self.speed_box.setMinimumWidth(70)
        tl.addWidget(self.play_button)
        tl.addWidget(self.speed_box)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self._advance)
        self.slider = QtWidgets.QSlider(Qt.Orientation.Horizontal)
        self.slider.valueChanged.connect(self._on_scrub)
        tl.addWidget(self.slider, 1)
        self.time_label = QtWidgets.QLabel("-")
        self.time_label.setMinimumWidth(190)
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

    @staticmethod
    def _card(title: str) -> tuple[QtWidgets.QFrame, QtWidgets.QVBoxLayout]:
        """A titled panel; returns the frame and the layout to fill."""
        frame = QtWidgets.QFrame()
        frame.setObjectName("card")
        lay = QtWidgets.QVBoxLayout(frame)
        lay.setContentsMargins(14, 12, 14, 14)
        lay.setSpacing(8)
        label = QtWidgets.QLabel(title.upper())
        label.setObjectName("card_title")
        lay.addWidget(label)
        return frame, lay

    def _toggle_advanced(self, on: bool) -> None:
        self.adv_inner.setVisible(on)
        self.adv_toggle.setArrowType(Qt.ArrowType.DownArrow if on else Qt.ArrowType.RightArrow)

    def _fill_motors(self, select: str | None = None) -> None:
        """(Re)build the motor list from the bundled and downloaded libraries, keeping or choosing a selection."""
        current = select or self.motor_box.currentData()
        self.motor_box.blockSignals(True)
        self.motor_box.clear()
        self.motor_info.clear()
        for m in list_motors():
            self.motor_box.addItem(m.label, m.key)
            self.motor_info[m.key] = m.summary
        idx = self.motor_box.findData(current) if current else -1
        if idx < 0 and select:
            idx = self.motor_box.findText(select)
        if idx < 0 and current:  # a motor file named by the vehicle itself, outside both libraries
            self.motor_box.addItem(Path(str(current)).stem + " (from vehicle)", current)
            idx = self.motor_box.count() - 1
        self.motor_box.setCurrentIndex(max(idx, 0))
        self.motor_box.blockSignals(False)
        self._on_motor_chosen()

    def open_motor_browser(self) -> None:
        dlg = MotorBrowser(self)
        dlg.motors_changed.connect(lambda stem: self._fill_motors(select=stem))
        dlg.exec()

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
        self.graphs = GraphPanel()
        return self.graphs

    def _build_3d_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        self.view3d = Trajectory3DView()
        for text, az, el in (("Isometric", None, None), ("Side (from south)", 0.0, 0.0), ("Top", 0.0, 89.0)):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda _=False, az=az, el=el: self.view3d.reset_view(az, el))
            bar.addWidget(b)
        self.follow_box = QtWidgets.QCheckBox("Follow rocket")
        self.follow_box.toggled.connect(self.view3d.set_follow)
        bar.addWidget(self.follow_box)
        bar.addStretch(1)
        v.addLayout(bar)
        v.addWidget(self.view3d, 1)
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
        entry = VehicleEntry(path.stem, path, f"Loaded from {path}")
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

    def _update_key_results(self, rec: FlightRecord) -> None:
        rows = dict(summary_rows(rec))
        items = (
            ("Apogee", "Apogee (AGL)"),
            ("Max speed", "Max velocity"),
            ("Max Mach", "Max Mach"),
            ("Burnout", "Burnout time"),
            ("Flight time", "Total flight time"),
            ("Landing distance", "Landing distance from pad"),
        )
        while self.key_row.count():
            item = self.key_row.takeAt(0)
            w = item.widget() if item is not None else None
            if w is not None:
                w.deleteLater()
        for name, key in items:
            tile = QtWidgets.QFrame()
            tile.setObjectName("stat_tile")
            tl = QtWidgets.QVBoxLayout(tile)
            tl.setContentsMargins(14, 8, 14, 10)
            tl.setSpacing(2)
            label = QtWidgets.QLabel(name)
            label.setObjectName("stat_label")
            value = QtWidgets.QLabel(rows.get(key, "n/a"))
            value.setObjectName("stat_value")
            tl.addWidget(label)
            tl.addWidget(value)
            self.key_row.addWidget(tile)
        self.key_row.addStretch(1)
        self.key_results.setVisible(True)

    def _toggle_play(self) -> None:
        if self.timer.isActive():
            self.timer.stop()
            self.play_button.setText("Play")
            return
        if self.record is None:
            return
        if self.slider.value() >= self.slider.maximum():
            self.slider.setValue(0)
        self.timer.start()
        self.play_button.setText("Pause")

    def _advance(self) -> None:
        rec = self.record
        if rec is None:
            self.timer.stop()
            return
        t = rec.col("t")
        target = t[self.slider.value()] + 0.033 * float(self.speed_box.currentData())
        i = int(np.searchsorted(t, target))
        if i >= self.slider.maximum():
            self.slider.setValue(self.slider.maximum())
            self.timer.stop()
            self.play_button.setText("Play")
        else:
            self.slider.setValue(max(i, self.slider.value() + 1))

    def set_record(self, rec: FlightRecord) -> None:
        self.record = rec
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
        self.graphs.set_record(rec)
        self.view3d.set_record(rec)
        self._update_key_results(rec)
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

    def _on_scrub(self, i: int) -> None:
        rec = self.record
        if rec is None:
            return
        t = rec.col("t")[i]
        self.graphs.set_index(i)
        self.view3d.set_index(i)
        ph = FlightPhase(int(rec.col("phase")[i])).name.replace("_", " ").title()
        self.time_label.setText(f"{t:8.2f} s   {ph}")
        self.readout.setText(
            f"alt {rec.col('altitude')[i]:.1f} m   speed {rec.col('speed')[i]:.1f} m/s   "
            f"Mach {rec.col('mach')[i]:.3f}   thrust {rec.col('thrust')[i]:.1f} N   mass {rec.col('mass')[i]:.3f} kg   "
            f"AoA {np.degrees(rec.col('aoa')[i]):.1f} deg   pitch {np.degrees(rec.col('pitch')[i]):.1f} deg   "
            f"static margin {rec.col('static_margin')[i]:.2f} cal"
        )

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
    apply_theme(app)
    w = MainWindow()
    w.show()
    app.exec()


if __name__ == "__main__":
    main()
