"""Graphical front end (PySide6 + Matplotlib).

The GUI is a thin client of the simulation engine: it builds a config dict, runs the simulation
in a background thread, and visualises the resulting FlightRecord. Large dataset generation is
headless-only (``rocketsim batch``); the dataset browser here reads metadata and reproduces single
runs on demand, so it never loads every timestep.

Wording is written for beginners (a ten-year-old should be able to use it); the Help panel (F1) explains every control.
"""

from __future__ import annotations

import copy
import html
import sys
from pathlib import Path
from typing import Any

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt

from ..config import apply_overrides, config_from_dict, config_to_dict, load_config
from ..data.export import export_record
from ..errors import RocketSimError
from ..motor import Motor, load_motor
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
from .catalog import (
    LEVEL_TITLES,
    VehicleEntry,
    check_fit,
    list_motors,
    list_vehicles,
    motor_path,
    rocket_text,
    save_run_results,
)
from .friendly import compass, flight_story, friendly_error, friendly_warnings, phase_word, steering_story
from .graphs import GraphPanel
from .help_content import ANCHORS, TOPICS, html_all
from .motor_browser import MotorBrowser
from .styles import apply_theme
from .view3d import Trajectory3DView

EVENT_WORDS = {
    "rail_exit": "Left the launch rail",
    "burnout": "Engine burned out",
    "apogee": "Highest point",
    "parachute_deploy": "Parachute opened",
    "landing": "Landed",
}
FIT_COLORS = {"good": "#6adf8a", "warn": "#f2c94c", "bad": "#ff6b6b"}
READY_TEXT = "Pick a rocket and an engine, then press Launch!   (F1 opens the help)"


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
        self.fit_level = "good"
        self._motor_cache: dict[str, Motor | None] = {}
        self._thread: QtCore.QThread | None = None
        self._worker: SimWorker | None = None
        self._build_ui()
        self._build_help_dock()
        QtGui.QShortcut(QtGui.QKeySequence("F1"), self, self.toggle_help)
        QtGui.QShortcut(QtGui.QKeySequence("F5"), self, lambda: self.run_simulation())
        self._select_default_vehicle()

    # ------------------------------------------------------------------------------------------ layout
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(14, 12, 14, 8)
        root.setSpacing(14)

        left = QtWidgets.QWidget()
        left.setObjectName("sidebar")
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 6, 28)  # the bottom margin keeps the last card clear of the window edge
        lv.setSpacing(10)

        title = QtWidgets.QLabel(
            f"<span style='font-size:16pt;font-weight:600'>Rocket Simulator</span>"
            f"&nbsp;&nbsp;<span style='color:#7d8590'>v{app_version()}</span>"
        )
        title.setObjectName("brand")
        lv.addWidget(title)

        # ---- 1  pick a rocket
        card, cl = self._card("1  Pick a rocket", "rocket")
        self.vehicle_box = QtWidgets.QComboBox()
        self.vehicle_box.setToolTip("Choose which rocket to launch")
        self._fill_vehicles()
        self.vehicle_box.currentIndexChanged.connect(self._on_vehicle_chosen)
        self.vehicle_info = self._note()
        self.vehicle_facts = self._note()
        self.open_button = QtWidgets.QPushButton("Open my own rocket file…")
        self.open_button.setObjectName("ghost")
        self.open_button.setToolTip("Load a rocket description you made or were given")
        self.open_button.clicked.connect(self.open_config)
        cl.addWidget(self.vehicle_box)
        cl.addWidget(self.vehicle_info)
        cl.addWidget(self.vehicle_facts)
        cl.addWidget(self.open_button, 0, Qt.AlignmentFlag.AlignLeft)
        lv.addWidget(card)

        # ---- 2  pick an engine
        card, cl = self._card("2  Pick an engine", "engine")
        self.motor_box = QtWidgets.QComboBox()
        self.motor_box.setEditable(True)
        self.motor_box.setInsertPolicy(QtWidgets.QComboBox.InsertPolicy.NoInsert)
        self.motor_box.setToolTip("Choose the engine. You can type to search the list.")
        completer = self.motor_box.completer()
        if completer is not None:
            completer.setFilterMode(Qt.MatchFlag.MatchContains)
            completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self.motor_info: dict[str, str] = {}
        self.motor_label = self._note()
        self.fit_label = self._note()
        self.fit_label.setObjectName("fit")
        self._fill_motors()
        self.motor_box.currentIndexChanged.connect(self._on_motor_chosen)
        self.find_motors_button = QtWidgets.QPushButton("Find more engines…")
        self.find_motors_button.setObjectName("ghost")
        self.find_motors_button.setToolTip("Search a big public list of engines and save them to your computer (needs the internet)")
        self.find_motors_button.clicked.connect(self.open_motor_browser)
        cl.addWidget(self.motor_box)
        cl.addWidget(self.motor_label)
        cl.addWidget(self.fit_label)
        cl.addWidget(self.find_motors_button, 0, Qt.AlignmentFlag.AlignLeft)
        lv.addWidget(card)

        # ---- 3  weather and launch
        card, cl = self._card("3  Launch angle and wind", "weather")
        f = QtWidgets.QFormLayout()
        f.setHorizontalSpacing(12)
        f.setVerticalSpacing(8)
        f.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.elevation = self._spin(5.0, 90.0, 0, " degrees", 5.0, "How steeply the rocket starts. 90 is straight up.")
        self.wind_speed = self._spin(0.0, 40.0, 1, " m/s", 0.5, "How hard the wind blows. 1 is a light breeze, 10 is strong.")
        self.wind_dir = self._spin(0.0, 360.0, 0, " degrees", 15.0, "Where the wind comes from: 0 north, 90 east, 180 south, 270 west.")
        f.addRow("Launch angle", self.elevation)
        f.addRow("Wind speed", self.wind_speed)
        f.addRow("Wind comes from", self.wind_dir)
        cl.addLayout(f)
        self.wind_label = self._note()
        cl.addWidget(self.wind_label)
        self.wind_speed.valueChanged.connect(self._update_wind_label)
        self.wind_dir.valueChanged.connect(self._update_wind_label)
        lv.addWidget(card)

        # ---- 4  steering
        card, cl = self._card("4  Steering with the engine", "steering")
        self.steer_check = QtWidgets.QCheckBox("Steer with the engine nozzle")
        self.steer_check.setToolTip("The engine tilts a little to keep the rocket pointing straight up (TVC)")
        self.steer_note = self._note()
        self.steer_note.setText("Off: the rocket flies on its fins alone.")
        self.steer_inner = QtWidgets.QWidget()
        sf = QtWidgets.QFormLayout(self.steer_inner)
        sf.setContentsMargins(0, 0, 0, 0)
        sf.setHorizontalSpacing(12)
        sf.setVerticalSpacing(8)
        sf.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.steer_angle = self._spin(0.5, 20.0, 1, " degrees", 0.5, "How far the nozzle can swing to each side")
        self.steer_rate = self._spin(10.0, 1000.0, 0, " deg/s", 10.0, "The quickest the nozzle can move")
        self.steer_wn = self._spin(1.0, 40.0, 1, "", 0.5, "How hard the computer pushes the rocket back to where it should point")
        self.steer_zeta = self._spin(0.1, 3.0, 2, "", 0.05, "How quickly it settles down. Around 0.7 to 1 is smooth.")
        self.steer_source = QtWidgets.QComboBox()
        self.steer_source.setToolTip("What the steering computer uses to know where the rocket is")
        self.steer_source.addItem("Its sensors (like a real rocket)", False)
        self.steer_source.addItem("The true position (not possible for real)", True)
        sf.addRow("Most it can tilt", self.steer_angle)
        sf.addRow("How fast it tilts", self.steer_rate)
        sf.addRow("Steering strength", self.steer_wn)
        sf.addRow("Calmness", self.steer_zeta)
        sf.addRow("It knows where it is from", self.steer_source)
        self.steer_inner.setVisible(False)
        self.steer_check.toggled.connect(self._on_steer_toggled)
        cl.addWidget(self.steer_check)
        cl.addWidget(self.steer_note)
        cl.addWidget(self.steer_inner)
        lv.addWidget(card)

        self.run_button = QtWidgets.QPushButton("Launch!")
        self.run_button.setObjectName("run_button")
        self.run_button.setMinimumHeight(52)
        self.run_button.setToolTip("Start the pretend launch  (F5)")
        self.run_button.clicked.connect(self.run_simulation)
        lv.addWidget(self.run_button)

        # ---- your flight
        card, cl = self._card("Your flight is saved here", "results")
        self.results_label = QtWidgets.QPlainTextEdit()  # selectable, wraps long paths anywhere
        self.results_label.setReadOnly(True)
        self.results_label.setObjectName("path")
        self.results_label.setWordWrapMode(QtGui.QTextOption.WrapMode.WrapAnywhere)
        self.results_label.setFixedHeight(76)
        self.results_label.setPlainText("Every launch is saved in its own folder under:\n" + str(output_dir() / "gui_runs"))
        self.folder_button = QtWidgets.QPushButton("Open folder")
        self.folder_button.setObjectName("ghost")
        self.folder_button.setToolTip("Open the folder with the saved flight")
        self.folder_button.clicked.connect(self.open_results_folder)
        self.export_button = QtWidgets.QPushButton("Save a copy…")
        self.export_button.setObjectName("ghost")
        self.export_button.setToolTip("Save the flight data somewhere else, in a few file formats")
        self.export_button.clicked.connect(self.export_record)
        self.export_button.setEnabled(False)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.folder_button)
        row.addWidget(self.export_button)
        cl.addWidget(self.results_label)
        cl.addLayout(row)
        lv.addWidget(card)

        # ---- more settings (collapsed)
        self.adv_toggle = QtWidgets.QToolButton()
        self.adv_toggle.setObjectName("disclosure")
        self.adv_toggle.setText("More settings (for experts)")
        self.adv_toggle.setCheckable(True)
        self.adv_toggle.setArrowType(Qt.ArrowType.RightArrow)
        self.adv_toggle.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.adv_inner, af = self._card("More settings", "advanced")
        f = QtWidgets.QFormLayout()
        f.setHorizontalSpacing(12)
        f.setVerticalSpacing(8)
        f.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self.dry_mass = self._spin(0.001, 500.0, 3, " kg", 0.01, "Weight of the rocket without its engine")
        self.thrust_scale = self._spin(0.5, 1.5, 3, "", 0.01, "1.0 is the real engine; 1.1 pushes 10 % harder")
        self.azimuth = self._spin(0.0, 360.0, 0, " degrees", 5.0, "Compass direction the rocket leans toward (only matters below 90 degrees)")
        self.temp_offset = self._spin(-40.0, 40.0, 1, " °C", 1.0, "Degrees warmer (+) or colder (-) than a standard day")
        self.site_elev = self._spin(-400.0, 5000.0, 0, " m", 50.0, "Height of the launch site above sea level")
        self.fidelity = QtWidgets.QComboBox()
        self.fidelity.setToolTip("How much the computer works out. 3 is a good choice; higher is slower.")
        for i, name in (
            (0, "0  straight up and down, no air"),
            (1, "1  flies in 3-D, no air"),
            (2, "2  flies in 3-D with air"),
            (3, "3  full rocket motion (recommended)"),
            (4, "4  adds sensors"),
            (5, "5  adds a steering computer"),
            (6, "6  most detail"),
        ):
            self.fidelity.addItem(name, i)
        self.dt = self._spin(0.0005, 0.1, 4, " s", 0.001, "Length of each tiny moment. Smaller is more exact but slower.")
        self.integrator = QtWidgets.QComboBox()
        self.integrator.setToolTip("The maths used to step through time. rk4 is the careful one.")
        self.integrator.addItems(["rk4", "midpoint", "euler"])
        self.seed = QtWidgets.QSpinBox()
        self.seed.setToolTip("Fixes the small random wobbles. The same number gives exactly the same flight.")
        self.seed.setRange(0, 2**30)
        f.addRow("Empty weight", self.dry_mass)
        f.addRow("Engine strength", self.thrust_scale)
        f.addRow("Launch direction", self.azimuth)
        f.addRow("Air temperature change", self.temp_offset)
        f.addRow("Launch site height", self.site_elev)
        f.addRow("Detail level", self.fidelity)
        f.addRow("Step size", self.dt)
        f.addRow("Method", self.integrator)
        f.addRow("Random seed", self.seed)
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
        scroll.setFixedWidth(384)
        self.sidebar_scroll = scroll
        self.sidebar = left
        root.addWidget(scroll)

        right = QtWidgets.QVBoxLayout()
        self.key_results = QtWidgets.QWidget()
        self.key_row = QtWidgets.QHBoxLayout(self.key_results)
        self.key_row.setContentsMargins(0, 0, 0, 0)
        self.key_row.setSpacing(10)
        self.key_results.setVisible(False)
        right.addWidget(self.key_results)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_start_tab(), "Start here")
        self.tabs.addTab(self._build_graph_tab(), "Charts")
        self.tabs.addTab(self._build_3d_tab(), "Watch it fly")
        self.summary_text = QtWidgets.QPlainTextEdit()
        self.summary_text.setReadOnly(True)
        self.summary_text.setStyleSheet("font-family: Consolas, monospace;")
        self.tabs.addTab(self.summary_text, "Numbers")
        self.tabs.addTab(self._build_browser_tab(), "Saved runs (experts)")
        self.help_button = QtWidgets.QPushButton("?  Help")
        self.help_button.setObjectName("help_button")
        self.help_button.setToolTip("Explains every part of this window  (F1)")
        self.help_button.clicked.connect(self.toggle_help)
        self.tabs.setCornerWidget(self.help_button, Qt.Corner.TopRightCorner)
        right.addWidget(self.tabs, 1)

        tl = QtWidgets.QHBoxLayout()
        self.play_button = QtWidgets.QPushButton("Play")
        self.play_button.setFixedWidth(70)
        self.play_button.setToolTip("Replay the flight")
        self.play_button.clicked.connect(self._toggle_play)
        self.speed_box = QtWidgets.QComboBox()
        self.speed_box.setToolTip("How fast the replay runs")
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
        self.slider.setToolTip("Drag to any moment of the flight")
        self.slider.valueChanged.connect(self._on_scrub)
        tl.addWidget(self.slider, 1)
        self.time_label = QtWidgets.QLabel("-")
        self.time_label.setMinimumWidth(230)
        tl.addWidget(self.time_label)
        right.addLayout(tl)
        self.readout = QtWidgets.QLabel("Press Launch! to start.")
        self.readout.setWordWrap(True)
        right.addWidget(self.readout)
        self.events_label = QtWidgets.QLabel()
        self.events_label.setWordWrap(True)
        self.events_label.setObjectName("muted")
        right.addWidget(self.events_label)
        root.addLayout(right, 1)
        self.statusBar().showMessage(READY_TEXT)
        self.statusBar().addPermanentWidget(QtWidgets.QLabel(f"{APP_NAME} v{app_version()}"))

    def _card(self, title: str, topic: str | None = None) -> tuple[QtWidgets.QFrame, QtWidgets.QVBoxLayout]:
        """A titled panel with a small ? button that opens the matching help topic; returns the frame and the layout to fill."""
        frame = QtWidgets.QFrame()
        frame.setObjectName("card")
        lay = QtWidgets.QVBoxLayout(frame)
        lay.setContentsMargins(14, 12, 14, 14)
        lay.setSpacing(8)
        head = QtWidgets.QHBoxLayout()
        label = QtWidgets.QLabel(title)
        label.setObjectName("card_title")
        head.addWidget(label, 1)
        if topic is not None:
            b = QtWidgets.QToolButton()
            b.setObjectName("help_chip")
            b.setText("?")
            b.setToolTip("What is this? (opens the help)")
            b.clicked.connect(lambda _=False, t=topic: self.show_help(t))
            head.addWidget(b)
        lay.addLayout(head)
        return frame, lay

    @staticmethod
    def _note() -> QtWidgets.QLabel:
        label = QtWidgets.QLabel()
        label.setObjectName("muted")
        label.setWordWrap(True)
        label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Preferred, QtWidgets.QSizePolicy.Policy.MinimumExpanding)
        return label

    def _spin(self, lo: float, hi: float, dec: int, suffix: str, step: float, tip: str = "") -> QtWidgets.QDoubleSpinBox:
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(dec)
        s.setSuffix(suffix)
        s.setSingleStep(step)
        s.setToolTip(tip)
        return s

    def _on_steer_toggled(self, on: bool) -> None:
        self.steer_inner.setVisible(on)
        self.steer_note.setText(
            "On: the nozzle tilts to keep the rocket straight. Only works while the engine is pushing."
            if on
            else "Off: the rocket flies on its fins alone."
        )

    def _toggle_advanced(self, on: bool) -> None:
        self.adv_inner.setVisible(on)
        self.adv_toggle.setArrowType(Qt.ArrowType.DownArrow if on else Qt.ArrowType.RightArrow)
        if on:  # bring the opened card into view
            QtCore.QTimer.singleShot(0, lambda: self.sidebar_scroll.ensureWidgetVisible(self.adv_inner))

    # ------------------------------------------------------------------------------------------ help
    def _build_help_dock(self) -> None:
        dock = QtWidgets.QDockWidget("Help", self)
        dock.setObjectName("help_dock")
        dock.setAllowedAreas(Qt.DockWidgetArea.RightDockWidgetArea | Qt.DockWidgetArea.LeftDockWidgetArea)
        body = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(body)
        v.setContentsMargins(8, 8, 8, 8)
        self.help_topics = QtWidgets.QComboBox()
        self.help_topics.setToolTip("Jump to a topic")
        for anchor, title, _ in TOPICS:
            self.help_topics.addItem(title, anchor)
        self.help_view = QtWidgets.QTextBrowser()
        self.help_view.setHtml(html_all())
        self.help_topics.activated.connect(lambda i: self.help_view.scrollToAnchor(self.help_topics.itemData(i)))
        v.addWidget(QtWidgets.QLabel("Jump to:"))
        v.addWidget(self.help_topics)
        v.addWidget(self.help_view, 1)
        dock.setWidget(body)
        dock.setMinimumWidth(380)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)
        dock.hide()
        self.help_dock = dock

    def toggle_help(self) -> None:
        if self.help_dock.isVisible():
            self.help_dock.hide()
        else:
            self.show_help()

    def show_help(self, topic: str | None = None) -> None:
        """Open the help panel, at ``topic`` (one of help_content.ANCHORS) if given."""
        self.help_dock.show()
        self.help_dock.raise_()
        if topic is not None and topic in ANCHORS:
            self.help_topics.setCurrentIndex(ANCHORS.index(topic))
            self.help_view.scrollToAnchor(topic)

    # ------------------------------------------------------------------------------------ rockets, engines
    def _fill_vehicles(self) -> None:
        self.vehicle_box.blockSignals(True)
        self.vehicle_box.clear()
        shown: str | None = None
        for v in list_vehicles():
            if v.level != shown:  # a heading that cannot be chosen
                shown = v.level
                self.vehicle_box.addItem(f"— {LEVEL_TITLES[v.level]} —")
                item = self.vehicle_box.model().item(self.vehicle_box.count() - 1)  # type: ignore[attr-defined]
                item.setEnabled(False)
            self.vehicle_box.addItem(v.label, v)
        self.vehicle_box.blockSignals(False)

    def _select_default_vehicle(self) -> None:
        """Start on the default example rocket (so a first launch needs no choices), else on the first rocket."""
        want = default_config().name
        first = None
        for i in range(self.vehicle_box.count()):
            entry = self.vehicle_box.itemData(i)
            if not isinstance(entry, VehicleEntry):
                continue
            first = i if first is None else first
            if entry.path.name == want:
                first = i
                break
        self.vehicle_box.setCurrentIndex(first if first is not None else 0)
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
                "Could not load the rocket",
                f"Could not load:\n{path}\n\n{exc}\n\nThe previous rocket is still selected.",
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
            self.vehicle_facts.setText(rocket_text(self.base_dict))

    def _fill_motors(self, select: str | None = None) -> None:
        """(Re)build the engine list from the bundled and downloaded libraries, keeping or choosing a selection."""
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
            if idx < 0:
                idx = self.motor_box.findText(select.replace("_", " "))
        if idx < 0 and current:  # an engine file named by the rocket itself, outside both libraries
            self.motor_box.addItem(Path(str(current)).stem.replace("_", " ") + " (from the rocket file)", current)
            idx = self.motor_box.count() - 1
        self.motor_box.setCurrentIndex(max(idx, 0))
        self.motor_box.blockSignals(False)
        self._on_motor_chosen()

    def _motor(self, key: str | None) -> Motor | None:
        if not key:
            return None
        if key not in self._motor_cache:
            try:
                self._motor_cache[key] = load_motor(motor_path(key))
            except (RocketSimError, OSError):
                self._motor_cache[key] = None
        return self._motor_cache[key]

    def _on_motor_chosen(self, _index: int = 0) -> None:
        key = self.motor_box.currentData()
        self.motor_label.setText(self.motor_info.get(key, "An engine from a file."))
        self._update_fit()

    def _update_fit(self) -> None:
        motor = self._motor(self.motor_box.currentData())
        if motor is None or not self.base_dict:
            self.fit_level = "good"
            self.fit_label.setText("")
            return
        self.fit_level, text = check_fit(self.base_dict, motor)
        mark = {"good": "OK:", "warn": "Careful:", "bad": "Will not work:"}[self.fit_level]
        self.fit_label.setText(f"<span style='color:{FIT_COLORS[self.fit_level]}'><b>{mark}</b> {html.escape(text)}</span>")

    def open_motor_browser(self) -> None:
        dlg = MotorBrowser(self)
        dlg.motors_changed.connect(lambda stem: self._fill_motors(select=stem))
        dlg.exec()

    def _update_wind_label(self, _v: float = 0.0) -> None:
        if self.wind_speed.value() <= 0:
            self.wind_label.setText("No wind: the air is still.")
        else:
            d = self.wind_dir.value()
            self.wind_label.setText(f"The wind blows from the {compass(d)}, toward the {compass(d + 180)}.")

    # ------------------------------------------------------------------------------------------ tabs
    def _build_start_tab(self) -> QtWidgets.QWidget:
        self.start_page = QtWidgets.QTextBrowser()
        self.start_page.setOpenExternalLinks(False)
        self.start_page.setOpenLinks(False)
        self.start_page.anchorClicked.connect(lambda url: self.show_help(url.toString().lstrip("#")))
        self._refresh_start_page()
        return self.start_page

    def _refresh_start_page(self, story: list[str] | None = None, notes: list[str] | None = None) -> None:
        paths = (
            f"<p style='color:#8b93a1'>{APP_NAME} v{app_version()} ({launch_description()}). The rockets are made-up examples, not "
            f"real kits. Data: {html.escape(str(resource_root()))}. Flights are saved in "
            f"{html.escape(str(output_dir() / 'gui_runs'))}.</p>"
        )
        how = """
            <h2>How to play</h2>
            <ol style="font-size:12pt">
              <li><b>Pick a rocket</b> in box 1 on the left.</li>
              <li><b>Pick an engine</b> in box 2 (or keep the one it comes with).</li>
              <li>Press the big blue <b>Launch!</b> button.</li>
            </ol>
            <p>Not sure what something is? Press the little <b>?</b> buttons, or <a href="#start">open the help</a> (the
            <b>Help</b> button at the top right, or F1).</p>
        """
        if story is None:
            body = (
                "<h1>Rocket Simulator</h1>"
                "<p style='font-size:13pt'>Pretend-launch rockets on your computer, then watch what happens.</p>" + how
            )
        else:
            items = "".join(f"<li>{html.escape(s)}</li>" for s in story)
            warn = ""
            if notes:
                warn = "<h2>Things to know</h2><ul>" + "".join(f"<li style='color:#f2c94c'>{html.escape(n)}</li>" for n in notes) + "</ul>"
            body = f"<h1>Your flight</h1><ul style='font-size:12pt'>{items}</ul>{warn}"
        self.start_page.setHtml(body + paths)

    def _build_graph_tab(self) -> QtWidgets.QWidget:
        self.graphs = GraphPanel()
        return self.graphs

    def _build_3d_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        self.view3d = Trajectory3DView()
        for text, az, el, tip in (
            ("Angled view", None, None, "A view from the side and a little above"),
            ("Side view", 0.0, 0.0, "Look at the flight from the side"),
            ("From above", 0.0, 89.0, "Look straight down on the flight"),
        ):
            b = QtWidgets.QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(lambda _=False, az=az, el=el: self.view3d.reset_view(az, el))
            bar.addWidget(b)
        self.follow_box = QtWidgets.QCheckBox("Follow the rocket")
        self.follow_box.setToolTip("Keep the rocket in the middle of the picture")
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
        self.ds_button = QtWidgets.QPushButton("Open a folder of many flights…")
        self.ds_button.clicked.connect(self.open_dataset)
        self.ds_label = QtWidgets.QLabel("Nothing open. This tab is for big collections made by the command-line tools.")
        self.plot_run_button = QtWidgets.QPushButton("Show the selected flight")
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

    # ------------------------------------------------------------------------------------ config <-> UI
    def _load_controls_from_dict(self) -> None:
        d = self.base_dict
        self.statusBar().showMessage(f"Rocket loaded: {self.vehicle_box.currentText() or d['name']}.  {READY_TEXT}")
        file = d["motor"]["file"].replace("\\", "/")
        idx = self.motor_box.findData(file)
        if idx < 0:  # the rocket's own engine file lies outside the bundled list: keep it selectable
            self.motor_box.addItem(Path(file).stem.replace("_", " ") + " (from the rocket file)", file)
            idx = self.motor_box.count() - 1
            self.motor_info[file] = "An engine named by the rocket file."
        self.motor_box.setCurrentIndex(idx)
        self._on_motor_chosen()
        dm = d["rocket"].get("dry_mass_kg")
        # component-based airframes carry their mass in the sections / mass items: nothing to edit here
        self.dry_mass.setEnabled(dm is not None)
        self.dry_mass.setToolTip(
            "Weight of the rocket without its engine"
            if dm is not None
            else "This rocket is built from parts, so its weight is added up from the parts."
        )
        if dm is not None:
            self.dry_mass.setValue(dm)
        ctl = d["controller"]
        steerable = ctl["type"] in ("none", "tvc_attitude")  # another controller type is left as the file has it
        self.steer_check.setEnabled(steerable)
        self.steer_check.setToolTip(
            "The engine tilts a little to keep the rocket pointing straight up (TVC)"
            if steerable
            else "This rocket already has its own steering computer."
        )
        self.steer_check.setChecked(ctl["type"] == "tvc_attitude")
        self.steer_angle.setValue(d["tvc"]["max_angle_deg"])
        self.steer_rate.setValue(d["tvc"]["max_rate_deg_s"])
        params = ctl.get("params", {})
        self.steer_wn.setValue(float(params.get("wn", 8.0)))
        self.steer_zeta.setValue(float(params.get("zeta", 0.8)))
        self.steer_source.setCurrentIndex(1 if ctl.get("use_truth") else 0)
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
        self._update_wind_label()

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
        if self.steer_check.isEnabled():
            ov.update(self._steering_overrides(d, ov))
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

    def _steering_overrides(self, d: dict[str, Any], ov: dict[str, Any]) -> dict[str, Any]:
        """Config changes for the steering box. Switching steering on also needs sensors, an estimate and pad time."""
        if not self.steer_check.isChecked():
            return {"controller.type": "none", "estimator.type": "none"} if d["controller"]["type"] == "tvc_attitude" else {}
        est = d["estimator"]["type"]
        return {
            "tvc.max_angle_deg": self.steer_angle.value(),
            "tvc.max_rate_deg_s": self.steer_rate.value(),
            "controller.type": "tvc_attitude",
            "controller.rate_hz": max(float(d["controller"]["rate_hz"]), 100.0),
            "controller.use_truth": bool(self.steer_source.currentData()),
            "controller.params.wn": self.steer_wn.value(),
            "controller.params.zeta": self.steer_zeta.value(),
            "controller.params.target": "vertical",
            "estimator.type": est if est != "none" else "nav_kf",
            "estimator.alignment_time_s": 1.5,
            "fidelity": max(5, int(ov["fidelity"])),
            "motor.ignition_delay_s": max(2.0, float(d["motor"]["ignition_delay_s"])),
            "simulation.dt_s": min(0.005, float(ov["simulation.dt_s"])),
            "sensors.accelerometer.saturation": max(400.0, float(d["sensors"]["accelerometer"]["saturation"])),
        }

    # ------------------------------------------------------------------------------------------ actions
    def open_config(self) -> None:
        p, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open a rocket file",
            str(configs_dir()),
            "Rocket files (*.yaml *.yml *.json *.toml)",
        )
        if not p:
            return
        path = Path(p)
        entry = VehicleEntry(path.stem, path, f"Loaded from {path}", "other")
        self.vehicle_box.blockSignals(True)
        self.vehicle_box.addItem(entry.label, entry)
        self.vehicle_box.setCurrentIndex(self.vehicle_box.count() - 1)
        self.vehicle_box.blockSignals(False)
        self._on_vehicle_chosen(self.vehicle_box.currentIndex())

    def run_simulation(self, blocking: bool = False) -> None:
        if self._thread is not None and self._thread.isRunning():
            return
        if self.fit_level == "bad":
            QtWidgets.QMessageBox.information(
                self,
                "This engine will not work",
                self.fit_label.text().split("</b>", 1)[-1].replace("</span>", "").strip()
                + "\n\nPick a different engine or rocket and press Launch! again.",
            )
            return
        self.run_button.setEnabled(False)
        self.run_button.setText("Launching…")
        self.statusBar().showMessage("The computer is working out the flight…")
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
        self.run_button.setText("Launch!")
        self.statusBar().showMessage("That launch did not work. " + READY_TEXT)
        QtWidgets.QMessageBox.critical(self, "Hmm, that did not work", friendly_error(msg))

    def _on_finished(self, rec: FlightRecord) -> None:
        self.run_button.setEnabled(True)
        self.run_button.setText("Launch!")
        self.set_record(rec)
        saved = ""
        try:
            self.results_dir = save_run_results(rec, str(self.base_dict.get("name", "run")))
            saved = f"Saved to:\n{self.results_dir}"
        except OSError as exc:
            saved = f"The flight could not be saved ({exc}).\nUse Save a copy to choose another folder."
        self.results_label.setPlainText(saved)
        self._refresh_start_page(flight_story(rec) + steering_story(rec), friendly_warnings(list(rec.meta.warnings)))
        self.tabs.setCurrentIndex(0)
        self.statusBar().showMessage(f"Launched! The computer took {rec.meta.wall_time_s:.1f} s to work it out.")

    def _update_key_results(self, rec: FlightRecord) -> None:
        rows = dict(summary_rows(rec))
        items = (
            ("Highest point", "Apogee (AGL)"),
            ("Top speed", "Max velocity"),
            ("Engine pushed for", "Burnout time"),
            ("Flight time", "Total flight time"),
            ("Landed this far away", "Landing distance from pad"),
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
            main, sub = friendly_value(rows.get(key, "n/a"))
            value = QtWidgets.QLabel(main)
            value.setObjectName("stat_value")
            tl.addWidget(label)
            tl.addWidget(value)
            if sub:
                s = QtWidgets.QLabel(sub)
                s.setObjectName("stat_label")
                tl.addWidget(s)
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
        ev = [f"{EVENT_WORDS[e.name]} at {e.t:.1f} s" for e in rec.events if e.name in EVENT_WORDS]
        self.events_label.setText("What happens when:   " + "   ·   ".join(ev))
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
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Where should the copy go?", str(output_dir()))
        if d:
            paths = export_record(self.record, d, formats=("csv", "parquet", "json"))
            self.statusBar().showMessage("Saved " + ", ".join(p.name for p in paths))

    def _on_scrub(self, i: int) -> None:
        rec = self.record
        if rec is None:
            return
        t = rec.col("t")[i]
        self.graphs.set_index(i)
        self.view3d.set_index(i)
        phase = phase_word(FlightPhase(int(rec.col("phase")[i])).name)
        self.time_label.setText(f"{t:7.2f} s   {phase}")
        speed = float(rec.col("speed")[i])
        lean = 90.0 - float(np.degrees(rec.col("pitch")[i]))
        self.readout.setText(
            f"Height {rec.col('altitude')[i]:,.0f} m    ·    Speed {speed:.0f} m/s ({speed * 3.6:.0f} km/h)    ·    "
            f"Engine push {rec.col('thrust')[i]:.0f} N    ·    Weight {rec.col('mass')[i]:.2f} kg    ·    "
            f"Leaning {abs(lean):.0f}° from straight up"
        )

    # ---------------------------------------------------------------------------------------- datasets
    def open_dataset(self, path: str | None = None) -> None:
        d = path or QtWidgets.QFileDialog.getExistingDirectory(self, "Folder of many flights", str(output_dir()))
        if not d:
            return
        try:
            from ..data.batch import read_manifest, read_runs

            m = read_manifest(d)
            runs = read_runs(d)
        except (OSError, RocketSimError, KeyError) as exc:
            QtWidgets.QMessageBox.critical(self, "That folder is not a collection of flights", str(exc))
            return
        self.dataset_dir = Path(d)
        c = m["counts"]
        self.ds_label.setText(
            f"{m['dataset_id']}: {c['simulations_accepted']}/{c['simulations_requested']} accepted, "
            f"{c['samples']} samples (detail level {m['fidelity_level']}, physics {m['versions']['physics']})"
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
            f"{len(rows)} of {runs.num_rows} flights shown (only the summary of each; the full data is loaded when you pick one)."
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
            QtWidgets.QMessageBox.critical(self, "Could not rebuild that flight", str(exc))
            return
        self.set_record(rec)
        self.tabs.setCurrentIndex(1)


def friendly_value(text: str) -> tuple[str, str]:
    """Split ``918 +/- 70* m`` into the plain value ``918 m`` and the note ``give or take 70 m``."""
    import re

    m = re.match(r"^(?P<v>[-\d.,]+) \+/- (?P<e>[\d.,]+)(?P<star>\*?) ?(?P<u>.*)$", text.strip())
    if not m:
        return text, ""
    unit = f" {m['u']}" if m["u"] else ""
    sub = f"give or take {m['e']}{unit}" + (" (a rough guess)" if m["star"] else "")
    return f"{m['v']}{unit}", sub


def main() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    assert isinstance(app, QtWidgets.QApplication)
    apply_theme(app)
    w = MainWindow()
    w.show()
    app.exec()


if __name__ == "__main__":
    main()
