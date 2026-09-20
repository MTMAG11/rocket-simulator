from pathlib import Path

from PySide6 import QtWidgets
from PySide6.QtCore import Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from ..config import SimulationConfig
from ..motor import get_available_motors
from ..simulation import run_simulation as run_simulation_backend


MAIN_STYLE = """
QMainWindow {
    background-color: #1e1e1e;
}

QWidget {
    color: #e0e0e0;
    font-size: 13px;
}

QGroupBox {
    border: 1px solid #444444;
    border-radius: 6px;
    margin-top: 10px;
    padding-top: 10px;
    font-weight: bold;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
}

QPushButton {
    background-color: #333333;
    border: 1px solid #555555;
    border-radius: 5px;
    padding: 7px 14px;
}

QPushButton:hover {
    background-color: #444444;
}

QPushButton:pressed {
    background-color: #222222;
}

QComboBox,
QDoubleSpinBox,
QSpinBox {
    background-color: #2b2b2b;
    border: 1px solid #555555;
    border-radius: 4px;
    padding: 5px;
}

QSlider::groove:horizontal {
    height: 5px;
    background: #444444;
    border-radius: 2px;
}

QSlider::handle:horizontal {
    width: 12px;
    margin: -4px 0;
    border-radius: 6px;
    background: #aaaaaa;
}

QTabWidget::pane {
    border: 1px solid #444444;
}

QTabBar::tab {
    background: #2b2b2b;
    padding: 8px 14px;
    border: 1px solid #444444;
}

QTabBar::tab:selected {
    background: #3a3a3a;
}

QLabel {
    color: #dddddd;
}
"""


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Rocket Simulator")
        self.resize(1400, 900)

        self.results = None
        self.current_graph = "Altitude"

        self.setup_ui()
        self.connect_signals()
        self.reset_simulation()

    def setup_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # ============================================================
        # ACTION BAR
        # ============================================================

        action_layout = QtWidgets.QHBoxLayout()

        self.run_button = QtWidgets.QPushButton("Run Simulation")
        self.reset_button = QtWidgets.QPushButton("Reset")

        action_layout.addWidget(self.run_button)
        action_layout.addWidget(self.reset_button)
        action_layout.addStretch()

        main_layout.addLayout(action_layout)

        # ============================================================
        # MAIN CONTENT
        # ============================================================

        content_layout = QtWidgets.QHBoxLayout()

        # ------------------------------------------------------------
        # LEFT SIDE: CONTROLS
        # ------------------------------------------------------------

        controls_widget = QtWidgets.QWidget()
        controls_layout = QtWidgets.QVBoxLayout(controls_widget)

        controls_widget.setMaximumWidth(330)

        # Rocket
        rocket_group = QtWidgets.QGroupBox("Rocket")
        rocket_layout = QtWidgets.QFormLayout(rocket_group)

        self.motor_dropdown = QtWidgets.QComboBox()

        motors = get_available_motors()

        for motor in motors:
            self.motor_dropdown.addItem(
                motor.stem,
                motor,
            )

        self.dry_mass_spinbox = QtWidgets.QDoubleSpinBox()
        self.dry_mass_spinbox.setRange(1.0, 100000.0)
        self.dry_mass_spinbox.setDecimals(2)
        self.dry_mass_spinbox.setSuffix(" g")
        self.dry_mass_spinbox.setValue(150.0)

        rocket_layout.addRow(
            "Motor:",
            self.motor_dropdown,
        )

        rocket_layout.addRow(
            "Dry mass:",
            self.dry_mass_spinbox,
        )

        controls_layout.addWidget(rocket_group)

        # Environment
        environment_group = QtWidgets.QGroupBox("Environment")
        environment_layout = QtWidgets.QFormLayout(
            environment_group
        )

        self.gravity_spinbox = QtWidgets.QDoubleSpinBox()
        self.gravity_spinbox.setRange(0.0, 30.0)
        self.gravity_spinbox.setDecimals(3)
        self.gravity_spinbox.setSuffix(" m/s²")
        self.gravity_spinbox.setValue(9.81)

        environment_layout.addRow(
            "Gravity:",
            self.gravity_spinbox,
        )

        controls_layout.addWidget(environment_group)

        # Simulation
        simulation_group = QtWidgets.QGroupBox("Simulation")
        simulation_layout = QtWidgets.QFormLayout(
            simulation_group
        )

        self.timestep_spinbox = QtWidgets.QDoubleSpinBox()
        self.timestep_spinbox.setRange(0.0001, 1.0)
        self.timestep_spinbox.setDecimals(4)
        self.timestep_spinbox.setSingleStep(0.001)
        self.timestep_spinbox.setSuffix(" s")
        self.timestep_spinbox.setValue(0.005)

        simulation_layout.addRow(
            "Timestep:",
            self.timestep_spinbox,
        )

        controls_layout.addWidget(simulation_group)

        controls_layout.addStretch()

        content_layout.addWidget(controls_widget)

        # ------------------------------------------------------------
        # RIGHT SIDE: FLIGHT DATA
        # ------------------------------------------------------------

        flight_widget = QtWidgets.QWidget()
        flight_layout = QtWidgets.QVBoxLayout(flight_widget)

        self.flight_tabs = QtWidgets.QTabWidget()

        # ============================================================
        # GRAPHS TAB
        # ============================================================

        graphs_tab = QtWidgets.QWidget()
        graphs_layout = QtWidgets.QVBoxLayout(graphs_tab)

        graph_selector_layout = QtWidgets.QHBoxLayout()

        graph_selector_layout.addWidget(
            QtWidgets.QLabel("Graph:")
        )

        self.graph_dropdown = QtWidgets.QComboBox()

        self.graph_dropdown.addItems(
            [
                "Altitude",
                "Velocity",
                "Acceleration",
                "G-Force",
                "Thrust",
                "TWR",
                "All",
            ]
        )

        graph_selector_layout.addWidget(
            self.graph_dropdown
        )

        graph_selector_layout.addStretch()

        graphs_layout.addLayout(graph_selector_layout)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)

        graphs_layout.addWidget(self.canvas)

        self.flight_tabs.addTab(
            graphs_tab,
            "Graphs",
        )

        # ============================================================
        # 3D TAB
        # ============================================================

        view_3d_tab = QtWidgets.QWidget()
        view_3d_layout = QtWidgets.QVBoxLayout(view_3d_tab)

        view_3d_label = QtWidgets.QLabel(
            "3D View\n\nComing later"
        )

        view_3d_label.setAlignment(
            Qt.AlignCenter
        )

        view_3d_label.setStyleSheet(
            "font-size: 20px; color: #888888;"
        )

        view_3d_layout.addWidget(
            view_3d_label
        )

        self.flight_tabs.addTab(
            view_3d_tab,
            "3D View",
        )

        flight_layout.addWidget(
            self.flight_tabs
        )

        # ============================================================
        # SUMMARY
        # ============================================================

        summary_group = QtWidgets.QGroupBox("Flight Summary")
        summary_layout = QtWidgets.QGridLayout(
            summary_group
        )

        self.max_altitude_label = QtWidgets.QLabel("-")
        self.max_velocity_label = QtWidgets.QLabel("-")
        self.max_acceleration_label = QtWidgets.QLabel("-")
        self.max_g_label = QtWidgets.QLabel("-")
        self.burnout_label = QtWidgets.QLabel("-")
        self.apogee_label = QtWidgets.QLabel("-")

        summary_layout.addWidget(
            QtWidgets.QLabel("Max altitude:"),
            0,
            0,
        )
        summary_layout.addWidget(
            self.max_altitude_label,
            0,
            1,
        )

        summary_layout.addWidget(
            QtWidgets.QLabel("Max velocity:"),
            0,
            2,
        )
        summary_layout.addWidget(
            self.max_velocity_label,
            0,
            3,
        )

        summary_layout.addWidget(
            QtWidgets.QLabel("Max acceleration:"),
            1,
            0,
        )
        summary_layout.addWidget(
            self.max_acceleration_label,
            1,
            1,
        )

        summary_layout.addWidget(
            QtWidgets.QLabel("Max G:"),
            1,
            2,
        )
        summary_layout.addWidget(
            self.max_g_label,
            1,
            3,
        )

        summary_layout.addWidget(
            QtWidgets.QLabel("Burnout:"),
            2,
            0,
        )
        summary_layout.addWidget(
            self.burnout_label,
            2,
            1,
        )

        summary_layout.addWidget(
            QtWidgets.QLabel("Apogee:"),
            2,
            2,
        )
        summary_layout.addWidget(
            self.apogee_label,
            2,
            3,
        )

        flight_layout.addWidget(
            summary_group
        )

        # ============================================================
        # TIME CONTROLS
        # ============================================================

        time_layout = QtWidgets.QHBoxLayout()

        time_layout.addWidget(
            QtWidgets.QLabel("Time:")
        )

        self.time_slider = QtWidgets.QSlider(
            Qt.Horizontal
        )

        self.time_slider.setMinimum(0)
        self.time_slider.setMaximum(0)

        time_layout.addWidget(
            self.time_slider
        )

        self.time_label = QtWidgets.QLabel(
            "0.000 s"
        )

        time_layout.addWidget(
            self.time_label
        )

        flight_layout.addLayout(
            time_layout
        )

        content_layout.addWidget(
            flight_widget,
            stretch=1,
        )

        main_layout.addLayout(
            content_layout,
            stretch=1,
        )

    # ================================================================
    # SIGNALS
    # ================================================================

    def connect_signals(self):
        self.run_button.clicked.connect(
            self.run_simulation
        )

        self.reset_button.clicked.connect(
            self.reset_simulation
        )

        self.graph_dropdown.currentTextChanged.connect(
            self.select_graph
        )

        self.time_slider.valueChanged.connect(
            self.update_time_cursor
        )

    # ================================================================
    # CONFIG
    # ================================================================

    def get_simulation_config(self):
        motor_path = self.motor_dropdown.currentData()

        return SimulationConfig(
            motor=Path(motor_path),
            rocket_dry_mass=self.dry_mass_spinbox.value(),
            gravity=self.gravity_spinbox.value(),
            dt=self.timestep_spinbox.value(),
        )

    # ================================================================
    # GRAPH DATA
    # ================================================================

    def get_graph_data(self):
        times = self.results["times"]

        altitudes = self.results["ys"]

        velocities = [
            (vx**2 + vy**2) ** 0.5
            for vx, vy in zip(
                self.results["vxs"],
                self.results["vys"],
            )
        ]

        accelerations = [
            (ax**2 + ay**2) ** 0.5
            for ax, ay in zip(
                self.results["axs"],
                self.results["ays"],
            )
        ]

        thrusts = self.results["thrusts"]
        twrs = self.results["twrs"]

        return (
            times,
            altitudes,
            velocities,
            accelerations,
            thrusts,
            twrs,
        )

    # ================================================================
    # RUN SIMULATION
    # ================================================================

    def run_simulation(self):
        config = self.get_simulation_config()

        self.results = run_simulation_backend(
            config
        )

        self.update_graph()
        self.update_summary()

        times = self.results["times"]

        if times:
            self.time_slider.setRange(
                0,
                len(times) - 1,
            )

            self.time_slider.setValue(0)

        self.update_time_cursor(0)

    # ================================================================
    # GRAPHING
    # ================================================================

    def update_graph(self):
        if self.results is None:
            return

        (
            times,
            altitudes,
            velocities,
            accelerations,
            thrusts,
            twrs,
        ) = self.get_graph_data()

        if not times:
            return

        g_forces = [
            acceleration / 9.80665
            for acceleration in accelerations
        ]

        self.figure.clear()

        if self.current_graph == "All":
            axes = self.figure.subplots(
                3,
                2,
            )

            graphs = [
                (
                    axes[0, 0],
                    times,
                    altitudes,
                    "Altitude",
                    "Altitude (m)",
                ),
                (
                    axes[0, 1],
                    times,
                    velocities,
                    "Velocity",
                    "Velocity (m/s)",
                ),
                (
                    axes[1, 0],
                    times,
                    accelerations,
                    "Acceleration",
                    "Acceleration (m/s²)",
                ),
                (
                    axes[1, 1],
                    times,
                    g_forces,
                    "G-Force",
                    "G",
                ),
                (
                    axes[2, 0],
                    times,
                    thrusts,
                    "Thrust",
                    "Thrust (N)",
                ),
                (
                    axes[2, 1],
                    times,
                    twrs,
                    "TWR",
                    "TWR",
                ),
            ]

            for (
                axis,
                x,
                y,
                title,
                ylabel,
            ) in graphs:
                axis.plot(
                    x,
                    y,
                )

                axis.set_title(title)
                axis.set_xlabel("Time (s)")
                axis.set_ylabel(ylabel)
                axis.grid(True)

        else:
            axis = self.figure.add_subplot(111)

            if self.current_graph == "Altitude":
                values = altitudes
                ylabel = "Altitude (m)"

            elif self.current_graph == "Velocity":
                values = velocities
                ylabel = "Velocity (m/s)"

            elif self.current_graph == "Acceleration":
                values = accelerations
                ylabel = "Acceleration (m/s²)"

            elif self.current_graph == "G-Force":
                values = g_forces
                ylabel = "G"

            elif self.current_graph == "Thrust":
                values = thrusts
                ylabel = "Thrust (N)"

            elif self.current_graph == "TWR":
                values = twrs
                ylabel = "TWR"

            else:
                values = []
                ylabel = ""

            axis.plot(
                times,
                values,
            )

            axis.set_title(
                self.current_graph
            )

            axis.set_xlabel(
                "Time (s)"
            )

            axis.set_ylabel(
                ylabel
            )

            axis.grid(True)

        self.figure.tight_layout()
        self.canvas.draw()

    # ================================================================
    # TIME CURSOR
    # ================================================================

    def update_time_cursor(self, index):
        if self.results is None:
            return

        (
            times,
            altitudes,
            velocities,
            accelerations,
            thrusts,
            twrs,
        ) = self.get_graph_data()

        if not times:
            return

        if index < 0:
            index = 0

        if index >= len(times):
            index = len(times) - 1

        self.time_label.setText(
            f"{times[index]:.3f} s"
        )

        self.update_graph_cursor(index)

    # ================================================================
    # GRAPH CURSOR
    # ================================================================

    def update_graph_cursor(self, index):
        if self.results is None:
            return

        (
            times,
            altitudes,
            velocities,
            accelerations,
            thrusts,
            twrs,
        ) = self.get_graph_data()

        if not times:
            return

        g_forces = [
            acceleration / 9.80665
            for acceleration in accelerations
        ]

        self.figure.clear()

        if self.current_graph == "All":
            axes = self.figure.subplots(
                3,
                2,
            )

            graphs = [
                (
                    axes[0, 0],
                    altitudes,
                    "Altitude",
                    "Altitude (m)",
                ),
                (
                    axes[0, 1],
                    velocities,
                    "Velocity",
                    "Velocity (m/s)",
                ),
                (
                    axes[1, 0],
                    accelerations,
                    "Acceleration",
                    "Acceleration (m/s²)",
                ),
                (
                    axes[1, 1],
                    g_forces,
                    "G-Force",
                    "G",
                ),
                (
                    axes[2, 0],
                    thrusts,
                    "Thrust",
                    "Thrust (N)",
                ),
                (
                    axes[2, 1],
                    twrs,
                    "TWR",
                    "TWR",
                ),
            ]

            for (
                axis,
                values,
                title,
                ylabel,
            ) in graphs:
                axis.plot(
                    times,
                    values,
                )

                axis.plot(
                    times[index],
                    values[index],
                    marker="o",
                )

                axis.set_title(title)
                axis.set_xlabel("Time (s)")
                axis.set_ylabel(ylabel)
                axis.grid(True)

        else:
            axis = self.figure.add_subplot(111)

            if self.current_graph == "Altitude":
                values = altitudes
                ylabel = "Altitude (m)"

            elif self.current_graph == "Velocity":
                values = velocities
                ylabel = "Velocity (m/s)"

            elif self.current_graph == "Acceleration":
                values = accelerations
                ylabel = "Acceleration (m/s²)"

            elif self.current_graph == "G-Force":
                values = g_forces
                ylabel = "G"

            elif self.current_graph == "Thrust":
                values = thrusts
                ylabel = "Thrust (N)"

            elif self.current_graph == "TWR":
                values = twrs
                ylabel = "TWR"

            else:
                values = []
                ylabel = ""

            axis.plot(
                times,
                values,
            )

            axis.plot(
                times[index],
                values[index],
                marker="o",
            )

            axis.set_title(
                self.current_graph
            )

            axis.set_xlabel(
                "Time (s)"
            )

            axis.set_ylabel(
                ylabel
            )

            axis.grid(True)

        self.figure.tight_layout()
        self.canvas.draw()

    # ================================================================
    # GRAPH SELECTION
    # ================================================================

    def select_graph(self, graph_name):
        self.current_graph = graph_name

        self.update_graph()

        if self.results is not None:
            index = self.time_slider.value()
            self.update_graph_cursor(index)

    # ================================================================
    # SUMMARY
    # ================================================================

    def update_summary(self):
        if self.results is None:
            return

        (
            times,
            altitudes,
            velocities,
            accelerations,
            thrusts,
            twrs,
        ) = self.get_graph_data()

        if not times:
            return

        max_altitude = max(
            altitudes
        )

        max_velocity = max(
            velocities
        )

        max_acceleration = max(
            accelerations
        )

        max_g = max_acceleration / 9.80665

        self.max_altitude_label.setText(
            f"{max_altitude:.2f} m"
        )

        self.max_velocity_label.setText(
            f"{max_velocity:.2f} m/s"
        )

        self.max_acceleration_label.setText(
            f"{max_acceleration:.2f} m/s²"
        )

        self.max_g_label.setText(
            f"{max_g:.2f} G"
        )

        burnout_time = self.results.get(
            "burnout_time"
        )

        burnout_altitude = self.results.get(
            "burnout_altitude"
        )

        apogee_time = self.results.get(
            "apogee_time"
        )

        apogee_altitude = self.results.get(
            "apogee_altitude"
        )

        if (
            burnout_time is not None
            and burnout_altitude is not None
        ):
            self.burnout_label.setText(
                f"{burnout_time:.3f} s / "
                f"{burnout_altitude:.2f} m"
            )
        else:
            self.burnout_label.setText("-")

        if (
            apogee_time is not None
            and apogee_altitude is not None
        ):
            self.apogee_label.setText(
                f"{apogee_time:.3f} s / "
                f"{apogee_altitude:.2f} m"
            )
        else:
            self.apogee_label.setText("-")

    # ================================================================
    # RESET
    # ================================================================

    def reset_simulation(self):
        motors = get_available_motors()

        self.motor_dropdown.blockSignals(True)

        self.motor_dropdown.clear()

        for motor in motors:
            self.motor_dropdown.addItem(
                motor.stem,
                motor,
            )

        self.motor_dropdown.blockSignals(False)

        self.dry_mass_spinbox.setValue(
            150.0
        )

        self.gravity_spinbox.setValue(
            9.81
        )

        self.timestep_spinbox.setValue(
            0.005
        )

        self.results = None

        self.time_slider.setRange(
            0,
            0,
        )

        self.time_slider.setValue(
            0
        )

        self.time_label.setText(
            "0.000 s"
        )

        self.max_altitude_label.setText("-")
        self.max_velocity_label.setText("-")
        self.max_acceleration_label.setText("-")
        self.max_g_label.setText("-")
        self.burnout_label.setText("-")
        self.apogee_label.setText("-")

        self.figure.clear()

        axis = self.figure.add_subplot(111)

        axis.set_title(
            "Run a simulation to view results"
        )

        axis.set_xlabel(
            "Time (s)"
        )

        axis.grid(True)

        self.figure.tight_layout()
        self.canvas.draw()


def main():
    app = QtWidgets.QApplication([])

    app.setStyleSheet(
        MAIN_STYLE
    )

    window = MainWindow()
    window.show()

    app.exec()


if __name__ == "__main__":
    main()