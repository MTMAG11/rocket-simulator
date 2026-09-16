from PySide6 import QtWidgets
from PySide6.QtCore import Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from ..config import SimulationConfig
from ..motor import get_available_motors
from ..simulation import run_simulation as run_simulation_backend
from .styles import MAIN_STYLE


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Rocket Simulator")
        self.resize(1200, 800)
        self.setStyleSheet(MAIN_STYLE)

        self.results = None

        self.setup_ui()
        self.connect_signals()

        self.statusBar().showMessage("Ready")

    def setup_ui(self):
        central_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout()

        main_layout.addLayout(self.create_action_bar())
        main_layout.addLayout(self.create_content_area(), 1)
        main_layout.addWidget(self.create_summary())
        main_layout.addLayout(self.create_time_controls())

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def create_action_bar(self):
        action_bar = QtWidgets.QHBoxLayout()

        self.run_button = QtWidgets.QPushButton("Run Simulation")
        self.reset_button = QtWidgets.QPushButton("Reset")
        self.status_label = QtWidgets.QLabel("Ready")

        action_bar.addWidget(self.run_button)
        action_bar.addWidget(self.reset_button)
        action_bar.addStretch()
        action_bar.addWidget(self.status_label)

        return action_bar

    def create_content_area(self):
        content_layout = QtWidgets.QHBoxLayout()

        content_layout.addWidget(self.create_controls(), 1)
        content_layout.addWidget(self.create_flight_view(), 4)

        return content_layout

    def create_controls(self):
        controls = QtWidgets.QWidget()
        controls_layout = QtWidgets.QVBoxLayout()

        controls_layout.addWidget(self.create_rocket_group())
        controls_layout.addWidget(self.create_environment_group())
        controls_layout.addWidget(self.create_simulation_group())
        controls_layout.addStretch()

        controls.setLayout(controls_layout)

        return controls

    def create_rocket_group(self):
        rocket_group = QtWidgets.QGroupBox("Rocket")
        rocket_form = QtWidgets.QFormLayout()

        self.motor_input = QtWidgets.QComboBox()

        for motor_path in get_available_motors():
            self.motor_input.addItem(motor_path.stem, motor_path)

        self.dry_mass_input = QtWidgets.QDoubleSpinBox()
        self.dry_mass_input.setRange(0.1, 10000)
        self.dry_mass_input.setValue(150)
        self.dry_mass_input.setSingleStep(1)
        self.dry_mass_input.setSuffix(" g")

        rocket_form.addRow("Motor:", self.motor_input)
        rocket_form.addRow("Dry Mass:", self.dry_mass_input)

        rocket_group.setLayout(rocket_form)

        return rocket_group

    def create_environment_group(self):
        environment_group = QtWidgets.QGroupBox("Environment")
        environment_form = QtWidgets.QFormLayout()

        self.gravity_input = QtWidgets.QDoubleSpinBox()
        self.gravity_input.setRange(0, 30)
        self.gravity_input.setValue(9.81)
        self.gravity_input.setSingleStep(0.01)
        self.gravity_input.setSuffix(" m/s²")

        environment_form.addRow("Gravity:", self.gravity_input)

        environment_group.setLayout(environment_form)

        return environment_group

    def create_simulation_group(self):
        simulation_group = QtWidgets.QGroupBox("Simulation")
        simulation_form = QtWidgets.QFormLayout()

        self.timestep_input = QtWidgets.QDoubleSpinBox()
        self.timestep_input.setRange(0.0001, 1)
        self.timestep_input.setValue(0.005)
        self.timestep_input.setSingleStep(0.001)
        self.timestep_input.setDecimals(4)
        self.timestep_input.setSuffix(" s")

        simulation_form.addRow("Timestep:", self.timestep_input)

        simulation_group.setLayout(simulation_form)

        return simulation_group

    def create_flight_view(self):
        self.flight_tabs = QtWidgets.QTabWidget()

        self.flight_tabs.addTab(
            self.create_graphs_tab(),
            "Graphs",
        )

        self.flight_tabs.addTab(
            self.create_3d_view_tab(),
            "3D View",
        )

        return self.flight_tabs

    def create_graphs_tab(self):
        graphs_widget = QtWidgets.QWidget()
        graphs_layout = QtWidgets.QHBoxLayout()

        navigation = QtWidgets.QVBoxLayout()

        self.graph_buttons = []

        graph_names = [
            "Altitude",
            "Velocity",
            "Acceleration",
            "G-Force",
            "Thrust",
            "TWR",
            "All",
        ]

        for index, name in enumerate(graph_names):
            button = QtWidgets.QPushButton(name)
            button.setCheckable(True)

            button.clicked.connect(
                lambda checked, i=index: self.select_graph(i)
            )

            navigation.addWidget(button)
            self.graph_buttons.append(button)

        navigation.addStretch()

        self.graph_stack = QtWidgets.QStackedWidget()

        self.graph_stack.addWidget(
            self.create_altitude_graph()
        )

        self.graph_stack.addWidget(
            self.create_placeholder_graph("Velocity")
        )

        self.graph_stack.addWidget(
            self.create_placeholder_graph("Acceleration")
        )

        self.graph_stack.addWidget(
            self.create_placeholder_graph("G-Force")
        )

        self.graph_stack.addWidget(
            self.create_placeholder_graph("Thrust")
        )

        self.graph_stack.addWidget(
            self.create_placeholder_graph("TWR")
        )

        self.graph_stack.addWidget(
            self.create_placeholder_graph("All")
        )

        graphs_layout.addLayout(navigation)
        graphs_layout.addWidget(self.graph_stack, 1)

        graphs_widget.setLayout(graphs_layout)

        self.select_graph(0)

        return graphs_widget

    def create_altitude_graph(self):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)

        self.altitude_ax = self.figure.add_subplot(111)

        self.altitude_ax.set_title("Altitude")
        self.altitude_ax.set_xlabel("Time (s)")
        self.altitude_ax.set_ylabel("Altitude (m)")
        self.altitude_ax.grid(True)

        self.altitude_line, = self.altitude_ax.plot(
            [],
            [],
        )

        self.altitude_marker, = self.altitude_ax.plot(
            [],
            [],
            marker="o",
            linestyle="",
        )

        layout.addWidget(self.canvas)

        widget.setLayout(layout)

        return widget

    def create_placeholder_graph(self, name):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()

        label = QtWidgets.QLabel(
            f"{name}\nComing later"
        )

        label.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

        layout.addWidget(label)

        widget.setLayout(layout)

        return widget

    def create_3d_view_tab(self):
        view = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()

        label = QtWidgets.QLabel(
            "3D View\nComing later"
        )

        label.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

        layout.addWidget(label)

        view.setLayout(layout)

        return view

    def create_summary(self):
        summary_group = QtWidgets.QGroupBox("Flight Summary")
        summary_layout = QtWidgets.QHBoxLayout()

        self.max_altitude = QtWidgets.QLabel("-- m")
        self.max_velocity = QtWidgets.QLabel("-- m/s")
        self.max_acceleration = QtWidgets.QLabel("-- m/s²")
        self.max_g = QtWidgets.QLabel("-- g")
        self.burnout_time = QtWidgets.QLabel("-- s")
        self.apogee_time = QtWidgets.QLabel("-- s")

        for value in (
            self.max_altitude,
            self.max_velocity,
            self.max_acceleration,
            self.max_g,
            self.burnout_time,
            self.apogee_time,
        ):
            value.setObjectName("summary_value")

        summary_layout.addWidget(
            self.create_summary_item(
                "Max Altitude",
                self.max_altitude,
            )
        )

        summary_layout.addWidget(
            self.create_summary_item(
                "Max Velocity",
                self.max_velocity,
            )
        )

        summary_layout.addWidget(
            self.create_summary_item(
                "Max Acceleration",
                self.max_acceleration,
            )
        )

        summary_layout.addWidget(
            self.create_summary_item(
                "Max G",
                self.max_g,
            )
        )

        summary_layout.addWidget(
            self.create_summary_item(
                "Burnout",
                self.burnout_time,
            )
        )

        summary_layout.addWidget(
            self.create_summary_item(
                "Apogee",
                self.apogee_time,
            )
        )

        summary_group.setLayout(summary_layout)

        return summary_group

    def create_time_controls(self):
        time_layout = QtWidgets.QVBoxLayout()

        time_labels = QtWidgets.QHBoxLayout()

        self.start_time = QtWidgets.QLabel("0.00 s")
        self.current_time = QtWidgets.QLabel("0.00 s")
        self.end_time = QtWidgets.QLabel("-- s")

        time_labels.addWidget(self.start_time)
        time_labels.addStretch()
        time_labels.addWidget(self.current_time)
        time_labels.addStretch()
        time_labels.addWidget(self.end_time)

        self.time_slider = QtWidgets.QSlider(
            Qt.Orientation.Horizontal
        )

        self.time_slider.setRange(0, 0)
        self.time_slider.setValue(0)
        self.time_slider.setEnabled(False)

        time_layout.addLayout(time_labels)
        time_layout.addWidget(self.time_slider)

        return time_layout

    def create_summary_item(self, title, value):
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()

        title_label = QtWidgets.QLabel(title)

        title_label.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

        value.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

        layout.addWidget(title_label)
        layout.addWidget(value)

        widget.setLayout(layout)

        return widget

    def connect_signals(self):
        self.run_button.clicked.connect(
            self.run_simulation
        )

        self.reset_button.clicked.connect(
            self.reset_simulation
        )

        self.time_slider.valueChanged.connect(
            self.update_time_cursor
        )

    def get_simulation_config(self):
        return SimulationConfig(
            motor=self.motor_input.currentData(),
            rocket_dry_mass=self.dry_mass_input.value(),
            gravity=self.gravity_input.value(),
            dt=self.timestep_input.value(),
        )

    def run_simulation(self):
        config = self.get_simulation_config()

        results = run_simulation_backend(config)

        self.results = results

        self.max_altitude.setText(
            f"{results['apogee_altitude']:.2f} m"
        )

        self.max_velocity.setText(
            f"{results['max_velocity']:.2f} m/s"
        )

        self.max_acceleration.setText(
            f"{results['max_acceleration']:.2f} m/s²"
        )

        max_g = results["max_acceleration"] / 9.80665

        self.max_g.setText(
            f"{max_g:.2f} g"
        )

        self.burnout_time.setText(
            f"{results['burnout_time']:.2f} s"
        )

        self.apogee_time.setText(
            f"{results['apogee_time']:.2f} s"
        )

        self.update_graph()

        times = results["times"]

        self.time_slider.setRange(
            0,
            len(times) - 1,
        )

        self.time_slider.setEnabled(True)
        self.time_slider.setValue(0)

        self.start_time.setText(
            f"{times[0]:.2f} s"
        )

        self.current_time.setText(
            f"{times[0]:.2f} s"
        )

        self.end_time.setText(
            f"{times[-1]:.2f} s"
        )

        self.update_time_cursor(0)

        self.statusBar().showMessage(
            "Simulation complete"
        )

    def update_graph(self):
        if self.results is None:
            return

        times = self.results["times"]
        altitudes = self.results["altitudes"]

        self.altitude_line.set_data(
            times,
            altitudes,
        )

        self.altitude_ax.relim()
        self.altitude_ax.autoscale_view()

        self.canvas.draw_idle()

    def update_time_cursor(self, index):
        if self.results is None:
            return

        times = self.results["times"]
        altitudes = self.results["altitudes"]

        if not times:
            return

        index = max(
            0,
            min(index, len(times) - 1),
        )

        current_time = times[index]
        current_altitude = altitudes[index]

        self.current_time.setText(
            f"{current_time:.2f} s"
        )

        self.altitude_marker.set_data(
            [current_time],
            [current_altitude],
        )

        self.canvas.draw_idle()

    def select_graph(self, index):
        self.graph_stack.setCurrentIndex(index)

        for i, button in enumerate(self.graph_buttons):
            button.setChecked(i == index)

    def reset_simulation(self):
        self.results = None

        self.max_altitude.setText("-- m")
        self.max_velocity.setText("-- m/s")
        self.max_acceleration.setText("-- m/s²")
        self.max_g.setText("-- g")
        self.burnout_time.setText("-- s")
        self.apogee_time.setText("-- s")

        self.start_time.setText("0.00 s")
        self.current_time.setText("0.00 s")
        self.end_time.setText("-- s")

        self.time_slider.setRange(0, 0)
        self.time_slider.setValue(0)
        self.time_slider.setEnabled(False)

        self.altitude_line.set_data([], [])
        self.altitude_marker.set_data([], [])

        self.altitude_ax.relim()
        self.altitude_ax.autoscale_view()

        self.canvas.draw_idle()

        self.statusBar().showMessage("Reset")


app = QtWidgets.QApplication([])
window = MainWindow()

window.show()

app.exec()