from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..config import SimulationConfig
from ..motor import get_available_motors
from .styles import MAIN_STYLE


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Rocket Simulator")
        self.resize(1200, 800)
        self.setStyleSheet(MAIN_STYLE)

        self.setup_ui()
        self.connect_signals()

        self.statusBar().showMessage("Ready")

    def setup_ui(self):
        central_widget = QWidget()
        main_layout = QVBoxLayout()

        main_layout.addLayout(self.create_action_bar())
        main_layout.addLayout(self.create_content_area(), 1)
        main_layout.addWidget(self.create_summary())
        main_layout.addLayout(self.create_time_controls())

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def create_action_bar(self):
        action_bar = QHBoxLayout()

        self.run_button = QPushButton("Run Simulation")
        self.reset_button = QPushButton("Reset")
        self.status_label = QLabel("Ready")

        action_bar.addWidget(self.run_button)
        action_bar.addWidget(self.reset_button)
        action_bar.addStretch()
        action_bar.addWidget(self.status_label)

        return action_bar

    def create_content_area(self):
        content_layout = QHBoxLayout()

        content_layout.addWidget(self.create_controls(), 1)
        content_layout.addWidget(self.create_flight_view(), 4)

        return content_layout

    def create_controls(self):
        controls = QWidget()
        controls_layout = QVBoxLayout()

        controls_layout.addWidget(self.create_rocket_group())
        controls_layout.addWidget(self.create_environment_group())
        controls_layout.addWidget(self.create_simulation_group())
        controls_layout.addStretch()

        controls.setLayout(controls_layout)

        return controls

    def create_rocket_group(self):
        rocket_group = QGroupBox("Rocket")
        rocket_form = QFormLayout()

        self.motor_input = QComboBox()

        for motor_path in get_available_motors():
            self.motor_input.addItem(motor_path.stem, motor_path)

        self.dry_mass_input = QDoubleSpinBox()
        self.dry_mass_input.setRange(0.1, 10000)
        self.dry_mass_input.setValue(150)
        self.dry_mass_input.setSingleStep(1)
        self.dry_mass_input.setSuffix(" g")

        rocket_form.addRow("Motor:", self.motor_input)
        rocket_form.addRow("Dry Mass:", self.dry_mass_input)

        rocket_group.setLayout(rocket_form)

        return rocket_group

    def create_environment_group(self):
        environment_group = QGroupBox("Environment")
        environment_form = QFormLayout()

        self.gravity_input = QDoubleSpinBox()
        self.gravity_input.setRange(0, 30)
        self.gravity_input.setValue(9.81)
        self.gravity_input.setSingleStep(0.01)
        self.gravity_input.setSuffix(" m/s²")

        environment_form.addRow("Gravity:", self.gravity_input)

        environment_group.setLayout(environment_form)

        return environment_group

    def create_simulation_group(self):
        simulation_group = QGroupBox("Simulation")
        simulation_form = QFormLayout()

        self.timestep_input = QDoubleSpinBox()
        self.timestep_input.setRange(0.0001, 1)
        self.timestep_input.setValue(0.005)
        self.timestep_input.setSingleStep(0.001)
        self.timestep_input.setDecimals(4)
        self.timestep_input.setSuffix(" s")

        simulation_form.addRow("Timestep:", self.timestep_input)

        simulation_group.setLayout(simulation_form)

        return simulation_group

    def create_flight_view(self):
        flight_view = QLabel("Flight View")
        flight_view.setObjectName("flight_view")
        flight_view.setAlignment(Qt.AlignmentFlag.AlignCenter)

        return flight_view

    def create_summary(self):
        summary_group = QGroupBox("Flight Summary")
        summary_layout = QHBoxLayout()

        self.max_altitude = QLabel("-- m")
        self.max_velocity = QLabel("-- m/s")
        self.max_acceleration = QLabel("-- m/s²")
        self.burnout_time = QLabel("-- s")
        self.apogee_time = QLabel("-- s")

        for value in (
            self.max_altitude,
            self.max_velocity,
            self.max_acceleration,
            self.burnout_time,
            self.apogee_time,
        ):
            value.setObjectName("summary_value")

        summary_layout.addWidget(
            self.create_summary_item("Max Altitude", self.max_altitude)
        )
        summary_layout.addWidget(
            self.create_summary_item("Max Velocity", self.max_velocity)
        )
        summary_layout.addWidget(
            self.create_summary_item("Max Acceleration", self.max_acceleration)
        )
        summary_layout.addWidget(
            self.create_summary_item("Burnout", self.burnout_time)
        )
        summary_layout.addWidget(
            self.create_summary_item("Apogee", self.apogee_time)
        )

        summary_group.setLayout(summary_layout)

        return summary_group

    def create_time_controls(self):
        time_layout = QVBoxLayout()

        time_labels = QHBoxLayout()

        self.start_time = QLabel("0.00 s")
        self.current_time = QLabel("0.00 s")
        self.end_time = QLabel("5.00 s")

        time_labels.addWidget(self.start_time)
        time_labels.addStretch()
        time_labels.addWidget(self.current_time)
        time_labels.addStretch()
        time_labels.addWidget(self.end_time)

        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_slider.setRange(0, 5000)
        self.time_slider.setValue(0)

        time_layout.addLayout(time_labels)
        time_layout.addWidget(self.time_slider)

        return time_layout

    def create_summary_item(self, title, value):
        widget = QWidget()
        layout = QVBoxLayout()

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(title_label)
        layout.addWidget(value)

        widget.setLayout(layout)

        return widget

    def connect_signals(self):
        self.run_button.clicked.connect(self.run_simulation)
        self.reset_button.clicked.connect(self.reset_simulation)

    def get_simulation_config(self):
        return SimulationConfig(
            motor=self.motor_input.currentData(),
            rocket_dry_mass=self.dry_mass_input.value(),
            gravity=self.gravity_input.value(),
            dt=self.timestep_input.value(),
        )

    def run_simulation(self):
        config = self.get_simulation_config()

        print(config)

        self.statusBar().showMessage("Configuration loaded")

    def reset_simulation(self):
        self.statusBar().showMessage("Reset")


app = QApplication([])
window = MainWindow()

window.show()

app.exec()