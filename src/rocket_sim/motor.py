from rocketpy import SolidMotor, Function
from pathlib import Path

motor_directory = Path(__file__).resolve().parents[2] / "data" / "motors"

motor_file = motor_directory / "Klima_A6.eng"

# Thrust properties
comments, description, data_points = SolidMotor.import_eng(
    str(motor_file)
)
get_thrust = Function(data_points, interpolation="linear", extrapolation="constant")