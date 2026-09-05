from rocketpy import SolidMotor, Function

# Thrust properties
comments, description, data_points = SolidMotor.import_eng(
    r"C:/Users/Rishaan/Downloads/Klima_A6.eng"
)
get_thrust = Function(data_points, interpolation="linear", extrapolation="constant")