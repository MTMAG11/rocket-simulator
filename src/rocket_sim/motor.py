from rocketpy import SolidMotor, Function
from pathlib import Path

motor_directory = Path(__file__).resolve().parents[2] / "data" / "motors"
motor_file = motor_directory / "Klima_A6.eng"

# Motor data
comments, header, data_points = SolidMotor.import_eng(
    str(motor_file)
)

# Calculate total impulse from thrust curve
total_impulse = sum(
    (data_points[i][1] + data_points[i + 1][1]) / 2
    * (data_points[i + 1][0] - data_points[i][0])
    for i in range(len(data_points) - 1)
)

print("Total impulse:", total_impulse, "Ns")

# Motor properties
motor_name = header[0]
diameter_mm = float(header[1])
length_mm = float(header[2])
delays = int(header[3])
propellant_mass = float(header[4]) * 1000
total_motor_mass = float(header[5]) * 1000
dry_motor_mass = total_motor_mass - propellant_mass
burn_time = data_points[-1][0]

# Thrust curve
get_thrust = Function(
    data_points,
    interpolation="linear",
    extrapolation="constant"
)


def get_propellant_mass(time):
    if time <= 0:
        return propellant_mass

    if time >= burn_time:
        return 0

    impulse_used = 0

    for i in range(len(data_points) - 1):
        t1, thrust1 = data_points[i]
        t2, thrust2 = data_points[i + 1]

        if time <= t1:
            break

        interval_end = min(time, t2)
        interval_duration = interval_end - t1

        if interval_duration > 0:
            if interval_end < t2:
                thrust_end = thrust1 + (
                    (thrust2 - thrust1)
                    * (interval_end - t1)
                    / (t2 - t1)
                )
            else:
                thrust_end = thrust2

            impulse_used += (
                (thrust1 + thrust_end) / 2
                * interval_duration
            )

        if time <= t2:
            break

    total_impulse = 0

    for i in range(len(data_points) - 1):
        t1, thrust1 = data_points[i]
        t2, thrust2 = data_points[i + 1]

        total_impulse += (
            (thrust1 + thrust2) / 2
            * (t2 - t1)
        )

    fraction_consumed = impulse_used / total_impulse

    return propellant_mass * (1 - fraction_consumed)