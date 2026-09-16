from dataclasses import dataclass
from pathlib import Path

from rocketpy import Function, SolidMotor


motor_directory = Path(__file__).resolve().parents[2] / "data" / "motors"


@dataclass
class Motor:
    name: str
    diameter_mm: float
    length_mm: float
    delays: str

    propellant_mass: float
    total_motor_mass: float
    dry_motor_mass: float

    burn_time: float
    total_impulse: float

    thrust: Function
    data_points: list[tuple[float, float]]

    def get_propellant_mass(self, time):
        if time <= 0:
            return self.propellant_mass

        if time >= self.burn_time:
            return 0

        impulse_used = 0

        for i in range(len(self.data_points) - 1):
            t1, thrust1 = self.data_points[i]
            t2, thrust2 = self.data_points[i + 1]

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

        fraction_consumed = impulse_used / self.total_impulse

        return self.propellant_mass * (1 - fraction_consumed)


def load_motor(motor_file: Path) -> Motor:
    comments, header, data_points = SolidMotor.import_eng(
        str(motor_file)
    )

    total_impulse = sum(
        (data_points[i][1] + data_points[i + 1][1]) / 2
        * (data_points[i + 1][0] - data_points[i][0])
        for i in range(len(data_points) - 1)
    )

    motor_name = header[0]
    diameter_mm = float(header[1])
    length_mm = float(header[2])
    delays = header[3]

    propellant_mass = float(header[4]) * 1000
    total_motor_mass = float(header[5]) * 1000
    dry_motor_mass = total_motor_mass - propellant_mass

    burn_time = data_points[-1][0]

    thrust = Function(
        data_points,
        interpolation="linear",
        extrapolation="constant",
    )

    return Motor(
        name=motor_name,
        diameter_mm=diameter_mm,
        length_mm=length_mm,
        delays=delays,
        propellant_mass=propellant_mass,
        total_motor_mass=total_motor_mass,
        dry_motor_mass=dry_motor_mass,
        burn_time=burn_time,
        total_impulse=total_impulse,
        thrust=thrust,
        data_points=data_points,
    )


def get_available_motors():
    return sorted(motor_directory.glob("*.eng"))