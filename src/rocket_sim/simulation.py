from .physics import physics
from .motor import load_motor
from .state import RocketState


def run_simulation(config):
    # Variables
    gravity = config.gravity
    dt = config.dt

    # Load motor data
    motor = load_motor(config.motor)

    # Mass properties
    rocket_dry_mass = config.rocket_dry_mass
    propellant_mass = motor.propellant_mass
    dry_mass = rocket_dry_mass + motor.dry_motor_mass
    mass = dry_mass + propellant_mass

    # Initial state
    state = RocketState()

    time = 0

    flight_started = False

    # Data storage
    times = []
    xs = []
    ys = []
    vxs = []
    vys = []
    axs = []
    ays = []
    thrusts = []
    twrs = []

    # Flight events
    burnout_time = None
    burnout_altitude = None
    burnout_velocity = None

    apogee_time = None
    apogee_altitude = None

    # Maximum values
    max_velocity = 0
    max_velocity_time = None

    max_acceleration = 0
    max_acceleration_time = None

    while True:
        thrust = float(motor.thrust(time))

        # Run physics
        state = physics(
            thrust,
            mass,
            gravity,
            state,
            dt,
        )

        time += dt

        # Flight detection
        if state.y > 0 or state.vy > 0:
            flight_started = True

        # Calculate velocity
        velocity = (state.vx**2 + state.vy**2) ** 0.5

        # Calculate acceleration
        acceleration = (state.ax**2 + state.ay**2) ** 0.5

        # Burnout
        if time >= motor.burn_time and burnout_time is None:
            burnout_time = time
            burnout_altitude = state.y
            burnout_velocity = velocity

        # Apogee
        if flight_started and state.vy <= 0 and apogee_time is None:
            apogee_time = time
            apogee_altitude = state.y

        # Maximum velocity
        if velocity > max_velocity:
            max_velocity = velocity
            max_velocity_time = time

        # Maximum acceleration
        if apogee_time is None and acceleration > max_acceleration:
            max_acceleration = acceleration
            max_acceleration_time = time

        # Burn propellant
        propellant_mass = motor.get_propellant_mass(time)

        # Update mass
        mass = dry_mass + propellant_mass

        # Calculate thrust-to-weight ratio
        weight = (mass / 1000) * gravity
        twr = thrust / weight

        # Store results
        times.append(time)
        xs.append(state.x)
        ys.append(state.y)
        vxs.append(state.vx)
        vys.append(state.vy)
        axs.append(state.ax)
        ays.append(state.ay)
        thrusts.append(thrust)
        twrs.append(twr)

        # Temporary stop condition
        if time >= motor.burn_time + 20:
            break

    # Simulation results
    simulation_results = {
        "times": times,
        "xs": xs,
        "ys": ys,
        "vxs": vxs,
        "vys": vys,
        "axs": axs,
        "ays": ays,
        "thrusts": thrusts,
        "twrs": twrs,
        "burnout_time": burnout_time,
        "burnout_altitude": burnout_altitude,
        "burnout_velocity": burnout_velocity,
        "apogee_time": apogee_time,
        "apogee_altitude": apogee_altitude,
        "max_velocity": max_velocity,
        "max_velocity_time": max_velocity_time,
        "max_acceleration": max_acceleration,
        "max_acceleration_time": max_acceleration_time,
    }

    return simulation_results