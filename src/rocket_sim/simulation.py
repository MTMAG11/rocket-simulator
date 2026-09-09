from physics import physics
from plotting import plot_results
from motor import (
    get_thrust,
    propellant_mass as initial_propellant_mass,
    dry_motor_mass,
    burn_time,
    get_propellant_mass,
)

def run_simulation():
    # Variables: 
    # time = time (s)
    # altitude = height (m)
    # velocity = velocity (m/s)
    # mass = mass (g)
    # thrust = thrust (N)
    # gravity = gravitational acceleration (m/s^2)
    # dt = amount of time per step (s)

    # Simulation parameters
    gravity = 9.81
    dt = 0.005

    # Mass properties
    rocket_dry_mass = 150  # g
    propellant_mass = initial_propellant_mass
    dry_mass = rocket_dry_mass + dry_motor_mass
    mass = dry_mass + propellant_mass

    # Initial conditions
    time = 0
    altitude = 0
    velocity = 0

    flight_started = False

    #Ground Timer
    ground_time = 0

    # Data storage
    # Values from each timestep are stored for plotting
    times = []
    altitudes = []
    velocities = []
    accelerations = []
    thrusts = []
    twrs = []

    burnout_time = None
    burnout_altitude = None
    burnout_velocity = None

    apogee_time = None
    apogee_altitude = None

    max_velocity = 0
    max_velocity_time = None

    max_acceleration = 0
    max_acceleration_time = None

    landing_time = None
    landing_velocity = None



    while True:


        thrust = float(get_thrust(time))


        # Run physics for one timestep
        acceleration, velocity, altitude, thrust, ground_contact = physics(
            thrust, mass, gravity, velocity, altitude, dt
            )


        time += dt


        if altitude > 0 or velocity > 0:
            flight_started = True


        if flight_started and ground_contact and landing_time is None:
            landing_time = time
            landing_velocity = velocity

        if ground_contact:
            velocity = 0


        # Retrieve data
        if time >= burn_time and burnout_time is None:
            burnout_time = time
            burnout_altitude = altitude
            burnout_velocity = velocity

        if flight_started and velocity <= 0 and apogee_time is None:
            apogee_time = time
            apogee_altitude = altitude

        if velocity > max_velocity:
            max_velocity = velocity
            max_velocity_time = time

        if apogee_time is None and acceleration > max_acceleration:
            max_acceleration = acceleration
            max_acceleration_time = time


        # Burn propellant based on thrust curve
        propellant_mass = get_propellant_mass(time)

        # Update current total mass
        mass = dry_mass + propellant_mass

        # Calculate thrust-to-weight ratio
        weight = (mass/1000) * gravity
        twr = thrust / weight

        # Stop after the rocket has been on the ground for 5 seconds
        if altitude == 0 and velocity == 0:
            ground_time += dt
            if ground_time >= 5:
                break

        
        # Store results for plotting
        times.append(time)
        altitudes.append(altitude)
        velocities.append(velocity)
        accelerations.append(acceleration)
        thrusts.append(thrust)
        twrs.append(twr)

    simulation_results = {
        "times": times,
        "altitudes": altitudes,
        "velocities": velocities,
        "accelerations": accelerations,
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
        "landing_time": landing_time,
        "landing_velocity": landing_velocity,
    }

    return simulation_results

if __name__ == "__main__":
    simulation_results = run_simulation()

    plot_results(
        simulation_results["times"],
        simulation_results["altitudes"],
        simulation_results["velocities"],
        simulation_results["accelerations"],
        simulation_results["thrusts"],
        simulation_results["twrs"],
    )