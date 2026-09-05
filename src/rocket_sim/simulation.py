from physics import physics
from motor import get_thrust
from plotting import plot_results


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
dt = 0.1

# Rocket properties
# Propellant decreases; dry mass stays constant
initial_mass = 150
propellant_mass = 50
dry_mass = initial_mass - propellant_mass
mass = dry_mass + propellant_mass

# Initial conditions
time = 0
altitude = 0
velocity = 0

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


for step in range(1000):

    thrust = float(get_thrust(time))


    # Run physics for one timestep
    acceleration, velocity, altitude, thrust = physics(
        thrust, mass, gravity, velocity, altitude, dt
        )
    
    time += dt

    # Burn propellant
    propellant_mass -= dt
    if propellant_mass < 0:
        propellant_mass = 0

    # Update current total mass
    mass = dry_mass + propellant_mass

    # Calculate thrust-to-weight ratio
    weight = (mass / 1000) * gravity
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

plot_results(times, altitudes, velocities, accelerations, thrusts, twrs)