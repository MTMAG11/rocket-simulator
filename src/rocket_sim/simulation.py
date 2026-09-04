import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np
from rocketpy import SolidMotor, Function



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

# Thrust properties
comments, description, data_points = SolidMotor.import_eng(
    r"C:/Users/Rishaan/Downloads/Klima_A6.eng"
)
get_thrust = Function(data_points, interpolation="linear", extrapolation="constant")

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

def physics(thrust, mass, gravity, velocity, altitude, dt):

    # Calculate acceleration from net force / mass
    acceleration = (thrust - (mass/1000) * gravity) / (mass/1000)

    # Update velocity and altitude
    velocity += acceleration * dt
    altitude += velocity * dt


    # Stop the rocket at the ground
    if altitude <= 0 and velocity < 0:
        altitude = 0
        velocity = 0
        acceleration = 0

    return acceleration, velocity, altitude, thrust

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

# Create a 2x3 grid of graphs
fig, axes = plt.subplots(2, 3, figsize=(14, 10))

# Altitude
axes[0][0].plot(times, altitudes)
axes[0][0].set_title("Altitude vs Time")
axes[0][0].set_xlabel("Time (s)")
axes[0][0].set_ylabel("Altitude (m)")

# Velocity
axes[0][1].plot(times, velocities)
axes[0][1].set_title("Velocity vs Time")
axes[0][1].set_xlabel("Time (s)")
axes[0][1].set_ylabel("Velocity (m/s)")

# Acceleration
axes[1][0].plot(times, accelerations)
axes[1][0].set_title("Acceleration vs Time")
axes[1][0].set_xlabel("Time (s)")
axes[1][0].set_ylabel("Acceleration (m/s^2)")

# Thrust
axes[1][1].plot(times, thrusts)
axes[1][1].set_title("Thrust vs Time")
axes[1][1].set_xlabel("Time (s)")
axes[1][1].set_ylabel("Thrust (N)")

# TWR
axes[0][2].plot(times, twrs)
axes[0][2].set_title("TWR vs Time")
axes[0][2].set_xlabel("Time (s)")
axes[0][2].set_ylabel("TWR")

plt.show()