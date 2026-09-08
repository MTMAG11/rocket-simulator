from physics import physics
from plotting import plot_results
from motor import (
    get_thrust,
    propellant_mass as initial_propellant_mass,
    total_motor_mass,
    dry_motor_mass,
    burn_time,
    get_propellant_mass,
)


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


while True:

    thrust = float(get_thrust(time))


    # Run physics for one timestep
    acceleration, velocity, altitude, thrust = physics(
        thrust, mass, gravity, velocity, altitude, dt
        )
    
    time += dt

    # Burn propellant based on thrust curve
    propellant_mass = get_propellant_mass(time)

    # Update current total mass
    mass = dry_mass + propellant_mass

    # Calculate thrust-to-weight ratio
    weight = mass * gravity
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

print("Burn time:", burn_time)
print("Initial mass:", dry_mass + initial_propellant_mass, "g")
print("Dry mass:", dry_mass, "g")
print("Peak thrust:", max(thrusts), "N")
print("Peak acceleration:", max(accelerations), "m/s^2")
print("Max velocity:", max(velocities), "m/s")
print("Max altitude:", max(altitudes), "m")

print("\nFlight data:")
for i in range(0, len(times), int(0.1 / dt)):
    print(
        f"t={times[i]:.2f}s | "
        f"T={thrusts[i]:.2f}N | "
        f"a={accelerations[i]:.2f}m/s² | "
        f"v={velocities[i]:.2f}m/s | "
        f"h={altitudes[i]:.2f}m"
    )

plot_results(times, altitudes, velocities, accelerations, thrusts, twrs)