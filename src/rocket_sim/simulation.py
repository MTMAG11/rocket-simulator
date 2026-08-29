import matplotlib.pyplot as plt

# Variables: 
# time = time (s)
# altitude = height (m)
# velocity = velocity (m/s)
# mass = mass (kg)
# thrust = thrust (N)
# gravity = gravitational acceleration (m/s^2)
# dt = amount of time per step (s)


time = 0
altitude = 0
velocity = 0
mass = 10
thrust = 0
gravity = 9.81
dt = 0.1
times = []
altitudes = []
velocities = []
accelerations = []

def physics(thrust, mass, gravity, velocity, altitude, dt):
    acceleration = (thrust - mass * gravity) / mass
    velocity += acceleration * dt
    altitude += velocity * dt

    return acceleration, velocity, altitude

for step in range(100):
    acceleration, velocity, altitude = physics(thrust, mass, gravity, velocity, altitude, dt)
    time += dt

    if altitude < 0:
        altitude = 0
        velocity = 0
        acceleration = 0

    times.append(time)
    altitudes.append(altitude)
    velocities.append(velocity)
    accelerations.append(acceleration)
#    print (round(time, 2), round(altitude, 2), round(velocity, 2), round(acceleration, 2))

plt.plot(times, altitudes)
plt.title("Altitude vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.show()