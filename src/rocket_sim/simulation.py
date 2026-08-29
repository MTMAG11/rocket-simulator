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

def physics(thrust, mass, gravity, velocity, altitude, dt):
    acceleration = (thrust - mass * gravity) / mass
    velocity += acceleration * dt
    altitude += velocity * dt

    return acceleration, velocity, altitude

for step in range(2):
    acceleration, velocity, altitude = physics(thrust, mass, gravity, velocity, altitude, dt)
    time += dt
    print (time, altitude, velocity, acceleration)