def physics(thrust, mass, gravity, velocity, altitude, dt):
    mass_kg = mass / 1000

    acceleration = (thrust - mass_kg * gravity) / mass_kg
    velocity += acceleration * dt
    altitude += velocity * dt

    if altitude <= 0 and velocity < 0:
        altitude = 0
        velocity = 0
        acceleration = 0

    return acceleration, velocity, altitude, thrust