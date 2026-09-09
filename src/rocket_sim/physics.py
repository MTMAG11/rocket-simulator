def physics(thrust, mass, gravity, velocity, altitude, dt):
    mass_kg = mass / 1000

    acceleration = (thrust - mass_kg * gravity) / mass_kg
    velocity += acceleration * dt
    altitude += velocity * dt

    ground_contact = False

    if altitude <= 0 and velocity < 0:
        ground_contact = True
        altitude = 0

    return acceleration, velocity, altitude, thrust, ground_contact