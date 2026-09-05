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