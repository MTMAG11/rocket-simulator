from .state import RocketState


def physics(thrust, mass, gravity, state, dt, launch_angle=0.0):
    mass_kg = mass / 1000

    # Convert launch angle from degrees to radians
    import math
    angle = math.radians(launch_angle)

    # Thrust components
    thrust_x = thrust * math.sin(angle)
    thrust_y = thrust * math.cos(angle)

    # Forces
    force_x = thrust_x
    force_y = thrust_y - mass_kg * gravity

    # Accelerations
    ax = force_x / mass_kg
    ay = force_y / mass_kg

    # Integrate velocity
    state.vx += ax * dt
    state.vy += ay * dt

    # Integrate position
    state.x += state.vx * dt
    state.y += state.vy * dt

    # Update acceleration in the state
    state.ax = ax
    state.ay = ay

    return state