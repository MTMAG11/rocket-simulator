# Rocket Simulator

This is a Python-based rocket flight simulator that I am building as part of my Autonomous Rocket Project.

The goal is to eventually use the simulator to develop and test the guidance, navigation, and control systems for an autonomous rocket. Right now, I am starting with a simple 1D vertical flight model and gradually adding more realistic physics and systems.

## What It Can Do

* Simulate vertical rocket flight
* Calculate acceleration, velocity, and altitude
* Account for gravity
* Track rocket and propellant mass
* Use real motor thrust curves
* Calculate thrust-to-weight ratio
* Record flight data
* Plot simulation results

## Project Structure

```text
rocket-simulator/
├── data/
│   └── motors/              # Motor data
├── src/
│   └── rocket_sim/
│       ├── motor.py         # Motor data and thrust curves
│       ├── physics.py       # Physics calculations
│       ├── plotting.py      # Graphs and visualization
│       └── simulation.py    # Runs the simulation
├── README.md
└── pyproject.toml
```

## Motor Data

The motor thrust curves used by the simulator are `.eng` files sourced from ThrustCurve.org.

Keeping the motor data separate from the simulation code makes it easier to test different motors without changing the physics code.

## What's Next

Some of the things I plan to add as the simulator develops:

* Improve the propellant mass model
* Add aerodynamic drag
* Add atmospheric effects
* Expand the flight model beyond 1D
* Add sensor models
* Add state estimation
* Develop guidance and control systems
* Simulate TVC and aerodynamic control surfaces
* Use the simulator to test the autonomous landing system

## About the Project

This simulator is part of my larger Autonomous Rocket Project. I am building the project from the ground up to learn the physics, mathematics, programming, and engineering behind autonomous rocket flight.

The simulator is still in an early stage and is not yet a complete or validated flight simulator.
