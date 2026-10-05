# Testing

```bash
pytest                      # 160 tests, ~10 min on a slow machine (includes 3 real-flight regression tests marked slow)
pytest -m "not slow"        # skip the slow tests
ruff check src tests && ruff format --check src tests
mypy src/rocket_sim
```

Expected answers come from closed-form physics or independent data wherever possible, not from the simulator.

| file | what it verifies |
|---|---|
| `test_math3d.py` | rotation/DCM/quaternion identities, composition order, launch attitude, FRD convention, Euler extraction |
| `test_environment.py` | ISA vs the 1976 table at every layer, pressure inversion, gravity `GM/(R+h)^2`, wind direction convention, profiles, turbulence statistics/determinism, gusts, terrain |
| `test_motor.py` | `.eng` parsing/units, impulse vs numeric quadrature, propellant depletion, interpolation, scaling, malformed files, multi-motor files, CSV |
| `test_dynamics.py` | free fall, vacuum projectile, rocket equation, quadratic drag (analytic), wind via relative velocity, Mach, torque-free precession, quaternion kinematics, weathercock frequency, instability, tail-first instability, TVC torque, rail hold |
| `test_numerics.py` | integrator orders 1/2/4, event root finding, timestep convergence dt 0.1 -> 0.001 (3-DOF) and 0.05 -> 0.001 (6-DOF), step-alignment independence |
| `test_simulation.py` | phase sequence, event order, impact/never-underground, energy-conservation impact speed, rocket-equation flight vs SciPy ODE, mass depletion, Mach logging, determinism (bitwise), no-liftoff, invalid configs, fidelity levels, fast mode, slope terrain, wind effects, launch angle, decimation |
| `test_data.py` | export round trips (4 formats), schema integrity, quality gates, seed derivation/distributions, windows/alignment, batch end-to-end, worker-count independence, crash/resume, rejection logging, split disjointness and hold-out, windowed shards |
| `test_gnc.py` | sensor statistics, bias walk, quantisation, saturation, rate, latency, truth != measurement, TRIAD, gyro integration, launch detector, Kalman filter accuracy, actuator, closed-loop TVC stabilisation of an unstable vehicle |
| `test_validation_cli.py` | metrics, telemetry import, self-consistency of the comparison pipeline, real-flight regression guards, uncertainty formatting, CLI |

`conftest.py` drives the dynamics directly with constructed vehicles for the analytic tests.

Bugs these tests caught during development: thrust returned 0 at the exact curve end points (breaking constant-thrust
analytic cases), a transposed rotation matrix in TRIAD alignment, tail-first flight being aerodynamically stable, GPS
latency ignored by the filter, off-by-one handling of loop variables in the event code.
