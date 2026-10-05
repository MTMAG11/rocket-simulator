# Architecture

## Language decision

Python (NumPy/SciPy/PyArrow) was **kept**, after inspecting the original project (a ~1500-line 1-D/2-D Euler point mass
plus a PySide6 GUI). Reasons, from measurements rather than taste:

* A flight is 10^3-10^4 steps; throughput comes from running *many flights in parallel processes*, which Python does
  well (`ProcessPoolExecutor`, no GIL contention across processes). Measured: ~0.3-0.6 s per 6-DOF flight on one core,
  see [performance.md](performance.md).
* The ML ecosystem the data feeds (NumPy, PyTorch, Parquet) is Python; no FFI boundary to maintain.
* Rewrite cost in C++/Rust would be large and the numerical core is not the bottleneck for the current requirement.
  If dataset throughput becomes the bottleneck the path is (in order): vectorised batch integration across runs in
  NumPy, then a compiled core for `PointMass3DOF`/`RigidBody6DOF.derivative`. Both are isolated behind the `Dynamics`
  interface, so the rest of the system would not change.

## What was kept / dropped from the original

Kept: the `.eng` motor files and the "separate motor data from physics" idea, the dark-theme GUI stylesheet. Dropped:
rocketpy dependency (only used to parse `.eng`; replaced by a 100-line parser), the state dataclass, per-step lists, the
grams/kg mix, the `O(n^2)` propellant lookup, the missing ground, the hard-coded 20 s stop. Bugs found in the original:
no ground contact (rocket flew underground), apogee detected at the first `vy <= 0`, thrust angle never followed the
vehicle, mass in grams mixed with kg.

## Pipeline

```
CONFIG (YAML/JSON/TOML) --validate--> SimConfig --resolve_fidelity--> builders
   VEHICLE (geometry, aero, mass model, parachutes)   ENVIRONMENT (atmosphere, gravity, wind, terrain)   MOTOR
                     \                                   |                                    /
                      +--------------------------> DYNAMICS (3-DOF / 6-DOF, RK4) <-----------+
                                                         | TRUE STATE
                                                  SENSORS (noise, bias, drift, quantisation, latency)
                                                         | MEASUREMENTS
                                                  ESTIMATOR (launch detector, TRIAD, Kalman filter)
                                                         | ESTIMATED STATE
                                                  CONTROLLER (pluggable)  -> command
                                                         |
                                                  ACTUATOR (delay, lag, rate/angle limits) -> gimbal angle --> DYNAMICS
```

The physics never imports control code: `Simulation.run` owns the loop and passes only a `Controls` object (gimbal
angles) into the dynamics. A controller is any object with `reset(ctx)` and `update(ControlInput) -> Command`
(`rocketsim` config `controller.type: python`), so a neural network or a hardware-in-the-loop serial bridge
(real flight computer receives `SensorReadings`, returns `Command`) plugs in without touching the physics.

## Packages (`src/rocket_sim`)

| package | responsibility |
|---|---|
| `config` | typed dataclass schema, strict parsing/validation with dotted-path errors, YAML/JSON/TOML loaders, hashing |
| `environment` | atmosphere (ISA, exponential, table), gravity, wind (+turbulence, gusts), terrain |
| `motor` | motor model, `.eng`/CSV loaders with a registry for new formats |
| `vehicle` | geometry, Barrowman/build-up/table aerodynamics, mass model, `Vehicle` aggregate |
| `physics` | quaternion/frame math, integrators and event refinement, 3-DOF and 6-DOF dynamics |
| `simulation` | builders, fidelity resolution, phase state machine, recorder, the simulation loop |
| `sensors` | accelerometer, gyro, barometer, GPS, magnetometer channels |
| `estimation` | launch detector, TRIAD alignment, linear navigation Kalman filter |
| `control` | TVC actuator, controller interface and built-in controllers |
| `data` | telemetry schema, export (CSV/JSON/NPZ/Parquet), Monte Carlo, quality gates, dataset/windowing, parallel batch engine, dataset browser |
| `validation` | standard telemetry, real-flight import, metrics, comparison, leave-one-out calibration |
| `ui` | PySide6 GUI (thin client of the engine) |
| `cli.py`, `benchmark.py`, `plotting.py`, `reporting.py`, `uncertainty.py` | front ends and helpers |

## Key design decisions

* **One state, one convention.** Launch-frame ENU for translation, quaternion body->launch for attitude (docs/frames.md).
* **Truth vs. measurement are different columns** (`pos_*` vs `meas_*`/`est_*`); the estimator only sees sensor readings.
* **Everything stochastic derives from one integer seed** via `SeedSequence`, with independent streams for wind, each
  sensor, and parameter draws; a run is a pure function of (config, seed).
* **Versioned artefacts**: `PHYSICS_VERSION`, `SCHEMA_VERSION`, `CONFIG_VERSION`, `SIM_VERSION` stored in every record,
  manifest and runs table.
* **Events, not output-grid snapping**: rail exit/apogee/impact are root-found; discontinuities are step boundaries.
* **Fail loudly**: invalid config -> `ConfigError` with the key path; NaN/diverged state -> `SimulationError`; bad runs in
  a batch are *rejected and logged*, never silently written to a dataset.
* **No pandas dependency on the critical path** (PyArrow only) -- this also made the project run on a machine whose
  application-control policy blocks pandas' compiled extension.
