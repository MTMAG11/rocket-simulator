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

### Truth, measurement, estimate, command, actual (V1.2)

Every column in the telemetry has exactly one **role** (`rocket_sim.data.schema.role_of`): `truth` (the simulator's state,
environment and mass properties), `measurement` (`meas_*`), `estimate` (`est_*`), `command` (`tvc_cmd_*`, `fin_cmd_*`, `fin_dcmd_*`) or
`actual` (`tvc_*`, `fin_*`: the physical actuator state the physics used). The controller is handed a `ControlInput` whose
`state_source` is `estimate` (the flight-computer state) or `truth` (an explicit development shortcut that is **recorded in the run
metadata and warned about**); a controller that consumes only sensor samples (the HIL bridge) is never handed a state. The estimator is
given measurements only (a test replays the logged measurements through a fresh filter and reproduces its estimates bit-for-bit).
Known leak that remains by design: the controller's authority/gain schedule is derived from the as-built mass properties (a flight
computer is assumed to know its as-designed vehicle).

```
physics (truth) -> sensors -> [measurements] -> estimator -> [estimate] -> controller -> [command] -> (compute + downlink latency)
   ^                                                  (or: [measurements] -> HIL protocol -> flight computer -> [command])      |
   +---------------------- [actual actuator state] <- actuator (delay, lag, rate, angle limits) <-----------------------------+
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
| `vehicle` | geometry, component `Assembly` (sections, control surfaces), aerodynamic model hierarchy (simplified / Barrowman / enhanced / table / table2d), mass model with inertia tensor, `Vehicle` aggregate |
| `physics` | quaternion/frame math, integrators and event refinement, 3-DOF and 6-DOF dynamics |
| `simulation` | builders, fidelity resolution, phase state machine, recorder, the simulation loop |
| `sensors` | accelerometer, gyro, barometer, GPS, magnetometer channels |
| `estimation` | `Estimator` interface; truth estimator, launch detector, TRIAD alignment, linear navigation Kalman filter with pad gyro-bias estimation |
| `control` | generic `ActuatorBank` (TVC and control fins), `FinMixer`, controller interface and built-in controllers |
| `data` | telemetry schema, export (CSV/JSON/NPZ/Parquet), Monte Carlo (copula, linked parameters), quality gates, leakage checks and statistics, dataset/windowing, parallel batch engine, dataset browser |
| `validation` | standard telemetry, real-flight import, metrics, comparison, calibration, flight registry + holdout log, input-uncertainty Monte Carlo, sensitivity/error budget |
| `ui` | PySide6 GUI (thin client of the engine) |
| `hil` | HIL protocol v2, transports (loopback, pipe, record/replay), the simulator-side bridge, flight-computer contract and a Python reference flight computer ([hil.md](hil.md)) |
| `vehicle/vehicle_file.py`, `vehicle/report.py` | the versioned vehicle file -> `rocket` config compiler and the derived mass-properties report ([vehicle_format.md](vehicle_format.md)) |
| `simulation/timing.py` | explicit timing report (rates, latencies, derived end-to-end latency) |
| `experiments.py` | versioned dataset experiments (provenance record, reproduction check) |
| `cli.py`, `benchmark.py`, `plotting.py`, `reporting.py`, `uncertainty.py` | front ends and helpers |

## Key design decisions

* **One state, one convention.** Launch-frame ENU for translation, quaternion body->launch for attitude (docs/frames.md).
* **Truth vs. measurement are different columns** (`pos_*` vs `meas_*`/`est_*`); the estimator only sees sensor readings.
* **Everything stochastic derives from one integer seed** via `SeedSequence`, with independent streams for wind, each
  sensor, and parameter draws; a run is a pure function of (config, seed).
* **The physics sees the actual actuator state**, never the command.
* **Honest validation by construction**: flights carry a split label; holdout results go through a logged protocol keyed to a
  fingerprint of the physics source ([validation.md](validation.md)).
* **Versioned artefacts**: `PHYSICS_VERSION`, `SCHEMA_VERSION`, `CONFIG_VERSION`, `DATASET_VERSION`, `SIM_VERSION` stored in every record,
  manifest and runs table.
* **Events, not output-grid snapping**: rail exit/apogee/impact are root-found; discontinuities are step boundaries.
* **Fail loudly**: invalid config -> `ConfigError` with the key path; NaN/diverged state -> `SimulationError`; bad runs in
  a batch are *rejected and logged*, never silently written to a dataset.
* **No pandas dependency on the critical path** (PyArrow only) -- this also made the project run on a machine whose
  application-control policy blocks pandas' compiled extension.
