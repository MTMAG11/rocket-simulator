# Architecture

## Language

Python (NumPy/SciPy/PyArrow). A flight is 10^3-10^4 steps; dataset throughput comes from running many flights in parallel
processes (about 0.3-0.6 s per 6-DOF flight per core, see [performance.md](performance.md)), and the downstream ML tooling is
Python. If the integrator becomes the bottleneck, `PointMass3DOF`/`RigidBody6DOF.derivative` sit behind the `Dynamics` interface
and could be replaced by a compiled core without changing the rest.

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

### Truth, measurement, estimate, command, actual

Each telemetry column has one role (`rocket_sim.data.schema.role_of`): `truth` (simulator state, environment, mass properties),
`measurement` (`meas_*`), `estimate` (`est_*`), `command` (`tvc_cmd_*`, `fin_cmd_*`, `fin_dcmd_*`) or `actual` (`tvc_*`, `fin_*`: the
actuator state the physics used). The controller receives a `ControlInput` whose `state_source` is `estimate` or `truth`; `truth` is a
development shortcut that is recorded in the run metadata and warned about. A controller that consumes only sensor samples (the HIL
bridge) gets no state. The estimator receives measurements only; a test replays the logged measurements through a fresh filter and
reproduces its estimates bit for bit. The controller's gain schedule uses the as-built mass properties (see
[hil.md](hil.md)).

```
physics (truth) -> sensors -> [measurements] -> estimator -> [estimate] -> controller -> [command] -> (compute + downlink latency)
   ^                                                  (or: [measurements] -> HIL protocol -> flight computer -> [command])      |
   +---------------------- [actual actuator state] <- actuator (delay, lag, rate, angle limits) <-----------------------------+
```

The physics never imports control code: `Simulation.run` owns the loop and passes only a `Controls` object (gimbal angles) to the
dynamics. A controller is any object with `reset(ctx)` and `update(ControlInput) -> Command` (`controller.type: python`).

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

## Design decisions

* One state convention: launch-frame ENU for translation, quaternion body->launch for attitude ([frames.md](frames.md)).
* Everything stochastic derives from one integer seed via `SeedSequence`, with independent streams for wind, each sensor and
  parameter draws; a run is a pure function of (config, seed).
* The physics sees the actual actuator state, never the command.
* Flights carry a split label; holdout results go through a logged protocol keyed to a fingerprint of the physics source
  ([validation.md](validation.md)).
* `PHYSICS_VERSION`, `SCHEMA_VERSION`, `CONFIG_VERSION`, `DATASET_VERSION` and `SIM_VERSION` are stored in every record, manifest and runs table.
* Rail exit, apogee and impact are root-found events; discontinuities fall on step boundaries.
* Invalid config raises `ConfigError` with the key path; a non-finite state raises `SimulationError`; bad runs in a batch are
  rejected and logged, never written to a dataset.
* pandas is not a dependency (PyArrow only).

## Repository layout

```
src/rocket_sim/      simulator package
configs/             example vehicle, batch and dataset configs
vehicles/            example vehicle file and its JSON schema
data/motors/         .eng thrust curves (RASP format, from ThrustCurve.org)
validation_data/     flight registry, real-flight definitions and telemetry, RocketPy cross-checks
validation_results/  recorded validation outputs and the holdout log
tests/  docs/  scripts/  packaging/
```

The EuRoC validation motors in `validation_data/raw/` are derived copies with the casing removed (documented in their headers).
