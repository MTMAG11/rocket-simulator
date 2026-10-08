# Rocket Simulator

Flight simulator and dataset generator for solid-motor rockets, used to develop guidance, navigation and control (GNC)
for an autonomous rocket. It simulates 6-DOF flight from a physical vehicle description and a motor thrust curve, models
sensors, state estimation and actuators, and generates reproducible Monte Carlo datasets.

## Quick Start

### Option 1: Windows executable

1. Unzip `RocketSimulator-<version>-win64.zip`. Keep the folder intact.
2. Run `RocketSimulator\RocketSimulator.exe`.

No release is published yet; build it with one command (see [docs/packaging.md](docs/packaging.md)). The executable is
unsigned, so SmartScreen may ask for confirmation.

### Option 2: Run from source

Requires Python 3.11 (the version used for development and testing) and Git.

```powershell
git clone https://github.com/MTMAG11/rocket-simulator.git
cd rocket-simulator
py -3.11 -m venv .venv
.venv\Scripts\activate
pip install -e .
python -m rocket_sim
```

If PowerShell blocks `activate`, run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once, or call the environment's
Python directly (`.venv\Scripts\python.exe -m rocket_sim`). `pip install` downloads about 500 MB, mostly Qt.

On later runs, double-click `run_simulator.bat`, or activate the environment and run `python -m rocket_sim`. The command works
from any directory. `python -m rocket_sim --check` verifies the data files and the GUI library.

### Using the GUI

Choose a vehicle and motor, set launch elevation and wind, and press Run simulation. Fidelity, timestep, seed and other
settings are under Advanced settings. Each run is saved (telemetry CSV, summary, overview plot) under `output/gui_runs/`
(source) or `Documents\RocketSimulator\output\gui_runs\` (executable). The bundled vehicles are placeholders, not measured rockets.

## Capabilities

* 6-DOF rigid-body flight (quaternion attitude, RK4 with event location), with 3-DOF and 1-D levels for fast runs.
* Motor thrust curves from `.eng` files, with mass, CG and inertia-tensor history and static margin.
* Vehicle described as components in a versioned `vehicle.json`; mass, CG, inertia tensor, CP and static margin are derived
  ([docs/vehicle_format.md](docs/vehicle_format.md)). `rocketsim vehicle FILE` prints the report.
* Aerodynamics behind one `CD/CL/Cm(M, alpha, Re, geometry)` interface: simplified, Barrowman, enhanced, table and 2-D table
  models, with per-quantity provenance ([docs/aerodynamics.md](docs/aerodynamics.md)).
* TVC and aerodynamic control surfaces with deflection, rate, lag and delay limits; the physics uses the actual actuator state.
* Sensors (accelerometer, gyro, barometer, GPS, magnetometer), a linear Kalman filter, and a controller interface. Truth,
  measurement, estimate, command and actual state are separate columns in the telemetry ([docs/sensors.md](docs/sensors.md)).
* HIL foundation: a wire protocol, loopback / child-process / record / replay transports, a reference flight computer and an
  explicit timing model ([docs/hil.md](docs/hil.md)). No hardware has been connected.
* Atmosphere, wind (constant, profile, turbulence, gusts), terrain and parachutes.
* Parallel, checkpointed Monte Carlo dataset generation (Parquet/NPZ) with quality gates, correlated parameters and leakage checks.
* Validation against real flights with a development / calibration / holdout protocol and a logged holdout evaluation
  ([docs/validation.md](docs/validation.md)).

What is implemented, analytically checked, cross-checked and validated, per capability: [docs/status_matrix.md](docs/status_matrix.md).

## Command line

`rocketsim <command>` or `python -m rocket_sim <command>`. `pip install -e ".[dev]"` adds the test tools.

```bash
rocketsim simulate configs/example_g80.yaml --plot flight.png
rocketsim batch configs/batch_example.yaml --runs 100
rocketsim generate-dataset configs/dataset_example.yaml --runs 100
rocketsim vehicle vehicles/example_tvc_demo.json
rocketsim timing configs/example_tvc_closed_loop.yaml
rocketsim hil configs/example_tvc_closed_loop.yaml --log loop.jsonl
rocketsim validate-registry --split development --out validation_results/registry_dev
rocketsim experiment configs/experiment_example.yaml
pytest
```

Example `simulate` summary:

```text
Apogee (AGL)               918 +/- 70* m
Max velocity               198 +/- 16 m/s
Static margin at launch    3.1 cal
```

`*` marks an uncertainty extrapolated from the validation flights; the 29 mm example vehicle itself is not validated.

## Limitations

Validated against seven 7-24 kg solid-motor flights, apogee error is -6.4 % to +13.4 % (RMS 6.2 %); only two of those are
holdouts (n = 2). Not validated: small rockets (< ~5 kg), transonic/supersonic flight, and any vehicle, actuator, sensor,
estimator or HIL model against real hardware or a weighed vehicle. Further limits:

* Drag level uncertain to about 7 %; unmeasured inputs alone move apogee 3-7 % (1 sigma).
* Flat, non-rotating Earth; no fin flutter, hinge moments or rail tip-off.
* Linear Kalman filter, not an EKF; default sensor parameters are generic.
* No serial/UDP transport or real-time pacing for HIL.
* Ground truth is amateur-grade (tiers 2/3).

Full lists: [docs/validation.md](docs/validation.md), [docs/error_budget.md](docs/error_budget.md), [docs/physics.md](docs/physics.md).

## Documentation

| | |
|---|---|
| [docs/usage.md](docs/usage.md) | install, configuration, controllers, CLI, GUI |
| [docs/packaging.md](docs/packaging.md) | resource paths, Windows launcher and executable build |
| [docs/architecture.md](docs/architecture.md) | pipeline, packages, design principles |
| [docs/physics.md](docs/physics.md) | models, assumptions, convergence, change log |
| [docs/aerodynamics.md](docs/aerodynamics.md) | aerodynamic models, coefficient interface, accuracy |
| [docs/frames.md](docs/frames.md) | frames, axes, quaternions, wind conventions |
| [docs/vehicle_format.md](docs/vehicle_format.md) | vehicle file format, derived quantities, CAD/OpenRocket mapping |
| [docs/vehicle.md](docs/vehicle.md) | airframe, CG/inertia, actuator chain, control surfaces |
| [docs/sensors.md](docs/sensors.md) | sensor models, estimator interface |
| [docs/hil.md](docs/hil.md) | HIL protocol, transports, timing |
| [docs/datasets.md](docs/datasets.md) | batch and dataset generation, reproducibility |
| [docs/validation.md](docs/validation.md) | flight registry, splits, results, holdout log |
| [docs/error_budget.md](docs/error_budget.md) | sensitivity study and error budget |
| [docs/status_matrix.md](docs/status_matrix.md) | implementation and validation status |
| [docs/config_reference.md](docs/config_reference.md), [docs/schema.md](docs/schema.md) | configuration keys and telemetry schema (generated) |
| [docs/testing.md](docs/testing.md), [docs/performance.md](docs/performance.md) | tests and benchmarks |

## Repository layout

```
src/rocket_sim/      simulator package
configs/             example vehicle, batch and dataset configs
vehicles/            example vehicle file and its JSON schema
data/motors/         .eng thrust curves
validation_data/     flight registry, real-flight definitions and telemetry, RocketPy cross-checks
validation_results/  recorded validation outputs and the holdout log
tests/  docs/  scripts/  packaging/
```

Motor thrust curves are RASP `.eng` files from ThrustCurve.org. The validation motors are in `validation_data/raw/`; the EuRoC
curves are derived copies with the casing removed (documented in their headers).
