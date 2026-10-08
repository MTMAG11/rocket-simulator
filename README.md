# Rocket Simulator

6-DOF flight simulator and dataset generator for solid-motor rockets, built to develop guidance, navigation and control (GNC)
for an autonomous rocket. It simulates flight from a physical vehicle description and a motor thrust curve, models sensors,
state estimation and actuators, and generates reproducible Monte Carlo datasets.

## Quick Start

### Option 1: Windows executable

Unzip `RocketSimulator-<version>-win64.zip` (keep the folder intact) and run `RocketSimulator\RocketSimulator.exe`.
No release is published yet; build it with one command ([docs/packaging.md](docs/packaging.md)). The executable is unsigned, so
SmartScreen may ask for confirmation.

### Option 2: Run from source

Requires Python 3.11 and Git.

```powershell
git clone https://github.com/MTMAG11/rocket-simulator.git
cd rocket-simulator
py -3.11 -m venv .venv
.venv\Scripts\activate
pip install -e .
python -m rocket_sim
```

If PowerShell blocks `activate`, run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once, or call
`.venv\Scripts\python.exe -m rocket_sim` directly. Later, double-click `run_simulator.bat`. `python -m rocket_sim --check`
verifies the installation.

In the window: choose a vehicle and motor, set launch elevation and wind, and press Run simulation. Results are saved under
`output/gui_runs/`. The bundled vehicles are placeholders, not measured rockets. Command-line tools: `rocketsim --help`
([docs/usage.md](docs/usage.md)).

## Capabilities

* 6-DOF rigid-body flight with 3-DOF and 1-D levels; motor thrust curves from `.eng` files.
* Vehicles described as components in a versioned `vehicle.json`; mass, CG, inertia tensor, CP and static margin are derived.
* Aerodynamic models (Barrowman, enhanced, tables) behind one `CD/CL/Cm(M, alpha, Re, geometry)` interface, with provenance.
* TVC and control-fin actuator models; sensors, a linear Kalman filter and a controller interface, with truth, measurement,
  estimate, command and actual state kept separate.
* HIL foundation: wire protocol, transports, reference flight computer and timing model.
* Parallel, reproducible Monte Carlo dataset generation, and a validation protocol against real flights.

## Limitations

Validated only against seven 7-24 kg subsonic solid-motor flights, using apogee and altitude; only two are holdouts. Not validated:
small rockets, transonic/supersonic flight, and any vehicle, actuator, sensor, estimator or HIL model against real hardware. The
estimator is a linear Kalman filter, not an EKF. No hardware has been connected to the HIL interface. Numbers and the full list:
[docs/validation.md](docs/validation.md), [docs/error_budget.md](docs/error_budget.md), [docs/status_matrix.md](docs/status_matrix.md).

## Documentation

| | |
|---|---|
| [docs/usage.md](docs/usage.md) | install, configuration, controllers, CLI, GUI |
| [docs/packaging.md](docs/packaging.md) | resource paths, Windows launcher and executable build |
| [docs/architecture.md](docs/architecture.md) | pipeline, packages, design principles |
| [docs/physics.md](docs/physics.md), [docs/aerodynamics.md](docs/aerodynamics.md), [docs/frames.md](docs/frames.md) | models, aerodynamics, conventions |
| [docs/vehicle_format.md](docs/vehicle_format.md), [docs/vehicle.md](docs/vehicle.md) | vehicle file, airframe, actuators |
| [docs/sensors.md](docs/sensors.md), [docs/hil.md](docs/hil.md) | sensors and estimation, HIL |
| [docs/datasets.md](docs/datasets.md) | dataset generation |
| [docs/validation.md](docs/validation.md), [docs/error_budget.md](docs/error_budget.md), [docs/status_matrix.md](docs/status_matrix.md) | validation, error budget, status |
| [docs/config_reference.md](docs/config_reference.md), [docs/schema.md](docs/schema.md) | configuration keys, telemetry schema (generated) |
| [docs/testing.md](docs/testing.md), [docs/performance.md](docs/performance.md) | tests, benchmarks |
