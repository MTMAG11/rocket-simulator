# Rocket Simulator

A physics-based rocket flight simulation and data-generation platform: the foundation for developing guidance,
navigation and control (GNC) for an autonomous **solid-motor** rocket (predetermined thrust curve, TVC / fin control
authority) and for generating large, reproducible datasets for machine learning.

> Status: research-grade tool (V1.2). Against **seven** 7-24 kg solid-motor flights (barometer-equivalent altitude, unmeasured
> wind/temperature, parameters from a secondary source) it predicts apogee to -6.4 ... +13.4 % (RMS 6.2 %). Only **two** of
> those flights are true hold-outs (RMS 5.5 %, n = 2: provisional); three are development flights (in-sample) and two are
> calibration flights. A global drag calibration was tested and **rejected**. It is **not validated** for small model rockets
> (< ~5 kg), supersonic flight, attitude/TVC/fin-control dynamics or sensors on real data. Not certified, not "NASA-level"
> (see [docs/validation.md](docs/validation.md) and [docs/error_budget.md](docs/error_budget.md)).

## What it does (V1.2: physical vehicle + HIL foundation)

* **A real rocket can be entered without magic numbers**: a versioned, human-readable `vehicle.json` of physical components (nose,
  tubes, transitions/boat-tails, fins, avionics, battery, payload, recovery, motor mount, control surfaces) with mass, position,
  dimensions, material and optional CAD inertia tensor. The simulator **derives** total/dry/propellant mass, CG (and its shift during the
  burn), the full 3x3 inertia tensor, CP, static margin and the aerodynamic inputs: `rocketsim vehicle vehicles/example_tvc_demo.json`
  ([docs/vehicle_format.md](docs/vehicle_format.md)). The example vehicle is a **placeholder**, not a measured rocket.
* **Truth / measurement / estimate / command / actual are separated and tested**: every telemetry column has a role; the estimator
  is given measurements only (bit-exact replay test); the controller is fed the estimate unless a truth shortcut is explicitly chosen
  (recorded and warned about); aerodynamic **provenance** is recorded per quantity and per dataset row.
* **HIL foundation**: protocol v2 (every timestamped sensor sample in, commands out), loopback / child-process / record / replay
  transports, a lock-step headless loop, a Python reference flight computer, and an explicit timing model with compute/uplink/downlink
  latency ([docs/hil.md](docs/hil.md)). **No physical hardware has been connected.**
* What is implemented vs verified vs validated: [docs/status_matrix.md](docs/status_matrix.md).

## What it does (V1.1 and earlier)

* 6-DOF rigid-body flight (quaternion attitude, RK4 with event location), plus faster 3-DOF/1-D levels and a fast mode
* Real motor thrust curves (`.eng`), mass depletion, mass/inertia/CG/CP history, static margin
* ISA atmosphere (or measured/simplified), `g(h)`, wind (constant/profile/power-law/turbulence/gusts) via *relative* air velocity
* **Aerodynamic model hierarchy** (simplified / Barrowman / enhanced / lookup / 2-D lookup-for-CFD) behind one
  `CD/CL/Cm(M, alpha, Re, geometry)` interface ([docs/aerodynamics.md](docs/aerodynamics.md))
* **Component-based airframe** (nose, tubes, transitions, boat-tails, fins, motor section, payload): CG and the full inertia
  tensor computed from component masses, CP from geometry, static margin logged ([docs/vehicle.md](docs/vehicle.md))
* TVC **and aerodynamic control surfaces** (fins/canards with deflection, rate, lag, delay; a mixer; roll control) driven by a
  reusable actuator model - the physics always sees the *actual* actuator state
* Simulated sensors (accelerometer, gyro, barometer, GPS with dropout/start-up delay, magnetometer; misalignment, async
  rates, latency), an `Estimator` interface (truth / linear Kalman filter; **no EKF yet**, design in [docs/sensors.md](docs/sensors.md))
* Flight phases, ground/terrain impact, parachutes (drogue + main)
* Monte Carlo + parallel, checkpointed, reproducible dataset generation (Parquet/NPZ), ML windows, quality gates,
  **domain randomisation with explicit, correlated and linked distributions**, causal (zero-order-hold) inputs,
  near-duplicate/temporal leakage tests, statistics reports, versioned experiments, 1/100/1000/10000-run benchmarks
* Real-flight validation: flight registry with source tiers and a DEVELOPMENT / CALIBRATION / HOLDOUT protocol (logged,
  fingerprinted), input-uncertainty Monte Carlo, calibration records, sensitivity study and error budget; CLI; PySide6 GUI

## Quick start

```bash
pip install -e ".[gui,dev]"
rocketsim simulate configs/example_g80.yaml --plot flight.png
rocketsim batch configs/batch_example.yaml --runs 100
rocketsim generate-dataset configs/dataset_example.yaml --runs 100
rocketsim vehicle vehicles/example_tvc_demo.json        # derived mass properties / CG / inertia / CP
rocketsim timing configs/example_tvc_closed_loop.yaml    # every rate and latency
rocketsim hil configs/example_tvc_closed_loop.yaml --log loop.jsonl   # headless HIL run
rocketsim validate-registry --split development --out validation_results/registry_dev
rocketsim experiment configs/experiment_example.yaml
rocketsim gui
pytest
```

```text
Apogee (AGL)               918 +/- 70* m        (rounded to an uncertainty extrapolated from the 20 kg-class
Max velocity               198 +/- 16 m/s       validation; the 29 mm example vehicle itself is NOT validated)
Static margin at launch    3.1 cal
```

## Documentation

| | |
|---|---|
| [docs/architecture.md](docs/architecture.md) | language decision, pipeline, packages, design principles |
| [docs/physics.md](docs/physics.md) | every model: equations, assumptions, sources, limitations, convergence, change log |
| [docs/aerodynamics.md](docs/aerodynamics.md) | aerodynamic model hierarchy, coefficient interface, accuracy by regime |
| [docs/vehicle_format.md](docs/vehicle_format.md) | the vehicle file: conventions, schema, derived quantities, CAD/OpenRocket mapping |
| [docs/vehicle.md](docs/vehicle.md) | component airframe, CG/inertia, actuator chain, control surfaces, TVC audit |
| [docs/hil.md](docs/hil.md) | HIL protocol, transports, timing model, what is and is not implemented |
| [docs/status_matrix.md](docs/status_matrix.md) | implemented / analytic / numerical / cross-checked / validated, per capability |
| [docs/sensors.md](docs/sensors.md) | sensor audit, estimator interface, EKF roadmap |
| [docs/frames.md](docs/frames.md) | frames, axes, quaternions, angle and wind conventions |
| [docs/usage.md](docs/usage.md) | install, configuration, controllers, CLI, GUI |
| [docs/config_reference.md](docs/config_reference.md) | every configuration key (generated) |
| [docs/datasets.md](docs/datasets.md) | batch/dataset generation, schema, reproducibility, ML use |
| [docs/schema.md](docs/schema.md) | telemetry schema v1.2.0 (generated) |
| [docs/validation.md](docs/validation.md) | flight registry, splits and tiers, results, holdout log, calibration record, cross-checks, limits |
| [docs/error_budget.md](docs/error_budget.md) | sensitivity study, model-form study, physics error budget |
| [docs/testing.md](docs/testing.md) | test strategy |
| [docs/performance.md](docs/performance.md) | benchmarks |
| [docs/review/](docs/review/) | V1.1 audit and independent critic reviews with the fixes they triggered |

## Repository layout

```
src/rocket_sim/   config  environment  motor  vehicle  physics  simulation  sensors  estimation  control  data  validation  ui
configs/          example vehicle, batch and dataset specs
data/motors/      .eng thrust curves
validation_data/  flight registry, real-flight definitions and raw telemetry, calibration records, RocketPy cross-checks
validation_results/  recorded validation outputs, holdout log, input-uncertainty MC, sensitivity/error-budget data
experiments/      versioned dataset experiments (created on demand)
tests/  docs/  scripts/
```

## Known limitations (summary)

Drag level uncertain to ~7 % (see the error budget); unmeasured inputs alone move apogee 3-7 % (1 sigma); no transonic or
supersonic flight validation (the enhanced fin model is verified analytically only, and one holdout flight, Erebus 11, has an
unexplained burnout-speed gap); no validation below ~5 kg; **no vehicle, actuator, sensor, estimator or HIL model has been validated against real hardware or a weighed vehicle**; no serial/UDP transport or real-time pacing; flat non-rotating Earth; no fin flutter, hinge moments, rail
tip-off; control surfaces, TVC, sensors and the Kalman filter verified in simulation only (no real flight data exist for
them in the registry); linear Kalman filter, not an EKF; sensor defaults are generic; amateur-grade ground truth (Tier 2/3,
no Tier 1). Full list in [docs/validation.md](docs/validation.md), [docs/error_budget.md](docs/error_budget.md) and
[docs/physics.md](docs/physics.md).

## Motor data

Thrust curves are RASP `.eng` files from ThrustCurve.org; keeping them separate from the physics makes motor swaps
trivial. The validation motors are in `validation_data/raw/` (three original `.eng` files; for the EuRoC flights, derived copies with the casing removed, documented in their header, and one curve converted from a RocketPy CSV).

## About

Part of an Autonomous Rocket Project: built from the ground up to learn the physics, mathematics and engineering behind
autonomous rocket flight, now structured so the same engine can drive large-scale simulation for control and learning.
