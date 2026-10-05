# Rocket Simulator

A physics-based rocket flight simulation and data-generation platform: the foundation for developing guidance,
navigation and control (GNC) for an autonomous **solid-motor** rocket (predetermined thrust curve, TVC / fin control
authority) and for generating large, reproducible datasets for machine learning.

> Status: research-grade tool. Compared with three ~20 kg-class university flights (barometer-equivalent altitude) it
> predicts apogee to +0.5 / +13 / -5 % (RMS 8 %) from catalogue inputs with unknown atmosphere and impulse; the model
> change that fixed the worst case was made after seeing it (in-sample). It is **not validated** for small model rockets,
> supersonic flight, attitude/TVC dynamics or sensors. Not certified, not "NASA-level" (see [docs/validation.md](docs/validation.md)).

## What it does

* 6-DOF rigid-body flight (quaternion attitude, RK4 with event location), plus faster 3-DOF/1-D levels and a fast mode
* Real motor thrust curves (`.eng`), mass depletion, mass/inertia/CG/CP history, static margin
* ISA atmosphere (or measured/simplified), `g(h)`, wind (constant/profile/power-law/turbulence/gusts) via *relative* air velocity
* Barrowman normal force + component drag build-up (Re, Mach, roughness), aerodynamic moments and damping
* TVC with actuator limits (angle/rate/lag/delay), pluggable controllers, simulated sensors (accelerometer, gyro,
  barometer, GPS, magnetometer), launch detection and a latency-compensated Kalman navigation filter
* Flight phases, ground/terrain impact, parachutes (drogue + main)
* Monte Carlo + parallel, checkpointed, reproducible dataset generation (Parquet/NPZ), ML windows, quality gates,
  train/val/test without seed leakage, versioned schema and manifests
* Real-flight validation tooling with quantitative metrics; CLI; PySide6 GUI

## Quick start

```bash
pip install -e ".[gui,dev]"
rocketsim simulate configs/example_g80.yaml --plot flight.png
rocketsim batch configs/batch_example.yaml --runs 100
rocketsim generate-dataset configs/dataset_example.yaml --runs 100
rocketsim validate validation_data/prometheus.yaml
rocketsim gui
pytest
```

```text
Apogee (AGL)               919 +/- 70 m        (rounded to an uncertainty extrapolated from the 20 kg-class
Max velocity               198 +/- 16 m/s       validation; the 29 mm example vehicle itself is NOT validated)
Static margin at launch    3.1 cal
```

## Documentation

| | |
|---|---|
| [docs/architecture.md](docs/architecture.md) | language decision, pipeline, packages, design principles |
| [docs/physics.md](docs/physics.md) | every model: equations, assumptions, sources, limitations, convergence, change log |
| [docs/frames.md](docs/frames.md) | frames, axes, quaternions, angle and wind conventions |
| [docs/usage.md](docs/usage.md) | install, configuration, controllers, CLI, GUI |
| [docs/config_reference.md](docs/config_reference.md) | every configuration key (generated) |
| [docs/datasets.md](docs/datasets.md) | batch/dataset generation, schema, reproducibility, ML use |
| [docs/schema.md](docs/schema.md) | telemetry schema v1.0.0 (generated) |
| [docs/validation.md](docs/validation.md) | real-flight validation, cross-check, calibration, limits |
| [docs/testing.md](docs/testing.md) | test strategy |
| [docs/performance.md](docs/performance.md) | benchmarks |
| [docs/review/](docs/review/) | independent critic reviews and the fixes they triggered |

## Repository layout

```
src/rocket_sim/   config  environment  motor  vehicle  physics  simulation  sensors  estimation  control  data  validation  ui
configs/          example vehicle, batch and dataset specs
data/motors/      .eng thrust curves
validation_data/  real-flight definitions, raw telemetry, RocketPy cross-check
validation_results/  recorded validation outputs by physics version
tests/  docs/  scripts/
```

## Known limitations (summary)

Single-diameter airframe (no boat-tails/transitions); drag good to ~10 % subsonic and worse above Mach 0.9 (no supersonic
validation); flat non-rotating Earth; no aerodynamic control surfaces yet; linear Kalman filter, not an EKF; sensor
defaults are generic; no real TVC or attitude flight data was available for validation; amateur-grade ground truth.
Full list in [docs/validation.md](docs/validation.md) and [docs/physics.md](docs/physics.md).

## Motor data

Thrust curves are RASP `.eng` files from ThrustCurve.org; keeping them separate from the physics makes motor swaps
trivial. The three validation motors (K828FJ, L1395, M1520) are in `validation_data/raw/`.

## About

Part of an Autonomous Rocket Project: built from the ground up to learn the physics, mathematics and engineering behind
autonomous rocket flight, now structured so the same engine can drive large-scale simulation for control and learning.
