# Usage: installation, configuration, CLI

## Install

Use an **editable/source install** (`pip install -e .`): data files (`data/motors`, `experiments/`, benchmark output) and the registry
are located relative to the source tree; a wheel install does not ship them. A clean install needs network access for the five
dependencies (`pip install -e . --no-deps` works offline if they are already present).

```bash
python -m venv .venv && .venv/Scripts/activate        # Windows (use bin/activate elsewhere)
pip install -e ".[gui,dev]"                           # numpy, scipy, pyarrow, pyyaml, matplotlib (+ PySide6, pytest, ruff, mypy)
rocketsim check configs/example_g80.yaml
```

Python >= 3.11. pandas is *not* required.

## Configuration files

YAML (also JSON/TOML). Strict: unknown keys, wrong types, negative masses, impossible geometry, bad timesteps, missing
motor files or malformed `.eng` files give an error naming the key (`rocket.dry_mass_kg: must be > 0 (got -1.0)`).
Units are in the key names. Every key with its type and default: [config_reference.md](config_reference.md). Minimal example:

```yaml
config_version: 1
fidelity: 3
simulation: {dt_s: 0.01, descent_dt_s: 0.05, integrator: rk4}
rocket:
  body_diameter_m: 0.041
  body_length_m: 1.0
  nose: {shape: ogive, length_m: 0.17}
  fins: {count: 4, root_chord_m: 0.10, tip_chord_m: 0.045, span_m: 0.07, sweep_m: 0.06, thickness_m: 0.002, position_from_nose_m: 0.82}
  dry_mass_kg: 0.45            # airframe WITHOUT the motor
  cg_from_nose_m: 0.60
  parachutes: [{cd: 1.5, diameter_m: 0.6, trigger: apogee}]
motor: {file: data/motors/AeroTech_G80T.eng}
environment: {wind: {model: constant, speed_ms: 3, direction_from_deg: 270}}
launch: {elevation_deg: 90, rail_length_m: 1.5}
```

Other models are selected by name: `environment.atmosphere.model: isa | exponential | table`, `wind.model: none |
constant | profile | power_law` (+ `turbulence_sigma_ms`, `gusts`), `rocket.aero.model: simplified | barrowman | enhanced | table | table2d | constant` ([aerodynamics.md](aerodynamics.md)),
`controller.type: none | tvc_attitude | schedule | python`, `estimator.type: none | nav_kf | truth`. Relative file paths resolve
against the config's directory, then the project root. Motor formats: `.eng` (RASP) and `.csv`; register others with
`rocket_sim.motor.register_loader`.

A vehicle with sensors and an estimator (fidelity 4-5) needs pad time for alignment: set `motor.ignition_delay_s >=
estimator.alignment_time_s` (validated), and a high-g accelerometer (`sensors.accelerometer.saturation`) if the motor
exceeds ~16 g (the default part clips there, as a real 16 g part would).

## Controllers

```python
from rocket_sim.control import Command

class MyController:
    def reset(self, ctx): ...                        # ctx['authority'](t), ctx['launch_axis']
    def update(self, inp):                           # inp: ControlInput (estimated state, phase, time since launch)
        return Command(tvc_y=0.0, tvc_z=0.0)         # radians
```

`controller: {type: python, params: {class: "mypkg.mod:MyController", kwargs: {...}}}`, or pass an instance to
`Simulation(cfg, controller=...)`. `controller.use_truth: true` feeds the *true* state (testing only).

## CLI (`rocketsim <command>`, also `python -m rocket_sim.cli`)

| command | purpose |
|---|---|
| `simulate CONFIG [--seed N] [--fidelity L] [--dt S] [--set key=value ...] [--out DIR --format csv,json,npz,parquet] [--plot PNG]` | one flight, summary with uncertainties |
| `batch SPEC [--runs N] [--workers W] [--out DIR] [--no-resume]` | Monte Carlo batch (parallel, checkpointed) |
| `generate-dataset SPEC` | same engine; ML dataset kinds |
| `validate FLIGHT.yaml [--out DIR]` | compare to real telemetry (metrics + plot + report) |
| `export SOURCE [--run ID] --format ...` | convert a saved record or reproduce a dataset run |
| `inspect DATASET [--run ID]` | browse a dataset without loading telemetry |
| `benchmark [--quick] [--scales 1,100,1000,10000]` | throughput measurements and scaling ladder |
| `validate-registry [--split development\|calibration\|holdout\|all] [--confirm-frozen] [--set k=v] [--calibration-id ID] [--migrate-fingerprint]` | flight registry with the holdout protocol ([validation.md](validation.md)) |
| `experiment FILE.yaml [--verify DIR] [--list]` | versioned, reproducible dataset experiments |
| `check CONFIG` | validate a config |
| `schema [--json]`, `motors`, `gui` | schema, motor list, GUI |

Exit codes: 0 ok, 1 error (message on stderr), 2 flight did not end normally, 130 interrupted (batch progress is kept).

Summaries print numbers rounded to the model's uncertainty (`801 -> 800 +/- 50 m`); a trailing `*` marks provisional
uncertainties (`rocket_sim/uncertainty.py`).

## GUI

`rocketsim gui`: configure motor, mass, launch, wind, temperature, fidelity, timestep and integrator; run in a background
thread; select any number of schema variables for stacked, zoomable plots (matplotlib toolbar) with event markers; scrub
the timeline (readout + 3-D rocket orientation/trajectory/ground/wind arrow, coloured by flight phase); Summary tab;
Data browser tab (open a dataset directory, select a run, reproduce and plot it). Large generation is headless only.


## Component-based vehicles, control surfaces (V1.1)

```yaml
rocket:
  sections:                                   # nose tip aft; masses give the CG and the inertia tensor
    - {type: nose, shape: ogive, length_m: 0.20, diameter_m: 0.054, mass_kg: 0.060}
    - {type: body, length_m: 0.30, diameter_m: 0.054, mass_kg: 0.110}
    - {type: transition, length_m: 0.08, aft_diameter_m: 0.041, mass_kg: 0.025}
    - {type: body, length_m: 0.50, diameter_m: 0.041, mass_kg: 0.120}
    - {type: boattail, length_m: 0.04, aft_diameter_m: 0.033, mass_kg: 0.010}
  masses: [{name: avionics, mass_kg: 0.08, position_from_nose_m: 0.40}]
  control_surfaces: {count: 4, root_chord_m: 0.10, tip_chord_m: 0.06, span_m: 0.09, position_from_nose_m: 0.88,
                     max_deflection_deg: 20, max_rate_deg_s: 400, time_constant_s: 0.01}
controller: {type: tvc_attitude, use_truth: true, params: {actuation: fins, roll_damping: 2.0}}   # tvc | fins | both
estimator: {type: truth}                      # truth | nav_kf | none
```

See [vehicle.md](vehicle.md), `configs/example_components.yaml` and `configs/domain_randomization.yaml`. A custom controller's
`Command` has `tvc_y, tvc_z, fin_pitch, fin_yaw, fin_roll` (radians). Overrides are *dotted paths*
(`rocket.inertia: {...}`); a nested mapping under a section name replaces the whole section.
