# Usage: installation, configuration, CLI

## Install

Start with the README's **Quick Start** (Windows executable, or `py -3.11 -m venv .venv`, `.venv\Scripts\activate`, `pip install -e .`,
`python -m rocket_sim`). This page is the reference for what comes after.

Use an **editable/source install** (`pip install -e .`): the default data (`data/motors`, `configs`, `vehicles`) is found by
`rocket_sim.resources` relative to the source tree, which a wheel install does not ship ([packaging.md](packaging.md) lists the
resolution order and the `ROCKETSIM_HOME` override). A clean install needs network access for the dependencies
(`pip install -e . --no-deps` works offline if they are already present).

```bash
python -m venv .venv && .venv/Scripts/activate        # Windows (use bin/activate elsewhere)
pip install -e .                                      # numpy, scipy, pyarrow, pyyaml, matplotlib, PySide6 (the GUI)
pip install -e ".[dev]"                               # + pytest, ruff, mypy, psutil
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
| `vehicle FILE [--json]` | derive and print mass properties, CG, inertia tensor, CP, static margin of a vehicle file or config ([vehicle_format.md](vehicle_format.md)) |
| `timing CONFIG [--json]` | every rate, period and latency of the loop ([hil.md](hil.md)) |
| `hil CONFIG [--replay LOG] [--command PROG ...] [--log LOG] [--uplink S]` | headless HIL run through the flight-computer protocol |
| `schema [--json]`, `motors`, `gui` | schema, motor list, GUI |

Exit codes: 0 ok, 1 error (message on stderr), 2 flight did not end normally, 130 interrupted (batch progress is kept).

Summaries print numbers rounded to the model's uncertainty (`801 -> 800 +/- 50 m`); a trailing `*` marks provisional
uncertainties (`rocket_sim/uncertainty.py`).

## GUI

`python -m rocket_sim` (same as `rocketsim gui`, `run_simulator.bat` or `RocketSimulator.exe`): the left panel is the run flow, **1 Vehicle**
(bundled configs / vehicle files, or browse), **2 Motor** (every `data/motors/*.eng`, with impulse and burn time), **3 Conditions**
(launch elevation, wind), **4 Run simulation**, **5 Results** (where this run was saved, open-folder button, optional export in
CSV/Parquet/JSON). *Advanced settings* (collapsed) hold dry mass, thrust scale, azimuth, temperature offset, site elevation,
fidelity, timestep, integrator and seed. Every run is written automatically to `<workspace>/output/gui_runs/<time>_<config>/`
(`telemetry.csv`, `summary.txt`, `flight_overview.png`); the workspace is the repository (source) or `Documents\RocketSimulator`
(executable), overridable with `ROCKETSIM_WORKSPACE`. The runs go in a background thread; the tabs hold the *Getting started* page
(version, how it was launched, where files are, the last run), stacked zoomable graphs of any schema variables with event
markers, a timeline scrubber with a 3-D view (orientation, trajectory, ground, wind arrow; coloured by flight phase), the Summary
and the Data browser (open a dataset directory, reproduce and plot one run). Large generation is headless only.

Launcher options: `python -m rocket_sim --version`, `--check` (data files and GUI library present?), `--self-test report.json`
(start the GUI off-screen, run the default vehicle, write a JSON report; used to test the executable). Errors that have a
user-level fix (missing data folder, PySide6 not installed) are shown as a message and written to
`<workspace>/logs/gui_error.log` instead of a traceback.


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


## Vehicle files, controller state source, HIL (V1.2)

```yaml
vehicle_file: ../vehicles/example_tvc_demo.json   # instead of a `rocket:` section (mutually exclusive)
controller:
  type: tvc_attitude        # none | tvc_attitude | schedule | python | hil
  rate_hz: 100
  state_source: estimate    # auto (default) | estimate (refuses without nav_kf at fidelity >= 5) | truth (recorded + warned)
  compute_time_s: 0.002     # flight-computer compute time and downlink latency: delay before the actuator sees the command
  downlink_latency_s: 0.001
  uplink_latency_s: 0.0     # HIL bridge only
  design_inertia_scale: 1.0 # flight computer's belief about I_yy/(T lever) relative to as-built (1 = perfect knowledge)
# HIL: controller: {type: hil, params: {fc: reference}}   or   {command: [python, -m, rocket_sim.hil.flight_computer]}
```
