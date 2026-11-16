# Autonomous Rocket Project — Technical Context

**Snapshot date:** November 16, 2026  
**Primary simulator repository:** https://github.com/MTMAG11/rocket-simulator  
**Latest visible commit at snapshot:** `e05f4e99b3e98f62a692ce2167862d8fd88fe865` (`V1.2.1`, 2026-10-09)

This document is a handoff reference. Treat dated claims as a snapshot, not as proof of the current local or physical state. Re-check the repository and ask for new evidence when needed.

## 1. Project mission and system boundaries

The Autonomous Rocket Project is a multi-year effort to develop the engineering capability and systems needed for an autonomous rocket. The long-term emphasis is guidance, navigation, and control (GNC), supported by a physics-based simulator, avionics, sensor modeling, state estimation, control, and eventually trajectory optimization.

Treat these as related but distinct workstreams:

1. **Rocket Simulator:** physics, vehicle/aero models, sensors, estimation, control, HIL interfaces, datasets, and validation.
2. **Avionics:** physical flight-computer electronics, sensor integration, data logging, wiring, packaging, and bench testing.
3. **GNC development:** state estimation, control algorithms, guidance logic, and optimization.
4. **Experimental validation:** measured bench and flight data, calibration, uncertainty, and repeatable test procedures.

Do not infer that progress in one workstream automatically validates another. A simulator model is not a validated physical sensor; a passing unit test is not a flight test; a GUI feature is not evidence of physical flight readiness.

## 2. Repository baseline

- Repository: `MTMAG11/rocket-simulator`
- Local path previously used: `C:\\Github\\rocket-simulator`
- Python requirement: >= 3.11
- Build backend: setuptools
- Main package: `src/rocket_sim`
- GUI: PySide6
- Numerical stack: NumPy, SciPy, Matplotlib, PyArrow, PyYAML
- Development tools: pytest, Ruff, mypy, psutil (dev extras)

The README describes a 6-DOF flight simulator and dataset generator for solid-motor rockets, intended to support autonomous-rocket GNC development.

### Latest visible Git history

- `e05f4e9` — `V1.2.1` — 2026-10-09
- `011d9be` — `1.2 UI Update` — 2026-10-08
- `e850537` — `Updated ReadMe` — 2026-10-08
- `3110e30` — `V1.2` — 2026-10-08
- `bfde233` — `V1.2` — 2026-10-07
- `8a8ff28` — `V1.1` — 2026-10-05
- `97daa17` — `V1` — 2026-10-05

No newer commit appeared in the recent history query as of November 16. This does not rule out local uncommitted changes or another branch.

### Version identifiers

`src/rocket_sim/version.py` declares:

- `SIM_VERSION = "1.2.0"`
- `PHYSICS_VERSION = "1.2.1"`
- `SCHEMA_VERSION = "1.2.0"`
- `CONFIG_VERSION = 1`
- `DATASET_VERSION = "1.1.0"`

The latest commit message says `V1.2.1`, but the software version identifier remains 1.2.0 while the physics version is 1.2.1. Treat these as separate version dimensions; do not silently rewrite them. Check the intended release convention before changing version metadata.

## 3. Current repository architecture

Top-level areas include:

- `configs/` — example simulation, dataset, batch, experiment, and domain-randomization configurations.
- `data/` — bundled runtime data, including motor data.
- `docs/` — architecture, physics, aerodynamics, frames, vehicle format, sensors, HIL, datasets, validation, testing, performance, packaging, and usage documentation.
- `packaging/` — executable packaging support.
- `scripts/` — helper scripts.
- `src/rocket_sim/` — application and simulation package.
- `tests/` — automated tests.
- `validation_data/` — registry, flight inputs, and independent comparison tools.
- `validation_results/` — logged validation results, uncertainty analysis, and physics-version result sets.
- `vehicles/` — vehicle descriptions.

Observed package areas under `src/rocket_sim/`:

- `config/`, `control/`, `data/`, `environment/`, `estimation/`, `hil/`, `motor/`, `physics/`, `sensors/`, `simulation/`, `ui/`, `validation/`, `vehicle/`.

UI files observed include `catalog.py`, `channels.py`, `graphs.py`, `launcher.py`, `main_window.py`, `motor_browser.py`, `styles.py`, and `view3d.py`.

Simulation files observed include `builder.py`, `phases.py`, `record.py`, `simulator.py`, and `timing.py`. Physics files observed include `dynamics.py`, `integrators.py`, and `math3d.py`.

For implementation questions, inspect the current file in GitHub or the user's current local checkout. Do not rely on old V0.1 file names or assume the previous architecture still applies.

## 4. Simulator capabilities documented in README

The repository documents:

- 6-DOF rigid-body flight, with 3-DOF and 1-D fidelity levels.
- Motor thrust curves from `.eng` files.
- Component-based, versioned `vehicle.json` descriptions.
- Derived mass, center of gravity, inertia tensor, center of pressure, and static margin.
- Aerodynamic model hierarchy for `CD/CL/Cm(M, alpha, Re, geometry)`.
- TVC and control-fin actuator models.
- Sensors, a linear Kalman filter, and controller interface.
- Separation of truth, measurement, estimate, command, and actual state.
- HIL wire protocol, transports, reference flight computer, and timing model.
- Bundled motors and online motor discovery/download through ThrustCurve.org.
- Parallel, reproducible Monte Carlo dataset generation.
- Real-flight validation protocol.

README capability statements indicate documented software features. Use `docs/status_matrix.md` to distinguish implemented, analytically verified, numerically verified, independently cross-checked, and experimentally validated capabilities.

## 5. Validation evidence and limitations

The status matrix uses evidence levels:

- **I:** implemented and exercised by tests.
- **A:** analytically verified.
- **N:** numerically verified.
- **X:** cross-checked against independent code or reimplementation.
- **E:** experimentally validated against real flight or bench data.

These levels are not interchangeable.

### Real-flight comparison

The validation document reports seven 7–24 kg subsonic solid-motor flights:

- Development: Bella Lui, Notre Dame 2020, Prometheus.
- Calibration: Genesis, Astra.
- Holdout: Erebus 11, Cavour.

Reported holdout apogee errors:

- Erebus 11: -6.40%.
- Cavour: +4.52%.
- Holdout RMS apogee error: 5.54%, n = 2.

Two holdouts do not establish broad generalization.

### Key validation limitations

- No small-rocket (<1 kg) validation data in the repository.
- No supersonic flight validation; no validation flight exceeded approximately Mach 0.9 in simulation.
- No component of the vehicle, actuator, sensor, estimator, or HIL model has been validated against real hardware.
- No hardware has been connected to the HIL interface.
- No Tier 1 instrument-grade validation data were available.
- Several flight inputs are secondary-source estimates, not measured values.
- The linear Kalman filter is not an EKF; EKF is design-only in the status matrix.
- Sensor models use generic MEMS assumptions and omit some real-world effects such as vibration, temperature dependence, and g-sensitivity.
- HIL has no serial/UDP transport documented in the status matrix; loopback, pipe, and record/replay are implemented.
- The holdout hash chain does not protect the final line or a truncated tail; an external anchor such as a Git commit is needed.

### Calibration

The validation document describes a proposed global drag multiplier of 0.957. It improved the two calibration flights but did not improve holdout RMS; it was not adopted. Do not revive it as an accepted global correction.

### Independent cross-checks

The validation document reports RocketPy cross-checks:

- Trajectory cross-check differences under 1% for the documented comparison case.
- Geometry/stability cross-checks for four vehicles, with matching normal-force slope values and small CP differences.

These are useful independent code checks, not experimental validation of all models.

### Required validation discipline

- Preserve the development/calibration/holdout split.
- Do not tune against holdout data and continue calling it holdout.
- After physics-source edits, inspect holdout status and follow the documented migration/re-evaluation process.
- Keep validation inputs, hashes, configuration, telemetry, motor files, and comparison code traceable.
- Record exact test commands and results.

## 6. Avionics — historical known configuration

Earlier project notes recorded:

- Flight-computer board: ESP32 XH-32S.
- Barometer: BMP388, I²C address `0x77`.
- IMU: ICM42688-P, I²C address `0x69` in a successful manual-read test.
- I²C: SDA 21, SCL 22.
- SD: CS 5, SCK 18, MISO 19, MOSI 23; previously tested at 1 MHz.
- Intended logging interval: 20 ms (50 Hz).
- Initial prototype: breadboard.

Earlier tests showed successful BMP388 readings and successful manual ICM42688 register/configuration reads. The library `begin()` result was inconsistent with successful manual reads. This is historical context only; confirm the current wiring, board, library version, and behavior before relying on it.

Earlier mechanical fit-check envelope: 1.25 in internal diameter and 5.5 in available length. A Fusion 360 fit-check model was being planned; completion is unconfirmed.

## 7. Project working preferences and collaboration rules

- Check the current GitHub repository before giving implementation advice. If the user pastes local code and it differs from GitHub, treat the pasted local code as authoritative for that debugging session.
- Work incrementally. Diagnose the actual issue before proposing broad refactors.
- Do not refactor unrelated code or expand scope just to make an implementation look cleaner.
- Explain the reasoning so the user learns; avoid dumping unexplained copy-paste solutions.
- When debugging, request exact errors, commands, logs, and relevant files rather than guessing.
- Distinguish verified facts, historical notes, assumptions, and unknowns.
- Never claim tests passed unless their output was actually observed.
- Keep simulator development, avionics bench work, GNC design, and flight validation as separate workstreams.
- Keep weekly reports factual and in a straightforward student-engineering voice. Do not invent completed work.
- The user wants robotics treated as a separate project, not folded into rocket-project reports.

## 8. Current status and immediate priorities

As of this snapshot, the repository's last visible commit is October 9, 2026. The current local checkout, test-suite result, physical avionics state, and fit-check CAD status are unknown.

Recommended order:

1. Compare the local checkout against GitHub and record uncommitted changes.
2. Run the repository's documented test suite and a minimal simulator smoke test; save exact commands and outputs.
3. Review `docs/status_matrix.md`, `docs/validation.md`, and `docs/testing.md` before modifying physics or validation logic.
4. Confirm the physical avionics assembly and repeat a controlled sensor + SD logging test.
5. Confirm actual board, connector, cable, and mounting dimensions before completing the mechanical fit-check.
6. Update this handoff only when new evidence or decisions exist.

## 9. How to work on this project

Start each technical task by identifying the relevant workstream and checking the current repository state. Summarize the current implementation, identify the smallest next step, and explain the test or measurement that will verify it. Do not assume a feature is complete merely because a module or interface exists. Keep all safety-critical design decisions subject to appropriate adult supervision, established rocketry safety codes, and qualified review; do not use simulation output alone as authorization for a real flight.
