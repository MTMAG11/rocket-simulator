# V1.1 audit of the committed V1 (done before any V1.1 change)

Scope: commit `97daa17` ("V1"). Method: run it, read it, test its claims. No language or architecture migration was done or
needed (Python 3.11, same package layout).

## 1. What was run

| check | result |
|---|---|
| full test suite, not slow | 159 passed |
| `rocketsim simulate configs/example_g80.yaml` | apogee 919 +- 70 m, max velocity 198 m/s, static margin 3.1 cal (printed uncertainties extrapolated from the 20 kg-class validation) |
| `rocketsim validate` x 3 | +0.50 / +13.41 / -4.90 % apogee (barometric-equivalent), identical to the numbers in the README and `docs/validation.md` |
| `crosscheck_rocketpy.py` | < 0.6 % vs RocketPy on dynamics |
| ruff, ruff format, mypy | clean |

## 2. Claims in the README/docs checked against the code and data

| claim | verdict |
|---|---|
| "6-DOF quaternion RK4 with event location" | true; verified by tests (precession, quaternion kinematics, order of accuracy) |
| "predicts apogee +0.5/+13/-5 % ... in-sample" | numbers reproduce. The Prometheus structure change was made after seeing that flight, so *all three* are development flights; there was **no hold-out** (fixed in V1.1) |
| "Barrowman normal force + component drag build-up" | true, but for a **single body diameter** only; a transition or boat-tail could not be represented (NDRT, Bella Lui, Erebus-type vehicles are not) |
| "TVC with actuator limits" | true; physics uses the actuator output. No aerodynamic control surfaces |
| "simulated sensors ... Kalman navigation filter" | true; the filter is *linear* (6 states), not an EKF - as stated |
| "fidelity 6 high-fidelity preset" | numerically identical to fidelity 5; a preset, not extra physics (documented, kept) |
| test counts / numbers in `docs/testing.md`, `physics.md`, `performance.md` | partly stale (counts, convergence table, performance run on a different machine load) - refreshed in V1.1 |

## 3. Findings

| # | finding | severity | V1.1 action |
|---|---|---|---|
| A1 | Aero is Barrowman-only; no model hierarchy; no `CD/CL/Cm(M, alpha, Re)` interface; CP is Mach-independent; no lookup-table path with alpha | high (spec objective) | model hierarchy simplified / barrowman / enhanced / table / table2d behind one interface; tests |
| A2 | Single-diameter airframe: no transitions, boat-tails, motor section or payload geometry; CG and inertia typed in by hand | high | `Assembly` of sections, computed CG, inertia tensor path, CP from geometry |
| A3 | Crossflow drag acted at a fixed fraction of length instead of the planform centroid | low | fixed; changes golden numbers by 0.01 % (physics 1.2.0) |
| A4 | Control: TVC only; no control-surface model; actuator class TVC-specific | high | generic `ActuatorBank`, `FinMixer`, fin forces/moments from the ACTUAL deflection, roll channel |
| A5 | Magnetometer used only at pad alignment; alignment ignored declination; no gyro-bias handling; GPS single noise value, no dropout/start-up delay; no sensor misalignment | medium | see `sensors.md`; the in-flight magnetometer use is *not* added (documented) |
| A6 | Dataset windows: truth inputs were linearly interpolated onto the grid (reads one native sample after the grid time) | medium (small, but a real causality leak) | all inputs zero-order held; detector with negative control |
| A7 | Leakage check only found bit-identical parameter vectors | medium | nearest-neighbour near-duplicate report |
| A8 | No domain-randomisation dependence structure (every parameter independent) | medium | copula correlations, linked parameters |
| A9 | Validation: 3 flights, all development, no registry, no tiers, no hold-out discipline, no input-uncertainty analysis | high | registry, tiers, DEV/CAL/HOLDOUT, logged holdout protocol, input MC, sensitivity, error budget |
| A10 | Telemetry schema: no per-fin columns; `columns_for` ignored estimator at fidelity < 5 | low | fin columns; truth estimator columns from fidelity 3 |
| A11 | Docs: single-diameter and "no control surfaces" limitation text; stale numbers | low | rewritten |
| A12 | Performance table measured on one run; shard sizing in the benchmark too coarse for the worker count | low | fixed; ladder 1/100/1000/10000 |

## 4. Not changed (deliberately)

Flat non-rotating Earth; solid motors only; linear KF (no EKF - design in `sensors.md`); in-flight magnetometer use;
rail friction/tip-off; Reynolds effects on pressure drag. Each is listed in the error budget.

## 5. Limits of this audit

It was performed by the same author/agent that produced V1.1 and the audit was written as findings were made; the independent
review cycles that follow (`docs/review/v1_1_critic_*.md`) exist to catch what this misses.
