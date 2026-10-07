# V1.2 audit of the committed V1.1 (`8a8ff28`), written before any V1.2 change

Method: read the code paths named in the V1.2 brief, traced the truth -> sensor -> estimator -> controller -> actuator -> physics
chain in `simulation/simulator.py`, ran the suite. The code is authoritative over the README.

## Already implemented (verified present; not rebuilt)

| brief item | where | status |
|---|---|---|
| component airframe (sections, transitions, boat-tails, fins, control surfaces, extra masses) | `vehicle/assembly.py`, `config/schema.py` (`sections`, `masses`) | implemented, analytically tested |
| CG/inertia computed from components, full 3x3 tensor path, CG shift during burn | `vehicle/mass.py` | implemented (`MassComponent` has offsets and products); tensor vs independent numpy sum |
| aero hierarchy + `CD/CL/Cm(M, alpha, Re, geometry)` | `vehicle/aero.py` | implemented; Reynolds enters **only** through skin friction |
| actuator chain: command -> delay -> lag -> rate -> angle limit -> actual state -> physics | `control/actuators.py`, `simulator.py` | implemented, tested incl. "physics sees the actual state" |
| sensors (noise, bias walk, scale, quantisation, saturation, rate, latency, dropout, start-up delay, misalignment) | `sensors/sensors.py` | implemented, tested |
| estimator interface (None / truth / linear KF), gyro bias on the pad | `estimation/estimators.py` | implemented; no EKF (stated) |
| truth vs `meas_` vs `est_` columns, commanded vs actual TVC and fins | `data/schema.py` | implemented (gaps below) |
| a HIL bridge (lock-step JSON lines over a byte transport) | `control/hil.py` | implemented (V1), one loopback test; **latest sample only** |
| RocketPy cross-checks | `validation_data/crosscheck_*.py` | implemented |

## Gaps found (these define V1.2)

| # | gap | consequence |
|---|---|---|
| G1 | **No vehicle file format.** A vehicle can only be described through the simulation YAML (`rocket:` section) with hand-typed lumped numbers or sections. No component list with type/position/material/shape, no CAD-friendly tensor input, no deterministic versioned format | the stated goal (take a real design and enter it without magic numbers) is not met |
| G2 | no per-component mass-properties report (component table, CG(t), tensor, CP, SM) | cannot audit what the simulator derived |
| G3 | `MassItemCfg` has only `ixx/iyy` (no `izz`, no products), no shape-derived inertia; shell masses cannot be derived from wall thickness x density | the full-tensor path exists internally but cannot be fed |
| G4 | **Truth leaks into the controller by default**: `use_truth_ctrl = use_truth or no sensors or no estimator` silently falls back to the true state; the controller authority callbacks use the *true* (dispersed) `mass_props` | a controller can be developed against truth without any indication in the record |
| G5 | no test proving the estimator and controller consume only measurements/estimates | claim in the V1.1 docs unproven |
| G6 | aerodynamic **provenance** is not recorded; table models silently look like the analytic ones in a dataset | estimated and imported coefficients are indistinguishable in ML data |
| G7 | HIL: only the latest sample per channel crosses the boundary (a 400 Hz IMU read at a 100 Hz tick loses 3 of 4 samples); no uplink/compute/downlink latency; no message log or deterministic replay; no process transport; no timing report | the interface is not yet what a real flight computer needs |
| G8 | schema: no true specific force, no `izz`/products, no true CG offsets, no estimated biases, no per-fin commanded deflections after mixing | truth/measurement/estimate/control separation incomplete for ML |
| G9 | timing is implicit (estimator runs per step; no report of rates/latencies) | HIL will expose it |
| G10 | actuators are held at the step-start value (left-endpoint hold, documented in V1.1 review 3) | up to ~h/2 extra lag in closed-loop runs; **documented, not changed in V1.2** (changing it alters physics and the holdout fingerprint) |

## Explicitly not in scope (kept as is)

No rewrite, no new dependencies, no EKF, no CFD, no autonomous-landing controller, no `.ork` importer, no Fusion plugin, no real-time
pacing, no GUI change.

## Evidence standard for V1.2

Each new capability is classified in `docs/status_matrix.md` as implemented / analytically verified / numerically verified /
cross-checked / experimentally validated / not validated. None of the V1.2 additions is experimentally validated.
