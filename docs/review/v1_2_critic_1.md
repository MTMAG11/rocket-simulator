# V1.2 critic review 1 (independent, skeptical, read-only)

Ran the full suite (394, all passed), ruff, mypy; verified the audit against commit `8a8ff28`; verified the vehicle-file tensor sign
convention end to end (JSON -> `MassComponent` -> `MassProps.tensor()` -> Euler dynamics: torque-free energy and |L| conserved to 1e-14,
a sign-flipped solver drifts 3 %); tried truth-leak mutations; reproduced the closed-loop tilts in the docs.

## Scorecard

| category | weight | score |
|---|---|---|
| Vehicle modeling | 15 % | 6.5 |
| Physics correctness | 15 % | 7.5 |
| Aerodynamics | 15 % | 6.0 |
| Sensors / estimation | 15 % | 6.5 |
| HIL architecture | 15 % | 6.0 |
| Numerical correctness | 10 % | 8.0 |
| Testing | 10 % | 6.5 |
| Architecture / documentation | 5 % | 6.5 |
| **weighted total** | | **6.65 / 10** |

Requirements: PASS R3, R7, R9, R10, R11, R17; PARTIAL R1, R2, R4, R5, R6, R8, R12-R16.

## Defects and disposition

| # | sev | defect | disposition |
|---|---|---|---|
| D1 | high | `controller.uplink_latency_s` silently ignored for `type: hil` (timing report lied) | **fixed**: the simulator fills the bridge from the config; explicit `params.uplink_latency_s` overrides; config-path test |
| D2 | high | design data / authority carries true (dispersed) mass properties; doc said no mass/CG crosses | **doc corrected** (as-built knowledge stated explicitly); **new knob** `controller.design_inertia_scale` for imperfect knowledge (tested); the as-built assumption itself remains |
| D7 | high | HIL tests could not detect a sample-value leak (noise-free sensors; one sensor checked; blacklist for `design`) | **fixed**: noisy sensors, every delivered value of every sensor bit-identical to a logged measurement, whitelist for init/design, **negative control** that tampers with samples |
| D3 | med | tensor not required PSD | **fixed** (eigenvalue check) |
| D4 | med | density/thickness unchecked | **fixed** |
| D5 | med | fin count truncation, raw ValueErrors, offsets outside airframe, unused `mount`, tensor+shape conflict | **fixed** (all raise `ConfigError`; `mount` removed) |
| D6 | med | compile mutated the input; report fin CG differed from the physics | **fixed**; tests compare every component row to the physics |
| D8 | med | no pipe timeout; child process leaked from the CLI | **fixed** (reader thread + timeout kills the process; CLI and bridge `close()`) |
| D9 | med | aero ranges only labels | **fixed as warnings** (Mach and angle of attack before apogee vs stated range); Barrowman alpha range changed from an unfounded 180 to 15 deg |
| D10 | low | equal-time samples ordered by name | **fixed** (gyro, accel, mag, baro, gps = in-process precedence) |
| D11 | low | latency test passed on a tumbling vehicle | **fixed** (40 ms; doc says 120 ms loses control) |
| D12-D14 | low | wrong module name in docstrings, 178 vs 180 deg, stderr context | fixed / fixed / not changed (stderr is inherited) |

Not addressed (acknowledged): per-component mass-changing flag and general mass-flow model; motor mount geometry; propellant as a fixed 0.9-scaled
cylinder; RocketPy cross-check covers CNa/CP only (von Karman vs ogive nose, 0.06 cal tolerance); `state_source: auto` falls back to truth with a warning; mixed
axis convention in one file (x aft for positions, x forward for the tensor: documented).
