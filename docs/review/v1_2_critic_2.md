# V1.2 critic review 2 (independent, skeptical, read-only)

Ran the full suite (420, all passed), ruff, mypy; adversarial HIL probes; vehicle compile edge cases recomputed with the reviewer's own numpy
(CG and full tensor to 2e-16 incl. an off-axis motor-mount mass); truth-leak probes; aero-warning checks on non-holdout flights; timing report vs simulated
latency; **35 mutation tests** on a copy of the repo (29 caught, 6 survived).

| category | weight | score |
|---|---|---|
| Vehicle modeling | 15 % | 7.0 |
| Physics correctness | 15 % | 7.0 |
| Aerodynamics | 15 % | 5.5 |
| Sensors / estimation | 15 % | 6.0 |
| HIL architecture | 15 % | 6.5 |
| Numerical correctness | 10 % | 7.5 |
| Testing | 10 % | 6.5 |
| Architecture / documentation | 5 % | 6.0 |
| **weighted total** | | **6.50 / 10** (round 1: 6.65) |

Round-1 defects: D1, D3-D8, D10, D13 verified fixed; D2 partly; **D9 regressed** (see H1); D11 not fixed.

## New defects and disposition

| # | sev | defect | disposition |
|---|---|---|---|
| H1 | high | the new AoA range warning fired on the pad (speed 0 -> alpha 90 deg): spurious on the flagship example and two development flights | **fixed**: only rows with airspeed > 20 m/s before apogee; regression test on a windy nominal flight and two validation vehicles |
| H2 | high | the TRUE flight phase reached in-process controllers (state_source "estimate") | **fixed**: estimate mode passes an estimated phase (0 pad / 2 launch detected); test |
| H3 | high | `estimator: truth` was labelled "estimate", silently | **fixed**: treated as truth-fed, warned, flagged |
| H4 | high | latency test passed on a tumbling vehicle; hil.md claim wrong | **fixed**: measured 0/10/20/40 ms -> 3.5/7.6/17/lost; test asserts the ladder; doc corrected |
| M1 | med | tests pinned only position and vz of the controller input | **fixed**: quaternion, rates, phase |
| M2 | med | fin command latency untested | **fixed** (test) |
| M3 | med | in-process authority keyed to the true clock, HIL to detected launch | **fixed**: both index by time since launch; the two loops now agree (3.5 deg both) |
| M4 | med | replay completeness unchecked | **fixed** (`finish()`, CLI) |
| M5 | med | NaN / negative uplink accepted | **fixed** (validated). Deadline-miss / late-reply semantics: **not implemented** (a stale reply aborts the run) |
| M6 | med | the tensor-sign reference shared the production helper (mutation survived) | **fixed**: literal hand-calculated matrix, also through the vehicle JSON |
| M7 | med | reference computer rebuilds estimator defaults; TVC only | documented |
| L1 | low | actuator queue head-of-line blocking | **fixed** (ordered insertion; test) |
| L2 | low | noisy timing warning; worst case included GPS | **fixed** (warning removed; IMU path reported separately) |
| L5 | low | legacy fin CG omits the sweep term | documented (changing it alters validated results) |
| L7 | low | disabled magnetometer silently gave a bad attitude | **fixed**: `nav_kf` requires accelerometer and magnetometer |

Not addressed: per-component mass-changing flag, motor-mount geometry, Reynolds range check, HIL jitter/deadline model, `authority` closure still holds the true vehicle (as-built knowledge, documented).
