# V1.1 independent critic review 2

Reviewer: independent subagent, read-only, no holdout simulation. Ran the full suite (292 tests, all passed), mypy (clean), ruff;
verified the copula, hash chain, near-duplicate detector and many document numbers independently; hand-computed a control-fin
moment (pitch 0.64139 N m, roll 0.13021 N m: simulator identical) and the inertia tensor against numpy (6e-16).

## Scorecard (strict): 62.5 / 100 (round 1: 62.5)

| category | max | score |
|---|---|---|
| physics | 20 | 13.5 |
| aerodynamics | 15 | 9 |
| validation | 20 | 11 |
| numerics | 10 | 6 |
| sensors/estimation | 10 | 6 |
| data/ML | 10 | 6.5 |
| architecture | 5 | 3.5 |
| testing | 5 | 3.5 |
| docs | 5 | 3.5 |

Confidence in headline claims: arithmetic and log **high**; "holdout is clean" **medium**; "aero hierarchy validated" **low**.

## Findings and disposition

| # | sev | finding | disposition |
|---|---|---|---|
| 1 | high | **physics bug**: `evaluate` memo key omitted `controls.fin`, so RK4 stage 1 used the previous step's fins; tests hid it by resetting the cache by hand | **fixed** (key includes fins; regression test without cache reset). Physics bumped to **1.2.1**; non-controlled results bit-identical (golden unchanged), closed-loop fin apogee 485.5658 -> 485.5657 m |
| 2 | high | holdout status stale for the shipped tree | **fixed at release**: `--migrate-fingerprint` re-run after the 1.2.1 change (all 4 entries reproduce to 1e-6 %); `--status` now `current`. The status rule stays: *any* later physics edit requires redoing it |
| 3 | med/high | log does not pin the inputs; chain does not protect the last line / tail; no git history | inputs (definition, sim config, telemetry, motor, comparison code) hashed from now on; gaps documented (needs an external anchor: commit the log). Not fully fixed |
| 4 | med | holdout flights also simulated in the error-budget model-form table (undisclosed) and in the MC; "6 of 7" pools all splits | disclosed in error_budget.md; MC text already says post hoc |
| 5 | med | controls and logged row one step out of phase | **fixed**: actuator output copied to the dynamics right after each actuator step |
| 6 | med | near-duplicate detector blind above ~0.05 sigma jitter | documented as detecting *copying* only; not strengthened |
| 7 | med | error_budget said "< 1 cm" for dt | **fixed** (<= 4 cm) |
| 8 | low | `validate` guard fails open if the registry file is missing | **fixed** (fails closed) |
| 9 | low | registry `mass_kg` metadata inconsistent with validation.md | **fixed** |
| 10 | low | "4-7 %" input sigma range | **fixed** (3-7 %) |
| 11 | low | stale docstring `I = diag`; convergence reference dt note | docstring fixed; physics.md states reference 0.0002 vs the test's 0.0005 (different checks) |
| 12 | low | static control-fin lift not Mach-corrected in `enhanced`; no stall/hinge for control fins; constant damping coefficients | documented limitation, not changed |
| 13 | low | ruff scope | **fixed** (third-party notebooks and docs excluded; `ruff check .` clean) |
