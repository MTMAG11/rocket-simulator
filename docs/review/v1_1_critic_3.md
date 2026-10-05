# V1.1 independent critic review 3 (final round, physics 1.2.1)

Reviewer: independent subagent, read-only, no holdout simulation. Ran the full suite (293 tests, all passed), ruff check/format and
mypy (clean), example and closed-loop simulations, bit-reproducibility of flights and datasets, `experiment --verify` (including
a tampered hash), an independent `solve_ivp` coast integration (agrees to 1.5e-9 m), a quaternion-norm probe without
renormalisation (|q| drift 1e-7, apogee unchanged), a step-size/control-rate sweep of a fin-controlled run, both RocketPy
cross-checks, and ~10 documentation numbers against recorded outputs. A clean offline install is not possible (needs network).

## Scorecard (strict): 63.5 / 100 (rounds 1 and 2: 62.5, 62.5)

| category | max | score |
|---|---|---|
| physics | 20 | 14 |
| aerodynamics | 15 | 9 |
| validation | 20 | 11 |
| numerics | 10 | 6.5 |
| sensors/estimation | 10 | 6 |
| data/ML | 10 | 6.5 |
| architecture | 5 | 3.5 |
| testing | 5 | 3.5 |
| docs | 5 | 3.5 |

Confidence: arithmetic and reproducibility **high**; "holdout is clean" **medium-low** (mtimes plus an uncommitted, trust-anchored log; n = 2);
"aero hierarchy validated" **low**; "1.2.1 changes only controlled flights" medium-high.

## Findings and disposition

| # | sev | finding | disposition |
|---|---|---|---|
| 1 | high | holdout protocol cwd/log-path dependent; no external anchor; `--status` on a missing log said "unevaluated"/"intact" | default log/registry now anchored to the registry directory; `--status` warns when the log is missing. **The external anchor (committing the log) is not done: nothing in this tree is committed.** Open |
| 2 | high | actuator output applied with a left-endpoint hold (~h/2 extra lag; ~5 % change in peak fin deflection across dt/control rate; apogee < 0.05 m) | **not changed** (physics change would need another migration); documented in physics.md. Open |
| 3 | med | example `test` split replaced rather than merged the parameter list (3 of 21 randomised) | example fixed (full list repeated); manifest now warns when a split drops global parameters; test |
| 4 | med | near-duplicate detector silently dropped parameters | report lists `parameters_not_compared`; shipped example now compares 21 |
| 5 | med | `experiment --verify` vacuous for zero shards; `git.dirty` ties no experiment to a code snapshot | zero-shard verify now fails. Snapshot tie requires committing (not done) |
| 6 | med | error budget is for geometric apogee, validation for barometric (temperature sign differs) | footnote added; MC uses the compared quantity |
| 7 | low | reporting.py says "3 validated flights" | fixed |
| 8 | low | stale numbers/docs (4-7 %, 919 vs 918, 5.55 vs 5.54, performance raw file, convergence script note, tvc config 302 vs 305) | fixed |
| 9 | low | `PROJECT_ROOT` assumes a source/editable install | documented in usage.md |
| 10 | low | migration only checks apogee error | acknowledged |
| 11 | low | control-fin lift linear (no incidence/stall/hinge) | documented limitation |
| 12 | low | DR example's `wind.model` choice has one value | acknowledged (it just switches wind on) |

Round-1/2 fixes confirmed by this reviewer: fin memo bug, control phase, `--log` wiring, freeze check, `validate` refusing holdout flights,
schema 1.1.0, relative near-duplicate threshold with a 20-D test, ruff scope. `inputs_sha256` pinning exists in code but, as
the reviewer noted, in none of the six log entries (they pre-date it).
