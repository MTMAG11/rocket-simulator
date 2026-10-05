# V1.1 independent critic review 1

Reviewer: independent subagent, read-only; did not simulate holdout flights. Ran 283 non-slow tests (all passed), ruff, mypy
(clean); recomputed every RMS in validation.md; reproduced the RocketPy geometry cross-check and Bella Lui.

## Scorecard (strict)

| category | max | score | main reason |
|---|---|---|---|
| physics | 20 | 14 | consistent moment algebra; control-fin lift is linear (no stall/hinge/local-alpha); supersonic unvalidated; Erebus 11 unexplained |
| aerodynamics | 15 | 9 | hierarchy delivered and honest; default unvalidated; enhanced model untested by data; CNa cross-check is the same equations |
| validation | 20 | 11 | honest documentation; protocol only procedural with hygiene defects; MC band partly circular |
| numerics | 10 | 6 | good invariants; loose convergence test; 3 cm event floor |
| sensors/estimation | 10 | 6 | real audit; no EKF (stated); no attitude states / accel bias; nothing validated on real data |
| data/ML | 10 | 6 | causal inputs, copula, KS, experiments; near-duplicate detector nearly vacuous; schema version not bumped |
| architecture | 5 | 3.5 | clean interfaces; holdout guard bypassable, CLI miswired |
| testing | 5 | 3.5 | broad; several tautological/loose tests |
| docs | 5 | 3.5 | candid; several stale items |
| **total** | 100 | **62.5** | confidence in headline claims: **medium** |

## Findings and disposition

| # | sev | finding | disposition |
|---|---|---|---|
| 1 | high | holdout status is *stale* for the shipped tree (fingerprint changed after the migration record) while docs read as current | **real.** Cause: further edits after the migration. Fixed procedurally: `validate-registry --status`, documented status rule; the migration is re-run as the *last* step of the release (see final report) |
| 2 | high | CLI never passed `--log`; holdout log was hand-assembled; `validate` bypassed the guard; `frozen_physics_version` unchecked; no tamper evidence | **fixed:** log path wired, holdout needs a log and a matching freeze, `validate` refuses holdout flights, SHA-256 hash chain + `verify_log`, `--status`; tests. The first four log entries remain trust-based (documented) |
| 3 | high | `cavour.yaml` edited seconds before its holdout run; doc said "altitude + velocity" | the edit was a YAML quoting fix (parse error stopped the run); disclosed in validation.md; table corrected. Order of events is attested, not proven |
| 4 | med | 7 % drag band chosen after seeing misses; n = 40 noisy | **addressed:** stated as a fitted nuisance term; MC rerun with n = 150 (see validation.md §6) |
| 5 | med | wind row of the sensitivity table was a no-op when wind model is `none` | **fixed:** sensitivity switches to a constant wind of 0 first; table regenerated |
| 6 | med | near-duplicate detector near-vacuous (absolute eps; 2-D test; tiny perturbation); temporal check not on windowed path | **fixed:** relative threshold, 20-D test at 0.02 sigma with a negative case, windowed path checked; doc claim reduced to "detects copying" |
| 7 | med | schema columns changed, `SCHEMA_VERSION` still 1.0.0 | **fixed:** 1.1.0 |
| 8 | med | convergence test loose; doc claim "< 1 cm" wrong for 3-DOF | **fixed:** test requires the finest dt < 1 cm and not beaten by coarser; docs say 3.5 cm floor |
| 9 | low | tautological tests; inertia "independent summation" shares helpers; norm test helped by renormalisation | trivial asserts removed; shared-helper limitation acknowledged (the Euler-invariant test is the independent check); not otherwise changed |
| 10 | low | silent clamps in mass properties | acknowledged, not changed |
| 11 | low | actuator delay quantised to step boundaries | documented |
| 12 | low | with-cal RMS 5.65 vs 5.67; geometric vs barometric nominal apogee | corrected / noted |

Score after fixes is the next review's job; this table is only the response, not a re-score.
