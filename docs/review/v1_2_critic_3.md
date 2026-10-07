# V1.2 critic review 3 (final round, independent, read-only)

Ran the full suite (430 passed), ruff, mypy; the CLI end to end (`vehicle`, `simulate`, `timing`, `hil` + replay, `generate-dataset`); bit-reproducibility;
the development registry (+0.50 / +13.41 / -4.90 %, unchanged); a vehicle of the reviewer's own design with products of inertia (CG and tensor to 1.8e-15 relative at
six burn times); a V1.1 vs V1.2 closed-loop comparison; 8 mutation tests on key tests (**all 8 caught**); a pipe-transport flight computer is byte-identical to the loopback one.

| category | weight | score |
|---|---|---|
| Vehicle modeling | 15 % | 7.0 |
| Physics correctness | 15 % | 7.5 |
| Aerodynamics | 15 % | 6.0 |
| Sensors / estimation | 15 % | 6.5 |
| HIL architecture | 15 % | 7.0 |
| Numerical correctness | 10 % | 8.0 |
| Testing | 10 % | 8.0 |
| Architecture / documentation | 5 % | 6.5 |
| **weighted total** | | **7.03 / 10** (rounds 1-2: 6.65, 6.50) |

Requirements: PASS R3, R5, R7-R15, R17; PARTIAL R1 (two HIL bridges), R2, R4, R6, R16. No critical defect; all round-1/2 fixes verified (D9/H1 regression gone, phase leak closed).

## New defects and disposition

| # | sev | defect | disposition |
|---|---|---|---|
| N1 | med | fin sets silently accepted and ignored `cg_x_m`, `offset_m`, `inertia_kgm2`, `mass_source` (and the schema allowed them) | **fixed**: those keys are errors for fin/control sets; `mass_source` removed; schema regenerated; tests |
| N2 | med | the supersonic 4.7 kg example printed a non-provisional "+-8 %" apogee | **fixed**: bands carry the provisional star and an "EXTRAPOLATED" line whenever the Mach range warning fired or the liftoff mass is outside 5-30 kg; test |
| N3 | med | `rocketsim hil` forced uplink 0 (flag default), ignoring the config: D1 reintroduced on the CLI path | **fixed** (flag default None; explicit flag overrides); CLI test on logged messages |
| N4 | med | `--command python -m ...` unusable (argparse treats `-m` as an option) | **fixed** (`REMAINDER`); CLI test against a child process |
| N5 | low-med | table models never got an alpha (or, for table2d, Mach) range | **fixed**: ranges come from the table axes; test |
| N6 | low-med | `state_source: estimate` rejected with `type: hil` | **fixed** (meaningless for a bridge, accepted) ; test |
| N7 | low | misleading timing warning wording | fixed |
| N8 | low | stale numbers; V1 bridge duplicates `hil/` | numbers refreshed; V1 bridge **marked deprecated** (kept for compatibility) |
| N9 | low | absurd fin spans accepted | compile-time **warning** (span > 2 body diameters) |

Still open (all documented): per-component mass-changing flag and general mass-flow model, motor-mount geometry, Reynolds range check, HIL jitter / deadline /
late-reply semantics (a stale reply aborts the run), reference computer mirrors only default estimator settings and TVC only, as-built knowledge in the controller's authority
(`design_inertia_scale` models imperfect knowledge), propellant as a fixed 0.9-scaled cylinder, one fin set.
