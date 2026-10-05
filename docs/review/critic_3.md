# Independent critic review 3 (final round)

Reviewer: independent. Date 2026-10-04. No source file was modified; scratch scripts lived outside the repo.

What I ran: full `pytest tests` (160 tests, all passed; ~20 min on the loaded machine, no failures; the `-q` summary line was not
flushed by the background launcher but every one of the 160 progress dots is a pass and `--collect-only` counts 160), `ruff check src tests`
(clean), `mypy src/rocket_sim` (clean, 59 files), `rocketsim simulate configs/example_g80.yaml`, `rocketsim batch configs/batch_example.yaml
--runs 12` (12/12 accepted), `rocketsim validate` on all three flights (the three `*_metrics.json` are byte-identical to
`validation_results/physics_v1.1.0/*`), a re-run of the whole sensitivity table (15 simulations), the geometric-altitude option (3
simulations), a held-out LOO point (Prometheus at drag_scale 1.430 -> -17.07 %), and several ad-hoc analyses of the Prometheus telemetry.
pandas/netCDF4 untouched. RocketPy cross-check not re-run.

## Score: 7.0 / 10   (round 2: 6.8, delta +0.2; round 1: 6.5)

Why up: the single biggest round-2 finding (N1, barometric vs geometric altitude) is genuinely fixed and I reproduced it end to end; the
headline numbers (+0.50 / +13.43 / -4.90 %, RMS 8.26 %) and every number in the sensitivity and LOO tables reproduce exactly from the
code; run id / config hash now include the motor file's content (tested); a golden-output test keyed to `PHYSICS_VERSION` exists; README,
physics.md, validation.md and `uncertainty.py` were largely re-worded and are now mutually consistent on the apogee numbers; the
overclaim in the README status line is gone; lint/types/tests stay clean (157 -> 160 tests).

Why only +0.2: (a) I found a new, concrete validation misreading in the Prometheus data (N6 below): the AltOS `speed` column is a *signed
vertical* velocity, not "total speed" as the docs, YAML and code comment assert (and the saved PNG visibly shows it). It does not change the
apogee headline but it makes the published Prometheus max-velocity error (+2.36 %) and velocity bias (+4.9 m/s) wrong; the correct like-for-like
numbers are -0.26 % and -9.4 m/s (the model decelerates too fast). (b) The large round-1/2 functional gaps are untouched: no aerodynamic
control surfaces, no guidance law, no run-level labels (landing x/y, miss distance...), no 100k demonstration, no gravity anchoring or
thrust-pressure correction, no input-uncertainty Monte Carlo of the validation flights, thin test coverage of GUI/benchmark/plotting/TableAero.
(c) Small residual doc drift and one hygiene claim ("git index synced") that is not quite true. Physics is not worse; nothing regressed.

Calibration note: 7 is "a sound, honest research-grade simulator and data engine, validated only on three amateur subsonic flights with
secondary-source inputs". NASA-level (10) needs measured atmosphere/wind, weighed vehicles, raw IMU/attitude/TVC flight telemetry,
supersonic validation and multi-tool agreement; none exists here. 8 needs control surfaces or an explicit descoping, run-level labels, a
demonstrated 100k run and at least one validation flight outside the 20 kg-class subsonic envelope.

---------------------------------------------------------------------------------------------------------------------------------

## 1. Status of the round-2 items (verified by running code)

| # | round-2 finding / ask | verdict | evidence |
|---|---|---|---|
| N1 | baro altimeter compared with geometric altitude | **FIXED, verified** | `telemetry.py::_baro_equivalent_altitude` (ISA inversion of the simulated pressure minus the pad value), default `altitude_reference: barometric` in `compare.py`. My re-run: apogee error +0.50 / +13.43 / -4.90 %; with `altitude_reference: geometric` -2.29 / +8.71 / +0.03 % (matches docs). Hand check of Prometheus: 3905 m geometric / (1 + 15 K / ~285 K) ~ 3710 m vs code 3712.6 m. RMS = 8.26 %, mean abs 6.28 %: README/validation.md "8 %"/"8.3 %"/"6.3 %" correct. Caveats: only AltOS is *known* to be barometric; Bella Lui/NDRT sources say "altimeter-derived"/"altimeter" (see docs item below); no unit test of `_baro_equivalent_altitude` at a known pressure (the round-2 ask), and the only self-consistency test runs with `altitude_reference: geometric`, so the new default path has only the slow real-flight regression test. |
| N2 | plot compared sim vertical velocity with real total speed | **CODE FIXED, premise wrong** | `plot_comparison` now plots `sim["speed"]` when the real channel is `speed`. But the real channel is a signed vertical velocity (N6), so the plot is now like-for-unlike the other way (see PNG: real goes to -30 m/s on descent, sim total speed stays positive; y label says "vertical velocity"). |
| N3 | run id ignored motor contents | **FIXED, verified** | `Simulation.cfg_hash = config_hash({"config":..., "input_files": sha})`; `test_run_id_depends_on_motor_file_contents` passes; example id changed to `5a86dcc42b2e0a43-s0`. Not fixed: `spec_hash` still hashes only the base config's motor (a motor swapped via `parameters` is not covered) and still has the bare `except Exception: msha = ""` (data/batch.py l.97); no test of spec_hash vs motor content (grep of tests: only the output_dir invariance test). git commit / numpy / scipy / pyarrow versions still not recorded. |
| N4 | git hygiene | **MOSTLY** | The ~30 `AD` entries are gone (status now: 121 `A`, 3 `AM`, 6 `M`, 5 `D`). But the claim "index synced" is not exact: `compare.py`, `tests/test_golden.py`, `tests/test_validation_cli.py` are `AM` (working tree newer than index); `.gitignore` still lists `*.egg-info/` twice; nothing committed (by design, but it means no iteration history exists, spec 82). |
| N5 | docs contradictions | **MOSTLY** | physics.md -0.3 % -> fixed; sensitivity table re-measured (I reproduced all 15 numbers); `uncertainty.py` rewritten. Residuals: "150 tests" (validation.md l.15, testing.md l.4; actual 160); README example block prints `920 +/- 70` and `200 +/- 20` but the code prints `919 +/- 70` and `198 +/- 16` (README example is hand-rounded, not pasted); `uncertainty.py` comment says max-velocity RMS 8.1 % and apogee-time RMS 4.7 % (actual from its own numbers: 8.37 % and 4.45 %); validation.md says "+-10 K moves the barometric apogee by 3-6 points" whereas the table shows 3.4/3.2, 3.1/2.9, 1.8/1.75 points per side (total span 6.5/6.0/3.6); `docs/datasets.md` l.107 still says the model "is only a few percent accurate". |
| item 2 | bound the temperature offsets with a small Monte Carlo | **NOT DONE** (sensitivity table only, one-at-a-time) | no input-uncertainty band on any validation apogee |
| item 6 | golden test keyed to PHYSICS_VERSION | **DONE** (`tests/test_golden.py`, 1e-9 relative, fidelity 2 and 3). Fragile across platforms/BLAS/libm, but that is the right direction. `test_real_flight_regression` tolerances are still loose and were widened to the new numbers (NDRT 16 %, i.e. 2.6 points of slack above the current 13.43 %). |
| items 8-13 | control surfaces, run-level labels, 100k, gravity, thrust pressure correction, TableAero/landing_zone/GUI tests, OpenRocket/RASAero | **NOT DONE** | unchanged |

---------------------------------------------------------------------------------------------------------------------------------

## A. Technical correctness

Verified again: the 6-DOF kernel, event normalisation (|q|-1 < 1e-12 test), ISA inversion, determinism, input validation; the baro-equivalent
altitude is correct in principle and its Prometheus value agrees with an independent hand estimate.

Open problems:
1. **(NEW, high for the validation, medium overall) The Prometheus `speed` channel is vertical velocity, not total speed.** See N6. The
   whole "inferred total speed" paragraph in `docs/validation.md`, the YAML comment ("AltOS speed = total speed"), the `STANDARD_CHANNELS`
   comment and the `compare.py` note are contradicted by the data.
2. The model is low in coast velocity for Prometheus: with a like-for-like (vertical vs vertical) comparison the ascent velocity bias is
   -9.4 m/s (max |e| 12.8 m/s, RMSE 10.7); the real flight decelerated more slowly (apogee 1.6 s later, 4.9 % higher in barometric terms).
   Integrating the real vertical velocity gives 4175 m by apogee time against 3904 m barometric height, i.e. real geometric apogee is
   probably ~4.1-4.2 km, so the geometric shortfall of the simulation is ~6-7 %, and the published geometric "+0.03 %" (v1.1.0) was a
   coincidence of two errors. Whatever the cause (drag, impulse, temperature profile), this is not "validated to a few percent".
3. Gravity bias (GM/R^2 mean radius; ~+0.1..0.25 %), no pressure-dependent thrust (1-2 % on the M1520 climbing to 3.9 km), Cd not
   AoA-dependent, Barrowman CP Mach-independent, turbulence altitude-independent, `thrust_scale` leaves propellant mass fixed, apogee event
   not guarded against a pre-burnout trigger. All unchanged, all unquantified in validation.
4. Alignment: both the altitude threshold (15 m) and the Prometheus 50 m/s speed alignment remove the launch offset only; fine. The
   Prometheus alignment channel was chosen after seeing the altitude lag (documented).
5. The temperature offsets (-8/-12/+15 K) act as hidden free parameters worth 3-6 points of apogee error (documented honestly now).

## B. Software quality

Strengths: clean package layout, typed config with key-naming errors, ruff/mypy clean, 160 deterministic analytic tests, atomic
checkpointing, reproducible recorded validation (bit-identical metrics on re-run), golden-output test, run id now includes inputs.

Weaknesses:
* New default validation path (`barometric`) has no direct unit test (known-pressure check) and the self-consistency test no longer
  exercises it.
* Regression tolerances for real flights are loose (6 %/16 %/7 %); the golden test protects only the example vehicle.
* `spec_hash` bare `except`; only the base motor hashed; no git commit/dependency versions in metadata.
* No tests for the rejection alarm/histogram, `TableAero`, `landing_zone`, plotting, benchmark, the whole `ui/` package, HIL timeouts.
* Pure-Python hot loop (~1 sim/s for the 29 mm vehicle at fidelity 3 under load; 0.9 sims/s in my 12-run batch with other load running);
  `_finalize` still O(N) in memory; nothing above 1000 runs ever executed.
* Docs drift remains (item N5), README example block not machine generated.

## C. Requirements 1..82

| # | verdict | one line |
|---|---|---|
| 1 | PARTIAL | full pipeline, but no guidance law / landing targeting beyond statistics |
| 2 | PARTIAL | TVC only; no aerodynamic control surfaces (README admits it) |
| 3 | PASS | specific initial assessment |
| 4 | PASS | physics/data first |
| 5 | PARTIAL | configurable dt, event truncation, two-rate; no adaptive step |
| 6 | PASS | explicit state incl. angular accel, mass split |
| 7 | PASS | frames.md + tests |
| 8 | PASS | constant and GM/(R+h)^2 (bias noted) |
| 9 | PASS | ISA/exponential/table/offsets |
| 10 | PASS | relative-velocity wind; simple turbulence |
| 11 | PARTIAL | `.eng`/CSV, no pressure-dependent thrust |
| 12 | PASS | m(t), CG/I/CP history |
| 13 | PASS | geometry dataclasses OK for later .ork |
| 14 | PARTIAL | Mach/Re build-up; no AoA-dependent Cd, single diameter, TableAero untested |
| 15 | PASS | Mach logged every step |
| 16 | PASS | alpha/beta from relative velocity |
| 17 | PASS | static margin, warnings |
| 18 | PASS | quaternion 6-DOF |
| 19 | PASS | gimbal limit/rate/lag/delay/rate/misalignment |
| 20 | PARTIAL | clean plug-in (python, HIL tested); only null/schedule/PD built in |
| 21 | PASS | full sensor suite with imperfections |
| 22 | PARTIAL | linear latency-compensated KF for pos/vel; attitude dead-reckoned; honest, not an EKF |
| 23 | PASS | logged phase machine |
| 24 | PASS | sensor-based launch detection |
| 25 | PARTIAL | impact/terrain; stops at contact; `landing_zone` untested |
| 26 | PARTIAL | broad randomisation with ids/seeds; independent parameters; limited initial-condition randomisation |
| 27 | PASS | headless config->batch->quality->export works (12-run batch executed); 100k not demonstrated |
| 28 | PASS | CSV/JSON/Parquet/NPZ |
| 29 | PASS | versioned schema in every manifest |
| 30 | PARTIAL | per-step targets only; no landing location/miss/corrections/control labels |
| 31 | PARTIAL | YAML feature/label lists; two derived features; no transforms |
| 32 | PASS | windowed datasets, stride, float32 NPZ |
| 33 | PARTIAL | three public flights, secondary parameters, no measured atmosphere/wind |
| 34 | PARTIAL | baro-equivalent apogee now like-for-like; but Prometheus velocity channel misidentified (N6) |
| 35 | PASS | RMSE/MAE/max/bias/NRMSE/%/timing |
| 36 | PARTIAL | justification given; v1.1.0 structure chosen on Prometheus (in-sample); temperature offsets are free inputs |
| 37 | PARTIAL | loop documented, LOO refuses calibration; alignment channel chosen post hoc; only a drag multiplier tried |
| 38 | PASS | this series |
| 39 | PASS | this review |
| 40 | PASS | three critic rounds done (6.5 -> 6.8 -> 7.0), scores not inflated, decreases investigated |
| 41 | PASS | requirement-based |
| 42 | PASS | analytic tests, golden test |
| 43 | PASS | convergence study |
| 44 | PARTIAL | multiprocessing/checkpoint/shards; modest efficiency; finalise O(N) |
| 45 | PASS | all commands exist and were exercised |
| 46 | PASS | YAML/JSON/TOML |
| 47 | PARTIAL | seed, versions, config, motor sha now in id; no git commit/dependency versions; spec_hash covers only base motor |
| 48 | PASS | four version constants, manual bump enforced by golden test; (loose real-flight tolerances) |
| 49 | PASS | required variables/selection/scrubbing/events (GUI untested) |
| 50 | PASS | 3-D tab |
| 51 | PASS | rich summary |
| 52 | PASS | manifest/runs.parquet browse |
| 53 | PASS | clean errors; systematic rejection alarmed (untested) |
| 54 | PASS | SI internally, units in names |
| 55 | PARTIAL | extensive docs; residual stale numbers (150 tests, README example, "few percent" in datasets.md, sensitivity "3-6 points", wrong total-speed claim) |
| 56 | PASS | layout matches |
| 57 | PASS | pipeline order enforced |
| 58 | PASS | randomised environments, no network |
| 59 | PASS | lock-step HIL bridge exists and is tested (undocumented in docs) |
| 60 | PASS | standard telemetry schema |
| 61 | PASS | declarative column-map importer |
| 62 | PARTIAL | tool/plots/metrics exist; Prometheus velocity panel compares sim total speed with real signed vertical velocity |
| 63 | PARTIAL | rounding to uncertainty is good; fixed-% from n=3; no star on apogee/velocity of the unvalidated 29 mm example (a footer line says so) |
| 64 | PASS | levels 0-6 (6 is a numerics preset, labelled) |
| 65 | PASS | fast mode recorded |
| 66 | PARTIAL | preset only; no evidence it is more accurate |
| 67 | PASS | gates work |
| 68 | PASS | complete manifest incl. rejection reasons |
| 69 | PASS | atomic chunks, spec-hash guard, resume tested |
| 70 | PARTIAL | parallel, measured, modest efficiency |
| 71 | PASS | benchmark command and recorded results (benchmark files unstamped) |
| 72 | PARTIAL | clean lint/types, 160 tests; UI/benchmark/plotting untested; unoptimised hot path |
| 73 | PASS | staged order evidenced |
| 74 | PARTIAL | few fake features; residual: fidelity-6 label, "AltOS total speed" assertion, README example rounding |
| 75 | PARTIAL | sources assessed, mostly secondary |
| 76 | PASS | equation/assumption/source/limitation per model |
| 77 | PARTIAL | hierarchy exists; "known calculations"/independent simulators thin |
| 78 | PARTIAL | RocketPy only (prescribed Cd); OpenRocket/RASAero not run; not re-run by me |
| 79 | PARTIAL | clean-install not tested; physics/data items verified; final score recorded here |
| 80 | PARTIAL | no standalone final engineering report |
| 81 | PASS | docs are candid, now including a self-corrected methodology section (but see N6) |
| 82 | PARTIAL | workflow followed through critic 3; nothing committed, no iteration history |

Tally: **PASS 51, PARTIAL 31, FAIL 0** (round 2: 50/32). Moves: 40 and 48 up; 34/55/62 stay PARTIAL for new reasons.

## D. Data capability

Good: deterministic per-run seeds independent of workers, split namespaces + hold-out, resumable shards, rich manifest with rejection
histogram/alarm, causal ZOH resampling of measured/estimated inputs, id now includes motor content, byte-reproducible, 12/12 accepted
in my run (64,082 samples).

Problems (unchanged from round 2 except where noted):
1. Labels are only per-step truth columns; no landing location, impact speed, miss distance, corrections or control targets, so a
   landing-guidance dataset still needs code changes.
2. `spec_hash` covers only `base_full["motor"]["file"]`, bare `except` hides a failed hash; git/dependency versions absent from metadata.
3. Discrete columns (`phase`, `launch_detected`, TVC command) are still linearly interpolated in windowed datasets; no warning when truth
   columns are used as deployable inputs.
4. Scale unproven: nothing above 1000 runs, `_finalize` loads all chunk JSON into Python lists.
5. Monte Carlo parameters independent (e.g. sea-level pressure and temperature offset drawn separately), few initial-condition variables.
6. Rejection alarm advisory only (exit code 0), untested.

## E. Validation honesty

Good: baro-equivalent comparison now like-for-like; explicit in-sample caveat; v1.0.0 vs v1.1.0 table with the geometric comparison kept;
LOO refuses calibration (RMS 8.26 -> 14.19 %); sensitivity table reproduced to the digit; README no longer says "few percent"; the NDRT
failure and 17 % landing-time error are reported; not claimed NASA-level.

Problems:
1. **N6: Prometheus speed channel is vertical velocity (signed).** The docs present "total speed" as an inference supported by an integral
   argument; the data refutes it (see repro). Consequences: reported max-velocity error +2.36 % should be -0.26 %; velocity bias +4.9 m/s
   should be -9.4 m/s (RMSE 11.1 -> 10.7 m/s); the velocity statistics therefore conceal a systematic coast-phase over-deceleration. The
   alignment (50 m/s crossing) and the apogee metrics are essentially unaffected (apogee error identical to 4 digits; time error -5.6 %).
   `uncertainty.py` max-velocity RMS (8.1/8.4 %) used the +2.36 %; with -0.26 % it becomes ~7.1 %.
2. The same integral (4175 m from vertical speed vs 3904 m barometric height) says the real geometric apogee is ~7 % above the barometric
   value, so the simulation's geometric apogee (3905 m) is ~6.5 % low, not +0.03 % right. The docs mention the 4175 figure but explain it with
   the wrong model.
3. "All three real altimeters are barometric" is stated as fact in validation.md, but only AltOS is documented as such; Bella Lui and NDRT
   sources only say "altimeter" (their errors +0.5 / +13.4 % would change by ~3-5 points if one was not barometric).
4. In-sample: Prometheus drove the v1.1.0 change; in the barometric frame v1.1.0 versus v1.0.0 is roughly neutral-to-slightly-better in RMS
   (my estimate by shifting the v1.0.0 geometric numbers by the baro offset: ~9.7 % -> 8.3 %), which the doc now hedges correctly but does not quantify.
5. `MODEL_UNCERTAINTY` percentages from n = 3, `apogee_m` etc. marked non-provisional although the demo vehicle is not in the validated class
   (mitigated by a printed footer); burnout-time +-2 % is the motor curve duration (one flight).
6. No input-uncertainty Monte Carlo of the validation flights (impulse +-5 % moves apogee +-10 %); validated envelope still subsonic
   20 kg-class coast; descent/wind/attitude/TVC/sensors/estimator not validated on real data; RocketPy cross-check has prescribed Cd.

---------------------------------------------------------------------------------------------------------------------------------

## Newly confirmed bugs / issues (with reproductions)

* **N6 (validation, medium-high): Prometheus `speed` is signed vertical velocity, documented as total speed.**
  Repro: load `validation_data/prometheus.yaml` via `load_flight_definition` + `load_telemetry`; `s = real["speed"]`: `s.y.min() = -37.83`,
  fraction of samples negative = 0.4526, value at the time of max altitude = 0.0, `np.trapezoid(s.y[t<=t_apogee], t) = 4174.7 m`.
  Simulation: max total speed 325.37, max `vel_z` 317.05 (real max 317.88). Replace the channel by `velocity_z` and `align_channel:
  velocity_z`: apogee error unchanged (-4.896 %), apogee-time -5.60 %, **max velocity -0.26 %** (published +2.36 %), velocity bias
  **-9.35 m/s** (published +4.86), RMSE 10.73 (published 11.10). Affected text: `docs/validation.md` (data table, Results note, "inferred" paragraph),
  `validation_data/prometheus.yaml`, `telemetry.py` STANDARD_CHANNELS comment, `compare.py` note and plot.
* **N7 (docs, low): residual drift**: "150 tests" (160); README example numbers not equal to program output (919 +/- 70, 198 +/- 16);
  `uncertainty.py` RMS comments (8.1 -> 8.4, 4.7 -> 4.45); "3-6 points" per +-10 K (really 1.8-3.4 per side); datasets.md "few percent".
* **N8 (hygiene, low): index not fully synced** (`AM` x3), `.gitignore` duplicate `*.egg-info/`, `spec_hash` bare except / base-motor-only,
  no unit test of `_baro_equivalent_altitude`, no test of the rejection alarm.

## Exact changes still needed (ordered)

1. Fix N6: in `prometheus.yaml` rename the channel to `velocity_z` (`align_channel: velocity_z`), delete every "total speed" statement
   (validation.md, `telemetry.py`, `compare.py`, `prometheus.yaml`), revert the plot special case, re-run the three flights, re-record
   `validation_results/physics_v1.1.0`, update max-velocity (-0.26 %, RMS ~7.1 %) and velocity bias in validation.md, `uncertainty.py`, and
   discuss the -9.4 m/s coast-velocity bias and the ~6.5 % geometric apogee shortfall as a model-vs-input question.
2. Add `tests/test_validation_cli.py::test_baro_equivalent_altitude_known_pressure` (e.g. pad 86444 Pa, a +15 K ISA column) and run the
   self-consistency test in barometric mode too; tighten `test_real_flight_regression` to +-0.5 points around the recorded numbers.
3. Replace the single-factor sensitivity table with a small Monte Carlo over temperature offset, thrust scale, mass, fin thickness and wind
   (the project already has `batch.py`) and report an apogee band per flight; state which altimeters are known to be barometric.
4. `data/batch.py::spec_hash`: hash every motor file reachable from `parameters`, remove the bare `except`; record git commit and numpy/
   scipy/pyarrow versions in `RunMetadata`/manifest; test it. Finish git hygiene (`git add -A`, de-dup `.gitignore`, commit in logical steps).
5. Docs: "160 tests", README example pasted from real output, `datasets.md` "few percent", "3-6 points" wording, document HIL, ZOH and event
   normalisation.
6. Functional gaps that cap the score: fin/control-surface forces or an explicit descoping of reqs 2/19; run-level labels (landing x/y,
   impact speed, miss distance vs `landing_zone`, time-to-apogee) in `data/spec.py`; ZOH for discrete columns; a recorded 100k fast-mode
   batch with peak RSS and a streaming `_finalize`; gravity anchored to g0 at the site; optional ambient-pressure thrust correction;
   guard the apogee event; tests for `TableAero`, `landing_zone`, rejection alarm, GUI smoke, plotting/benchmark.
7. A validation flight outside the current envelope (small mid-power or supersonic) with raw data, measured atmosphere and wind, and an
   OpenRocket/RASAero run on identical inputs.

Re-score guidance: item 1-5 done properly -> ~7.3-7.5; with item 6 (control surfaces or explicit descoping, run-level labels, 100k
demonstration) -> ~8; 9-10 requires item 7 plus primary-source measured data and TVC/attitude flight telemetry.
