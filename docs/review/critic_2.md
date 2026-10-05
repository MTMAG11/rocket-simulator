# Independent critic review 2

Reviewer: independent (did not build the project; also not the round-1 reviewer's author). Review date 2026-10-04.
Scope: `src/rocket_sim` (59 modules), `configs/`, `validation_data/`, `validation_results/`, `docs/`, `tests/`, git state.
Source files were not modified; scratch scripts were kept outside the repo.

What I actually ran: `pytest tests -q` (157 tests, all passed, none failed; ~13 min on this loaded machine), `ruff check src tests`
(clean), `mypy src/rocket_sim` (clean, 59 files), `rocketsim simulate configs/example_g80.yaml`, `rocketsim batch configs/batch_example.yaml
--runs 20` (new output dir), `rocketsim generate-dataset configs/dataset_example.yaml --runs 30`, a deliberately bad batch (alarm test),
`rocketsim validate` on all three flights (the numbers equal the recorded `validation_results/physics_v1.1.0/*`), and several ad-hoc analyses
(motor-content hash test, Prometheus atmosphere query, real-data integral consistency, barometric-equivalent altitude of the three
validation simulations). pandas/netCDF4 untouched. RocketPy cross-check was NOT re-run (read the recorded txt only).

## Score: 6.8 / 10  (round 1: 6.5, delta +0.3)

Why up: the round-1 data-corrupting bug is truly fixed (20/20 accepted, every row has |q|-1 < 1e-12, new test), the Prometheus atmosphere now
does what the config says (86444 Pa), motor contents are hashed into the batch spec hash and per-run metadata, committed stale artefacts are
gone from the working tree, the batch has a rejection alarm + reason histogram, measured/estimated dataset inputs are zero-order-hold
resampled, an HIL bridge and a python-controller test exist, tests grew 152 -> 157, lint/types still clean.

Why only +0.3 and not +0.5: (a) my own new, material finding E1 below (real altimeter altitude is barometric/ISA; the simulator's geometric
altitude is compared with it directly; with the sim's own +15 K / -12 K / -8 K temperature offsets applied consistently the apogee errors would be
-4.9 / +13.4 / +0.5 % rather than +0.0 / +8.7 / -2.3 %), which undermines the "few percent" headline more than round 1 realised;
(b) several claimed fixes are only partial (sim id/config hash still ignore motor contents; README headline and `uncertainty.py` not reworded;
stale numbers now contradict each other in docs); (c) nothing in the large round-1 gaps (control surfaces, guidance law, run-level labels,
100k-scale proof, golden regression test, gravity bias, thrust pressure correction) was touched. The score is not lower than round 1 because
the fixed items were real defects and were fixed correctly; I did not find a regression in physics.

---------------------------------------------------------------------------------------------------------------------------------

## 1. Status of each round-1 item (verified by running code)

| # | claim | verdict | evidence |
|---|---|---|---|
| 1 | event-state quaternion bug (4/20 rejections) | **FIXED** | `simulator.py` l.478 `y_new = dyn.post_step(ye)`. `rocketsim batch configs/batch_example.yaml --runs 20`: "20/20 accepted, 0 rejected" (round 1: 16/20). `example_g80` max ||q|-1| = 2.2e-16 over all rows including landing. Test `test_every_recorded_row_has_unit_quaternion_including_event_rows` and `test_example_batch_rejects_nothing...` exist. Caveat: the test runs only 6 runs; the previously failing runs 1,2 are within them, OK. |
| 2 | Prometheus atmosphere pressure | **FIXED** | `ISAAtmosphere(+15 K, 101470.2 Pa).at(1401 m).pressure = 86443.98 Pa` (measured 86444). Prometheus apogee error moved -0.27 % -> +0.03 %. Doc text updated. |
| 3 | hash ignores motor contents; dataset id depends on output_dir | **PARTLY** | Batch: `spec_hash` now pops `output_dir`/`workers` and includes motor SHA-256 (OK; unit-tested only for output_dir, NOT for motor content). Per-run: `RunMetadata.input_files` holds the motor sha. BUT `config_hash` and therefore `Simulation.simulation_id` are unchanged: I replaced the .eng content (thrust x1.1) and got the identical id `3ebb97964a6fa970-s1` before and after; only `input_files` differs. `spec_hash` swallows any exception while hashing (`except Exception: msha = ""`). Git commit / numpy / scipy / pyarrow versions still not recorded. `input_files` is not in the dataset runs table. Only the motor is hashed (the only external file type today). |
| 4 | stale committed output artefacts removed | **MOSTLY** | Working tree `output/` now holds only benchmarks, two GUI PNGs and two txt logs; `output/` is in `.gitignore`. But the index is a mess: `git status` shows `AD` for ~30 stale files (staged adds that no longer exist on disk), `AM` for latest.json, and the whole new package is still uncommitted (HEAD is "updates" from before the rewrite; 115 files changed vs HEAD). A plain `git commit` would commit the stale mc-demo/validation artefacts from the index. `.gitignore` now lists `*.egg-info/` twice. `output/benchmarks/latest.json` has no timestamp/physics-version/git stamp. |
| 5 | Prometheus validation channel inconsistencies | **PARTLY** | Done: speed channel is compared with simulated *total* speed, alignment channel option (`align_channel: speed`, 50 m/s), ascent altitude bias reported separately. NOT resolved: (i) the claim "AltOS speed = total speed" is asserted, not shown; (ii) integrating the real speed channel from t=0 gives 4175 m by apogee time (4111 m even after x sin 80 deg) vs real height 3904 m, i.e. a 5-7 % inconsistency that neither the "total speed" nor the "baro lag" story explains (see E1/E2 for the likely cause); (iii) the saved comparison PNG still plots the simulated *vertical* velocity against the real *total* speed (`compare.py::plot_comparison` uses `sim["velocity_z"]` unconditionally) so the picture compares unlike quantities while the metrics do not; (iv) the choice of alignment channel was made after seeing the altitude lag, and the same speed channel is then used to score velocity RMSE: a post-hoc degree of freedom. |
| 6 | reworded validation claims (in-sample caveat) | **PARTLY** | `docs/validation.md` now has an explicit in-sample caveat paragraph and a v1.0.0 vs v1.1.0 table: good. NOT reworded: README headline "validated against three amateur flights to a few percent in apogee" (errors are -2.3/+8.7/+0.0, RMS 5.2 %); README example prints a 29 mm model rocket as `919 +/- 60 m` with no star (`apogee_m` is `provisional=False`); `uncertainty.py` comment still quotes stale/incorrect figures ("max velocity ... +0.0 %, RMS 5.9 %", "apogee time ... -5.3 %": current are +2.36 % and -5.56 %); `burnout_time_s` +-2 % still marked validated although it is the motor curve duration; MODEL_UNCERTAINTY still fixed percentages from n = 3. |
| 7 | batch rejection-rate alarm + reason histogram | **FIXED (untested)** | A 3-run all-reject batch printed `WARNING: HIGH REJECTION RATE ...` and `rejection_reasons {"run status 'no_liftoff'": 3}` in the manifest. No test asserts either (`grep` finds none). The alarm is advisory only (exit code 0, dataset still written). Histogram key is the first `;` segment truncated to 80 chars (fine). |
| 8 | causal (ZOH) resampling of meas_/est_ inputs | **FIXED for the stated scope** | `dataset.py::resample` + test. Gaps: the causal flag is derived from a column-name prefix, so `phase`, `launch_detected`, `tvc_cmd_*`, and the derived `time_since_launch_detected` (all discrete or step-like) are still linearly interpolated; hold is on the recorded (step) rows, which is fine because sensor columns are held per row. Not documented in `docs/datasets.md`. |
| 9 | HIL bridge | **FIXED as lock-step plumbing, over-sold in places** | `control/hil.py` (69 lines, JSON-lines over any send/recv transport), tested with a loopback fake. Limitations: one request per *control tick*, so a 400 Hz IMU is decimated to the controller rate (a real flight computer's estimator would not see every sample); no timeouts/pacing; not in `docs/usage.md` or any doc (grep for HIL in docs: nothing). Requirement 59 only needs "possible", so PASS. |
| 10 | python-controller test | **FIXED** | `test_custom_python_controller_plugs_in`, `test_python_controller_from_yaml_dotted_path`. Note: importing an arbitrary dotted class from a YAML spec is code execution from config; batch specs from third parties are therefore not safe (undocumented). |

Round-1 items that were NOT claimed and remain open: gravity +0.1..0.25 % bias (`InverseSquareGravity` still GM/R^2 mean radius; 9.8202 at sea level);
no thrust pressure correction; fidelity 6 identical to 5 except numerics; no aerodynamic control surfaces; no guidance law; labels limited to
per-step columns; `_finalize` still O(N) lists in the parent; no golden/physics-checksum test; silent `PROJECT_ROOT`/`cwd` path fallback; no tests
for `TableAero`, `landing_zone`, plotting, benchmark, the GUI; apogee event still unguarded; motor `thrust_scale` still leaves propellant mass fixed.

---------------------------------------------------------------------------------------------------------------------------------

## A. Technical correctness

Verified again / still good: quaternion kinematics, Euler equations, TVC moment signs, wind only through relative velocity, ISA layering,
RK4 + bisection-with-reintegration event location (now followed by the same invariants as ordinary steps), determinism, input validation.
The test suite is genuinely analytic (Tsiolkovsky, quadratic drag, torque-free precession, ISA table).

Problems (new ones marked NEW):

1. **NEW (high): the sim is compared to barometric altitude without modelling the barometer.** `validation.md` itself states AltOS height is
   barometric (and the other two altimeters are altimeter-derived). A barometric altimeter reports ISA pressure-altitude (referenced to the pad),
   which differs from geometric height by roughly (T_actual/T_ISA - 1) (about +5 % on a +15 K day, -4 % at -12 K). The validation compares
   `rec.col("altitude")` (geometric, from a sim with temperature offsets of +15 / -12 / -8 K that the validation docs call "climatological
   estimates, not tuned") with that channel. I recomputed the same three simulations in the barometer's frame (sim pressure -> ISA pressure
   altitude minus pad value, at apogee):

   | flight | T offset | geometric apogee err (as published) | barometer-equivalent apogee err |
   |---|---|---|---|
   | Bella Lui | -8 K | -2.29 % | **+0.50 %** |
   | NDRT 2020 | -12 K | +8.71 % | **+13.43 %** |
   | Prometheus | +15 K | +0.03 % | **-4.90 %** |

   RMS over the three flights goes from 5.2 % to 9.2 %. This also gives the likely explanation of the Prometheus mystery: real speed integrated
   gives ~4.1-4.2 km, real baro height says 3.90 km: a ~5-7 % gap equal to the +15 K hot-day pressure-altitude compression. Caveat: whether each
   team's device applies a plain ISA conversion is an assumption (true for AltOS; unknown for the others), but the project's own docs say the data
   is barometric and the offsets are modelling choices, so the comparison is not like-for-like and the offsets act as hidden fit parameters
   (+/- 5 % in apogee is ~ the same magnitude as the headline errors). The simulator already has the machinery (`meas_baro_altitude`, ISA
   `pressure_to_altitude`) to do this properly.
2. Gravity bias (+0.1..0.25 %), unchanged and unquantified in validation (Prometheus: g used 9.816 vs ~9.79 real).
3. No thrust altitude/pressure correction (1-2 % on the 98 mm M1520 to 3.9 km; apogee sensitivity ~2x). Unquantified.
4. Cd not AoA dependent; Barrowman CP/CNa Mach-independent; max validated Mach 0.95; fidelity 6 label.
5. Turbulence time-indexed, altitude-independent, 900 s pre-generated; level-2 weathercock shortcut.
6. `thrust_scale` Monte-Carlo leaves propellant mass fixed (changes Isp).
7. Apogee = first vz zero crossing after liftoff; no guard against a pre-burnout "apogee" (arms chutes). Not reproduced as a failure in the shipped configs, but unprotected.
8. NEW (low): physics output changed (event states are now normalised and the Prometheus config changed) but `PHYSICS_VERSION` stays 1.1.0; this is within the stated "no silent physics changes" rule only because the effect is 1e-6, but there is no mechanism (golden test) that would catch a bigger accidental change.
9. NEW (low): `physics.md` change log still says v1.1.0 gives Prometheus "-0.3 %"; validation.md and the recorded JSON say +0.03 %.

## B. Software quality

Strengths: package separation, typed config with key-naming errors, mypy/ruff clean, atomic chunk writes, truth->sensors->estimator->controller
pipeline enforced, good docstrings, reproducible validation recording.

Weaknesses:
* Repo hygiene is poor for a "final-looking" state: the entire rewrite is still only staged/uncommitted (no history of build/critic iterations,
  contrary to the workflow in spec 82), the index contains deleted-from-disk stale outputs (see 1.4).
* Regression guard still loose (`test_real_flight_regression`: 6 / 12 / 5 % apogee, 4 / 10 / 5 % NRMSE; marked slow) and there is still no golden
  checksum test keyed to `PHYSICS_VERSION`. The new Prometheus alignment/speed handling has no test of its own (a regression to vertical-vs-total
  speed would pass).
* New code with no direct test: rejection alarm/histogram, motor-content hash effect on `spec_hash`, ZOH on non-prefixed columns, HIL timeouts.
* Untested: `TableAero`, `landing_zone`, plotting, benchmark, the entire `ui/` package.
* `_finalize` loads every chunk JSON into Python lists and runs O(N) loops; nothing run above 1000 runs. Hot loop is pure Python (~9k steps/s).
* `resolve_path` falls back silently to `PROJECT_ROOT` and `cwd`; the resolved path is not recorded (only the config string and content hash are).
* `spec_hash` bare `except Exception: msha = ""` hides a missing/unreadable motor in the hash (the run then fails later, but the hash is wrong).
* Docs drift: "150 tests" (157), README example `Max velocity 190 +/- 10` (current run: 198 +/- 12), module docstring of `compare.py` still says alignment is always the 15 m altitude crossing, validation.md sensitivity table is explicitly stale ("measured before the correction"), HIL / causal resampling / event normalisation undocumented.

## C. Requirements checklist (1-82)

| # | verdict | justification |
|---|---|---|
| 1 | PARTIAL | full pipeline exists; no landing guidance law, no landing-zone targeting beyond a statistic |
| 2 | PARTIAL | TVC modelled; no fin/aerodynamic-surface control (`Command` has tvc_y/tvc_z only; README admits it) |
| 3 | PASS | `initial_assessment.md` specific |
| 4 | PASS | physics/data first |
| 5 | PARTIAL | configurable dt, event truncation, two-rate step; no adaptive stepping |
| 6 | PASS | full explicit state incl. angular accel and mass split |
| 7 | PASS | frames.md, tested |
| 8 | PASS | constant and GM/(R+h)^2 (bias noted in A2) |
| 9 | PASS | ISA/exponential/table/offsets |
| 10 | PASS | via relative velocity; turbulence simplified |
| 11 | PARTIAL | `.eng` + CSV, linear interpolation, no pressure-dependent thrust |
| 12 | PASS | m(t), CG/I/CP vs time |
| 13 | PASS | geometry dataclasses suitable for .ork later |
| 14 | PARTIAL | Mach/Re build-up; no AoA-dependent Cd, single-diameter body, weak transonic; TableAero untested |
| 15 | PASS | `mach` logged every step |
| 16 | PASS | alpha/beta from body-frame relative velocity |
| 17 | PASS | static margin logged, warnings |
| 18 | PASS | 6-DOF quaternion dynamics |
| 19 | PASS | gimbal limit/rate/lag/delay/update rate/misalignment |
| 20 | PARTIAL | clean plug-in (python loader + HIL, now tested); only null, schedule, PD attitude controllers built in |
| 21 | PASS | accel/gyro/baro/GPS/mag with noise, bias, walk, scale, quantisation, saturation, rate, latency |
| 22 | PARTIAL | latency-compensated linear KF for pos/vel; attitude is gyro dead-reckoning; honest, not an EKF |
| 23 | PASS | phase machine with logged transitions |
| 24 | PASS | sensor-based launch detector |
| 25 | PARTIAL | impact detection and terrain; stops at first contact; `landing_zone` untested |
| 26 | PARTIAL | broad randomisation with ids/seeds; independent parameters; thrust-scale/propellant inconsistency; no initial-condition randomisation beyond angle |
| 27 | PASS | headless config->batch->quality->export works (20-run and 30-run runs executed); 100k not demonstrated |
| 28 | PASS | CSV, JSON, Parquet, NPZ |
| 29 | PASS | versioned schema 1.0.0 in every manifest |
| 30 | PARTIAL | per-step targets only; no landing location / miss distance / correction / control-output labels |
| 31 | PARTIAL | YAML feature/label lists; two derived features; no transforms/normalisation/augmentation |
| 32 | PASS | windowed history->future/trajectory, stride, float32 NPZ |
| 33 | PARTIAL | three public flights, secondary parameters, filtered/baro data, no measured atmosphere/wind |
| 34 | PARTIAL | metrics implemented, but altitude comparison is not like-for-like with a barometric channel (A1) |
| 35 | PASS | RMSE/MAE/max/bias/NRMSE/%/timing |
| 36 | PARTIAL | physical justification given; v1.1.0 structure selected on Prometheus; temperature offsets and fin thickness are free inputs |
| 37 | PARTIAL | loop documented, LOO refuses calibration; only a global drag multiplier tested; alignment channel chosen post hoc |
| 38 | PASS | this review series |
| 39 | PASS | this review |
| 40 | PARTIAL | two of three iterations done |
| 41 | PASS | requirement-based |
| 42 | PASS | analytic tests are real |
| 43 | PASS | convergence study reproduced in round 1 |
| 44 | PARTIAL | headless, multiprocessing, shards, checkpoint; 40-50 % parallel efficiency; finalise O(N) in memory |
| 45 | PASS | all commands present and exercised |
| 46 | PASS | YAML/JSON/TOML |
| 47 | PARTIAL | seed, versions, config, motor sha (per run + spec hash) stored; simulation id/config hash ignore motor contents; no git commit or dependency versions |
| 48 | PARTIAL | four version constants; physics bump is manual; loose regression tolerances; no golden test |
| 49 | PASS | required variables, selection, scrubbing, events (GUI untested) |
| 50 | PASS | 3-D tab |
| 51 | PASS | rich summary |
| 52 | PASS | manifest/runs.parquet browse |
| 53 | PASS | useful errors; systematic rejection now alarmed |
| 54 | PASS | SI internally, units in names |
| 55 | PARTIAL | extensive docs, but stale/contradictory numbers (physics.md -0.3 % vs +0.03 %, README 190 vs 198 m/s, "150 tests", stale sensitivity table), HIL/ZOH/event-normalisation undocumented |
| 56 | PASS | layout matches |
| 57 | PASS | pipeline order enforced |
| 58 | PASS | randomised environments, no network |
| 59 | PASS | lock-step HIL bridge exists and is tested ("must be possible") |
| 60 | PASS | standard telemetry schema |
| 61 | PASS | declarative column-map importer |
| 62 | PARTIAL | tool and plots exist; the Prometheus plot compares sim vertical velocity with real total speed |
| 63 | PARTIAL | rounding to uncertainty is good in principle; fixed-% values from n = 3 presented as +/-, no star on unvalidated vehicles, stale justification text |
| 64 | PASS | levels 0-6 table (6 is a numerics preset, labelled so) |
| 65 | PASS | fast mode recorded |
| 66 | PARTIAL | preset only; no evidence it is more accurate |
| 67 | PASS | gates work; false-rejection bug gone |
| 68 | PASS | manifest complete, now with rejection reasons |
| 69 | PASS | per-chunk atomic markers, spec-hash guard, resume tested |
| 70 | PARTIAL | parallel with measured, modest efficiency |
| 71 | PASS | benchmark command and recorded results (1000-run, dataset); benchmark files carry no physics/commit stamp |
| 72 | PARTIAL | clean lint/types, 157 tests; UI/benchmark/plotting untested; un-optimised hot path |
| 73 | PASS | staged order evidenced |
| 74 | PARTIAL | few fake features; README headline "a few percent" and the fidelity-6 label are residual overclaims |
| 75 | PARTIAL | sources assessed, mostly secondary |
| 76 | PASS | equation/assumption/source/limitation per model |
| 77 | PARTIAL | hierarchy exists; "known calculations" and multi-flight generalisation weak |
| 78 | PARTIAL | RocketPy only, with prescribed Cd (dynamics, not drag); static-margin agreement asserted not tabulated (recorded txt shows sim CP in m, RocketPy in cal); OpenRocket/RASAero not run; not re-run by me |
| 79 | PARTIAL | many items verified by me; 3 iterations and final score pending; clean install not tested |
| 80 | PARTIAL | no final engineering report yet |
| 81 | PASS | docs are candid (but see 74) |
| 82 | PARTIAL | workflow followed to critic 2 |

Tally: **PASS 50, PARTIAL 32, FAIL 0** (round 1: 51 / 31). Net movement: 59 (HIL), 67 (gate), 53 (alarm), 28 up; 34, 55, 62, 74 down (new findings). The checklist is lenient: no FAIL, but #2/#20/#30 under-state the original intent (control surfaces, guidance, run-level labels).

## D. Data capability

Good: per-run deterministic seeds independent of worker count/chunking, split namespaces + hold-out distributions, resumable shards, rich manifest
(now with rejection histogram + alarm), dataset id independent of output dir/workers, motor content in spec hash, causal measured inputs, byte-reproducible. I
ran a 20-run telemetry batch (106,695 rows, 20/20 accepted) and a 30-run windowed set (4,125 samples, 0 rejected, 1.4 sims/s while another process
was using the CPU).

Problems:
1. Labels: still only per-step truth columns. No landing location, impact speed, miss distance, corrections, control targets. A guidance dataset needs code changes.
2. `simulation_id`/`config_hash` do not include external-file content (reproduced, see 1.3); only the dataset-level hash does. Anyone referencing a run by id after editing a motor file silently aliases two different flights.
3. Discrete/step columns (`phase`, `launch_detected`, TVC command) are linearly interpolated; no guard or warning when truth columns are used as inputs ("deployable inputs" is the user's problem).
4. Scale unproven: `_finalize` O(N) lists; leakage check hashes every row in Python; skipped disjointness check above 200k runs; nothing beyond 1000 runs executed.
5. Monte Carlo parameters independent; no correlations; no initial-condition randomisation beyond launch angle; the example batch draws `sea_level_pressure_pa` and `temperature_offset_k` independently.
6. The rejection alarm is advisory: a 100 % reject batch exits 0.
7. HIL decimation to controller rate (see 1.9) limits realism of any hardware test.

## E. Validation

Good: real telemetry, quantitative metrics, bit-reproducible (I re-ran all three; identical to `validation_results/physics_v1.1.0`), failures
reported (NDRT +8.7 %, landing time +16.7 %), LOO refuses calibration, explicit in-sample caveat in validation.md.

Problems and overclaims:
1. **Barometric vs geometric altitude (NEW, most important).** See A1: the three published apogee errors are not like-for-like; the barometer-equivalent errors are +0.5 / +13.4 / -4.9 %. The headline "few percent" and the sensitivity discussion ("NDRT is not evidence of a model defect") are both conditional on this. The hot-day offset (+15 K) also happens to produce the best match, i.e. the free climatological offsets are doing 5 % worth of work. The Prometheus real-speed-vs-height inconsistency (my integral: 4175 m vs 3904 m) is exactly what a pressure-altitude artefact predicts and is still described in the docs as "baro lag, not proven".
2. The "total speed" assertion for AltOS is untested; the validation uses a channel (speed) both to align time and to score velocity, with the choice of that channel made after seeing the altitude channel disagree. Apogee-time error (-5.6 %) is the visible residual. The plot bug (E/B above) hides the total-vs-vertical difference visually.
3. In-sample: caveat now present in validation.md; README still states the headline without it. Only Bella Lui and NDRT are even partly out-of-sample.
4. `MODEL_UNCERTAINTY` fixed percentages from n = 3 with stale comments; applied to a never-validated 29 mm rocket in the README without a star. Burnout time "validated" is the motor's own duration.
5. Validation cannot discriminate drag-model quality (+-5 % impulse -> +-10 % apogee) and there is still no input-uncertainty Monte Carlo of the validation flights, although the project has the machinery.
6. RocketPy cross-check uses a prescribed Cd = 0.45: validates dynamics, atmosphere, gravity and integrator, not drag/CP/damping; recorded txt shows no sim static margin in calibres; I did not re-run it.
7. Validated envelope remains subsonic 20 kg-class coast rockets; descent (assumed chutes, -4 .. +17 %), wind, attitude, TVC, small vehicles and supersonic are all unvalidated, i.e. the intended GNC use is outside it.
8. NRMSE "of range" is dominated by the long parachute descent.

---------------------------------------------------------------------------------------------------------------------------------

## Newly confirmed bugs / issues (with reproductions)

* **N1 (validation methodology, high):** barometric-vs-geometric altitude. Repro: load `validation_data/{prometheus,ndrt_2020,bella_lui}_sim.yaml`, `run_simulation(cfg, seed=0)`, `h = [ISAAtmosphere().pressure_to_altitude(p) for p in rec.col("pressure")]`; `h[argmax(altitude)] - h[0]` = 3712.6 / 1497.7 / 461.2 m vs published geometric apogee 3905.0 / 1435.3 / 448.4 and real 3903.8 / 1320.4 / 459.0. Real-data consistency: integrate the AltOS `speed` column (col 11) from t = 0: 4174.6 m at apogee time vs col 10 height 3903.8 m.
* **N2 (plot, low):** `validation/compare.py::plot_comparison` (velocity panel) always draws `sim["velocity_z"]` even when the real channel is `speed` (Prometheus), so `validation_results/physics_v1.1.0/prometheus_2022.png` compares vertical sim velocity with total real speed; metrics use total vs total.
* **N3 (reproducibility, medium):** `Simulation.simulation_id` / `config_hash` unchanged when the motor `.eng` content changes (id `3ebb97964a6fa970-s1` for both a stock and a thrust x1.1 motor file); only `meta.input_files` and the batch spec hash differ.
* **N4 (hygiene, low):** `git status` shows ~30 `AD` entries; committing the index commits stale output artefacts; rewrite not committed; `.gitignore` duplicate line.
* **N5 (docs, low):** contradictory numbers (physics.md "-0.3 %" vs +0.03 %; README 190 vs 198 m/s; stale sensitivity table; `uncertainty.py` comment).
* Not bugs but unprotected: apogee event before burnout; `spec_hash` bare except; python-controller import from YAML executes arbitrary code.

## Strengths (summary)
Correct, convergent 6-DOF kernel; event location now consistent; honest, specific docs with limitations and failure admissions; truth/sensor/estimator/
controller/actuator separation incl. HIL plumbing; a real, deterministic, resumable data system with rejection accounting; analytic tests; clean lint/types;
validation recorded and reproducible by version.

## Weaknesses (summary)
Validation compares unlike altitudes (N1) and leans on free atmosphere offsets and an in-sample structural change; headline claims in README/uncertainty not
reworded; run ids still ignore motor contents; no control surfaces / guidance / run-level labels; no 100k demonstration; no golden regression test; gravity bias and
no pressure thrust correction; thin test coverage of UI/benchmark/plotting/TableAero; uncommitted rewrite.

---------------------------------------------------------------------------------------------------------------------------------

## Ordered list of exact changes

1. `validation/telemetry.py::telemetry_from_record` + `validation/compare.py::compare_flight` + `validation_data/*.yaml`: add a per-flight option `altitude_reference: barometric` (default for AltOS/altimeter data) that converts the simulated altitude through the sim's own pressure (`rec.col("pressure")` -> `ISAAtmosphere().pressure_to_altitude` minus pad) before comparing; report both geometric and barometric errors in the table; re-run all three flights, re-derive the headline, and update `docs/validation.md`, `README.md`, `uncertainty.py`, `loo_drag_scale.txt`, `physics.md` change log. Add a test asserting the conversion at a known pressure.
2. `docs/validation.md`, `validation_data/prometheus.yaml`: replace "AltOS speed is total speed" with what is shown by the data (integral check above), state that the +15 K offset trades directly against the barometric conversion, and bound the three temperature offsets with a small Monte Carlo (`batch.py` over offset +-8 K, thrust scale 3 %, mass, fin thickness, wind) to give an honest input-uncertainty band for each apogee.
3. `config/loader.py::config_hash` / `simulation/simulator.py::Simulation.__init__`: fold `self.input_files` (motor sha) into `simulation_id` (or add a short `inputs_hash` suffix); extend `spec_hash` to hash every `motor.file` choice in `parameters`; remove the bare `except Exception` in `data/batch.py::spec_hash`; record git commit (if available), numpy/scipy/pyarrow versions in `RunMetadata` and the manifest; put `input_files` into `runs.parquet`. Add `tests/test_data.py::test_spec_hash_changes_with_motor_file_content` and a test that simulation ids differ.
4. `README.md` (status line, example block), `uncertainty.py`: reword to "apogee error -2/+9/0 % on three amateur flights (one used to select the drag model; barometric caveat)"; mark `apogee_m`, `max_velocity_ms`, `apogee_time_s`, `burnout_time_s` as provisional or print `+/-` only for vehicles in the validated envelope (mass > ~10 kg, Mach < 0.95); fix the stale numbers in the comments; refresh the README example (198 +/- 12 m/s).
5. `validation/compare.py::plot_comparison`: plot `sim[rk]` (total speed) when the real channel is `speed`; regenerate PNGs under `validation_results/physics_v1.1.0`.
6. Add `tests/test_golden.py`: tight (1e-9 relative) checksum of a fixed 3-DOF and 6-DOF run (apogee, burnout velocity, impact position) keyed to `PHYSICS_VERSION`; tighten `test_real_flight_regression` to +-0.5 percentage points around the recorded numbers; add a test that the Prometheus config yields 86444 Pa at the site.
7. Repo hygiene: `git add -A` so the index matches the working tree (drop `output/mc-demo`, `output/run1`, `output/validation` from the index), commit the package in logical commits, de-duplicate `.gitignore`, stamp `output/benchmarks/latest.json` with commit/physics version/date.
8. Control and labels: implement fin-deflection force/moment (`control/actuators.py::Command`, `vehicle/aero.py`, `physics/dynamics.py` hook) or explicitly demote requirements 2/19 in README; add run-level and derived labels (landing x/y, impact speed, miss distance vs `landing_zone`, time-to-apogee, control outputs) in `data/spec.py`/`data/dataset.py`; ZOH for discrete columns (`phase`, `launch_detected`, `tvc_*`) and a warning when truth columns are listed as inputs.
9. Scale: stream `_finalize` (per-chunk `ParquetWriter` append, per-chunk leakage hashing), run a 100k FAST batch in `benchmark` and record peak RSS; document the HIL decimation and add timeout handling in `control/hil.py`; document code-execution risk of `controller.type: python`.
10. Physics: gravity anchored to g0 at the site (or latitude/centrifugal-aware) in `environment/gravity.py`; optional ambient-pressure thrust correction in `motor/motor.py`; guard `apogee` event in `simulation/simulator.py` (require burnout or altitude/time threshold); make `thrust_scale` optionally scale propellant mass; differentiate or relabel fidelity 6 (`simulation/builder.py`, `docs/physics.md`).
11. Tests: add rejection alarm/histogram test (`data/batch.py::_finalize`), `TableAero`, `landing_zone`, offscreen-GUI smoke test, plotting/benchmark smoke tests, planar-terrain dynamics.
12. Docs: document event-state normalisation, ZOH resampling, HIL, input_files in `docs/physics.md`, `docs/datasets.md`, `docs/usage.md`; fix "150 tests", physics.md "-0.3 %", stale sensitivity table (re-measure after item 1); `compare.py` module docstring on alignment channel.
13. Validate the actual target: add at least one small mid-power or supersonic flight with raw (unfiltered) data and measured atmosphere/wind; run OpenRocket/RASAero on the same inputs; tabulate static margin (cal) from the RocketPy cross-check.

Re-score guidance: items 1-7 done properly (especially 1 and 2, whatever numbers they produce) would justify ~7.3; adding run-level labels, control surfaces or an explicit demotion, a demonstrated 100k run and a held-out small/supersonic flight would justify ~7.8-8. 9-10 still requires primary-source measured atmosphere/wind, weighed vehicles, raw IMU/attitude/TVC telemetry and multi-tool agreement.
