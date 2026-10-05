# Independent critic review 1

Reviewer: independent (did not build the project). Date of review run: 2026-10-04. Scope: `src/rocket_sim` (59 files, ~8.4 kLOC),
`configs/`, `validation_data/`, `validation_results/`, `docs/`, `tests/`. Source files were not modified.

What I actually ran: `pytest tests -q` (152 passed, 0 skipped), `ruff check src tests` (clean), `mypy src/rocket_sim` (clean),
`rocketsim simulate` on both example configs, `rocketsim validate` on all three flights (reproduced the recorded numbers exactly),
a 20-run batch from `configs/batch_example.yaml`, a 30-run windowed dataset from `configs/dataset_example.yaml`, `inspect`/`export`/`check`,
the GUI headless (offscreen), a timestep-convergence study (reproduced the table in physics.md exactly), edge-case configs, and ad-hoc analyses
of the validation data. pandas/netCDF4 were not touched.

## Score: 6.5 / 10

Honest middle-to-upper: the engineering is far above a typical hobby simulator (correct 6-DOF kernel, analytic tests, honest docs, real
reproducibility machinery, working batch/dataset system). It is held below 7.5+ by: a confirmed data-quality bug that rejects ~20 % of the
shipped example batch, validation that is thinner and more in-sample than the headline claims state, no aerodynamic control surfaces or any
guidance/landing logic, fixed-percentage "uncertainty" values that are not statistically derived, stale committed artefacts, and Python-speed
throughput that makes the "100,000+ runs" goal an overnight job. A good UI, 152 tests and clean lint did not earn points beyond "competent".

---------------------------------------------------------------------------------------------------------------------------------

## A. Technical correctness

### Verified correct (I re-derived these)
* Quaternion kinematics `q_dot = 1/2 q (x) (0, w)` component form in `dynamics.py::RigidBody6DOF.derivative` is right (Hamilton, body->launch).
* Euler equations for `I = diag(Ixx, Iyy, Iyy)`: `wdot_y = (My + (Iyy-Ixx) r p)/Iyy`, `wdot_z = (Mz + (Ixx-Iyy) p q)/Iyy` is the correct expansion of `I w_dot = M - w x Iw`.
* TVC moment `M = r_noz x F` with `rn = x_cg - x_nozzle < 0` gives `My = -rn Fz`, `Mz = rn Fy`; matches the code and frames.md sign statement.
* Lateral aero force moment about CG and strip-theory damping signs (`my -= cdamp*q`, `mz -= cdamp*r`) are consistent.
* Wind is applied only through `v - wind` (3-DOF and 6-DOF), never added to ground velocity. ISA layered pressure, closed-form inverse, Sutherland viscosity are standard.
* Convergence table in physics.md reproduces to all printed digits (I re-ran dt = 0.05/0.02/0.01/0.001 vs 0.0005). RK4 with step truncation at thrust-curve corners and bisection-with-reintegration event location is a sound approach. A 32 mm / 60 g vehicle with 0.2 cal margin was stable and converged for dt 0.05 ... 0.0005.
* Determinism: two separate processes produce byte-identical CSV.
* Input validation is good (negative mass, dt = 0/NaN, elevation outside (0, 90], motor wider than body all give `ConfigError`).

### Physics / numerics problems found
1. **CONFIRMED BUG: event states are not renormalised.** `simulator.py` calls `refine_event(step, f, ...)` (`physics/integrators.py`), which calls the raw `step`
   and never `dyn.post_step`. The located event state (apogee, rail exit, **landing = last row of every flight**) has an un-normalised quaternion
   (error 2e-6 observed) and, for rail exit, no rail clamp. See "Confirmed bugs" below; it causes false rejections in the dataset quality gate.
2. **Gravity bias.** `inverse_square` uses `GM/R^2` with mean R: 9.8202 m/s^2 at sea level (the CSV shows 9.82022 at t = 0), vs the 9.80665 default of the constant model and
   ~9.79-9.81 real mid-latitude values (centrifugal term and oblateness ignored). A systematic +0.1...0.25 % g bias in the default model, in the validation runs (Prometheus at 32.9 deg N, 1401 m, real g ~ 9.792 vs 9.816 used) and in the estimator. Documented as "latitude variation ~0.5 %", but it is a bias, not just spread.
3. **Prometheus atmosphere does not do what the config comment/doc says.** `prometheus_sim.yaml` and validation.md say the sea-level pressure is "derived from the measured pad pressure (86444 Pa at 1401 m) so ISA reproduces it".
   With `temperature_offset_k: 15` and `sea_level_pressure_pa: 102334.2` the simulator's atmosphere at 1401 m gives **87180 Pa (+0.85 %)**, i.e. density ~0.85 % high. Small, but the documented claim is false (the +15 K was evidently applied after the pressure was derived).
4. **No thrust altitude/pressure correction**; sea-level static thrust only. For the 98 mm M1520 flown to 3.9 km this is a ~1-2 % thrust error, same order as the model's claimed accuracy, and docs say apogee changes ~2x per 1 % impulse. Documented as a limitation, unquantified.
5. **Barrowman CP/CNa are Mach-independent**; no transonic CP shift, no supersonic normal force, nose-wave drag is a smoothstep ramp that then stays constant above M 1.2. Max validated Mach 0.95. Documented, fine, but "fidelity level 6 / high-fidelity preset" is only a numerics preset (fidelity 6 gives bit-for-bit the same apogee as 5: 918.55 m). The label is mildly misleading.
6. **Cd is not angle-of-attack dependent** (cd0 * cos(alpha) + projected normal force). Acceptable below ~10 deg; no stall/fin-body interference. OK for a prior, not for a landing-flip or high-AoA controller study.
7. **Jet damping, dI/dt * w, propellant momentum, motor CG regression are neglected** (documented). Matters for roll/pitch damping of TVC studies, not for apogee.
8. **Turbulence is time-indexed, not space-indexed, altitude-independent**, active on the rail, and pre-generated for a fixed 900 s horizon (silently clamped beyond). Stated as simplified; unsuitable as a faithful training-domain gust model for landing-GNC.
9. **Level 2 "perfect weathercock" thrust along relative wind** is a modelling shortcut that differs from 6-DOF by >100 m in landing position (their own number). Fast-mode datasets therefore teach a systematically different wind response; the fidelity flag is recorded, which is the honest part.
10. **Motor `thrust_scale` Monte-Carlo keeps propellant mass fixed** so +-3 % thrust draws silently change Isp +-3 %. Physically a real motor-to-motor variation changes total impulse via propellant mass/burn rate, not Isp.
11. Apogee event is the first vz zero-crossing after liftoff: a tumbling/low-elevation or unstable-TVC flight can "apogee" (and arm an apogee parachute) during the burn. No guard.

---------------------------------------------------------------------------------------------------------------------------------

## B. Software quality

Strengths: clean package separation (config / environment / motor / vehicle / physics / simulation / sensors / estimation / control / data / validation / ui / cli); typed dataclass config with a validator that names the offending key; single `Simulation` class; Truth->sensors->estimator->controller->actuator->physics pipeline actually enforced (controller gets estimated state); mypy and ruff clean; atomic chunk writes; good docstrings with equations and references; generated schema/config docs; `performance.md` is honest about 40-50 % parallel efficiency.

Weaknesses:
* **Quality gate coupled to an integration artefact (bug 1)** and has no test that exercises the gate on a *normal* run with events at the end. `test_data.py` tests the gate with synthetic bad records only.
* **Untested code paths**: `TableAero`, `landing_zone`, `controller: type: python` loader, `benchmark.py`, `plotting.py`, `reporting.py`, the whole `ui/` package (I ran the GUI offscreen and it worked, but nothing guards it), terrain "planar" model through the dynamics. Only 2 tests are marked slow; "3 min" runtime claim is fine.
* **Regression guards are loose**: real-flight regression tests use 4-6 % tolerances (`test_validation_cli.py`), so a ~2 % silent physics change passes without a `PHYSICS_VERSION` bump. "No silent physics changes" (spec 48) rests on developer discipline, not on a golden-output test (e.g. checksum of a fixed telemetry run keyed to PHYSICS_VERSION).
* **Hot loop is pure Python/NumPy-scalar** (~9k RK4 steps/s per core). `evaluate` re-builds an `Eval` NamedTuple with 24 fields per call and caches with `y.tobytes()` keys. Fine for correctness, limits dataset scale.
* **`_finalize` loads all chunk JSONs (every run's ~100 summary and parameter fields) into one Python list in the parent**, and the leakage check/seed-disjointness are O(N) Python loops. At 100k runs this will be GB-scale and minutes; never exercised above 1000 runs.
* `config` resolution falls back to `PROJECT_ROOT` and `Path.cwd()` silently (`loader.resolve_path`), so the same config can pick different motor files depending on cwd.
* Repo hygiene: ~143 changed paths, almost all of the new package only *staged* (HEAD is still "updates"); `output/` is in `.gitignore` yet output files (parquet, png, benchmarks) are staged; no commit history for the build/critic iterations; the committed `output/mc-demo` is stale (physics 1.0.0, see D).
* Error handling: broad `except Exception` in `execute_chunk` records a reason, which is good, but a systematic bug (like bug 1) becomes a quiet 20 % rejection rate with no warning threshold (no "rejection rate > x %" alarm).

---------------------------------------------------------------------------------------------------------------------------------

## C. Requirements checklist (1-82)

| # | verdict | justification |
|---|---|---|
| 1 | PARTIAL | launch..impact, sensors, estimator, MC, dataset, validation all exist; "landing research" has no guidance/landing law, no landing-zone targeting beyond a stat, no fin control |
| 2 | PARTIAL | solid motor/TVC modelled; fin/aerodynamic-surface control authority absent (`Command` has only tvc_y/tvc_z) |
| 3 | PASS | `initial_assessment.md` is specific (8 itemised bugs, decision rationale) |
| 4 | PASS | physics/data first; GUI is a thin client |
| 5 | PARTIAL | configurable dt, truncation at events, event timestamps; variable step is only a two-rate (dt / descent_dt) scheme, no adaptive step |
| 6 | PASS | explicit state, quaternion + Euler, p,q,r, angular acceleration columns, total/prop/dry mass |
| 7 | PASS | frames.md covers launch/body/aero/ECI(not modelled), handedness, quaternion and wind conventions; tested |
| 8 | PASS | constant and GM/(R+h)^2 (slight +0.1 % bias, see A2) |
| 9 | PASS | ISA, exponential, table, offsets; verified against 1976 table |
| 10 | PASS | constant/profile/power-law/Gauss-Markov/gusts through relative velocity (turbulence simplified) |
| 11 | PARTIAL | `.eng` retained, CSV loader, scaling; only two formats; no pressure-dependent thrust; interpolation linear only |
| 12 | PASS | m(t) = dry + remaining propellant from impulse, CG/I/CP vs time (propellant CG fixed) |
| 13 | PASS | geometry dataclasses are .ork-importable in principle; single-diameter airframe is a limitation, not a blocker |
| 14 | PARTIAL | Mach/Re/component build-up/lift/moments present; no AoA-dependent Cd0, no multi-diameter bodies, transonic/supersonic weak, lookup-table model untested |
| 15 | PASS | `mach` column every step, coefficients depend on it |
| 16 | PASS | alpha/beta from body-frame relative velocity; drives lift, moments, stability |
| 17 | PASS | static margin logged every step, warnings for <1 cal and unstable; tested vs RocketPy |
| 18 | PASS | 6-DOF with quaternions and moments; Euler for display |
| 19 | PASS | gimbal limit, rate limit, lag, delay, update rate, misalignment; ideal vs realistic |
| 20 | PARTIAL | clean separation and plug-in point incl. python-class loader and HIL; only a null, an open-loop schedule and a PD attitude controller built in (no trajectory-following/guidance/state-feedback); `type: python` untested |
| 21 | PASS | accel, gyro, baro, GPS, mag with noise, bias+walk, scale error, quantisation, saturation, rate, latency; no vibration/misalignment/temperature (documented) |
| 22 | PARTIAL | genuine linear KF for pos/vel with latency compensation; attitude is gyro dead-reckoning (no correction, no bias estimation); not an EKF; honestly stated |
| 23 | PASS | phase machine with logged transitions (pre-launch, ignition, powered, burnout, coast, apogee, descent, landing) |
| 24 | PASS | sensor-based launch detector (|f| > 2.5 g for N samples) |
| 25 | PARTIAL | impact detection, speed/time/position, never underground, flat/slope/grid terrain; stops at first contact (no rebound/tip-over/post-impact state); `landing_zone` untested |
| 26 | PARTIAL | broad parameter randomisation with unique id and seed; parameters independent (no correlations), thrust-scale ignores propellant mass; no initial-condition randomisation beyond launch angle |
| 27 | PASS | headless, config->params->batch->quality->export works (I ran it); scale to 100k not demonstrated and memory/finalise design untested there |
| 28 | PARTIAL | CSV, JSON, Parquet, NPZ implemented; no native binary beyond NPZ (acceptable); pandas not needed |
| 29 | PASS | versioned schema (1.0.0) generated to docs/schema.md, in every manifest |
| 30 | PARTIAL | configurable windows/tabular with measured vs truth, split namespaces; targets limited to per-step columns: **no run-level labels (landing location, impact velocity), no derived targets (corrections, control outputs, deltas)** |
| 31 | PARTIAL | YAML feature/label lists without code changes; only 2 derived features, no transforms/normalisation/noise augmentation hooks |
| 32 | PASS | windowed history->future/trajectory with stride, float32 NPZ, sliding_window_view |
| 33 | PARTIAL | three public flights with source/quality tables; amateur, secondary-source parameters, filtered data, no measured atmosphere/wind |
| 34 | PASS | apogee, apogee time, max v/a, burnout, landing time, series metrics implemented |
| 35 | PASS | RMSE, MAE, max, bias, NRMSE, % and timing errors |
| 36 | PARTIAL | physical justification given for v1.1.0 change; but see E: the justification used the validation flight itself, and some inputs (fin thickness, thin-tube inertia, temperature offsets) are modeller choices |
| 37 | PARTIAL | loop documented; leave-one-out for one scalar only; structural change selected on Prometheus (in-sample); only 3 flights |
| 38 | PASS | this review |
| 39 | PASS | this review |
| 40 | PARTIAL | cannot verify (only this critic exists so far); `docs/review/` has no earlier iterations |
| 41 | PASS | this review is requirement-based |
| 42 | PASS | genuinely analytic tests (Tsiolkovsky, quadratic drag, torque-free precession, ISA table, quadrature impulse); I confirmed they are not simulator-vs-simulator |
| 43 | PASS | dt 0.1..0.001 table reproduced; 6-DOF cap 0.05 s documented |
| 44 | PARTIAL | headless, multiprocessing, streaming shards, checkpointing exist; throughput 8-24 sims/s, 40-50 % parallel efficiency, memory of the finalise step unbounded in N |
| 45 | PASS | simulate, batch, generate-dataset, validate, export, inspect, benchmark, check, schema, motors, gui all work |
| 46 | PASS | YAML/JSON (TOML not verified) with every requested section |
| 47 | PARTIAL | config, seed, versions stored; **motor file contents/hash and git commit are not recorded; config hash does not cover motor data** (see D) |
| 48 | PARTIAL | four version constants; physics version is a manual bump with only loose regression tests, and the committed demo dataset is stale w.r.t. it |
| 49 | PASS | required variables plotted, selection, scrubbing slider, events (I ran the GUI headless); no zoom test, untested |
| 50 | PASS | 3-D tab (secondary); not tested beyond launching |
| 51 | PASS | rich summary (apogee, max v/a/Mach, burnout, impact, flight time, landing position) |
| 52 | PASS | `inspect` and browser read manifest/runs.parquet only; row-group per run |
| 53 | PASS | good ConfigError/MotorError messages (verified several); but systematic bad runs are not alarmed |
| 54 | PASS | SI internally, units in column names/printouts |
| 55 | PASS | extensive docs; simplified models labelled. Docs contain stale/incorrect items (Prometheus pressure claim; "~150 tests") |
| 56 | PASS | directory layout matches the list |
| 57 | PASS | pipeline implemented in the loop in that order |
| 58 | PASS | randomised environments in place, no network trained |
| 59 | PARTIAL | `HilController` lock-step JSON bridge exists and is tested with a fake transport; no real-time pacing/timeouts, no serial test |
| 60 | PASS | standardised telemetry schema with truth/measured/estimated groups |
| 61 | PASS | declarative column map importer, unit handling (ft, ms) and source metadata for three CSV formats |
| 62 | PASS | comparison tool with plots and metrics |
| 63 | PARTIAL | rounding to stated uncertainty is nice, but the 1-sigma percentages are fixed constants from n = 3 and applied to vehicles never validated (see E4) |
| 64 | PASS | levels 0-6 table; level 6 is a numerics preset only |
| 65 | PASS | `fast: true` with exponential atmosphere and Cd(M) table; fidelity+fast stored in datasets and manifests |
| 66 | PARTIAL | high-fidelity preset exists; no demonstration it is more accurate against flight data (all validation is level 3 at dt 0.005) |
| 67 | PARTIAL | NaN, speed, altitude, rates, quaternion, landing, status gates exist; **the quaternion gate wrongly rejects ~20 % of valid runs** and is purely a numerical-artefact check |
| 68 | PASS | manifest has id, date, versions, config, counts, seeds, fidelity, feature+label schema, checksums |
| 69 | PASS | per-chunk atomic markers, spec-hash guard; resume tested by deleting a chunk |
| 70 | PARTIAL | parallel with measured speed-up and honest accounting; efficiency only ~45 % |
| 71 | PASS | benchmark command, single/100/1000/dataset, sims/s, steps/s, memory (output/benchmarks/latest.json) |
| 72 | PARTIAL | mypy/ruff clean, 152 tests, but UI/benchmark/plotting untested and the hot path is un-optimised Python |
| 73 | PASS | staged order evidenced in docs/fidelity levels |
| 74 | PASS | few fake features; limitations stated repeatedly (fidelity-6 label and fixed-% uncertainties are the exceptions) |
| 75 | PARTIAL | sources assessed (RocketPy repository, teams, ThrustCurve); parameters are secondary-source, no primary documents checked |
| 76 | PASS | equation/assumption/source/limitation per model in physics.md |
| 77 | PARTIAL | analytic -> unit -> independent simulator -> real data hierarchy present; "known calculations" and multiple-flight generalisation weak |
| 78 | PARTIAL | RocketPy cross-check with constant Cd (tests dynamics, not drag); OpenRocket/RASAero not run; static-margin agreement is asserted, not tabulated |
| 79 | PARTIAL | most items verified; "three critic iterations" and "final score" not yet applicable; clean-install not verified by me |
| 80 | PARTIAL | no final engineering report yet; README/docs do refuse "NASA-level" |
| 81 | PASS | docs are candid, README states what is not done |
| 82 | PARTIAL | workflow followed to critic 1; remaining iterations pending |

Tally: PASS 51, PARTIAL 31, FAIL 0 (checklist is lenient where the intent is met in principle; no requirement is flatly absent, but #2/#20/#30 under-state the original intent: control surfaces, guidance, run-level labels).

---------------------------------------------------------------------------------------------------------------------------------

## D. Data capability

Good: seed derivation from (master, split namespace, run index) is order/worker independent; split namespaces with optional distinct hold-out distribution; per-run id, per-run parameter draws and results in `runs.parquet`; Parquet row-group per run; resume works; byte-identical re-run of the same chunk; manifest rich; `inspect` cheap; windowed shards are float32 NPZ with run references.

Problems:
1. **Quality gate false positives (bug 1).** `rocketsim batch configs/batch_example.yaml --runs 20` -> 16/20 accepted, 4 rejected "quaternion norm drift". The valid runs are discarded, and *which* runs are discarded is correlated with the parameters (the rejection is threshold-marginal at 1e-6 and depends on descent dynamics), i.e. **selection bias in the training distribution**.
2. **Committed demo artefacts are stale.** `output/mc-demo` in the repo was produced with physics 1.0.0 (0/20 rejected, mean apogee ~14 % lower, 378,875 samples); the current code gives physics 1.1.0, 16/20, 85,202 samples. A single drag-submodel change moved the generic example vehicle's apogee ~14 % (779.5 -> 895.5 m for run 0), which also shows how weakly the small-rocket regime is constrained (the 1.1.0 change was validated only on 20 kg vehicles, and the "+-6 %" uncertainty is applied to this one).
3. **Reproducibility hole:** `config_hash` (and the dataset id) hash the config text only. The `.eng` file is referenced by path; replacing its contents leaves the config hash, simulation id and manifest unchanged. No motor-file hash, no git commit/dirty flag, no package/numpy versions in the manifest. `dataset_id` also depends on `output_dir` because `asdict(spec)` includes it.
4. **Labels:** only per-step truth columns; no landing location / miss distance / impact speed as supervised targets, no "correction" or control-output labels, only two derived features. A guidance-learning dataset (the stated goal) cannot be produced without editing code.
5. **Sensor channels are sample-and-hold natively but are linearly `np.interp`ed onto the resample grid** (`data/dataset.py::resample`): a window value can contain information from the next native sample (<= one native step; small, but it is a non-causal operation on measured inputs). Discrete columns (phase, flags, quaternion) are also linearly interpolated.
6. **Scale:** measured 8-24 sims/s; 100k runs are 1.5-3.5 h by their own numbers *if* the finalise step (all rows in one Python list, O(N) loops) survives; untested beyond 1000.
7. `time_since_ignition` derived feature reads the true config ignition delay; fine as a "known to the flight computer" assumption but inputs may still leak truth if the user lists truth columns (documented as the user's responsibility; no guard/warning).
8. Monte Carlo parameters are independent; no tolerance correlation (e.g. mass vs CG vs inertia; thrust vs propellant mass).

---------------------------------------------------------------------------------------------------------------------------------

## E. Validation (the weakest dimension relative to the headline)

What is good: real telemetry is used, metrics are quantitative and reproducible bit-for-bit (I re-ran all three), failures are reported (NDRT +8.7 %, landing-time +16.7 %), the leave-one-out protocol correctly refuses to calibrate, and the RocketPy cross-check agrees to < 0.6 %.

Problems and overclaims:
1. **The headline "validated against three amateur flights to a few percent in apogee" is partly in-sample.** The v1.1.0 base-drag change was found by looking at the Prometheus failure (-11 %) and the team's own RASAero Cd(M) curve for *that* flight. Prometheus -0.27 % is therefore a fit-quality, not a held-out result; the leave-one-out test covers only a global drag multiplier, not the structural choice. "Uncalibrated" in validation.md/README/uncertainty.py is too strong: the model form was selected on 1 of 3 test flights. Honest wording: "one flight informed model selection".
2. **NDRT's +8.7 % is explained away without evidence.** The doc says a -4 % impulse would give ~0 %, so "not evidence of a model defect". That is an untestable rescue (the same logic would excuse any error, in either sign, up to +-10 %). Their own sensitivity table shows +-5 % thrust moves apogee +-10 %, which means **the validation cannot discriminate drag-model quality at the few-percent level at all**; the RMS "5.2 %" mostly measures input uncertainty.
3. **Prometheus time-series is internally inconsistent and not discussed.** My analysis: integrating the *real velocity channel* from t0 gives ~4170 m at apogee-time vs the real altitude channel 3890 m, i.e. the real altitude and "speed" disagree by ~7 %; at 4 s the real altitude is 464 m while sim is 800 m (sim velocity there matches the real velocity within 1 m/s: 316.8 vs 317.6). So the *simulation is consistent with the real velocity channel and inconsistent with the real altitude channel* during ascent (altitude RMSE 152 m, max error 596 m, bias +119 m), while apogee agreement is -0.27 %. Either the AltOS speed is axial/total speed (launch elevation is 80 deg; sim total max speed 325 m/s vs vertical 316.8; the comparison uses sim *vertical* velocity against a quantity whose definition is not established) or the barometric altitude lags badly through transonic flight. The analysis note ("filtered") does not resolve it, and the apogee time error (-5.3 %, sim reaches apogee 1.5 s early) is the visible symptom. The time alignment is also a single-point 15 m crossing on the very channel that lags.
4. **Uncertainty numbers are fixed percentages**, not propagated or fitted: `uncertainty.MODEL_UNCERTAINTY` assigns 6 % apogee / 6 % Vmax / 2 % burnout time. "Burnout time ±2 %, validated" is essentially the motor's own thrust-curve duration (not an independent physics prediction). The README example prints `919 +/- 60 m` for a 29 mm model rocket that was never validated at all (drag regime, Reynolds, roughness differ; the v1.1.0 change alone shifted that vehicle ~14 %). A 1-sigma from n = 3 flights is not a confidence interval, as docs say, but the output formatting presents it as one (`+/-`).
5. **Gaps stated but significant:** no supersonic, no small vehicle, no TVC/attitude/accelerometer-grade truth, no wind data, descent unvalidated (landing time -4 % .. +17 %). The model's whole intended purpose (autonomous landing GNC) is entirely outside the validated envelope; the validated envelope is "apogee of 20 kg subsonic coast rockets".
6. **Input choices are free parameters:** temperature offsets (-8, -12, +15 K), fin thickness (3-4 mm), "implausible" published inertias replaced by thin-tube estimates for two vehicles, single-diameter NDRT airframe, assumed winds. Each is documented, none is bounded in a sensitivity study (only thrust and drag scale are). A formal input-uncertainty Monte Carlo of the validation flights (the project has the machinery!) would have shown whether -2/+9/0 % lies inside the input envelope.
7. **RocketPy cross-check uses a prescribed constant Cd = 0.45**, so it validates integrator/atmosphere/gravity/rotation, not drag, CP, or damping; and both runs were driven by the same author-chosen inputs. It is a good regression oracle, not an independent accuracy statement. OpenRocket/RASAero not run.
8. NRMSE "of range" (1.8-7.3 %) is dominated by the long parachute descent whose parameters are assumed; it flatters the series metrics.
9. Calibration module exists and refuses to accept non-improving fits, which is the right design; but nothing is calibrated, so spec items 36/37 are "process present, outcome none".

---------------------------------------------------------------------------------------------------------------------------------

## Confirmed bugs (with reproduction)

**BUG 1 (data-corrupting, confirmed): un-normalised quaternion at event states -> quality gate rejects valid runs.**
* Cause: `physics/integrators.py::refine_event` integrates with the raw stepper and returns `y_hi`; `simulation/simulator.py` then adopts `y_new = ye` without calling `dyn.post_step` (only the ordinary step path does `dyn.post_step(step(...))`). Landing is always the final recorded row, so every flight ends with a non-unit quaternion (4e-9 .. 2e-6 observed); `data/quality.py` rejects `|norm-1| > 1e-6`.
* Repro 1: `rocketsim batch configs/batch_example.yaml --runs 20` (output to a temp dir) -> "16/20 accepted, 4 rejected", every rejection "quaternion norm drift (invalid orientation)" (runs 1, 2, 7, 9).
* Repro 2 (Python): build `Simulation` for run index 1 of that spec (`derive_seeds(2024, ns, 1)`, `sample_run`) and compute `|sqrt(sum q^2)-1|` of `rec`: last row 2.0e-6, previous row 0.
* Side effects: rail-exit event state is not rail-clamped either; apogee event state un-normalised until the next step (harmless there).
* Impact: ~20 % of the shipped demo batch lost, non-randomly; any user-supplied threshold/tight gate behaves arbitrarily.

**BUG 2 (documentation vs code, confirmed): Prometheus atmosphere.** `ISAAtmosphere(15.0, 102334.2).at(1401).pressure = 87180 Pa`, not the claimed measured 86444 Pa (0.85 % high). Reproduce: build the environment from `validation_data/prometheus_sim.yaml` and query `env.atmosphere.at(env.site_elevation)`.

**BUG 3 (reproducibility, confirmed by inspection):** `config_hash`/`dataset_id` ignore motor-file contents (and include `output_dir` for the dataset). Replace `data/motors/AeroTech_G80T.eng` content, rerun: same simulation id and config hash, different flight.

**Stale artefact (confirmed):** `output/mc-demo/manifest.json` records physics 1.0.0 and 20/20 accepted; regeneration with current code gives physics 1.1.0 and 16/20 (see BUG 1). Docs/README text quotes figures for 1.1.0.

---------------------------------------------------------------------------------------------------------------------------------

## Strengths (summary)
* Correct, well-structured 6-DOF kernel with quaternions, truthful frame documentation, event location by re-integration, convergence demonstrated and reproducible.
* Strong, honest documentation culture: limitations listed per model, v1.0.0 -> v1.1.0 change log with a failing flight admitted, leave-one-out refusal to calibrate.
* Real separation of truth / sensors / estimator / controller / actuator; HIL bridge; closed-loop TVC example that stabilises an unstable airframe (and honestly tumbles after burnout).
* Data system with deterministic per-run seeds, hold-out distributions, resumable sharding, manifest/browser, ML windows.
* Analytic tests are real, not tautological; static analysis clean.

## Weaknesses (summary)
Bug 1; in-sample and under-discriminating validation with an inconsistent Prometheus channel; fixed-percentage uncertainty presented as 1-sigma; no control surfaces, no guidance/landing law, controller library minimal; labels limited to per-step columns; throughput and finalise-memory not proven at target scale; stale committed artefacts and uncommitted work; reproducibility hole for motor files; gravity bias; fidelity-6 label.

---------------------------------------------------------------------------------------------------------------------------------

## Ordered list of exact changes (highest impact first)

1. `simulation/simulator.py` (event adoption block, ~line 468) : after `te, name, ye = min(candidates...)` call `y_new = dyn.post_step(ye)` (or make `physics/integrators.refine_event` accept a `post` callable and apply it to every trial and the returned state); for `rail_exit` keep the clamp semantics. Add `tests/test_simulation.py::test_event_states_are_normalised` asserting `|q|-1 < 1e-12` on every recorded row of a landing flight and a regression test that `batch_example.yaml --runs 20` rejects 0 runs.
2. `data/quality.py::check_record`: raise default `max_quat_norm_error` to ~1e-6 only after fix 1, and add a batch-level alarm in `data/batch.py::_finalize`/`run_batch` (warn/fail if rejection rate > configurable % and report reason histogram in the manifest) so a systematic defect cannot silently bias a dataset. Record `rejection_reason_counts` in manifest `counts`.
3. Regenerate or delete `output/mc-demo`, `output/run1`, `output/validation`, `output/benchmarks` with physics 1.1.0 or stop tracking `output/` (it is already in `.gitignore`); commit the new package so history exists.
4. `config/loader.py::config_hash`, `data/batch.py::spec_hash` and `data/spec.py`: include SHA-256 of every referenced external file (`.eng`, terrain grids, wind CSV), the git commit (if available), numpy/scipy/pyarrow versions, and exclude `output_dir`/`workers` from `spec_hash`. Store them in `RunMetadata` and the manifest.
5. `validation/compare.py` + `validation_data/prometheus.yaml`: resolve what the AltOS `speed` column is (vertical vs along-axis/total) and compare like with like (sim vertical vs total speed accordingly); add a diagnostic that integrates real velocity and compares to real altitude and print the inconsistency; align time on apogee-independent and non-lagging features (e.g. burnout from the accelerometer column `acceleration` already present in the AltOS CSV) rather than the 15 m crossing of the lagging altitude; report ascent-phase error separately in the headline table.
6. `docs/validation.md`, `README.md`, `uncertainty.py`: reword "validated... uncalibrated" to state that the v1.1.0 drag-model structure was selected using Prometheus, treat Prometheus as in-sample, and flag the 29 mm example uncertainty as extrapolated; replace the fixed `MODEL_UNCERTAINTY` percentages with values derived by a Monte-Carlo of the *input* uncertainties for the validation flights (use `batch.py` over temperature offset, thrust scale 3 %, mass, fin thickness, inertia, wind) plus the observed model residual, or remove the `+/-` from printouts for unvalidated regimes. Drop the "burnout time validated ±2 %" claim (it is the motor curve duration).
7. Validate the actual target regime: add at least one small mid-power/model-rocket flight with published altimeter data and one supersonic flight; add an input-uncertainty envelope to each `validation_data/*.yaml` run; add a physics-version golden test (checksum or tight 1e-6 comparison of a fixed 3-DOF and 6-DOF run, keyed to `PHYSICS_VERSION`) so unversioned physics changes fail CI (`tests/test_validation_cli.py`, new `tests/test_golden.py`).
8. Control/guidance gap: implement aerodynamic control surface actuation (`control/actuators.py::Command` + `vehicle/aero.py` fin deflection force/moment + `dynamics.py` hook) or explicitly demote requirement 2/19 in README; add at least one guidance-capable reference controller (e.g. trajectory tracking or landing-point PD/MPC stub) in `control/controllers.py` with tests, and a test for `type: python`.
9. Dataset labels: extend `data/spec.py::DatasetSection` and `data/dataset.py` with run-level targets (`res.landing_x_m`, `res.impact_speed_ms`, miss distance vs `landing_zone`, remaining-time-to-apogee/landing as derived per-step labels, control command columns as targets) and with feature transforms (difference, normalisation constants stored in the manifest); build the resample for sensor columns causally (zero-order hold, not `np.interp`) and for discrete columns (nearest/hold); warn when truth columns are listed as inputs.
10. Scale and performance: stream `_finalize` (append rows via `pq.ParquetWriter` per chunk instead of one Python list; compute leakage check with hashing per chunk), run a 100k-run FAST dataset to completion as part of `benchmark`, cache motor/config parsing per worker, vectorise or compile the 3-DOF fast path (NumPy across runs or numba); report real efficiency.
11. Physics fixes with small cost: (a) gravity: use a latitude/centrifugal-aware surface g (or default `inverse_square` anchored at g0 = 9.80665 at the site) and record the site latitude; (b) correct `prometheus_sim.yaml` pressure (derive `sea_level_pressure_pa` from 86444 Pa *with* the +15 K offset applied) and re-run validation; (c) add ambient-pressure thrust correction (`motor.py` + nozzle exit area) as an option; (d) make `motor.thrust_scale` also scale propellant mass (option) so Isp stays fixed; (e) add `apogee` guard (only after burnout / vz crossing with altitude > threshold); (f) make turbulence altitude/space dependent (Dryden/von Karman by airspeed*time) or label it clearly in the schema.
12. `physics.md`/`simulation/builder.py`: rename or differentiate fidelity 6 (currently identical to 5 apart from numerics), or add something genuinely different (e.g. Mach-dependent CP/CNa, pressure thrust).
13. Add tests for `TableAero`, `landing_zone`, controller `type: python`, `benchmark`, plotting and a GUI smoke test (offscreen Qt, as I ran manually), and a test for planar terrain through the dynamics.
14. `config/loader.py::resolve_path`: remove the silent `PROJECT_ROOT`/`cwd` fallbacks or record the resolved absolute path (and its hash) in the manifest.

Re-score guidance for the next critic: fixing 1-6 and the Prometheus consistency question would justify ~7.0; adding a held-out small/supersonic flight, control surfaces, run-level labels and a demonstrated 100k run would justify ~7.5-8. Scores of 9-10 would need independent primary-source data (measured atmosphere, wind, weighed vehicle, TVC/attitude telemetry) and OpenRocket/RASAero agreement, none of which exist yet.
