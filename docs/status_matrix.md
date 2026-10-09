# Status matrix: what is implemented, verified, cross-checked, validated

Evidence levels (a row can have several; **later levels do not follow from earlier ones**):

* **I** implemented (code runs and is exercised by tests)
* **A** analytically verified (compared with a closed form / hand calculation / independent numpy formula in the tests)
* **N** numerically verified (convergence, invariants, determinism, round trips)
* **X** cross-checked against an independent code (RocketPy) or an independent re-implementation
* **E** experimentally validated against real flight/bench data
* **-** not validated in that sense

"E" appears only where real data were compared. **No component of the vehicle, actuator, sensor, estimator or HIL model has been
validated against a real vehicle or real hardware.** The only real-flight comparison is apogee/altitude of seven Tier 2/3 flights
([validation.md](validation.md)).

| capability | I | A | N | X | E | notes |
|---|---|---|---|---|---|---|
| 6-DOF rigid-body dynamics, quaternions, RK4 | yes | yes | yes | yes (trajectory, <1 %) | apogee only | [physics.md](physics.md) |
| Component vehicle file, compile, validation | yes | yes | yes | CN_alpha/CP of the example | **no** | placeholder example; no weighed vehicle |
| Mass, CG, **full inertia tensor**, CG shift in burn | yes | yes (numpy/by hand) | yes | - | **no** | thin-shell inertia estimates unless supplied |
| Shell/plate mass from material x thickness | yes | yes | - | - | **no** | ignores adhesive, fillets, hardware |
| CP / CN_alpha from geometry (Barrowman) | yes | yes | - | yes (4 vehicles + example) | static margin never measured | same equations as RocketPy |
| Aerodynamic model hierarchy + Cd/Cl/Cm interface | yes | yes | yes | trajectory with constant Cd | apogee of 7 flights (subsonic) | enhanced/supersonic: **not validated** |
| Aerodynamic **provenance** per quantity | yes | yes (behaviour matches declared Re dependence) | - | - | n/a | declared by judgement, not computed |
| Reynolds number dependence | skin friction only | - | - | - | **no** | pressure drag and lift are Re-independent; table models have no Re axis (stated in provenance) |
| Actuator chain (delay, lag, rate, angle, saturation) | yes | yes (closed forms) | yes | - | **no** | left-endpoint hold -> ~h/2 extra lag; no load-dependent torque model |
| Servo deadband, acceleration (torque/inertia) limit | yes (default off; defaults bit-identical to the old chain) | yes (stopping point, exact first step, discrete ramp) | yes | - | **no** | the acceleration limit is a constant you must supply (torque limit / load inertia) |
| Servo backlash | wiring only | hand table in the derivation (strict xfail) | with a test-local reference operator | - | **no** | `actuators.play_operator` is the owner's TODO; a non-zero backlash raises until it is implemented; feedback is modelled at the motor shaft |
| Servo bench-data fit (`rocketsim fit-servo`): delay, lag, rate limit, deadband (backlash opt-in) | yes | recovers known parameters from data made by an independent integrator; reported standard errors match the scatter over noise draws | yes | - | **no** | not yet run on real bench data (the bench file format has not been given); the rate limit is reported as not identifiable when no step saturates it |
| TVC force/moment from the ACTUAL state | yes | yes | yes | - | **no** | |
| Control fins, mixer, roll control | yes | yes (force/moment/mixer) | closed-loop sim | - | **no** | linear lift, no stall/hinge model |
| Sensors: noise, bias, walk, scale, quantisation, saturation, rate, latency, dropout, start-up | yes | yes (statistics, exact timing) | yes (seeded determinism) | - | **no** | generic MEMS numbers; no vibration, temperature, g-sensitivity |
| Truth / measurement / estimate / command / actual separation | yes | - | yes (bit-exact estimator replay, spy tests) | - | n/a | roles in the schema |
| Linear Kalman filter | yes | partial (alignment, bias) | yes (replay) | - | **no** | not an EKF; attitude is gyro-integrated |
| Error-state EKF (`estimator.type: ekf`): attitude, velocity, position, gyro and accelerometer bias | yes | transition matrix vs finite differences of the nominal propagation; Jacobian oracle vs a finite difference of the measurement | yes (bit-exact replay of logged measurements, symmetric PSD covariance, NEES over 8 flights, never given truth) | - | **no** | simulation only; GPS/baro only unless the magnetometer update is enabled, which needs `attitude_measurement_jacobian` (owner's TODO); initial attitude uncertainty has placeholder floors (0.5 deg tilt, 1 deg heading); not run on real data |
| EKF vs linear KF on identical simulated flights (`scripts/compare_estimators.py`) | yes | - | yes | - | **no** | see validation_results/estimator_comparison.md; nominal: position 0.49 vs 1.13 m, velocity 0.23 vs 0.96 m/s, attitude 1.00 vs 1.16 deg RMS (8 seeds); a vertical, mostly unwinded flight is a weak test of attitude estimation |
| Estimator interface (null / truth / linear KF / EKF) | yes | - | yes | - | n/a | |
| HIL protocol v2, transports (loopback, pipe, record/replay) | yes | - | yes (replay, process == in-process) | - | **no** | no serial/UDP; no real hardware |
| Explicit timing model and report | yes | yes (derived latencies) | yes | - | **no** | no clock drift/jitter model |
| Closed-loop control through the HIL boundary | yes | - | simulation only | - | **no** | reference computer = same algorithms as in-process |
| Motor thrust curves (`.eng`), propellant depletion | yes | yes (impulse vs numeric quadrature, depletion proportional to impulse) | yes | - | **no** | no static-fire data; the simulation uses the file's curve, not the catalogue values |
| Motor library (ThrustCurve.org client, `fetch-motors`, sidecar provenance) | yes | - | yes (fake-API tests; SHA-256 of the bytes on disk; no line-ending conversion, enforced by `.gitattributes` and a test) | - | n/a | `Estes_F15` is certification-lab data; `AeroTech_G40W` is a user-contributed 2005 curve (impulse 99.0 vs 97.14 N·s certified, peak 74.3 vs 84.5 N). Certified-impulse check of the flight motors: pending (test file to be supplied) |
| Structural load screen: peak q·alpha vs a configured limit, summary + warning | yes | yes (hand values; independent recomputation from the log) | yes | - | **no** | limits are user inputs, none ships (the TVC example has a marked placeholder); lateral load needs `body_normal_force()` (owner's TODO, see docs/derivations/structural_load.md) and is reported as unavailable until then; a screen, not a stress analysis |
| Drag from the coast phase of a flight log (`validation/drag_fit.py`): Cd vs Mach with bootstrap + systematic error | yes | recovers a known Cd from a SciPy-integrated coast with noisy altitude (about +/-10 % scatter for a 25 Hz barometer with 0.3 m noise) and from the 6-DOF simulation (0.497 +/- 0.016 for a true 0.50); accelerometer route ~4 % | yes | - | **no** | `solve_cd()` is the owner's TODO (hand value in the test, strict xfail), so the tool raises until it is implemented; not run on a real log (the log importer, T3, is pending); vertical-flight assumption for the barometer route; one Mach bin for a low-speed flight |
| Flight-log importer (`rocketsim import-log`, `validation/flightlog.py`): format file, time base and counter wraparound, duplicates / out-of-order / bad rows, gaps and dropped samples (reported, not filled), accel g vs m/s^2 and axis sign, gyro units, saturation (full scale or flag column), launch detection, pressure to altitude | yes | hand/independent values for units, wraps, gaps, launch time (vs the true specific force) and saturation | yes (a synthetic log with every fault injected; the converted log drives `validate` to a few % apogee error) | - | **no** | not run on a real log: the log columns, units and timestamp format of the ESP32 logger have not been given (the format file is where they go); `pressure_to_altitude_m()` is the owner's TODO (a log with a pressure column raises until it is implemented); the barometer sees bay pressure, not free-stream static pressure; accel axis mapping is whatever the format file says |
| Q-01 trade-study harness (`scripts/q01_trade_study.py`, `rocket_sim/studies`): rail exit speed, thrust/weight, gimbal torque, authority time, closed-loop Monte Carlo with a passive reference | yes | Wilson interval, CG-shift mechanics, disturbance statistics, success criteria, torque pipeline against a test-local oracle | yes (end to end on the example vehicle with test-only numbers) | - | **no** | **not run for Block 0**: the added masses, servo parameters, dispersions and success limits have not been supplied (the template has null for each and refuses to run); `max_correctable_torque()` is the owner's TODO (torque rows read n/a until then); the success definition is a proposal |
| CG recomputation from a vehicle's component table (`vehicle/cg.py`) | scaffold | the example vehicle's reported CG equals the mass-weighted mean of its components, with and without the motor (using a test-local mean) | yes | - | **no** | `center_of_gravity()` is the owner's TODO; the Block 0 vehicle file does not exist yet (its masses and positions have not been provided) |
| Datasets: roles, provenance and state-source columns | yes | - | yes | - | n/a | |

## Aerodynamic data classes (for ML consumers)

| `aero_provenance_kind` in `runs.parquet` | meaning |
|---|---|
| `estimate` | every coefficient is an analytical/empirical estimate made by this simulator (Barrowman, build-up) |
| `mixed` | some coefficients are user/imported data (e.g. a Cd table) and some are still estimates (stability from geometry) |
| `imported` | all coefficients come from supplied data. **Currently unreachable**: the damping derivatives are always a simulator estimate, so any model is at best `mixed` (the label exists so a future fully imported model is not mislabelled) |

Nothing in this repository is labelled `experimental` or `cfd` unless the user declares it; declared provenance is a statement by the user and is not checked.
