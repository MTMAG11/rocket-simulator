# Testing

```bash
pytest                      # ~430 tests (3 real-flight regression tests are marked slow)
pytest -m "not slow"        # skip the slow tests
ruff check src tests && ruff format --check src tests
mypy src/rocket_sim
```

Expected answers come from closed-form physics or independent data wherever possible, not from the simulator.

| file | what it verifies |
|---|---|
| `test_math3d.py` | rotation/DCM/quaternion identities, composition order, launch attitude, FRD convention, Euler extraction |
| `test_environment.py` | ISA vs the 1976 table at every layer, pressure inversion, gravity `GM/(R+h)^2`, wind direction convention, profiles, turbulence statistics/determinism, gusts, terrain |
| `test_motor.py` | `.eng` parsing/units, impulse vs numeric quadrature, propellant depletion, interpolation, scaling, malformed files, multi-motor files, CSV |
| `test_dynamics.py` | free fall, vacuum projectile, rocket equation, quadratic drag (analytic), wind via relative velocity, Mach, torque-free precession, quaternion kinematics, weathercock frequency, instability, tail-first instability, TVC torque, rail hold |
| `test_numerics.py` | integrator orders 1/2/4, event root finding, timestep convergence dt 0.1 -> 0.001 (3-DOF) and 0.05 -> 0.001 (6-DOF), step-alignment independence |
| `test_simulation.py` | phase sequence, event order, impact/never-underground, energy-conservation impact speed, rocket-equation flight vs SciPy ODE, mass depletion, Mach logging, determinism (bitwise), no-liftoff, invalid configs, fidelity levels, fast mode, slope terrain, wind effects, launch angle, decimation |
| `test_data.py` | export round trips (4 formats), schema integrity, quality gates, seed derivation/distributions, windows/alignment, batch end-to-end, worker-count independence, crash/resume, rejection logging, split disjointness and hold-out, windowed shards |
| `test_gnc.py` | sensor statistics, bias walk, quantisation, saturation, rate, latency, truth != measurement, TRIAD, gyro integration, launch detector, Kalman filter accuracy, actuator, closed-loop TVC stabilisation of an unstable vehicle |
| `test_validation_cli.py` | metrics, telemetry import, self-consistency of the comparison pipeline, real-flight regression guards, uncertainty formatting, CLI |
| `test_golden.py` | tight (1e-9) golden numbers keyed to `PHYSICS_VERSION` (3-DOF and 6-DOF reference runs) |
| `test_geometry_mass.py` (V1.1) | Barrowman by hand, telescoping invariant, boat-tail, assembly validation, CG from components and burn shift, inertia tensor vs independent summation, parallel axis, Euler invariants for an asymmetric body |
| `test_aero_v11.py` (V1.1) | model hierarchy interface (CD/CL/Cm), small-angle limits, alpha/beta relative-wind geometry independent of the simulator, Reynolds/Mach trends, enhanced fin lift vs Helmbold/Ackeret, fin stall, boat-tail attached/separated, lookup and 2-D table models |
| `test_control_v11.py` (V1.1) | TVC zero/+/-/max gimbal, actuator saturation/rate/lag/delay exactness, physics uses the ACTUAL actuator state, control-fin force/moment analytics, mixer decoupling, closed-loop fin stabilisation |
| `test_sensors_v11.py` (V1.1) | GPS dropout rate and start-up delay, velocity vs position noise, misalignment, async rates and latency, truth estimator, TRIAD with declination, gyro-bias recovery |
| `test_numerics_v11.py` (V1.1) | quaternion algebra and long-run integration, conservation limits (no force / gravity / thrust / drag / torque), 5-step convergence ladder |
| `test_dataset_v11.py` (V1.1) | copula marginals and dependence, linked parameters, near-duplicate and temporal-leakage detectors (with negative controls), statistics/KS/traceability in the manifest, quality-gate classes, versioned experiment reproduction |
| `test_validation_v11.py` (V1.1) | registry/splits, AST fingerprint, holdout skip/log/stale/migration, calibration record, documented-number guards, input-uncertainty sampler |

`conftest.py` drives the dynamics directly with constructed vehicles for the analytic tests.

Bugs these tests caught during development: thrust returned 0 at the exact curve end points (breaking constant-thrust
analytic cases), a transposed rotation matrix in TRIAD alignment, tail-first flight being aerodynamically stable, GPS
latency ignored by the filter, off-by-one handling of loop variables in the event code.


## V1.1 policy notes

* Holdout flights are never simulated by the test-suite (only by the guarded CLI); tests of the holdout mechanics use a
  throw-away registry that points at a development flight.
* Negative controls: the temporal-leakage detector is shown to *fail* on a leaky pipeline, and migration is shown to be
  refused when a logged result does not reproduce - a detector that cannot fail proves nothing.
* Numbers quoted in the documentation are guarded: `test_logged_holdout_results_match_the_documented_numbers`,
  `test_recorded_input_mc_results_are_consistent`, `test_calibration_flight_regression`, `test_golden.py`.


## V1.2 additions

| file | what it verifies |
|---|---|
| `test_vehicle_file.py` | component mass summation, CG and full inertia tensor against independent numpy summation, shell/plate/shape formulas, tensor sign convention, off-axis components, CG travel during the burn, geometry placement, boat-tail, all validation errors, determinism, traceability, machine-independent config hash, equivalence with a hand-written config, JSON-schema consistency, CLI |
| `test_truth_separation.py` | estimator never given truth; **bit-exact replay of logged measurements through a fresh filter**; controller input equals the logged estimate, not the truth; state-source flags and refusals; ideal sensor == true specific force; noise statistics; saturation; barometer derivation; seeded determinism; column roles |
| `test_aero_provenance.py` | provenance for every model, estimate never labelled measured, declared Re dependence matches behaviour, dataset/record propagation, coefficient bounds and consistency over the whole envelope, Barrowman hand calculation + regression pins, RocketPy cross-check of the example vehicle |
| `test_hil_v12.py` | protocol encoding/validation, lock-step loop, every sample delivered, no truth across the boundary, uplink and compute/downlink latency, bit-identical live runs, replay without a flight computer and divergence detection, child-process transport == in-process, dead process, closed-loop stabilisation and latency degradation, timing report, CLI |
