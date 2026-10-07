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
| Actuator chain (delay, lag, rate, angle, saturation) | yes | yes (closed forms) | yes | - | **no** | left-endpoint hold -> ~h/2 extra lag; no backlash/deadband/torque limit |
| TVC force/moment from the ACTUAL state | yes | yes | yes | - | **no** | |
| Control fins, mixer, roll control | yes | yes (force/moment/mixer) | closed-loop sim | - | **no** | linear lift, no stall/hinge model |
| Sensors: noise, bias, walk, scale, quantisation, saturation, rate, latency, dropout, start-up | yes | yes (statistics, exact timing) | yes (seeded determinism) | - | **no** | generic MEMS numbers; no vibration, temperature, g-sensitivity |
| Truth / measurement / estimate / command / actual separation | yes | - | yes (bit-exact estimator replay, spy tests) | - | n/a | roles in the schema |
| Linear Kalman filter | yes | partial (alignment, bias) | yes (replay) | - | **no** | not an EKF; attitude is gyro-integrated |
| Estimator interface (null / truth / linear KF) | yes | - | yes | - | n/a | EKF: **design only** |
| HIL protocol v2, transports (loopback, pipe, record/replay) | yes | - | yes (replay, process == in-process) | - | **no** | no serial/UDP; no real hardware |
| Explicit timing model and report | yes | yes (derived latencies) | yes | - | **no** | no clock drift/jitter model |
| Closed-loop control through the HIL boundary | yes | - | simulation only | - | **no** | reference computer = same algorithms as in-process |
| Datasets: roles, provenance and state-source columns | yes | - | yes | - | n/a | |

## Aerodynamic data classes (for ML consumers)

| `aero_provenance_kind` in `runs.parquet` | meaning |
|---|---|
| `estimate` | every coefficient is an analytical/empirical estimate made by this simulator (Barrowman, build-up) |
| `mixed` | some coefficients are user/imported data (e.g. a Cd table) and some are still estimates (stability from geometry) |
| `imported` | all coefficients come from supplied data. **Currently unreachable**: the damping derivatives are always a simulator estimate, so any model is at best `mixed` (the label exists so a future fully imported model is not mislabelled) |

Nothing in this repository is labelled `experimental` or `cfd` unless the user declares it; declared provenance is a statement by the user and is not checked.
