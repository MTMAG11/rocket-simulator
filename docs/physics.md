# Physics models

Physics model version: **1.2.1** (`rocket_sim.version.PHYSICS_VERSION`, stored in every record and dataset).
Every model below lists its equation, assumptions, source and limitations. Simplified models are
labelled as such; nothing here is exact physics.

## Fidelity levels (model hierarchy)

| level | model | notes |
|---|---|---|
| 0 | 1-D vertical, constant *g*, vacuum | analytic checks; very fast |
| 1 | 3-DOF point mass, *g(h)*, vacuum | thrust along the launch axis |
| 2 | 3-DOF + atmosphere + wind + drag | zero-angle-of-attack ("perfect weathercock") thrust/drag along the relative wind; no lift, no moments |
| 3 | 6-DOF rigid body, Barrowman normal force, aerodynamic moments + damping, TVC | the reference physics level |
| 4 | level 3 + simulated sensors | accelerometer, gyro, barometer, GPS, magnetometer |
| 5 | level 4 + estimator + controller + actuator model | closed loop |
| 6 | level 5 numerics: dt <= 5 ms, RK4, ISA, inverse-square gravity | "high-fidelity preset". **Not** a claim of validated high fidelity: the validation status is whatever `docs/validation.md` says |

`fast: true` (levels <= 2): exponential atmosphere + a Cd(Mach) lookup table evaluated once at a fixed
air state (no Reynolds/altitude dependence at run time). Measured effect on apogee: within ~4 % of level 2
(`tests/test_simulation.py::test_fast_mode_runs_and_is_recorded`). The fidelity level and the fast flag are
stored with every dataset.

Fidelity 2 vs 3 differ in horizontal drift under wind (zero-AoA weathercocking vs real attitude dynamics):
for the same inputs the apogee differs by ~1.4 % but the landing position can differ by >100 m.

## Frames, state and units

See [frames.md](frames.md). SI units internally. State (6-DOF): position (launch frame), velocity,
quaternion body->launch, body angular velocity; mass properties are explicit functions of time.

## Equations of motion (levels 3-5)

```
r_dot = v
m v_dot = R(q) (F_thrust + F_aero)_B + F_chute_L + m g_L ,          g_L = (0, 0, -g(h))
q_dot   = 1/2 q (x) (0, w)
I w_dot = M - w x (I w) ,                         I = 3x3 tensor (diag(Ixx, Iyy, Iyy) in the axisymmetric fast path)
```

* Thrust is an *external* force (momentum thrust is inside the thrust-curve value). Neglected: jet damping,
  `dI/dt * w`, propellant relative momentum (small for hobby/mid-power motors; unquantified for others).
* Time integration: fixed-step RK4 (default), midpoint, Euler. Quaternion renormalised after each step.
* Steps are truncated to land on discontinuities (thrust-curve corners, parachute opening, sensor/controller
  ticks). Zero crossings (rail exit, apogee, ground impact, altitude-triggered chute) are located by bisection with
  re-integration, so event times do not depend on the output grid.
* Rail: while `on_rail` the vehicle is constrained to slide along the rail axis (no rotation, no backward slide);
  frictionless. Released when the travel exceeds `rail_length_m`.

### Convergence (measured at physics 1.2.0; ladder dt = 0.1 / 0.05 / 0.01 / 0.005 / 0.001 s)

Error against a dt = 0.0002 s reference, example vehicle, 3 m/s crosswind, no parachute, RK4 (reference dt 0.0002 here, 0.0005 in the
automated ladder is `tests/test_numerics_v11.py::test_convergence_ladder`):

| fidelity | dt [s] | apogee error [m] | apogee time error [s] | landing-x error [m] | impact speed error [m/s] |
|---|---|---|---|---|---|
| 2 (3-DOF) | 0.1 | +0.409 | +0.0035 | -0.170 | +0.0073 |
| 2 | 0.05 | +0.028 | -0.0002 | -0.013 | +0.0005 |
| 2 | 0.01 | +0.035 | +0.0001 | -0.023 | +0.0006 |
| 2 | 0.005 | +0.017 | 0.0000 | -0.007 | +0.0003 |
| 2 | 0.001 | 0.000 | 0.0000 | 0.000 | 0.0000 |
| 3 (6-DOF) | 0.1 | *rejected by config validation (6-DOF needs dt <= 0.05 s)* | | | |
| 3 | 0.05 | -0.149 | -0.0018 | +0.033 | -0.0027 |
| 3 | 0.01 | +0.012 | 0.0000 | -0.005 | +0.0002 |
| 3 | 0.005 | -0.007 | -0.0002 | +0.001 | -0.0001 |
| 3 | 0.001 | 0.000 | 0.0000 | 0.000 | 0.0000 |

Apogee is ~900 m, so even dt = 0.1 s (3-DOF) or 0.05 s (6-DOF) is accurate to < 0.05 % in apogee. The error does not fall
monotonically below ~0.03 m: that floor comes from piecewise-linear thrust-curve corners and event location, not from the
integrator order (RK4 order is verified separately: `test_integrator_order_of_accuracy`). 6-DOF is capped at dt <= 0.05 s by
config validation because attitude dynamics of statically stiff vehicles become unstable for larger steps (omega_n dt must
stay O(1)): the 0.1 s case is therefore a *rejected configuration*, not a result. Euler at dt = 0.01 is >5x worse than RK4
(slow test).

### Verified invariants (V1.1, `tests/test_numerics_v11.py`)

Quaternion: identity and conjugate algebra, 90/180 degree rotations about every axis, 4000-step integration keeps |q| = 1 to
1e-12 and matches the exact exponential map to 1e-6, (heading, pitch, roll) round trip, gimbal-lock finiteness.
Conservation limits: no forces -> constant velocity and zero rate; gravity only -> mechanical energy and horizontal momentum
conserved (1e-9); thrust only -> Tsiolkovsky within 0.2 %; drag only -> energy decreases monotonically and v(t) follows the
closed form `1/(k t + 1/v0)` to 1e-8; torque only -> `omega = M t / I`; torque-free asymmetric body -> |H| and kinetic energy
conserved ([vehicle.md](vehicle.md)).

## Gravity (`environment/gravity.py`)

`g(h) = GM / (R + h)^2`, GM = 3.986004418e14 m^3/s^2 (WGS-84), R = 6371008.8 m (IUGG mean), h = altitude MSL.
Constant-g option. Gravity always acts along -z of the launch frame (flat-Earth). Neglected: latitude variation
(~0.5 %), J2, Earth rotation/Coriolis (matters only for range > ~50 km or high-precision work).

## Atmosphere (`environment/atmosphere.py`)

* **ISA** U.S. Standard Atmosphere 1976 (NASA-TM-X-74335): layered lapse rates to 84.852 km geopotential,
  isothermal beyond; `T`, `p`, `rho = p/(R T)`, `a = sqrt(gamma R T)`, Sutherland viscosity. Optional uniform
  temperature offset and sea-level pressure to represent the day's conditions. Verified against the published table
  at every layer boundary (`tests/test_environment.py`, 2e-5 relative).
* **Exponential** (fast mode): `rho = rho0 exp(-h/H)` with H = 8400 m, T from a 6.5 K/km lapse down to 216.65 K.
* **Table**: measured T(h), p(h); density from the ideal-gas law; out-of-range queries are errors.
* Limitations: dry air (no humidity), no horizontal/temporal variation.

## Wind (`environment/wind.py`)

Wind is the velocity of the *air*; the aerodynamic velocity is always `v_vehicle - v_wind` (never added to the
ground velocity). Models: constant, altitude profile (u/v interpolation), power law, first-order Gauss-Markov
turbulence (pre-generated on a fixed grid from the run seed, so identical for all step sizes), 1-cos gusts.
Simplified: turbulence is time-correlated, not spatially correlated (not Dryden/von Karman); vertical gust
components are scaled by 0.5.

## Motor (`motor/`)

Piecewise-linear thrust curve with a prepended (0, 0) point when the file starts later; closed-form cumulative impulse;
`m_p(t) = m_p0 (1 - I(t)/I_total)` (constant effective exhaust velocity). Sources: RASP `.eng` format (ThrustCurve.org).
Limitations: sea-level static thrust (no altitude/pressure correction); propellant CG fixed at the motor centre;
grain geometry not modelled. Total-impulse variation between motors of the same type is typically a few percent and
is the dominant uncertainty in apogee prediction (see validation: +-5 % thrust gives about +-10 % apogee).

## Mass properties (`vehicle/mass.py`, `vehicle/assembly.py`; details in [vehicle.md](vehicle.md))

Rigid components (airframe sections, fins, extra masses, motor casing, propellant), each with its own mass, axial CG and
(optionally) lateral offsets and a full inertia tensor, combined with the parallel-axis theorem about the instantaneous
CG. With a component-based airframe (`rocket.sections`, `rocket.masses`) the CG and inertia are *computed* from the parts;
the legacy lumped description (`dry_mass_kg`, `cg_from_nose_m`, optional `inertia`) is still accepted. When any inertia is
estimated (thin shell / thin tube, never measured) the vehicle records a note and a warning. An axisymmetric fast path
(V1-identical) is used when all offsets are zero; otherwise the general 3x3 tensor path (`MassProps.tensor()`) and the
general Euler equations are used. Propellant: solid cylinder, mass from the thrust-curve impulse.

## Aerodynamics (`vehicle/aero.py`)

Since 1.2.0 the aerodynamics are a model *hierarchy* behind one coefficient interface (`CD/CL/Cm(M, alpha, Re, geometry)`);
see [aerodynamics.md](aerodynamics.md). The text below describes the default `barrowman` model (the V1 build-up, generalised
to multi-diameter airframes, transitions and boat-tails).

Coefficients referenced to `S = pi d_ref^2/4`, d_ref = largest body diameter.

**Force law (alpha = total angle of attack, 0..pi):**
`F_x = -q S Cd0 cos(alpha)` (axial);
`F_lat = q [ S CNa sin(alpha)|cos(alpha)| (at CP) + Cd_cf A_planform sin^2(alpha) (at planform centroid) ]`,
directed opposite to the lateral relative air velocity. For alpha -> 0: `D = q S Cd0`, `N = q S CNa alpha`.
Induced drag emerges from projecting the normal force on the velocity. The `|cos|` makes tail-first flight
statically unstable (a body with CP aft of CG is unstable in reversed flow, like a thrown dart); an earlier
`sin cos` form made the reversed attitude stable, which the post-deployment angle-of-attack plot exposed and the
test `test_tail_first_flight_is_unstable_and_flips_to_nose_first` now guards.

**Normal force / CP (Barrowman 1966-67):** nose `CNa = 2`, CP at 0.466 L (tangent ogive) / 2/3 L (cone) / 0.5 L /
0.333 L; fins `CNa = K_fb 4 N (s/d)^2 / (1 + sqrt(1 + (2 L_f/(C_r+C_t))^2))`, `K_fb = 1 + R/(s+R)`, CP from the
root leading edge `x_m/3 (C_r+2C_t)/(C_r+C_t) + 1/6 ((C_r+C_t) - C_r C_t/(C_r+C_t))`. Body tubes contribute no
normal force (Barrowman). Valid for small alpha and subsonic-to-low-transonic Mach.

**Moments:** CP moment of the lateral forces about the instantaneous CG, plus damping from strip theory: each lifting
surface sees an extra local angle of attack `w x r / V`, giving `M_damp = -1/2 rho V S sum(CNa_i x_i^2) w` (pitch/yaw)
and `-rho V S CNa_fin r^2 w` (roll damping). Fin cant / roll forcing not modelled.

**Drag build-up (default `BuildupAero`):**
`Cd0 = Cf_body FF S_wet/S + Cf_fin (1+2t/c) 2 N A_fin/S + fin-edge leading-edge pressure + nose wave + base`

* Skin friction: laminar below Re 5e5, else Schlichting turbulent `0.455/log10(Re)^2.58`, Re capped at the roughness
  cut-off `38.21 (L/k)^1.053` (Raymer), times the compressibility factor `(1+0.144 M^2)^-0.65`. Default roughness 60 um
  (ordinary paint).
* Form factor `1 + 60/f^3 + 0.0025 f` (Hoerner), f = fineness ratio.
* Fin leading edge: `Cstag(M) cos^2(sweep)` x edge area x 0.5 (rounded); fin trailing edge as base drag.
* Nose wave drag: `0.8 sin^2(phi)` smoothly switched on between M 0.8 and 1.2 (approximate).
* **Base drag (changed in v1.1.0):** subsonic Hoerner `0.029/sqrt(Cd_forebody)` (referenced to base area), blended to
  Barrowman supersonic `0.25/M` over M 0.8-1.0; reduced by the nozzle area while the motor burns.
* Sources: Barrowman & Barrowman (1966/67); Niskanen, *OpenRocket technical documentation* (2013); Hoerner,
  *Fluid-Dynamic Drag* (1965); Raymer, *Aircraft Design* ch. 12; Schlichting, *Boundary-Layer Theory*.
* **Accuracy:** component build-up is a prior, good to roughly +-10-15 % at subsonic speed (see validation:
  three recovered flights: +0.5/+13/-5 % barometric apogee, RMS 8 %; see validation.md), +-25-30 % in the transonic/supersonic regime. Not modelled:
  launch lugs/rail buttons (use `extra_cd`), fin flutter, fin cant, interference beyond `K_fb`. Multi-diameter airframes,
  transitions and boat-tails are supported since 1.2.0 (assembly-based; boat-tail base area depends on a flow-separation
  rule of thumb). `drag_scale` multiplies the total and should only be changed with a physical reason
  (leave-one-out tests found no justification for a global tune).

Other aero models: `simplified` (flat Cd), `enhanced` (Mach-dependent fin lift/CP, fin stall, fin wave drag), `table` (Cd(M)),
`table2d` (Cd/Cl/Cm over Mach x alpha from a CSV: the CFD / wind-tunnel interface), `constant`.

## Static margin / stability

`(x_cp - x_cg)/d_ref` in calibers, from the subsonic Barrowman CP of the assembly (including transitions and boat-tails) and
the instantaneous CG; logged each step (`static_margin`) and warned at launch if < 1 cal (error-level warning if <= 0).
The CP of the normal force used for *moments* and the static CP are separate (`ForceCoefficients.x_cp_force` /
`x_cp_static`): the enhanced model moves the fin CP with Mach. Cross-checked against RocketPy: CNa identical, CP within
0.04 cal (`validation_data/crosscheck_geometry.py`, [validation.md](validation.md)).

## Thrust-vector control and aerodynamic control surfaces

*TVC.* Thrust is deflected by (theta_y about y_B, theta_z about z_B) plus a fixed misalignment; applied at the nozzle exit
plane, producing `M = r_nozzle x F` (including lateral CG offsets). The physics receives the **actual actuator state**, not
the command (tested with a 0.5 s transport delay: no torque before the delayed command arrives). Verified analytically for
zero, positive, negative and maximum gimbal (`tests/test_control_v11.py`).

*Actuator model* (`control/actuators.py::ActuatorBank`, shared by TVC and fins): per channel clip to the command limit ->
transport delay -> first-order lag (exact exponential) -> rate limit -> angle limit. The delay is applied at step
boundaries (a delayed command is released at the first step at or after its due time), so it can be late by up to one step;
the tests use aligned steps. **Hold:** the plant sees the actuator state at the *start* of each step (left-endpoint hold), i.e. an
effective extra lag of about h/2 and a one-step response even for tau = delay = 0; in a fin-controlled run peak deflection
varied ~5 % with dt and controller rate (apogee < 0.05 m). Not changed (it would alter physics and the holdout fingerprint). Not modelled: backlash, deadband,
load-dependent torque limit, structural compliance.

*Control surfaces* (`rocket.control_surfaces`, canards or tail fins with their own geometry, deflection, max angle, max rate,
lag and delay): each fin is a static lifting surface (Barrowman/Helmbold slope) whose deflection delta_i gives a normal force
`L_i = q S CNa_fin(M) delta_i` along n_i = (0, -sin phi_i, cos phi_i) at the fin aerodynamic centre, and an induced drag
`-|L_i delta_i|`; the resulting moment is `r_i x F_i` about the instantaneous CG (pitch, yaw **and roll**). A `FinMixer`
maps body-axis commands to individual deflections (matched filter; orthogonal channels for evenly spaced fins). The
attitude controller can actuate `tvc`, `fins` or `both`. Verified: single-fin force/moment analytics, sign symmetry,
mixer decoupling, saturation/rate limits, and a closed-loop run in which fins hold an otherwise tumbling (SM < 0.2 cal)
vehicle within 6 deg of vertical. **Not validated against any real fin-control data; no fin flutter, no hinge moment, no
fin-body interference of the movable surfaces beyond the static fin set.**

## Recovery

Parachutes: drag `1/2 rho V^2 Cd A f(t)` along -v_rel with linear inflation ramp; triggers: apogee (+delay) or
descending through an altitude (+delay). In 6-DOF the canopy load acts at a shock-cord attachment point, so the airframe
hangs from the canopy instead of tumbling. Simplified: rigid attachment, no canopy dynamics, no line stretch, no
opening shock beyond the ramp, one attachment point shared by all parachutes.

## Ground and impact

Terrain height `h(x, y)` (flat, planar slope, or grid); the first crossing of the ground after liftoff terminates the
run. The impact state is the root-finder state at the crossing (impact speed, vertical speed, landing position).
No ground rebound, tip-over or slide.

## Sensors, estimation, control

See [sensors.md](sensors.md) for the audited list. Key honesty points: the estimator is a *linear* Kalman filter on
position/velocity with gyro-integrated attitude (TRIAD alignment on the pad with the configured reference magnetic field,
pad gyro-bias estimation; no in-flight accelerometer/magnetometer attitude correction); it compensates known sensor
latency; it is **not** an EKF (an EKF is a roadmap item; the `Estimator` interface is the extension point, and there is also
a `truth` estimator for development). Default sensor parameters are representative MEMS orders of magnitude, not a
datasheet. The attitude controller is a PD pointing law with gain scheduling from the nominal motor curve.

## Change log (physics versions)

| version | change | evidence |
|---|---|---|
| 1.0.0 | initial release: Barrowman base drag `0.12 + 0.13 M^2`; roughness floor without compressibility | baseline flight errors (geometric comparison): apogee -4.4 / +0.2 / -11.1 % (Bella Lui / NDRT / Prometheus) |
| 1.1.0 | Hoerner subsonic base drag blended to Barrowman supersonic; Raymer roughness cut-off with compressibility on both smooth and rough Cf | Prometheus Cd(M) in v1.0.0 *rose* 0.45->0.55 over M 0.1-0.85 while the team's RASAero curve falls 0.42->0.30. After the change: apogee -2.3 / +8.7 / +0.03 % (geometric) or +0.5 / +13.4 / -4.9 % (barometer-equivalent, the like-for-like comparison). Prometheus is in-sample. Details and what the NDRT regression means: `docs/validation.md` |
| 1.2.0 | Aero model hierarchy (simplified/barrowman/enhanced/table/table2d) behind a CD/CL/CM(M, alpha, Re) interface; geometry-built CG/inertia (full tensor path); control-surface forces from the ACTUAL actuator state; crossflow drag acts at the planform centroid (was a fixed fraction of length); pad gyro-bias estimation | The only intended change to existing results is the crossflow centroid: fidelity-3 reference apogee 918.562 -> 918.465 m (-0.01 %); fidelity 2 unchanged; the three validation flights are unchanged to the printed precision (+0.50 / +13.41 / -4.90 %). New golden numbers in `tests/test_golden.py` |
| 1.2.1 | `evaluate` memo key now includes the control-fin deflections (stage 1 of RK4 used the previous step's fins); the actuator output is copied to the dynamics right after each actuator step, so the force evaluation and the logged row at the next time see the actuator state at that time (they were one step stale) | found by critic review 2. Only flights with an active actuator command change: golden numbers (no control) are bit-identical; closed-loop fin test apogee 485.5658 -> 485.5657 m. Holdout flights have no control surfaces: results re-verified (`--migrate-fingerprint`) |

Datasets generated with different physics versions are not comparable; the version is in every manifest and runs table.
