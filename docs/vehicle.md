# Vehicle geometry, mass properties, control surfaces and TVC

## 0. Vehicle file (V1.2)

The recommended way to define a vehicle is the versioned **vehicle file** (`vehicle.json`): a list of physical components with
mass, position, dimensions, material and optional measured inertia tensor, from which everything is derived
([vehicle_format.md](vehicle_format.md), `rocketsim vehicle FILE`). The sections below describe what the simulator does with the
description; they apply equally to a `rocket:` section written by hand.

## 1. Component-based airframe

`rocket.sections` is a list stacked from the nose tip aft; each entry is a `nose`, `body`, `transition` or `boattail`
(a transition that narrows) with its length, diameters and (optionally) mass and CG fraction. Together with `rocket.fins`
(trapezoidal set), optional `rocket.control_surfaces` (movable fins), extra rigid `rocket.masses` and the motor, this
defines everything the aerodynamic and mass models need. Validation: sections must be contiguous, diameters continuous (a
section starts at the previous aft diameter), fins must lie inside the body, and the CG/inertia sources must be consistent.
See `configs/example_components.yaml` (nose + payload bay + transition + motor section + boat-tail).

The legacy single-diameter description (`body_diameter_m`, `body_length_m`, `nose`, `dry_mass_kg`, `cg_from_nose_m`) is
internally converted to the same assembly (`legacy_assembly`), and reproduces V1 results exactly.

**What the geometry feeds**: wetted area and planform area (drag build-up, crossflow), local radius (fin interference,
fin position), Barrowman CNa/CP per section (transitions and boat-tails give correct *negative* normal force
for a boat-tail), base area (boat-tail drag), static margin (CP from geometry), reference diameter.

## 2. Centre of gravity and inertia

* With sections/masses carrying mass: **CG = sum(m_i x_i)/m** and the inertia **tensor** are computed (parallel-axis
  theorem, each component's own principal inertia, lateral offsets `y, z`, off-diagonal terms `ixy, ixz, iyz`).
* Propellant mass follows the thrust-curve impulse; propellant CG is the motor centre (fixed) - **the CG therefore
  shifts aft-to-forward during the burn** (tested: component-built CG moves by the hand-computed amount).
* An axisymmetric fast path (V1-identical results) is used when all lateral offsets and off-diagonals are zero; otherwise the
  general tensor path and the general Euler equations `I w_dot = M - w x (I w)` are used. Both are tested to agree.
* Inertia source: measured/CAD (`rocket.inertia`) if given; otherwise thin-shell/thin-tube estimates *flagged in the vehicle
  notes and the run warnings*. Propellant: solid cylinder.
* Tests (`tests/test_geometry_mass.py`): Barrowman by hand, telescoping invariant, boat-tail negative CNa, assembly
  validation errors, CG from components and its burn shift, cylinder formulas, parallel axis, tensor vs an independent
  summation on random components, propellant in the tensor, fast path == general path, Euler invariants (|H|, kinetic energy)
  for an asymmetric body, general vs axisymmetric Euler.

## 3. Centre of pressure and static margin

CP from geometry (assembly Barrowman), logged with the CG each step; the aerodynamic **moments** use the force CP of the
active model (Mach-dependent for `enhanced`), the *static margin* uses the subsonic CP. Moments are computed as `r x F`
about the instantaneous CG including lateral CG offsets, so a lateral CG offset produces a roll/pitch moment from thrust
and aerodynamic forces.

## 3b. Actuator chain (V1.2 statement)

`commanded state -> [compute + downlink latency] -> saturate -> transport delay -> first-order lag -> rate limit -> angle limit ->
actual state -> physical force/moment`. The conceptual order in the V1.2 brief (rate limit before delay) differs from the implemented
order (delay before lag and rate limit): the delay is the *communication* latency before the actuator sees the command, while lag and
rate limit are the actuator's own dynamics, so the actuator slews toward the delayed command. The physics reads `actuator.state`
only. One reusable `ActuatorBank` serves TVC (2 channels) and control fins (N channels); future servos would be another bank with their own limits.

## 4. TVC (audited in V1.1)

Thrust vector `T (cos th_z cos th_y, sin th_z, -cos th_z sin th_y)` in the body frame about the fixed misalignment; applied at
the nozzle exit. Audit findings: the physics uses the *actuator output*, not the command; gimbal sign conventions are
tested for zero / + / - / maximum deflection and for combined axes; max angle, rate limit, lag and delay are exact (tests
compare against closed forms); the commanded value is saturated before the actuator and the saturated command is logged
(`tvc_cmd_*` columns) next to the actual deflection (`tvc_*`).

## 5. Control surfaces (fins/canards)

Configuration (`rocket.control_surfaces`): `count`, root/tip chord, span, sweep, thickness, `position_from_nose_m`,
`roll_angle0_deg`, `max_deflection_deg`, `max_rate_deg_s`, `time_constant_s`, `delay_s`, `mass_kg`. Model: see
[physics.md](physics.md#thrust-vector-control-and-aerodynamic-control-surfaces). Logged columns: `fin_cmd_{roll,pitch,yaw}` and the actual
`fin_0..fin_{n-1}` deflections (present only for vehicles with control surfaces).

**Closed-loop result (experiment, reproduced by `tests/test_control_v11.py::test_closed_loop_control_surfaces_stabilise_an_unstable_vehicle`)**:
a 0.45 kg vehicle with a 0.6 deg thrust misalignment and the CG moved so the static margin is ~0 flies uncontrolled to 178 deg
(tumble, apogee 60-67 m); with fin control (`actuation: fins`, wn 8 rad/s, roll damper) it stays within 3-6 deg (apogee 542 m).
Fins with span 0.05 m lacked authority at low speed: authority scales with `q`, so fin control cannot stabilise the
vehicle on the rail and in the first moments of flight - an intrinsic property, reproduced by the model, that is why TVC remains
the primary actuator at low speed.

## 6. Limits

* **Known inconsistency**: the legacy single-diameter path places the fin-set mass at leading edge + half the mean chord, while the component path
  (sections / vehicle file) adds the sweep centroid term. Changing the legacy path would change validated results, so it is documented, not changed.
* Only the motor propellant changes mass in flight; there is no per-component mass-changing flag, no motor-mount geometry and no off-axis motor.

No pods/strakes/launch lugs (use `extra_cd`), single trapezoidal fin set plus movable sets, no fin flutter or hinge-moment
model, no actuator backlash/deadband/torque limit, no structural flexibility, propellant CG fixed at the motor centre, no
sloshing (solid motors).
