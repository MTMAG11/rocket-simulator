# Vehicle geometry, mass properties, control surfaces and TVC

The recommended way to define a vehicle is a `vehicle.json` ([vehicle_format.md](vehicle_format.md), `rocketsim vehicle FILE`). The
sections below describe what the simulator does with the description; they also apply to a hand-written `rocket:` section.

## Component-based airframe

`rocket.sections` is a list stacked from the nose tip aft; each entry is a `nose`, `body`, `transition` or `boattail` (a
transition that narrows) with length, diameters and optionally mass and CG fraction. With `rocket.fins` (trapezoidal set),
optional `rocket.control_surfaces` (movable fins), extra rigid `rocket.masses` and the motor, this defines everything the
aerodynamic and mass models need. Sections must be contiguous with continuous diameters, fins must lie inside the body, and the
CG/inertia sources must be consistent. Example: `configs/example_components.yaml`.

The legacy single-diameter description (`body_diameter_m`, `body_length_m`, `nose`, `dry_mass_kg`, `cg_from_nose_m`) is converted to the
same assembly (`legacy_assembly`) and reproduces the original results exactly.

The geometry feeds wetted and planform area (drag build-up, crossflow), local radius (fin position), Barrowman CNa/CP per section
(a boat-tail gives negative normal force), base area (boat-tail drag), static margin and the reference diameter.

## Centre of gravity and inertia

* CG = sum(m_i x_i)/m, and the inertia tensor is computed with the parallel-axis theorem from each component's own inertia, lateral
  offsets `y, z` and products `ixy, ixz, iyz`.
* Propellant mass follows the thrust-curve impulse; the propellant CG is the motor centre, so the CG moves forward during the burn.
* When all lateral offsets and off-diagonal terms are zero an axisymmetric fast path is used; otherwise the general tensor and
  Euler equations `I w_dot = M - w x (I w)`. Both paths are tested to agree.
* Inertia comes from measured/CAD values (`rocket.inertia`) if given; otherwise thin-shell/thin-tube estimates, flagged in the vehicle
  notes and run warnings. Propellant is a solid cylinder.
* Tests (`tests/test_geometry_mass.py`): hand-computed Barrowman, boat-tail negative CNa, assembly validation, CG from components and
  its burn shift, cylinder formulas, parallel axis, tensor vs independent summation, fast path == general path, Euler invariants
  (|H|, kinetic energy) for an asymmetric body.

## Centre of pressure and static margin

CP comes from the assembly's Barrowman build-up and is logged with the CG each step. Aerodynamic moments use the force CP of the
active model (Mach-dependent for `enhanced`); the static margin uses the subsonic CP. Moments are `r x F` about the instantaneous CG,
including lateral CG offsets.

## Actuator chain

`command -> [compute + downlink latency] -> saturate -> transport delay -> first-order lag -> rate limit -> angle limit -> actual
state -> force/moment`. The delay is communication latency before the actuator sees the command; lag and rate limit are the
actuator's own dynamics, so the actuator slews toward the delayed command. The physics reads `actuator.state` only. One
`ActuatorBank` serves TVC (2 channels) and control fins (N channels).

## TVC

Thrust vector `T (cos th_z cos th_y, sin th_z, -cos th_z sin th_y)` in the body frame about the fixed misalignment, applied at the
nozzle exit. Gimbal sign conventions are tested for zero, positive, negative and maximum deflection and for combined axes; angle
limit, rate limit, lag and delay are compared with closed forms. The commanded value is saturated before the actuator and logged
(`tvc_cmd_*`) next to the actual deflection (`tvc_*`).

## Control surfaces (fins/canards)

`rocket.control_surfaces`: `count`, root/tip chord, span, sweep, thickness, `position_from_nose_m`, `roll_angle0_deg`,
`max_deflection_deg`, `max_rate_deg_s`, `time_constant_s`, `delay_s`, `mass_kg`. Model: [physics.md](physics.md#thrust-vector-control-and-aerodynamic-control-surfaces).
Columns: `fin_cmd_{roll,pitch,yaw}` and the actual `fin_0..fin_{n-1}` deflections (only for vehicles with control surfaces).

Closed-loop check (`tests/test_control_v11.py::test_closed_loop_control_surfaces_stabilise_an_unstable_vehicle`): a 0.45 kg vehicle with
0.6 deg thrust misalignment and about zero static margin tumbles to 178 deg uncontrolled (apogee 60-67 m); with fin control
(`actuation: fins`, wn 8 rad/s, roll damper) it stays within 3-6 deg (apogee 542 m). Fin authority scales with dynamic pressure, so
fins cannot stabilise the vehicle on the rail or at low speed; TVC remains the primary actuator there.

## Limits

* The legacy single-diameter path places the fin-set mass at the leading edge plus half the mean chord, while the component path
  adds the sweep centroid term. Changing the legacy path would change validated results, so the difference is left in place.
* Only motor propellant changes mass in flight; there is no per-component mass-changing flag, motor-mount geometry or off-axis motor.
* No pods/strakes/launch lugs (use `extra_cd`), one trapezoidal fin set plus movable sets, no fin flutter or hinge moments, no
  actuator backlash/deadband/torque limit, no structural flexibility, propellant CG fixed at the motor centre.
