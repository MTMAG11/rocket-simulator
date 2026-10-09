# Derivation: what a thrust-vector gimbal can correct (Q-01 trade study)

The study compares motors for a TVC vehicle by asking how much disturbance the gimbal can cancel while the motor burns, and for
how long. This note derives the torque the gimbal produces and defines every quantity the report shows.

## 1. Torque from a deflected thrust vector

Take the body x axis along the rocket (nose forward) and the gimbal pivot at the nozzle exit, a distance ℓ aft of the centre of
gravity. The motor pushes with thrust T along its own axis. With the gimbal deflected by an angle δ from the body axis, the thrust
vector has an axial component T cos δ (still pointing along the body, so about the CG it produces no torque) and a lateral
component T sin δ at the nozzle.

The torque about the CG is the lateral force times its lever arm:

    τ = T sin δ · ℓ                                                     ← the *maximum* of this over δ is your TODO

so the largest torque the gimbal can apply at a given moment is, with δ_max the mechanical limit,

    τ_max(t) = T(t) sin(δ_max) ℓ(t)      (`max_correctable_torque(thrust, lever_arm, delta_max)`)

T(t) comes from the thrust curve; ℓ(t) = x_nozzle − x_cg(t) grows as the CG moves forward when propellant burns (the CG
moves toward the nose, away from the nozzle). Both are in the simulation output (`thrust`, `cg`) and in the vehicle (`nozzle_x`).

Hand-calculated check used by the test: T = 40 N, ℓ = 0.30 m, δ_max = 6°: sin 6° = 0.104528, τ_max = 40 × 0.104528 × 0.30 = 1.2543 N·m.
(Units: N × m = N·m. If δ is in degrees the sine needs radians.)

Dividing by the pitch moment of inertia I_yy gives the largest angular acceleration the gimbal can command, τ_max / I_yy [rad/s²]: the
same torque is a much larger acceleration on a light vehicle, which is why the added gimbal and servo masses matter.

## 2. The other reported quantities

* **Rail exit speed**: the speed when the vehicle leaves the rail, from the simulation (it depends on the thrust curve, the mass and
  the rail length). A guided flight needs enough airspeed at rail exit for the fins and the sensors to be useful; the minimum is
  an input you set, not a number this repository knows.
* **Thrust-to-weight**: peak thrust over the liftoff weight, T_peak / (m g).
* **Authority time**: the seconds between rail exit and burnout (the gimbal only works while the motor thrusts). The study can also
  report the time during which τ_max exceeds a reference disturbance torque that you supply; no default is invented.
* **Success rate** (closed-loop Monte Carlo): the fraction of runs that satisfy the success definition in the study file (below),
  with a Wilson 95 % interval. Runs vary wind, a thrust misalignment of fixed magnitude and random direction, an axial CG error
  (a chosen mass is moved so the total CG shifts by the stated amount, with a random sign), and sensor noise.

## 3. The success definition (proposed; change it in the study file)

A run succeeds when

1. the nose stays within `max_tilt_deg` of vertical from rail exit to `settle_s` after burnout; and
2. the rail exit speed is at least `min_rail_exit_speed_ms` (omitted if not set).

The defaults in the template are marked *proposed*: they are choices for you to confirm, not facts about your vehicle. With a
vehicle that is statically stable and flies passively, criterion 1 would also pass without any control; the study therefore also
runs every disturbance case without the controller (a passive reference), so the report shows what the gimbal adds.

## 4. What the numbers do not say

* The Monte Carlo covers the dispersions you list. Anything else (a servo that jams, a gust stronger than the sampled wind, a
  structural failure, wind shear) is outside the success rate.
* A success rate from tens of runs has a wide interval: 28 of 30 is 93 % with a 95 % interval of about 79–98 %.
* The comparison is only as good as the servo parameters (delay, lag, rate limit, backlash from the bench fit) and the added
  masses; with placeholders the result says nothing about the real vehicle.
