# Derivation: servo / TVC actuator model with delay, lag, rate and acceleration limits, deadband and backlash

Everything the physics sees is the actuator's *output angle* θ_out, never the command (see `vehicle.md`). This note derives the
discrete-time model in `control/actuators.py` and states what each parameter means, so that the fitting tool
(`control/servo_fit.py`) fits the same quantities the simulation uses.

Signal chain for one channel, commanded angle c(t) → output angle θ_out(t):

    c → [saturate ±θmax] → [transport delay T_d] → [dead zone δ] → [first-order lag τ] → [rate limit ω_max]
      → [acceleration limit α_max] → [angle limit ±θmax] → shaft angle θ_s → [backlash, gap 2b] → θ_out

## 1. Transport delay

The servo acts on c(t − T_d). In discrete form each command is queued with a due time t_k + T_d and becomes the *effective*
command c_eff when the clock passes it (commands keep their order). T_d is the pure latency of the electronics and signal path;
it shifts the response and does not change its shape.

## 2. First-order lag, exact discretisation

A position servo with a proportional drive behaves, away from limits, like

    τ dθ/dt = c_eff − θ,   θ(t_k) = θ_k   (c_eff constant over the step h)

whose exact solution is θ(t_k + h) = c_eff + (θ_k − c_eff) e^(−h/τ), i.e.

    θ_{k+1} = θ_k + (c_eff − θ_k) (1 − e^(−h/τ)).

The gain g = 1 − e^(−h/τ) lies in (0, 1) for every h > 0, so the update is stable for any step size (an explicit Euler step
h/τ would not be). For τ = 0 the servo is ideal: g = 1. Time constant check: after one τ a step reaches 1 − e^(−1) = 63.2 %.

## 3. Rate limit and acceleration (torque) limit

A real servo cannot slew faster than its motor and gearing allow, ω_max [rad/s]. The lag increment is clipped:
|Δθ| ≤ ω_max h. This is what turns a large step into a ramp of slope ω_max: if the step is small enough that
|c − θ|/τ < ω_max the limit never acts, and the rate limit **cannot be identified** from that data (the fitting tool says so).

Torque limit: with a motor torque limit T_max acting on a load of inertia J (horn, gimbal, nozzle), the angular acceleration is
bounded by α_max = T_max / J. The model takes α_max directly (`max_accel_deg_s2`), because J is a property of your gimbal that
this repository does not know; measure or compute it, then α_max = T_max / J. Implementation: the shaft keeps its angular
velocity ω_k; the velocity the lag would like, ω* = Δθ / h, is limited to ω_{k+1} ∈ [ω_k − α_max h, ω_k + α_max h] (and to
±ω_max), and θ_{k+1} = θ_k + ω_{k+1} h. With α_max omitted (default) this path is skipped and the model is identical to the
three-element chain above.

## 4. Dead zone (deadband)

Hobby-servo electronics do not drive the motor while the position error is smaller than a threshold δ (to avoid hunting). With
e = c_eff − θ_s, the drive error is

    e_d = 0             if |e| ≤ δ
    e_d = e − sign(e) δ otherwise

and the lag acts on e_d. Consequence: the servo stops when it is within δ of the command; a command change smaller than δ does
nothing. δ = 0 recovers the pure lag.

## 5. Backlash (gear play)

Between the motor-side shaft and the horn there is play. Let the total gap be 2b (b is the half-gap). While the shaft moves
inside the gap the output does not move; when the shaft reaches either edge it pushes the output ahead of it. This is the *play*
(backlash) operator. In terms of the shaft angle θ_s and the previous output θ_out⁻:

    if the output is inside the gap, θ_out = θ_out⁻ ;  if the shaft moves past an edge, the output is carried to that edge.

**Your TODO:** implement `play_operator(shaft, output_prev, half_gap)` in `src/rocket_sim/control/actuators.py` from that
description (it is a two-line piecewise function of the difference θ_s − θ_out⁻ and b). Hand-calculated check used by the test,
with b = 0.5°:

| shaft θ_s | previous output | expected output |
|---|---|---|
| 2.0° | 0.0° | 1.5° (shaft at the +b edge of the gap) |
| 1.8° | 1.5° | 1.5° (inside the gap: no motion) |
| 0.5° | 1.5° | 1.0° (shaft reversed past the −b edge) |
| −3.0° | 1.0° | −2.5° |

Model simplification: the feedback that closes the servo loop is taken at the motor shaft, so backlash does not make the shaft
hunt; it only delays and shifts the output on every reversal. If your servo's potentiometer is on the output shaft the real
behaviour is a limit cycle or a larger effective dead zone, which this model does not reproduce.

## 6. What the fit can and cannot recover

The fitting tool minimises Σ (θ_meas − θ_model)² over (T_d, τ, ω_max, b, δ), with the covariance estimated as
σ² (JᵀJ)⁻¹, σ² = RSS/(n − p), from the Jacobian J of the model. Identifiability, so you design the bench test accordingly:

* T_d and τ need a step: the dead time and the 63 % time.
* ω_max needs a step large enough to saturate the rate. The tool compares the best fit with the same fit without a rate limit
  (chi-square difference < 4, about 2σ); if the limit adds no evidence it reports "not identifiable" and no limit (∞).
* δ needs small steps or a slow ramp (the stopping error is visible).
* b needs direction reversals (up-step then down-step): the lag between the shaft and the output on the reversal.
* The piecewise-linear nonlinearities make the least-squares surface non-smooth; the tool starts from several initial guesses and
  reports whether they agree.
