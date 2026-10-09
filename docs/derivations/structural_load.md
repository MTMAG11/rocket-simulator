# Derivation: structural load screen (q·α and lateral aerodynamic load)

The simulator reports two numbers per run and compares them with limits you configure (`structure.max_q_alpha_pa_rad`,
`structure.max_lateral_load_n`). It ships no limits: the right values depend on your airframe (tube, joints, fin attachment), so a
limit is an input you measure or justify. The screen only says "this run exceeded the limit you set".

## 1. Why q·α

Dynamic pressure is q = ½ ρ v² [Pa]. For small total angle of attack α [rad] the aerodynamic normal force on a slender body is

    N ≈ q S C_Nα α

so N is proportional to q·α, with S C_Nα fixed by the geometry. Peak q·α therefore ranks how hard the airframe was bent across
flights of the same vehicle without needing the coefficient. Units: Pa·rad. At large α the relation is no longer linear (the
simulator's normal force then includes the crossflow term), which is why section 2 exists.

Implemented: `q_alpha(q, alpha) = q |alpha|`.

## 2. The check window

Both quantities are taken only where the airflow defines α: before apogee (phase < 5) and above 20 m/s airspeed (the same cut-off
as the aerodynamic range warning). Parachute opening shock and landing loads are not covered.

## 3. Lateral (normal) force from the logged lift and drag  ← your TODO

The telemetry logs the aerodynamic force in wind axes: **drag** D along −v (opposing the relative wind) and **lift** L normal to v
in the plane containing the body axis. A structure feels the force resolved on the body axes: the **axial** component A along the
body axis and the **normal** (lateral) component N perpendicular to it. The two frames differ by the angle of attack α.

Work it out:

1. Draw the body axis x_b and the relative-wind direction v̂ at angle α to it, in the plane of the two.
2. Write the unit vectors "along −v̂" and "normal to v̂ (in the plane)" in components along x_b and the perpendicular y_b.
3. The force is F = D (−v̂) + L (n̂). Project it on x_b and y_b. The y_b component is N, the (negative) x_b component is A.

Check the result with the aerodynamic definitions used in `vehicle/aero.py`: C_D = C_A cos α + C_N sin α and
C_L = C_N cos α − C_A sin α (the module docstring has them). Inverting those two equations expresses C_N in terms of C_D, C_L
and α, which is the same relation as your N in terms of D, L and α.

Hand-calculated check used by the test: L = 50 N, D = 100 N, α = 10°.

    cos 10° = 0.984808, sin 10° = 0.173648

Compute N from your formula; it must equal 66.605 N (to three decimals: 66.6052). Also compute A for yourself and confirm
N² + A² = L² + D² (a rotation does not change the magnitude), which is a quick sanity check on signs.

Implement `body_normal_force(lift, drag, alpha)` in `src/rocket_sim/simulation/structure.py`. Until then the lateral check is
reported as not available (and a warning says so if you set `structure.max_lateral_load_n`). The chute drag is removed before the
call (`drag − chute_drag`), but before apogee it is zero anyway.

## 4. What this does not tell you

* Not a stress analysis: no bending moment distribution, no joint or fin-root load, no fatigue.
* q·α and N are peaks over the sampled steps; a short transient between samples can be missed if the time step is coarse.
* The simulator's own aerodynamic model is an estimate at large angles of attack (see aerodynamics.md), so a run with α near
  180° (tumble) is flagged but its load values are not predictions.
