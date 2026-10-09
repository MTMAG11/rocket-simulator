# Derivation: drag coefficient from the coast phase of a flight log

After burnout and before apogee the only forces on the rocket are gravity and aerodynamic drag (the parachute is not out and the
motor is off). Drag is the one unknown, so the deceleration of the rocket along its path gives C_D directly. This note derives
the relation, what is measured, what is assumed and where the error comes from.

## 1. Equation of motion along the flight path

Let v be the speed, γ the flight-path angle above the horizontal (90° for a vertical flight), m the (constant) mass, g the local
gravity, S the reference area (π d²/4 for the body diameter d) and ρ the air density. Drag opposes the velocity:

    D = ½ ρ v² S C_D

Newton along the velocity direction:

    m dv/dt = − D − m g sin γ

Solve for the drag-induced deceleration a_D = D/m = −dv/dt − g sin γ (positive number). Then

    ½ ρ v² S C_D = m a_D      ⇒      C_D = 2 m a_D / (ρ v² S)         ← **your TODO** (`solve_cd`)

Hand-calculated check used by the test: m = 0.600 kg, ρ = 1.000 kg/m³, v = 100 m/s, S = 0.004 m², a_D = 20 m/s²:
C_D = 2 · 0.6 · 20 / (1 · 100² · 0.004) = 24 / 40 = 0.600.

## 2. Two ways to get a_D from a log

* **Barometer altitude only.** Differentiate the altitude: v_z = dh/dt, a_z = d²h/dt². For a vertical coast γ = 90°, v = |v_z| and
  dv/dt = a_z, so a_D = −a_z − g (while rising: a_z is negative and larger in magnitude than g). Differentiating noisy altitude
  amplifies noise (the second derivative most of all), so the tool smooths with a Savitzky–Golay filter before differentiating;
  the window length trades noise against bias and is part of the reported uncertainty.
* **Accelerometer.** A strapdown accelerometer measures *specific force*, the non-gravitational acceleration: during coast that is
  the drag deceleration along the body axis, f_x ≈ −a_D (when the angle of attack is small, so drag is along the body axis).
  Then a_D = −f_x. Gravity drops out, which is why this is much less noisy than the barometer route, but it needs the
  accelerometer scale and bias to be right and the axis to be aligned with the flight path.

Both routes feed the same `solve_cd`.

## 3. Mach number and density

Density and speed of sound come from the ISA atmosphere at the sample's altitude (site elevation and temperature offset are
inputs, not measured: a 10 K temperature error changes ρ by about 3 % and so C_D by about 3 %). Mach = v / a(h). The result is
C_D versus Mach, binned.

## 4. Where the uncertainty comes from

* **Random (noise).** Bootstrap: resample the samples inside each Mach bin (in blocks, because neighbouring samples are correlated
  after smoothing) and recompute the bin mean C_D; the spread is the random error.
* **Systematic.** Relative errors in m, S and ρ add in quadrature: (σ_C/C)² = (σ_m/m)² + (σ_S/S)² + (σ_ρ/ρ)². The tool takes
  these as inputs; it has no way to know them.
* **Method.** γ ≠ 90° (a tilted coast) makes the vertical-flight assumption wrong: the error is of order
  g (1 − sin γ) / a_D. Angle of attack ≠ 0 means drag is not along the body axis (accelerometer route) and the induced drag is in
  C_D (that is correct physically but it is not the zero-lift drag).
* **Not separable.** Using the same coast data one cannot tell skin friction from form drag from base drag, only their sum at that
  Mach number; and a coast of ~10 s covers a narrow Mach range (a 3-inch rocket on a G motor stays below Mach 0.3).

## 5. What the tool will say

C_D(Mach bin) ± random ± systematic, the number of samples per bin, and the coast window used. It refuses to report a bin with too
few samples. A single Mach bin is all you get for a low-speed flight; do not read a "curve" into it.
