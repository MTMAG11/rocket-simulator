# Derivation: error-state EKF (attitude, position, velocity, gyro and accelerometer bias)

`estimator.type: ekf` runs next to the linear Kalman filter (`nav_kf`). Both take only sensor samples. The linear filter keeps
attitude from gyro integration outside the filter (so its covariance knows nothing about attitude error, and a gyro bias makes
the attitude drift unobserved). The EKF puts the attitude error and both sensor biases into the state so that GPS and barometer
data can correct them, and the covariance says how sure it is.

## 1. Nominal state and error state

Nominal (large-signal) state, propagated with the exact nonlinear equations:

    q   unit quaternion, body → launch frame (Hamilton, [w, x, y, z])
    p   position in the launch frame (East, North, Up), v velocity, b_g gyro bias, b_a accelerometer bias

True = nominal + error. Additive for p, v, b_g, b_a. For the attitude the error is a small rotation applied on the right,

    q_true = q ⊗ δq,   δq ≈ [1, δθ/2],   δθ ∈ R³ (small angle, expressed in the body frame).

The **error state** is x = [δθ, δv, δp, δb_g, δb_a] (15 numbers). A rotation has three degrees of freedom; using δθ instead of the
four quaternion components avoids a singular covariance (the unit-norm constraint) and keeps the filter linear in a small error.
After every measurement update the error mean is moved into the nominal state and reset to zero:

    q ← q ⊗ [1, δθ/2] (normalised),  v ← v + δv,  p ← p + δp,  b_g ← b_g + δb_g,  b_a ← b_a + δb_a.

## 2. Propagation (one IMU sample, step dt)

Measured gyro ω_m = ω + b_g + n_g, measured specific force f_m = f + b_a + n_a. Nominal propagation:

    ω̂ = ω_m − b̂_g,           q ← q ⊗ exp(ω̂ dt)              (exact quaternion exponential, as in `integrate_gyro`)
    â = R(q) (f_m − b̂_a) + g,   v ← v + â dt,   p ← p + v dt + ½ â dt²        g = (0, 0, −g)

Error dynamics (first order in the error): linearise the true equations about the nominal and subtract.

    δθ̇ = −[ω̂]× δθ − δb_g − n_g
    δv̇ = −R [f_m − b̂_a]× δθ − R δb_a − R n_a
    δṗ = δv
    δḃ_g = n_bg,   δḃ_a = n_ba          (random walks)

where [a]× is the skew matrix (a × b = [a]× b). The attitude error couples into velocity through the *specific force*: a tilt error
makes the thrust acceleration point the wrong way. That is the only way velocity/GPS data can reveal the tilt, and it needs a
nonzero specific force (it is weak in coast, where f ≈ 0, which is why the attitude becomes unobservable then).

Discrete: Φ = I + F dt (F the 15×15 matrix from the equations above), P ← Φ P Φᵀ + Q. Q is built from the sensors' noise
densities: gyro noise σ_g, accelerometer noise σ_a (each per-sample standard deviations scaled by √dt), and the bias random-walk
standard deviations ("per √s") from the sensor configuration. The filter uses the nominal sensor configuration as its model of the
noise, never the true noise realisation; `estimator.noise_scale` multiplies the assumed noise to study a mismatch (for example a
vibrating airframe whose accelerometer noise is larger than the datasheet).

### Delayed measurements and the unmodelled accelerometer error

* **Latency.** The GPS (0.2 s by default) and barometer data describe the state at the time they were taken. The filter keeps the
  last second of its own [p, v] estimates and forms the innovation against the estimate *at the measurement time*, then applies the
  correction to the current state. This needs no model of the acceleration during the delay (the linear filter extrapolates with its
  latest acceleration, which is wrong near burnout where the acceleration drops by about 150 m/s^2 within a few ms). It is an approximation:
  the error covariances of the past and present states are taken as equal.
* **Accelerometer scale factor and misalignment** are not states, but they corrupt the integrated velocity in proportion to the
  specific force (0.2 % of 150 m/s^2 is 0.3 m/s^2). The process noise therefore includes (k |f|)^2 with k from the configured
  `scale_error_std` and `misalignment_std_deg`. Without this term the filter was overconfident (velocity error about 2.5 m/s with a
  reported sigma of 0.05 m/s); with it the normalised error is consistent (see the NEES test).

## 3. Measurement updates

For a measurement z = h(x_nominal) + H x + noise with innovation y = z − h(x̂) the update is the standard one,

    S = H P Hᵀ + R,   K = P Hᵀ S⁻¹,   δx = K y,   P ← (I − K H) P (I − K H)ᵀ + K R Kᵀ   (Joseph form),

then inject δx and reset.

* **Barometer altitude** z = h_baro: H has a 1 in the δp_z column and zeros elsewhere. Implemented.
* **GPS position and velocity** (launch frame): H picks δp (3 rows) and δv (3 rows). Implemented. Latency is handled as in the
  linear filter (the delayed fix is propagated forward with the filter's own acceleration).
* **A vector measured in the body frame** (the magnetometer's Earth field before/after burn, or the accelerometer's gravity
  direction on the pad): z_b = R(q_true)ᵀ m_L, with m_L the known reference vector in the launch frame. With
  R_true = R(q) R(δθ) and R(δθ) ≈ I + [δθ]×, and the predicted measurement ẑ_b = R(q)ᵀ m_L,

        z_b = R(δθ)ᵀ ẑ_b ≈ ẑ_b − [δθ]× ẑ_b        and        z_b ≈ ẑ_b + H_δθ δθ,   H_δθ = ?        ← **your TODO**
                                                                                      (`attitude_measurement_jacobian`)

  Get the 3×3 block by moving δθ to the right of the cross product (hint: a × b = −(b × a), i.e. [a]× b = −[b]× a). Hand-calculated
  checks used by the test: ẑ_b = (0, 0, 1) gives a block with nonzero entries only in its first two rows; ẑ_b = (1, 2, 3) gives a
  block whose rows you can write down from the definition of [·]× (the sign is fixed by your derivation).
  Without this function the magnetometer update is switched off (`estimator.ekf_use_magnetometer: false`, the default), and the
  filter is exactly the GPS/barometer filter described above.

## 4. Initial condition

The pad alignment is the same TRIAD alignment as the linear filter (stationary accelerometer = "up", magnetometer = heading,
configured reference field), giving q₀, and the stationary gyro mean as the initial b̂_g. The initial covariance is set from the
alignment accuracy: a tilt/heading standard deviation (placeholder values in the code, marked as such), the gyro bias
standard deviation from `bias_std`, and a position/velocity of zero uncertainty on the pad. Before launch detection the
position and velocity are held at the pad values (zero-velocity update), as in the linear filter.

## 5. What to expect, and what not to

* With GPS and baro, tilt is observable during the burn through δv̇ = −R[f]×δθ; in coast it is not, so the attitude covariance
  grows again. Gyro bias is observable only through the attitude it corrupts.
* If both aiding sensors drop out, every error state grows with the IMU noise and bias; no filter can prevent that.
* An EKF linearises about its own estimate: a large attitude error (the vehicle tumbling) violates the small-angle assumption.
* "Better than the linear KF" is a claim to be measured, not assumed: `scripts/compare_estimators.py` runs both on the same
  simulated flights (same seeds, so identical sensor noise) and reports the errors against truth, including baro and GPS dropout,
  no GPS, and noisier-than-assumed accelerometers; the table is in `validation_results/estimator_comparison.md`. What it shows
  (8 seeds, passive flight of the 29 mm example vehicle): the EKF has the smaller mean position and velocity error in every scenario (without GPS the difference is inside the seed spread)
  (nominal 0.49 vs 1.13 m and 0.23 vs 0.96 m/s), probably mostly from the delayed-measurement handling and the accelerometer
  error term rather than from the attitude states (not isolated by an ablation); the attitude error is only slightly smaller (1.0 vs 1.16 deg, with a
  seed-to-seed spread of about 0.65 deg, so that difference is within the noise); without GPS both drift (horizontal position
  several metres); inflating `noise_scale` for a vibrating accelerometer made the EKF *worse* because it also down-weights the
  aiding sensors. It is a simulation comparison only, on a mostly vertical flight that exercises attitude estimation weakly; no
  filter here has been run on real data.
