# Sensors and estimation (audit, models, roadmap)

Status: **simulation only.** None of the sensor models or estimators has been compared with real flight data. Default
parameters are generic MEMS orders of magnitude, not a datasheet.

## 1. Sensor audit

| sensor | measures | model (`sensors/sensors.py`) | not modelled |
|---|---|---|---|
| accelerometer | specific force, body axes | scale error, **axis misalignment** (random small rotation per run), bias + random walk, white noise, quantisation, saturation, **latency**, own rate, optional **dropout / start-up delay** | temperature drift, g-sensitivity, vibration/aliasing, cross-axis beyond misalignment, accelerometer dynamics |
| gyroscope | angular rate, body axes | same error chain (bias default 2e-3 rad/s 1-sigma) | g-sensitivity, rate-saturation recovery, temperature, coning/sculling effects of sample-rate integration |
| barometer | **static pressure [Pa]** | noise, bias + walk, quantisation, saturation, latency; altitude conversion is the *consumer's* job (ISA) | avionics-bay vent lag and transonic port disturbance (real altimeters show both; seen in the Prometheus data), temperature drift |
| GPS | position + velocity, launch frame | separate **position and velocity noise** (default velocity = 0.05 x position), start-up delay (time-to-first-fix), **Bernoulli dropout**, latency, own rate (default 5 Hz) | multipath, ionosphere, COCOM altitude/velocity limits at high speed, dilution of precision, outages by attitude (antenna shadowing) |
| magnetometer | Earth field, body axes | constant reference field (configurable, east/declination component supported), noise, bias, quantisation, misalignment | hard/soft-iron distortion, motor-current interference, field variation over the flight |

Findings of the V1.1 audit (V1 behaviour, then change):

1. The magnetometer is used **only for pad alignment** (TRIAD), never in flight. *Unchanged; documented.* In flight the
   attitude estimate is gyro-integrated and **drifts** at the gyro bias rate.
2. V1 assumed a field with no east component; alignment error with real declination was not captured. *Changed:* the sensor
   uses the configured reference field and `triad_attitude` handles a general reference (tested with a 15-deg declination:
   exact recovery, whereas the old construction is wrong by > 1e-3).
3. V1 had no gyro-bias handling. *Changed:* with `estimator.estimate_gyro_bias` (default on) the stationary pad gyro mean is
   subtracted after alignment (tested: bias recovered to 2e-3 rad/s with 2e-3 rad/s noise; raw rate otherwise).
   Accelerometer bias is **not** estimated.
4. GPS had one noise value for position and velocity and no outage model. *Changed* (see table).
5. No sensor misalignment. *Changed* (random per-run rotation, `misalignment_std_deg`; tested as a proper rotation that preserves
   the vector norm, with the expected Rayleigh-like statistics).
6. Rates/latency: every channel samples on its own clock and releases its sample `latency_s` later; consumers see
   `*_new` flags and hold values (tested: five channels at their configured rates over 5 s within 2 samples).
7. Fidelity 6 = fidelity 5 numerically (it is a *preset*, see `physics.md`).

## 2. Estimator interface

`estimation/estimators.py::Estimator.update(t, readings, truth=None) -> EstimatedState` - the controller only ever sees
the returned state (valid flag, position, velocity, quaternion, bias-corrected rates, launch-detect, extras).

| implementation | what it is |
|---|---|
| `NullEstimator` (`type: none`) | no estimate (open loop) |
| `TruthEstimator` (`type: truth`, fidelity >= 3) | **perfect state**, for controller development and as the upper bound when judging estimator-induced degradation. Not deployable. Tested to have exactly zero error |
| `NavigationFilter` (`type: nav_kf`, fidelity >= 5) | **linear Kalman filter** on position/velocity (6 states) driven by accelerometer + gyro-integrated attitude (no attitude states in the covariance), baro and GPS updates, latency compensation, pad hold (ZUPT) before launch detection, TRIAD alignment, pad gyro-bias estimation |

**There is no EKF in this repository and nothing claims to be one.**

Measured behaviour of the `nav_kf` on simulated flights (`tests/test_gnc.py`): altitude error ~1.3 m RMS, velocity ~1-2 m/s
RMS, attitude within 5 degrees at 6 s with default sensors (the simulator's own noise models - not real hardware).

## 3. EKF roadmap (design only, not implemented)

Purpose: remove the two structural weaknesses of `nav_kf` - attitude uncertainty is not in the filter, and accelerometer/gyro
biases are not estimated in flight.

* **State** (error-state / multiplicative EKF): position (3), velocity (3), attitude error (3, quaternion nominal), gyro bias
  (3), accelerometer bias (3) = 15 states; optionally baro bias.
* **Propagation**: strapdown at IMU rate with the same latency handling; process noise from the sensor noise densities and
  bias random-walk parameters already in `SensorCfg` (these are the *design inputs*: `noise_std`, `bias_walk_std`).
* **Updates**: baro altitude, GPS position/velocity (honouring dropout and start-up delay), magnetometer (before burn and
  in coast, rejected under motor current), quasi-static gravity vector on the pad, ZUPT on the pad.
* **Acceptance tests to write first**: convergence of injected biases; NEES/NIS consistency over Monte Carlo runs
  (covariance honest); degradation under GPS dropout; no divergence under saturated acceleration at ignition;
  identical interface to `Estimator` so controllers need no change; head-to-head against `nav_kf` and `truth` on the same seeds.

## 4. Tests (`tests/test_sensors_v11.py`, `tests/test_gnc.py`)

Dropout rate (30 % -> 0.30 +- 0.02), zero/full dropout, start-up delay, latency, async rates, GPS velocity vs position noise,
default velocity ratio, GPS fix flag, misalignment (rotation, norm preserved, statistics), reproducibility with dropout, truth
estimator, TRIAD with declination, gyro-bias recovery and its disabled case, a full flight with 30 % GPS dropout and 3 s
start-up delay that still estimates altitude to < 5 m RMS.
