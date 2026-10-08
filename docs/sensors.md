# Sensors and estimation

Status: simulation only. No sensor model or estimator has been compared with real flight data. Default parameters are generic
MEMS orders of magnitude, not a datasheet.

## Data flow

```
true state -> sensor models -> measurements -> estimator -> estimate -> controller
                                    \-> (HIL) protocol v2 -> flight computer (own estimator + controller)
```

* Sensors take the true state and return only what a sensor would report. Telemetry keeps them in separate columns
  (`pos_*`/`fsp_*` truth, `meas_*` measurement, `est_*` estimate); every column has a role in the schema.
* The controller gets the estimate, or the truth only if `controller.state_source: truth` (or `use_truth`) says so; this is recorded as
  `controller_state_source` and warned about. `state_source: estimate` refuses to start without an estimator. At fidelity < 5 there is
  no estimator, so `state_source: auto` falls back to truth with a warning.
* Estimator outputs in telemetry: position, velocity, attitude and the pad-estimated gyro bias (`est_gyro_bias_*`). Accelerometer
  bias is not estimated and attitude uncertainty is not in the filter.
* Tests (`tests/test_truth_separation.py`): an ideal sensor reads the true specific force exactly; noise statistics match the
  configured sigma; saturation clips the measurement but not the truth; barometer altitude is ISA-inverted from pressure;
  realisations are seeded; the navigation filter is never given truth; replaying the logged measurements through a fresh filter
  reproduces the logged estimates bit for bit.

## Sensor models (`sensors/sensors.py`)

| sensor | measures | modelled | not modelled |
|---|---|---|---|
| accelerometer | specific force, body axes | scale error, random axis misalignment per run, bias + random walk, white noise, quantisation, saturation, latency, own rate, dropout / start-up delay | temperature drift, g-sensitivity, vibration/aliasing, accelerometer dynamics |
| gyroscope | angular rate, body axes | same chain (default bias 2e-3 rad/s 1-sigma) | g-sensitivity, rate-saturation recovery, temperature, coning/sculling |
| barometer | static pressure [Pa] | noise, bias + walk, quantisation, saturation, latency; altitude conversion (ISA) is done by the consumer | bay vent lag and transonic port disturbance (both visible in the Prometheus data), temperature drift |
| GPS | position + velocity, launch frame | separate position and velocity noise (default velocity = 0.05 x position), start-up delay, Bernoulli dropout, latency, own rate (default 5 Hz) | multipath, ionosphere, COCOM limits at high speed, dilution of precision, antenna shadowing |
| magnetometer | Earth field, body axes | configurable reference field (including east/declination component), noise, bias, quantisation, misalignment | hard/soft-iron distortion, motor-current interference, field variation over the flight |

Every channel samples on its own clock and releases the sample `latency_s` later; consumers see `*_new` flags and held values.

The magnetometer is used only for pad alignment (TRIAD), never in flight. In flight the attitude estimate is gyro-integrated and
drifts at the gyro bias rate. `triad_attitude` handles a general reference field (tested with a 15 deg declination). With
`estimator.estimate_gyro_bias` (default on) the stationary pad gyro mean is subtracted after alignment.

## Estimator interface

`estimation/estimators.py::Estimator.update(t, readings, truth=None) -> EstimatedState`. The controller sees only the returned
state (valid flag, position, velocity, quaternion, bias-corrected rates, launch-detect, extras).

| implementation | description |
|---|---|
| `NullEstimator` (`type: none`) | no estimate (open loop) |
| `TruthEstimator` (`type: truth`, fidelity >= 3) | perfect state, for controller development and as the upper bound on estimator-induced degradation |
| `NavigationFilter` (`type: nav_kf`, fidelity >= 5) | linear Kalman filter on position/velocity (6 states) driven by the accelerometer and gyro-integrated attitude (no attitude states in the covariance); baro and GPS updates, latency compensation, pad hold (ZUPT) before launch detection, TRIAD alignment, pad gyro-bias estimation |

There is no EKF in this repository. On simulated flights (`tests/test_gnc.py`) the `nav_kf` gives about 1.3 m RMS altitude error,
1-2 m/s RMS velocity error, and attitude within 5 degrees at 6 s, using the simulator's own noise models.

## EKF design (not implemented)

Goal: put attitude uncertainty and accelerometer/gyro biases into the filter.

* State (error-state EKF): position (3), velocity (3), attitude error (3), gyro bias (3), accelerometer bias (3); optionally baro bias.
* Propagation: strapdown at IMU rate with the same latency handling; process noise from `noise_std` and `bias_walk_std` in `SensorCfg`.
* Updates: baro altitude, GPS position/velocity (with dropout and start-up delay), magnetometer (before burn and in coast), gravity
  vector and ZUPT on the pad.
* Acceptance tests: convergence of injected biases; NEES/NIS consistency over Monte Carlo runs; degradation under GPS dropout; no
  divergence under saturated acceleration at ignition; same `Estimator` interface; comparison with `nav_kf` and `truth` on the same seeds.

## Tests

`tests/test_sensors_v11.py`, `tests/test_gnc.py`: dropout rate, start-up delay, latency, async rates, GPS velocity vs position noise,
misalignment statistics, reproducibility, truth estimator, TRIAD with declination, gyro-bias recovery, and a full flight with 30 %
GPS dropout and 3 s start-up delay that still estimates altitude to < 5 m RMS.
