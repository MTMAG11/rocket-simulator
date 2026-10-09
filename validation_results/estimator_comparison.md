# Linear Kalman filter vs error-state EKF (simulation)

Example 29 mm vehicle (`configs/example_g80.yaml`), passive flight, 8 seeds per scenario, identical sensor noise for both
filters. RMS error against truth from ignition to apogee, mean (standard deviation over seeds). Simulation only.

| scenario | filter | position [m] | velocity [m/s] | attitude [deg] |
|---|---|---|---|---|
| nominal | nav_kf | 1.13 (0.08) | 0.96 (0.14) | 1.16 (0.71) |
| nominal | ekf | 0.49 (0.14) | 0.23 (0.06) | 1.00 (0.65) |
| wind 6 m/s | nav_kf | 1.16 (0.10) | 1.02 (0.17) | 1.15 (0.71) |
| wind 6 m/s | ekf | 0.49 (0.13) | 0.24 (0.06) | 0.90 (0.56) |
| baro dropout 90 % | nav_kf | 1.37 (0.10) | 0.99 (0.13) | 1.16 (0.71) |
| baro dropout 90 % | ekf | 0.45 (0.13) | 0.23 (0.06) | 1.00 (0.65) |
| GPS dropout 90 % | nav_kf | 0.25 (0.14) | 0.35 (0.19) | 1.16 (0.71) |
| GPS dropout 90 % | ekf | 0.11 (0.06) | 0.10 (0.03) | 1.00 (0.66) |
| no GPS | nav_kf | 6.32 (4.16) | 0.93 (0.53) | 1.16 (0.71) |
| no GPS | ekf | 4.49 (3.88) | 0.81 (0.52) | 1.12 (0.70) |
| vibration, filter not told | nav_kf | 0.98 (0.08) | 0.97 (0.10) | 1.30 (0.75) |
| vibration, filter not told | ekf | 0.48 (0.13) | 0.26 (0.06) | 1.12 (0.66) |
| vibration, filter told | nav_kf | 0.98 (0.08) | 0.97 (0.10) | 1.30 (0.75) |
| vibration, filter told | ekf | 0.57 (0.33) | 0.45 (0.24) | 1.18 (0.70) |

Scenarios:

* **nominal**: default sensors, no wind
* **wind 6 m/s**: constant 6 m/s wind from 270 deg (the vehicle weathercocks and drifts)
* **baro dropout 90 %**: 90 % of the barometer samples lost at random
* **GPS dropout 90 %**: 90 % of the GPS fixes lost at random
* **no GPS**: GPS not used by the filter (barometer and IMU only)
* **vibration, filter not told**: accelerometer noise 1.0 m/s^2 in the sensor (33x the configured 0.03), filter still assumes 0.03
* **vibration, filter told**: same sensor noise; EKF noise_scale 33, which inflates the assumed noise of ALL sensors (so barometer and GPS are down-weighted too); the linear filter has no such setting and is unchanged
