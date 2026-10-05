# Telemetry schema v1.1.0

| column | unit | dtype | group | min fidelity | description |
|---|---|---|---|---|---|
| `t` | s | float64 | time | 0 | simulation time |
| `dt` | s | float64 | time | 0 | step length that produced this row (0 for the first row) |
| `phase` | - | int8 | flight | 0 | flight phase code (see FlightPhase) |
| `pos_x` | m | float64 | state | 0 | position, launch frame (E, N, U) (x) |
| `pos_y` | m | float64 | state | 0 | position, launch frame (E, N, U) (y) |
| `pos_z` | m | float64 | state | 0 | position, launch frame (E, N, U) (z) |
| `vel_x` | m/s | float64 | state | 0 | velocity, launch frame (x) |
| `vel_y` | m/s | float64 | state | 0 | velocity, launch frame (y) |
| `vel_z` | m/s | float64 | state | 0 | velocity, launch frame (z) |
| `acc_x` | m/s^2 | float64 | state | 0 | coordinate acceleration, launch frame (x) |
| `acc_y` | m/s^2 | float64 | state | 0 | coordinate acceleration, launch frame (y) |
| `acc_z` | m/s^2 | float64 | state | 0 | coordinate acceleration, launch frame (z) |
| `quat_w` | - | float64 | state | 0 | attitude quaternion body->launch (Hamilton) (w) |
| `quat_x` | - | float64 | state | 0 | attitude quaternion body->launch (Hamilton) (x) |
| `quat_y` | - | float64 | state | 0 | attitude quaternion body->launch (Hamilton) (y) |
| `quat_z` | - | float64 | state | 0 | attitude quaternion body->launch (Hamilton) (z) |
| `roll` | rad | float64 | state | 0 | roll about the nose axis (display Euler, 3-2-1 vs local NED) |
| `pitch` | rad | float64 | state | 0 | nose elevation above the horizon (display Euler) |
| `yaw` | rad | float64 | state | 0 | heading of the nose, clockwise from North (display Euler) |
| `omega_p` | rad/s | float64 | state | 0 | angular velocity, body frame (p) |
| `omega_q` | rad/s | float64 | state | 0 | angular velocity, body frame (q) |
| `omega_r` | rad/s | float64 | state | 0 | angular velocity, body frame (r) |
| `alpha_p` | rad/s^2 | float64 | state | 0 | angular acceleration, body frame (p) |
| `alpha_q` | rad/s^2 | float64 | state | 0 | angular acceleration, body frame (q) |
| `alpha_r` | rad/s^2 | float64 | state | 0 | angular acceleration, body frame (r) |
| `mass` | kg | float64 | mass | 0 | total mass |
| `prop_mass` | kg | float64 | mass | 0 | remaining propellant mass |
| `dry_mass` | kg | float64 | mass | 0 | mass with zero propellant |
| `cg` | m | float64 | mass | 0 | centre of gravity, aft of nose tip |
| `cp` | m | float64 | mass | 0 | centre of pressure, aft of nose tip |
| `static_margin` | cal | float64 | mass | 0 | (cp - cg)/diameter; > 0 is statically stable |
| `ixx` | kg m^2 | float64 | mass | 0 | roll moment of inertia about CG |
| `iyy` | kg m^2 | float64 | mass | 0 | pitch/yaw moment of inertia about CG |
| `altitude` | m | float64 | environment | 0 | height above local ground (AGL) |
| `altitude_msl` | m | float64 | environment | 0 | height above mean sea level |
| `temperature` | K | float64 | environment | 0 | air temperature |
| `pressure` | Pa | float64 | environment | 0 | static air pressure |
| `density` | kg/m^3 | float64 | environment | 0 | air density |
| `speed_of_sound` | m/s | float64 | environment | 0 | speed of sound |
| `wind_x` | m/s | float64 | environment | 0 | air velocity, launch frame (x) |
| `wind_y` | m/s | float64 | environment | 0 | air velocity, launch frame (y) |
| `wind_z` | m/s | float64 | environment | 0 | air velocity, launch frame (z) |
| `airspeed` | m/s | float64 | flight | 0 | |v_vehicle - v_wind| |
| `speed` | m/s | float64 | flight | 0 | |v_vehicle| (ground-relative) |
| `gravity` | m/s^2 | float64 | flight | 0 | gravitational acceleration magnitude |
| `thrust` | N | float64 | flight | 0 | thrust magnitude |
| `drag` | N | float64 | flight | 0 | aerodynamic drag incl. parachute (along -v_rel) |
| `lift` | N | float64 | flight | 0 | aerodynamic force normal to the relative wind |
| `chute_drag` | N | float64 | flight | 0 | parachute drag |
| `mach` | - | float64 | flight | 0 | airspeed / speed of sound |
| `aoa` | rad | float64 | flight | 0 | total angle of attack (nose to relative wind) |
| `sideslip` | rad | float64 | flight | 0 | sideslip angle |
| `qdyn` | Pa | float64 | flight | 0 | dynamic pressure |
| `tvc_cmd_y` | rad | float64 | control | 3 | commanded thrust deflection about y_B |
| `tvc_cmd_z` | rad | float64 | control | 3 | commanded thrust deflection about z_B |
| `tvc_y` | rad | float64 | control | 3 | actual thrust deflection about y_B |
| `tvc_z` | rad | float64 | control | 3 | actual thrust deflection about z_B |
| `fin_cmd_pitch` | rad | float64 | control_surfaces | 3 | commanded pitch deflection-equivalent (control surfaces) |
| `fin_cmd_yaw` | rad | float64 | control_surfaces | 3 | commanded yaw deflection-equivalent (control surfaces) |
| `fin_cmd_roll` | rad | float64 | control_surfaces | 3 | commanded roll deflection-equivalent (control surfaces) |
| `fin_0` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 0 |
| `fin_1` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 1 |
| `fin_2` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 2 |
| `fin_3` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 3 |
| `fin_4` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 4 |
| `fin_5` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 5 |
| `fin_6` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 6 |
| `fin_7` | rad | float64 | control_surfaces | 3 | actual deflection of control fin 7 |
| `meas_accel_x` | m/s^2 | float64 | sensors | 4 | accelerometer specific force, body (x) |
| `meas_accel_y` | m/s^2 | float64 | sensors | 4 | accelerometer specific force, body (y) |
| `meas_accel_z` | m/s^2 | float64 | sensors | 4 | accelerometer specific force, body (z) |
| `meas_gyro_x` | rad/s | float64 | sensors | 4 | gyroscope rate, body (x) |
| `meas_gyro_y` | rad/s | float64 | sensors | 4 | gyroscope rate, body (y) |
| `meas_gyro_z` | rad/s | float64 | sensors | 4 | gyroscope rate, body (z) |
| `meas_baro_pressure` | Pa | float64 | sensors | 4 | barometer static pressure |
| `meas_baro_altitude` | m | float64 | sensors | 4 | ISA altitude from barometer relative to the first sample |
| `meas_baro_new` | - | int8 | sensors | 4 | 1 when a new barometer sample became visible this row |
| `meas_gps_pos_x` | m | float64 | sensors | 4 | GPS position, launch frame (x) |
| `meas_gps_pos_y` | m | float64 | sensors | 4 | GPS position, launch frame (y) |
| `meas_gps_pos_z` | m | float64 | sensors | 4 | GPS position, launch frame (z) |
| `meas_gps_vel_x` | m/s | float64 | sensors | 4 | GPS velocity, launch frame (x) |
| `meas_gps_vel_y` | m/s | float64 | sensors | 4 | GPS velocity, launch frame (y) |
| `meas_gps_vel_z` | m/s | float64 | sensors | 4 | GPS velocity, launch frame (z) |
| `meas_gps_new` | - | int8 | sensors | 4 | 1 when a new GPS sample became visible this row |
| `meas_mag_x` | T | float64 | sensors | 4 | magnetometer field, body (x) |
| `meas_mag_y` | T | float64 | sensors | 4 | magnetometer field, body (y) |
| `meas_mag_z` | T | float64 | sensors | 4 | magnetometer field, body (z) |
| `est_valid` | - | int8 | estimator | 3 | 1 when the estimator output is valid (aligned) |
| `launch_detected` | - | int8 | estimator | 3 | 1 once the flight computer has detected launch |
| `est_pos_x` | m | float64 | estimator | 3 | estimated position, launch frame (x) |
| `est_pos_y` | m | float64 | estimator | 3 | estimated position, launch frame (y) |
| `est_pos_z` | m | float64 | estimator | 3 | estimated position, launch frame (z) |
| `est_vel_x` | m/s | float64 | estimator | 3 | estimated velocity, launch frame (x) |
| `est_vel_y` | m/s | float64 | estimator | 3 | estimated velocity, launch frame (y) |
| `est_vel_z` | m/s | float64 | estimator | 3 | estimated velocity, launch frame (z) |
| `est_quat_w` | - | float64 | estimator | 3 | estimated attitude quaternion (w) |
| `est_quat_x` | - | float64 | estimator | 3 | estimated attitude quaternion (x) |
| `est_quat_y` | - | float64 | estimator | 3 | estimated attitude quaternion (y) |
| `est_quat_z` | - | float64 | estimator | 3 | estimated attitude quaternion (z) |
