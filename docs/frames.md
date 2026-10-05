# Reference frames, units and conventions

All quantities are SI (m, kg, s, N, Pa, K, rad). Config files use degrees for angles (keys end in `_deg`); everything
internal and in telemetry is radians.

| frame | origin | axes (right-handed) | use |
|---|---|---|---|
| **Launch frame L** (treated as inertial) | launch pad (`z = 0` = pad elevation) | x East, y North, z Up | position, velocity, acceleration, wind, terrain |
| **Body frame B** | vehicle CG | x toward the nose, y "right", z completes the set (aircraft FRD: level and heading North => y East, z Down) | angular velocity, thrust, aerodynamic forces, accelerometer/gyro/magnetometer |
| **Aerodynamic frame** | CG | x along the relative air velocity `v_rel = v - wind` | angle of attack, sideslip, drag (along -x), lift (normal) |
| **Earth/ECI** | -- | **not modelled**: the flat, non-rotating launch frame is used. Earth rotation, curvature and Coriolis are neglected | valid for ranges below ~50 km |

## Attitude

Quaternion `q = (w, x, y, z)`, Hamilton convention, unit norm, maps **body -> launch**: `v_L = q (x) v_B (x) q*`. Used internally
because it has no gimbal lock; renormalised after every step. `quat_from_pointing(elevation, azimuth, roll)` builds the
launch attitude: nose at `elevation` above the horizon and `azimuth` clockwise from North (bearing), `roll` about the nose.

Display Euler angles (columns `yaw`, `pitch`, `roll`) follow the aerospace 3-2-1 sequence relative to local North-East-Down:
`yaw` = heading of the nose axis, clockwise from North; `pitch` = elevation of the nose above the horizon (+90 deg
on the pad); `roll` = rotation about the nose axis, positive right-handed. They are singular at pitch = +-90 deg
(heading and roll become coupled; a vertical rocket reports heading 0 by convention) -- never use them for computation.

## Aerodynamic angles

With relative air velocity in body axes `(u, v, w)`:

* total angle of attack `alpha = atan2(sqrt(v^2 + w^2), u)`, 0..pi (pi = flying tail first)
* sideslip `beta = asin(v / |V|)`
* Mach = `|V| / a(T)`, dynamic pressure `q = 1/2 rho |V|^2`.

Wind: `wind.at(t, z)` is the **air velocity in launch axes**. Direction convention is meteorological: `direction_from_deg` is
the bearing the wind blows *from*, clockwise from North; 270 deg (a westerly) blows toward +East. The vehicle's aerodynamic
velocity is `v_vehicle - wind`; wind is never added to ground velocity.

## Positions along the rocket

Axial positions in configs (`cg_from_nose_m`, fins, motor, parachute attachment) are measured **aft from the nose tip**.
Internally the lever arm of a feature relative to the CG, "forward positive", is `x_cg - x_feature`.

## Gimbal sign convention

`tvc_y` rotates the thrust vector about +y_B, `tvc_z` about +z_B (right-handed). With the nozzle aft of the CG, a positive
`tvc_y` produces a negative pitch moment (about +y_B): `M_y = (x_cg - x_nozzle) T sin(tvc_y)`. Test:
`tests/test_dynamics.py::test_tvc_moment_direction_and_magnitude`.

## Specific force and accelerometers

Telemetry `acc_*` is the coordinate acceleration `dv/dt` in launch axes. The accelerometer measures *specific force*
`(a - g)` in body axes (`meas_accel_*`), so a vehicle at rest nose-up reads +g along body x.
