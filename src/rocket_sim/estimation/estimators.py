"""State estimation: truth -> sensors -> ESTIMATE.

``NavigationFilter`` is a genuine (linear) Kalman filter on position and velocity in the launch
frame, driven by gyro-integrated attitude:

    attitude   q_hat: initialised on the pad by TRIAD alignment (accelerometer = local "up",
               magnetometer = heading), then propagated with the gyro (exact quaternion
               exponential). NO accelerometer/magnetometer correction in flight: during motor
               burn the accelerometer does not measure gravity, so tilt cannot be observed from
               it. Gyro bias therefore causes attitude drift (documented limitation).
    translation  x = [p, v] (6 states). Predict with IMU specific force rotated by q_hat:
               a = R(q_hat) f_b + g, v += a dt, p += v dt + a dt^2/2. Update with barometric
               altitude (z) and GPS position/velocity using the sensors' NOMINAL noise as R.
               A random-acceleration process noise accounts for accelerometer noise plus an
               attitude-error allowance.
    latency      known sensor latency is compensated by propagating each delayed measurement forward
               with the filter's own velocity/acceleration estimate (first-order; residual error ~ jerk * tau^2).
    launch detection  specific-force magnitude above a threshold for N consecutive IMU samples
               (the flight computer does not "know" the launch time). Before detection, a
               zero-velocity/zero-position update holds the pad solution.

Not implemented (and not claimed): accelerometer/gyro bias estimation, an error-state EKF with
attitude covariance, GPS lever arm, outlier rejection. Replace with an EKF later without
touching the rest of the pipeline: any object with this interface works.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..config.schema import EstimatorCfg, SensorsCfg
from ..constants import G0
from ..environment import Gravity, ISAAtmosphere
from ..physics.math3d import (
    Quat,
    dcm_to_quat,
    quat_from_axis_angle,
    quat_mul,
    quat_normalize,
    quat_rotate,
)
from ..sensors import SensorReadings


@dataclass
class EstimatedState:
    valid: bool = False
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: Quat = (1.0, 0.0, 0.0, 0.0)
    omega: tuple[float, float, float] = (0.0, 0.0, 0.0)  # bias-uncorrected gyro [rad/s]
    launch_detected: bool = False
    launch_time: float | None = None
    extra: dict[str, float] = field(default_factory=dict)


class LaunchDetector:
    """Declares launch when |specific force| > threshold for ``count`` consecutive samples."""

    def __init__(self, threshold: float = 2.5 * G0, count: int = 5) -> None:
        self.threshold = threshold
        self.count = count
        self._run = 0
        self.detected = False
        self.time: float | None = None
        self._first_t: float | None = None

    def update(self, t: float, accel: np.ndarray) -> bool:
        if self.detected:
            return True
        if float(np.linalg.norm(accel)) > self.threshold:
            if self._run == 0:
                self._first_t = t
            self._run += 1
            if self._run >= self.count:
                self.detected = True
                self.time = self._first_t  # report the start of the sustained exceedance
        else:
            self._run = 0
        return self.detected


class Estimator:
    """Interface: ``update(t, readings) -> EstimatedState``."""

    name = "estimator"

    def update(self, t: float, r: SensorReadings) -> EstimatedState:  # pragma: no cover
        raise NotImplementedError


class NullEstimator(Estimator):
    name = "none"

    def update(self, t: float, r: SensorReadings) -> EstimatedState:
        return EstimatedState()


def triad_attitude(up_b: np.ndarray, mag_b: np.ndarray) -> Quat:
    """Body->launch quaternion from the measured 'up' and magnetic-field vectors (body frame)."""
    up = up_b / np.linalg.norm(up_b)
    east = np.cross(mag_b, up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    # columns of R^T (launch->body) are the launch axes expressed in body coordinates, so the rows
    # of R (body->launch) are (east_b, north_b, up_b)
    r = (
        (float(east[0]), float(east[1]), float(east[2])),
        (float(north[0]), float(north[1]), float(north[2])),
        (float(up[0]), float(up[1]), float(up[2])),
    )
    return dcm_to_quat(r)


def integrate_gyro(q: Quat, omega: np.ndarray, dt: float) -> Quat:
    """Propagate q with a constant body rate over dt (exact exponential map)."""
    w = float(np.linalg.norm(omega))
    if w * dt < 1e-12:
        return q
    dq = quat_from_axis_angle((float(omega[0]), float(omega[1]), float(omega[2])), w * dt)
    return quat_normalize(quat_mul(q, dq))


class NavigationFilter(Estimator):
    name = "nav_kf"

    def __init__(
        self,
        cfg: EstimatorCfg,
        sensors: SensorsCfg,
        gravity: Gravity,
        site_elevation: float,
        pad_position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.cfg = cfg
        self.sc = sensors
        self.gravity = gravity
        self.site = site_elevation
        self.isa = ISAAtmosphere()
        self.detector = LaunchDetector()
        self.q: Quat = (1.0, 0.0, 0.0, 0.0)
        self.aligned = False
        self._align_up: list[np.ndarray] = []
        self._align_mag: list[np.ndarray] = []
        self._align_p: list[float] = []
        self._p_ref = 101325.0
        self._h_ref = 0.0
        self.x = np.zeros(6)
        self.x[:3] = pad_position
        self.pad = np.asarray(pad_position, float)
        self.P = np.diag([1.0] * 3 + [0.1] * 3)
        self._last_imu_t: float | None = None
        self._omega = np.zeros(3)
        self._a_last = np.zeros(3)  # latest estimated inertial acceleration (for latency compensation)
        self._f_b = np.array([0.0, 0.0, G0])
        # measurement noise (nominal sensor specs known to the filter)
        sa = sensors.accelerometer
        self._sigma_a = math.sqrt(sa.noise_std**2 + 1e-6)
        self._q_att = 0.35  # [m/s^2] allowance for attitude-error induced acceleration error
        sb = sensors.barometer
        self._r_baro = (max(sb.noise_std / (1.2 * G0), 0.05) ** 2) + 1.0  # m^2 (+1 m^2 bias allowance)
        sg = sensors.gps
        self._r_gps_p = max(sg.noise_std, 0.1) ** 2 + 0.25
        self._r_gps_v = max(sg.noise_std * 0.05, 0.02) ** 2 + 0.01

    # ----------------------------------------------------------------------------------------
    def _baro_altitude(self, pressure: float) -> float:
        try:
            return self.isa.pressure_to_altitude(max(pressure, 1.0)) - self._h_ref
        except ValueError:
            return 0.0

    def _try_align(self, t: float, r: SensorReadings) -> None:
        if r.accel_new:
            self._align_up.append(r.accel.copy())
        if r.mag_new:
            self._align_mag.append(r.mag.copy())
        if r.baro_new:
            self._align_p.append(r.baro_pressure)
        if t >= self.cfg.alignment_time_s and self._align_up and self._align_mag:
            up = np.mean(self._align_up, axis=0)
            mag = np.mean(self._align_mag, axis=0)
            self.q = triad_attitude(up, mag)
            if self._align_p:
                self._p_ref = float(np.mean(self._align_p))
                self._h_ref = self.isa.pressure_to_altitude(self._p_ref)
            self.aligned = True

    def update(self, t: float, r: SensorReadings) -> EstimatedState:
        if not self.aligned:
            self._try_align(t, r)
            return EstimatedState(valid=False)

        if r.gyro_new:
            self._omega = r.gyro.copy()
        if r.accel_new:
            self._f_b = r.accel.copy()
            dt = 0.0 if self._last_imu_t is None else t - self._last_imu_t
            self._last_imu_t = t
            was = self.detector.detected
            self.detector.update(t, r.accel)
            if dt > 0:
                self.q = integrate_gyro(self.q, self._omega, dt)
                if self.detector.detected:
                    self._predict(dt)
                else:
                    self.x[:] = np.concatenate([self.pad, np.zeros(3)])  # pad hold (ZUPT)
            if self.detector.detected and not was:
                # P is inflated slightly at launch to admit the attitude error accumulated so far
                self.P = np.diag([0.5] * 3 + [0.1] * 3)
        if self.detector.detected:
            if r.baro_new:
                tau_b = self.sc.barometer.latency_s  # measurement describes the state tau seconds ago
                alt = self._baro_altitude(r.baro_pressure) + float(self.x[5]) * tau_b
                self._update(np.array([alt]), np.array([[0, 0, 1, 0, 0, 0.0]]), np.array([[self._r_baro]]))
            if r.gps_new and self.cfg.gps_enabled:
                tau = self.sc.gps.latency_s  # propagate the delayed fix to "now" with the filter's own a, v
                p_m, v_m = r.gps[:3], r.gps[3:]
                z = np.concatenate(
                    [p_m + v_m * tau + 0.5 * self._a_last * tau * tau, v_m + self._a_last * tau]
                )
                h = np.eye(6)
                rm = np.diag([self._r_gps_p] * 3 + [self._r_gps_v] * 3)
                self._update(z, h, rm)

        return EstimatedState(
            valid=True,
            position=(float(self.x[0]), float(self.x[1]), float(self.x[2])),
            velocity=(float(self.x[3]), float(self.x[4]), float(self.x[5])),
            quaternion=self.q,
            omega=(float(self._omega[0]), float(self._omega[1]), float(self._omega[2])),
            launch_detected=self.detector.detected,
            launch_time=self.detector.time,
        )

    def _predict(self, dt: float) -> None:
        f_l = quat_rotate(self.q, (float(self._f_b[0]), float(self._f_b[1]), float(self._f_b[2])))
        g = self.gravity.g(self.site + float(self.x[2]))
        a = np.array([f_l[0], f_l[1], f_l[2] - g])
        self._a_last = a
        self.x[:3] += self.x[3:] * dt + 0.5 * a * dt * dt
        self.x[3:] += a * dt
        f = np.eye(6)
        f[0:3, 3:6] = np.eye(3) * dt
        qa = self._sigma_a**2 * dt + (self._q_att**2) * dt  # random-acceleration spectral density
        g_mat = np.vstack([np.eye(3) * 0.5 * dt * dt, np.eye(3) * dt])
        q = (g_mat @ g_mat.T) * (qa / max(dt, 1e-9))
        self.P = f @ self.P @ f.T + q

    def _update(self, z: np.ndarray, h: np.ndarray, rm: np.ndarray) -> None:
        y = z - h @ self.x
        s = h @ self.P @ h.T + rm
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        i_kh = np.eye(6) - k @ h
        self.P = i_kh @ self.P @ i_kh.T + k @ rm @ k.T  # Joseph form


def build_estimator(
    cfg: EstimatorCfg,
    sensors: SensorsCfg,
    gravity: Gravity,
    site_elevation: float,
) -> Estimator:
    if cfg.type == "nav_kf":
        return NavigationFilter(cfg, sensors, gravity, site_elevation)
    return NullEstimator()
