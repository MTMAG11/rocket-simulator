"""Error-state extended Kalman filter (``estimator.type: ekf``): attitude, position, velocity, gyro and accelerometer bias.

Derivation, state definition and what to expect: docs/derivations/error_state_ekf.md. The estimator receives sensor samples
only (``update`` never reads its ``truth`` argument). Pad alignment, launch detection and the barometer/GPS conventions are
those of the linear filter (``NavigationFilter``), so the two can be compared on the same flights.

The one equation left to the project owner is ``attitude_measurement_jacobian``; it is needed only for the optional
magnetometer update (``estimator.ekf_use_magnetometer``, off by default).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from ..config.schema import EstimatorCfg, SensorsCfg
from ..constants import G0
from ..environment import Gravity
from ..physics.math3d import quat_mul, quat_normalize, quat_to_dcm
from ..sensors import SensorReadings
from .estimators import EstimatedState, NavigationFilter, TruthState, integrate_gyro

TH, V, P, BG, BA = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15)  # error-state blocks


def skew(v: np.ndarray) -> np.ndarray:
    """Matrix [v]x with [v]x w = v x w."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def attitude_measurement_jacobian(z_pred_b: np.ndarray) -> np.ndarray:
    """TODO(you): the 3x3 block H_dtheta of the measurement Jacobian for a vector measured in the body frame.

    ``z_pred_b`` is the predicted measurement R(q)^T m_L (the reference vector m_L of the launch frame, rotated into the body
    frame with the nominal attitude). For the small attitude error d_theta (body frame, q_true = q (x) [1, d_theta / 2]) the
    measurement is z_b = z_pred_b + H_dtheta d_theta. Derive H_dtheta in docs/derivations/error_state_ekf.md, section 3, and check it
    against the hand-calculated values in tests/test_ekf.py (``pytest --runxfail -m user_todo`` shows it failing until implemented).
    """
    raise NotImplementedError("attitude_measurement_jacobian: see the TODO in rocket_sim/estimation/ekf.py")


def nominal_step(
    q: tuple[float, float, float, float],
    v: np.ndarray,
    p: np.ndarray,
    bg: np.ndarray,
    ba: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    dt: float,
    g: float,
) -> tuple[tuple[float, float, float, float], np.ndarray, np.ndarray]:
    """Propagate the nominal state over one IMU step (gyro and accelerometer are the measured, bias-containing values)."""
    q_new = integrate_gyro(q, gyro - bg, dt)
    r = np.array(quat_to_dcm(q_new))
    a = r @ (accel - ba) + np.array([0.0, 0.0, -g])
    return q_new, v + a * dt, p + v * dt + 0.5 * a * dt * dt


def transition_matrix(
    q: tuple[float, float, float, float], gyro_hat: np.ndarray, f_hat: np.ndarray, dt: float
) -> np.ndarray:
    """Discrete error-state transition Phi = exp(F dt) to second order, for bias-corrected gyro ``gyro_hat`` and specific force
    ``f_hat`` (both body frame) and the nominal attitude ``q`` (after the step)."""
    r = np.array(quat_to_dcm(q))
    a = np.zeros((15, 15))
    a[TH, TH] = -skew(gyro_hat)
    a[TH, BG] = -np.eye(3)
    a[V, TH] = -r @ skew(f_hat)
    a[V, BA] = -r
    a[P, V] = np.eye(3)
    return np.eye(15) + a * dt + 0.5 * (a * dt) @ (a * dt)


class ErrorStateEKF(NavigationFilter):
    name = "ekf"

    def __init__(
        self,
        cfg: EstimatorCfg,
        sensors: SensorsCfg,
        gravity: Gravity,
        site_elevation: float,
        pad_position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        super().__init__(cfg, sensors, gravity, site_elevation, pad_position)
        s = cfg.noise_scale
        self.accel_bias = np.zeros(3)
        self.P15 = np.eye(15) * 1e-6  # replaced when the pad alignment finishes
        # noise the filter ASSUMES (nominal sensor configuration, scaled by the tuning factor)
        self._sa = sensors.accelerometer.noise_std * s
        self._sg = sensors.gyroscope.noise_std * s
        self._sbg = sensors.gyroscope.bias_walk_std * s
        self._sba = sensors.accelerometer.bias_walk_std * s
        # relative accelerometer error that the state does not carry (scale factor, axis misalignment): acts on |f|
        acc = sensors.accelerometer
        self._k_acc = math.hypot(acc.scale_error_std, math.radians(acc.misalignment_std_deg)) * s
        self._r_baro *= s * s
        self._r_gps_p *= s * s
        self._r_gps_v *= s * s
        self._ekf_ready = False
        self._hist: deque[tuple[float, np.ndarray]] = (
            deque()
        )  # recent (time, [p, v]) for delayed measurements
        self._gyro_raw = np.zeros(3)  # latest measured gyro sample (bias NOT removed)
        self._f_hat = np.array([0.0, 0.0, G0])

    # ----------------------------------------------------------------------------------------- initialisation
    def _init_covariance(self) -> None:
        """Initial error covariance from the configured sensors and the alignment (see the derivation, section 4)."""
        sa, sg, sm = self.sc.accelerometer, self.sc.gyroscope, self.sc.magnetometer
        n_up = max(len(self._align_up), 1)
        tilt = math.hypot(sa.noise_std / math.sqrt(n_up), sa.bias_std) / G0
        b_h = math.hypot(float(self.mag_ref[0]), float(self.mag_ref[1]))
        heading = (sm.noise_std / math.sqrt(max(len(self._align_mag), 1))) / max(b_h, 1e-9)
        tilt, heading = (
            max(tilt, math.radians(0.5)),
            max(heading, math.radians(1.0)),
        )  # placeholder floors, not measured limits
        bg0 = (
            sg.noise_std / math.sqrt(max(len(self._align_gyro), 1))
            if self.cfg.estimate_gyro_bias
            else sg.bias_std
        )
        p = np.zeros((15, 15))
        p[TH, TH] = np.diag(
            [tilt**2, tilt**2, heading**2]
        )  # body x/y are tilt axes only when the vehicle stands vertical
        p[V, V] = np.eye(3) * 0.1  # same pad-hold values as the linear filter (variances)
        p[P, P] = np.eye(3) * 0.5
        p[BG, BG] = np.eye(3) * max(bg0, 1e-6) ** 2
        p[BA, BA] = np.eye(3) * max(sa.bias_std, 1e-6) ** 2
        self.P15 = p
        self.accel_bias[:] = 0.0
        self._gyro_raw = self.gyro_bias.copy()
        self._ekf_ready = True

    # ------------------------------------------------------------------------------------------------ update
    def update(self, t: float, r: SensorReadings | None, truth: TruthState | None = None) -> EstimatedState:
        if r is None:
            return EstimatedState(valid=False)
        if not self.aligned:
            self._try_align(t, r)
            if self.aligned and not self._ekf_ready:
                self._init_covariance()
            return EstimatedState(valid=False)

        if r.gyro_new:
            self._gyro_raw = r.gyro.copy()
            self._omega = r.gyro - self.gyro_bias
        if r.accel_new:
            self._f_b = r.accel.copy()
            dt = 0.0 if self._last_imu_t is None else t - self._last_imu_t
            self._last_imu_t = t
            was = self.detector.detected
            self.detector.update(t, r.accel)
            if dt > 0:
                if self.detector.detected:
                    self._propagate(dt, r.accel)
                else:
                    self.q = integrate_gyro(self.q, self._omega, dt)
                    self.x[:] = np.concatenate([self.pad, np.zeros(3)])  # pad hold (zero-velocity update)
            if self.detector.detected and not was:
                self.P15[V, V] = np.eye(3) * 0.1
                self.P15[P, P] = np.eye(3) * 0.5
            self._hist.append((t, self.x.copy()))
            while len(self._hist) > 2 and t - self._hist[0][0] > 1.0:
                self._hist.popleft()
        if self.detector.detected:
            if r.baro_new:
                alt = self._baro_altitude(r.baro_pressure)
                h = np.zeros((1, 15))
                h[0, 8] = 1.0  # d(altitude)/d(delta p_z)
                past = self._state_at(t - self.sc.barometer.latency_s)
                self._measure(np.array([alt - past[2]]), h, np.array([[self._r_baro]]))
            if r.gps_new and self.cfg.gps_enabled:
                h = np.zeros((6, 15))
                h[0:3, P] = np.eye(3)
                h[3:6, V] = np.eye(3)
                rm = np.diag([self._r_gps_p] * 3 + [self._r_gps_v] * 3)
                past = self._state_at(t - self.sc.gps.latency_s)
                self._measure(r.gps - past, h, rm)
            if r.mag_new and self.cfg.ekf_use_magnetometer:
                self._mag_update(r.mag)
        return EstimatedState(
            valid=True,
            position=(float(self.x[0]), float(self.x[1]), float(self.x[2])),
            velocity=(float(self.x[3]), float(self.x[4]), float(self.x[5])),
            quaternion=self.q,
            omega=(float(self._omega[0]), float(self._omega[1]), float(self._omega[2])),
            launch_detected=self.detector.detected,
            launch_time=self.detector.time,
            extra={
                "gyro_bias_x": float(self.gyro_bias[0]),
                "gyro_bias_y": float(self.gyro_bias[1]),
                "gyro_bias_z": float(self.gyro_bias[2]),
                "accel_bias_x": float(self.accel_bias[0]),
                "accel_bias_y": float(self.accel_bias[1]),
                "accel_bias_z": float(self.accel_bias[2]),
            },
        )

    # ------------------------------------------------------------------------------------------ propagation
    def _propagate(self, dt: float, accel: np.ndarray) -> None:
        g = self.gravity.g(self.site + float(self.x[2]))
        q, v, p = nominal_step(
            self.q, self.x[3:], self.x[:3], self.gyro_bias, self.accel_bias, self._gyro_raw, accel, dt, g
        )
        self._omega = self._gyro_raw - self.gyro_bias
        self.q = q
        r = np.array(quat_to_dcm(q))
        self._a_last = r @ (accel - self.accel_bias) + np.array([0.0, 0.0, -g])
        self.x[3:], self.x[:3] = v, p
        f_hat = accel - self.accel_bias
        self._f_hat = f_hat
        phi = transition_matrix(q, self._omega, f_hat, dt)  # bias-corrected rate and specific force
        qd = np.zeros((15, 15))
        qd[TH, TH] = np.eye(3) * (self._sg * dt) ** 2
        # position/velocity noise from the accelerometer (launch frame; R R^T = I): white noise plus the scale-factor and
        # misalignment error, which grows with the specific force and is NOT in the state (see the derivation, section 2)
        sa2 = self._sa**2 + (self._k_acc * float(np.linalg.norm(f_hat))) ** 2
        qd[V, V] = np.eye(3) * sa2 * dt * dt
        qd[P, P] = np.eye(3) * sa2 * (0.5 * dt * dt) ** 2
        qd[V, P] = qd[P, V] = np.eye(3) * sa2 * 0.5 * dt**3
        qd[BG, BG] = np.eye(3) * self._sbg**2 * dt
        qd[BA, BA] = np.eye(3) * self._sba**2 * dt
        self.P15 = phi @ self.P15 @ phi.T + qd
        self.P15 = 0.5 * (self.P15 + self.P15.T)

    def _state_at(self, t_meas: float) -> np.ndarray:
        """The filter's own [p, v] estimate at the time a delayed measurement was taken (linear interpolation of the history)."""
        if not self._hist or t_meas >= self._hist[-1][0]:
            return self.x
        if t_meas <= self._hist[0][0]:
            return self._hist[0][1]
        times = [h[0] for h in self._hist]
        k = int(np.searchsorted(times, t_meas))
        (t0, x0), (t1, x1) = self._hist[k - 1], self._hist[k]
        w = (t_meas - t0) / max(t1 - t0, 1e-12)
        return x0 + w * (x1 - x0)

    # ----------------------------------------------------------------------------------------------- updates
    def _measure(self, y: np.ndarray, h: np.ndarray, rm: np.ndarray) -> None:
        """Kalman update with innovation ``y`` (measurement minus prediction), then inject the error state and reset it."""
        s = h @ self.P15 @ h.T + rm
        k = self.P15 @ h.T @ np.linalg.inv(s)
        dx = k @ y
        i_kh = np.eye(15) - k @ h
        self.P15 = i_kh @ self.P15 @ i_kh.T + k @ rm @ k.T  # Joseph form
        self._inject(dx)

    def _inject(self, dx: np.ndarray) -> None:
        dth = dx[TH]
        angle = float(np.linalg.norm(dth))
        if angle > 1e-12:
            dq = (math.cos(angle / 2.0), *(dth / angle * math.sin(angle / 2.0)))
            self.q = quat_normalize(quat_mul(self.q, dq))
        self.x[3:] += dx[V]
        self.x[:3] += dx[P]
        self.gyro_bias = self.gyro_bias + dx[BG]
        self.accel_bias = self.accel_bias + dx[BA]
        g_reset = np.eye(15)
        g_reset[TH, TH] -= 0.5 * skew(dth)  # the error frame moved: re-express the covariance
        self.P15 = g_reset @ self.P15 @ g_reset.T
        self.P15 = 0.5 * (self.P15 + self.P15.T)

    def _mag_update(self, mag_b: np.ndarray) -> None:
        r = np.array(quat_to_dcm(self.q))  # body -> launch
        z_pred = r.T @ self.mag_ref
        h = np.zeros((3, 15))
        h[:, TH] = attitude_measurement_jacobian(z_pred)
        sm = self.sc.magnetometer
        rm = np.eye(3) * (max(sm.noise_std, 1e-9) * self.cfg.noise_scale) ** 2
        self._measure(mag_b - z_pred, h, rm)
