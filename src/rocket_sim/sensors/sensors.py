"""Simulated sensors: TRUE state in, MEASURED state out.

Each sensor channel applies, in this order,
    m = (1 + k) * truth + b(t) + n          k: scale-factor error (fixed per run, ~N(0, s))
    m = round(m / lsb) * lsb                b: initial bias ~N(0, bias_std) plus random walk
    m = clip(m, -fs, +fs)                   n: white noise N(0, noise_std)
and the result becomes visible ``latency`` seconds after the sample instant. Sampling happens
at ``rate_hz`` (zero-order hold between samples). All randomness is drawn from a per-sensor
generator derived from the run seed, so runs are reproducible.

Sensor list: accelerometer (specific force, body frame), gyroscope (angular rate, body frame),
barometer (static pressure), GPS (position + velocity, launch frame), magnetometer (Earth
field, body frame).

Simplifications (documented, not hidden): no temperature dependence, no cross-axis
misalignment, no vibration/aliasing model, barometer reads *static* pressure (no
dynamic-pressure/transonic port disturbance), no GPS dropout or COCOM altitude/speed limits,
Earth magnetic field constant over the flight. Noise is white + bias random walk (a
first-order approximation of Allan-variance behaviour).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from ..config.schema import SensorCfg, SensorsCfg
from ..constants import EARTH_MAG_FIELD_ENU_T
from ..physics.math3d import Quat, quat_rotate_inv

GPS_VEL_NOISE_RATIO = 0.05  # GPS velocity 1-sigma = ratio * position 1-sigma


class SensorChannel:
    """Generic multi-axis sensor with the error model described in the module docstring."""

    def __init__(self, cfg: SensorCfg, dim: int, rng: np.random.Generator, noise_scale=None) -> None:
        self.cfg = cfg
        self.dim = dim
        self.rng = rng
        self.period = 1.0 / cfg.rate_hz
        self.next_time = 0.0
        scale = np.ones(dim) if noise_scale is None else np.asarray(noise_scale, float)
        self._noise_scale = scale
        self.bias = rng.normal(0.0, cfg.bias_std, dim) * scale if cfg.bias_std > 0 else np.zeros(dim)
        self.scale_err = (
            rng.normal(0.0, cfg.scale_error_std, dim) if cfg.scale_error_std > 0 else np.zeros(dim)
        )
        self._queue: deque[tuple[float, np.ndarray]] = deque()
        self.value = np.zeros(dim)
        self.has_value = False
        self.new = False  # True on steps where a fresh sample became visible

    def due(self, t: float) -> bool:
        return t >= self.next_time - 1e-9

    def sample(self, t: float, truth: np.ndarray) -> None:
        cfg = self.cfg
        if cfg.bias_walk_std > 0:
            self.bias += (
                self.rng.normal(0.0, cfg.bias_walk_std * math.sqrt(self.period), self.dim) * self._noise_scale
            )
        m = (1.0 + self.scale_err) * truth + self.bias
        if cfg.noise_std > 0:
            m = m + self.rng.normal(0.0, cfg.noise_std, self.dim) * self._noise_scale
        if cfg.quantization > 0:
            m = np.round(m / cfg.quantization) * cfg.quantization
        if cfg.saturation > 0:
            m = np.clip(m, -cfg.saturation, cfg.saturation)
        self._queue.append((t + cfg.latency_s, m))
        while self.next_time <= t + 1e-9:
            self.next_time += self.period

    def poll(self, t: float) -> bool:
        """Release samples whose latency has elapsed. Returns True if a new value appeared."""
        fresh = False
        while self._queue and self._queue[0][0] <= t + 1e-9:
            _, self.value = self._queue.popleft()
            self.has_value = True
            fresh = True
        self.new = fresh
        return fresh


@dataclass
class SensorReadings:
    """Latest visible measurements (zero until the first sample arrives; check ``*_new``)."""

    accel: np.ndarray
    gyro: np.ndarray
    baro_pressure: float
    gps: np.ndarray  # [x, y, z, vx, vy, vz]
    mag: np.ndarray
    accel_new: bool = False
    gyro_new: bool = False
    baro_new: bool = False
    gps_new: bool = False
    mag_new: bool = False


class SensorSuite:
    def __init__(self, cfg: SensorsCfg, seeds: list[np.random.SeedSequence]) -> None:
        self.cfg = cfg
        g = [np.random.default_rng(s) for s in seeds]
        self.accel = SensorChannel(cfg.accelerometer, 3, g[0]) if cfg.accelerometer.enabled else None
        self.gyro = SensorChannel(cfg.gyroscope, 3, g[1]) if cfg.gyroscope.enabled else None
        self.baro = SensorChannel(cfg.barometer, 1, g[2]) if cfg.barometer.enabled else None
        gps_scale = [1, 1, 1, GPS_VEL_NOISE_RATIO, GPS_VEL_NOISE_RATIO, GPS_VEL_NOISE_RATIO]
        self.gps = SensorChannel(cfg.gps, 6, g[3], gps_scale) if cfg.gps.enabled else None
        self.mag = SensorChannel(cfg.magnetometer, 3, g[4]) if cfg.magnetometer.enabled else None
        self._channels = [c for c in (self.accel, self.gyro, self.baro, self.gps, self.mag) if c]
        self.mag_ref = np.asarray(EARTH_MAG_FIELD_ENU_T)

    def next_event_time(self) -> float:
        return min((c.next_time for c in self._channels), default=math.inf)

    def update(
        self,
        t: float,
        spec_force_b: tuple[float, float, float],
        omega_b: tuple[float, float, float],
        pressure: float,
        position: tuple[float, float, float],
        velocity: tuple[float, float, float],
        quat: Quat,
    ) -> SensorReadings:
        """Sample every due channel from the TRUE state, release latent samples, return latest."""
        if self.accel and self.accel.due(t):
            self.accel.sample(t, np.asarray(spec_force_b))
        if self.gyro and self.gyro.due(t):
            self.gyro.sample(t, np.asarray(omega_b))
        if self.baro and self.baro.due(t):
            self.baro.sample(t, np.array([pressure]))
        if self.gps and self.gps.due(t):
            self.gps.sample(t, np.array([*position, *velocity]))
        if self.mag and self.mag.due(t):
            b_body = quat_rotate_inv(
                quat, (float(self.mag_ref[0]), float(self.mag_ref[1]), float(self.mag_ref[2]))
            )
            self.mag.sample(t, np.asarray(b_body))
        for c in self._channels:
            c.poll(t)
        z3, z6 = np.zeros(3), np.zeros(6)
        return SensorReadings(
            accel=self.accel.value if self.accel else z3,
            gyro=self.gyro.value if self.gyro else z3,
            baro_pressure=float(self.baro.value[0]) if self.baro else 0.0,
            gps=self.gps.value if self.gps else z6,
            mag=self.mag.value if self.mag else z3,
            accel_new=self.accel.new if self.accel else False,
            gyro_new=self.gyro.new if self.gyro else False,
            baro_new=self.baro.new if self.baro else False,
            gps_new=self.gps.new if self.gps else False,
            mag_new=self.mag.new if self.mag else False,
        )
