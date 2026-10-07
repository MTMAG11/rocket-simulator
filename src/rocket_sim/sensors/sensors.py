"""Simulated sensors: TRUE state in, MEASURED state out.

Each sensor channel applies, in this order,
    m = R_mis ((1 + k) * truth) + b(t) + n   k: scale-factor error (fixed per run, ~N(0, s))
    m = round(m / lsb) * lsb                 R_mis: small random axis misalignment (3-axis sensors)
    m = clip(m, -fs, +fs)                    b: initial bias ~N(0, bias_std) plus random walk
                                             n: white noise N(0, noise_std)
and the result becomes visible ``latency`` seconds after the sample instant. Channels sample at their own
``rate_hz`` (asynchronously: IMU, barometer, GPS and magnetometer have different rates), hold the last value
between samples, and expose ``new`` flags so consumers can tell a fresh sample from a held one.
All randomness comes from per-sensor generators derived from the run seed, so runs are reproducible.

GPS additionally has ``startup_delay_s`` (no fix before this time), ``dropout_probability`` (each fix is lost
independently with this probability: Bernoulli) and separate position / velocity noise
(``noise_std`` / ``velocity_noise_std``; default velocity noise 0.05 x position noise).

Sensors (see docs/sensors.md for the audited list of what is and is not modelled):
  accelerometer  specific force, body axes          gyroscope   angular rate, body axes
  barometer      STATIC PRESSURE [Pa] (the altitude conversion lives in the consumer, see estimation)
  GPS            position + velocity, launch frame  magnetometer  Earth field in body axes

Simplifications: no temperature dependence, no vibration/aliasing, no g-sensitivity, no accelerometer
dynamics, barometer reads static pressure (no dynamic-pressure / transonic port disturbance and no
avionics-bay vent lag: real altimeters show this in boost, see validation notes), no COCOM GPS limits, field
constant over the flight. Noise is white + bias random walk (a first-order approximation of Allan-variance
behaviour). Default parameters are representative MEMS orders of magnitude, not a datasheet.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from ..config.schema import SensorCfg, SensorsCfg
from ..physics.math3d import Quat, quat_rotate_inv

GPS_VEL_NOISE_RATIO = 0.05  # default GPS velocity 1-sigma = ratio * position 1-sigma


def _small_rotation(rng: np.random.Generator, std_rad: float) -> np.ndarray:
    """Random small-angle rotation matrix (axis misalignment), exact Rodrigues form."""
    v = rng.normal(0.0, std_rad, 3)
    ang = float(np.linalg.norm(v))
    if ang < 1e-15:
        return np.eye(3)
    k = v / ang
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(ang) * kx + (1.0 - math.cos(ang)) * (kx @ kx)


class SensorChannel:
    """Generic multi-axis sensor with the error model described in the module docstring."""

    def __init__(
        self,
        cfg: SensorCfg,
        dim: int,
        rng: np.random.Generator,
        noise_std_vec: np.ndarray | None = None,
    ) -> None:
        self.cfg = cfg
        self.dim = dim
        self.rng = rng
        self.period = 1.0 / cfg.rate_hz
        self.next_time = 0.0
        self._noise = (
            np.full(dim, cfg.noise_std) if noise_std_vec is None else np.asarray(noise_std_vec, float)
        )
        bias_scale = (
            self._noise / cfg.noise_std if (cfg.noise_std > 0 and noise_std_vec is not None) else np.ones(dim)
        )
        self._bias_scale = bias_scale
        self.bias = rng.normal(0.0, cfg.bias_std, dim) * bias_scale if cfg.bias_std > 0 else np.zeros(dim)
        self.scale_err = (
            rng.normal(0.0, cfg.scale_error_std, dim) if cfg.scale_error_std > 0 else np.zeros(dim)
        )
        self.misalign = (
            _small_rotation(rng, math.radians(cfg.misalignment_std_deg))
            if (cfg.misalignment_std_deg > 0 and dim == 3)
            else None
        )
        self._queue: deque[tuple[float, np.ndarray]] = deque()
        self.value = np.zeros(dim)
        self.has_value = False
        self.new = False  # True on steps where a fresh sample became visible
        self.dropped = 0  # fixes lost (startup delay or dropout)
        self.collect: list[tuple[float, float, np.ndarray]] | None = (
            None  # (t_visible, t_sample, value) since last drain
        )

    def due(self, t: float) -> bool:
        return t >= self.next_time - 1e-9

    def _advance(self, t: float) -> None:
        while self.next_time <= t + 1e-9:
            self.next_time += self.period

    def sample(self, t: float, truth: np.ndarray) -> None:
        cfg = self.cfg
        if t < cfg.startup_delay_s - 1e-12 or (
            cfg.dropout_probability > 0 and self.rng.random() < cfg.dropout_probability
        ):
            self.dropped += 1
            self._advance(t)
            return
        if cfg.bias_walk_std > 0:
            self.bias += (
                self.rng.normal(0.0, cfg.bias_walk_std * math.sqrt(self.period), self.dim) * self._bias_scale
            )
        x = (1.0 + self.scale_err) * truth
        if self.misalign is not None:
            x = self.misalign @ x
        m = x + self.bias
        if cfg.noise_std > 0 or self._noise.any():
            m = m + self.rng.normal(0.0, 1.0, self.dim) * self._noise
        if cfg.quantization > 0:
            m = np.round(m / cfg.quantization) * cfg.quantization
        if cfg.saturation > 0:
            m = np.clip(m, -cfg.saturation, cfg.saturation)
        self._queue.append((t + cfg.latency_s, m))
        self._advance(t)

    def poll(self, t: float) -> bool:
        """Release samples whose latency has elapsed. Returns True if a new value appeared."""
        fresh = False
        while self._queue and self._queue[0][0] <= t + 1e-9:
            tv, self.value = self._queue.popleft()
            if self.collect is not None:
                self.collect.append((tv, tv - self.cfg.latency_s, self.value.copy()))
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
    gps_has_fix: bool = False  # True once any GPS fix has been received


class SensorSuite:
    def __init__(self, cfg: SensorsCfg, seeds: list[np.random.SeedSequence]) -> None:
        self.cfg = cfg
        g = [np.random.default_rng(s) for s in seeds]
        self.accel = SensorChannel(cfg.accelerometer, 3, g[0]) if cfg.accelerometer.enabled else None
        self.gyro = SensorChannel(cfg.gyroscope, 3, g[1]) if cfg.gyroscope.enabled else None
        self.baro = SensorChannel(cfg.barometer, 1, g[2]) if cfg.barometer.enabled else None
        gp = cfg.gps
        vel_std = (
            gp.velocity_noise_std if gp.velocity_noise_std is not None else GPS_VEL_NOISE_RATIO * gp.noise_std
        )
        gps_noise = np.array([gp.noise_std] * 3 + [vel_std] * 3)
        self.gps = SensorChannel(gp, 6, g[3], gps_noise) if gp.enabled else None
        self.mag = SensorChannel(cfg.magnetometer, 3, g[4]) if cfg.magnetometer.enabled else None
        self._channels = [c for c in (self.accel, self.gyro, self.baro, self.gps, self.mag) if c]
        self.mag_ref = np.asarray(cfg.magnetic_field_enu_t, float)

    def _named(self) -> list[tuple[str, SensorChannel]]:
        pairs = (
            ("accel", self.accel),
            ("gyro", self.gyro),
            ("baro", self.baro),
            ("gps", self.gps),
            ("mag", self.mag),
        )
        return [(n, c) for n, c in pairs if c is not None]

    def enable_collection(self) -> None:
        """Start keeping EVERY released sample (for flight computers that must see all of them, not just the latest)."""
        for _, c in self._named():
            if c.collect is None:
                c.collect = []

    def drain(self) -> list[dict]:
        """Samples released since the last drain: ``{sensor, t_sample, t_visible, value}``, ordered by visibility time."""
        out: list[dict] = []
        for name, c in self._named():
            if c.collect:
                out += [
                    {"sensor": name, "t_sample": ts, "t_visible": tv, "value": [float(x) for x in v]}
                    for tv, ts, v in c.collect
                ]
                c.collect.clear()
        order = {
            "gyro": 0,
            "accel": 1,
            "mag": 2,
            "baro": 3,
            "gps": 4,
        }  # same precedence as the in-process filter: rates before specific force
        out.sort(key=lambda d: (d["t_visible"], order[d["sensor"]]))
        return out

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
            gps_has_fix=self.gps.has_value if self.gps else False,
        )
