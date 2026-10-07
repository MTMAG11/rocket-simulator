"""Explicit timing model: every rate, period and latency in the loop, and the derived end-to-end latencies.

The simulator does NOT run everything at one frequency:

* physics: variable step, at most ``simulation.dt_s`` (``descent_dt_s`` under a parachute). Steps are CUT at every sensor sample
  time, controller tick, thrust-curve corner and event, so the physics is advanced exactly to each of them;
* each sensor samples at its own ``rate_hz`` and releases the sample ``latency_s`` later (``startup_delay_s`` for the first);
* the estimator is EVENT-DRIVEN: it updates whenever a new sample is visible (no fixed estimator rate exists);
* the controller (flight computer) ticks at ``controller.rate_hz``; HIL adds ``uplink_latency_s`` (sensor -> computer),
  ``compute_time_s`` and ``downlink_latency_s`` (command -> actuator);
* the actuator advances at every physics step: transport delay ``delay_s``, first-order lag, rate and angle limits.

``timing_report`` returns these as data; ``format_timing`` prints them; ``timing_warnings`` flags combinations that are
suspicious (e.g. a command latency of several control periods, or a physics step coarse relative to the actuator lag).
"""

from __future__ import annotations

from typing import Any

from ..config.schema import SimConfig


def timing_report(cfg: SimConfig) -> dict[str, Any]:
    s = cfg.sensors
    c = cfg.controller
    sensors = {}
    for name, attr in (
        ("accel", "accelerometer"),
        ("gyro", "gyroscope"),
        ("baro", "barometer"),
        ("gps", "gps"),
        ("mag", "magnetometer"),
    ):
        x = getattr(s, attr)
        if x.enabled:
            sensors[name] = {
                "rate_hz": x.rate_hz,
                "period_s": 1.0 / x.rate_hz,
                "latency_s": x.latency_s,
                "startup_delay_s": x.startup_delay_s,
            }
    ctrl_active = c.type != "none"
    ctrl_period = 1.0 / c.rate_hz if ctrl_active else None
    act_delay, act_tau = cfg.tvc.delay_s, cfg.tvc.time_constant_s
    cmd_latency = c.compute_time_s + c.downlink_latency_s
    imu_rate = max((sensors[k]["rate_hz"] for k in ("accel", "gyro") if k in sensors), default=None)
    rep: dict[str, Any] = {
        "physics": {
            "dt_max_s": cfg.simulation.dt_s,
            "dt_descent_s": cfg.simulation.descent_dt_s,
            "integrator": cfg.simulation.integrator,
            "step_cut_at": "sensor sample times, controller ticks, thrust-curve corners, events",
        },
        "sensors": sensors,
        "estimator": {
            "type": cfg.estimator.type,
            "schedule": "event-driven: updates on every newly visible sample (no fixed rate)",
        },
        "controller": {
            "type": c.type,
            "rate_hz": c.rate_hz if ctrl_active else None,
            "period_s": ctrl_period,
            "compute_time_s": c.compute_time_s,
            "uplink_latency_s": c.uplink_latency_s,
            "downlink_latency_s": c.downlink_latency_s,
            "state_source": c.state_source,
        },
        "actuator": {
            "delay_s": act_delay,
            "time_constant_s": act_tau,
            "max_rate_deg_s": cfg.tvc.max_rate_deg_s,
            "update": "every physics step (state held over the step)",
        },
        "derived": {
            "command_to_actuator_latency_s": cmd_latency + act_delay,
            "worst_case_imu_to_actuator_s": (
                (ctrl_period or 0.0)
                + c.uplink_latency_s
                + cmd_latency
                + act_delay
                + max((sensors[k]["latency_s"] for k in ("accel", "gyro") if k in sensors), default=0.0)
            ),
            "worst_case_sensor_to_actuator_s": (
                (ctrl_period or 0.0)  # waits for the next controller tick
                + c.uplink_latency_s
                + cmd_latency
                + act_delay
                + (max(v["latency_s"] for v in sensors.values()) if sensors else 0.0)
            ),
            "imu_samples_per_control_tick": (imu_rate / c.rate_hz) if (imu_rate and ctrl_active) else None,
        },
    }
    rep["warnings"] = timing_warnings(cfg, rep)
    return rep


def timing_warnings(cfg: SimConfig, rep: dict[str, Any]) -> list[str]:
    w: list[str] = []
    c = rep["controller"]
    per = c["period_s"]
    if per is not None:
        if rep["derived"]["command_to_actuator_latency_s"] > 2 * per:
            w.append(
                "command-to-actuator latency exceeds two controller periods: the control loop effectively runs on stale commands"
            )
        if c["uplink_latency_s"] > 0 and cfg.controller.type != "hil":
            w.append(
                "controller.uplink_latency_s only affects the HIL bridge (type: hil); it is ignored here"
            )
        ratio = rep["derived"]["imu_samples_per_control_tick"]
        if ratio is not None and ratio > 1.0 and cfg.controller.type not in ("hil",):
            w.append(
                f"the IMU delivers {ratio:.1f} samples per controller tick; the in-process estimator consumes every one of them, but the controller "
                "only reads the latest estimate once per tick (a HIL flight computer receives all samples)"
            )
    tau = rep["actuator"]["time_constant_s"]
    if tau > 0 and cfg.simulation.dt_s > 2.0 * tau and cfg.controller.type != "none":
        w.append(
            "physics step is more than twice the actuator time constant: the actuator is advanced exactly, "
            "but the physics sees it held constant over the step (left-endpoint hold)"
        )
    return w


def format_timing(rep: dict[str, Any]) -> str:
    p, c, a, d = rep["physics"], rep["controller"], rep["actuator"], rep["derived"]
    L = ["Timing model", "-" * 60]
    L.append(
        f"physics     dt <= {p['dt_max_s'] * 1000:g} ms ({p['dt_descent_s'] * 1000:g} ms under parachute), {p['integrator']}; steps cut at {p['step_cut_at']}"
    )
    for k, v in rep["sensors"].items():
        L.append(
            f"sensor {k:<6} {v['rate_hz']:g} Hz  (period {v['period_s'] * 1000:g} ms)  latency {v['latency_s'] * 1000:g} ms  start-up {v['startup_delay_s']:g} s"
        )
    L.append(f"estimator   {rep['estimator']['type']}: {rep['estimator']['schedule']}")
    if c["rate_hz"]:
        L.append(
            f"controller  {c['type']} at {c['rate_hz']:g} Hz (period {c['period_s'] * 1000:g} ms), compute {c['compute_time_s'] * 1000:g} ms, "
            f"uplink {c['uplink_latency_s'] * 1000:g} ms, downlink {c['downlink_latency_s'] * 1000:g} ms, state source: {c['state_source']}"
        )
    L.append(
        f"actuator    delay {a['delay_s'] * 1000:g} ms, lag {a['time_constant_s'] * 1000:g} ms, rate limit {a['max_rate_deg_s']:g} deg/s; {a['update']}"
    )
    L.append(
        f"derived     command -> actuator {d['command_to_actuator_latency_s'] * 1000:g} ms; worst-case IMU -> actuator {d['worst_case_imu_to_actuator_s'] * 1000:g} ms, any sensor (incl. GPS latency) {d['worst_case_sensor_to_actuator_s'] * 1000:g} ms"
    )
    for w in rep["warnings"]:
        L.append(f"WARNING: {w}")
    return "\n".join(L)
