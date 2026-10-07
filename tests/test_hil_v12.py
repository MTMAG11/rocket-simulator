"""V1.2 HIL foundation: protocol v2, transports, lock-step loop, timing, deterministic replay, no truth across the boundary."""

import json
import math
import sys

import numpy as np
import pytest

from rocket_sim.config import apply_overrides, config_from_dict, read_mapping
from rocket_sim.constants import G0
from rocket_sim.control import ControlInput
from rocket_sim.errors import ConfigError, SimulationError
from rocket_sim.hil import (
    HilBridgeController,
    LoopbackTransport,
    PipeTransport,
    RecordingTransport,
    ReferenceFlightComputer,
    ReplayTransport,
    build_bridge,
    decode,
    encode,
    handle_message,
)
from rocket_sim.hil.protocol import command_from_reply
from rocket_sim.simulation import Simulation
from rocket_sim.simulation.timing import format_timing, timing_report
from tests.conftest import CONFIG_G80, cfg_from


class LaunchStepFC:
    """Minimal flight software: commands a fixed gimbal once it has MEASURED more than 3 g."""

    def __init__(self, y=0.01):
        self.y = y
        self.inits = []
        self.ticks = []
        self.launched = False

    def reset(self, init):
        self.inits.append(init)
        self.launched = False

    def on_tick(self, tick):
        self.ticks.append(tick)
        for s in tick["samples"]:
            if s["sensor"] == "accel" and math.sqrt(sum(v * v for v in s["value"])) > 3 * G0:
                self.launched = True
        return {"tvc_y": self.y if self.launched else 0.0}


def loopback(fc):
    return LoopbackTransport(lambda b: encode(handle_message(fc, decode(b))))


def hil_cfg(**kw):
    kw.setdefault("sensors", {"accelerometer": {"saturation": 400.0}, "gyroscope": {"rate_hz": 400.0}})
    base = dict(
        fidelity=4,
        motor={"ignition_delay_s": 1.0},
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
        controller={"type": "none", "rate_hz": 50},
        simulation={"t_max_s": 4.0},
        tvc={"max_angle_deg": 5.0, "max_rate_deg_s": 1e4, "time_constant_s": 0.0, "delay_s": 0.0},
    )
    base.update(kw)
    return cfg_from(**base)


def run_hil(fc, seed=1, transport=None, uplink=None, **kw):
    t = transport or loopback(fc)
    bridge = HilBridgeController(t, uplink)
    sim = Simulation(hil_cfg(**kw), seed=seed, controller=bridge)
    return sim.run(), bridge, t


# ------------------------------------------------------------------------------------------- protocol
def test_encoding_is_deterministic_and_rejects_non_finite_numbers():
    a = encode({"v": 2, "type": "tick", "b": 1, "a": [1.5, 2]})
    assert a == encode({"a": [1.5, 2], "type": "tick", "v": 2, "b": 1}) and a.endswith(b"\n")
    assert a == b'{"a":[1.5,2],"b":1,"type":"tick","v":2}\n'
    with pytest.raises(SimulationError, match="non-finite"):
        encode({"v": 2, "x": float("nan")})
    with pytest.raises(SimulationError, match="version"):
        decode(b'{"v":1}\n')
    with pytest.raises(SimulationError, match="malformed"):
        decode(b"{oops\n")


@pytest.mark.parametrize(
    "reply,match",
    [
        ({"v": 2, "type": "command", "seq": 5, "cmd": {}}, "tick 3"),
        ({"v": 2, "type": "ready", "seq": 3}, "tick 3"),
        ({"v": 2, "type": "command", "seq": 3, "cmd": {"tvc_y": "x"}}, "finite number"),
        ({"v": 2, "type": "command", "seq": 3, "cmd": {"tvc_y": float("inf")}}, "finite number"),
        ({"v": 2, "type": "command", "seq": 3, "cmd": {"warp": 1.0}}, "unknown command"),
        ({"v": 2, "type": "command", "seq": 3}, "no 'cmd'"),
    ],
)
def test_malformed_replies_are_rejected(reply, match):
    with pytest.raises(SimulationError, match=match):
        command_from_reply(reply, 3)


def test_missing_command_fields_default_to_zero():
    out = command_from_reply({"v": 2, "type": "command", "seq": 1, "cmd": {"tvc_y": 0.02}}, 1)
    assert out["tvc_y"] == 0.02 and out["tvc_z"] == 0.0 and out["fin_roll"] == 0.0


# ----------------------------------------------------------------------------------- the loop and the boundary
def test_lock_step_loop_runs_and_commands_reach_the_actuator_state():
    fc = LaunchStepFC()
    rec, bridge, _ = run_hil(fc)
    assert len(fc.ticks) == len(bridge.ticks) > 100
    assert [t["seq"] for t in fc.ticks] == list(range(1, len(fc.ticks) + 1))  # strictly ordered, none lost
    t = rec.col("t")
    assert np.all(rec.col("tvc_y")[t < 1.0] == 0.0)  # nothing before launch is measured
    assert np.max(np.abs(rec.col("tvc_y"))) == pytest.approx(
        0.01, rel=1e-6
    )  # the flight computer's command reached the physics
    assert rec.meta.controller_state_source == "measurements_only"
    assert not any("TRUE state" in w for w in rec.meta.warnings)


def test_the_boundary_carries_only_timestamped_sensor_samples_and_design_data():
    fc = LaunchStepFC()
    run_hil(fc)
    init = fc.inits[0]
    assert set(init) >= {"controller_rate_hz", "sensors", "design", "estimator"} and init["type"] == "init"
    forbidden = {
        "pos",
        "vel",
        "position",
        "velocity",
        "quaternion",
        "quat",
        "omega",
        "altitude",
        "mass",
        "cg",
        "phase",
        "truth",
    }
    assert not (forbidden & set(init["design"])) and not (forbidden & set(init))
    for tick in fc.ticks:
        assert set(tick) == {"v", "type", "seq", "t", "samples"}
        for s in tick["samples"]:
            assert set(s) == {"sensor", "t_sample", "t_visible", "value"} and s["sensor"] in (
                "accel",
                "gyro",
                "baro",
                "gps",
                "mag",
            )
            assert s["t_visible"] <= tick["t"] + 1e-9 and s["t_sample"] <= s["t_visible"]


def test_flight_computer_receives_every_sample_not_just_the_latest():
    fc = LaunchStepFC()
    rec, _, _ = run_hil(fc)
    by = {}
    for tick in fc.ticks:
        for s in tick["samples"]:
            by.setdefault(s["sensor"], []).append(s)
    # gyro 400 Hz, accel 100 Hz (config default) with a 50 Hz controller: 8 and 2 samples per tick
    assert len(by["gyro"]) / len(fc.ticks) == pytest.approx(8.0, rel=0.03)
    assert len(by["accel"]) / len(fc.ticks) == pytest.approx(2.0, rel=0.05)
    for name, ss in by.items():  # strictly time-ordered per sensor, no duplicates
        tv = [s["t_visible"] for s in ss]
        assert all(b > a for a, b in zip(tv, tv[1:])), name
    # delivered samples ARE the measurements the simulator logged (and not the truth)
    meas = {round(float(v), 12) for v in rec.col("meas_gyro_y")}
    assert all(round(s["value"][1], 12) in meas for s in by["gyro"])


def test_controller_is_never_handed_a_state_over_the_bridge():
    seen = []

    class Probe(HilBridgeController):
        def update(self, inp: ControlInput):
            seen.append(inp)
            return super().update(inp)

    fc = LaunchStepFC()
    sim = Simulation(hil_cfg(), seed=1, controller=Probe(loopback(fc)))
    sim.run()
    assert seen and all(
        not i.valid and i.position == (0.0, 0.0, 0.0) and i.velocity == (0.0, 0.0, 0.0) for i in seen
    )
    assert all(i.quaternion == (1.0, 0.0, 0.0, 0.0) and i.omega == (0.0, 0.0, 0.0) for i in seen)
    assert all(i.state_source == "none" for i in seen)


def test_uplink_latency_delays_when_samples_reach_the_flight_computer():
    fc0, fc1 = LaunchStepFC(), LaunchStepFC()
    run_hil(fc0)
    run_hil(fc1, uplink=0.08)
    for tick in fc1.ticks:
        for s in tick["samples"]:
            assert (
                s["t_visible"] + 0.08 <= tick["t"] + 1e-9
            )  # delivered only once the uplink delay has elapsed
    # same samples, just later: total delivered differs by at most the in-flight window
    n0 = sum(len(t["samples"]) for t in fc0.ticks)
    n1 = sum(len(t["samples"]) for t in fc1.ticks)
    assert 0 < n0 - n1 < 0.08 * (100 + 400 + 50 + 50 + 5) * 1.5


def test_compute_and_downlink_latency_delay_the_actuator_by_exactly_that_much():
    lat = 0.12
    base, _, _ = run_hil(LaunchStepFC())
    late, _, _ = run_hil(
        LaunchStepFC(),
        controller={"type": "none", "rate_hz": 50, "compute_time_s": 0.05, "downlink_latency_s": 0.07},
    )
    t0, t1 = base.col("t"), late.col("t")
    first = lambda r, t: t[np.argmax(np.abs(r.col("tvc_y")) > 1e-9)]  # noqa: E731
    first_cmd = lambda r, t: t[np.argmax(np.abs(r.col("tvc_cmd_y")) > 1e-9)]  # noqa: E731
    d0 = first(base, t0) - first_cmd(base, t0)
    d1 = first(late, t1) - first_cmd(late, t1)
    assert d0 <= 0.011  # ideal actuator: the command takes effect within a step
    assert d1 - d0 == pytest.approx(
        lat, abs=0.011
    )  # compute + downlink, quantised to the physics/sensor step
    cfg = hil_cfg(
        controller={"type": "none", "rate_hz": 50, "compute_time_s": 0.05, "downlink_latency_s": 0.07}
    )
    assert timing_report(cfg)["derived"]["command_to_actuator_latency_s"] == pytest.approx(lat)


# ------------------------------------------------------------------------- determinism and replay
def test_live_runs_are_bit_identical_and_replay_reproduces_without_a_flight_computer():
    rec_t = RecordingTransport(loopback(LaunchStepFC()))
    r1, _, _ = run_hil(None, transport=rec_t)
    r2, _, _ = run_hil(LaunchStepFC())
    assert np.array_equal(r1.col("pos_z"), r2.col("pos_z")) and np.array_equal(
        r1.col("tvc_y"), r2.col("tvc_y")
    )
    assert len(rec_t.entries) > 100
    # replay: no flight computer exists any more
    r3, _, rep = run_hil(None, transport=ReplayTransport(rec_t.entries))
    assert rep.i == len(rec_t.entries)  # every recorded message was consumed
    for c in ("pos_z", "vel_z", "tvc_y", "tvc_cmd_y", "meas_accel_x", "meas_gyro_y"):
        assert np.array_equal(r1.col(c), r3.col(c)), c


def test_replay_detects_a_changed_simulation(tmp_path):
    rec_t = RecordingTransport(loopback(LaunchStepFC()))
    run_hil(None, transport=rec_t)
    p = tmp_path / "s.jsonl"
    rec_t.write_jsonl(p)
    again = ReplayTransport.from_jsonl(p)
    with pytest.raises(SimulationError, match="diverged"):
        run_hil(None, transport=again, seed=99)  # different sensor noise => different requests
    short = ReplayTransport(rec_t.entries[:5])
    with pytest.raises(SimulationError, match="exhausted"):
        run_hil(None, transport=short)


def test_a_child_process_flight_computer_gives_the_same_flight_as_the_in_process_one():
    """The same bytes over a pipe: what a firmware host build or a serial link would carry."""
    pipe = PipeTransport([sys.executable, "-m", "rocket_sim.hil.flight_computer"])
    try:
        r_pipe, b, _ = run_hil(
            None, transport=pipe, simulation={"t_max_s": 3.0}, motor={"ignition_delay_s": 0.5}
        )
    finally:
        pipe.close()
    fc = ReferenceFlightComputer()
    r_loop, _, _ = run_hil(
        None, transport=loopback(fc), simulation={"t_max_s": 3.0}, motor={"ignition_delay_s": 0.5}
    )
    assert len(b.ticks) > 100
    assert np.array_equal(r_pipe.col("pos_z"), r_loop.col("pos_z"))
    assert np.array_equal(r_pipe.col("tvc_cmd_y"), r_loop.col("tvc_cmd_y"))


def test_a_dead_flight_computer_process_is_an_error_not_a_hang():
    pipe = PipeTransport([sys.executable, "-c", "import sys; sys.exit(3)"])
    with pytest.raises(SimulationError, match="process"):
        run_hil(None, transport=pipe)
    pipe.close()


# ----------------------------------------------------------------------------- closed loop with real software
def tvc_closed_loop_cfg(extra):
    raw = read_mapping(CONFIG_G80.parent / "example_tvc_closed_loop.yaml")
    ov = {"simulation.t_max_s": 5.5, "fidelity": 4, "estimator.type": "none", **extra}
    return config_from_dict(apply_overrides(raw, ov), base_dir=CONFIG_G80.parent)


def tilt_during_burn(rec):
    t = rec.col("t")
    burn = rec.summary["burnout_time_s"]
    w = (t > 2.2) & (t < burn)
    return np.degrees(np.arccos(np.clip(np.sin(rec.col("pitch")), -1, 1)))[w].max()


def test_reference_flight_computer_stabilises_an_unstable_vehicle_through_the_protocol():
    cfg = tvc_closed_loop_cfg({"controller.type": "hil", "controller.params": {"fc": "reference"}})
    rec = Simulation(cfg, seed=3).run()
    free = Simulation(tvc_closed_loop_cfg({"controller.type": "none"}), seed=3).run()
    assert tilt_during_burn(rec) < 8.0  # held near vertical by software that only saw sensor samples
    assert tilt_during_burn(free) > 60.0  # the same vehicle tumbles without it
    assert rec.meta.controller_state_source == "measurements_only"
    assert (
        max(np.max(np.abs(rec.col("tvc_y"))), np.max(np.abs(rec.col("tvc_z")))) > 5e-4
    )  # it actually steered


def test_hil_command_latency_degrades_the_loop_measurably_and_eventually_loses_control():
    def tilt(lat):
        extra = {"controller.compute_time_s": lat} if lat else {}
        cfg = tvc_closed_loop_cfg(
            {"controller.type": "hil", "controller.params": {"fc": "reference"}, **extra}
        )
        return tilt_during_burn(Simulation(cfg, seed=3).run())

    t0, t10, t20, t40 = tilt(0.0), tilt(0.01), tilt(0.02), tilt(0.04)
    assert t0 < 5.0  # measured 3.5 deg
    assert t10 > t0 + 2.0  # 10 ms of latency doubles the excursion (measured 7.6 deg)
    assert t20 > t10 + 3.0  # 20 ms: 17 deg
    assert t40 > 90.0  # 40 ms: the unstable vehicle is lost (tumbles)


def test_in_process_and_hil_loops_behave_the_same_with_the_same_latency():
    """Both paths index the gimbal-authority schedule by time since detected launch and use the same algorithms."""

    def tilt(extra):
        return tilt_during_burn(Simulation(tvc_closed_loop_cfg(extra), seed=3).run())

    hil = tilt(
        {
            "controller.type": "hil",
            "controller.params": {"fc": "reference"},
            "controller.compute_time_s": 0.01,
        }
    )
    inproc = tilt({"estimator.type": "nav_kf", "fidelity": 5, "controller.compute_time_s": 0.01})
    assert hil == pytest.approx(inproc, abs=1.0)


def test_command_latency_delays_fin_commands_too():
    cs = {
        "count": 4,
        "root_chord_m": 0.10,
        "tip_chord_m": 0.06,
        "span_m": 0.09,
        "sweep_m": 0.02,
        "position_from_nose_m": 0.88,
        "max_deflection_deg": 20.0,
        "max_rate_deg_s": 1e4,
        "time_constant_s": 0.0,
    }
    kw = {"rocket": {"parachutes": [], "control_surfaces": cs}, "simulation": {"t_max_s": 3.0}}
    base, _, _ = run_hil(FinFC(), **kw)
    late, _, _ = run_hil(FinFC(), controller={"type": "none", "rate_hz": 50, "compute_time_s": 0.1}, **kw)

    def lag(r):
        t = r.col("t")
        return (
            t[np.argmax(np.abs(r.col("fin_0")) > 1e-9)] - t[np.argmax(np.abs(r.col("fin_cmd_pitch")) > 1e-9)]
        )

    assert lag(late) - lag(base) == pytest.approx(0.1, abs=0.015)


def test_replay_must_be_complete_and_uplink_parameters_are_validated():
    rec_t = RecordingTransport(loopback(LaunchStepFC()))
    run_hil(None, transport=rec_t)
    full = ReplayTransport(rec_t.entries)
    r, bridge, _ = run_hil(None, transport=full)
    bridge.finish()  # complete: no error
    short_run = ReplayTransport(rec_t.entries)
    _, bridge2, _ = run_hil(
        None, transport=short_run, simulation={"t_max_s": 2.0}
    )  # fewer ticks than recorded
    with pytest.raises(SimulationError, match="replay incomplete"):
        bridge2.finish()
    for bad in (float("nan"), -0.5, float("inf")):
        with pytest.raises(ConfigError, match="uplink_latency_s"):
            HilBridgeController(loopback(LaunchStepFC()), bad)
    with pytest.raises(ConfigError, match="uplink_latency_s"):
        build_bridge({"fc": "reference", "uplink_latency_s": float("nan")})


# ----------------------------------------------------------------------------------- configuration / CLI
def test_build_bridge_validates_parameters(tmp_path):
    with pytest.raises(ConfigError, match="give fc"):
        build_bridge({"fc": "nonsense"})
    assert isinstance(build_bridge({"fc": "reference"}), HilBridgeController)
    assert isinstance(build_bridge({"fc": "reference", "record": True}).transport, RecordingTransport)


def test_hil_needs_sensors():
    cfg = cfg_from(
        fidelity=3, controller={"type": "hil", "params": {"fc": "reference"}}, simulation={"t_max_s": 2}
    )
    with pytest.raises(SimulationError, match="HIL init"):
        Simulation(cfg, seed=0).run()


def test_cli_hil_run_logs_the_loop_and_replays_it(tmp_path, capsys):
    from rocket_sim.cli import main

    log = tmp_path / "loop.jsonl"
    cfgp = CONFIG_G80.parent / "example_tvc_closed_loop.yaml"
    args = ["hil", str(cfgp), "--log", str(log)]
    # shorten through a temporary copy of the config
    raw = read_mapping(cfgp)
    raw["simulation"]["t_max_s"] = 4.0
    raw["motor"]["file"] = str(CONFIG_G80.parent.parent / "data" / "motors" / "AeroTech_G80T.eng")
    short = tmp_path / "short.yaml"
    import yaml

    short.write_text(yaml.safe_dump(raw), encoding="utf-8")
    assert main(["hil", str(short), "--log", str(log)]) == 0
    out = capsys.readouterr().out
    assert "HIL run:" in out and log.exists()
    lines = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()]
    assert (
        json.loads(lines[0]["request"])["type"] == "init"
        and json.loads(lines[0]["response"])["type"] == "ready"
    )
    assert all(json.loads(e["request"])["v"] == 2 for e in lines)
    assert main(["hil", str(short), "--replay", str(log)]) == 0
    assert args  # (argument vector kept for readability)


def test_timing_report_lists_every_rate_and_latency_and_warns_sensibly(capsys):
    cfg = cfg_from(
        fidelity=4,
        sensors={
            "gps": {"rate_hz": 10.0, "latency_s": 0.3},
            "accelerometer": {"rate_hz": 800.0, "latency_s": 0.002},
        },
        controller={
            "type": "schedule",
            "rate_hz": 20,
            "compute_time_s": 0.1,
            "downlink_latency_s": 0.1,
            "uplink_latency_s": 0.02,
            "params": {"table": [[0, 0, 0]]},
        },
        tvc={"max_angle_deg": 5, "max_rate_deg_s": 100, "time_constant_s": 0.01, "delay_s": 0.02},
        simulation={"dt_s": 0.01},
    )
    rep = timing_report(cfg)
    assert rep["sensors"]["gps"] == {
        "rate_hz": 10.0,
        "period_s": 0.1,
        "latency_s": 0.3,
        "startup_delay_s": 0.0,
    }
    assert rep["controller"]["period_s"] == pytest.approx(0.05)
    assert rep["derived"]["command_to_actuator_latency_s"] == pytest.approx(0.22)
    assert rep["derived"]["worst_case_sensor_to_actuator_s"] == pytest.approx(
        0.05 + 0.02 + 0.2 + 0.02 + 0.3
    )  # includes the 300 ms GPS latency
    assert rep["derived"]["worst_case_imu_to_actuator_s"] == pytest.approx(0.05 + 0.02 + 0.2 + 0.02 + 0.002)
    assert rep["derived"]["imu_samples_per_control_tick"] == pytest.approx(40.0)
    w = " ".join(rep["warnings"])
    assert (
        "two controller periods" in w
        and "uplink_latency_s only affects the HIL" in w
        and "samples per controller tick" in w
    )
    assert "event-driven" in format_timing(rep)
    from rocket_sim.cli import main

    assert main(["timing", str(CONFIG_G80)]) == 0
    assert "Timing model" in capsys.readouterr().out


def test_hil_v1_message_no_longer_leaks_the_flight_phase():
    from rocket_sim.control import HilController

    sent = []

    class Link:
        def send(self, data):
            sent.append(json.loads(data.decode()))

        def recv(self):
            return b'{"tvc_y": 0.0, "tvc_z": 0.0}\n'

    Simulation(hil_cfg(), seed=1, controller=HilController(Link())).run()
    assert sent and all("phase" not in m for m in sent)


# ------------------------------------------------------------------------- round-1 review additions
NOISY = {
    "accelerometer": {"saturation": 400.0, "noise_std": 0.05, "rate_hz": 100.0},
    "gyroscope": {"rate_hz": 400.0, "noise_std": 0.002},
    "barometer": {"noise_std": 3.0},
    "gps": {"noise_std": 1.5},
    "magnetometer": {"noise_std": 1e-7},
}


def assert_delivered_samples_are_the_logged_measurements(rec, fc):
    """Every delivered value is bit-identical to a measurement the simulator logged (so no truth, no alteration)."""
    cols = {
        "accel": (["meas_accel_x", "meas_accel_y", "meas_accel_z"], "meas_accel_new"),
        "gyro": (["meas_gyro_x", "meas_gyro_y", "meas_gyro_z"], "meas_gyro_new"),
        "baro": (["meas_baro_pressure"], "meas_baro_new"),
        "gps": (
            [f"meas_gps_{k}_{a}" for k in ("pos", "vel") for a in "xyz"],
            "meas_gps_new",
        ),
        "mag": (["meas_mag_x", "meas_mag_y", "meas_mag_z"], "meas_mag_new"),
    }
    logged = {}
    for name, (cs, flag) in cols.items():
        new = rec.col(flag) > 0.5
        logged[name] = {tuple(float(rec.col(c)[i]) for c in cs) for i in np.flatnonzero(new)}
    n = {k: 0 for k in cols}
    for tick in fc.ticks:
        for s in tick["samples"]:
            n[s["sensor"]] += 1
            assert tuple(s["value"]) in logged[s["sensor"]], (s["sensor"], s["value"])
    assert all(v > 0 for v in n.values())  # every sensor type was exercised


def test_delivered_samples_equal_the_logged_measurements_for_every_sensor_with_noisy_sensors():
    fc = LaunchStepFC()
    rec, _, _ = run_hil(fc, sensors=NOISY, simulation={"t_max_s": 6.0})
    # the sensors really are imperfect here: measurement != truth, so a leak of truth would be visible
    new = rec.col("meas_accel_new") > 0.5
    assert np.median(np.abs(rec.col("meas_accel_z")[new] - rec.col("fsp_z")[new])) > 0.01
    assert_delivered_samples_are_the_logged_measurements(rec, fc)


def test_the_leak_check_itself_can_fail(monkeypatch):
    """Negative control: if the bridge delivered altered values the check above must fire."""
    from rocket_sim.sensors import SensorSuite

    orig = SensorSuite.drain

    def tampered(self):
        out = orig(self)
        for s in out:
            if s["sensor"] == "accel":
                s["value"] = [
                    v + 1e-3 for v in s["value"]
                ]  # stand-in for 'truth substituted for the measurement'
        return out

    monkeypatch.setattr(SensorSuite, "drain", tampered)
    fc = LaunchStepFC()
    rec, _, _ = run_hil(fc, sensors=NOISY, simulation={"t_max_s": 4.0})
    with pytest.raises(AssertionError):
        assert_delivered_samples_are_the_logged_measurements(rec, fc)


def test_design_block_and_init_are_exactly_the_documented_whitelist():
    fc = LaunchStepFC()
    run_hil(fc)
    init = fc.inits[0]
    assert set(init) == {"v", "type", "controller_rate_hz", "sensors", "design", "estimator"}
    assert set(init["design"]) == {
        "authority_since_ignition",
        "gravity_m_s2",
        "site_elevation_m",
        "launch_axis",
        "alignment_time_s",
        "magnetic_field_enu_t",
    }
    assert all(set(v) == {"rate_hz", "noise_std", "latency_s"} for v in init["sensors"].values())
    assert set(init["estimator"]) == {"alignment_time_s"}


def test_design_inertia_scale_models_imperfect_knowledge_of_the_vehicle():
    a, b = LaunchStepFC(), LaunchStepFC()
    run_hil(a)
    run_hil(b, controller={"type": "none", "rate_hz": 50, "design_inertia_scale": 2.0})
    ta, tb = (
        a.inits[0]["design"]["authority_since_ignition"],
        b.inits[0]["design"]["authority_since_ignition"],
    )
    pairs = [(x[1], y[1]) for x, y in zip(ta, tb) if x[1] is not None]
    assert pairs and all(y == pytest.approx(2.0 * x, rel=1e-12) for x, y in pairs)


def test_uplink_latency_from_the_simulation_config_is_honoured():
    fc = LaunchStepFC()
    run_hil(fc, controller={"type": "none", "rate_hz": 50, "uplink_latency_s": 0.2})
    lags = [tick["t"] - s["t_visible"] for tick in fc.ticks for s in tick["samples"]]
    assert (
        lags and min(lags) >= 0.2 - 1e-9
    )  # the config field reaches the bridge (it used to be silently ignored)
    ref = LaunchStepFC()
    run_hil(ref)
    assert min(tick["t"] - s["t_visible"] for tick in ref.ticks for s in tick["samples"]) < 0.02


def test_samples_with_equal_visibility_time_are_ordered_like_the_in_process_filter():
    fc = LaunchStepFC()
    run_hil(fc, sensors=NOISY)
    order = {"gyro": 0, "accel": 1, "mag": 2, "baro": 3, "gps": 4}
    for tick in fc.ticks:
        keys = [(s["t_visible"], order[s["sensor"]]) for s in tick["samples"]]
        assert keys == sorted(keys)


class FinFC(LaunchStepFC):
    def on_tick(self, tick):
        out = super().on_tick(tick)
        return {
            **out,
            "tvc_y": 0.0,
            "fin_pitch": 0.03 if self.launched else 0.0,
            "fin_roll": 0.01 if self.launched else 0.0,
        }


def test_fin_commands_travel_through_the_bridge_to_the_fin_actuators():
    cs = {
        "count": 4,
        "root_chord_m": 0.10,
        "tip_chord_m": 0.06,
        "span_m": 0.09,
        "sweep_m": 0.02,
        "position_from_nose_m": 0.88,
        "max_deflection_deg": 20.0,
        "max_rate_deg_s": 400.0,
        "time_constant_s": 0.01,
    }
    rec, _, _ = run_hil(
        FinFC(), rocket={"parachutes": [], "control_surfaces": cs}, simulation={"t_max_s": 3.0}
    )
    t = rec.col("t")
    assert np.all(np.abs(rec.col("fin_0")[t < 1.0]) < 1e-12)
    fins = np.stack([rec.col(f"fin_{i}") for i in range(4)])
    assert np.abs(fins).max() > np.radians(0.3) and np.abs(fins).max() <= np.radians(20.0) + 1e-9
    assert np.abs(rec.col("fin_cmd_pitch")).max() == pytest.approx(0.03)
    assert (
        np.abs(fins - np.stack([rec.col(f"fin_dcmd_{i}") for i in range(4)])).max() > 1e-4
    )  # actual != commanded


def test_a_silent_flight_computer_process_times_out_instead_of_hanging():
    pipe = PipeTransport([sys.executable, "-c", "import time; time.sleep(30)"], timeout_s=0.5)
    try:
        with pytest.raises(SimulationError, match="did not reply"):
            run_hil(None, transport=pipe)
    finally:
        pipe.close()


def test_bridge_close_closes_the_transport():
    pipe = PipeTransport([sys.executable, "-m", "rocket_sim.hil.flight_computer"])
    HilBridgeController(pipe).close()
    assert pipe.proc.poll() is not None


# --------------------------------------------------------------------------- round-3 review additions (CLI path)
def short_tvc_yaml(tmp_path, **controller):
    import yaml

    raw = read_mapping(CONFIG_G80.parent / "example_tvc_closed_loop.yaml")
    raw["simulation"]["t_max_s"] = 4.0
    raw["motor"]["file"] = str(CONFIG_G80.parent.parent / "data" / "motors" / "AeroTech_G80T.eng")
    raw["controller"] = {**raw["controller"], **controller}
    p = tmp_path / "short.yaml"
    p.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return p


def test_cli_hil_honours_the_config_uplink_latency_and_an_explicit_flag_overrides_it(tmp_path):
    from rocket_sim.cli import main

    p = short_tvc_yaml(tmp_path, uplink_latency_s=0.05)

    def min_lag(log):
        lags = []
        for line in log.read_text(encoding="utf-8").splitlines():
            req = json.loads(json.loads(line)["request"])
            if req["type"] == "tick":
                lags += [req["t"] - s["t_visible"] for s in req["samples"]]
        return min(lags)

    log1, log2 = tmp_path / "a.jsonl", tmp_path / "b.jsonl"
    assert main(["hil", str(p), "--log", str(log1)]) == 0
    assert (
        min_lag(log1) >= 0.05 - 1e-9
    )  # config value reaches the bridge (used to be forced to 0 by the CLI default)
    assert main(["hil", str(p), "--log", str(log2), "--uplink", "0.2"]) == 0
    assert min_lag(log2) >= 0.2 - 1e-9


def test_cli_hil_can_launch_a_flight_computer_process_with_dash_m(tmp_path):
    from rocket_sim.cli import main

    p = short_tvc_yaml(tmp_path)
    log = tmp_path / "pipe.jsonl"
    assert (
        main(
            [
                "hil",
                str(p),
                "--log",
                str(log),
                "--command",
                sys.executable,
                "-m",
                "rocket_sim.hil.flight_computer",
            ]
        )
        == 0
    )
    ref = tmp_path / "ref.jsonl"
    assert main(["hil", str(p), "--log", str(ref)]) == 0
    assert log.read_bytes() == ref.read_bytes()  # a child process produces the identical session


def test_state_source_estimate_is_accepted_for_the_hil_bridge(tmp_path):
    from rocket_sim.cli import main

    p = short_tvc_yaml(
        tmp_path, state_source="estimate"
    )  # meaningless for a bridge, but must not break the CLI
    assert main(["hil", str(p)]) == 0
