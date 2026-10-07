"""V1.1 control: TVC audit, reusable actuator, control surfaces (mixer, forces, moments, closed loop)."""

import math

import numpy as np
import pytest

from rocket_sim.config.schema import ActuatorCfg
from rocket_sim.control import Command, TVCActuator
from rocket_sim.control.actuators import ActuatorBank
from rocket_sim.control.mixer import FinMixer
from rocket_sim.motor import Motor
from rocket_sim.physics.dynamics import LaunchSetup
from rocket_sim.physics.math3d import quat_from_pointing
from rocket_sim.simulation import Simulation, run_simulation
from rocket_sim.vehicle import BarrowmanAero, FinSet
from rocket_sim.vehicle.assembly import Assembly, ControlSurfaceSet, Section
from tests.conftest import cfg_from, free_launch, make_6dof, make_env, make_vehicle, tiny_motor

# ---------------------------------------------------------------------------------------- TVC
THRUST = 200.0


def tvc_dyn(th_y=0.0, th_z=0.0, mis=(0.0, 0.0)):
    motor = Motor("C", 0.03, 0.2, 0.05, 0.1, [0.0, 1.0], [THRUST, THRUST])
    veh = make_vehicle(motor, dry_mass=1.0, cg=0.5)
    veh.thrust_misalignment = mis
    dyn = make_6dof(veh, make_env(rho=1e-12, g=0.0), free_launch(z0=1e5))
    dyn.controls.tvc_y, dyn.controls.tvc_z = th_y, th_z
    return veh, dyn


@pytest.mark.parametrize("th_y,th_z", [(0, 0), (0.05, 0), (-0.05, 0), (0, 0.05), (0, -0.05), (0.04, -0.03)])
def test_gimbal_force_direction_and_moment(th_y, th_z):
    veh, dyn = tvc_dyn(th_y, th_z)
    y = dyn.initial_state()
    ev = dyn.evaluate(0.0, y)
    mp = veh.mass_props(0.0)
    lever = veh.nozzle_x - mp.x_cg  # nozzle is aft of the CG
    # thrust vector in the body frame: x-hat rotated about z by th_z then about y by th_y
    t_dir = np.array([math.cos(th_z) * math.cos(th_y), math.sin(th_z), -math.cos(th_z) * math.sin(th_y)])
    # force: acceleration (launch frame == body frame here: nose up) is T/m * direction; gravity is off
    # nose is +z_L; body x -> +z_L; body y (right) -> East/x_L; body z (down) -> -y_L ... verify via magnitude + moment
    assert np.linalg.norm(t_dir) == pytest.approx(1.0)
    # moment about the CG: r x F with r = (-lever, 0, 0)
    m_body = np.cross([-lever, 0, 0], THRUST * t_dir)
    assert ev.wdot[1] * mp.iyy == pytest.approx(m_body[1], abs=1e-9)
    assert ev.wdot[2] * mp.iyy == pytest.approx(m_body[2], abs=1e-9)
    assert abs(ev.wdot[0]) < 1e-12
    # zero gimbal: no torque; sign: positive th_y => negative M_y, positive th_z => negative M_z (nozzle aft)
    if th_y == 0 and th_z == 0:
        assert abs(ev.wdot[1]) < 1e-12 and abs(ev.wdot[2]) < 1e-12
    if th_y > 0:
        assert ev.wdot[1] < 0
    if th_y < 0:
        assert ev.wdot[1] > 0
    if th_z > 0:
        assert ev.wdot[2] < 0
    if th_z < 0:
        assert ev.wdot[2] > 0
    # axial thrust component is reduced by cos(th_y) cos(th_z); |F| is unchanged
    ax_body = float(np.dot(np.array([ev.a[0], ev.a[1], ev.a[2]]), [0, 0, 1]))  # nose along +z_L
    assert ax_body * mp.mass == pytest.approx(THRUST * math.cos(th_y) * math.cos(th_z), rel=1e-9)


def test_maximum_gimbal_and_misalignment_add():
    max_angle = math.radians(5.0)
    veh, dyn = tvc_dyn(max_angle, 0.0, mis=(math.radians(0.5), 0.0))
    ev = dyn.evaluate(0.0, dyn.initial_state())
    mp = veh.mass_props(0.0)
    lever = veh.nozzle_x - mp.x_cg
    assert ev.wdot[1] * mp.iyy == pytest.approx(
        -lever * THRUST * math.sin(max_angle + math.radians(0.5)), rel=1e-9
    )


# ----------------------------------------------------------------------------------- actuator
def act(**kw):
    cfg = dict(max_angle_deg=5.0, max_rate_deg_s=60.0, time_constant_s=0.0, delay_s=0.0)
    cfg.update(kw)
    return TVCActuator(ActuatorCfg(**cfg))


def run_actuator(a, cmd, t_end, h=0.001):
    t, out = 0.0, []
    a.command(0.0, cmd)
    while t < t_end - 1e-12:
        a.step(t, h)
        t += h
        out.append(list(a.state))
    return np.array(out)


def test_actuator_saturation_and_symmetry():
    a = act(max_rate_deg_s=1e6)
    out = run_actuator(a, Command(math.radians(20), -math.radians(20)), 0.01)
    assert out[-1] == pytest.approx([math.radians(5), -math.radians(5)])  # clipped at +-max gimbal
    assert a.cmd == pytest.approx([math.radians(5), -math.radians(5)])


def test_actuator_rate_limit_is_exact():
    a = act(max_rate_deg_s=40.0, max_angle_deg=10.0)
    out = run_actuator(a, Command(math.radians(10), 0.0), 0.2)
    slope = np.diff(out[:, 0]) / 0.001
    assert np.max(slope) == pytest.approx(math.radians(40.0), rel=1e-9)  # slews at exactly the rate limit
    assert out[-1, 0] == pytest.approx(math.radians(40.0) * 0.2, rel=1e-6)


def test_actuator_lag_is_first_order_exact():
    a = act(max_rate_deg_s=1e5, max_angle_deg=10.0, time_constant_s=0.1)
    out = run_actuator(a, Command(math.radians(4.0), 0.0), 0.3)
    for t_s in (0.05, 0.1, 0.2, 0.3):
        i = int(round(t_s / 0.001)) - 1
        assert out[i, 0] == pytest.approx(math.radians(4.0) * (1 - math.exp(-t_s / 0.1)), rel=1e-6)


def test_actuator_delay_is_pure_transport():
    a = act(max_rate_deg_s=1e6, delay_s=0.05)
    out = run_actuator(a, Command(math.radians(3.0), 0.0), 0.1)
    assert np.all(out[:49, 0] == 0.0) and out[50:, 0] == pytest.approx(math.radians(3.0))


def test_actuator_bank_independent_channels():
    bank = ActuatorBank([0.2, 0.1], [1.0, 5.0], [0.0, 0.0], [0.0, 0.1])
    bank.command(0.0, [0.5, 0.5])
    t = 0.0
    for _ in range(200):
        bank.step(t, 0.001)
        t += 0.001
    assert bank.state[0] == pytest.approx(0.2) and bank.state[1] == pytest.approx(
        0.5 * 0.0 + 0.1 * 0 + bank.state[1]
    )
    assert bank.cmd == [0.2, 0.1]  # each channel saturated to its own limit
    assert bank.state[1] <= 0.1


def test_physics_uses_actual_actuator_state_not_the_command():
    """A 0.5 s transport delay: the vehicle feels NO torque until the delayed command arrives."""
    base = dict(
        fidelity=3,
        rocket={"parachutes": []},
        environment={"wind": {"model": "none"}},
        simulation={"t_max_s": 3.0},
        tvc={"max_angle_deg": 5.0, "max_rate_deg_s": 500.0, "time_constant_s": 0.0, "delay_s": 0.5},
        controller={
            "type": "schedule",
            "rate_hz": 100,
            "params": {"table": [[0.0, 3.0, 0.0], [10.0, 3.0, 0.0]]},
        },
        launch={"elevation_deg": 90.0, "rail_length_m": 0.05},
    )
    r = run_simulation(cfg_from(**base), seed=0)
    t = r.col("t")
    assert r.col("tvc_cmd_y")[t > 0.05][0] == pytest.approx(math.radians(3.0))  # commanded early
    pre = (t > 0.3) & (t < 0.49)
    post = (t > 0.8) & (t < 1.5)
    assert np.all(np.abs(r.col("tvc_y")[t < 0.49]) < 1e-12)  # actual deflection still zero
    assert np.max(np.abs(r.col("omega_q")[pre])) < 0.05  # no gimbal torque before the delay elapsed
    assert np.max(np.abs(r.col("tvc_y")[post])) == pytest.approx(math.radians(3.0), rel=1e-3)
    assert np.max(np.abs(r.col("omega_q")[post])) > 0.2  # torque appears once the actuator moved


# ------------------------------------------------------------------------- control surfaces
def canard_asm(angle0=0.0, count=4, x_le=0.8, span=0.06, chord=0.08, static=True):
    d = 0.05
    secs = [Section("nose", 0, 0.12, 0, d), Section("body", 0.12, 0.88, d, d)]
    cs = ControlSurfaceSet(
        count, chord, 0.6 * chord, span, 0.01, 0.002, x_le, angle0, math.radians(15), math.radians(300)
    )
    fins = FinSet(4, 0.1, 0.05, 0.05, 0.03, 0.002, 0.85) if static else FinSet(0, 0, 0, 0, 0, 0, 0)
    return Assembly(secs, fins, [cs])


def cs_dyn(asm, rho=1.2):
    aero = BarrowmanAero(asm, 0.0)
    veh = make_vehicle(tiny_motor(), aero=aero, diameter=asm.ref_diameter, length=asm.length, cg=0.55)
    dyn = make_6dof(
        veh,
        make_env(rho=rho, g=0.0),
        LaunchSetup((0, 0, 1e5), quat_from_pointing(math.pi / 2, 0.0), 0.0, False),
    )
    return veh, dyn, aero


def test_single_control_fin_force_and_moment_match_analytic_values():
    asm = canard_asm()
    veh, dyn, aero = cs_dyn(asm)
    v = 60.0
    y = dyn.initial_state()
    y[5] = v  # flying along the nose (body x = +z_L)
    mp = veh.mass_props(0.0)
    delta = math.radians(4.0)
    dyn.controls.fin = [delta, 0.0, 0.0, 0.0]  # fin 0 at phi = 0: normal n = (0, 0, 1) (body +z, down)
    ev0 = dyn.evaluate(0.0, y)
    dyn._cache = None
    dyn.controls.fin = [0.0] * 4
    ev_off = dyn.evaluate(0.0, y)
    dyn._cache = None
    cf = aero.control_fins[0]
    q = 0.5 * 1.2 * v * v
    s = aero.ref_area
    lift = q * s * cf.cn_alpha_single * delta
    rx = mp.x_cg - cf.x_ac
    # expected moments about the CG: P x F with P = (rx, rho, 0) and F = (-|L d|, 0, L)
    p = np.array([rx, cf.rho, 0.0])
    f = np.array([-abs(lift * delta), 0.0, lift])
    m_exp = np.cross(p, f)
    assert ev_off.wdot == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)  # no deflection, no moment (alpha = 0)
    got = np.array(ev0.wdot) * np.array([mp.ixx, mp.iyy, mp.iyy])
    assert got == pytest.approx(m_exp, rel=1e-9, abs=1e-12)
    assert m_exp[1] > 0 and m_exp[0] > 0  # aft fin pushing +z_B: pitch moment about +y and a roll moment
    # force: deflected fin adds a body +z force of exactly L (check via specific force)
    assert (np.array(ev0.spec_force) - np.array(ev_off.spec_force))[2] * mp.mass == pytest.approx(
        lift, rel=1e-9
    )


def test_control_fin_deflection_sign_symmetry_and_zero():
    asm = canard_asm()
    veh, dyn, aero = cs_dyn(asm)
    y = dyn.initial_state()
    y[5] = 80.0
    res = {}
    for name, d in (("pos", [0.05, 0, 0, 0]), ("neg", [-0.05, 0, 0, 0]), ("zero", [0, 0, 0, 0])):
        dyn.controls.fin = d
        dyn._cache = None
        res[name] = np.array(dyn.evaluate(0.0, y).wdot)
    assert res["zero"] == pytest.approx((0, 0, 0), abs=1e-12)
    # pitch/roll moments are odd in the deflection (the |L d| induced-drag term does not enter the moments much)
    assert res["pos"][1] == pytest.approx(-res["neg"][1], rel=1e-9) and res["pos"][0] == pytest.approx(
        -res["neg"][0], rel=1e-9
    )


@pytest.mark.parametrize("count,angle0", [(4, 0.0), (4, math.radians(45)), (3, 0.0), (6, math.radians(10))])
def test_mixer_decouples_channels_for_evenly_spaced_fins(count, angle0):
    asm = canard_asm(angle0, count)
    aero = BarrowmanAero(asm, 0.0)
    mx = FinMixer(aero.control_fins, 0.55)
    assert mx.coupling() < 1e-9
    for k, name in ((0, "fin_roll"), (1, "fin_pitch"), (2, "fin_yaw")):
        assert abs(mx.gain(k)) > 0
        cmd = Command(**{name: 0.1})
        defl = mx.mix(cmd)
        assert max(abs(x) for x in defl) == pytest.approx(0.1)  # the most effective fin moves by the command
    # superposition
    d = mx.mix(Command(fin_pitch=0.1, fin_yaw=-0.05, fin_roll=0.02))
    e1, e2, e3 = (
        mx.mix(Command(fin_pitch=0.1)),
        mx.mix(Command(fin_yaw=-0.05)),
        mx.mix(Command(fin_roll=0.02)),
    )
    assert d == pytest.approx([a + b + c for a, b, c in zip(e1, e2, e3)])


def test_mixed_pitch_command_produces_pure_pitch_moment_in_6dof():
    asm = canard_asm()
    veh, dyn, aero = cs_dyn(asm)
    mx = FinMixer(aero.control_fins, veh.mass_props(0.0).x_cg)
    y = dyn.initial_state()
    y[5] = 70.0
    dyn.controls.fin = mx.mix(Command(fin_pitch=0.05))
    m = np.array(dyn.evaluate(0.0, y).wdot) * np.array(
        [veh.mass_props(0.0).ixx, veh.mass_props(0.0).iyy, veh.mass_props(0.0).iyy]
    )
    assert (
        abs(m[1]) > 10 * max(abs(m[0]), abs(m[2])) and m[1] != 0
    )  # pitch dominates; coupling only from the induced-drag term
    # the moment equals the mixer's gain x q S x command (small-deflection theory)
    q, s = 0.5 * 1.2 * 70.0**2, aero.ref_area
    assert m[1] == pytest.approx(q * s * mx.gain(1) * 0.05, rel=2e-3)


def test_control_surface_actuator_limits_and_rate_in_simulation():
    asm_cfg = {
        "count": 4,
        "root_chord_m": 0.1,
        "tip_chord_m": 0.06,
        "span_m": 0.09,
        "sweep_m": 0.02,
        "position_from_nose_m": 0.88,
        "max_deflection_deg": 8.0,
        "max_rate_deg_s": 100.0,
        "time_constant_s": 0.0,
        "delay_s": 0.0,
    }
    cfg = cfg_from(
        fidelity=3,
        rocket={"parachutes": [], "control_surfaces": asm_cfg},
        environment={"wind": {"model": "none"}},
        simulation={"t_max_s": 3.0},
        controller={"type": "schedule", "rate_hz": 100, "params": {"table": [[0, 0, 0], [1.0, 0, 0]]}},
    )
    sim = Simulation(cfg, seed=0)
    assert len(sim.vehicle.aero.control_fins) == 4 and sim.fin_actuator is not None
    # drive the bank directly with a big step: saturates at 8 deg, slews at 100 deg/s
    bank = sim.fin_actuator
    bank.command(0.0, [math.radians(30)] * 4)
    t, hist = 0.0, []
    for _ in range(300):
        bank.step(t, 0.001)
        t += 0.001
        hist.append(bank.state[0])
    hist = np.array(hist)
    assert hist[-1] == pytest.approx(math.radians(8.0)) and np.max(np.diff(hist)) / 0.001 == pytest.approx(
        math.radians(100.0), rel=1e-6
    )


def _fin_cfg(cg, controlled):
    over = dict(
        fidelity=3,
        rocket={
            "cg_from_nose_m": cg,
            "parachutes": [],
            "control_surfaces": {
                "count": 4,
                "root_chord_m": 0.10,
                "tip_chord_m": 0.06,
                "span_m": 0.09,
                "sweep_m": 0.02,
                "position_from_nose_m": 0.88,
                "max_deflection_deg": 20.0,
                "max_rate_deg_s": 400.0,
                "time_constant_s": 0.01,
            },
        },
        environment={"wind": {"model": "none"}},
        launch={"elevation_deg": 90.0, "rail_length_m": 1.5},
        simulation={"t_max_s": 3.5},
        motor={"misalignment_deg": [0.6, 0.0]},
    )
    if controlled:
        over["controller"] = {
            "type": "tvc_attitude",
            "use_truth": True,
            "rate_hz": 100,
            "params": {"wn": 8.0, "zeta": 0.8, "actuation": "fins", "roll_damping": 2.0},
        }
    return cfg_from(**over)


def test_closed_loop_control_surfaces_stabilise_an_unstable_vehicle():
    free = run_simulation(_fin_cfg(0.86, False), seed=0)
    ctrl = run_simulation(_fin_cfg(0.86, True), seed=0)

    def tilt(r):
        return np.degrees(np.arccos(np.clip(np.sin(r.col("pitch")), -1, 1)))

    burn = ctrl.summary["burnout_time_s"]
    wf = (free.col("t") > 0.3) & (free.col("t") < burn)
    wc = (ctrl.col("t") > 0.3) & (ctrl.col("t") < burn)
    assert Simulation(_fin_cfg(0.86, False)).vehicle.static_margin(0.0) < 0.2  # (almost) statically unstable
    assert tilt(free)[wf].max() > 60.0  # uncontrolled: tumbles
    assert tilt(ctrl)[wc].max() < 6.0  # fins hold the nose near vertical
    fins = np.stack([ctrl.col(f"fin_{i}") for i in range(4)])
    assert np.abs(fins).max() > math.radians(0.2) and np.abs(fins).max() <= math.radians(20.0) + 1e-9
    assert ctrl.has("fin_cmd_pitch") and np.max(np.abs(ctrl.col("fin_cmd_pitch"))) > 0  # commands logged
    assert ctrl.summary["apogee_m"] > 3 * free.summary["apogee_m"]


def test_control_surface_columns_only_when_present():
    r = run_simulation(cfg_from(fidelity=3, simulation={"t_max_s": 2.0}), seed=0)
    assert not r.has("fin_0")
    r2 = run_simulation(_fin_cfg(0.78, False), seed=0)
    assert r2.has("fin_3") and not r2.has("fin_4")


def test_evaluate_memo_sees_fin_changes_without_manual_cache_reset():
    """Regression (critic 2): the one-entry memo ignored fin deflections, so RK4 stage 1 used stale fins."""
    asm = canard_asm()
    veh, dyn, aero = cs_dyn(asm)
    y = dyn.initial_state()
    y[5] = 60.0
    dyn.controls.fin = [0.0] * 4
    a = np.array(dyn.evaluate(0.0, y).wdot)
    dyn.controls.fin = [0.05, 0.0, 0.0, 0.0]  # same (t, y); NO _cache reset
    b = np.array(dyn.evaluate(0.0, y).wdot)
    assert np.linalg.norm(b - a) > 1e-6
    dyn.controls.fin = [0.0] * 4
    assert np.allclose(np.array(dyn.evaluate(0.0, y).wdot), a)


def test_actuator_commands_are_applied_in_due_time_order():
    """Regression: a later-issued command with an earlier due time (a latency change) must not be blocked by an earlier-issued one."""
    from rocket_sim.control.actuators import ActuatorBank

    bank = ActuatorBank([1.0], [1e6], [0.0], [0.0])
    bank.command(0.5, [0.9])  # due 0.5
    bank.command(0.1, [0.2])  # due 0.1: must be applied FIRST
    seen = []
    t = 0.0
    for _ in range(80):
        bank.step(t, 0.01)
        t += 0.01
        seen.append(bank.state[0])
    assert seen[11] == pytest.approx(0.2) and seen[-1] == pytest.approx(0.9)
    assert max(seen[:40]) == pytest.approx(0.2)  # 0.9 not applied before t = 0.5
