"""V1.1 aerodynamic model hierarchy: coefficient interface, Mach / alpha / Reynolds dependence, CP and moments."""

import math

import numpy as np
import pytest

from rocket_sim.errors import ConfigError
from rocket_sim.physics.dynamics import LaunchSetup
from rocket_sim.physics.math3d import angle_between, quat_from_pointing, quat_rotate, quat_rotate_inv
from rocket_sim.vehicle import (
    BarrowmanAero,
    BodyTube,
    ConstantAero,
    EnhancedAero,
    FinSet,
    NoseCone,
    SimplifiedAero,
    Table2DAero,
    TableAero,
    legacy_assembly,
)
from rocket_sim.vehicle.aero import (
    base_drag_coefficient,
    boat_tail_attached_fraction,
    fin_lift_slope_ratio,
    skin_friction_coefficient,
    stagnation_pressure_coefficient,
)
from rocket_sim.vehicle.assembly import Assembly, Section
from rocket_sim.vehicle.tables import load_table2d_csv
from tests.conftest import free_launch, make_6dof, make_env, make_vehicle, tiny_motor

FINS = FinSet(4, 0.10, 0.045, 0.07, 0.06, 0.002, 0.82)


def barrowman(**kw):
    asm = legacy_assembly(BodyTube(0.041, 1.0), NoseCone("ogive", 0.17), FINS)
    return BarrowmanAero(asm, 0.0172, **kw)


def test_wind_axis_coefficients_are_consistent_with_body_axis_forces():
    a = barrowman()
    for alpha in (0.0, 0.05, 0.2, 0.7, 1.4):
        f = a.force_coefficients(0.3, alpha, 5e6, False)
        assert a.cd(0.3, alpha, 5e6) == pytest.approx(f.ca * math.cos(alpha) + f.cn * math.sin(alpha))
        assert a.cl(0.3, alpha, 5e6) == pytest.approx(f.cn * math.cos(alpha) - f.ca * math.sin(alpha))
        # rotation of (axial, normal) -> (drag, lift) preserves the resultant coefficient magnitude
        assert math.hypot(a.cd(0.3, alpha, 5e6), a.cl(0.3, alpha, 5e6)) == pytest.approx(
            math.hypot(f.ca, f.cn)
        )


def test_small_angle_limits_cd_cl_cm():
    a = barrowman()
    c0 = a.coefficients(0.3, 5e6, False)
    assert a.cd(0.3, 0.0, 5e6) == pytest.approx(c0.cd0) and a.cl(0.3, 0.0, 5e6) == 0.0
    al = 1e-3
    assert a.cl(0.3, al, 5e6) == pytest.approx(
        (c0.cn_alpha - c0.cd0) * al, rel=2e-3
    )  # CL = CN cos a - CA sin a
    # restoring moment about a CG ahead of the CP: Cm = -CNa alpha (x_cp - x_cg)/d
    x_cg = 0.6
    assert a.cm(0.3, al, 5e6, x_cg) == pytest.approx(
        -c0.cn_alpha * al * (c0.x_cp - x_cg) / a.ref_diameter, rel=3e-3
    )
    assert a.cm(0.3, al, 5e6, x_cg) < 0  # CP aft of the reference => restoring (stable)
    assert a.cm(0.3, al, 5e6, 0.95) > 0  # reference aft of the CP => destabilising


def test_drag_rises_with_alpha_through_projection_and_crossflow():
    a = barrowman()
    cds = [a.cd(0.3, math.radians(x), 5e6) for x in (0, 5, 10, 20, 40, 90)]
    assert all(b > c for b, c in zip(cds[1:], cds[:-1])) is False or cds[-1] > cds[0]
    assert cds[0] < cds[2] < cds[4]  # monotone through the practical range
    # at 90 degrees the force is crossflow drag + a small |cos| term: CD ~ Cd_cf A_planform / S
    cd90 = a.cd(0.3, math.pi / 2, 5e6)
    assert cd90 == pytest.approx(a.crossflow_cd * a.planform_area / a.ref_area, rel=1e-6)
    # CL peaks and falls: it is zero again at 90 degrees
    assert abs(a.cl(0.3, math.pi / 2, 5e6)) < 1e-9


def test_normal_force_is_symmetric_about_90_degrees_and_cp_moves_with_crossflow():
    a = barrowman()
    for al in (0.2, 0.9):
        f1 = a.force_coefficients(0.3, al, 5e6, False)
        f2 = a.force_coefficients(0.3, math.pi - al, 5e6, False)
        assert f1.cn == pytest.approx(f2.cn)  # reversed flow: same lateral force magnitude (|cos| law)
    small = a.force_coefficients(0.3, 0.02, 5e6, False)
    big = a.force_coefficients(0.3, 1.2, 5e6, False)
    assert small.x_cp_static == big.x_cp_static  # static CP is the linear (small-angle) one
    assert small.x_cp_force == pytest.approx(
        small.x_cp_static, rel=0.05
    )  # linear term dominates at small alpha
    assert big.x_cp_force != big.x_cp_static  # crossflow pulls the force CP toward the body centroid


def test_skin_friction_reynolds_and_roughness_trends():
    cf = lambda re, m=0.0, k=0.0: skin_friction_coefficient(re, m, k, 1.0)  # noqa: E731
    assert cf(1e5) == pytest.approx(1.328 / math.sqrt(1e5))  # laminar Blasius
    assert cf(1e7) == pytest.approx(0.455 / math.log10(1e7) ** 2.58)  # turbulent Schlichting
    assert cf(1e7) > cf(1e8)  # decreases with Re (smooth)
    assert cf(1e9, k=60e-6) == cf(1e8, k=60e-6)  # roughness cut-off caps the benefit of Re
    assert cf(1e7, m=1.5) < cf(1e7, m=0.0)  # compressibility reduces Cf
    # body friction (smooth wall, finless so no laminar->turbulent fin switch): drag falls as Re rises
    asm = Assembly(
        [Section("nose", 0, 0.15, 0, 0.05), Section("body", 0.15, 0.85, 0.05, 0.05)],
        FinSet(0, 0, 0, 0, 0, 0, 0),
    )
    smooth = BarrowmanAero(asm, 0.0, surface_roughness=0.0)
    assert (
        smooth.coefficients(0.3, 1e6, False).cd0
        > smooth.coefficients(0.3, 1e7, False).cd0
        > smooth.coefficients(0.3, 1e8, False).cd0
    )
    rough = BarrowmanAero(asm, 0.0, surface_roughness=60e-6)
    assert (
        rough.coefficients(0.3, 1e8, False).cd0 == rough.coefficients(0.3, 1e9, False).cd0
    )  # roughness-limited


def test_reynolds_number_definition_in_dynamics():
    """Re = rho V L / mu with L the total airframe length (what the dynamics passes to the aero model)."""
    seen = {}

    class Probe(BarrowmanAero):
        def force_coefficients(self, mach, alpha, reynolds, powered):
            seen["re"], seen["mach"] = reynolds, mach
            return super().force_coefficients(mach, alpha, reynolds, powered)

    asm = legacy_assembly(BodyTube(0.041, 1.0), NoseCone("ogive", 0.17), FINS)
    veh = make_vehicle(tiny_motor(), aero=Probe(asm, 0.0172), diameter=0.041, length=1.0)
    env = make_env(rho=None)
    dyn = make_6dof(veh, env, free_launch(z0=300.0))
    y = dyn.initial_state()
    y[5] = 120.0
    ev = dyn.evaluate(0.0, y)
    atm = env.atmosphere.at(300.0)
    assert seen["re"] == pytest.approx(atm.density * 120.0 * 1.0 / atm.viscosity, rel=1e-9)
    assert seen["mach"] == pytest.approx(120.0 / atm.speed_of_sound, rel=1e-9) and ev.mach == pytest.approx(
        seen["mach"]
    )


def test_mach_dependence_of_drag_components():
    # the two branches of the Niskanen fit meet to < 0.6 % at M = 1 (1.275 vs 1.281)
    assert stagnation_pressure_coefficient(0.999999) == pytest.approx(
        stagnation_pressure_coefficient(1.000001), abs=7e-3
    )
    # base drag: Hoerner subsonic (Mach independent), blended to Barrowman 0.25/M supersonic
    assert base_drag_coefficient(0.3, 0.35) == base_drag_coefficient(0.6, 0.35)
    assert base_drag_coefficient(0.9, 0.35) > base_drag_coefficient(0.6, 0.35)
    assert base_drag_coefficient(2.0) == pytest.approx(0.125) and base_drag_coefficient(1.0) == pytest.approx(
        0.25
    )
    a = barrowman()
    cd = [a.coefficients(m, 5e6, False).cd0 for m in (0.2, 0.6, 0.85, 1.05, 1.5, 2.5)]
    assert cd[3] > cd[1] and cd[0] > 0  # drag rise through the transonic range


def test_enhanced_fin_lift_slope_follows_linear_theory():
    fins = FinSet(4, 0.10, 0.045, 0.07, 0.06, 0.002, 0.82)
    ar = 2 * fins.span**2 / fins.planform_area
    tan_l = math.tan(math.atan2(fins.sweep + 0.5 * (fins.tip_chord - fins.root_chord), fins.span))

    def helmbold(m):
        return 2 * math.pi * ar / (2 + math.sqrt(4 + ar**2 * (1 - m * m) * (1 + tan_l**2 / (1 - m * m))))

    assert fin_lift_slope_ratio(0.0, fins) == pytest.approx(1.0)
    assert fin_lift_slope_ratio(0.6, fins) == pytest.approx(helmbold(0.6) / helmbold(0.0), rel=1e-12)
    assert fin_lift_slope_ratio(0.6, fins) > 1.0  # Prandtl-Glauert: compressibility raises the slope
    beta = math.sqrt(2.0**2 - 1)
    ackeret = (4 / beta) * (1 - 1 / (2 * ar * beta))
    assert fin_lift_slope_ratio(2.0, fins) == pytest.approx(ackeret / helmbold(0.0), rel=1e-12)
    assert fin_lift_slope_ratio(3.0, fins) < fin_lift_slope_ratio(1.5, fins)  # supersonic: falls like 1/beta
    # continuous through the transonic blend
    for m in (0.8, 1.2):
        assert fin_lift_slope_ratio(m - 1e-9, fins) == pytest.approx(
            fin_lift_slope_ratio(m + 1e-9, fins), rel=1e-6
        )


def test_enhanced_model_cp_shifts_aft_supersonic_and_matches_barrowman_at_low_mach():
    asm = legacy_assembly(BodyTube(0.041, 1.0), NoseCone("ogive", 0.17), FINS)
    base, enh = BarrowmanAero(asm, 0.0172), EnhancedAero(asm, 0.0172)
    c0b, c0e = base.coefficients(0.05, 5e6, False), enh.coefficients(0.05, 5e6, False)
    assert c0e.cn_alpha == pytest.approx(c0b.cn_alpha, rel=2e-3) and c0e.x_cp == pytest.approx(
        c0b.x_cp, abs=1e-3
    )
    assert enh.coefficients(2.0, 5e6, False).x_cp != c0b.x_cp  # supersonic fin CP at the planform centroid
    assert (
        enh.coefficients(2.0, 5e6, False).cd0 > base.coefficients(2.0, 5e6, False).cd0
    )  # fin wave drag added


def test_enhanced_fin_stall_saturates_normal_force():
    asm = legacy_assembly(BodyTube(0.041, 1.0), NoseCone("ogive", 0.17), FINS)
    enh = EnhancedAero(asm, 0.0172, stall_angle_deg=15.0)
    base = BarrowmanAero(asm, 0.0172)
    small = (
        enh.force_coefficients(0.3, 0.02, 5e6, False).cn / base.force_coefficients(0.3, 0.02, 5e6, False).cn
    )
    large = enh.force_coefficients(0.3, 0.5, 5e6, False).cn / base.force_coefficients(0.3, 0.5, 5e6, False).cn
    assert small == pytest.approx(1.0, rel=0.02) and large < 0.9  # stalled fins carry less load at 29 degrees
    with pytest.raises(ConfigError):
        EnhancedAero(asm, 0.0172, stall_angle_deg=1.0)


def test_simplified_model_constant_cd_but_geometry_stability():
    asm = legacy_assembly(BodyTube(0.041, 1.0), NoseCone("ogive", 0.17), FINS)
    s = SimplifiedAero(asm, 0.45, 0.0172)
    b = BarrowmanAero(asm, 0.0172)
    assert s.coefficients(0.1, 1e6, False).cd0 == s.coefficients(1.5, 1e8, True).cd0 == 0.45
    assert s.coefficients(0.1, 1e6, False).x_cp == b.coefficients(0.1, 1e6, False).x_cp
    assert SimplifiedAero(asm, 0.45, 0.0172, drag_scale=1.2).coefficients(
        0.1, 1e6, False
    ).cd0 == pytest.approx(0.54)


def _vehicle_asm(secs):
    out, x = [], 0.0
    for k, ln, d1, d2 in secs:
        out.append(Section(k, x, ln, d1, d2))
        x += ln
    return Assembly(out, FinSet(0, 0, 0, 0, 0, 0, 0), ref_diameter=0.06)


def test_boat_tail_reduces_base_drag_only_when_flow_stays_attached():
    d = 0.06
    plain = BarrowmanAero(_vehicle_asm([("nose", 0.15, 0, d), ("body", 0.6, d, d)]), 0.0, extra_cd=0)
    gentle = BarrowmanAero(
        _vehicle_asm([("nose", 0.15, 0, d), ("body", 0.55, d, d), ("transition", 0.15, d, 0.045)]), 0.0
    )
    steep = BarrowmanAero(
        _vehicle_asm([("nose", 0.15, 0, d), ("body", 0.59, d, d), ("transition", 0.02, d, 0.03)]), 0.0
    )
    assert math.degrees(abs(math.atan2(0.0075, 0.15))) < 8  # 2.9 deg half angle: attached
    assert boat_tail_attached_fraction(math.radians(3.0)) == 1.0
    assert boat_tail_attached_fraction(math.radians(20.0)) == 0.0
    assert 0 < boat_tail_attached_fraction(math.radians(11.0)) < 1
    # base area ratio: attached boat-tail shrinks the base, separated one does not
    assert gentle._a_base == pytest.approx((0.045 / d) ** 2)
    assert steep._a_base == pytest.approx(1.0)
    assert plain._a_base == 1.0


def test_expansion_step_adds_pressure_drag():
    d = 0.04
    straight = BarrowmanAero(_vehicle_asm([("nose", 0.12, 0, d), ("body", 0.6, d, d)]), 0.0)
    shoulder = BarrowmanAero(
        _vehicle_asm(
            [
                ("nose", 0.12, 0, d),
                ("body", 0.3, d, d),
                ("transition", 0.02, d, 0.06),
                ("body", 0.28, 0.06, 0.06),
            ]
        ),
        0.0,
    )
    assert shoulder.coefficients(0.3, 5e6, False).cd0 > straight.coefficients(0.3, 5e6, False).cd0


@pytest.mark.parametrize(
    "el,az,roll,v,wind",
    [
        (90, 0, 0, (0, 0, 80), (0, 0, 0)),
        (60, 40, 20, (30, 10, 60), (0, 0, 0)),
        (45, 200, -30, (-20, 25, 40), (5, -3, 0)),
        (80, 90, 0, (0, 0, -50), (0, 8, 0)),
        (30, 300, 70, (10, 10, -10), (-6, 6, 0)),
    ],
)
def test_alpha_beta_geometry_independent_of_the_simulator(el, az, roll, v, wind):
    from rocket_sim.environment import ConstantWind

    q = quat_from_pointing(math.radians(el), math.radians(az), math.radians(roll))
    veh = make_vehicle(tiny_motor(), aero=ConstantAero(0.05, 0.2, 5.0, 0.8), diameter=0.05)
    env = make_env(rho=1.2, g=0.0)
    speed = math.hypot(wind[0], wind[1])
    if speed > 0:  # blows FROM bearing b toward the opposite direction
        b = math.degrees(math.atan2(-wind[0], -wind[1]))
        env.wind = ConstantWind(speed, b)
    dyn = make_6dof(veh, env, LaunchSetup((0, 0, 200.0), q, 0.0, on_rail=False))
    y = dyn.initial_state()
    y[3:6] = v
    ev = dyn.evaluate(0.0, y)
    vrel = np.array(v, float) - np.array(env.wind.at(0.0, 200.0))
    nose = np.array(quat_rotate(q, (1, 0, 0)))
    # independent definition: alpha = angle between the nose axis and the relative velocity vector
    assert ev.alpha == pytest.approx(angle_between(tuple(nose), tuple(vrel)), abs=1e-12)
    vb = np.array(quat_rotate_inv(q, tuple(vrel)))
    assert ev.beta == pytest.approx(math.asin(vb[1] / np.linalg.norm(vrel)), abs=1e-12)
    assert ev.airspeed == pytest.approx(np.linalg.norm(vrel), rel=1e-12)  # wind subtracted, not added


def test_aerodynamic_force_acts_along_relative_wind_at_zero_alpha():
    veh = make_vehicle(tiny_motor(), aero=ConstantAero(0.05, 0.4, 5.0, 0.8), diameter=0.05)
    dyn = make_6dof(veh, make_env(rho=1.2, g=0.0), free_launch(z0=200.0))
    y = dyn.initial_state()
    y[5] = 100.0  # moving along the nose
    ev = dyn.evaluate(0.0, y)
    s = math.pi * 0.05**2 / 4
    m = veh.mass_props(0.0).mass
    assert ev.a[2] == pytest.approx(
        -0.5 * 1.2 * 100.0**2 * 0.4 * s / m, rel=1e-6
    ) and ev.lift == pytest.approx(0.0, abs=1e-12)
    assert abs(ev.a[0]) < 1e-12 and abs(ev.a[1]) < 1e-12


def test_restoring_moment_direction_and_magnitude_in_6dof():
    """Nose tilted by alpha from the velocity with CP aft of CG: moment opposes alpha, M = -q S CNa alpha (xcp - xcg)."""
    d, cna, xcp, cg = 0.05, 12.0, 0.8, 0.5
    veh = make_vehicle(tiny_motor(), aero=ConstantAero(d, 0.0, cna, xcp), cg=cg, diameter=d)
    dyn = make_6dof(veh, make_env(rho=1.2, g=0.0), free_launch(z0=1e5, elevation=0.0))
    alpha = math.radians(3.0)
    # nose heading north (level); velocity rotated by alpha toward body +z (down) => lateral velocity w > 0
    q = quat_from_pointing(0.0, 0.0)
    v_dir = quat_rotate(q, (math.cos(alpha), 0.0, math.sin(alpha)))
    y = dyn.initial_state()
    y[3:6] = [100.0 * c for c in v_dir]
    y[6:10] = q
    ev = dyn.evaluate(0.0, y)
    mp = veh.mass_props(0.0)
    qd, s = 0.5 * 1.2 * 100**2, math.pi * d * d / 4
    expected_my = (
        qd * s * cna * math.sin(alpha) * math.cos(alpha) * (xcp - mp.x_cg)
    )  # = -rx * Fz with Fz = -N lz...
    assert ev.wdot[1] * mp.iyy == pytest.approx(
        -expected_my, rel=1e-9
    )  # exactly -q S CNa sin cos (xcp - xcg)
    # restoring: rotation about +y_B moves the nose toward -z_B; here the velocity is toward +z_B (below the
    # nose), so a restoring moment must be NEGATIVE about y, rotating the nose toward the velocity
    assert ev.wdot[1] < 0
    nose_rate_dir = np.cross([0, ev.wdot[1], 0], [1, 0, 0])  # d(nose)/dt for that angular acceleration
    assert nose_rate_dir[2] > 0 and abs(ev.wdot[2]) < 1e-9 and abs(ev.wdot[0]) < 1e-9


def test_table_aero_interpolates_and_keeps_stability_data():
    t = TableAero(0.05, [0.0, 0.5, 1.0], [0.4, 0.5, 0.9], 8.0, 0.7, [0.3, 0.4, 0.8])
    assert t.coefficients(0.25, 1e6, False).cd0 == pytest.approx(0.45)
    assert t.coefficients(0.25, 1e6, True).cd0 == pytest.approx(0.35)  # powered table
    assert t.coefficients(5.0, 1e6, False).cd0 == pytest.approx(0.9)  # clamped
    assert t.coefficients(0.1, 1e6, False).cn_alpha == 8.0
    with pytest.raises(ConfigError):
        TableAero(0.05, [0.0, 0.0], [0.4, 0.5])


def test_table2d_bilinear_exactness_and_conversion(tmp_path):
    machs, alphas = [0.0, 0.5, 1.0], [0.0, 0.1, 0.3, 0.6]
    f = lambda m, a: 0.3 + 0.2 * m + 1.5 * a  # noqa: E731 - bilinear in (m, a)
    g = lambda m, a: (4.0 + m) * a  # noqa: E731
    cd = [[f(m, a) for a in alphas] for m in machs]
    cl = [[g(m, a) for a in alphas] for m in machs]
    t = Table2DAero(0.05, machs, alphas, cd, cl, x_cp=0.7)
    for m, a in ((0.25, 0.05), (0.8, 0.2), (0.1, 0.45)):
        assert t._bilinear(t.cd_t, m, a) == pytest.approx(f(m, a), rel=1e-12)
        assert t._bilinear(t.cl_t, m, a) == pytest.approx(g(m, a), rel=1e-12)
    a = 0.1
    fc = t.force_coefficients(0.5, a, 1e6, False)
    cd_, cl_ = f(0.5, a), g(0.5, a)
    assert fc.cn == pytest.approx(cl_ * math.cos(a) + cd_ * math.sin(a))
    assert fc.ca == pytest.approx(cd_ * math.cos(a) - cl_ * math.sin(a))
    assert t.cd(0.5, a, 1e6) == pytest.approx(cd_) and t.cl(0.5, a, 1e6) == pytest.approx(cl_)
    # CM column sets the CP: x_cp = x_ref + Cm d / CN  (Cm positive nose-up)
    cm = [[0.02 * (a_ + 0.1) for a_ in alphas] for _ in machs]
    t2 = Table2DAero(0.05, machs, alphas, cd, cl, cm, x_cm_ref=0.5)
    fc2 = t2.force_coefficients(0.5, 0.3, 1e6, False)
    assert fc2.x_cp_force == pytest.approx(0.5 + (0.02 * 0.4) / fc2.cn * 0.05)
    p = tmp_path / "t.csv"
    rows = ["mach,alpha_deg,cd,cl"] + [
        f"{m},{math.degrees(a)},{cd[i][j]},{cl[i][j]}"
        for i, m in enumerate(machs)
        for j, a in enumerate(alphas)
    ]
    p.write_text("\n".join(rows))
    loaded = load_table2d_csv(p)
    assert loaded["mach"] == machs and np.allclose(loaded["alpha"], alphas) and loaded["cm"] is None
    p.write_text("\n".join(rows[:-1]))
    with pytest.raises(ConfigError, match="complete"):
        load_table2d_csv(p)
    with pytest.raises(ConfigError):
        Table2DAero(0.05, machs, [0.1, 0.2], [[0, 0]] * 3, [[0, 0]] * 3)


def test_table2d_through_a_full_simulation(tmp_path):
    """A table that reproduces a constant-Cd vehicle gives the same flight as the constant model."""
    from rocket_sim.simulation import run_simulation
    from tests.conftest import cfg_from

    machs = [0.0, 0.5, 1.0, 3.0]
    alphas = [0.0, 0.05, 0.1, 0.2, 0.4, 0.8, 1.6, 3.14159265]
    cd0 = 0.5
    cna = 20.0
    rows = ["mach,alpha_deg,cd,cl"]
    for m in machs:
        for a in alphas:
            cn = cna * math.sin(a) * abs(math.cos(a))
            ca = cd0 * math.cos(a)
            cd = ca * math.cos(a) + cn * math.sin(a)
            cl = cn * math.cos(a) - ca * math.sin(a)
            rows.append(f"{m},{math.degrees(a)},{cd},{cl}")
    p = tmp_path / "const.csv"
    p.write_text("\n".join(rows))
    tab = run_simulation(
        cfg_from(fidelity=3, rocket={"parachutes": [], "aero": {"model": "table2d", "table2d_file": str(p)}}),
        seed=0,
    )
    const = run_simulation(
        cfg_from(
            fidelity=3, rocket={"parachutes": [], "aero": {"model": "constant", "cd": cd0, "cn_alpha": cna}}
        ),
        seed=0,
    )
    assert tab.summary["apogee_m"] == pytest.approx(const.summary["apogee_m"], rel=2e-3)
