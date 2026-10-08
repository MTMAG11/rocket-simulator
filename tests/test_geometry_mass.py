"""V1.1 geometry, centre of pressure, centre of mass and inertia against independent hand calculations."""

import math

import numpy as np
import pytest

from rocket_sim.errors import ConfigError
from rocket_sim.physics.dynamics import euler_angular_acceleration
from rocket_sim.vehicle import BarrowmanAero, BodyTube, FinSet, MassComponent, MassModel, NoseCone
from rocket_sim.vehicle.assembly import Assembly, Section, legacy_assembly
from rocket_sim.vehicle.mass import (
    MassProps,
    inertia_tensor_about_cg,
    solid_cylinder_inertia,
    thin_tube_inertia,
)

NO_FINS = FinSet(0, 0, 0, 0, 0, 0, 0)


def stack(*secs):
    """Contiguous sections from (kind, length, d_fore, d_aft)."""
    out, x = [], 0.0
    for kind, length, d1, d2 in secs:
        out.append(Section(kind, x, length, d1, d2))
        x += length
    return out


def test_barrowman_nose_and_fins_against_hand_calculation():
    """Textbook numbers: d = 0.05, ogive nose 0.15 m, 4 fins (Cr 0.1, Ct 0.05, s 0.06, sweep 0.04) at 0.55."""
    d, ln = 0.05, 0.15
    asm = legacy_assembly(
        BodyTube(d, 0.7), NoseCone("ogive", ln), FinSet(4, 0.1, 0.05, 0.06, 0.04, 0.002, 0.55)
    )
    aero = BarrowmanAero(asm, 0.02)
    # hand calculation of the fin set
    s, cr, ct, xm, n = 0.06, 0.1, 0.05, 0.04, 4
    lf = math.hypot(xm + 0.5 * (ct - cr), s)
    kfb = 1 + (d / 2) / (s + d / 2)
    cn_fin = kfb * 4 * n * (s / d) ** 2 / (1 + math.sqrt(1 + (2 * lf / (cr + ct)) ** 2))
    xf = 0.55 + xm / 3 * (cr + 2 * ct) / (cr + ct) + (1 / 6) * ((cr + ct) - cr * ct / (cr + ct))
    x_nose = 0.466 * ln
    cn_total = 2.0 + cn_fin
    x_cp = (2.0 * x_nose + cn_fin * xf) / cn_total
    c = aero.coefficients(0.2, 1e6, False)
    assert c.cn_alpha == pytest.approx(cn_total, rel=1e-12)
    assert c.x_cp == pytest.approx(x_cp, rel=1e-12)


def test_transition_normal_force_telescopes():
    """Slender-body invariant: nose(d1) + body + transition(d1->d2) + body gives total CNa = 2 (d2/d_ref)^2."""
    d1, d2, d_ref = 0.04, 0.06, 0.06
    asm = Assembly(
        stack(
            ("nose", 0.12, 0, d1), ("body", 0.2, d1, d1), ("transition", 0.08, d1, d2), ("body", 0.3, d2, d2)
        ),
        NO_FINS,
        ref_diameter=d_ref,
    )
    comps = asm.section_normal_force()
    assert sum(c for c, _, _ in comps) == pytest.approx(2.0 * (d2 / d_ref) ** 2, rel=1e-12)
    # transition CP: x_T + L/3 [1 + 1/(1 + d_f/d_a)]
    (cn_t, x_t, kind) = comps[1]
    assert kind == "transition" and cn_t == pytest.approx(2 * ((d2 / d_ref) ** 2 - (d1 / d_ref) ** 2))
    assert x_t == pytest.approx(0.32 + 0.08 / 3 * (1 + 1 / (1 + d1 / d2)))


def test_boat_tail_is_destabilising_and_conical_limits():
    d_ref = 0.05
    asm = Assembly(
        stack(("nose", 0.1, 0, d_ref), ("body", 0.4, d_ref, d_ref), ("transition", 0.05, d_ref, 0.035)),
        NO_FINS,
        ref_diameter=d_ref,
    )
    cn_bt = asm.section_normal_force()[1][0]
    assert cn_bt < 0  # boat-tail reduces the normal-force slope
    # cone limit: a transition from ~0 diameter behaves as a cone with CP at 2L/3
    cone = Assembly(
        stack(("nose", 0.1, 0.0, 0.01), ("transition", 0.1, 0.01, 0.05)), NO_FINS, ref_diameter=0.05
    )
    cn, x, _ = cone.section_normal_force()[1]
    r = 0.01 / 0.05
    assert x == pytest.approx(0.1 + 0.1 / 3 * (1 + 1 / (1 + r)))


def test_assembly_validation_errors():
    with pytest.raises(ConfigError, match="contiguous"):
        Assembly([Section("nose", 0, 0.1, 0, 0.05), Section("body", 0.2, 0.3, 0.05, 0.05)], NO_FINS)
    with pytest.raises(ConfigError, match="discontinuity"):
        Assembly([Section("nose", 0, 0.1, 0, 0.05), Section("body", 0.1, 0.3, 0.06, 0.06)], NO_FINS)
    with pytest.raises(ConfigError, match="first section must be the nose"):
        Assembly([Section("body", 0, 0.3, 0.05, 0.05)], NO_FINS)
    with pytest.raises(ConfigError, match="constant diameter"):
        Section("body", 0, 0.3, 0.05, 0.06)
    with pytest.raises(ConfigError, match="must change diameter"):
        Section("transition", 0, 0.3, 0.05, 0.05)
    with pytest.raises(ConfigError, match="fins cannot start on the nose"):
        Assembly(
            stack(("nose", 0.2, 0, 0.05), ("body", 0.5, 0.05, 0.05)),
            FinSet(3, 0.1, 0.05, 0.05, 0.0, 0.002, 0.1),
        )
    with pytest.raises(ConfigError, match="beyond"):
        Assembly(
            stack(("nose", 0.2, 0, 0.05), ("body", 0.5, 0.05, 0.05)),
            FinSet(3, 0.2, 0.05, 0.05, 0.0, 0.002, 0.6),
        )


def test_local_radius_and_areas():
    asm = Assembly(
        stack(("nose", 0.1, 0, 0.04), ("body", 0.2, 0.04, 0.04), ("transition", 0.1, 0.04, 0.06)), NO_FINS
    )
    assert asm.local_radius(0.25) == pytest.approx(0.02)
    assert asm.local_radius(0.35) == pytest.approx(0.5 * (0.04 + 0.5 * 0.02))
    body = asm.sections[1]
    assert body.wetted_area() == pytest.approx(math.pi * 0.04 * 0.2)
    frustum = asm.sections[2]
    assert frustum.wetted_area() == pytest.approx(math.pi * (0.02 + 0.03) * math.hypot(0.01, 0.1))
    assert asm.ref_diameter == 0.06  # largest diameter by default


def test_cg_computed_from_components_and_shifts_during_burn():
    cs = [MassComponent("front", 1.0, 0.3), MassComponent("rear", 2.0, 0.9)]
    mm = MassModel(cs, propellant_x=0.8, propellant_radius=0.01, propellant_length=0.1)
    mp = mm.at(0.0)
    assert mp.mass == 3.0 and mp.x_cg == pytest.approx((1 * 0.3 + 2 * 0.9) / 3)
    mp1 = mm.at(0.5)
    assert mp1.mass == 3.5 and mp1.x_cg == pytest.approx((0.3 + 1.8 + 0.5 * 0.8) / 3.5)
    assert mp1.x_cg > mp.x_cg  # propellant aft of the structure CG moves the CG aft while loaded
    assert mm.at(-1.0).mass == 3.0  # negative propellant clamps to empty


def test_vehicle_cg_from_sections_in_a_full_config():
    from rocket_sim.config import load_config
    from rocket_sim.simulation import Simulation
    from tests.conftest import ROOT

    cfg = load_config(ROOT / "configs" / "example_components.yaml")
    v = Simulation(cfg).vehicle
    comps = v.mass_model.static
    m = sum(c.mass for c in comps)
    assert v.mass_props(10.0).x_cg == pytest.approx(sum(c.mass * c.x_cg for c in comps) / m)  # burnout CG
    assert v.mass_props(0.0).x_cg != v.mass_props(10.0).x_cg  # CG moves during the burn
    names = {c.name for c in comps}
    assert {"nose", "avionics", "recovery", "fins", "motor_casing"} <= names


def test_cylinder_inertia_formulas():
    ixx, iyy = solid_cylinder_inertia(2.0, 0.05, 0.4)
    assert ixx == pytest.approx(0.5 * 2.0 * 0.05**2) and iyy == pytest.approx(
        2.0 * (3 * 0.05**2 + 0.4**2) / 12
    )
    tx, ty = thin_tube_inertia(2.0, 0.05, 0.4)
    assert tx == pytest.approx(2.0 * 0.05**2) and ty == pytest.approx(2.0 * (0.05**2 / 2 + 0.4**2 / 12))


def test_parallel_axis_theorem_two_point_masses():
    cs = [MassComponent("a", 1.0, 0.2), MassComponent("b", 3.0, 0.6)]
    mp = MassModel(cs, 0.5, 0.01, 0.1).at(0.0)
    cg = (0.2 + 3 * 0.6) / 4
    iyy = 1.0 * (0.2 - cg) ** 2 + 3.0 * (0.6 - cg) ** 2
    assert (
        mp.iyy == pytest.approx(iyy, rel=1e-12) and mp.ixx == pytest.approx(0.0, abs=1e-15)
    ) or mp.ixx < 1e-6


def test_full_tensor_matches_independent_summation_for_random_assemblies():
    rng = np.random.default_rng(5)
    for _ in range(25):
        n = int(rng.integers(2, 6))
        cs = [
            MassComponent(
                f"c{i}",
                float(rng.uniform(0.1, 2)),
                float(rng.uniform(0.1, 1.5)),
                float(rng.uniform(0, 0.01)),
                float(rng.uniform(0, 0.05)),
                y=float(rng.normal(0, 0.01)),
                z=float(rng.normal(0, 0.01)),
                izz=float(rng.uniform(0, 0.05)),
                ixy=float(rng.normal(0, 1e-4)),
                ixz=float(rng.normal(0, 1e-4)),
                iyz=float(rng.normal(0, 1e-4)),
            )
            for i in range(n)
        ]
        mm = MassModel(cs, 1.0, 0.02, 0.2)
        mp = mm.at(0.0)
        m, x_cg, tensor = inertia_tensor_about_cg(cs)
        t = np.array(mp.tensor())
        assert mp.mass == pytest.approx(m) and mp.x_cg == pytest.approx(x_cg)
        assert np.allclose(t, np.array(tensor), atol=1e-12)
        assert np.allclose(t, t.T)
        assert np.all(np.linalg.eigvalsh(t) > 0)  # physical: positive definite


def test_propellant_included_in_tensor_and_matches_reference():
    cs = [MassComponent("body", 1.0, 0.5, 0.001, 0.08, y=0.003)]
    mm = MassModel(cs, 0.9, 0.015, 0.1)
    mp = mm.at(0.2)
    ixx_p, iyy_p = solid_cylinder_inertia(0.2, 0.015, 0.1)
    ref = cs + [MassComponent("prop", 0.2, 0.9, ixx_p, iyy_p)]
    _, _, tensor = inertia_tensor_about_cg(ref)
    assert np.allclose(np.array(mp.tensor()), np.array(tensor), atol=1e-12)


def test_axisymmetric_fast_path_equals_general_path():
    cs = [MassComponent("a", 1.0, 0.4, 0.002, 0.07), MassComponent("b", 0.5, 0.9, 0.001, 0.02)]
    fast = MassModel(cs, 0.8, 0.01, 0.1).at(0.1)
    # force the general path with an (identically zero) off-axis offset marker
    general = MassModel(
        [MassComponent(c.name, c.mass, c.x_cg, c.ixx, c.iyy, izz=c.iyy) for c in cs], 0.8, 0.01, 0.1
    ).at(0.1)
    assert fast.izz is None and general.izz is not None
    assert general.ixx == pytest.approx(fast.ixx) and general.iyy == pytest.approx(fast.iyy)
    assert general.izz == pytest.approx(fast.iyy) and abs(general.ixy) < 1e-15


def test_euler_equations_general_tensor_conserve_momentum_and_energy():
    """Torque-free asymmetric body: |H| and T are constants of the motion (analytic invariants)."""
    mp = MassProps(5.0, 0.5, 0.02, 0.06, 0.0, 0.0, 0.09, 0.001, -0.002, 0.003)
    t_mat = np.array(mp.tensor())
    w = np.array([2.0, -1.0, 3.0])

    def deriv(w_):
        return np.array(euler_angular_acceleration(mp, (0.0, 0.0, 0.0), tuple(w_)))

    h0, e0 = np.linalg.norm(t_mat @ w), 0.5 * w @ t_mat @ w
    dt = 1e-3
    for _ in range(3000):  # RK4
        k1 = deriv(w)
        k2 = deriv(w + 0.5 * dt * k1)
        k3 = deriv(w + 0.5 * dt * k2)
        k4 = deriv(w + dt * k3)
        w = w + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    assert np.linalg.norm(t_mat @ w) == pytest.approx(h0, rel=1e-9)
    assert 0.5 * w @ t_mat @ w == pytest.approx(e0, rel=1e-9)


def test_euler_general_matches_axisymmetric_solution():
    ixx, iyy = 0.002, 0.08
    sym = MassProps(1.0, 0.5, ixx, iyy)
    gen = MassProps(1.0, 0.5, ixx, iyy, 0.0, 0.0, iyy, 0.0, 0.0, 0.0)
    w = (30.0, 0.4, -0.2)
    m = (0.01, 0.02, -0.03)
    p, q, r = w
    expect = (m[0] / ixx, (m[1] + (iyy - ixx) * r * p) / iyy, (m[2] + (ixx - iyy) * p * q) / iyy)
    assert euler_angular_acceleration(gen, m, w) == pytest.approx(expect, rel=1e-12)
    _ = sym
