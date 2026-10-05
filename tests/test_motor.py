import numpy as np
import pytest

from rocket_sim.constants import G0
from rocket_sim.errors import MotorError
from rocket_sim.motor import Motor, available_motors, load_motor, parse_eng
from tests.conftest import ROOT

MOTORS = sorted((ROOT / "data" / "motors").glob("*.eng"))

EXAMPLE = """; comment
TEST 29 100 0 0.020 0.050 ACME
0.1 10
0.5 20
1.0 0
"""


def test_all_bundled_motors_load_and_are_plausible():
    assert len(MOTORS) >= 4
    for p in MOTORS:
        m = load_motor(p)
        assert m.total_impulse > 0 and m.burn_time > 0
        assert m.times[0] == 0.0 and m.thrust[0] == 0.0  # (0, 0) point prepended
        assert not m.sanity_warnings(), (p.name, m.sanity_warnings())
        # published total impulses (ThrustCurve.org) within a few percent
    g80 = load_motor(ROOT / "data/motors/AeroTech_G80T.eng")
    assert g80.total_impulse == pytest.approx(120.0, rel=0.15)
    assert g80.impulse_class == "G"
    assert load_motor(ROOT / "data/motors/Estes_E16.eng").impulse_class == "E"


def test_parse_header_units_and_masses():
    (m,) = parse_eng(EXAMPLE)
    assert m.diameter == pytest.approx(0.029) and m.length == pytest.approx(0.100)
    assert m.propellant_mass == pytest.approx(0.020) and m.total_mass == pytest.approx(0.050)
    assert m.casing_mass == pytest.approx(0.030)
    assert m.times == [0.0, 0.1, 0.5, 1.0] and m.thrust == [0.0, 10, 20, 0]


def test_impulse_matches_numeric_integration():
    (m,) = parse_eng(EXAMPLE)
    ts = np.linspace(0, m.burn_time, 200001)
    th = np.array([m.thrust_at(t) for t in ts[::100]])
    analytic = 0.5 * 0.1 * 10 + 0.5 * (10 + 20) * 0.4 + 0.5 * 20 * 0.5  # trapezoids
    assert m.total_impulse == pytest.approx(analytic)
    assert np.trapezoid(th, ts[::100]) == pytest.approx(analytic, rel=1e-4)
    # closed-form partial impulse vs numeric at an interior time
    t = 0.37
    tt = np.linspace(0, t, 100001)
    num = np.trapezoid([m.thrust_at(x) for x in tt], tt)
    assert m.impulse_at(t) == pytest.approx(num, rel=1e-6)


def test_mass_depletion_proportional_to_impulse():
    (m,) = parse_eng(EXAMPLE)
    assert m.propellant_at(-1.0) == pytest.approx(0.020)
    assert m.propellant_at(0.0) == pytest.approx(0.020)
    assert m.propellant_at(m.burn_time) == 0.0
    assert m.propellant_at(m.burn_time + 5) == 0.0
    ts = np.linspace(0, m.burn_time, 50)
    mp = [m.propellant_at(t) for t in ts]
    assert all(b <= a + 1e-15 for a, b in zip(mp, mp[1:]))  # monotone
    t = 0.3
    assert m.propellant_at(t) == pytest.approx(0.020 * (1 - m.impulse_at(t) / m.total_impulse))
    # mass-flow integrates to total propellant mass
    tt = np.linspace(0, m.burn_time, 20001)
    assert np.trapezoid([m.mass_flow_at(x) for x in tt], tt) == pytest.approx(0.020, rel=1e-3)


def test_thrust_interpolation_and_bounds():
    (m,) = parse_eng(EXAMPLE)
    assert m.thrust_at(0.3) == pytest.approx(15.0)
    assert m.thrust_at(0.05) == pytest.approx(5.0)  # ramp from the prepended (0,0) point
    assert (
        m.thrust_at(-0.1) == 0.0 and m.thrust_at(1.0) == 0.0 and m.thrust_at(9.0) == 0.0
    )  # curve ends at 0 N


def test_isp_is_reported():
    (m,) = parse_eng(EXAMPLE)
    assert m.specific_impulse == pytest.approx(m.total_impulse / (0.020 * G0))


def test_scaled_motor():
    (m,) = parse_eng(EXAMPLE)
    s = m.scaled(1.1, 0.9)
    assert s.total_impulse == pytest.approx(m.total_impulse * 1.1 * 0.9)
    assert s.burn_time == pytest.approx(m.burn_time * 0.9)
    assert s.propellant_mass == m.propellant_mass


@pytest.mark.parametrize(
    "text,msg",
    [
        ("", "no motor"),
        ("TEST 29 100 0 0.02\n0.1 5\n", "unrecognised"),  # short header
        ("0.1 5\n", "before motor header"),
        ("TEST 29 100 0 abc 0.05 X\n0.1 5\n", "unrecognised|malformed"),
        ("TEST 29 100 0 0.02 0.05 X\n", "no thrust data"),
        ("TEST 29 100 0 0.02 0.05 X\n0.5 5\n0.2 5\n", "strictly increasing"),
        ("TEST 29 100 0 0.05 0.02 X\n0.1 5\n0.2 0\n", "total mass"),
        ("TEST 29 100 0 0.02 0.05 X\n0.1 -5\n0.2 0\n", "thrust must be"),
    ],
)
def test_malformed_eng_files_raise(text, msg):
    with pytest.raises(MotorError, match=msg):
        parse_eng(text)


def test_missing_file_and_unknown_format(tmp_path):
    with pytest.raises(MotorError, match="not found"):
        load_motor(tmp_path / "nope.eng")
    p = tmp_path / "x.foo"
    p.write_text("x")
    with pytest.raises(MotorError, match="no motor loader"):
        load_motor(p)


def test_multi_motor_file_and_designation(tmp_path):
    p = tmp_path / "multi.eng"
    p.write_text(EXAMPLE + "SECOND 29 100 0 0.020 0.050 ACME\n0.2 30\n0.4 0\n")
    assert load_motor(p).designation == "TEST"
    assert load_motor(p, "second").designation == "SECOND"
    with pytest.raises(MotorError, match="not found"):
        load_motor(p, "nope")


def test_csv_loader(tmp_path):
    p = tmp_path / "m.csv"
    p.write_text("time,thrust\n0.1,10\n0.5,20\n1.0,0\n")
    m = load_motor(p, propellant_mass=0.02, total_mass=0.05, diameter=0.029, length=0.1)
    assert m.total_impulse == pytest.approx(0.5 * 0.1 * 10 + 0.5 * 30 * 0.4 + 0.5 * 20 * 0.5)
    with pytest.raises(MotorError, match="CSV motors need"):
        load_motor(p)


def test_available_motors():
    assert len(available_motors(ROOT / "data/motors")) >= 4


def test_motor_validation_direct():
    with pytest.raises(MotorError):
        Motor("x", 0.03, 0.1, 0.0, 0.05, [0.0, 1.0], [1.0, 1.0])
    with pytest.raises(MotorError):
        Motor("x", 0.03, 0.1, 0.01, 0.05, [0.1, 1.0], [1.0, 1.0])
