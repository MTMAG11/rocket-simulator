import math

import numpy as np
import pytest

from rocket_sim.constants import GM_EARTH, R_EARTH
from rocket_sim.environment import (
    CompositeWind,
    ConstantGravity,
    ConstantWind,
    ExponentialAtmosphere,
    GridTerrain,
    GustWind,
    InverseSquareGravity,
    ISAAtmosphere,
    PowerLawWind,
    ProfileWind,
    SlopeTerrain,
    TableAtmosphere,
    TurbulenceWind,
)
from rocket_sim.errors import ConfigError

R0 = 6_356_766.0


def geometric(hg):  # geopotential -> geometric height [m]
    return R0 * hg / (R0 - hg)


# Reference values: U.S. Standard Atmosphere 1976 tables (NASA-TM-X-74335) at the layer
# boundaries, indexed by GEOPOTENTIAL height [km] -> (T [K], p [Pa]).
ISA_TABLE = [
    (0.0, 288.15, 101325.0),
    (11.0, 216.65, 22632.06),
    (20.0, 216.65, 5474.889),
    (32.0, 228.65, 868.0187),
    (47.0, 270.65, 110.9063),
    (51.0, 270.65, 66.93887),
    (71.0, 214.65, 3.956420),
]


@pytest.mark.parametrize("hg_km,t_ref,p_ref", ISA_TABLE)
def test_isa_matches_published_table(hg_km, t_ref, p_ref):
    s = ISAAtmosphere().at(geometric(hg_km * 1000.0))
    assert s.temperature == pytest.approx(t_ref, abs=1e-6)
    assert s.pressure == pytest.approx(p_ref, rel=2e-5)
    assert s.density == pytest.approx(p_ref / (287.05287 * t_ref), rel=2e-5)


def test_isa_sea_level_derived_quantities():
    s = ISAAtmosphere().at(0.0)
    assert s.density == pytest.approx(1.225, rel=1e-3)
    assert s.speed_of_sound == pytest.approx(340.294, rel=1e-4)
    assert s.viscosity == pytest.approx(1.7894e-5, rel=1e-3)


def test_isa_offsets_and_pressure_inverse():
    hot = ISAAtmosphere(temperature_offset=15.0, sea_level_pressure=100000.0)
    s = hot.at(0.0)
    assert s.temperature == pytest.approx(303.15) and s.pressure == pytest.approx(100000.0)
    isa = ISAAtmosphere()
    for h in (-100.0, 0.0, 500.0, 3000.0, 11500.0, 25000.0, 40000.0, 60000.0):
        assert isa.pressure_to_altitude(isa.at(h).pressure) == pytest.approx(h, abs=1e-3)


def test_isa_monotonic_pressure_density_continuity():
    isa = ISAAtmosphere()
    hs = np.linspace(0, 80000, 801)
    p = [isa.at(h).pressure for h in hs]
    assert all(b < a for a, b in zip(p, p[1:]))
    t = [isa.at(h).temperature for h in hs]
    assert (
        max(abs(b - a) for a, b in zip(t, t[1:])) < 0.7
    )  # steepest lapse is 6.5 K/km = 0.65 K per 100 m; no jumps at layer edges


def test_exponential_and_table_atmospheres():
    e = ExponentialAtmosphere().at(8400.0)
    assert e.density == pytest.approx(1.225 / math.e, rel=1e-9)
    isa = ISAAtmosphere()
    rows = [isa.at(h) for h in range(0, 20001, 1000)]
    tab = TableAtmosphere(
        list(range(0, 20001, 1000)), [r.temperature for r in rows], [r.pressure for r in rows]
    )
    for h in (0, 1000, 2500, 7777, 19999):
        assert tab.at(h).pressure == pytest.approx(isa.at(h).pressure, rel=2e-3)
    with pytest.raises(ValueError, match="outside"):
        tab.at(25000)
    with pytest.raises(ConfigError):
        TableAtmosphere([0, 0], [288, 288], [1e5, 1e5])


def test_gravity_models():
    assert ConstantGravity(9.81).g(5000.0) == 9.81
    ig = InverseSquareGravity()
    assert ig.g(0.0) == pytest.approx(GM_EARTH / R_EARTH**2)
    assert ig.g(0.0) == pytest.approx(9.82, abs=0.01)
    # g(h) = GM/(R+h)^2: at one earth radius height, g/4
    assert ig.g(R_EARTH) == pytest.approx(ig.g(0.0) / 4.0)
    # decreases ~3.1e-6 m/s^2 per metre near the surface
    assert ig.g(0.0) - ig.g(1000.0) == pytest.approx(0.003086, rel=2e-2)


def test_wind_direction_convention():
    # wind FROM the west (270 deg) blows toward +East
    w = ConstantWind(5.0, 270.0).at(0.0, 0.0)
    assert w == pytest.approx((5.0, 0.0, 0.0), abs=1e-12)
    w = ConstantWind(5.0, 0.0).at(0.0, 0.0)  # from the north -> blows toward -y
    assert w == pytest.approx((0.0, -5.0, 0.0), abs=1e-12)


def test_profile_and_power_law_wind():
    p = ProfileWind([(0, 2.0, 270), (100, 6.0, 270), (300, 10.0, 270)])
    assert p.at(0, 50)[0] == pytest.approx(4.0)
    assert p.at(0, -5)[0] == pytest.approx(2.0)  # held below first row
    assert p.at(0, 1000)[0] == pytest.approx(10.0)  # held above last row
    pl = PowerLawWind(5.0, 270.0, ref_height=10.0, exponent=1 / 7)
    assert pl.at(0, 10.0)[0] == pytest.approx(5.0)
    assert pl.at(0, 80.0)[0] == pytest.approx(5.0 * 8 ** (1 / 7))
    with pytest.raises(ConfigError):
        ProfileWind([(0, 1, 0), (0, 2, 0)])


def test_turbulence_is_deterministic_and_has_requested_statistics():
    a = TurbulenceWind(2.0, 3.0, seed=11, horizon=4000.0)
    b = TurbulenceWind(2.0, 3.0, seed=11, horizon=4000.0)
    c = TurbulenceWind(2.0, 3.0, seed=12, horizon=4000.0)
    ts = np.arange(0, 3990, 0.37)
    xa = np.array([a.at(t, 0)[0] for t in ts])
    assert np.array_equal(xa, [b.at(t, 0)[0] for t in ts])
    assert not np.array_equal(xa, [c.at(t, 0)[0] for t in ts])
    assert xa.std() == pytest.approx(2.0, rel=0.1)
    # time correlation: lag-1 autocorrelation at dt=0.37, tau=3 -> exp(-0.37/3)
    assert np.corrcoef(xa[:-1], xa[1:])[0, 1] == pytest.approx(math.exp(-0.37 / 3.0), abs=0.06)
    # independent of query pattern (no hidden state)
    assert a.at(5.0, 0) == a.at(5.0, 0)


def test_gust_and_composite():
    g = GustWind(2.0, 1.0, 4.0, 270.0)
    assert g.at(1.9, 0)[0] == 0.0 and g.at(3.1, 0)[0] == 0.0
    assert g.at(2.5, 0)[0] == pytest.approx(4.0)  # peak of 1-cos
    comp = CompositeWind([ConstantWind(1.0, 270.0), g])
    assert comp.at(2.5, 0)[0] == pytest.approx(5.0)


def test_terrain():
    s = SlopeTerrain(10.0, 90.0)  # rises toward the East
    assert s.height(100.0, 0.0) == pytest.approx(100.0 * math.tan(math.radians(10)))
    assert s.height(0.0, 100.0) == pytest.approx(0.0, abs=1e-12)
    g = GridTerrain([0, 10], [0, 10], [[0, 10], [20, 30]])
    assert g.height(5, 5) == pytest.approx(15.0)
    assert g.height(-50, 5) == pytest.approx(g.height(0, 5))  # clamped at the edge
