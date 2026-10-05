"""Independent cross-check of the geometry-derived stability (CP, CNalpha) against RocketPy 1.x.

For each reconstructed EuRoC vehicle, build the SAME nose / body / (boat-tail) / trapezoidal-fin geometry in RocketPy and
compare the subsonic normal-force slope and centre of pressure with this simulator's Barrowman build-up.
RocketPy implements the same Barrowman equations independently (different code, different author), so agreement tests the
implementation, not the theory. Known, expected differences: RocketPy's von Karman nose CP (0.5 L) vs this simulator's
tangent-ogive approximation (0.466 L), and RocketPy's treatment of the fin-body interference factor.

Run:  python validation_data/crosscheck_geometry.py
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
try:
    import netCDF4  # noqa: F401
except ImportError:
    sys.modules["netCDF4"] = types.ModuleType("netCDF4")

from rocketpy import Rocket  # noqa: E402

from rocket_sim.config import config_from_dict, read_mapping  # noqa: E402
from rocket_sim.simulation import Simulation  # noqa: E402

# name -> (sim yaml, rocketpy kwargs). Coordinates are RocketPy's (tail_to_nose, origin as in the example notebooks).
CASES = {
    "erebus11": dict(
        radius=0.04575,
        tip=2.3841,
        nose=(0.391113, 2.3841),
        fins=(4, 0.08, 0.12, 0.05, 0.395787),
        tail=(0.04575, 0.03575, 0.079119, 0.290787),
    ),
    "genesis": dict(radius=0.047, tip=2.372, nose=(0.27, 2.372), fins=(4, 0.105, 0.2, 0.11, 0.25), tail=None),
    "cavour": dict(
        radius=0.052, tip=2.7224, nose=(0.52, 2.7224), fins=(4, 0.1, 0.2, 0.07, 0.2104), tail=None
    ),
    "astra": dict(
        radius=0.047, tip=2.5214, nose=(0.27, 2.5214), fins=(4, 0.117, 0.2, 0.118, 0.9134), tail=None
    ),
}


def rocketpy_numbers(c: dict) -> tuple[float, float]:
    r = Rocket(
        radius=c["radius"],
        mass=10.0,
        inertia=(1, 1, 0.01),
        power_off_drag=0.5,
        power_on_drag=0.5,
        center_of_mass_without_motor=1.0,
        coordinate_system_orientation="tail_to_nose",
    )
    r.add_nose(length=c["nose"][0], kind="vonKarman", position=c["nose"][1])
    n, span, root, tip, pos = c["fins"]
    r.add_trapezoidal_fins(n=n, span=span, root_chord=root, tip_chord=tip, position=pos)
    if c["tail"]:
        top, bottom, length, tpos = c["tail"]
        r.add_tail(top_radius=top, bottom_radius=bottom, length=length, position=tpos)
    cna = float(r.total_lift_coeff_der(0.0))
    cp_z = float(r.cp_position(0.0))  # position in rocket coordinates (tail_to_nose)
    return cna, c["tip"] - cp_z  # CP measured from the nose tip


def ours(name: str) -> tuple[float, float]:
    p = ROOT / "validation_data" / f"{name}_sim.yaml"
    sim = Simulation(config_from_dict(read_mapping(p), base_dir=p.parent))
    co = sim.vehicle.aero.coefficients(0.05, 5e6, False)
    return co.cn_alpha, co.x_cp


if __name__ == "__main__":
    print(
        f"{'vehicle':<10}{'CNa RocketPy':>14}{'CNa ours':>10}{'diff %':>8}{'CP RocketPy [m]':>17}{'CP ours [m]':>13}{'diff [cal]':>12}"
    )
    worst = 0.0
    for name, c in CASES.items():
        cna_r, cp_r = rocketpy_numbers(c)
        cna_o, cp_o = ours(name)
        cal = 2 * c["radius"]
        d_cp = (cp_o - cp_r) / cal
        worst = max(worst, abs(d_cp))
        print(
            f"{name:<10}{cna_r:14.3f}{cna_o:10.3f}{100 * (cna_o / cna_r - 1):8.2f}{cp_r:17.4f}{cp_o:13.4f}{d_cp:12.3f}"
        )
    print(f"\nworst CP difference: {worst:.3f} calibres")
