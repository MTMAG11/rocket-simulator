"""Aerodynamic models: coefficient providers used by the 3-DOF and 6-DOF dynamics.

All coefficients are referenced to S_ref = pi d^2 / 4 (body cross-section).

An ``AerodynamicModel`` returns, for a flow condition (Mach, Reynolds, motor-burning flag):
    cd0       axial/drag coefficient at zero angle of attack
    cn_alpha  normal-force-curve slope [1/rad] (linear, small-angle)
    x_cp      centre of pressure of that normal force, aft of nose tip [m]
and exposes the crossflow parameters used at large angle of attack.

Force law used by the 6-DOF dynamics (alpha = total angle of attack, 0..pi):
    axial   F_x   = -q S cd0 cos(alpha)                       (body x, along nose axis)
    lateral F_lat = q [ S cn_alpha sin(alpha)|cos(alpha)|     acting at x_cp
                      + Cd_cf A_planform sin^2(alpha) ]       acting at planform centroid
directed opposite to the lateral relative air velocity (also for reversed flow, alpha > 90 deg,
where a body with its CP aft of the CG is therefore *unstable*, as a thrown dart is). At alpha -> 0 this reduces to
D = q S cd0, N = q S cn_alpha alpha. Induced drag emerges from the projection of the normal
force onto the velocity vector (no separate k*CL^2 term is needed).

Models
------
ConstantAero   fixed cd0/cn_alpha/x_cp (analysis, unit tests)
TableAero      cd0 interpolated against Mach (e.g. from CFD/wind-tunnel/OpenRocket export)
BuildupAero    component build-up (default): skin friction (Re, Mach, roughness), form factor,
               fin friction, fin-edge pressure drag, base drag (reduced when the motor burns),
               nose wave drag, Barrowman normal-force/CP.

References
----------
[1] Barrowman, J.S., Barrowman, J.A. (1966/1967) "The Theoretical Prediction of the Center of
    Pressure", NARAM-8 / "Stability of a Model Rocket in Flight" (Centuri TIR-33).
[2] Niskanen, S. (2013) "Development of an Open Source model rocket simulation software"
    (OpenRocket technical documentation), M.Sc. thesis, Helsinki Univ. of Technology.
[3] Hoerner, S.F. (1965) "Fluid-Dynamic Drag" (form factor, crossflow Cd of cylinders).
[4] Schlichting, H. (1979) "Boundary-Layer Theory" (flat-plate skin friction).

Limitations (be honest about accuracy): Barrowman normal force/CP is valid for small alpha and
subsonic-to-low-transonic Mach (< ~0.8-1.0). Drag in the transonic regime and above uses
a smooth ramp of a Newtonian-like nose-wave term and is only good to roughly +-25-30%.
Fin flutter, fin-body interference beyond K_fb, launch lugs, rail buttons, surface
imperfections and boat-tails are not modelled. Treat component drag as a prior to be
calibrated against flight data (``drag_scale``), not as truth.
"""

from __future__ import annotations

import bisect
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from ..errors import ConfigError
from .geometry import NOSE_WETTED_FACTOR, NOSE_XCP_FRACTION, BodyTube, FinSet, NoseCone


@dataclass(frozen=True)
class AeroCoefficients:
    cd0: float
    cn_alpha: float
    x_cp: float  # [m] aft of nose tip


class AerodynamicModel(ABC):
    name = "aero"
    ref_diameter: float
    ref_area: float
    crossflow_cd: float = 1.2
    planform_area: float = 0.0
    x_crossflow: float = 0.0  # [m] aft of nose where crossflow force acts
    # strip-theory damping geometry: list of (cn_alpha_i, x_i) pairs and roll data
    damping_surfaces: tuple[tuple[float, float], ...] = ()
    roll_damping_cn: float = 0.0  # total fin CNalpha for roll damping
    roll_damping_radius: float = 0.0  # [m]

    @abstractmethod
    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        """Flow-condition dependent coefficients."""


class ConstantAero(AerodynamicModel):
    name = "constant"

    def __init__(
        self,
        diameter: float,
        cd: float,
        cn_alpha: float = 0.0,
        x_cp: float = 0.0,
        crossflow_cd: float = 0.0,
        planform_area: float = 0.0,
        x_crossflow: float = 0.0,
    ) -> None:
        if cd < 0 or diameter <= 0:
            raise ConfigError("ConstantAero: cd >= 0 and diameter > 0 required")
        self.ref_diameter = diameter
        self.ref_area = math.pi * diameter**2 / 4.0
        self._c = AeroCoefficients(cd, cn_alpha, x_cp)
        self.crossflow_cd = crossflow_cd
        self.planform_area = planform_area
        self.x_crossflow = x_crossflow
        if cn_alpha > 0:
            self.damping_surfaces = ((cn_alpha, x_cp),)

    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        return self._c


class TableAero(AerodynamicModel):
    """cd0 vs Mach table (linear interpolation, held constant outside the table)."""

    name = "table"

    def __init__(
        self,
        diameter: float,
        mach: list[float],
        cd: list[float],
        cn_alpha: float = 0.0,
        x_cp: float = 0.0,
        cd_powered: list[float] | None = None,
        crossflow_cd: float = 0.0,
        planform_area: float = 0.0,
        x_crossflow: float = 0.0,
    ) -> None:
        if len(mach) < 1 or len(mach) != len(cd):
            raise ConfigError("cd table needs equal-length, non-empty mach and cd columns")
        if any(b <= a for a, b in zip(mach, mach[1:], strict=False)):
            raise ConfigError("cd table Mach values must be strictly increasing")
        if min(cd) < 0 or (cd_powered is not None and min(cd_powered) < 0):
            raise ConfigError("cd table values must be >= 0")
        if cd_powered is not None and len(cd_powered) != len(mach):
            raise ConfigError("cd_powered must have the same length as the mach column")
        self.ref_diameter = diameter
        self.ref_area = math.pi * diameter**2 / 4.0
        self.m, self.cd, self.cdp = list(mach), list(cd), cd_powered
        self.cn, self.xcp = cn_alpha, x_cp
        self.crossflow_cd, self.planform_area, self.x_crossflow = (
            crossflow_cd,
            planform_area,
            x_crossflow,
        )
        if cn_alpha > 0:
            self.damping_surfaces = ((cn_alpha, x_cp),)

    @staticmethod
    def _interp(xs: list[float], ys: list[float], x: float) -> float:
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        i = bisect.bisect_right(xs, x) - 1
        f = (x - xs[i]) / (xs[i + 1] - xs[i])
        return ys[i] + f * (ys[i + 1] - ys[i])

    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        table = self.cdp if (powered and self.cdp is not None) else self.cd
        return AeroCoefficients(self._interp(self.m, table, mach), self.cn, self.xcp)


def stagnation_pressure_coefficient(mach: float) -> float:
    """Stagnation pressure coefficient (compressible Pitot), Niskanen (2013) eq. 3.xx."""
    if mach < 1.0:
        return 1.0 + mach**2 / 4.0 + mach**4 / 40.0
    return 1.84 - 0.76 / mach**2 + 0.166 / mach**4 + 0.035 / mach**6


def base_drag_coefficient(mach: float, forebody_cd: float = 0.35) -> float:
    """Base-pressure drag coefficient referenced to base area.

    Subsonic: Hoerner (Fluid-Dynamic Drag, 1965, ch. 3) for turbulent axisymmetric bodies,
        C_Db = 0.029 / sqrt(C_D,forebody)        (forebody drag referenced to the base area).
    Supersonic (M >= 1): Barrowman C_Db = 0.25 / M. Between M = 0.8 and 1.0 the two are blended
    with a smoothstep (base-pressure rise toward the sonic condition).
    This replaces the Barrowman subsonic 0.12 + 0.13 M^2 of physics v1.0.0, which made total Cd
    rise by ~0.08 between M = 0.1 and 0.85, contrary to the flight data and RASAero curves of the
    validation flights (see docs/validation.md, change log v1.1.0).
    """
    sub = 0.029 / math.sqrt(max(forebody_cd, 0.05))
    sup = 0.25 / max(mach, 1.0)
    if mach <= 0.8:
        return sub
    if mach >= 1.0:
        return sup
    w = smoothstep((mach - 0.8) / 0.2)
    return (1.0 - w) * sub + w * 0.25


def skin_friction_coefficient(reynolds: float, mach: float, roughness: float, length: float) -> float:
    """Flat-plate skin-friction coefficient.

    Laminar below Re = 5e5, otherwise turbulent (Schlichting) with the Reynolds number capped at
    the roughness cut-off Re_cutoff = 38.21 (L/k)^1.053 (Raymer, Aircraft Design, ch. 12), i.e.
    roughness limits the benefit of higher Re. The compressibility factor (1 + 0.144 M^2)^-0.65
    applies to the turbulent result (smooth or rough-capped).
    """
    re = max(reynolds, 1.0e3)
    if re < 5.0e5:
        return 1.328 / math.sqrt(re)
    if roughness > 0 and length > 0:
        re = min(re, 38.21 * (length / roughness) ** 1.053)
    cf = 0.455 / (math.log10(re) ** 2.58)
    return cf / (1.0 + 0.144 * mach**2) ** 0.65


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class BuildupAero(AerodynamicModel):
    """Component build-up drag + Barrowman normal force / CP. See module docstring."""

    name = "buildup"

    def __init__(
        self,
        body: BodyTube,
        nose: NoseCone,
        fins: FinSet,
        nozzle_exit_diameter: float,
        surface_roughness: float = 60e-6,
        crossflow_cd: float = 1.2,
        extra_cd: float = 0.0,
        drag_scale: float = 1.0,
        fin_leading_edge_factor: float = 0.5,
        fin_trailing_edge_factor: float = 1.0,
    ) -> None:
        if nozzle_exit_diameter < 0 or nozzle_exit_diameter >= body.diameter:
            raise ConfigError("nozzle exit diameter must be in [0, body diameter)")
        if surface_roughness < 0 or extra_cd < 0 or drag_scale <= 0:
            raise ConfigError("aero: roughness/extra_cd >= 0 and drag_scale > 0 required")
        self.body, self.nose, self.fins = body, nose, fins
        self.ref_diameter = body.diameter
        self.ref_area = body.reference_area
        self.crossflow_cd = crossflow_cd
        self.roughness = surface_roughness
        self.extra_cd = extra_cd
        self.drag_scale = drag_scale
        self.f_le, self.f_te = fin_leading_edge_factor, fin_trailing_edge_factor
        self.nozzle_exit_diameter = nozzle_exit_diameter

        d, r = body.diameter, body.radius
        ln = nose.length
        tube_len = body.length - ln
        # wetted area of body (nose approximated from cone area x shape factor, plus tube)
        s_nose = math.pi * r * math.sqrt(r * r + ln * ln) * NOSE_WETTED_FACTOR[nose.shape]
        self._s_wet_body = s_nose + math.pi * d * tube_len
        self._fineness = body.length / d
        self._form_factor = 1.0 + 60.0 / self._fineness**3 + 0.0025 * self._fineness
        self._nose_half_angle = math.atan2(r, ln)

        # -- Barrowman normal force and centre of pressure -----------------------------------
        self._cn_nose = 2.0
        self._x_nose = NOSE_XCP_FRACTION[nose.shape] * ln
        self._cn_fins = 0.0
        self._x_fins = 0.0
        if fins.count > 0:
            s = fins.span
            k_fb = 1.0 + r / (s + r)
            lf = fins.midchord_length
            n = fins.count
            denom = 1.0 + math.sqrt(1.0 + (2.0 * lf / (fins.root_chord + fins.tip_chord)) ** 2)
            cn = 4.0 * n * (s / d) ** 2 / denom * k_fb
            if n == 2:
                cn *= 0.5  # roll-averaged two-fin set: only half the planform is normal to flow
            self._cn_fins = cn
            cr, ct, xm = fins.root_chord, fins.tip_chord, fins.sweep
            x_f = (xm / 3.0) * (cr + 2.0 * ct) / (cr + ct) + (1.0 / 6.0) * ((cr + ct) - cr * ct / (cr + ct))
            self._x_fins = fins.leading_edge_from_nose + x_f
        self._cn_total = self._cn_nose + self._cn_fins
        self._x_cp = (self._cn_nose * self._x_nose + self._cn_fins * self._x_fins) / self._cn_total
        surfaces = [(self._cn_nose, self._x_nose)]
        if self._cn_fins > 0:
            surfaces.append((self._cn_fins, self._x_fins))
        self.damping_surfaces = tuple(surfaces)
        self.roll_damping_cn = self._cn_fins
        self.roll_damping_radius = r + 0.5 * fins.span if fins.count else 0.0

        # crossflow planform (side projected area of the body) acts at its centroid
        self.planform_area = d * (tube_len + 0.5 * ln)
        self.x_crossflow = 0.5 * body.length
        self._base_area_ratio = 1.0  # base area / S_ref when coasting
        self._powered_area_ratio = 1.0 - (nozzle_exit_diameter / d) ** 2

        # fin geometry terms precomputed
        self._fin_wet = 2.0 * fins.count * fins.planform_area if fins.count else 0.0
        self._fin_edge_len = fins.count * fins.span if fins.count else 0.0
        self._cos2_le = math.cos(fins.leading_edge_sweep_angle) ** 2 if fins.count else 1.0
        self._fin_tc = (fins.thickness / fins.mean_chord) if fins.count else 0.0

    # -- static (Mach-independent) stability data --------------------------------------------
    @property
    def cn_alpha_total(self) -> float:
        return self._cn_total

    @property
    def x_cp_subsonic(self) -> float:
        return self._x_cp

    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        s_ref = self.ref_area
        # friction on the body, Reynolds number based on total length
        re_body = max(reynolds, 1.0)
        cf_b = skin_friction_coefficient(re_body, mach, self.roughness, self.body.length)
        cd_body_f = cf_b * self._form_factor * self._s_wet_body / s_ref
        cd = cd_body_f
        fins = self.fins
        if fins.count:
            re_fin = re_body * fins.mean_chord / self.body.length
            cf_f = skin_friction_coefficient(re_fin, mach, self.roughness, fins.mean_chord)
            cd += cf_f * (1.0 + 2.0 * self._fin_tc) * self._fin_wet / s_ref
            # leading-edge (stagnation) pressure drag of the fin edges
            edge = fins.thickness * self._fin_edge_len / s_ref
            cd += edge * self.f_le * stagnation_pressure_coefficient(mach) * self._cos2_le
        # nose wave drag: ramps in between M = 0.8 and 1.2, Newtonian-like 0.8 sin^2(phi)
        cd += 0.8 * math.sin(self._nose_half_angle) ** 2 * smoothstep((mach - 0.8) / 0.4)
        # base drag (Hoerner) from the forebody drag; blunt trailing edges of the fins are base-like
        ratio = self._powered_area_ratio if powered else self._base_area_ratio
        cdb = base_drag_coefficient(mach, cd)
        cd += cdb * ratio
        if fins.count:
            cd += self.fins.thickness * self._fin_edge_len / s_ref * self.f_te * cdb
        cd = (cd + self.extra_cd) * self.drag_scale
        return AeroCoefficients(cd, self._cn_total, self._x_cp)
