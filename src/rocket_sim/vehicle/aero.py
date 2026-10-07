"""Aerodynamic models: coefficient providers used by the 3-DOF and 6-DOF dynamics.

All coefficients are referenced to S_ref = pi d_ref^2 / 4 (the reference diameter, normally the largest
body diameter).

Model hierarchy (select with ``rocket.aero.model``)
---------------------------------------------------
simplified  constant Cd (user value) + geometry-derived stability (Barrowman CNa/CP from the assembly)
barrowman   Barrowman normal force / CP (Mach-independent) + component drag build-up (Re, Mach, roughness,
            base/boat-tail, transitions). The V1 model; the baseline against which others are compared.
enhanced    barrowman + Mach-dependent fin lift slope and fin CP (Helmbold/Diederich subsonic, Ackeret
            supersonic, blended through the transonic range), fin stall at large angle of attack, supersonic /
            transonic fin wave drag.
table       Cd(Mach) lookup (e.g. a team's RASAero/CFD/wind-tunnel curve), coast and powered variants,
            geometry-derived (or supplied) CNa/CP.
table2d     full Cd/Cl/Cm(Mach, alpha) lookup from a file (CFD / wind-tunnel / OpenRocket export).
constant    fixed numbers (analysis and unit tests).

Coefficient interface (spec: Cd, Cl, Cm as functions of Mach, alpha, Reynolds, geometry)
-----------------------------------------------------------------------------------------
``coefficients(mach, reynolds, powered)`` -> small-angle data (cd0, CNa, x_cp), and the generic
``force_coefficients(mach, alpha, reynolds, powered)`` -> body-axis (CA, CN) plus the CP of the normal force,
valid for the full angle-of-attack range 0..pi. Wind-axis coefficients are derived consistently:

    CD(M, alpha, Re) = CA cos(alpha) + CN sin(alpha)        (drag, along -v_rel)
    CL(M, alpha, Re) = CN cos(alpha) - CA sin(alpha)        (lift, normal to v_rel in the alpha plane)
    Cm(M, alpha, Re; x_ref) = -CN (x_cp - x_ref) / d_ref    (about x_ref; negative = restoring)

The geometry enters through the ``Assembly`` the model was built from.

Force law (all assembly-based models), alpha = total angle of attack in 0..pi:
    axial   F_x   = -q S CA(M,Re) cos(alpha)
    lateral F_lat = q S CN(M, alpha) opposing the lateral relative air velocity, acting at x_cp_force
    CN = CNa sin(alpha)|cos(alpha)| + Cd_cf (A_planform/S) sin^2(alpha)
(the |cos| makes tail-first flight unstable, as for a thrown dart). Induced drag comes from projecting CN.

References
----------
[1] Barrowman & Barrowman (1966/67) "The Theoretical Prediction of the Center of Pressure"; Centuri TIR-33.
[2] Niskanen (2013) OpenRocket technical documentation.
[3] Hoerner (1965) Fluid-Dynamic Drag (form factor, crossflow Cd, base and boat-tail drag).
[4] Schlichting (1979) Boundary-Layer Theory; Raymer, Aircraft Design (skin friction, roughness cut-off,
    Helmbold/Diederich lift slope, supersonic Ackeret lift slope).
[5] Ackeret linear theory for supersonic thin-airfoil lift and wave drag.

Accuracy (honest): Barrowman normal force/CP is valid for small alpha and subsonic-low-transonic Mach; the
transonic regime (M ~ 0.8-1.2) in the enhanced model is a smooth blend between two linear-theory limits and
carries +-30-50 % uncertainty in fin CNa and wave drag; component drag is good to roughly +-10-15 % subsonic.
Boat-tail separation uses a rule-of-thumb threshold. Reynolds number enters through skin friction only.
Fin flutter, fin cant, protuberances (use ``extra_cd``) and plume effects are not modelled.
"""

from __future__ import annotations

import bisect
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from ..errors import ConfigError
from .assembly import Assembly, ControlSurfaceSet, legacy_assembly
from .geometry import BodyTube, FinSet, NoseCone


@dataclass(frozen=True)
class AeroCoefficients:
    """Small-angle data at a flow condition."""

    cd0: float  # axial / zero-alpha drag coefficient
    cn_alpha: float  # normal-force-curve slope [1/rad]
    x_cp: float  # [m] aft of nose tip (CP of the linear normal force = static CP)


@dataclass(frozen=True)
class ForceCoefficients:
    """Body-axis force coefficients at (Mach, alpha, Re)."""

    ca: float  # axial force coefficient: F_x = -q S ca (includes the cos(alpha) factor)
    cn: float  # total normal force coefficient (>= 0, directed against the lateral velocity)
    x_cp_force: float  # CP of the total normal force (includes crossflow) [m aft of nose]
    x_cp_static: float  # static (linear) CP [m aft of nose], for stability / static margin


@dataclass(frozen=True)
class ControlFin:
    """One movable fin: geometry needed for its force and moment."""

    phi: float  # angular position about the nose axis [rad], from +y_B toward +z_B
    x_ac: float  # aerodynamic centre, aft of the nose [m]
    rho: float  # radial distance of the surface centre from the axis [m]
    cn_alpha_single: float  # lift-curve slope of ONE fin referenced to S_ref [1/rad] (subsonic)
    max_deflection: float
    max_rate: float
    time_constant: float
    delay: float


PROVENANCE_KINDS = (
    "analytical",
    "barrowman",
    "empirical",
    "imported_table",
    "experimental",
    "cfd",
    "user_defined",
)
# kinds that are ESTIMATES made by this simulator or by a textbook method (as opposed to data brought in from outside)
ESTIMATE_KINDS = ("analytical", "barrowman", "empirical")


@dataclass(frozen=True)
class ProvItem:
    """Where one group of aerodynamic numbers comes from."""

    kind: str
    source: str
    confidence: str = "unknown"  # low | medium | high | unknown (a judgement, stated, not computed)

    def __post_init__(self) -> None:
        if self.kind not in PROVENANCE_KINDS:
            raise ConfigError(f"aero provenance kind must be one of {PROVENANCE_KINDS}, got {self.kind!r}")

    @property
    def is_estimate(self) -> bool:
        return self.kind in ESTIMATE_KINDS


@dataclass(frozen=True)
class AeroProvenance:
    """Provenance of every aerodynamic quantity a model supplies, plus its stated range of applicability.

    ``reynolds_dependence`` states honestly how (or whether) Reynolds number enters: it is NOT silently assumed."""

    model: str
    drag: ProvItem
    normal_force_cp: ProvItem
    damping: ProvItem
    mach_range: tuple[float, float] | None = None
    alpha_range_deg: tuple[float, float] | None = None
    reynolds_range: tuple[float, float] | None = None
    reynolds_dependence: str = "not modelled"
    notes: tuple[str, ...] = ()

    @property
    def kind(self) -> str:
        """One label for datasets: 'estimate' (all analytical), 'imported' (all external data), or 'mixed'."""
        est = [self.drag.is_estimate, self.normal_force_cp.is_estimate, self.damping.is_estimate]
        return "estimate" if all(est) else ("imported" if not any(est) else "mixed")

    def to_dict(self) -> dict:
        return {
            "model": self.model,
            "kind": self.kind,
            "drag": vars(self.drag).copy(),
            "normal_force_cp": vars(self.normal_force_cp).copy(),
            "damping": vars(self.damping).copy(),
            "mach_range": list(self.mach_range) if self.mach_range else None,
            "alpha_range_deg": list(self.alpha_range_deg) if self.alpha_range_deg else None,
            "reynolds_range": list(self.reynolds_range) if self.reynolds_range else None,
            "reynolds_dependence": self.reynolds_dependence,
            "notes": list(self.notes),
        }


_BARROWMAN_CP = ProvItem(
    "barrowman", "Barrowman (1966/67) normal force and CP from the component geometry", "medium"
)
_BARROWMAN_DAMP = ProvItem("analytical", "strip-theory damping from the Barrowman lift slopes", "low")


def default_provenance(
    name: str,
    user: dict | None = None,
    mach_range: tuple[float, float] | None = None,
    alpha_range_deg: tuple[float, float] | None = None,
    stability_supplied: bool = False,
) -> AeroProvenance:
    """Provenance of a model before any user declaration. ``user`` ({kind, source, confidence}) describes the DRAG /
    coefficient data of table-type and constant models (the numbers the user brought)."""
    u = None
    if user:
        u = ProvItem(
            str(user.get("kind", "user_defined")),
            str(user.get("source", "unspecified")),
            str(user.get("confidence", "unknown")),
        )
    if name in ("barrowman", "simplified", "enhanced"):
        drag = {
            "barrowman": ProvItem(
                "empirical",
                "component drag build-up (Hoerner form factor, Schlichting/Raymer skin friction, Hoerner base drag)",
                "medium",
            ),
            "simplified": u
            or ProvItem("user_defined", "flat Cd supplied in the config (aero.cd)", "unknown"),
            "enhanced": ProvItem(
                "empirical", "Barrowman drag build-up plus Ackeret fin wave drag (linear theory)", "low"
            ),
        }[name]
        mr = {"barrowman": (0.0, 0.9), "simplified": mach_range, "enhanced": (0.0, 2.0)}[name]
        notes = {
            "barrowman": (
                "subsonic validity; no flight data above ~Mach 0.9 (docs/validation.md)",
                "linear-theory normal force to ~15 deg angle of attack; beyond it a crossflow estimate",
            ),
            "simplified": ("drag is the user's number; stability is Barrowman",),
            "enhanced": (
                "transonic fin lift is a blend of two linear-theory limits (+-30-50 %); NO real-flight validation",
            ),
        }[name]
        return AeroProvenance(
            name,
            drag,
            _BARROWMAN_CP,
            _BARROWMAN_DAMP,
            mr,
            alpha_range_deg or (0.0, 15.0),
            None,
            "skin friction only (laminar/turbulent Cf and roughness cut-off); pressure drag and lift are Re-independent",
            notes,
        )
    drag = u or ProvItem(
        "user_defined", "coefficients supplied in the config with no declared provenance", "unknown"
    )
    stab = ProvItem("user_defined", "supplied CNa/CP", "unknown") if stability_supplied else _BARROWMAN_CP
    damp = _BARROWMAN_DAMP
    if name == "table2d":
        stab = u or ProvItem("imported_table", "CL/CM columns of the imported table", "unknown")
        damp = _BARROWMAN_DAMP
    notes = (
        ("no Reynolds-number axis: Cd/Cl/Cm are taken as Re-independent",)
        if name != "constant"
        else ("fixed coefficients: no Mach, alpha or Re dependence",)
    )
    return AeroProvenance(
        name, drag, stab, damp, mach_range, alpha_range_deg, None, "none (data has no Reynolds axis)", notes
    )


def smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class AerodynamicModel(ABC):
    name = "aero"
    ref_diameter: float
    ref_area: float
    crossflow_cd: float = 1.2
    planform_area: float = 0.0
    x_crossflow: float = 0.0  # [m] aft of nose where the crossflow force acts
    # strip-theory damping geometry: (cn_alpha_i, x_i) pairs and roll data
    damping_surfaces: tuple[tuple[float, float], ...] = ()
    roll_damping_cn: float = 0.0
    roll_damping_radius: float = 0.0
    control_fins: tuple[ControlFin, ...] = ()
    provenance: AeroProvenance | None = None  # set by the builder; see default_provenance

    @abstractmethod
    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        """Small-angle coefficients for a flow condition."""

    def force_coefficients(
        self, mach: float, alpha: float, reynolds: float, powered: bool
    ) -> ForceCoefficients:
        """Default body-axis law from the small-angle data plus crossflow (see module docstring)."""
        c = self.coefficients(mach, reynolds, powered)
        sa = math.sin(alpha)
        ca_ = math.cos(alpha)
        cn_lin = c.cn_alpha * sa * abs(ca_)
        cn_cf = (
            self.crossflow_cd * self.planform_area / self.ref_area * sa * sa if self.planform_area else 0.0
        )
        cn = cn_lin + cn_cf
        x_cp = (cn_lin * c.x_cp + cn_cf * self.x_crossflow) / cn if cn > 1e-12 else c.x_cp
        return ForceCoefficients(c.cd0 * ca_, cn, x_cp, c.x_cp)

    # -- wind-axis / moment coefficients derived from the force coefficients -----------------------
    def cd(self, mach: float, alpha: float, reynolds: float, powered: bool = False) -> float:
        """Drag coefficient (wind axes): D = CA cos(alpha) + CN sin(alpha)."""
        f = self.force_coefficients(mach, alpha, reynolds, powered)
        return f.ca * math.cos(alpha) + f.cn * math.sin(alpha)

    def cl(self, mach: float, alpha: float, reynolds: float, powered: bool = False) -> float:
        """Lift coefficient (wind axes, in the alpha plane): L = CN cos(alpha) - CA sin(alpha)."""
        f = self.force_coefficients(mach, alpha, reynolds, powered)
        return f.cn * math.cos(alpha) - f.ca * math.sin(alpha)

    def cm(self, mach: float, alpha: float, reynolds: float, x_ref: float, powered: bool = False) -> float:
        """Pitching-moment coefficient about axial station x_ref (aft of nose); negative = restoring."""
        f = self.force_coefficients(mach, alpha, reynolds, powered)
        return -f.cn * (f.x_cp_force - x_ref) / self.ref_diameter

    def control_cn_alpha(self, mach: float) -> float:
        """Mach factor on the single-fin lift slope of control fins (1 for models without Mach dependence)."""
        return 1.0


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


def _interp1(xs: list[float], ys: list[float], x: float) -> float:
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_right(xs, x) - 1
    f = (x - xs[i]) / (xs[i + 1] - xs[i])
    return ys[i] + f * (ys[i + 1] - ys[i])


class TableAero(AerodynamicModel):
    """cd0 vs Mach lookup (linear interpolation, held constant outside the table); coast + powered variants.

    Stability (CNa, CP) are supplied constants or, when built from an assembly, the Barrowman values.
    """

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
        damping_surfaces: tuple[tuple[float, float], ...] | None = None,
        roll_damping_cn: float = 0.0,
        roll_damping_radius: float = 0.0,
        control_fins: tuple[ControlFin, ...] = (),
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
        self.m, self.cd_table, self.cdp = list(mach), list(cd), cd_powered
        self.cn, self.xcp = cn_alpha, x_cp
        self.crossflow_cd, self.planform_area, self.x_crossflow = crossflow_cd, planform_area, x_crossflow
        if damping_surfaces is not None:
            self.damping_surfaces = damping_surfaces
        elif cn_alpha > 0:
            self.damping_surfaces = ((cn_alpha, x_cp),)
        self.roll_damping_cn, self.roll_damping_radius = roll_damping_cn, roll_damping_radius
        self.control_fins = control_fins

    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        table = self.cdp if (powered and self.cdp is not None) else self.cd_table
        return AeroCoefficients(_interp1(self.m, table, mach), self.cn, self.xcp)


class Table2DAero(AerodynamicModel):
    """Full CD / CL (and optional CM) lookup over (Mach, alpha): the interface for CFD / wind-tunnel data.

    Tables are bilinear-interpolated, clamped at the edges. ``alpha`` axis in radians, covering [0, pi]
    (or [0, alpha_max]; beyond alpha_max the last row is held and the |sin cos| law is NOT extrapolated,
    so supply data over the range you fly). CD/CL are wind-axis coefficients about the reference area;
    CM (optional) is about ``x_cm_ref`` (aft of nose), positive nose-up. Without CM the normal-force CP is
    taken at ``x_cp`` (constant). Converted to body axes: CN = CL cos(a) + CD sin(a), CA = CD cos(a) - CL sin(a).
    """

    name = "table2d"

    def __init__(
        self,
        diameter: float,
        mach: list[float],
        alpha: list[float],
        cd: list[list[float]],
        cl: list[list[float]],
        cm: list[list[float]] | None = None,
        x_cm_ref: float = 0.0,
        x_cp: float = 0.0,
        damping_surfaces: tuple[tuple[float, float], ...] = (),
        roll_damping_cn: float = 0.0,
        roll_damping_radius: float = 0.0,
    ) -> None:
        nm, na = len(mach), len(alpha)
        if nm < 1 or na < 2:
            raise ConfigError("table2d needs >= 1 Mach and >= 2 alpha values")
        for arr, nm_ in ((cd, "cd"), (cl, "cl")) + (((cm, "cm"),) if cm is not None else ()):
            if len(arr) != nm or any(len(r) != na for r in arr):
                raise ConfigError(f"table2d {nm_} must have shape (len(mach), len(alpha)) = ({nm}, {na})")
        if any(b <= a for a, b in zip(mach, mach[1:], strict=False)) or any(
            b <= a for a, b in zip(alpha, alpha[1:], strict=False)
        ):
            raise ConfigError("table2d axes must be strictly increasing")
        if alpha[0] != 0.0:
            raise ConfigError("table2d alpha axis must start at 0")
        self.ref_diameter = diameter
        self.ref_area = math.pi * diameter**2 / 4.0
        self.m, self.a = list(mach), list(alpha)
        self.cd_t, self.cl_t, self.cm_t = cd, cl, cm
        self.x_cm_ref, self.xcp = x_cm_ref, x_cp
        self.damping_surfaces = damping_surfaces
        self.roll_damping_cn, self.roll_damping_radius = roll_damping_cn, roll_damping_radius
        self.crossflow_cd = 0.0
        self.planform_area = 0.0

    def _bilinear(self, tab: list[list[float]], mach: float, alpha: float) -> float:
        ms, as_ = self.m, self.a
        mc = min(max(mach, ms[0]), ms[-1])
        ac = min(max(alpha, as_[0]), as_[-1])
        i = min(max(bisect.bisect_right(ms, mc) - 1, 0), max(len(ms) - 2, 0))
        j = min(max(bisect.bisect_right(as_, ac) - 1, 0), len(as_) - 2)
        fm = 0.0 if len(ms) == 1 else (mc - ms[i]) / (ms[i + 1] - ms[i])
        fa = (ac - as_[j]) / (as_[j + 1] - as_[j])
        i1 = min(i + 1, len(ms) - 1)
        top = tab[i][j] * (1 - fa) + tab[i][j + 1] * fa
        bot = tab[i1][j] * (1 - fa) + tab[i1][j + 1] * fa
        return top * (1 - fm) + bot * fm

    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        a0 = self.a[1] * 0.25  # small angle for the slope
        cl = self._bilinear(self.cl_t, mach, a0)
        cn_a = cl / a0 if a0 > 0 else 0.0
        return AeroCoefficients(self._bilinear(self.cd_t, mach, 0.0), cn_a, self._cp_static(mach))

    def _cp_static(self, mach: float) -> float:
        if self.cm_t is None:
            return self.xcp
        a0 = self.a[1] * 0.25
        cl = self._bilinear(self.cl_t, mach, a0)
        cm = self._bilinear(self.cm_t, mach, a0)
        if abs(cl) < 1e-12:
            return self.xcp
        return self.x_cm_ref + cm / cl * self.ref_diameter  # CM positive nose-up: x_cp = x_ref + Cm d / CL

    def force_coefficients(
        self, mach: float, alpha: float, reynolds: float, powered: bool
    ) -> ForceCoefficients:
        cd = self._bilinear(self.cd_t, mach, alpha)
        cl = self._bilinear(self.cl_t, mach, alpha)
        sa, ca = math.sin(alpha), math.cos(alpha)
        cn = cl * ca + cd * sa
        axial = cd * ca - cl * sa
        if self.cm_t is not None and abs(cn) > 1e-9:
            cm = self._bilinear(self.cm_t, mach, alpha)
            x_cp = self.x_cm_ref + cm / cn * self.ref_diameter
        else:
            x_cp = self.xcp
        return ForceCoefficients(axial, max(cn, 0.0), x_cp, self._cp_static(mach))


# ------------------------------------------------------------------------------------------------
# component drag helpers
# ------------------------------------------------------------------------------------------------
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
    rise by ~0.08 between M = 0.1 and 0.85 (see docs/validation.md, change log v1.1.0).
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


def boat_tail_attached_fraction(half_angle: float) -> float:
    """Fraction of a boat-tail's base-area reduction that is realised (flow stays attached).

    Rule of thumb (Hoerner 1965, ch. 3): attached flow for conical boat-tail half-angles up to ~8 deg,
    fully separated beyond ~15 deg; linear in between. Approximate."""
    a = abs(math.degrees(half_angle))
    if a <= 8.0:
        return 1.0
    if a >= 15.0:
        return 0.0
    return (15.0 - a) / 7.0


def fin_normal_force(fins: FinSet, ref_diameter: float, local_radius: float) -> tuple[float, float]:
    """Barrowman fin-set CNa (referenced to d_ref) and CP position aft of the nose [m].

    CNa = K_fb 4 N (s/d)^2 / (1 + sqrt(1 + (2 L_f/(C_r+C_t))^2)),  K_fb = 1 + R/(s+R); a 2-fin set is halved
    (roll-averaged). CP from the root-chord leading edge: x_m/3 (Cr+2Ct)/(Cr+Ct) + 1/6 ((Cr+Ct) - CrCt/(Cr+Ct)).
    """
    if fins.count == 0:
        return 0.0, 0.0
    s, n = fins.span, fins.count
    k_fb = 1.0 + local_radius / (s + local_radius)
    lf = fins.midchord_length
    denom = 1.0 + math.sqrt(1.0 + (2.0 * lf / (fins.root_chord + fins.tip_chord)) ** 2)
    cn = 4.0 * n * (s / ref_diameter) ** 2 / denom * k_fb
    if n == 2:
        cn *= 0.5
    cr, ct, xm = fins.root_chord, fins.tip_chord, fins.sweep
    x_f = (xm / 3.0) * (cr + 2.0 * ct) / (cr + ct) + (1.0 / 6.0) * ((cr + ct) - cr * ct / (cr + ct))
    return cn, fins.leading_edge_from_nose + x_f


def fin_centroid_chordwise(fins: FinSet) -> float:
    """Chordwise centroid of the trapezoidal planform from the root leading edge [m]."""
    cr, ct, xm = fins.root_chord, fins.tip_chord, fins.sweep
    return (xm * (cr + 2.0 * ct) + cr * cr + cr * ct + ct * ct) / (3.0 * (cr + ct))


def fin_lift_slope_ratio(mach: float, fins: FinSet) -> float:
    """CLa(M)/CLa(0) of the fin set from linear theory, blended through the transonic range.

    Subsonic (M < 0.8): Helmbold/Diederich with Prandtl-Glauert compressibility,
        CLa = 2 pi AR / (2 + sqrt(4 + AR^2 beta^2 (1 + tan^2(L_mid)/beta^2))),  beta = sqrt(1 - M^2),
    with AR the effective aspect ratio 2 s^2 / A_fin (fin plus its mirror in the body).
    Supersonic (M > 1.2): Ackeret with finite-span tip loss, CLa = (4/b)(1 - 1/(2 AR b)), b = sqrt(M^2 - 1),
    limited below by the slender-wing value pi AR / 2. Between 0.8 and 1.2 the two end values are joined
    with a smoothstep: the true transonic fin lift is NOT represented (uncertainty +-30-50 %).
    """
    ar = 2.0 * fins.span**2 / fins.planform_area
    tan_l = math.tan(math.atan2(fins.sweep + 0.5 * (fins.tip_chord - fins.root_chord), fins.span))

    def sub(m: float) -> float:
        b2 = 1.0 - m * m
        return 2.0 * math.pi * ar / (2.0 + math.sqrt(4.0 + ar * ar * b2 * (1.0 + tan_l * tan_l / b2)))

    def sup(m: float) -> float:
        b = math.sqrt(m * m - 1.0)
        return max((4.0 / b) * (1.0 - 1.0 / (2.0 * ar * b)), 0.5 * math.pi * ar if ar * b < 1.0 else 0.0)

    ref = sub(0.0)
    if mach <= 0.8:
        return sub(mach) / ref
    lo, hi = sub(0.8) / ref, sup(1.2) / ref
    if mach >= 1.2:
        return sup(mach) / ref
    return lo + (hi - lo) * smoothstep((mach - 0.8) / 0.4)


class BarrowmanAero(AerodynamicModel):
    """Barrowman normal force / CP from the assembly + component drag build-up. See module docstring."""

    name = "barrowman"

    def __init__(
        self,
        assembly: Assembly,
        nozzle_exit_diameter: float,
        surface_roughness: float = 60e-6,
        crossflow_cd: float = 1.2,
        extra_cd: float = 0.0,
        drag_scale: float = 1.0,
        fin_leading_edge_factor: float = 0.5,
        fin_trailing_edge_factor: float = 1.0,
        constant_cd: float | None = None,
    ) -> None:
        asm = assembly
        if nozzle_exit_diameter < 0 or nozzle_exit_diameter >= asm.base_diameter:
            raise ConfigError("nozzle exit diameter must be in [0, base diameter)")
        if surface_roughness < 0 or extra_cd < 0 or drag_scale <= 0:
            raise ConfigError("aero: roughness/extra_cd >= 0 and drag_scale > 0 required")
        self.assembly = asm
        self.fins = asm.fins
        self.ref_diameter = asm.ref_diameter
        self.ref_area = asm.ref_area
        self.crossflow_cd = crossflow_cd
        self.roughness = surface_roughness
        self.extra_cd = extra_cd
        self.drag_scale = drag_scale
        self.f_le, self.f_te = fin_leading_edge_factor, fin_trailing_edge_factor
        self.nozzle_exit_diameter = nozzle_exit_diameter
        self.constant_cd = constant_cd
        d = self.ref_diameter
        s_ref = self.ref_area

        self._length = asm.length
        self._s_wet = asm.wetted_area()
        self._fineness = asm.length / d
        self._form_factor = 1.0 + 60.0 / self._fineness**3 + 0.0025 * self._fineness
        nose = asm.nose
        self._nose_half_angle = math.atan2(0.5 * nose.d_aft, nose.length)
        self._nose_area_ratio = (nose.d_aft / d) ** 2

        # -- Barrowman normal force and centre of pressure ----------------------------------------
        comps = list(asm.section_normal_force())  # (cna, x_cp, kind)
        fins = asm.fins
        self._cn_fins = 0.0
        self._x_fins = 0.0
        self._r_fin = 0.0
        if fins.count > 0:
            r_fin = asm.local_radius(fins.leading_edge_from_nose + 0.5 * fins.root_chord)
            self._r_fin = r_fin
            self._cn_fins, self._x_fins = fin_normal_force(fins, d, r_fin)
            comps.append((self._cn_fins, self._x_fins, "fins"))
        # control surfaces count as fixed lifting surfaces for the static (undeflected) stability
        cfins: list[ControlFin] = []
        for cs in asm.controls:
            rf = asm.local_radius(cs.leading_edge_from_nose + 0.5 * cs.root_chord)
            cn_set, x_cs = fin_normal_force(cs.fin, d, rf)
            if cs.count not in (2, 3, 4, 5, 6, 8):
                cn_set = 0.0
            comps.append((cn_set, x_cs, "control"))
            single = 2.0 * cn_set / cs.count
            rho = rf + 0.5 * cs.span
            for phi in cs.fin_angles():
                cfins.append(
                    ControlFin(
                        phi, x_cs, rho, single, cs.max_deflection, cs.max_rate, cs.time_constant, cs.delay
                    )
                )
        self.control_fins = tuple(cfins)
        self._cn_static = sum(c for c, _, _ in comps)
        if self._cn_static <= 0:
            raise ConfigError(
                "net normal-force slope is not positive: nothing keeps the nose pointed into the flow"
            )
        self._x_cp = sum(c * x for c, x, _ in comps) / self._cn_static
        self._comps = comps
        self.damping_surfaces = tuple((c, x) for c, x, k in comps if abs(c) > 0)
        self.roll_damping_cn = self._cn_fins
        self.roll_damping_radius = (self._r_fin + 0.5 * fins.span) if fins.count else 0.0

        # crossflow: planform of the whole stack at its centroid
        self.planform_area, self.x_crossflow = asm.planform()

        # base and step geometry
        self._a_base = (asm.base_diameter / d) ** 2  # base area / S_ref
        self._a_nozzle = (nozzle_exit_diameter / d) ** 2
        self._expansions: list[tuple[float, float]] = []  # (sin^2(phi), dA/S_ref)
        self._contraction_exposed = 0.0  # exposed rear-facing area / S_ref in the stack (not the final base)
        last = asm.sections[-1]
        for s in asm.sections:
            if s.kind != "transition":
                continue
            da = (math.pi / 4.0) * (s.d_aft**2 - s.d_fore**2) / s_ref
            phi = s.half_angle
            if da > 0:
                self._expansions.append((math.sin(phi) ** 2, da))
            else:
                exposed = -da * (1.0 - boat_tail_attached_fraction(phi))
                if s is last:
                    # final boat-tail: only the attached fraction of the area reduction reaches the base
                    self._a_base = (s.d_aft / d) ** 2 + exposed
                else:
                    self._contraction_exposed += exposed

        # fin geometry terms
        self._fin_wet = 2.0 * fins.count * fins.planform_area if fins.count else 0.0
        self._fin_edge_len = fins.count * fins.span if fins.count else 0.0
        self._cos2_le = math.cos(fins.leading_edge_sweep_angle) ** 2 if fins.count else 1.0
        self._fin_tc = (fins.thickness / fins.mean_chord) if fins.count else 0.0

    # -- static stability data ---------------------------------------------------------------------
    @property
    def cn_alpha_total(self) -> float:
        return self._cn_static

    @property
    def x_cp_subsonic(self) -> float:
        return self._x_cp

    # -- drag -------------------------------------------------------------------------------------
    def _drag(self, mach: float, reynolds: float, powered: bool) -> float:
        s_ref = self.ref_area
        re_body = max(reynolds, 1.0)
        cf_b = skin_friction_coefficient(re_body, mach, self.roughness, self._length)
        cd = cf_b * self._form_factor * self._s_wet / s_ref
        fins = self.fins
        if fins.count:
            re_fin = re_body * fins.mean_chord / self._length
            cf_f = skin_friction_coefficient(re_fin, mach, self.roughness, fins.mean_chord)
            cd += cf_f * (1.0 + 2.0 * self._fin_tc) * self._fin_wet / s_ref
            edge = fins.thickness * self._fin_edge_len / s_ref
            cd += edge * self.f_le * stagnation_pressure_coefficient(mach) * self._cos2_le
        cd += (
            0.8
            * math.sin(self._nose_half_angle) ** 2
            * smoothstep((mach - 0.8) / 0.4)
            * self._nose_area_ratio
        )
        if self._expansions:
            cstag = stagnation_pressure_coefficient(mach)
            for sin2, da in self._expansions:
                cd += cstag * sin2 * da  # forward-facing step / shoulder: stagnation-like pressure drag
        # base drag (Hoerner) from the forebody drag; exposed rear-facing areas and fin trailing edges are base-like
        base = self._a_base - (self._a_nozzle if powered else 0.0)
        cdb = base_drag_coefficient(mach, cd)
        cd += cdb * (max(base, 0.0) + self._contraction_exposed)
        if fins.count:
            cd += fins.thickness * self._fin_edge_len / s_ref * self.f_te * cdb
        return cd

    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        cd = self.constant_cd if self.constant_cd is not None else self._drag(mach, reynolds, powered)
        return AeroCoefficients((cd + self.extra_cd) * self.drag_scale, self._cn_static, self._x_cp)


class SimplifiedAero(BarrowmanAero):
    """Constant Cd (user value) with geometry-derived stability and crossflow."""

    name = "simplified"

    def __init__(self, assembly: Assembly, cd: float, nozzle_exit_diameter: float, **kw) -> None:
        if cd < 0:
            raise ConfigError("simplified aero: cd must be >= 0")
        super().__init__(assembly, nozzle_exit_diameter, constant_cd=cd, **kw)


class EnhancedAero(BarrowmanAero):
    """Barrowman + Mach-dependent fin lift slope / CP, fin stall, transonic-supersonic fin wave drag."""

    name = "enhanced"

    def __init__(
        self, assembly: Assembly, nozzle_exit_diameter: float, stall_angle_deg: float = 18.0, **kw
    ) -> None:
        super().__init__(assembly, nozzle_exit_diameter, **kw)
        if not 3.0 <= stall_angle_deg <= 60.0:
            raise ConfigError("enhanced aero: stall_angle_deg must be in [3, 60]")
        self.stall = math.radians(stall_angle_deg)
        fins = self.fins
        self._cn_fixed = sum(c for c, _, k in self._comps if k != "fins")
        self._x_fixed_moment = sum(c * x for c, x, k in self._comps if k != "fins")
        self._x_fin_sup = (fins.leading_edge_from_nose + fin_centroid_chordwise(fins)) if fins.count else 0.0

    def _fin_state(self, mach: float) -> tuple[float, float]:
        """(fin CNa, fin CP) at this Mach."""
        if not self.fins.count:
            return 0.0, 0.0
        g = fin_lift_slope_ratio(mach, self.fins)
        w = smoothstep((mach - 0.8) / 0.4)
        return self._cn_fins * g, self._x_fins * (1.0 - w) + self._x_fin_sup * w

    def control_cn_alpha(self, mach: float) -> float:
        cs = self.assembly.controls
        return fin_lift_slope_ratio(mach, cs[0].fin) if cs else 1.0

    def _wave_drag_fins(self, mach: float) -> float:
        """Ackeret thin-section wave drag of the fins, switched on across M 0.9-1.2 (referenced to S_ref)."""
        fins = self.fins
        if not fins.count or mach <= 0.9:
            return 0.0
        m = max(mach, 1.05)
        beta = math.sqrt(m * m - 1.0)
        cdw = 4.0 * self._fin_tc**2 / beta * (fins.count * fins.planform_area) / self.ref_area
        return cdw * smoothstep((mach - 0.9) / 0.3)

    def coefficients(self, mach: float, reynolds: float, powered: bool) -> AeroCoefficients:
        base = super().coefficients(mach, reynolds, powered)
        cn_f, x_f = self._fin_state(mach)
        cn = self._cn_fixed + cn_f
        x_cp = (self._x_fixed_moment + cn_f * x_f) / cn if cn > 0 else base.x_cp
        cd = base.cd0 + self._wave_drag_fins(mach) * self.drag_scale
        return AeroCoefficients(cd, cn, x_cp)

    def force_coefficients(
        self, mach: float, alpha: float, reynolds: float, powered: bool
    ) -> ForceCoefficients:
        c = self.coefficients(mach, reynolds, powered)
        sa, ca = math.sin(alpha), math.cos(alpha)
        cn_f, x_f = self._fin_state(mach)
        # fin lift stalls: the effective angle saturates smoothly at the stall angle
        a_eff = self.stall * math.tanh(min(alpha, math.pi - alpha) / self.stall)
        fin_term = cn_f * math.sin(a_eff) * abs(math.cos(a_eff))
        fixed_term = self._cn_fixed * sa * abs(ca)
        cn_lin = fixed_term + fin_term
        cn_cf = self.crossflow_cd * self.planform_area / self.ref_area * sa * sa
        cn = cn_lin + cn_cf
        if cn > 1e-12:
            x_lin_moment = self._x_fixed_moment * sa * abs(ca) + x_f * cn_f * math.sin(a_eff) * abs(
                math.cos(a_eff)
            )
            x_cp = (x_lin_moment + cn_cf * self.x_crossflow) / cn
        else:
            x_cp = c.x_cp
        return ForceCoefficients(c.cd0 * ca, cn, x_cp, c.x_cp)


class BuildupAero(BarrowmanAero):
    """V1-compatible constructor: single-diameter body + nose + one fin set (builds a legacy Assembly)."""

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
        super().__init__(
            legacy_assembly(body, nose, fins),
            nozzle_exit_diameter,
            surface_roughness,
            crossflow_cd,
            extra_cd,
            drag_scale,
            fin_leading_edge_factor,
            fin_trailing_edge_factor,
        )
        self.body, self.nose = body, nose


__all__ = [
    "AeroCoefficients",
    "AerodynamicModel",
    "BarrowmanAero",
    "BuildupAero",
    "ConstantAero",
    "ControlFin",
    "ControlSurfaceSet",
    "EnhancedAero",
    "ForceCoefficients",
    "SimplifiedAero",
    "Table2DAero",
    "TableAero",
]
