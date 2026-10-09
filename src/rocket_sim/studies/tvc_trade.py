"""Q-01 trade study: which motor suits a TVC version of a vehicle (rail exit speed, thrust-to-weight, gimbal torque and authority
time, closed-loop Monte Carlo success rate). Definitions and caveats: docs/derivations/tvc_authority.md.

Everything physical comes from a study file; fields set to ``null`` there are required and the study refuses to run until they are
filled in (no defaults are invented for masses, servo parameters, dispersions or limits).
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ..config import apply_overrides, config_from_dict, config_to_dict, load_config
from ..constants import G0
from ..errors import ConfigError
from ..simulation import Simulation
from ..simulation.record import FlightRecord


def max_correctable_torque(thrust_n: Any, lever_arm_m: Any, delta_max_rad: float) -> Any:
    """TODO(you): the largest torque about the CG that the gimbal can apply [N m], from the thrust ``thrust_n`` [N] (scalar or
    numpy array), the lever arm ``lever_arm_m`` [m] from the CG to the gimbal pivot (the nozzle exit) and the mechanical
    limit ``delta_max_rad`` [rad].

    Derive it in docs/derivations/tvc_authority.md (section 1); check it against the hand-calculated value in
    tests/test_tvc_trade.py (``pytest --runxfail -m user_todo`` shows that test failing until this is implemented).
    """
    raise NotImplementedError("max_correctable_torque: see the TODO in rocket_sim/studies/tvc_trade.py")


# --------------------------------------------------------------------------------------------- study file
REQUIRED = (
    "base_config",
    "motors",
    "added_masses.*.mass_kg",
    "added_masses.*.position_from_nose_m",
    "tvc.max_angle_deg",
    "tvc.max_rate_deg_s",
    "tvc.time_constant_s",
    "tvc.delay_s",
    "disturbances.wind_speed_mean_ms",
    "disturbances.wind_speed_std_ms",
    "disturbances.thrust_misalignment_deg",
    "disturbances.cg_error_m",
    "disturbances.cg_shift_item",
    "success.max_tilt_deg",
)


@dataclass
class StudySpec:
    base_config: str
    motors: list[str]
    added_masses: list[dict[str, Any]]
    tvc: dict[str, Any]
    disturbances: dict[str, Any]
    success: dict[str, Any]
    controller: dict[str, Any] = field(default_factory=lambda: {"wn": 8.0, "zeta": 0.8})
    estimator: str = "nav_kf"
    monte_carlo: dict[str, Any] = field(default_factory=lambda: {"runs": 40, "seed": 1})
    reference_disturbance_torque_nm: float | None = (
        None  # optional: time with torque margin >= 1 is reported when set
    )
    settle_s: float = 1.0
    useful_thrust_fraction: float = (
        0.1  # the smallest torque is taken while thrust is above this fraction of its peak
    )
    base_dir: Path = Path(".")

    @property
    def runs(self) -> int:
        return int(self.monte_carlo["runs"])


def _missing(d: Any, path: str, prefix: str = "") -> list[str]:
    """The dotted paths in ``path`` that are absent, null or empty in ``d``."""
    head, _, tail = path.partition(".")
    here = f"{prefix}{head}"
    if (
        not isinstance(d, dict)
        or head not in d
        or d[head] is None
        or (isinstance(d[head], list) and not d[head])
    ):
        return [here + ("." + tail if tail else "")]
    return _missing(d[head], tail, here + ".") if tail else []


def load_study(path: str | Path) -> StudySpec:
    p = Path(path)
    raw = yaml.safe_load(p.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ConfigError(f"{p}: a study file is a mapping")
    known = {f for f in StudySpec.__dataclass_fields__ if f != "base_dir"}
    unknown = sorted(set(raw) - known)
    if unknown:
        raise ConfigError(f"{p}: unknown study keys {unknown}")
    missing = []
    for req in REQUIRED:
        if req.startswith("added_masses.*"):
            masses = raw.get("added_masses")
            if not masses:
                missing.append("added_masses (at least one: the gimbal)")
                continue
            for i, m in enumerate(masses):
                missing += [
                    f"added_masses[{i}].{k}" for k in ("mass_kg", "position_from_nose_m") if m.get(k) is None
                ]
            continue
        missing += _missing(raw, req)
    if missing:
        raise ConfigError(
            f"{p}: the study needs values for {sorted(set(missing))}.\n"
            "Fill them in with your own numbers (masses from the CAD report, servo parameters from the bench fit); "
            "none is guessed."
        )
    spec = StudySpec(**{k: v for k, v in raw.items() if k in known}, base_dir=p.parent)
    if spec.estimator not in ("nav_kf", "ekf"):
        raise ConfigError("study: estimator must be nav_kf or ekf")
    if spec.runs < 1:
        raise ConfigError("study: monte_carlo.runs must be >= 1")
    names = [m.get("name") for m in spec.added_masses]
    if spec.disturbances["cg_shift_item"] not in names and spec.disturbances["cg_shift_item"] != "none":
        # the item may also be a mass already in the base vehicle: checked when the configuration is built
        pass
    return spec


# --------------------------------------------------------------------------------------------- building runs
def _resolve(spec: StudySpec, ref: str) -> Path:
    p = Path(ref)
    return p if p.is_absolute() else (spec.base_dir / p if (spec.base_dir / p).exists() else Path.cwd() / p)


def base_dict(spec: StudySpec) -> tuple[dict[str, Any], Path]:
    path = _resolve(spec, spec.base_config)
    cfg = load_config(path)
    return config_to_dict(cfg), path.parent


def configure(spec: StudySpec, base: dict[str, Any], motor: str, *, control: bool = True) -> dict[str, Any]:
    """The base vehicle plus the added masses, the servo model, the estimator and the controller, on ``motor``."""
    d = copy.deepcopy(base)
    d.setdefault("rocket", {}).setdefault("masses", [])
    for m in spec.added_masses:
        d["rocket"]["masses"].append({k: v for k, v in m.items()})
    d["motor"]["file"] = motor
    d["motor"]["designation"] = None
    d["tvc"].update({k: v for k, v in spec.tvc.items() if v is not None})
    d["fidelity"] = 5
    d["estimator"] = {**d.get("estimator", {}), "type": spec.estimator}
    d["estimator"].setdefault("alignment_time_s", 1.0)
    d["motor"]["ignition_delay_s"] = max(
        float(d["motor"].get("ignition_delay_s", 0.0)), float(d["estimator"]["alignment_time_s"]) + 0.5
    )
    d["controller"] = (
        {
            "type": "tvc_attitude",
            "rate_hz": 100.0,
            "state_source": "auto",
            "params": {**spec.controller, "target": "vertical"},
        }
        if control
        else {"type": "none"}
    )
    if not control:
        d["estimator"]["type"] = "none"
    return d


def run_cfg(
    d: dict[str, Any], base_dir: Path, seed: int, t_end: float | None = None
) -> tuple[Simulation, FlightRecord]:
    if t_end is not None:
        d = apply_overrides(d, {"simulation.t_max_s": t_end})
    sim = Simulation(config_from_dict(d, base_dir=base_dir), seed=seed)
    return sim, sim.run()


def disturb(
    spec: StudySpec, d: dict[str, Any], rng: np.random.Generator, liftoff_mass_kg: float
) -> tuple[dict[str, Any], dict[str, float]]:
    """One draw of the disturbances: wind speed and direction, thrust misalignment (fixed size, random direction) and a CG shift
    (a chosen mass is moved so that the whole vehicle's CG moves by +/- ``cg_error_m``)."""
    dist = spec.disturbances
    out = copy.deepcopy(d)
    wind = max(float(rng.normal(dist["wind_speed_mean_ms"], dist["wind_speed_std_ms"])), 0.0)
    direction = float(rng.uniform(0.0, 360.0))
    out["environment"]["wind"].update(
        {"model": "constant", "speed_ms": wind, "direction_from_deg": direction}
    )
    ang = float(rng.uniform(0.0, 2.0 * math.pi))
    mis = float(dist["thrust_misalignment_deg"])
    out["motor"]["misalignment_deg"] = [mis * math.cos(ang), mis * math.sin(ang)]
    shift = float(dist["cg_error_m"]) * (1.0 if rng.random() < 0.5 else -1.0)
    item = dist["cg_shift_item"]
    if item != "none":
        masses = out["rocket"]["masses"]
        hit = [m for m in masses if m.get("name") == item]
        if not hit:
            raise ConfigError(
                f"study: cg_shift_item {item!r} is not a mass of the vehicle (have {[m.get('name') for m in masses]})"
            )
        m = hit[0]
        m["position_from_nose_m"] = float(m["position_from_nose_m"]) + shift * liftoff_mass_kg / float(
            m["mass_kg"]
        )
    return out, {"wind_ms": wind, "wind_dir_deg": direction, "misalignment_deg": mis, "cg_shift_m": shift}


# --------------------------------------------------------------------------------------------- metrics
def _num(summary: dict[str, Any], key: str) -> float | None:
    """A numeric summary entry as a float, or None."""
    v = summary.get(key)
    return float(v) if isinstance(v, int | float) and not isinstance(v, bool) else None


def tilt_deg(rec: FlightRecord) -> np.ndarray:
    return 90.0 - np.degrees(rec.col("pitch"))  # pitch is the nose elevation above the horizon


def burn_window(rec: FlightRecord, settle_s: float) -> np.ndarray:
    ev = {e.name: e.t for e in rec.events}
    t0 = ev.get("rail_exit")
    t1 = ev.get("burnout")
    if t0 is None or t1 is None:
        return np.zeros(len(rec.col("t")), dtype=bool)
    t = rec.col("t")
    return (t >= t0) & (t <= t1 + settle_s)


def success(spec: StudySpec, rec: FlightRecord) -> dict[str, Any]:
    w = burn_window(rec, spec.settle_s)
    if not w.any():
        return {
            "ok": False,
            "reason": "no_rail_exit_or_burnout",
            "max_tilt_deg": float("nan"),
            "rail_exit_ms": float("nan"),
        }
    tilt = float(np.max(tilt_deg(rec)[w]))
    v_rail = _num(rec.summary, "rail_exit_velocity_ms")
    reasons = []
    if not tilt <= float(spec.success["max_tilt_deg"]):
        reasons.append("tilt")
    vmin = spec.success.get("min_rail_exit_speed_ms")
    if vmin is not None and (v_rail is None or v_rail < float(vmin)):
        reasons.append("rail_exit_speed")
    return {
        "ok": not reasons,
        "reason": "+".join(reasons),
        "max_tilt_deg": tilt,
        "rail_exit_ms": float(v_rail or float("nan")),
    }


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for a success count."""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, centre - half), min(1.0, centre + half))


@dataclass
class MotorResult:
    motor: str
    rail_exit_speed_ms: float | None
    thrust_to_weight: float
    burn_time_s: float
    authority_time_s: float
    t: np.ndarray  # nominal run, seconds
    thrust: np.ndarray
    lever_arm: np.ndarray
    max_torque: np.ndarray | None  # N m, None until max_correctable_torque is implemented
    torque_min_nm: float | None
    torque_at_rail_exit_nm: float | None
    max_ang_accel: float | None  # rad/s^2 at the weakest point of the burn
    time_with_margin_s: float | None
    successes: int
    runs: int
    passive_successes: int
    failures: dict[str, int]
    tilts: np.ndarray
    passive_tilts: np.ndarray
    winds: np.ndarray
    ok: np.ndarray

    @property
    def rate(self) -> float:
        return self.successes / max(self.runs, 1)

    @property
    def interval(self) -> tuple[float, float]:
        return wilson(self.successes, self.runs)


def evaluate_motor(
    spec: StudySpec, base: dict[str, Any], base_dir: Path, motor: str, progress: Any = None
) -> MotorResult:
    nominal_cfg = configure(spec, base, motor, control=True)
    nominal_cfg["environment"]["wind"] = {"model": "none"}
    sim, rec = run_cfg(nominal_cfg, base_dir, seed=0)
    delta = math.radians(float(spec.tvc["max_angle_deg"]))
    ev = {e.name: e.t for e in rec.events}
    t_end = float(ev.get("burnout", rec.col("t")[-1])) + spec.settle_s + 0.5
    m0 = float(rec.col("mass")[0])
    w = (rec.col("t") >= ev.get("rail_exit", 0.0)) & (rec.col("t") <= ev.get("burnout", 0.0))
    thrust = rec.col("thrust")
    lever = np.abs(sim.vehicle.nozzle_x - rec.col("cg"))
    try:
        tau = np.asarray(max_correctable_torque(thrust, lever, delta), dtype=float)
    except NotImplementedError:
        tau = None
    tau_min = tau_rail = accel = margin_t = None
    if tau is not None and w.any():
        useful = thrust[w] >= spec.useful_thrust_fraction * float(thrust[w].max())
        tau_min = float(np.min(tau[w][useful]))
        tau_rail = float(tau[w][0])
        accel = float(np.min((tau[w] / np.maximum(rec.col("iyy")[w], 1e-12))[useful]))
        ref = spec.reference_disturbance_torque_nm
        if ref is not None:
            tt = rec.col("t")
            margin_t = float(np.sum(np.diff(tt[w], prepend=tt[w][0])[tau[w] >= ref]))
    rng = np.random.default_rng(int(spec.monte_carlo["seed"]))
    tilts, ptilts, winds, oks = [], [], [], []
    fails: dict[str, int] = {}
    passive = 0
    for i in range(spec.runs):
        dcfg, draw = disturb(spec, configure(spec, base, motor, control=True), rng, m0)
        try:
            _, r = run_cfg(dcfg, base_dir, seed=1000 + i, t_end=t_end)
        except ConfigError as exc:
            if "position_from_nose_m" not in str(exc):
                raise
            raise ConfigError(
                f"study: a CG error of {spec.disturbances['cg_error_m']} m cannot be made by moving '{spec.disturbances['cg_shift_item']}': "
                f"it would move that mass out of the airframe ({exc}). Use a heavier cg_shift_item or a smaller cg_error_m."
            ) from exc
        s = success(spec, r)
        tilts.append(s["max_tilt_deg"])
        winds.append(draw["wind_ms"])
        oks.append(s["ok"])
        if not s["ok"]:
            fails[s["reason"] or "unknown"] = fails.get(s["reason"] or "unknown", 0) + 1
        pcfg = copy.deepcopy(dcfg)
        pcfg["controller"] = {"type": "none"}
        pcfg["estimator"]["type"] = "none"
        _, rp = run_cfg(pcfg, base_dir, seed=1000 + i, t_end=t_end)  # the same disturbance, no controller
        sp = success(spec, rp)
        passive += int(sp["ok"])
        ptilts.append(sp["max_tilt_deg"])
        if progress:
            progress(i + 1, spec.runs)
    return MotorResult(
        motor=Path(motor).stem,
        rail_exit_speed_ms=_num(rec.summary, "rail_exit_velocity_ms"),
        thrust_to_weight=float(thrust.max()) / (m0 * G0),  # peak thrust over the liftoff weight
        burn_time_s=float(ev.get("burnout", float("nan")) - ev.get("liftoff", 0.0)),
        authority_time_s=float(ev.get("burnout", float("nan")) - ev.get("rail_exit", float("nan"))),
        t=rec.col("t")[w],
        thrust=thrust[w],
        lever_arm=lever[w],
        max_torque=None if tau is None else tau[w],
        torque_min_nm=tau_min,
        torque_at_rail_exit_nm=tau_rail,
        max_ang_accel=accel,
        time_with_margin_s=margin_t,
        successes=int(sum(oks)),
        runs=spec.runs,
        passive_successes=passive,
        failures=fails,
        tilts=np.array(tilts),
        passive_tilts=np.array(ptilts),
        winds=np.array(winds),
        ok=np.array(oks),
    )


def run_study(spec: StudySpec, progress: Any = None) -> list[MotorResult]:
    base, base_dir = base_dict(spec)
    return [evaluate_motor(spec, base, base_dir, m, progress) for m in spec.motors]
