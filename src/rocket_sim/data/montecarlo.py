"""Monte-Carlo parameter sampling with fully reproducible per-run seeds.

Seed derivation (documented in docs/datasets.md and stored in every manifest):

    ss          = SeedSequence([master_seed, split_namespace, run_index])
    param_rng   = default_rng(ss.spawn(1)[0])          -> draws every parameter (in spec order)
    sim_seed    = ss.generate_state(1, uint64)[0] >> 1  -> passed to Simulation (sensor noise,
                                                           turbulence, ...), stored as random_seed

so any single run can be regenerated from (master_seed, split_namespace, run_index) without
generating its neighbours, independent of worker count, chunking or order. Distinct
``split_namespace`` values give statistically independent, non-overlapping seed streams
(train/val/test), and ``check_seed_disjointness`` verifies no sim_seed is shared.

Distributions (``dist``):
  normal       mean/std, or rel_std (std = rel_std * |base value|; mean defaults to base value)
  truncnormal  as normal plus low/high (resampled until inside; max 1000 tries)
  uniform      low/high, or rel_range (base*(1-r) .. base*(1+r))
  lognormal    median (default: base value) and sigma (log-space std)
  choice       values (and optional weights)
Optional ``low``/``high`` clip any distribution (clipping, not resampling, except truncnormal).
``path`` is a dotted config path; list elements use integer segments (motor.misalignment_deg.0).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..errors import ConfigError

_DISTS = ("normal", "truncnormal", "uniform", "lognormal", "choice")


@dataclass
class ParamSpec:
    path: str
    dist: str = "normal"
    mean: float | None = None
    std: float | None = None
    rel_std: float | None = None
    low: float | None = None
    high: float | None = None
    rel_range: float | None = None
    median: float | None = None
    sigma: float | None = None
    values: list[Any] | None = None
    weights: list[float] | None = None

    def validate(self) -> None:
        p = f"parameters[{self.path}]"
        if self.dist not in _DISTS:
            raise ConfigError(f"{p}: dist must be one of {_DISTS}")
        if self.dist in ("normal", "truncnormal"):
            if (self.std is None) == (self.rel_std is None):
                raise ConfigError(f"{p}: give exactly one of std / rel_std")
            if (self.std is not None and self.std < 0) or (self.rel_std is not None and self.rel_std < 0):
                raise ConfigError(f"{p}: std must be >= 0")
        if self.dist == "truncnormal" and self.low is None and self.high is None:
            raise ConfigError(f"{p}: truncnormal needs low and/or high")
        if self.dist == "uniform":
            has_abs = self.low is not None and self.high is not None
            if has_abs == (self.rel_range is not None):
                raise ConfigError(f"{p}: uniform needs (low and high) or rel_range")
            if has_abs and self.low >= self.high:  # type: ignore[operator]
                raise ConfigError(f"{p}: low must be < high")
        if self.dist == "lognormal" and (self.sigma is None or self.sigma < 0):
            raise ConfigError(f"{p}: lognormal needs sigma >= 0")
        if self.dist == "choice":
            if not self.values:
                raise ConfigError(f"{p}: choice needs values")
            if self.weights is not None and (
                len(self.weights) != len(self.values) or min(self.weights) < 0 or sum(self.weights) <= 0
            ):
                raise ConfigError(f"{p}: weights must match values, be >= 0 and sum > 0")


def get_path(data: dict[str, Any] | list[Any], path: str) -> Any:
    node: Any = data
    for seg in path.split("."):
        try:
            node = node[int(seg)] if isinstance(node, list) else node[seg]
        except (KeyError, IndexError, ValueError) as exc:
            raise ConfigError(f"parameter path '{path}' does not exist in the base config") from exc
    return node


def set_path(data: dict[str, Any], path: str, value: Any) -> None:
    segs = path.split(".")
    node: Any = data
    for seg in segs[:-1]:
        node = node[int(seg)] if isinstance(node, list) else node.setdefault(seg, {})
    last = segs[-1]
    if isinstance(node, list):
        node[int(last)] = value
    else:
        node[last] = value


def sample_param(spec: ParamSpec, base_value: Any, rng: np.random.Generator) -> Any:
    d = spec.dist
    if d == "choice":
        idx = rng.choice(
            len(spec.values or []),
            p=None if spec.weights is None else np.asarray(spec.weights) / sum(spec.weights),
        )
        return (spec.values or [])[int(idx)]
    base = (
        float(base_value)
        if isinstance(base_value, int | float) and not isinstance(base_value, bool)
        else None
    )
    if d in ("normal", "truncnormal"):
        mean = spec.mean if spec.mean is not None else base
        if mean is None:
            raise ConfigError(f"parameters[{spec.path}]: no mean and base value is not numeric")
        std = spec.std if spec.std is not None else spec.rel_std * abs(mean)  # type: ignore[operator]
        if d == "normal":
            x = float(rng.normal(mean, std))
        else:
            lo = -math.inf if spec.low is None else spec.low
            hi = math.inf if spec.high is None else spec.high
            for _ in range(1000):
                x = float(rng.normal(mean, std))
                if lo <= x <= hi:
                    break
            else:
                raise ConfigError(f"parameters[{spec.path}]: truncnormal bounds unreachable")
    elif d == "uniform":
        if spec.rel_range is not None:
            if base is None:
                raise ConfigError(f"parameters[{spec.path}]: rel_range needs a numeric base value")
            lo, hi = base * (1 - spec.rel_range), base * (1 + spec.rel_range)
        else:
            lo, hi = spec.low, spec.high  # type: ignore[assignment]
        x = float(rng.uniform(lo, hi))
    else:  # lognormal
        med = spec.median if spec.median is not None else base
        if med is None or med <= 0:
            raise ConfigError(f"parameters[{spec.path}]: lognormal needs a positive median/base")
        assert spec.sigma is not None
        x = float(med * math.exp(rng.normal(0.0, spec.sigma)))
    if d != "truncnormal":
        if spec.low is not None:
            x = max(x, spec.low)
        if spec.high is not None:
            x = min(x, spec.high)
    return x


@dataclass
class RunSeeds:
    run_index: int
    split_namespace: int
    master_seed: int
    sim_seed: int
    param_rng: np.random.Generator = field(repr=False)


def derive_seeds(master_seed: int, split_namespace: int, run_index: int) -> RunSeeds:
    ss = np.random.SeedSequence([master_seed, split_namespace, run_index])
    param_rng = np.random.default_rng(ss.spawn(1)[0])
    sim_seed = int(ss.generate_state(1, dtype=np.uint64)[0] >> np.uint64(1))
    return RunSeeds(run_index, split_namespace, master_seed, sim_seed, param_rng)


def sample_run(params: list[ParamSpec], base_dict: dict[str, Any], seeds: RunSeeds) -> dict[str, Any]:
    """Draw one value per parameter (in spec order) -> {dotted path: value}."""
    out: dict[str, Any] = {}
    for spec in params:
        base = get_path(base_dict, spec.path)
        out[spec.path] = sample_param(spec, base, seeds.param_rng)
    return out


def check_seed_disjointness(master_seed: int, splits: dict[str, tuple[int, int]]) -> None:
    """Verify sim seeds of all splits are pairwise distinct. ``splits``: name -> (namespace, n)."""
    seen: dict[int, str] = {}
    for name, (ns, n) in splits.items():
        for i in range(n):
            s = derive_seeds(master_seed, ns, i).sim_seed
            if s in seen:
                raise ConfigError(f"seed collision between {seen[s]} and {name} (run {i})")
            seen[s] = f"{name}[{i}]"
