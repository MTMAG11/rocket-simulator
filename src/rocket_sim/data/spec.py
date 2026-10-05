"""Batch / dataset specification (YAML) -> typed dataclasses.

Example (see configs/batch_example.yaml and configs/dataset_example.yaml)::

    name: demo
    base_config: configs/example_g80.yaml
    master_seed: 2024
    shard_runs: 50
    workers: 0                      # 0 = all but one CPU core, 1 = serial
    overrides: {fidelity: 2}        # applied to the base config
    parameters:
      - {path: rocket.dry_mass_kg, dist: normal, rel_std: 0.03}
      - {path: environment.wind.speed_ms, dist: uniform, low: 0, high: 8}
    splits:
      train: {runs: 800}
      val:   {runs: 100}
      test:  {runs: 100, parameters: [...]}   # different distributions = true hold-out
    dataset:
      kind: windowed                # telemetry | tabular | windowed
      sample_dt_s: 0.02
      inputs:  [{column: meas_accel_x}, {derived: time_since_launch_detected}]
      targets: [{column: vel_z}]
      window: {history: 50, stride: 5, target: future, horizon_s: 0.5}
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..errors import ConfigError
from .montecarlo import ParamSpec
from .quality import QualityLimits

DERIVED_FEATURES = ("time_since_ignition", "time_since_launch_detected")


@dataclass
class FeatureSpec:
    column: str | None = None
    derived: str | None = None
    lag: int = 0  # use the value `lag` resampled steps in the past (inputs only)
    name: str | None = None

    def validate(self, where: str) -> None:
        if (self.column is None) == (self.derived is None):
            raise ConfigError(f"{where}: give exactly one of column / derived")
        if self.derived is not None and self.derived not in DERIVED_FEATURES:
            raise ConfigError(f"{where}: derived must be one of {DERIVED_FEATURES}")
        if self.lag < 0:
            raise ConfigError(f"{where}: lag must be >= 0")

    @property
    def label(self) -> str:
        if self.name:
            return self.name
        base = self.column or self.derived or "?"
        return f"{base}_lag{self.lag}" if self.lag else base


@dataclass
class WindowSpec:
    history: int = 50  # samples per input window
    stride: int = 5  # step between window ends (samples)
    target: str = "current"  # current | future | trajectory
    horizon_s: float = 0.5  # future: target time = window end + horizon
    trajectory_steps: int = 10  # trajectory: next K samples


@dataclass
class DatasetSection:
    kind: str = "telemetry"  # telemetry | tabular | windowed
    sample_dt_s: float | None = None
    inputs: list[FeatureSpec] = field(default_factory=list)
    targets: list[FeatureSpec] = field(default_factory=list)
    window: WindowSpec = field(default_factory=WindowSpec)
    trim: str = "flight"  # none | flight (liftoff -> landing)


@dataclass
class SplitSpec:
    runs: int = 100
    namespace: int | None = None  # seed namespace; default = position in the split list
    parameters: list[ParamSpec] | None = None  # replaces the global list (hold-out distributions)

    @property
    def ns(self) -> int:
        """Resolved seed namespace (``split_table`` assigns one to every split)."""
        assert self.namespace is not None
        return self.namespace


@dataclass
class QualitySpec:
    max_speed_ms: float = 3000.0
    max_altitude_m: float = 150_000.0
    min_apogee_m: float = 1.0
    max_rate_rad_s: float = 1000.0
    require_landing: bool = True
    allowed_status: list[str] = field(default_factory=lambda: ["ok"])

    def limits(self, required: tuple[str, ...] = ()) -> QualityLimits:
        return QualityLimits(
            max_speed_ms=self.max_speed_ms,
            max_altitude_m=self.max_altitude_m,
            min_apogee_m=self.min_apogee_m,
            max_rate_rad_s=self.max_rate_rad_s,
            require_landing=self.require_landing,
            allowed_status=tuple(self.allowed_status),
            required_columns=required,
        )


@dataclass
class BatchSpec:
    name: str = "batch"
    base_config: str | None = None
    config: dict[str, Any] | None = None
    overrides: dict[str, Any] = field(default_factory=dict)
    master_seed: int = 0
    runs: int | None = None
    splits: dict[str, SplitSpec] = field(default_factory=dict)
    parameters: list[ParamSpec] = field(default_factory=list)
    shard_runs: int = 50
    workers: int = 0
    dataset: DatasetSection = field(default_factory=DatasetSection)
    quality: QualitySpec = field(default_factory=QualitySpec)
    output_dir: str = "output/batch"

    def validate(self) -> None:
        if (self.base_config is None) == (self.config is None):
            raise ConfigError("batch spec: give exactly one of base_config / config")
        if self.runs is None and not self.splits:
            raise ConfigError("batch spec: give 'runs' or 'splits'")
        if self.runs is not None and self.splits:
            raise ConfigError("batch spec: give either 'runs' or 'splits', not both")
        if self.runs is not None and self.runs < 1:
            raise ConfigError("batch spec: runs must be >= 1")
        if self.shard_runs < 1:
            raise ConfigError("batch spec: shard_runs must be >= 1")
        if self.workers < 0:
            raise ConfigError("batch spec: workers must be >= 0")
        for p in self.parameters:
            p.validate()
        for nm, sp in self.splits.items():
            if sp.runs < 1:
                raise ConfigError(f"splits.{nm}: runs must be >= 1")
            for p in sp.parameters or []:
                p.validate()
        ds = self.dataset
        if ds.kind not in ("telemetry", "tabular", "windowed"):
            raise ConfigError("dataset.kind must be telemetry|tabular|windowed")
        if ds.kind != "telemetry":
            if not ds.inputs or not ds.targets:
                raise ConfigError("dataset: tabular/windowed kinds need inputs and targets")
            for i, f in enumerate(ds.inputs):
                f.validate(f"dataset.inputs[{i}]")
            for i, f in enumerate(ds.targets):
                f.validate(f"dataset.targets[{i}]")
                if f.derived is not None or f.lag:
                    raise ConfigError(f"dataset.targets[{i}]: targets must be plain columns without lag")
            for what, lst in (("inputs", ds.inputs), ("targets", ds.targets)):
                names = [f.label for f in lst]
                if len(set(names)) != len(names):
                    raise ConfigError(f"dataset.{what}: duplicate names (use 'name:' or different lags)")
        if ds.sample_dt_s is not None and ds.sample_dt_s <= 0:
            raise ConfigError("dataset.sample_dt_s must be > 0")
        w = ds.window
        if ds.kind == "windowed":
            if ds.sample_dt_s is None:
                raise ConfigError("dataset.sample_dt_s is required for windowed datasets")
            if w.history < 1 or w.stride < 1:
                raise ConfigError("window.history and window.stride must be >= 1")
            if w.target not in ("current", "future", "trajectory"):
                raise ConfigError("window.target must be current|future|trajectory")
            if w.target == "future" and w.horizon_s <= 0:
                raise ConfigError("window.horizon_s must be > 0")
            if w.target == "trajectory" and w.trajectory_steps < 1:
                raise ConfigError("window.trajectory_steps must be >= 1")
        if ds.trim not in ("none", "flight"):
            raise ConfigError("dataset.trim must be none|flight")

    def split_table(self) -> dict[str, SplitSpec]:
        """Resolved splits with namespaces assigned (a plain ``runs`` count becomes 'train')."""
        splits = dict(self.splits) if self.splits else {"train": SplitSpec(runs=self.runs or 0)}
        out = {}
        for i, (nm, sp) in enumerate(splits.items()):
            out[nm] = SplitSpec(sp.runs, sp.namespace if sp.namespace is not None else i, sp.parameters)
        spaces = [s.namespace for s in out.values()]
        if len(set(spaces)) != len(spaces):
            raise ConfigError("splits must use distinct seed namespaces")
        return out
