"""Load, override, validate and fingerprint simulation configs (YAML / JSON / TOML)."""

from __future__ import annotations

import copy
import hashlib
import json
import tomllib
from dataclasses import asdict
from pathlib import Path
from typing import Any

import yaml

from ..errors import ConfigError
from .schema import SimConfig, from_dict

PROJECT_ROOT = Path(__file__).resolve().parents[3]


def read_mapping(path: str | Path) -> dict[str, Any]:
    """Read a YAML/JSON/TOML file into a plain dict."""
    p = Path(path)
    if not p.exists():
        raise ConfigError(f"config file not found: {p}")
    ext = p.suffix.lower()
    try:
        text = p.read_text(encoding="utf-8")
        if ext in (".yaml", ".yml"):
            data = yaml.safe_load(text)
        elif ext == ".json":
            data = json.loads(text)
        elif ext == ".toml":
            data = tomllib.loads(text)
        else:
            raise ConfigError(f"unsupported config extension '{ext}' (use .yaml/.json/.toml)")
    except (yaml.YAMLError, json.JSONDecodeError, tomllib.TOMLDecodeError) as exc:
        raise ConfigError(f"{p}: cannot parse: {exc}") from exc
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ConfigError(f"{p}: top level must be a mapping")
    return data


def apply_overrides(data: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    """Return a deep copy of ``data`` with dotted-path overrides applied.

    List elements are addressed by integer segments (``motor.misalignment_deg.0``).
    """
    out = copy.deepcopy(data)
    for dotted, value in overrides.items():
        keys = dotted.split(".")
        node: Any = out
        for k in keys[:-1]:
            if isinstance(node, list):
                try:
                    node = node[int(k)]
                except (ValueError, IndexError) as exc:
                    raise ConfigError(f"override '{dotted}': bad list index '{k}'") from exc
                continue
            nxt = node.get(k)
            if nxt is None:
                nxt = {}
                node[k] = nxt
            if not isinstance(nxt, dict | list):
                raise ConfigError(f"override '{dotted}': '{k}' is not a section")
            node = nxt
        last = keys[-1]
        if isinstance(node, list):
            try:
                node[int(last)] = value
            except (ValueError, IndexError) as exc:
                raise ConfigError(f"override '{dotted}': bad list index '{last}'") from exc
        else:
            node[last] = value
    return out


def config_from_dict(
    data: dict[str, Any], base_dir: str | Path | None = None, overrides: dict[str, Any] | None = None
) -> SimConfig:
    """Parse + validate a config dict. ``base_dir`` resolves relative file paths."""
    if overrides:
        data = apply_overrides(data, overrides)
    cfg = from_dict(SimConfig, data)
    cfg.validate()
    cfg_base = Path(base_dir) if base_dir is not None else Path.cwd()
    object.__setattr__(cfg, "_base_dir", str(cfg_base))
    return cfg


def load_config(path: str | Path, overrides: dict[str, Any] | None = None) -> SimConfig:
    p = Path(path)
    return config_from_dict(read_mapping(p), base_dir=p.parent, overrides=overrides)


def config_to_dict(cfg: SimConfig) -> dict[str, Any]:
    return asdict(cfg)


def config_hash(cfg: SimConfig | dict[str, Any]) -> str:
    """Stable SHA-256 (first 16 hex chars) of the canonical JSON form of a config."""
    data = config_to_dict(cfg) if isinstance(cfg, SimConfig) else cfg
    blob = json.dumps(data, sort_keys=True, default=str, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()[:16]


def resolve_path(cfg: SimConfig, file: str) -> Path:
    """Resolve a config-relative path: absolute, then config dir, then project root."""
    p = Path(file)
    if p.is_absolute():
        return p
    base = Path(getattr(cfg, "_base_dir", "."))
    for candidate in (base / p, PROJECT_ROOT / p, Path.cwd() / p):
        if candidate.exists():
            return candidate
    raise ConfigError(f"file '{file}' not found (searched {base}, {PROJECT_ROOT}, {Path.cwd()})")
