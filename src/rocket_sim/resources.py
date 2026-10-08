"""Where the simulator's files live: ONE place that knows about source checkouts, installed packages and PyInstaller bundles.

Two different locations matter and must not be confused:

* the **resource root**: read-only defaults that ship with the program (``data/motors``, ``configs``, ``vehicles``).
  - source checkout / editable install: the repository root;
  - PyInstaller build: ``<bundle>/resources`` (copied in by ``scripts/build_windows.py``);
  - ``ROCKETSIM_HOME`` (environment variable) overrides both, for a relocated copy of the data.
* the **workspace**: where the program WRITES (results, experiments, logs).
  - source checkout: the repository root (``output/`` is git-ignored);
  - packaged executable: ``Documents/RocketSimulator`` (a bundle may sit in a read-only folder);
  - ``ROCKETSIM_WORKSPACE`` overrides.

Every other module asks this one; nothing else computes ``Path(__file__).parents[n]``.
"""

from __future__ import annotations

import os
import sys
from functools import lru_cache
from pathlib import Path

from .errors import RocketSimError
from .version import SIM_VERSION

APP_NAME = "Autonomous Rocket Simulator"
_MARKER = ("data", "motors")  # a directory that must exist for a folder to count as a resource root


class ResourceError(RocketSimError, FileNotFoundError):
    """Bundled data (motors, configs, vehicles) could not be found; the message says what to do about it."""


def is_frozen() -> bool:
    """True inside a PyInstaller executable."""
    return bool(getattr(sys, "frozen", False))


def app_version() -> str:
    return SIM_VERSION


def launch_description() -> str:
    """How this process was started, in words a user recognises."""
    if is_frozen():
        return "packaged Windows executable"
    return "Python source (python -m rocket_sim)"


def _is_root(p: Path) -> bool:
    return (p.joinpath(*_MARKER)).is_dir()


def _checkout_root() -> Path | None:
    """The repository root when running from a source checkout or an editable install."""
    cand = Path(__file__).resolve().parents[2]
    return cand if (cand / "pyproject.toml").exists() and _is_root(cand) else None


def _cwd_root() -> Path | None:
    """A source checkout above the working directory (regular, non-editable install run from inside a clone)."""
    here = Path.cwd().resolve()
    for p in (here, *here.parents):
        if (p / "pyproject.toml").exists() and _is_root(p):
            return p
    return None


@lru_cache(maxsize=1)
def _resolve_root() -> Path | None:
    env = os.environ.get("ROCKETSIM_HOME")
    if env:
        p = Path(env).expanduser()
        if not _is_root(p):
            raise ResourceError(
                f"ROCKETSIM_HOME is set to '{p}' but it has no data/motors folder.\n"
                "Point it at a folder containing data/, configs/ and vehicles/, or unset the variable."
            )
        return p.resolve()
    if is_frozen():
        base = Path(getattr(sys, "_MEIPASS", Path(sys.executable).parent))
        for p in (base / "resources", Path(sys.executable).parent / "resources"):
            if _is_root(p):
                return p
        return None
    return _checkout_root() or _cwd_root()


def resource_root() -> Path:
    """Folder holding ``data/``, ``configs/`` and ``vehicles/``. Raises :class:`ResourceError` when there is none."""
    root = _resolve_root()
    if root is None:
        raise ResourceError(
            "Could not find the simulator's data files (expected a 'data/motors' folder).\n"
            + (
                "The installation is incomplete: extract the whole distribution folder and run RocketSimulator.exe from inside it."
                if is_frozen()
                else "Run from a clone of the repository (pip install -e . inside it), or set ROCKETSIM_HOME to the folder that holds data/, configs/ and vehicles/."
            )
        )
    return root


def project_root() -> Path:
    """The resource root, or the working directory when none exists (never raises; callers that need files use the
    specific getters below, which raise with a useful message)."""
    root = _resolve_root()
    return root if root is not None else Path.cwd()


def _subdir(name: str, what: str) -> Path:
    d = resource_root() / name
    if not d.is_dir():
        raise ResourceError(
            f"No {what} folder found.\n\nExpected: {d}\nCheck that the repository's data files are installed correctly."
        )
    return d


def motors_dir() -> Path:
    d = _subdir("data", "data") / "motors"
    if not d.is_dir() or not any(d.glob("*.eng")):
        raise ResourceError(f"No motor files were found.\n\nExpected motor data (*.eng) in:\n{d}")
    return d


def configs_dir() -> Path:
    return _subdir("configs", "configuration")


def vehicles_dir() -> Path:
    return _subdir("vehicles", "vehicle")


def default_config() -> Path:
    p = configs_dir() / "example_g80.yaml"
    if not p.exists():
        raise ResourceError(
            f"Could not find the default simulation config:\n{p}\n\nCheck that the repository's data files are installed correctly."
        )
    return p


def workspace_dir() -> Path:
    """Writable folder for results, experiments and logs (created on demand by the caller)."""
    env = os.environ.get("ROCKETSIM_WORKSPACE")
    if env:
        return Path(env).expanduser()
    if is_frozen():
        docs = Path.home() / "Documents"
        return (docs if docs.is_dir() else Path.home()) / "RocketSimulator"
    return project_root()


def output_dir() -> Path:
    return workspace_dir() / "output"


def check_installation() -> list[str]:
    """Problems that would stop the simulator from running, as readable sentences (empty list: all good)."""
    problems: list[str] = []
    try:
        motors_dir()
        configs_dir()
        vehicles_dir()
        default_config()
    except ResourceError as exc:
        problems.append(str(exc))
    return problems
