"""Launching, resource paths, the run page and the packaged-layout contract (no real display needed)."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from rocket_sim import resources
from rocket_sim.errors import RocketSimError
from rocket_sim.version import SIM_VERSION

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture
def fresh_roots(monkeypatch):
    """Resource-root resolution is cached; tests that change the environment must start and end with a clean cache."""
    resources._resolve_root.cache_clear()
    yield monkeypatch
    resources._resolve_root.cache_clear()


def _run(*args: str, cwd: Path, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    e = {**os.environ, **(env or {})}
    return subprocess.run(
        [sys.executable, "-m", "rocket_sim", *args],
        cwd=cwd,
        capture_output=True,
        text=True,
        env=e,
        timeout=120,
    )


def test_source_checkout_root_is_the_repository():
    assert resources.resource_root() == REPO
    assert resources.motors_dir() == REPO / "data" / "motors"
    assert resources.default_config().exists()
    assert resources.check_installation() == []
    assert not resources.is_frozen()


def test_frozen_layout_resolves_to_the_bundled_resources(tmp_path, fresh_roots):
    """What build_windows.py produces: <bundle>/resources/{data/motors,configs,vehicles}; no repository anywhere."""
    bundle = tmp_path / "_internal"
    for src, dst in {"data/motors": "data/motors", "configs": "configs", "vehicles": "vehicles"}.items():
        shutil.copytree(REPO / src, bundle / "resources" / dst)
    fresh_roots.setattr(sys, "frozen", True, raising=False)
    fresh_roots.setattr(sys, "_MEIPASS", str(bundle), raising=False)
    fresh_roots.delenv("ROCKETSIM_HOME", raising=False)
    fresh_roots.delenv("ROCKETSIM_WORKSPACE", raising=False)
    fresh_roots.setattr(Path, "home", lambda: tmp_path / "home")
    assert resources.resource_root() == bundle / "resources"
    assert resources.check_installation() == []
    assert (
        resources.workspace_dir() == tmp_path / "home" / "RocketSimulator"
    )  # never inside the (read-only) bundle
    assert resources.output_dir() == tmp_path / "home" / "RocketSimulator" / "output"
    from rocket_sim.ui.catalog import list_motors, list_vehicles

    assert len(list_vehicles()) >= 3 and len(list_motors()) >= 4


def test_incomplete_bundle_gives_an_actionable_message(tmp_path, fresh_roots):
    fresh_roots.setattr(sys, "frozen", True, raising=False)
    fresh_roots.setattr(sys, "_MEIPASS", str(tmp_path), raising=False)
    fresh_roots.delenv("ROCKETSIM_HOME", raising=False)
    (msg,) = resources.check_installation()
    assert "data/motors" in msg and "RocketSimulator.exe" in msg


def test_rocketsim_home_overrides_and_is_validated(tmp_path, fresh_roots):
    fresh_roots.setenv("ROCKETSIM_HOME", str(tmp_path))
    with pytest.raises(resources.ResourceError, match="no data/motors"):
        resources.resource_root()
    (tmp_path / "data" / "motors").mkdir(parents=True)
    resources._resolve_root.cache_clear()
    assert resources.resource_root() == tmp_path.resolve()
    with pytest.raises(resources.ResourceError, match="No motor files were found"):
        resources.motors_dir()  # folder exists but holds no .eng


def test_resource_errors_are_simulator_errors(tmp_path, fresh_roots):
    fresh_roots.setenv("ROCKETSIM_HOME", str(tmp_path))
    with pytest.raises(RocketSimError):
        resources.resource_root()


def test_workspace_override(tmp_path, monkeypatch):
    monkeypatch.setenv("ROCKETSIM_WORKSPACE", str(tmp_path))
    assert resources.output_dir() == tmp_path / "output"


def test_no_module_computes_repository_paths_itself():
    """Path(__file__).parents[n] belongs in resources.py (and the package-internal fingerprint code), nowhere else."""
    offenders = []
    for p in (REPO / "src" / "rocket_sim").rglob("*.py"):
        if p.name == "resources.py" or "validation" in p.parts:
            continue
        text = p.read_text(encoding="utf-8")
        if "__file__" in text and "parents[" in text:
            offenders.append(p.name)
    assert offenders == []


def test_launch_works_from_any_working_directory(tmp_path):
    r = _run("--check", cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    assert "data files: OK" in r.stdout and "PySide6: OK" in r.stdout


def test_cli_commands_still_dispatch_through_the_module_entry(tmp_path):
    r = _run("motors", cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    assert "AeroTech_G80T.eng" in r.stdout


def test_version_has_a_single_source():
    cfg = tomllib.loads((REPO / "pyproject.toml").read_text(encoding="utf-8"))
    assert "version" in cfg["project"]["dynamic"] and "version" not in cfg["project"]
    assert cfg["tool"]["setuptools"]["dynamic"]["version"]["attr"] == "rocket_sim.version.SIM_VERSION"
    r = _run("--version", cwd=REPO)
    assert SIM_VERSION in r.stdout
    import rocket_sim

    assert rocket_sim.__version__ == SIM_VERSION


def test_pyside6_is_a_core_dependency_so_plain_install_launches_the_gui():
    cfg = tomllib.loads((REPO / "pyproject.toml").read_text(encoding="utf-8"))
    assert any(d.lower().startswith("pyside6") for d in cfg["project"]["dependencies"])


def test_missing_data_is_reported_not_traceback(tmp_path):
    empty = tmp_path / "nothing"
    empty.mkdir()
    r = _run("--check", cwd=tmp_path, env={"ROCKETSIM_HOME": str(empty)})
    assert r.returncode != 0
    assert "Traceback" not in r.stderr
    assert "ROCKETSIM_HOME" in r.stderr and "data/motors" in r.stderr


def test_build_script_packages_only_resources_not_the_repository():
    text = (REPO / "scripts" / "build_windows.py").read_text(encoding="utf-8")
    for needed in ("data/motors", "configs", "vehicles", "--onedir", "--windowed", "--version-file"):
        assert needed in text
    for never in ('"tests"', '"docs"', "validation_data", "validation_results"):
        assert never not in text.split("RESOURCES = {", 1)[1].split("}", 1)[0]
    assert (REPO / "packaging" / "rocket_simulator_entry.py").exists()
    # every resource the bundle needs is relative to the resource root in the shipped configs
    cfgs = (REPO / "configs").glob("*.yaml")
    assert all("C:\\" not in c.read_text(encoding="utf-8") for c in cfgs)


def test_vehicle_and_motor_catalog():
    from rocket_sim.ui.catalog import list_motors, list_vehicles

    names = [v.path.name for v in list_vehicles()]
    assert "example_g80.yaml" in names and "example_vehicle_sim.yaml" in names
    assert not any("batch" in n or "dataset" in n or "experiment" in n or "domain" in n for n in names)
    assert not any(n == "example_tvc_demo.json" for n in names)  # already wrapped by a config: listed once
    motors = list_motors()
    assert {m.label for m in motors} >= {"AeroTech_G80T", "AeroTech_K828FJ", "Estes_E16"}
    assert all(m.key.startswith("data/motors/") for m in motors)  # portable: relative to the resource root


@pytest.fixture(scope="module")
def qapp():
    pytest.importorskip("PySide6")
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def window(qapp, tmp_path, monkeypatch):
    from PySide6 import QtWidgets

    monkeypatch.setenv("ROCKETSIM_WORKSPACE", str(tmp_path / "ws"))
    shown: list[str] = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "critical", lambda *a, **k: shown.append(str(a[-1])))
    from rocket_sim.ui.main_window import MainWindow

    w = MainWindow()
    w.dialogs = shown  # type: ignore[attr-defined]
    yield w
    w.close()


def test_window_shows_version_and_how_it_was_launched(window):
    assert SIM_VERSION in window.windowTitle()
    page = window.start_page.toPlainText()
    assert SIM_VERSION in page and "python -m rocket_sim" in page
    assert window.tabs.tabText(0) == "Getting started"


def test_every_listed_vehicle_loads_and_selects_its_own_motor(window):
    for i in range(window.vehicle_box.count()):
        window.vehicle_box.setCurrentIndex(i)
        entry = window.vehicle_box.itemData(i)
        assert window.vehicle_box.currentIndex() == i
        assert window.base_path == entry.path
        assert window.motor_box.currentText()  # the vehicle's own motor, never blank
        assert window.dialogs == [], window.dialogs
    # the vehicle-file based entry names the K828FJ motor of vehicles/example_tvc_demo.json
    window.vehicle_box.setCurrentIndex(
        [v.path.name for v in map(window.vehicle_box.itemData, range(window.vehicle_box.count()))].index(
            "example_vehicle_sim.yaml"
        )
    )
    assert "K828FJ" in window.motor_box.currentText()


def test_run_saves_results_and_shows_where(window, tmp_path):
    window.run_simulation(blocking=True)
    rec = window.record
    assert rec is not None and rec.summary["apogee_m"] > 100
    d = window.results_dir
    assert d is not None and d.parent == tmp_path / "ws" / "output" / "gui_runs"
    assert {p.name for p in d.iterdir()} >= {"telemetry.csv", "summary.txt", "flight_overview.png"}
    assert str(d) in window.results_label.toPlainText()
    assert "Last run" in window.start_page.toPlainText()
    assert "Apogee" in window.summary_text.toPlainText()


def test_choosing_another_motor_changes_the_flight(window):
    window.run_simulation(blocking=True)
    a = window.record.summary["apogee_m"]
    window.motor_box.setCurrentIndex(window.motor_box.findText("Estes_E16"))
    window.run_simulation(blocking=True)
    b = window.record.summary["apogee_m"]
    assert abs(a - b) > 20.0 and window.dialogs == []


def test_simulation_failure_is_a_message_not_a_crash(window):
    window.motor_box.addItem("ghost", "data/motors/does_not_exist.eng")
    window.motor_box.setCurrentIndex(window.motor_box.count() - 1)
    window.run_simulation(blocking=True)
    assert window.run_button.isEnabled()
    assert window.dialogs and "does_not_exist" in window.dialogs[-1] and "Traceback" not in window.dialogs[-1]


def test_bad_vehicle_file_keeps_the_previous_selection(window, tmp_path):
    bad = tmp_path / "broken.yaml"
    bad.write_text("rocket: [this is not a rocket", encoding="utf-8")
    before = window.base_path
    window._load_vehicle(bad)
    assert window.base_path == before
    assert window.dialogs and "broken.yaml" in window.dialogs[-1]


def test_self_test_report(tmp_path, monkeypatch, qapp):
    from rocket_sim.ui.launcher import main

    monkeypatch.setenv("ROCKETSIM_WORKSPACE", str(tmp_path / "ws"))
    out = tmp_path / "report.json"
    assert main(["--self-test", str(out)]) == 0
    rep = json.loads(out.read_text(encoding="utf-8"))
    assert rep["ok"] and rep["version"] == SIM_VERSION and rep["result_files"]
