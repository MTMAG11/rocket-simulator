"""Beginner-friendly front end: bundled rockets, plain-language text, help panel, and a sidebar that is not clipped."""

from __future__ import annotations

import pytest

from rocket_sim.config import load_config
from rocket_sim.motor import load_motor
from rocket_sim.resources import configs_dir
from rocket_sim.simulation import Simulation
from rocket_sim.ui import friendly
from rocket_sim.ui.catalog import check_fit, list_motors, list_vehicles, motor_path, power_word, rocket_text
from rocket_sim.ui.help_content import ANCHORS, TOPICS

STARTERS = (
    "rocket_mini", "rocket_backyard", "rocket_three_inch", "rocket_high_power", "rocket_needle", "rocket_stubby",
)


@pytest.mark.parametrize("stem", STARTERS)
def test_bundled_rockets_fly_land_and_are_stable(stem):
    cfg = load_config(configs_dir() / f"{stem}.yaml")
    rec = Simulation(cfg, seed=1).run()
    s = rec.summary
    assert 50 < s["apogee_m"] < 3000
    assert s["max_mach"] < 0.9
    assert s.get("impact_speed_ms") is not None, "must land before t_max"
    assert not [w for w in rec.meta.warnings if "UNSTABLE" in w or "static margin" in w]


def test_catalog_groups_and_titles_every_listed_rocket():
    vs = list_vehicles()
    names = {v.path.stem: v for v in vs}
    for stem in STARTERS:
        assert names[stem].level == "starter" and names[stem].description
    assert names["example_tvc_closed_loop"].level == "advanced"
    levels = [v.level for v in vs]
    assert levels == sorted(levels, key=("starter", "advanced", "other").index)


def test_fit_check_blocks_impossible_launches_and_passes_the_default():
    mini = load_config(configs_dir() / "rocket_mini.yaml")
    from rocket_sim.config import config_to_dict

    d = config_to_dict(mini)
    k = load_motor(motor_path("data/motors/AeroTech_K828FJ.eng"))
    a6 = load_motor(motor_path("data/motors/Klima_A6.eng"))
    assert check_fit(d, k)[0] == "bad"
    assert check_fit(d, a6)[0] in ("good", "warn")
    assert "mm wide" in rocket_text(d)
    assert power_word("A") != power_word("K")


def test_every_motor_has_a_plain_summary():
    for m in list_motors():
        assert "engine" in m.summary


def test_flight_story_is_plain_language():
    rec = Simulation(load_config(configs_dir() / "rocket_backyard.yaml"), seed=1).run()
    text = " ".join(friendly.flight_story(rec))
    assert "climbed to" in text and "landed" in text and "ft" in text


def test_friendly_helpers():
    assert friendly.compass(0) == "north" and friendly.compass(270) == "west" and friendly.compass(350) == "north"
    assert friendly.phase_word("COAST") == "Coasting upward"
    assert "unbalanced" in friendly.friendly_warnings(["rocket is statically UNSTABLE (margin -0.2)"])[0]
    msg = friendly.friendly_error("MotorError: motor does not fit in the body")
    assert msg.startswith("This engine does not fit") and "MotorError" in msg
    assert friendly.friendly_error("something odd") == "something odd"


def test_help_topics_are_complete_and_chips_point_at_real_topics():
    assert len(ANCHORS) == len(set(ANCHORS)) == len(TOPICS)
    for _anchor, title, body in TOPICS:
        assert title and len(body) > 40


@pytest.fixture
def window(tmp_path, monkeypatch):
    pytest.importorskip("PySide6")
    from PySide6 import QtWidgets

    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setenv("ROCKETSIM_WORKSPACE", str(tmp_path / "ws"))
    monkeypatch.setattr(QtWidgets.QMessageBox, "critical", lambda *a, **k: None)
    monkeypatch.setattr(QtWidgets.QMessageBox, "information", lambda *a, **k: None)
    from rocket_sim.ui.main_window import MainWindow

    w = MainWindow()
    yield w
    w.close()


def test_help_panel_opens_jumps_and_f1_toggles(window):
    window.show()
    assert not window.help_dock.isVisible()
    window.toggle_help()
    assert window.help_dock.isVisible()
    window.show_help("engine")
    assert window.help_topics.currentData() == "engine"
    window.toggle_help()
    assert not window.help_dock.isVisible()
    from PySide6 import QtWidgets

    chips = [b for b in window.findChildren(QtWidgets.QToolButton) if b.objectName() == "help_chip"]
    assert len(chips) >= 4


def test_sidebar_bottom_is_fully_reachable(window):
    from PySide6 import QtWidgets

    window.resize(1500, 600)  # a short window forces scrolling
    window.show()
    QtWidgets.QApplication.processEvents()
    sc = window.sidebar_scroll
    sc.verticalScrollBar().setValue(sc.verticalScrollBar().maximum())
    QtWidgets.QApplication.processEvents()
    last = window.adv_toggle
    bottom = last.mapTo(sc.viewport(), last.rect().bottomLeft()).y()
    assert bottom <= sc.viewport().height() - 8, "the last entry is cut off"
    window.adv_toggle.setChecked(True)  # opening the expert card must stay reachable too
    QtWidgets.QApplication.processEvents()
    sc.verticalScrollBar().setValue(sc.verticalScrollBar().maximum())
    QtWidgets.QApplication.processEvents()
    inner = window.adv_inner
    assert inner.mapTo(sc.viewport(), inner.rect().bottomLeft()).y() <= sc.viewport().height() - 8


def test_too_big_engine_is_refused_with_a_message(window, monkeypatch):
    from PySide6 import QtWidgets

    shown: list[str] = []
    monkeypatch.setattr(QtWidgets.QMessageBox, "information", lambda *a, **k: shown.append(str(a[-1])))
    idx = next(i for i in range(window.vehicle_box.count()) if getattr(window.vehicle_box.itemData(i), "path", None) and window.vehicle_box.itemData(i).path.stem == "rocket_mini")
    window.vehicle_box.setCurrentIndex(idx)
    window.motor_box.setCurrentIndex(window.motor_box.findText("AeroTech K828FJ"))
    assert window.fit_level == "bad"
    window.run_simulation(blocking=True)
    assert window.record is None and shown and "too wide" in shown[0]


def _choose(window, stem):
    for i in range(window.vehicle_box.count()):
        e = window.vehicle_box.itemData(i)
        if getattr(e, "path", None) is not None and e.path.stem == stem:
            window.vehicle_box.setCurrentIndex(i)
            return
    raise AssertionError(stem)


def test_steering_box_reads_the_rocket_file_and_help_has_the_topic(window):
    assert "steering" in ANCHORS and "tips" not in ANCHORS
    _choose(window, "rocket_backyard")
    assert not window.steer_check.isChecked() and window.steer_check.isEnabled()
    assert window.current_dict()["controller"]["type"] == "none"
    _choose(window, "example_tvc_closed_loop")
    assert window.steer_check.isChecked()
    assert window.steer_angle.value() == 6.0 and window.steer_wn.value() == 8.0


def test_steering_on_builds_a_valid_closed_loop_config_and_moves_the_nozzle(window):
    _choose(window, "example_tvc_closed_loop")
    window.steer_check.setChecked(False)
    off = window.current_dict()
    assert off["controller"]["type"] == "none" and off["estimator"]["type"] == "none"
    window.run_simulation(blocking=True)
    tumbling = window.record.summary["apogee_m"]
    window.steer_check.setChecked(True)
    d = window.current_dict()
    assert d["controller"]["type"] == "tvc_attitude" and d["fidelity"] >= 5 and d["motor"]["ignition_delay_s"] >= 2.0
    window.run_simulation(blocking=True)
    rec = window.record
    assert rec.summary["apogee_m"] > 2.0 * tumbling  # steering holds the unbalanced rocket up (docs: ~305 m vs ~94 m)
    assert friendly.steering_story(rec) and "nozzle" in friendly.steering_story(rec)[0]
    assert "nozzle" in window.start_page.toPlainText()


def test_steering_can_be_turned_on_for_an_ordinary_rocket(window):
    _choose(window, "rocket_backyard")
    window.steer_check.setChecked(True)
    window.steer_angle.setValue(4.0)
    d = window.current_dict()
    assert d["tvc"]["max_angle_deg"] == 4.0 and d["estimator"]["type"] == "nav_kf"
    window.run_simulation(blocking=True)
    assert window.record is not None and window.record.summary["apogee_m"] > 50
