"""Graph catalogue, graph panel and 3-D view (offscreen)."""

from __future__ import annotations

import math
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from rocket_sim.config import load_config
from rocket_sim.resources import default_config
from rocket_sim.simulation import Simulation
from rocket_sim.ui.channels import CATALOG, PRESETS, available_channels, preset_ids


@pytest.fixture(scope="module")
def rec():
    return Simulation(load_config(default_config()), seed=0).run()


@pytest.fixture(scope="module")
def qapp():
    pytest.importorskip("PySide6")
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_catalogue_entries_are_labelled_and_unique():
    ids = [c.id for c in CATALOG]
    assert len(ids) == len(set(ids))
    for c in CATALOG:
        assert c.title and c.group and c.series
        assert all(s.label for s in c.series)
    for name, members in PRESETS.items():
        assert set(members) <= set(ids), name


def test_available_channels_cover_every_plottable_column(rec):
    chans = available_channels(rec)
    covered = {col for c in chans for col in c.columns()}
    skipped = {"t", "dt", "phase"}
    missing = [c for c in rec.columns if c not in covered and c not in skipped and not c.endswith("_new")]
    assert missing == []
    assert preset_ids("Flight overview", chans) == ["altitude", "speed", "load_factor", "thrust"]


def test_display_units_and_derived_values(rec):
    by_id = {c.id: c for c in available_channels(rec)}
    (_, lf) = by_id["load_factor"].values(rec)[0]
    fsp = np.sqrt(sum(rec.col(f"fsp_{a}") ** 2 for a in "xyz"))
    assert np.allclose(lf, fsp / 9.80665)
    assert by_id["attitude"].unit == "deg"
    pitch = dict(by_id["attitude"].values(rec))["Pitch (above horizon)"]
    assert np.allclose(pitch, np.degrees(rec.col("pitch")))
    assert [lbl for lbl, _ in by_id["position"].values(rec)] == ["East", "North", "Up"]


def test_graph_panel_draws_named_axes_with_units(qapp, rec):
    from rocket_sim.ui.graphs import GraphPanel

    g = GraphPanel()
    g.set_record(rec)
    axes = g.figure.axes
    assert [a.get_title(loc="left") for a in axes] == [
        "Altitude above ground",
        "Speed",
        "Load factor",
        "Thrust",
    ]
    assert [a.get_ylabel() for a in axes] == ["m", "m/s", "g", "N"]
    assert axes[-1].get_xlabel() == "Time (s)"
    assert axes[1].get_legend() is not None  # ground speed + airspeed

    g._set_checked(["position"])
    (ax,) = g.figure.axes
    assert [t.get_text() for t in ax.get_legend().get_texts()] == ["East", "North", "Up"]
    g.range_box.setCurrentText("Until apogee")
    (ax,) = g.figure.axes  # the figure is rebuilt on every change
    tmax = ax.get_xlim()[1]
    assert tmax < rec.col("t")[-1] and tmax >= rec.summary["apogee_time_s"]
    g.set_index(len(rec.col("t")) // 2)  # moving the cursor must not raise


def test_selection_survives_a_new_run(qapp, rec):
    from rocket_sim.ui.graphs import GraphPanel

    g = GraphPanel()
    g.set_record(rec)
    g._set_checked(["mass", "aoa"])
    g.set_record(rec)
    assert g.checked == ["mass", "aoa"]


def test_3d_camera_is_a_pure_function_of_the_view(qapp, rec):
    from rocket_sim.ui.view3d import Trajectory3DView

    v = Trajectory3DView()
    v.resize(800, 600)
    v.set_record(rec)
    pt = np.array([[100.0, 50.0, 300.0]])
    before = v._project(pt)[0].copy()
    # rendering and re-projecting must not change anything
    for _ in range(3):
        v.grab()
    assert np.array_equal(v._project(pt)[0], before)
    extent, center = v.extent, v.center.copy()
    az0, el0 = v.az, v.el
    v.az, v.el = az0 + 90.0, el0 + 20.0
    moved = v._project(pt)[0]
    assert not np.allclose(moved, before)
    v.az, v.el = az0, el0
    assert np.allclose(v._project(pt)[0], before)  # returning to the same view gives the same picture
    assert v.extent == extent and np.array_equal(v.center, center)  # the scene never rescales


def test_3d_projection_keeps_true_proportions(qapp, rec):
    from rocket_sim.ui.view3d import Trajectory3DView

    v = Trajectory3DView()
    v.resize(800, 800)
    v.set_record(rec)
    v.reset_view(
        0.0, 0.0
    )  # looking horizontally: a vertical segment must project vertically with its true size
    base = v.center.copy()
    s, _ = v._project(np.array([base, base + [0.0, 0.0, 10.0]]))
    assert abs(s[0][0] - s[1][0]) < 1e-6 and s[0][1] > s[1][1]
    s2, _ = v._project(np.array([base, base + [10.0, 0.0, 0.0]]))
    assert abs(s2[0][1] - s2[1][1]) < 1e-6
    assert math.isclose(abs(s[0][1] - s[1][1]), abs(s2[0][0] - s2[1][0]), rel_tol=0.02)


def test_3d_mouse_orbit_pan_zoom(qapp, rec):
    from PySide6 import QtCore, QtGui

    from rocket_sim.ui.view3d import Trajectory3DView

    v = Trajectory3DView()
    v.resize(800, 600)
    v.set_record(rec)
    az, el, dist = v.az, v.el, v.dist

    def mouse(kind, pos, button=QtCore.Qt.MouseButton.LeftButton):
        e = QtGui.QMouseEvent(
            kind, QtCore.QPointF(*pos), button, button, QtCore.Qt.KeyboardModifier.NoModifier
        )
        {
            QtCore.QEvent.Type.MouseButtonPress: v.mousePressEvent,
            QtCore.QEvent.Type.MouseMove: v.mouseMoveEvent,
            QtCore.QEvent.Type.MouseButtonRelease: v.mouseReleaseEvent,
        }[kind](e)

    mouse(QtCore.QEvent.Type.MouseButtonPress, (100, 100))
    mouse(QtCore.QEvent.Type.MouseMove, (150, 120))
    mouse(QtCore.QEvent.Type.MouseButtonRelease, (150, 120))
    assert v.az != az and v.el != el and v.dist == dist
    assert -10.0 <= v.el <= 89.0
    pan0 = v.pan.copy()
    mouse(QtCore.QEvent.Type.MouseButtonPress, (100, 100), QtCore.Qt.MouseButton.RightButton)
    mouse(QtCore.QEvent.Type.MouseMove, (130, 100), QtCore.Qt.MouseButton.RightButton)
    assert not np.array_equal(v.pan, pan0)
    v.reset_view()
    assert (v.az, v.el) == (v.DEFAULT_AZ, v.DEFAULT_EL) and not v.pan.any()


def test_follow_mode_targets_the_rocket(qapp, rec):
    from rocket_sim.ui.view3d import Trajectory3DView

    v = Trajectory3DView()
    v.set_record(rec)
    v.set_index(len(v.pts) // 4)
    v.set_follow(True)
    assert np.allclose(v._target(), v.pts[v.idx])
    v.set_follow(False)
    assert np.allclose(v._target(), v.center)
