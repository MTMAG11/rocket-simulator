"""3-D flight view: a small orbit-camera renderer drawn with QPainter.

The camera is the only thing that changes while the user drags, so nothing is re-scaled or re-sorted between frames.
World axes are East (x), North (y), Up (z) in metres, drawn at true scale.
"""

from __future__ import annotations

import math

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt

from ..simulation import FlightPhase
from ..simulation.record import FlightRecord

PHASE_COLORS = {
    0: "#9a9a9a",
    1: "#f2c94c",
    2: "#ff7a3d",
    3: "#d78bf0",
    4: "#4fc1ff",
    5: "#6adf8a",
    6: "#8fd3f4",
    7: "#ffffff",
}
PHASE_LABELS = {
    0: "Pre-launch",
    1: "Ignition",
    2: "Powered",
    3: "Burnout",
    4: "Coast",
    5: "Apogee",
    6: "Descent",
    7: "Landed",
}
BG = QtGui.QColor("#14161c")
GRID = QtGui.QColor(90, 96, 108, 110)
TEXT = QtGui.QColor("#cfd3da")
MAX_POINTS = 3000


def _nice_step(span: float) -> float:
    raw = span / 8.0
    mag = 10 ** math.floor(math.log10(max(raw, 1e-9)))
    for m in (1, 2, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


class Trajectory3DView(QtWidgets.QWidget):
    """Orbit (left drag), pan (right/middle drag), zoom (wheel), reset (double click)."""

    DEFAULT_AZ, DEFAULT_EL = 35.0, 22.0

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(400, 300)
        self.setMouseTracking(False)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.az, self.el = self.DEFAULT_AZ, self.DEFAULT_EL
        self.dist = 100.0
        self.pan = np.zeros(3)
        self.follow = False
        self._drag: tuple[QtCore.QPoint, str] | None = None
        self.rec: FlightRecord | None = None
        self.idx = 0
        self.pts = np.zeros((0, 3))
        self.phase = np.zeros(0, dtype=int)
        self.sel = np.zeros(0, dtype=int)  # indices of the plotted subset
        self.center = np.zeros(3)
        self.extent = 50.0
        self.hspan = 50.0
        self.height_m = 50.0

    # ----------------------------------------------------------------------------- data
    def set_record(self, rec: FlightRecord) -> None:
        self.rec = rec
        p = np.column_stack([rec.col("pos_x"), rec.col("pos_y"), rec.col("pos_z")])
        n = len(p)
        self.sel = np.unique(np.linspace(0, n - 1, min(n, MAX_POINTS)).astype(int))
        self.pts = p
        self.phase = rec.col("phase").astype(int)
        lo, hi = p.min(axis=0), p.max(axis=0)
        lo[2] = min(lo[2], 0.0)
        self.center = 0.5 * (lo + hi)
        self.extent = max(float(np.max(hi - lo)), 20.0)
        self.hspan = max(float(np.max((hi - lo)[:2])), 1.0)
        self.height_m = float(hi[2] - lo[2])
        self.idx = 0
        self.reset_view()

    def set_index(self, i: int) -> None:
        self.idx = i
        self.update()

    def set_follow(self, on: bool) -> None:
        self.follow = on
        self.update()

    # ---------------------------------------------------------------------------- camera
    def reset_view(self, az: float | None = None, el: float | None = None) -> None:
        self.az = self.DEFAULT_AZ if az is None else az
        self.el = self.DEFAULT_EL if el is None else el
        self.dist = self.extent * 1.35
        self.pan = np.zeros(3)
        self.update()

    def _target(self) -> np.ndarray:
        if self.follow and len(self.pts):
            return self.pts[self.idx] + self.pan
        return self.center + self.pan

    def _basis(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        az, el = math.radians(self.az), math.radians(self.el)
        direction = np.array([math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el)])
        cam = self._target() + self.dist * direction
        fwd = -direction
        right = np.cross(fwd, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        return cam, fwd, right, up

    def _project(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Screen coordinates (N x 2, NaN behind the camera) and depth."""
        cam, fwd, right, up = self._basis()
        v = np.atleast_2d(pts) - cam
        xc, yc, zc = v @ right, v @ up, v @ fwd
        f = 0.95 * min(self.width(), self.height())
        with np.errstate(divide="ignore", invalid="ignore"):
            sx = self.width() / 2 + f * xc / zc
            sy = self.height() / 2 - f * yc / zc
        bad = zc < 0.05 * self.dist
        sx[bad] = np.nan
        sy[bad] = np.nan
        return np.column_stack([sx, sy]), zc

    # --------------------------------------------------------------------------- mouse
    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        mode = "orbit" if e.button() == Qt.MouseButton.LeftButton else "pan"
        self._drag = (e.position().toPoint(), mode)
        self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, e: QtGui.QMouseEvent) -> None:
        if self._drag is None:
            return
        last, mode = self._drag
        pos = e.position().toPoint()
        dx, dy = pos.x() - last.x(), pos.y() - last.y()
        self._drag = (pos, mode)
        if mode == "orbit":
            self.az = (self.az - 0.4 * dx) % 360.0
            self.el = float(np.clip(self.el + 0.4 * dy, -10.0, 89.0))
        else:
            _, _, right, up = self._basis()
            scale = self.dist / (0.95 * min(self.width(), self.height()))
            self.pan += (-dx * right + dy * up) * scale
        self.update()

    def mouseReleaseEvent(self, e: QtGui.QMouseEvent) -> None:
        self._drag = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def mouseDoubleClickEvent(self, e: QtGui.QMouseEvent) -> None:
        self.reset_view()

    def wheelEvent(self, e: QtGui.QWheelEvent) -> None:
        steps = e.angleDelta().y() / 120.0
        self.dist = float(np.clip(self.dist * (0.88**steps), self.extent * 0.05, self.extent * 20))
        self.update()

    # -------------------------------------------------------------------------- drawing
    def paintEvent(self, _e: QtGui.QPaintEvent) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), BG)
        if self.rec is None or not len(self.pts):
            p.setPen(TEXT)
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Run a simulation to see the flight.")
            return
        self._draw_ground(p)
        self._draw_trajectory(p)
        self._draw_rocket(p)
        self._draw_gizmo(p)
        self._draw_hud(p)

    def _line(self, p: QtGui.QPainter, a: np.ndarray, b: np.ndarray) -> None:
        s, _ = self._project(np.array([a, b]))
        if np.all(np.isfinite(s)):
            p.drawLine(QtCore.QPointF(*s[0]), QtCore.QPointF(*s[1]))

    def _polyline(self, p: QtGui.QPainter, pts: np.ndarray) -> None:
        if len(pts) < 2:
            return
        s, _ = self._project(pts)
        path = QtGui.QPainterPath()
        started = False
        for x, y in s:
            if not (math.isfinite(x) and math.isfinite(y)):
                started = False
                continue
            if started:
                path.lineTo(x, y)
            else:
                path.moveTo(x, y)
                started = True
        p.drawPath(path)

    def _draw_ground(self, p: QtGui.QPainter) -> None:
        half = max(0.55 * self.hspan, 0.3 * self.height_m, 30.0)
        step = _nice_step(2 * half)
        cx = round(self.center[0] / step) * step
        cy = round(self.center[1] / step) * step
        n = int(half / step) + 1
        lo_x, hi_x, lo_y, hi_y = cx - n * step, cx + n * step, cy - n * step, cy + n * step
        corners = np.array([[lo_x, lo_y, 0], [hi_x, lo_y, 0], [hi_x, hi_y, 0], [lo_x, hi_y, 0]])
        s, _ = self._project(corners)
        if np.all(np.isfinite(s)):
            poly = QtGui.QPolygonF([QtCore.QPointF(*q) for q in s])
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor(60, 80, 62, 70))
            p.drawPolygon(poly)
        p.setBrush(Qt.BrushStyle.NoBrush)
        pen = QtGui.QPen(GRID)
        pen.setWidthF(1.0)
        p.setPen(pen)
        font = QtGui.QFont(self.font())
        font.setPointSizeF(8)
        p.setFont(font)
        for k in range(-n, n + 1):
            x, y = cx + k * step, cy + k * step
            self._line(p, np.array([x, lo_y, 0]), np.array([x, hi_y, 0]))
            self._line(p, np.array([lo_x, y, 0]), np.array([hi_x, y, 0]))
        p.setPen(TEXT)
        for k in range(-n + 1, n, 2 if n > 4 else 1):
            for pt, txt in (
                (np.array([cx + k * step, lo_y, 0]), f"{cx + k * step:g}"),
                (np.array([lo_x, cy + k * step, 0]), f"{cy + k * step:g}"),
            ):
                s, _ = self._project(pt)
                if np.all(np.isfinite(s)):
                    p.drawText(QtCore.QPointF(s[0][0] + 3, s[0][1] + 12), txt)
        # axis names at the far ends of the grid
        for pt, txt in ((np.array([hi_x, lo_y, 0]), "East (m)"), (np.array([lo_x, hi_y, 0]), "North (m)")):
            s, _ = self._project(pt)
            if np.all(np.isfinite(s)):
                p.drawText(QtCore.QPointF(s[0][0] + 6, s[0][1] + 14), txt)
        # launch point
        s, _ = self._project(np.zeros(3))
        if np.all(np.isfinite(s)):
            p.setPen(QtGui.QPen(QtGui.QColor("#f2c94c"), 2))
            p.drawEllipse(QtCore.QPointF(*s[0]), 5, 5)

    def _draw_trajectory(self, p: QtGui.QPainter) -> None:
        sel = self.sel
        pts, ph = self.pts[sel], self.phase[sel]
        i = int(np.searchsorted(sel, self.idx))
        # ground track
        shadow = pts.copy()
        shadow[:, 2] = 0.0
        p.setPen(QtGui.QPen(QtGui.QColor(200, 200, 200, 70), 1))
        self._polyline(p, shadow)
        for lo, hi, alpha, w in ((0, i + 1, 255, 2.4), (i, len(pts), 90, 1.6)):
            seg_ph = ph[lo:hi]
            if len(seg_ph) < 2:
                continue
            start = 0
            for k in range(1, len(seg_ph) + 1):
                if k == len(seg_ph) or seg_ph[k] != seg_ph[start]:
                    c = QtGui.QColor(PHASE_COLORS.get(int(seg_ph[start]), "#ffffff"))
                    c.setAlpha(alpha)
                    p.setPen(
                        QtGui.QPen(
                            c, w, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin
                        )
                    )
                    self._polyline(p, pts[lo + start : lo + k + 1])
                    start = k

    def _draw_rocket(self, p: QtGui.QPainter) -> None:
        from ..physics.math3d import quat_rotate

        rec, i = self.rec, self.idx
        assert rec is not None
        pos = self.pts[i]
        # drop line to the ground
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 90), 1, Qt.PenStyle.DashLine))
        self._line(p, pos, np.array([pos[0], pos[1], 0.0]))
        q = (rec.col("quat_w")[i], rec.col("quat_x")[i], rec.col("quat_y")[i], rec.col("quat_z")[i])
        nose = np.array(quat_rotate(q, (1.0, 0.0, 0.0)))
        length = self.dist * 0.07  # a marker of constant on-screen size, not the true rocket length
        tail = pos - nose * length * 0.5
        head = pos + nose * length * 0.5
        if rec.col("thrust")[i] > 0.0:
            p.setPen(QtGui.QPen(QtGui.QColor("#ff9f43"), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            self._line(p, tail, tail - nose * length * 0.45)
        p.setPen(QtGui.QPen(QtGui.QColor("#f5f5f5"), 5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        self._line(p, tail, head)
        p.setPen(QtGui.QPen(QtGui.QColor("#ff5d5d"), 5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        self._line(p, pos + nose * length * 0.28, head)
        # wind arrow at the rocket
        w = np.array([rec.col("wind_x")[i], rec.col("wind_y")[i], 0.0])
        if np.linalg.norm(w) > 0.05:
            p.setPen(QtGui.QPen(QtGui.QColor("#56b4e9"), 2))
            self._line(p, pos, pos + w / np.linalg.norm(w) * length * 0.9)

    def _draw_gizmo(self, p: QtGui.QPainter) -> None:
        _, _, right, up = self._basis()
        origin = QtCore.QPointF(48, self.height() - 48)
        font = QtGui.QFont(self.font())
        font.setPointSizeF(9)
        font.setBold(True)
        p.setFont(font)
        for vec, name, col in (
            ([1, 0, 0], "E", "#ff6b6b"),
            ([0, 1, 0], "N", "#6adf8a"),
            ([0, 0, 1], "Up", "#4fc1ff"),
        ):
            v = np.array(vec, float)
            end = origin + QtCore.QPointF(float(v @ right) * 32, -float(v @ up) * 32)
            p.setPen(QtGui.QPen(QtGui.QColor(col), 2))
            p.drawLine(origin, end)
            p.drawText(end + QtCore.QPointF(3, 4), name)

    def _draw_hud(self, p: QtGui.QPainter) -> None:
        rec, i = self.rec, self.idx
        assert rec is not None
        font = QtGui.QFont(self.font())
        font.setPointSizeF(9)
        p.setFont(font)
        x, y = self.width() - 118, 14
        for ph, label in PHASE_LABELS.items():
            if ph not in set(np.unique(self.phase)):
                continue
            p.setPen(QtGui.QPen(QtGui.QColor(PHASE_COLORS[ph]), 3))
            p.drawLine(x, y + 6, x + 18, y + 6)
            p.setPen(TEXT)
            p.drawText(x + 26, y + 10, label)
            y += 16
        cur = FlightPhase(int(self.phase[i])).name.replace("_", " ").title()
        p.setPen(TEXT)
        p.drawText(12, 20, f"t = {rec.col('t')[i]:.2f} s    {cur}    altitude {rec.col('altitude')[i]:.0f} m")
        p.setPen(QtGui.QColor(150, 155, 165))
        p.drawText(12, 38, "Drag: rotate   Right-drag: pan   Wheel: zoom   Double-click: reset")
