"""Dialog for searching ThrustCurve.org and saving thrust curves to the local motor library."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

from PySide6 import QtCore, QtWidgets
from PySide6.QtCore import Qt

from ..errors import RocketSimError
from ..motor import thrustcurve as tc
from ..resources import motors_dir, user_motors_dir

DIAMETERS = ["Any", "13", "18", "24", "29", "38", "54", "75", "98", "150"]
CLASSES = ["Any", *"ABCDEFGHIJKLMNO"]
COLUMNS = [
    "Motor",
    "Manufacturer",
    "Class",
    "Dia (mm)",
    "Impulse (N·s)",
    "Burn (s)",
    "Avg thrust (N)",
    "Status",
]


class _Job(QtCore.QObject):
    done = QtCore.Signal(object)
    failed = QtCore.Signal(str)
    progress = QtCore.Signal(int, int, str)

    def __init__(self, fn: Callable[[Callable[[int, int, str], None]], Any]) -> None:
        super().__init__()
        self.fn = fn

    @QtCore.Slot()
    def run(self) -> None:
        try:
            self.done.emit(self.fn(lambda i, n, name: self.progress.emit(i, n, name)))
        except RocketSimError as exc:
            self.failed.emit(str(exc))
        except Exception as exc:
            self.failed.emit(f"{type(exc).__name__}: {exc}")


class MotorBrowser(QtWidgets.QDialog):
    motors_changed = QtCore.Signal(str)  # stem of the last motor saved

    def __init__(self, parent: QtWidgets.QWidget | None = None, directory: Path | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Motor library: ThrustCurve.org")
        self.resize(980, 620)
        self.directory = directory or user_motors_dir()
        self.results: list[tc.MotorInfo] = []
        self._thread: QtCore.QThread | None = None
        self._job: _Job | None = None
        self._working = False
        self._on_done: Callable[[Any], None] = lambda _r: None
        self.last_saved: str | None = None

        v = QtWidgets.QVBoxLayout(self)
        bar = QtWidgets.QHBoxLayout()
        self.name = QtWidgets.QLineEdit()
        self.name.setPlaceholderText("Motor name, e.g. G40 or F15")
        self.name.returnPressed.connect(self.search)
        self.manufacturer = QtWidgets.QLineEdit()
        self.manufacturer.setPlaceholderText("Manufacturer")
        self.manufacturer.setMaximumWidth(150)
        self.manufacturer.returnPressed.connect(self.search)
        self.diameter = QtWidgets.QComboBox()
        self.diameter.addItems(DIAMETERS)
        self.impulse_class = QtWidgets.QComboBox()
        self.impulse_class.addItems(CLASSES)
        self.search_button = QtWidgets.QPushButton("Search")
        self.search_button.setObjectName("primary")
        self.search_button.clicked.connect(self.search)
        bar.addWidget(self.name, 1)
        bar.addWidget(self.manufacturer)
        bar.addWidget(QtWidgets.QLabel("Diameter"))
        bar.addWidget(self.diameter)
        bar.addWidget(QtWidgets.QLabel("Class"))
        bar.addWidget(self.impulse_class)
        bar.addWidget(self.search_button)
        v.addLayout(bar)

        self.table = QtWidgets.QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSortingEnabled(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self._sync_buttons)
        v.addWidget(self.table, 1)

        self.status = QtWidgets.QLabel(
            "Search by name, manufacturer, diameter or class. Downloads are saved to the local library."
        )
        self.status.setWordWrap(True)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setVisible(False)
        self.progress.setTextVisible(False)
        self.progress.setMaximumHeight(6)
        v.addWidget(self.progress)
        v.addWidget(self.status)

        buttons = QtWidgets.QHBoxLayout()
        self.dl_selected = QtWidgets.QPushButton("Download selected")
        self.dl_selected.setObjectName("primary")
        self.dl_selected.clicked.connect(self.download_selected)
        self.dl_results = QtWidgets.QPushButton("Download all results")
        self.dl_results.clicked.connect(self.download_results)
        self.dl_all = QtWidgets.QPushButton("Download full catalogue…")
        self.dl_all.clicked.connect(self.download_catalogue)
        self.close_button = QtWidgets.QPushButton("Close")
        self.close_button.clicked.connect(self.accept)
        self.folder_label = QtWidgets.QLabel(f"Library: {self.directory}")
        self.folder_label.setObjectName("muted")
        buttons.addWidget(self.folder_label, 1)
        for b in (self.dl_selected, self.dl_results, self.dl_all, self.close_button):
            buttons.addWidget(b)
        v.addLayout(buttons)
        self._sync_buttons()

    # ----------------------------------------------------------------------------- jobs
    def _busy(self, on: bool, text: str | None = None) -> None:
        self._working = on
        for w in (self.search_button, self.dl_selected, self.dl_results, self.dl_all):
            w.setEnabled(not on)
        self.progress.setVisible(on)
        if on:
            self.progress.setRange(0, 0)
        if text is not None:
            self.status.setText(text)
        if not on:
            self._sync_buttons()

    def _run(
        self, fn: Callable[[Callable[[int, int, str], None]], Any], on_done: Callable[[Any], None], text: str
    ) -> None:
        if self._working:
            return
        self._busy(True, text)
        self._job = _Job(fn)
        self._thread = QtCore.QThread(self)
        self._job.moveToThread(self._thread)
        self._thread.started.connect(self._job.run)
        self._job.progress.connect(self._on_progress)
        self._on_done = on_done
        self._job.done.connect(self._job_done)  # bound methods run in the GUI thread, lambdas would not
        self._job.failed.connect(self._job_failed)
        self._job.done.connect(self._thread.quit)
        self._job.failed.connect(self._thread.quit)
        self._thread.start()

    @QtCore.Slot(object)
    def _job_done(self, result: Any) -> None:
        self._busy(False)
        self._on_done(result)

    @QtCore.Slot(str)
    def _job_failed(self, message: str) -> None:
        self._busy(False, message)

    def _on_progress(self, i: int, n: int, name: str) -> None:
        self.progress.setRange(0, n)
        self.progress.setValue(i)
        self.status.setText(f"Downloading {i}/{n}: {name}")

    def wait(self) -> None:
        """Block until the running job (if any) has finished; used by tests."""
        while self._thread is not None and self._thread.isRunning():
            QtCore.QCoreApplication.processEvents()  # the job's result and quit() are delivered through the event loop
            self._thread.wait(10)
        QtCore.QCoreApplication.processEvents()

    # --------------------------------------------------------------------------- search
    def _criteria(self) -> dict[str, Any]:
        d = self.diameter.currentText()
        c = self.impulse_class.currentText()
        return {
            "name": self.name.text().strip() or None,
            "manufacturer": self.manufacturer.text().strip() or None,
            "diameter_mm": float(d) if d != "Any" else None,
            "impulse_class": c if c != "Any" else None,
        }

    def search(self) -> None:
        crit = self._criteria()
        if not any(crit.values()):
            self.status.setText("Enter a name or choose a filter first (or use Download full catalogue).")
            return
        self._run(lambda _p: tc.search(**crit), self._show_results, "Searching ThrustCurve.org…")

    def _local(self, m: tc.MotorInfo) -> bool:
        return (self.directory / f"{m.stem}.eng").exists() or (motors_dir() / f"{m.stem}.eng").exists()

    def _show_results(self, infos: list[tc.MotorInfo], announce: bool = True) -> None:
        self.results = sorted(infos, key=lambda m: (m.manufacturer, m.total_impulse_ns))
        self.table.setRowCount(len(self.results))
        for r, m in enumerate(self.results):
            status = ("Saved" if self._local(m) else "Online") + (
                " (out of production)" if m.availability == "OOP" else ""
            )
            cells = [
                m.designation,
                m.manufacturer,
                m.impulse_class,
                f"{m.diameter_mm:.0f}",
                f"{m.total_impulse_ns:.1f}",
                f"{m.burn_time_s:.2f}",
                f"{m.avg_thrust_n:.1f}",
                status,
            ]
            for c, text in enumerate(cells):
                it = QtWidgets.QTableWidgetItem(text)
                if c in (2, 3, 4, 5, 6):
                    it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.table.setItem(r, c, it)
        self.table.resizeColumnsToContents()
        if announce:
            self.status.setText(f"{len(self.results)} motors found." if self.results else "No motors found.")
        self._sync_buttons()

    def _sync_buttons(self) -> None:
        have = bool(self.results)
        busy = self._working
        self.dl_selected.setEnabled(
            not busy and bool(self.table.selectionModel() and self.table.selectionModel().selectedRows())
        )
        self.dl_results.setEnabled(not busy and have)
        self.dl_all.setEnabled(not busy)
        self.search_button.setEnabled(not busy)

    # ------------------------------------------------------------------------ downloads
    def _download(self, infos: list[tc.MotorInfo]) -> None:
        if not infos:
            return

        def job(
            progress: Callable[[int, int, str], None],
        ) -> tuple[list[Path], list[tuple[tc.MotorInfo, str]]]:
            return tc.download_motors(infos, self.directory, progress=progress)

        self._run(job, self._downloaded, f"Downloading {len(infos)} motors…")

    def _downloaded(self, result: tuple[list[Path], list[tuple[tc.MotorInfo, str]]]) -> None:
        saved, failed = result
        msg = f"Saved {len(saved)} motors to {self.directory}."
        if failed:
            msg += f" {len(failed)} skipped (no usable thrust curve)."
        self.status.setText(msg)
        if saved:
            self.last_saved = saved[-1].stem
            self.motors_changed.emit(self.last_saved)
        if self.results:
            self._show_results(self.results, announce=False)

    def download_selected(self) -> None:
        rows = sorted({i.row() for i in self.table.selectionModel().selectedRows()})
        self._download([self.results[r] for r in rows])

    def download_results(self) -> None:
        self._download(list(self.results))

    def download_catalogue(self) -> None:
        ok = QtWidgets.QMessageBox.question(
            self,
            "Download full catalogue",
            "This downloads every thrust curve on ThrustCurve.org (about 1,000 small files) into the motor library. Continue?",
        )
        if ok != QtWidgets.QMessageBox.StandardButton.Yes:
            return
        self._run(lambda p: self._catalogue(p), self._downloaded, "Fetching the catalogue…")

    def _catalogue(
        self, progress: Callable[[int, int, str], None]
    ) -> tuple[list[Path], list[tuple[tc.MotorInfo, str]]]:
        return tc.download_motors(tc.search(), self.directory, progress=progress)
