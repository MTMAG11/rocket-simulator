"""The one way to start the simulator GUI: ``python -m rocket_sim`` (source) or ``RocketSimulator.exe`` (packaged).

Both end up in :func:`main`. ``rocketsim gui`` calls the same function. Failures that a user can act on (missing data files,
PySide6 not installed, a broken display) are shown as a plain message, in a dialog when possible and always on stderr and in
``<workspace>/logs/gui_error.log``, never as a bare traceback.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import traceback
from pathlib import Path
from typing import Any

from ..errors import RocketSimError
from ..resources import (
    APP_NAME,
    app_version,
    check_installation,
    is_frozen,
    launch_description,
    resource_root,
    workspace_dir,
)

_HOW_TO_FIX_PYSIDE = (
    "PySide6 (the GUI library) is not installed in this Python environment.\n\n"
    "Activate your virtual environment and install the simulator:\n"
    "    .venv\\Scripts\\activate\n"
    "    pip install -e .\n"
    "Then start it again with:\n"
    "    python -m rocket_sim"
)


def _log_path() -> Path:
    return workspace_dir() / "logs" / "gui_error.log"


def report_fatal(title: str, message: str, detail: str = "") -> None:
    """Show an error where the user will see it: stderr (if any), a log file, and a dialog (if Qt works)."""
    text = f"{title}\n\n{message}"
    if sys.stderr is not None:  # a windowed executable has no console
        print(f"\n{text}\n", file=sys.stderr)
    try:
        lp = _log_path()
        lp.parent.mkdir(parents=True, exist_ok=True)
        lp.write_text(
            f"{APP_NAME} {app_version()} ({launch_description()})\n\n{text}\n\n{detail}", encoding="utf-8"
        )
        text += f"\n\n(details written to {lp})"
    except OSError:
        pass
    try:
        from PySide6 import QtWidgets

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
        QtWidgets.QMessageBox.critical(None, f"{APP_NAME}: {title}", text)
        del app
    except Exception:  # Qt itself is the problem: the stderr/log copy above is all there is
        pass


def _install_excepthook() -> None:
    def hook(exc_type, exc, tb):
        detail = "".join(traceback.format_exception(exc_type, exc, tb))
        if issubclass(exc_type, RocketSimError):
            report_fatal("Simulator error", str(exc), detail)
        else:
            report_fatal("Unexpected error", f"{exc_type.__name__}: {exc}", detail)

    sys.excepthook = hook


def launch_gui() -> int:
    """Check the installation, then open the main window. Returns the process exit code."""
    problems = check_installation()
    if problems:
        report_fatal("The simulator's data files were not found", "\n\n".join(problems))
        return 2
    try:
        from PySide6 import QtWidgets  # noqa: F401
    except ImportError:
        report_fatal("PySide6 is not installed", _HOW_TO_FIX_PYSIDE)
        return 2
    _install_excepthook()
    try:
        from .main_window import main as gui_main

        gui_main()
    except RocketSimError as exc:
        report_fatal("The simulator could not start", str(exc), traceback.format_exc())
        return 2
    except Exception as exc:
        report_fatal("The simulator could not start", f"{type(exc).__name__}: {exc}", traceback.format_exc())
        return 1
    return 0


def self_test(out: Path) -> int:
    """Start the GUI off-screen, run the default vehicle with its default motor, and write a JSON report to ``out``.

    Used to test the source install, the launcher and the packaged executable without a person at the screen."""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    report: dict[str, Any] = {
        "ok": False,
        "version": app_version(),
        "launched_as": launch_description(),
        "frozen": is_frozen(),
        "python": sys.version.split()[0],
    }
    code = 1
    try:
        report["resource_root"] = str(resource_root())
        from PySide6 import QtWidgets

        from .main_window import MainWindow

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
        w = MainWindow()
        report["vehicles"] = [w.vehicle_box.itemText(i) for i in range(w.vehicle_box.count())]
        report["motors"] = [w.motor_box.itemText(i) for i in range(w.motor_box.count())]
        report["selected_vehicle"] = w.vehicle_box.currentText()
        report["selected_motor"] = w.motor_box.currentText()
        w.run_simulation(blocking=True)
        rec = w.record
        if rec is None:
            raise RuntimeError("the GUI run produced no flight record")
        report["apogee_m"] = float(rec.summary["apogee_m"])  # type: ignore[arg-type]  # KeyError (reported below) if the summary lacks it
        report["results_dir"] = str(w.results_dir)
        report["result_files"] = sorted(p.name for p in w.results_dir.iterdir()) if w.results_dir else []
        report["summary_tab_text_chars"] = len(w.summary_text.toPlainText())
        report["ok"] = bool(w.results_dir and report["result_files"] and report["summary_tab_text_chars"] > 0)
        code = 0 if report["ok"] else 1
        del app
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return code


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="rocket_sim", description=f"{APP_NAME}: opens the simulator window.")
    p.add_argument("--version", action="store_true", help="print the version and exit")
    p.add_argument(
        "--check", action="store_true", help="check that the data files and GUI library are present, and exit"
    )
    p.add_argument(
        "--self-test", metavar="REPORT.json", help="run the default simulation off-screen and write a report"
    )
    a = p.parse_args(argv)
    if a.version:
        print(f"{APP_NAME} {app_version()}")
        return 0
    if a.check:
        problems = check_installation()
        print(f"{APP_NAME} {app_version()}  ({launch_description()})")
        if problems:
            print("\n\n".join(problems), file=sys.stderr)
            return 2
        print(f"data files: OK ({resource_root()})")
        try:
            import PySide6

            print(f"PySide6: OK ({PySide6.__version__})")
        except ImportError:
            print(_HOW_TO_FIX_PYSIDE, file=sys.stderr)
            return 2
        return 0
    if a.self_test:
        return self_test(Path(a.self_test))
    return launch_gui()
