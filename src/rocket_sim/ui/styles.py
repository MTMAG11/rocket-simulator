"""Application theme: one dark palette applied with the Fusion style, so the look does not depend on the Windows theme."""

from __future__ import annotations

from PySide6 import QtGui, QtWidgets

BG = "#12141a"
SURFACE = "#1a1d24"
RAISED = "#232733"
BORDER = "#2b303c"
TEXT = "#e6e9ef"
MUTED = "#8b93a1"
ACCENT = "#4c8dff"
ACCENT_HOVER = "#6ba1ff"
ACCENT_PRESSED = "#3a74de"
PLOT_BG = "#14161c"

MAIN_STYLE = f"""
* {{ font-family: "Segoe UI", "Inter", sans-serif; font-size: 10pt; }}
QMainWindow, QDialog, QWidget#sidebar, QScrollArea#sidebar_scroll {{ background: {BG}; }}
QWidget {{ color: {TEXT}; }}
QToolTip {{ background: {RAISED}; color: {TEXT}; border: 1px solid {BORDER}; padding: 4px 6px; }}

QLabel {{ background: transparent; }}
QLabel#muted {{ color: {MUTED}; font-size: 9pt; }}
QLabel#brand {{ padding: 2px 2px 4px 2px; }}
QLabel#card_title {{ color: {MUTED}; font-size: 8pt; font-weight: 600; letter-spacing: 1px; }}

QFrame#card {{ background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 10px; }}
QFrame#card QLabel {{ border: none; }}
QFrame#stat_tile {{ background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 10px; }}
QLabel#stat_label {{ color: {MUTED}; font-size: 8pt; border: none; }}
QLabel#stat_value {{ font-size: 13pt; font-weight: 600; border: none; }}

QPushButton {{ background: {RAISED}; border: 1px solid {BORDER}; border-radius: 7px; padding: 7px 14px; }}
QPushButton:hover {{ border-color: {ACCENT}; }}
QPushButton:pressed {{ background: {BORDER}; }}
QPushButton:disabled {{ color: #5b6270; background: {SURFACE}; border-color: {BORDER}; }}
QPushButton#ghost {{ background: transparent; padding: 6px 10px; }}
QPushButton#ghost:hover {{ background: {RAISED}; }}
QPushButton#primary, QPushButton#run_button {{ background: {ACCENT}; border: none; color: white; font-weight: 600; }}
QPushButton#primary:hover, QPushButton#run_button:hover {{ background: {ACCENT_HOVER}; }}
QPushButton#primary:pressed, QPushButton#run_button:pressed {{ background: {ACCENT_PRESSED}; }}
QPushButton#primary:disabled, QPushButton#run_button:disabled {{ background: {RAISED}; color: #6b7280; }}
QPushButton#run_button {{ font-size: 11pt; border-radius: 9px; }}

QToolButton#disclosure {{ background: transparent; border: none; color: {MUTED}; padding: 4px 2px; text-align: left; }}
QToolButton#disclosure:hover {{ color: {TEXT}; }}

QComboBox, QDoubleSpinBox, QSpinBox, QLineEdit {{
    background: {BG}; border: 1px solid {BORDER}; border-radius: 6px; padding: 5px 8px; selection-background-color: {ACCENT};
}}
QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover, QLineEdit:hover {{ border-color: #3a4150; }}
QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus {{ border-color: {ACCENT}; }}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{ background: {SURFACE}; border: 1px solid {BORDER}; selection-background-color: {ACCENT}; outline: 0; }}
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button, QSpinBox::up-button, QSpinBox::down-button {{ width: 16px; border: none; background: transparent; }}

QTabWidget::pane {{ border: none; border-top: 1px solid {BORDER}; top: -1px; }}
QTabBar {{ background: transparent; }}
QTabBar::tab {{ background: transparent; color: {MUTED}; padding: 9px 16px; border: none; border-bottom: 2px solid transparent; margin-right: 2px; }}
QTabBar::tab:hover {{ color: {TEXT}; }}
QTabBar::tab:selected {{ color: {TEXT}; border-bottom: 2px solid {ACCENT}; }}

QTextBrowser, QPlainTextEdit, QListWidget, QTreeWidget, QTableWidget {{
    background: {SURFACE}; border: 1px solid {BORDER}; border-radius: 8px; color: {TEXT}; selection-background-color: {ACCENT};
}}
QPlainTextEdit#path {{ background: {BG}; color: {MUTED}; font-size: 9pt; border-radius: 6px; }}
QTreeWidget::item {{ padding: 3px 2px; }}
QTreeWidget::item:hover {{ background: {RAISED}; }}
QTableWidget {{ alternate-background-color: #1e222b; gridline-color: {BORDER}; }}
QHeaderView::section {{ background: {RAISED}; color: {MUTED}; border: none; border-bottom: 1px solid {BORDER}; padding: 6px 8px; font-weight: 600; }}
QTableWidget::item:selected, QTreeWidget::item:selected {{ background: {ACCENT}; color: white; }}

QSlider::groove:horizontal {{ height: 4px; background: {BORDER}; border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 2px; }}
QSlider::handle:horizontal {{ width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; background: {TEXT}; }}
QSlider::handle:horizontal:hover {{ background: white; }}

QProgressBar {{ background: {BORDER}; border: none; border-radius: 3px; }}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 3px; }}

QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border-radius: 4px; border: 1px solid #3a4150; background: {BG}; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}
QTreeWidget::indicator {{ width: 15px; height: 15px; border-radius: 4px; border: 1px solid #3a4150; background: {BG}; }}
QTreeWidget::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: #333a48; border-radius: 4px; min-height: 28px; }}
QScrollBar::handle:vertical:hover {{ background: #46506a; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 2px; }}
QScrollBar::handle:horizontal {{ background: #333a48; border-radius: 4px; min-width: 28px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

QStatusBar {{ background: {BG}; color: {MUTED}; border-top: 1px solid {BORDER}; }}
QStatusBar QLabel {{ color: {MUTED}; }}
QMessageBox {{ background: {SURFACE}; }}
"""


def apply_theme(app: QtWidgets.QApplication) -> None:
    app.setStyle("Fusion")
    p = QtGui.QPalette()
    role = QtGui.QPalette.ColorRole
    for r, c in (
        (role.Window, BG),
        (role.WindowText, TEXT),
        (role.Base, SURFACE),
        (role.AlternateBase, RAISED),
        (role.Text, TEXT),
        (role.Button, RAISED),
        (role.ButtonText, TEXT),
        (role.ToolTipBase, RAISED),
        (role.ToolTipText, TEXT),
        (role.Highlight, ACCENT),
        (role.HighlightedText, "#ffffff"),
        (role.PlaceholderText, MUTED),
    ):
        p.setColor(r, QtGui.QColor(c))
    app.setPalette(p)
    app.setStyleSheet(MAIN_STYLE)
