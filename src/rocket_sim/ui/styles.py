MAIN_STYLE = """
QMainWindow {
    background-color: #1e1e1e;
}

QWidget {
    color: #e0e0e0;
    font-size: 10pt;
}

QGroupBox {
    font-weight: bold;
    border: 1px solid #444444;
    border-radius: 6px;
    margin-top: 12px;
    padding: 10px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
}

QPushButton {
    padding: 7px 14px;
}

#run_button {
    background-color: #0e639c;
    color: #ffffff;
    font-weight: bold;
    font-size: 11pt;
}

#run_button:hover {
    background-color: #1177bb;
}

#run_button:disabled {
    background-color: #3a3d41;
    color: #888888;
}

/* explicit colours: the light/dark Windows theme must not decide whether text is readable */
QTextBrowser,
QPlainTextEdit,
QListWidget,
QTableWidget {
    background-color: #252526;
    color: #e0e0e0;
}

QDoubleSpinBox,
QComboBox {
    padding: 4px;
}

QLabel {
    font-size: 10pt;
}

#flight_view {
    border: 1px solid #444444;
    border-radius: 6px;
}

#summary_value {
    font-size: 14pt;
    font-weight: bold;
}

QStatusBar {
    border-top: 1px solid #444444;
}

QSlider::groove:horizontal {
    height: 4px;
}

QSlider::handle:horizontal {
    width: 12px;
    margin: -5px 0;
}
"""
