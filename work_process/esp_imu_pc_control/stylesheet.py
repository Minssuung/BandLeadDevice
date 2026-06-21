"""PyQt5 stylesheet for WT61C-TTL IMU tester."""

import os

STYLESHEET = """
QMainWindow, QWidget {
    background-color: #FFFFFF;
}

QLabel {
    color: #000000;
    font-size: 12px;
}

QLabel#secondary {
    color: #666666;
    font-size: 11px;
}

QLabel#emphasized {
    color: #000000;
    font-weight: 600;
}

/* Default button: black contained */
QPushButton,
QPushButton#contained {
    background-color: #000000;
    color: #FFFFFF;
    border: 1px solid #000000;
    border-radius: 4px;
    padding: 4px 12px;
    min-height: 24px;
    max-height: 24px;
    font-size: 11px;
    font-weight: 600;
}

QPushButton:hover,
QPushButton#contained:hover {
    background-color: #2F2F2F;
    border: 1px solid #2F2F2F;
}

QPushButton:pressed,
QPushButton#contained:pressed {
    background-color: #1A1A1A;
    border: 1px solid #1A1A1A;
}

QPushButton:disabled,
QPushButton#contained:disabled {
    background-color: #E0E0E0;
    color: #999999;
    border: 1px solid #CCCCCC;
}

/* Relay stop button: primary pink contained */
QPushButton#relayStop {
    background-color: #FB0082;
    color: #FFFFFF;
    border: 1px solid #FB0082;
}

QPushButton#relayStop:hover {
    background-color: #E60070;
    border: 1px solid #E60070;
}

QPushButton#relayStop:pressed {
    background-color: #D10064;
    border: 1px solid #D10064;
}

QPushButton#relayStop:disabled {
    background-color: #E0E0E0;
    color: #999999;
    border: 1px solid #CCCCCC;
}

/* Reusable outlined button: black outlined */
QPushButton#outlined {
    background-color: #FFFFFF;
    color: #000000;
    border: 1px solid #000000;
}

QPushButton#outlined:hover {
    background-color: #F7F7F7;
    color: #000000;
    border: 1px solid #000000;
}

QPushButton#outlined:pressed {
    background-color: #EFEFEF;
    color: #000000;
    border: 1px solid #000000;
}

QPushButton#outlined:disabled {
    background-color: #F5F5F5;
    color: #AAAAAA;
    border: 1px solid #CCCCCC;
}

/* Important button: pink outlined (connect/disconnect only) */
QPushButton#important {
    background-color: #FFFFFF;
    color: #FB0082;
    border: 1px solid #FB0082;
}

QPushButton#important:hover {
    background-color: #FFF3F9;
    color: #E60070;
    border: 1px solid #E60070;
}

QPushButton#important:pressed {
    background-color: #FFE4F0;
    color: #D10064;
    border: 1px solid #D10064;
}

QPushButton#important:disabled {
    background-color: #F5F5F5;
    color: #B3B3B3;
    border: 1px solid #D9D9D9;
}

QComboBox {
    background-color: #FFFFFF;
    color: #000000;
    border: 1px solid #CCCCCC;
    border-radius: 3px;
    padding: 3px 8px;
    min-height: 26px;
    max-height: 26px;
    font-size: 11px;
}

QComboBox:hover {
    border: 1px solid #999999;
}

QComboBox:focus {
    border: 1px solid #FB0082;
}

QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid #DDDDDD;
    background-color: #FAFAFA;
}

QComboBox::down-arrow {
    image: url(__COMBO_ARROW__);
    width: 10px;
    height: 10px;
}

QComboBox QAbstractItemView {
    background-color: #FFFFFF;
    color: #000000;
    border: 1px solid #CCCCCC;
    selection-background-color: #EFEFEF;
    selection-color: #000000;
    outline: 0;
}

QGroupBox {
    color: #000000;
    font-size: 12px;
    font-weight: 600;
    border: 1px solid #CCCCCC;
    border-radius: 6px;
    margin-top: 12px;
    padding: 10px 12px 12px 12px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 8px;
    top: 3px;
    padding: 0 4px;
    background-color: #FFFFFF;
    color: #000000;
}

QTableWidget {
    background-color: #FFFFFF;
    alternate-background-color: #FAFAFA;
    color: #000000;
    gridline-color: #D6D6D6;
    border: 1px solid #CCCCCC;
    border-radius: 3px;
}

QHeaderView::section {
    background-color: #F4F4F4;
    color: #000000;
    font-size: 11px;
    font-weight: 600;
    padding: 6px;
    border: 0px;
    border-right: 1px solid #D6D6D6;
    border-bottom: 1px solid #D6D6D6;
}

QTableCornerButton::section {
    background-color: #F4F4F4;
    border: 0px;
    border-right: 1px solid #D6D6D6;
    border-bottom: 1px solid #D6D6D6;
}

QTableWidget::item {
    color: #000000;
    padding: 4px 6px;
    border: 0px;
    border-right: 1px solid #E1E1E1;
    border-bottom: 1px solid #E1E1E1;
}

QTableWidget::item:selected {
    background-color: #EAEAEA;
    color: #000000;
}

/* Table-specific scrollbar: make handle clearly rounded */
QTableWidget QScrollBar:vertical {
    background: #F5F5F5;
    width: 14px;
    border: none;
    border-radius: 7px;
    margin: 2px;
}

QTableWidget QScrollBar::handle:vertical {
    background: #9B9B9B;
    min-height: 28px;
    border-radius: 7px;
    border: 1px solid #F5F5F5;
    margin: 1px;
}

QTableWidget QScrollBar::handle:vertical:hover {
    background: #707070;
}

QTableWidget QScrollBar::add-line:vertical,
QTableWidget QScrollBar::sub-line:vertical {
    height: 0px;
    border: none;
    background: none;
}

QTableWidget QScrollBar::add-page:vertical,
QTableWidget QScrollBar::sub-page:vertical {
    background: transparent;
    border: none;
    border-radius: 7px;
}

QTableWidget QScrollBar:horizontal {
    background: #F5F5F5;
    height: 14px;
    border: none;
    border-radius: 7px;
    margin: 2px;
}

QTableWidget QScrollBar::handle:horizontal {
    background: #9B9B9B;
    min-width: 28px;
    border-radius: 7px;
    border: 1px solid #F5F5F5;
    margin: 1px;
}

QTableWidget QScrollBar::handle:horizontal:hover {
    background: #707070;
}

QTableWidget QScrollBar::add-line:horizontal,
QTableWidget QScrollBar::sub-line:horizontal {
    width: 0px;
    border: none;
    background: none;
}

QTableWidget QScrollBar::add-page:horizontal,
QTableWidget QScrollBar::sub-page:horizontal {
    background: transparent;
    border: none;
    border-radius: 7px;
}

QLineEdit {
    background-color: #FFFFFF;
    color: #000000;
    border: 1px solid #CCCCCC;
    border-radius: 4px;
    padding: 3px 8px;
    min-height: 24px;
    max-height: 24px;
    font-size: 11px;
}

QLineEdit:focus {
    border: 1px solid #FB0082;
}

QLineEdit::placeholder {
    color: #999999;
}

QTextEdit {
    background-color: #1E1E1E;
    color: #F8F8F2;
    border: 1px solid #CCCCCC;
    border-radius: 4px;
    padding: 8px;
    font-size: 10px;
    font-family: 'Courier New', monospace;
}

QTextEdit:read-only {
    background-color: #1E1E1E;
    color: #F8F8F2;
}

QScrollBar:vertical {
    background: #F5F5F5;
    width: 12px;
    margin: 1px 1px 1px 0px;
    border: none;
}

QScrollBar::handle:vertical {
    background: #9B9B9B;
    min-height: 20px;
    border-radius: 999px;
    border: 2px solid #F5F5F5;
    margin: 2px;
}

QScrollBar::handle:vertical:hover {
    background: #707070;
}

QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical {
    height: 0px;
    border: none;
    background: none;
}

QScrollBar:horizontal {
    background: #F5F5F5;
    height: 12px;
    margin: 0px 1px 1px 1px;
    border: none;
}

QScrollBar::handle:horizontal {
    background: #9B9B9B;
    min-width: 20px;
    border-radius: 999px;
    border: 2px solid #F5F5F5;
    margin: 2px;
}

QScrollBar::handle:horizontal:hover {
    background: #707070;
}

QScrollBar::add-line:horizontal,
QScrollBar::sub-line:horizontal {
    width: 0px;
    border: none;
    background: none;
}

QSpinBox {
    background-color: #FFFFFF;
    color: #000000;
    border: 1px solid #CCCCCC;
    border-radius: 4px;
    padding: 3px 8px;
    min-height: 24px;
    max-height: 24px;
    font-size: 11px;
}

QSpinBox:hover {
    border: 1px solid #999999;
}

QSpinBox:focus {
    border: 1px solid #FB0082;
}

QSpinBox::up-button,
QSpinBox::down-button {
    width: 18px;
    border: none;
    background-color: #F5F5F5;
}

QSpinBox::up-button:hover,
QSpinBox::down-button:hover {
    background-color: #EFEFEF;
}

QSpinBox::up-arrow {
    image: url(__SPIN_UP_ARROW__);
    width: 8px;
    height: 8px;
}

QSpinBox::down-arrow {
    image: url(__SPIN_DOWN_ARROW__);
    width: 8px;
    height: 8px;
}

QSlider::groove:horizontal {
    border: none;
    height: 4px;
    background: transparent;
    border-radius: 2px;
}

QSlider::sub-page:horizontal {
    background: #FB0082;
    border-radius: 2px;
}

QSlider::add-page:horizontal {
    background: #eee;
}

QSlider::handle:horizontal {
    background: #FB0082;
    border: none;
    width: 12px;
    height: 10px;
    margin: -4px 0;
    border-radius: 5px;
}

QSlider::handle:horizontal:hover {
    background: #E60070;
    border: none;
}

QSlider::handle:horizontal:pressed {
    background: #D10064;
    border: none;
}

QSlider::groove:horizontal:disabled {
    background: #FFFFFF;
}

QSlider::add-page:horizontal:disabled {
    background: #FFFFFF;
}

QSlider::sub-page:horizontal:disabled {
    background: #DDDDDD;
}

QSlider::handle:horizontal:disabled {
    background: #CCCCCC;
    border: none;
}

/* Fallback for platforms that honor widget-state-first selector order */
QSlider:disabled::sub-page:horizontal {
    background: #DDDDDD;
}

QSlider:disabled::handle:horizontal {
    background: transparent;
}
"""


def apply_stylesheet(app):
    base_dir = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

    def _pick_icon(candidates):
        for path in candidates:
            if os.path.exists(path):
                return path.replace("\\", "/")
        return ""

    combo_arrow = _pick_icon(
        [
            os.path.join(base_dir, "icons", "arrow_down.svg"),
            os.path.join(base_dir, "..", "..", "..", "icons", "arrow_down.svg"),
        ]
    )
    spin_up_arrow = _pick_icon(
        [
            os.path.join(base_dir, "icons", "arrow_up.svg"),
            os.path.join(base_dir, "..", "..", "..", "icons", "arrow_up.svg"),
        ]
    )
    spin_down_arrow = _pick_icon(
        [
            os.path.join(base_dir, "icons", "arrow_down.svg"),
            os.path.join(base_dir, "..", "..", "..", "icons", "arrow_down.svg"),
        ]
    )

    def _image_rule(path):
        # 아이콘이 있으면 url(...), 없으면 none 으로 처리해
        # 빈 url() 로 인한 "QFSFileEngine::open: No file name specified" 경고를 막는다.
        return f"url({path})" if path else "none"

    stylesheet = (
        STYLESHEET
        .replace("url(__COMBO_ARROW__)", _image_rule(combo_arrow))
        .replace("url(__SPIN_UP_ARROW__)", _image_rule(spin_up_arrow))
        .replace("url(__SPIN_DOWN_ARROW__)", _image_rule(spin_down_arrow))
    )
    app.setStyleSheet(stylesheet)
