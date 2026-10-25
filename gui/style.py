"""Minimal black visual theme for the CubeSat Ground Station."""
BACKGROUND = '#050505'
PANEL = '#0d0d0d'
PANEL_ALT = '#121212'
BORDER = '#2b2b2b'
TEXT = '#f2f2f2'
MUTED = '#9a9a9a'
AXIS_COLORS = ('#2f8fff', '#4ee58a', '#ffd24a')

STYLESHEET = r'''
QWidget { color: #f2f2f2; font-family: "Segoe UI", "DejaVu Sans"; font-size: 10pt; }
QMainWindow, QWidget#Root, QWidget#Page, QScrollArea { background: #050505; }
QLabel { background: transparent; }
QLabel#Brand { font-size: 18pt; font-weight: 700; color: #ffffff; }
QLabel#Subtitle { color: #909090; font-size: 9pt; }
QLabel#Muted { color: #9a9a9a; }
QLabel#CardTitle { font-size: 11pt; font-weight: 650; color: #f0f0f0; }
QLabel#SectionValue { font-size: 12pt; font-weight: 650; color: #ffffff; }
QLabel#Badge { color: #6ef0a0; background: #0d2115; border: 1px solid #1e5b35;
               border-radius: 5px; padding: 6px 10px; font-size: 9pt; font-weight: 600; }
QLabel#BadgeWarn { color: #ffd27c; background: #241b0c; border: 1px solid #6c4f1f;
                   border-radius: 5px; padding: 6px 10px; font-size: 9pt; font-weight: 600; }
QLabel#BadgeDanger { color: #ff9ca6; background: #2c1014; border: 1px solid #7d2632;
                     border-radius: 5px; padding: 6px 10px; font-size: 9pt; font-weight: 700; }
QFrame#Card { background: #0d0d0d; border: 1px solid #2b2b2b; border-radius: 8px; }
QFrame#Header { background: #080808; border-bottom: 1px solid #292929; }
QFrame#MiniPanel { background: #101010; border: 1px solid #292929; border-radius: 6px; }
QPushButton { background: #151515; border: 1px solid #343434; border-radius: 5px;
              padding: 7px 12px; font-weight: 550; min-height: 20px; }
QPushButton:hover { background: #1d1d1d; border-color: #5a5a5a; }
QPushButton:pressed { background: #0f0f0f; }
QPushButton:disabled { color: #5e5e5e; background: #0c0c0c; border-color: #232323; }
QPushButton#Primary { background: #126de5; color: white; border-color: #2384ff; font-weight: 650; }
QPushButton#Primary:hover { background: #1e7df0; }
QPushButton#Emergency { background: #d92e36; color: white; border: 1px solid #ff5159;
                        font-size: 10pt; font-weight: 750; padding: 8px 18px; }
QPushButton#Emergency:hover { background: #ef3942; }
QPushButton#DangerOutline { color: #ff9ca6; border-color: #80333c; }
QDoubleSpinBox, QSpinBox, QComboBox, QLineEdit { background: #0a0a0a; border: 1px solid #353535;
    border-radius: 4px; padding: 5px 7px; min-height: 20px; selection-background-color: #1e5ea8; }
QComboBox QAbstractItemView { background: #111111; border: 1px solid #353535; selection-background-color: #1e5ea8; }
QPlainTextEdit { background: #080808; border: 1px solid #2d2d2d; border-radius: 5px;
                 font-family: "Consolas", "DejaVu Sans Mono"; font-size: 9pt; padding: 6px; }
QScrollArea { border: none; }
QScrollBar:vertical { background: #080808; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #333333; min-height: 30px; border-radius: 4px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QStatusBar { color: #8f8f8f; background: #080808; border-top: 1px solid #292929; font-size: 9pt; }
QToolTip { color: #f2f2f2; background: #171717; border: 1px solid #454545; padding: 5px; }
'''
