import sys
from PyQt5.QtWidgets import QApplication
from gui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName('CubeSat Ground Station')
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
