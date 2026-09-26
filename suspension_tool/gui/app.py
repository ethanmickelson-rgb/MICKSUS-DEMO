"""Application entry point:  python -m suspension_tool.gui"""

import sys

from PySide6.QtWidgets import QApplication, QSplashScreen

from .. import __version__
from .branding import app_icon, apply_dark_theme, splash_pixmap


def main() -> None:
    app = QApplication(sys.argv)
    apply_dark_theme(app)
    app.setWindowIcon(app_icon())
    splash = QSplashScreen(splash_pixmap(__version__))
    splash.show()
    app.processEvents()          # paint the splash before the heavy import
    from .main_window import MainWindow   # pulls in pyvista/vtk — slow
    win = MainWindow()
    win.resize(1500, 900)
    win.show()
    splash.finish(win)
    sys.exit(app.exec())
