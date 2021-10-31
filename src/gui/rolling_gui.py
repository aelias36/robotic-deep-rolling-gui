import sys
import os

from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox
)
from PyQt5.uic import loadUi


class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        abspath = os.path.join(os.path.dirname(os.path.realpath(__file__)), "rolling_gui.ui")
        loadUi(abspath, self)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()

    win.show()

    sys.exit(app.exec())