import sys

from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox
)
from PyQt5.uic import loadUi


class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        loadUi("rolling_gui.ui", self)

        self.startButton.setText('Text Changed')
        self.world_x.display(1234)
        self.startButton.clicked.connect(self.about)

    def about(self):
        QMessageBox.about(
            self,
            "About Sample Editor",
            "<p>A sample text editor app built with:</p>"
            "<p>- PyQt</p>"
            "<p>- Qt Designer</p>"
            "<p>- Python</p>",
        )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()

    win.show()

    sys.exit(app.exec())