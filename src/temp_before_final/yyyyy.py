
import sys
from functools import partial
from PyQt4.QtGui import *
from PyQt4.QtCore import *


class Form1(QWidget):
    showForm2Signal = pyqtSignal()

    def __init__(self, parent=None):
        super(Form1, self).__init__(parent)
        self.newGameButton = QPushButton("New Game", self)
        self.quitButton = QPushButton("Quit", self)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<html>My Game<br>Start Page</html>"))
        layout.addWidget(self.newGameButton)
        layout.addWidget(self.quitButton)
        self.newGameButton.clicked.connect(self.showForm2Signal.emit)
        self.quitButton.clicked.connect(qApp.quit)


class Form2(QWidget):
    showForm1Signal = pyqtSignal()

    def __init__(self, parent=None):
        super(Form2, self).__init__(parent)
        self.backButton = QPushButton("Back", self)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("New Game Started!"))
        layout.addWidget(self.backButton)
        self.backButton.clicked.connect(self.showForm1Signal.emit)


class MainWidget(QWidget):
    def __init__(self, parent=None):
        super(MainWidget, self).__init__(parent)
        self.stack = QStackedWidget()
        layout = QVBoxLayout(self)
        layout.addWidget(self.stack)
        self.form1 = Form1(self)
        self.form2 = Form2(self)
        self.stack.addWidget(self.form1)
        self.stack.addWidget(self.form2)
        self.form1.showForm2Signal.connect(partial(self.stack.setCurrentWidget,
                                               self.form2))
        self.form2.showForm1Signal.connect(partial(self.stack.setCurrentWidget,
                                               self.form1))
        self.stack.setCurrentWidget(self.form1)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWidget()
    w.show()
    app.exec_()
    sys.exit()