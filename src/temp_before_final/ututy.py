import sys
from PyQt4 import QtGui,QtCore
class LayoutTest(QtGui.QWidget):
    def __init__(self):
        super(LayoutTest, self).__init__()
        self.first_box  = QtGui.QVBoxLayout()
        self.second_box = QtGui.QVBoxLayout()
        self.third_box  = QtGui.QVBoxLayout()
        self.fourth_box = QtGui.QVBoxLayout()

        self.zvbox = QtGui.QVBoxLayout()

        vbox = QtGui.QVBoxLayout()
        vbox.addLayout(self.first_box)
        vbox.addLayout(self.second_box)
        vbox.addLayout(self.third_box)
        vbox.addLayout(self.fourth_box)
        self.setLayout(vbox)

        self.first_view()

        self.setGeometry(300, 200, 400, 300)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.close()

    def first_view(self):

        self.next1 = QtGui.QPushButton("NEXT 2")

        self.first_box.addWidget(self.next1)

        self.connect(self.next1, QtCore.SIGNAL("clicked()"), self.first_second_view)

    def first_second_view(self):
        self.remove_first_view()
        self.second_view()

    def second_view(self):


        self.next3 = QtGui.QPushButton("NEXT 3")
        self.back1 = QtGui.QPushButton("BACK 1")

        self.second_box.addWidget(self.next3)
        self.second_box.addWidget(self.back1)

        self.connect(self.next3, QtCore.SIGNAL("clicked()"), self.second_third_view)
        self.connect(self.back1, QtCore.SIGNAL("clicked()"), self.second_first_view)

    def second_first_view(self):
        self.remove_second_view()
        self.first_view()



    def second_third_view(self):
        self.remove_second_view()
        self.third_view()




    def third_view(self):

        self.next4 = QtGui.QPushButton("NEXT 4")
        self.back2 = QtGui.QPushButton("BACK 2")

        self.third_box.addWidget(self.next4)
        self.third_box.addWidget(self.back2)

        self.connect(self.next4, QtCore.SIGNAL("clicked()"), self.third_fourth_view)
        self.connect(self.back2, QtCore.SIGNAL("clicked()"), self.third_second_view)

    def third_second_view(self):
        self.remove_third_view()
        self.second_view()


    def third_fourth_view(self):
        self.remove_third_view()
        self.back3 = QtGui.QPushButton("BACK 3")

        self.fourth_box.addWidget(self.back3)

        self.connect(self.back3, QtCore.SIGNAL("clicked()"), self.fourth_third_view)

    def fourth_third_view(self):
        self.remove_fourth_view()
        self.third_view()

    def remove_first_view(self):
        for cnt in reversed(range(self.first_box.count())):
            # takeAt does both the jobs of itemAt and removeWidget
            # namely it removes an item and returns it
            widget = self.first_box.takeAt(cnt).widget()

            if widget is not None:
                # widget will be None if the item is a layout
                widget.deleteLater()

    def remove_second_view(self):
        for cnt in reversed(range(self.second_box.count())):
            # takeAt does both the jobs of itemAt and removeWidget
            # namely it removes an item and returns it
            widget = self.second_box.takeAt(cnt).widget()

            if widget is not None:
                # widget will be None if the item is a layout
                widget.deleteLater()

    def remove_third_view(self):
        for cnt in reversed(range(self.third_box.count())):
            # takeAt does both the jobs of itemAt and removeWidget
            # namely it removes an item and returns it
            widget = self.third_box.takeAt(cnt).widget()

            if widget is not None:
                # widget will be None if the item is a layout
                widget.deleteLater()

    def remove_fourth_view(self):
        for cnt in reversed(range(self.fourth_box.count())):
            # takeAt does both the jobs of itemAt and removeWidget
            # namely it removes an item and returns it
            widget = self.fourth_box.takeAt(cnt).widget()

            if widget is not None:
                # widget will be None if the item is a layout
                widget.deleteLater()


    def checkItems(self):
        QtGui.QMessageBox.information(self, 'Count',"You have %s Items in Layout" % self.dvbox.count(), QtGui.QMessageBox.Ok)

def run():

    app = QtGui.QApplication(sys.argv)
    ex = LayoutTest()
    ex.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    run()