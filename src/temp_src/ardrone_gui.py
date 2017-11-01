# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ardrone_gui.ui'
#
# Created: Thu Mar 16 01:02:14 2017
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(350, 530)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.ListWidget = QtGui.QListView(self.centralwidget)
        self.ListWidget.setObjectName(_fromUtf8("ListWidget"))
        self.verticalLayout_2.addWidget(self.ListWidget)
        self.verticalLayout.addLayout(self.verticalLayout_2)
        self.toggle_AR_cam = QtGui.QPushButton(self.centralwidget)
        self.toggle_AR_cam.setIconSize(QtCore.QSize(24, 24))
        self.toggle_AR_cam.setAutoExclusive(False)
        self.toggle_AR_cam.setAutoDefault(False)
        self.toggle_AR_cam.setObjectName(_fromUtf8("toggle_AR_cam"))
        self.verticalLayout.addWidget(self.toggle_AR_cam)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.toggle_AR_cam.setText(_translate("MainWindow", "Toggle Camera", None))

