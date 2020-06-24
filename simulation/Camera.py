# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Camera.ui'
#
# Created by: PyQt5 UI code generator 5.13.0
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainCamera(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(984, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.Raw_camera = QtWidgets.QLabel(self.centralwidget)
        self.Raw_camera.setGeometry(QtCore.QRect(34, 50, 256, 256))
        self.Raw_camera.setObjectName("Raw_camera")
        self.filtered_camera = QtWidgets.QLabel(self.centralwidget)
        self.filtered_camera.setGeometry(QtCore.QRect(540, 50, 256, 256))
        self.filtered_camera.setObjectName("filtered_camera")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 984, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.Raw_camera.setText(_translate("MainWindow", "TextLabel"))
        self.filtered_camera.setText(_translate("MainWindow", "TextLabel"))
