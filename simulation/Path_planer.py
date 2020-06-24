# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Path_planer.ui'
#
# Created by: PyQt5 UI code generator 5.13.0
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Path_window(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(738, 509)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(600, 120, 113, 22))
        self.lineEdit.setObjectName("lineEdit")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(510, 120, 71, 21))
        self.label.setObjectName("label")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QtCore.QRect(600, 160, 113, 22))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(484, 160, 81, 20))
        self.label_2.setObjectName("label_2")
        self.planer_field = QtWidgets.QLabel(self.centralwidget)
        self.planer_field.setGeometry(QtCore.QRect(39, 39, 410, 340))
        self.planer_field.setMouseTracking(True)
        self.planer_field.setObjectName("planer_field")
        self.create_path = QtWidgets.QPushButton(self.centralwidget)
        self.create_path.setGeometry(QtCore.QRect(50, 400, 93, 28))
        self.create_path.setObjectName("create_path")
        self.reset_path = QtWidgets.QPushButton(self.centralwidget)
        self.reset_path.setGeometry(QtCore.QRect(160, 400, 93, 28))
        self.reset_path.setObjectName("reset_path")
        self.auto_planer = QtWidgets.QPushButton(self.centralwidget)
        self.auto_planer.setGeometry(QtCore.QRect(322, 400, 111, 28))
        self.auto_planer.setObjectName("auto_planer")
        self.lineEdit.raise_()
        self.label.raise_()
        self.lineEdit_2.raise_()
        self.label_2.raise_()
        self.create_path.raise_()
        self.reset_path.raise_()
        self.auto_planer.raise_()
        self.planer_field.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 738, 26))
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
        self.label.setText(_translate("MainWindow", "start point"))
        self.label_2.setText(_translate("MainWindow", "mission area"))
        self.create_path.setText(_translate("MainWindow", "create path"))
        self.reset_path.setText(_translate("MainWindow", "reset path"))
        self.auto_planer.setText(_translate("MainWindow", "automatic planer"))
