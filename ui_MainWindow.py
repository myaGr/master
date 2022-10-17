# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(402, 422)
        MainWindow.setMinimumSize(QtCore.QSize(0, 0))
        self.pathlist =[]
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit_filePath = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_filePath.setMinimumSize(QtCore.QSize(0, 25))
        self.lineEdit_filePath.setObjectName("lineEdit_filePath")
        self.horizontalLayout.addWidget(self.lineEdit_filePath)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setSpacing(50)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pushBtn_startDataParse = QtWidgets.QPushButton(self.centralwidget)
        self.pushBtn_startDataParse.setMinimumSize(QtCore.QSize(0, 30))
        self.pushBtn_startDataParse.setObjectName("pushBtn_startDataParse")
        self.horizontalLayout_2.addWidget(self.pushBtn_startDataParse)
        self.pushBtn_clearMsg = QtWidgets.QPushButton(self.centralwidget)
        self.pushBtn_clearMsg.setMinimumSize(QtCore.QSize(0, 30))
        self.pushBtn_clearMsg.setObjectName("pushBtn_clearMsg")
        self.horizontalLayout_2.addWidget(self.pushBtn_clearMsg)
        self.pushBtn_selectFile = QtWidgets.QPushButton(self.centralwidget)
        self.pushBtn_selectFile.setMinimumSize(QtCore.QSize(0, 30))
        self.pushBtn_selectFile.setObjectName("pushBtn_selectFile")
        self.horizontalLayout_2.addWidget(self.pushBtn_selectFile)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.textBrowser_showMsg = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_showMsg.setObjectName("textBrowser_showMsg")
        self.verticalLayout.addWidget(self.textBrowser_showMsg)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "数据帧解包工具 Powered by 算法部 2022/7/29"))
        self.label.setText(_translate("MainWindow", "文件路径"))
        self.pushBtn_startDataParse.setText(_translate("MainWindow", "开始解析"))
        self.pushBtn_clearMsg.setText(_translate("MainWindow", "清空消息"))
        self.pushBtn_selectFile.setText(_translate("MainWindow", "选择文件"))
