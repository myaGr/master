# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Dialog_import_test.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1076, 391)
        self.verticalLayout = QtWidgets.QVBoxLayout(Dialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget = QtWidgets.QWidget(Dialog)
        self.widget.setEnabled(True)
        self.widget.setObjectName("widget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.widget_2)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.comboBox_algorithmType = QtWidgets.QComboBox(self.widget_2)
        self.comboBox_algorithmType.setObjectName("comboBox_algorithmType")
        self.comboBox_algorithmType.addItem("")
        self.comboBox_algorithmType.addItem("")
        self.horizontalLayout.addWidget(self.comboBox_algorithmType)
        self.addTestFile = QtWidgets.QPushButton(self.widget_2)
        self.addTestFile.setObjectName("addTestFile")
        self.horizontalLayout.addWidget(self.addTestFile)
        spacerItem = QtWidgets.QSpacerItem(273, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.verticalLayout_3.addWidget(self.widget_2)
        self.groupBox_ins = QtWidgets.QGroupBox(self.widget)
        self.groupBox_ins.setEnabled(True)
        self.groupBox_ins.setObjectName("groupBox_ins")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.groupBox_ins)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget_3 = QtWidgets.QWidget(self.groupBox_ins)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_3)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pushBtn_SelectIns = QtWidgets.QPushButton(self.widget_3)
        self.pushBtn_SelectIns.setObjectName("pushBtn_SelectIns")
        self.horizontalLayout_2.addWidget(self.pushBtn_SelectIns)
        self.lineEdit_GetInsFile = QtWidgets.QLineEdit(self.widget_3)
        self.lineEdit_GetInsFile.setObjectName("lineEdit_GetInsFile")
        self.horizontalLayout_2.addWidget(self.lineEdit_GetInsFile)
        self.comboBox_GetInsType = QtWidgets.QComboBox(self.widget_3)
        self.comboBox_GetInsType.setObjectName("comboBox_GetInsType")
        self.comboBox_GetInsType.addItem("")
        self.comboBox_GetInsType.addItem("")
        self.horizontalLayout_2.addWidget(self.comboBox_GetInsType)
        self.verticalLayout_2.addWidget(self.widget_3)
        self.verticalLayout_3.addWidget(self.groupBox_ins)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.groupBox_gnss = QtWidgets.QGroupBox(self.widget)
        self.groupBox_gnss.setEnabled(False)
        self.groupBox_gnss.setObjectName("groupBox_gnss")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.groupBox_gnss)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.widget_4 = QtWidgets.QWidget(self.groupBox_gnss)
        self.widget_4.setEnabled(False)
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.widget_4)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_2 = QtWidgets.QLabel(self.widget_4)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_10.addWidget(self.label_2)
        self.dateEdit_testDate = QtWidgets.QDateEdit(self.widget_4)
        self.dateEdit_testDate.setObjectName("dateEdit_testDate")
        self.horizontalLayout_10.addWidget(self.dateEdit_testDate)
        spacerItem2 = QtWidgets.QSpacerItem(376, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_10.addItem(spacerItem2)
        self.verticalLayout_4.addWidget(self.widget_4)
        self.widget_5 = QtWidgets.QWidget(self.groupBox_gnss)
        self.widget_5.setEnabled(False)
        self.widget_5.setObjectName("widget_5")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widget_5)
        self.horizontalLayout_4.setContentsMargins(9, 6, 9, 6)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.pushBtn_SelectGnss = QtWidgets.QPushButton(self.widget_5)
        self.pushBtn_SelectGnss.setObjectName("pushBtn_SelectGnss")
        self.horizontalLayout_4.addWidget(self.pushBtn_SelectGnss)
        self.lineEdit_GetGnssFile = QtWidgets.QLineEdit(self.widget_5)
        self.lineEdit_GetGnssFile.setObjectName("lineEdit_GetGnssFile")
        self.horizontalLayout_4.addWidget(self.lineEdit_GetGnssFile)
        self.comboBox_GetGnssType = QtWidgets.QComboBox(self.widget_5)
        self.comboBox_GetGnssType.setObjectName("comboBox_GetGnssType")
        self.comboBox_GetGnssType.addItem("")
        self.comboBox_GetGnssType.addItem("")
        self.comboBox_GetGnssType.addItem("")
        self.horizontalLayout_4.addWidget(self.comboBox_GetGnssType)
        self.pushButton_setCsv = QtWidgets.QPushButton(self.widget_5)
        self.pushButton_setCsv.setEnabled(False)
        self.pushButton_setCsv.setObjectName("pushButton_setCsv")
        self.horizontalLayout_4.addWidget(self.pushButton_setCsv)
        self.verticalLayout_4.addWidget(self.widget_5)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem3)
        self.verticalLayout_3.addWidget(self.groupBox_gnss)
        self.verticalLayout.addWidget(self.widget)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "导入测试数据"))
        self.label.setText(_translate("Dialog", "算法类型*："))
        self.comboBox_algorithmType.setItemText(0, _translate("Dialog", "INS"))
        self.comboBox_algorithmType.setItemText(1, _translate("Dialog", "GNSS"))
        self.addTestFile.setText(_translate("Dialog", "新增测试文件"))
        self.groupBox_ins.setTitle(_translate("Dialog", "INS测试文件"))
        self.pushBtn_SelectIns.setText(_translate("Dialog", "选择文件"))
        self.comboBox_GetInsType.setItemText(0, _translate("Dialog", "导远自定义格式"))
        self.comboBox_GetInsType.setItemText(1, _translate("Dialog", "北云明文格式"))
        self.groupBox_gnss.setTitle(_translate("Dialog", "GNSS测试文件"))
        self.label_2.setText(_translate("Dialog", "测试日期(UTC)："))
        self.pushBtn_SelectGnss.setText(_translate("Dialog", "选择文件"))
        self.comboBox_GetGnssType.setItemText(0, _translate("Dialog", "NMEA"))
        self.comboBox_GetGnssType.setItemText(1, _translate("Dialog", "导远自定义-GPS"))
        self.comboBox_GetGnssType.setItemText(2, _translate("Dialog", "CSV自定义"))
        self.pushButton_setCsv.setText(_translate("Dialog", "配置CSV"))