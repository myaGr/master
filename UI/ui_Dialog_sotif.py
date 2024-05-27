# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Dialog_sotif.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(610, 182)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(Dialog)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget = QtWidgets.QWidget(Dialog)
        self.widget.setObjectName("widget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton_selectBDDB = QtWidgets.QPushButton(self.widget_2)
        self.pushButton_selectBDDB.setObjectName("pushButton_selectBDDB")
        self.horizontalLayout.addWidget(self.pushButton_selectBDDB)
        self.lineEdit_PLPath = QtWidgets.QLineEdit(self.widget_2)
        self.lineEdit_PLPath.setObjectName("lineEdit_PLPath")
        self.horizontalLayout.addWidget(self.lineEdit_PLPath)
        self.pushButton_exportCsv = QtWidgets.QPushButton(self.widget_2)
        self.pushButton_exportCsv.setObjectName("pushButton_exportCsv")
        self.horizontalLayout.addWidget(self.pushButton_exportCsv)
        self.verticalLayout_3.addWidget(self.widget_2)
        self.groupBox = QtWidgets.QGroupBox(self.widget)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget_ins = QtWidgets.QWidget(self.groupBox)
        self.widget_ins.setObjectName("widget_ins")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget_ins)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.checkBox_pos_horizontal_pl = QtWidgets.QCheckBox(self.widget_ins)
        self.checkBox_pos_horizontal_pl.setEnabled(True)
        self.checkBox_pos_horizontal_pl.setCheckable(True)
        self.checkBox_pos_horizontal_pl.setChecked(True)
        self.checkBox_pos_horizontal_pl.setObjectName("checkBox_pos_horizontal_pl")
        self.horizontalLayout_3.addWidget(self.checkBox_pos_horizontal_pl)
        self.checkBox_pos_lateral_pl = QtWidgets.QCheckBox(self.widget_ins)
        self.checkBox_pos_lateral_pl.setChecked(False)
        self.checkBox_pos_lateral_pl.setObjectName("checkBox_pos_lateral_pl")
        self.horizontalLayout_3.addWidget(self.checkBox_pos_lateral_pl)
        self.checkBox_pos_longitudinal_pl = QtWidgets.QCheckBox(self.widget_ins)
        self.checkBox_pos_longitudinal_pl.setChecked(False)
        self.checkBox_pos_longitudinal_pl.setObjectName("checkBox_pos_longitudinal_pl")
        self.horizontalLayout_3.addWidget(self.checkBox_pos_longitudinal_pl)
        self.checkBox_pos_vertical_pl = QtWidgets.QCheckBox(self.widget_ins)
        self.checkBox_pos_vertical_pl.setChecked(False)
        self.checkBox_pos_vertical_pl.setObjectName("checkBox_pos_vertical_pl")
        self.horizontalLayout_3.addWidget(self.checkBox_pos_vertical_pl)
        spacerItem = QtWidgets.QSpacerItem(35, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.verticalLayout.addWidget(self.widget_ins)
        self.buttonBox = QtWidgets.QDialogButtonBox(self.groupBox)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)
        self.verticalLayout_3.addWidget(self.groupBox)
        self.verticalLayout_2.addWidget(self.widget)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButton_selectBDDB.setText(_translate("Dialog", "选择文件"))
        self.lineEdit_PLPath.setPlaceholderText(_translate("Dialog", "Feather File Only"))
        self.pushButton_exportCsv.setText(_translate("Dialog", "导出csv"))
        self.groupBox.setTitle(_translate("Dialog", "PL时间序列分布图"))
        self.checkBox_pos_horizontal_pl.setText(_translate("Dialog", "水平误差"))
        self.checkBox_pos_lateral_pl.setText(_translate("Dialog", "横向误差"))
        self.checkBox_pos_longitudinal_pl.setText(_translate("Dialog", "纵向误差"))
        self.checkBox_pos_vertical_pl.setText(_translate("Dialog", "垂向误差"))