# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Dialog_gnss_multi_plot.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(638, 398)
        self.verticalLayout = QtWidgets.QVBoxLayout(Dialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget = QtWidgets.QWidget(Dialog)
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget_2)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.checkBox_timeGap = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_timeGap.setChecked(True)
        self.checkBox_timeGap.setObjectName("checkBox_timeGap")
        self.verticalLayout_2.addWidget(self.checkBox_timeGap)
        self.checkBox_ageDiff = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_ageDiff.setChecked(False)
        self.checkBox_ageDiff.setObjectName("checkBox_ageDiff")
        self.verticalLayout_2.addWidget(self.checkBox_ageDiff)
        self.checkBox_gpsQualityPosError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_gpsQualityPosError.setChecked(True)
        self.checkBox_gpsQualityPosError.setObjectName("checkBox_gpsQualityPosError")
        self.verticalLayout_2.addWidget(self.checkBox_gpsQualityPosError)
        self.checkBox_posLateralError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_posLateralError.setObjectName("checkBox_posLateralError")
        self.verticalLayout_2.addWidget(self.checkBox_posLateralError)
        self.checkBox_posLongitudinalError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_posLongitudinalError.setObjectName("checkBox_posLongitudinalError")
        self.verticalLayout_2.addWidget(self.checkBox_posLongitudinalError)
        self.checkBox_posHorizontalError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_posHorizontalError.setObjectName("checkBox_posHorizontalError")
        self.verticalLayout_2.addWidget(self.checkBox_posHorizontalError)
        self.checkBox_elevationError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_elevationError.setObjectName("checkBox_elevationError")
        self.verticalLayout_2.addWidget(self.checkBox_elevationError)
        self.checkBox_velError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_velError.setChecked(False)
        self.checkBox_velError.setObjectName("checkBox_velError")
        self.verticalLayout_2.addWidget(self.checkBox_velError)
        self.checkBox_headingError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_headingError.setChecked(False)
        self.checkBox_headingError.setObjectName("checkBox_headingError")
        self.verticalLayout_2.addWidget(self.checkBox_headingError)
        self.checkBox_doubleheadingError = QtWidgets.QCheckBox(self.widget_2)
        self.checkBox_doubleheadingError.setChecked(False)
        self.checkBox_doubleheadingError.setObjectName("checkBox_doubleheadingError")
        self.verticalLayout_2.addWidget(self.checkBox_doubleheadingError)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        self.horizontalLayout.addWidget(self.widget_2)
        self.widget_3 = QtWidgets.QWidget(self.widget)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout.addWidget(self.widget_3)
        self.verticalLayout.addWidget(self.widget)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept) # type: ignore
        self.buttonBox.rejected.connect(Dialog.reject) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "GNSS综合误差统计图配置"))
        self.checkBox_timeGap.setText(_translate("Dialog", "历元间隔分布图"))
        self.checkBox_ageDiff.setText(_translate("Dialog", "差分龄期统计图"))
        self.checkBox_gpsQualityPosError.setText(_translate("Dialog", "解状态水平误差序列图"))
        self.checkBox_posLateralError.setText(_translate("Dialog", "位置横向误差历元分布图"))
        self.checkBox_posLongitudinalError.setText(_translate("Dialog", "位置纵向误差历元分布图"))
        self.checkBox_posHorizontalError.setText(_translate("Dialog", "位置水平误差历元分布图"))
        self.checkBox_elevationError.setText(_translate("Dialog", "高程误差历元分布图"))
        self.checkBox_velError.setText(_translate("Dialog", "速度误差(前向)历元分布图"))
        self.checkBox_headingError.setText(_translate("Dialog", "姿态误差(航向)历元分布图"))
        self.checkBox_doubleheadingError.setText(_translate("Dialog", "双天线航向误差历元分布图"))
