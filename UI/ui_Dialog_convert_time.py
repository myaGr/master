# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Dialog_convert_time.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(796, 386)
        self.verticalLayout = QtWidgets.QVBoxLayout(Dialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.widget = QtWidgets.QWidget(Dialog)
        self.widget.setObjectName("widget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget_5 = QtWidgets.QWidget(self.widget)
        self.widget_5.setObjectName("widget_5")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widget_5)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.pushButton_getCurrentTime = QtWidgets.QPushButton(self.widget_5)
        self.pushButton_getCurrentTime.setObjectName("pushButton_getCurrentTime")
        self.horizontalLayout_4.addWidget(self.pushButton_getCurrentTime)
        self.lineEdit_getCurrentTime = QtWidgets.QLineEdit(self.widget_5)
        self.lineEdit_getCurrentTime.setObjectName("lineEdit_getCurrentTime")
        self.horizontalLayout_4.addWidget(self.lineEdit_getCurrentTime)
        self.lineEdit_getCurrentTime_Unix = QtWidgets.QLineEdit(self.widget_5)
        self.lineEdit_getCurrentTime_Unix.setObjectName("lineEdit_getCurrentTime_Unix")
        self.horizontalLayout_4.addWidget(self.lineEdit_getCurrentTime_Unix)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.verticalLayout_2.addWidget(self.widget_5)
        self.widget_4 = QtWidgets.QWidget(self.widget)
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout(self.widget_4)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_2 = QtWidgets.QLabel(self.widget_4)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_7.addWidget(self.label_2)
        self.comboBox_timeType = QtWidgets.QComboBox(self.widget_4)
        self.comboBox_timeType.setObjectName("comboBox_timeType")
        self.comboBox_timeType.addItem("")
        self.comboBox_timeType.addItem("")
        self.comboBox_timeType.addItem("")
        self.comboBox_timeType.addItem("")
        self.comboBox_timeType.addItem("")
        self.comboBox_timeType.addItem("")
        self.horizontalLayout_7.addWidget(self.comboBox_timeType)
        spacerItem1 = QtWidgets.QSpacerItem(538, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_7.addItem(spacerItem1)
        self.verticalLayout_2.addWidget(self.widget_4)
        self.widget_6 = QtWidgets.QWidget(self.widget)
        self.widget_6.setObjectName("widget_6")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget_6)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.widget_unixTime = QtWidgets.QWidget(self.widget_6)
        self.widget_unixTime.setObjectName("widget_unixTime")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout(self.widget_unixTime)
        self.horizontalLayout_6.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_8 = QtWidgets.QLabel(self.widget_unixTime)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_6.addWidget(self.label_8)
        self.lineEdit_unixTime = QtWidgets.QLineEdit(self.widget_unixTime)
        self.lineEdit_unixTime.setObjectName("lineEdit_unixTime")
        self.horizontalLayout_6.addWidget(self.lineEdit_unixTime)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem2)
        self.verticalLayout_3.addWidget(self.widget_unixTime)
        self.widget_pkTime = QtWidgets.QWidget(self.widget_6)
        self.widget_pkTime.setEnabled(True)
        self.widget_pkTime.setObjectName("widget_pkTime")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.widget_pkTime)
        self.horizontalLayout_8.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_17 = QtWidgets.QLabel(self.widget_pkTime)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_8.addWidget(self.label_17)
        self.dateTimeEdit_pkTime = QtWidgets.QDateTimeEdit(self.widget_pkTime)
        self.dateTimeEdit_pkTime.setDateTime(QtCore.QDateTime(QtCore.QDate(2000, 1, 1), QtCore.QTime(0, 0, 0)))
        self.dateTimeEdit_pkTime.setTime(QtCore.QTime(0, 0, 0))
        self.dateTimeEdit_pkTime.setObjectName("dateTimeEdit_pkTime")
        self.horizontalLayout_8.addWidget(self.dateTimeEdit_pkTime)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem3)
        self.verticalLayout_3.addWidget(self.widget_pkTime)
        self.widget_utcTime = QtWidgets.QWidget(self.widget_6)
        self.widget_utcTime.setEnabled(True)
        self.widget_utcTime.setObjectName("widget_utcTime")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout(self.widget_utcTime)
        self.horizontalLayout_12.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_14 = QtWidgets.QLabel(self.widget_utcTime)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_12.addWidget(self.label_14)
        self.dateTimeEdit_utcTime = QtWidgets.QDateTimeEdit(self.widget_utcTime)
        self.dateTimeEdit_utcTime.setDateTime(QtCore.QDateTime(QtCore.QDate(2000, 1, 1), QtCore.QTime(0, 0, 0)))
        self.dateTimeEdit_utcTime.setTime(QtCore.QTime(0, 0, 0))
        self.dateTimeEdit_utcTime.setObjectName("dateTimeEdit_utcTime")
        self.horizontalLayout_12.addWidget(self.dateTimeEdit_utcTime)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_12.addItem(spacerItem4)
        self.verticalLayout_3.addWidget(self.widget_utcTime)
        self.widget_gpsTime = QtWidgets.QWidget(self.widget_6)
        self.widget_gpsTime.setEnabled(True)
        self.widget_gpsTime.setObjectName("widget_gpsTime")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.widget_gpsTime)
        self.horizontalLayout_10.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_16 = QtWidgets.QLabel(self.widget_gpsTime)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_10.addWidget(self.label_16)
        self.dateTimeEdit_gpsTime = QtWidgets.QDateTimeEdit(self.widget_gpsTime)
        self.dateTimeEdit_gpsTime.setDateTime(QtCore.QDateTime(QtCore.QDate(2000, 1, 1), QtCore.QTime(0, 0, 0)))
        self.dateTimeEdit_gpsTime.setTime(QtCore.QTime(0, 0, 0))
        self.dateTimeEdit_gpsTime.setObjectName("dateTimeEdit_gpsTime")
        self.horizontalLayout_10.addWidget(self.dateTimeEdit_gpsTime)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_10.addItem(spacerItem5)
        self.verticalLayout_3.addWidget(self.widget_gpsTime)
        self.widget_itows = QtWidgets.QWidget(self.widget_6)
        self.widget_itows.setEnabled(True)
        self.widget_itows.setObjectName("widget_itows")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.widget_itows)
        self.horizontalLayout_3.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_11 = QtWidgets.QLabel(self.widget_itows)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_3.addWidget(self.label_11)
        self.label_6 = QtWidgets.QLabel(self.widget_itows)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_3.addWidget(self.label_6)
        self.lineEdit_gpsWeek = QtWidgets.QLineEdit(self.widget_itows)
        self.lineEdit_gpsWeek.setObjectName("lineEdit_gpsWeek")
        self.horizontalLayout_3.addWidget(self.lineEdit_gpsWeek)
        self.label_3 = QtWidgets.QLabel(self.widget_itows)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.lineEdit_gpsItow = QtWidgets.QLineEdit(self.widget_itows)
        self.lineEdit_gpsItow.setObjectName("lineEdit_gpsItow")
        self.horizontalLayout_3.addWidget(self.lineEdit_gpsItow)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem6)
        self.verticalLayout_3.addWidget(self.widget_itows)
        self.widget_doy = QtWidgets.QWidget(self.widget_6)
        self.widget_doy.setEnabled(True)
        self.widget_doy.setObjectName("widget_doy")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.widget_doy)
        self.horizontalLayout_5.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_12 = QtWidgets.QLabel(self.widget_doy)
        self.label_12.setObjectName("label_12")
        self.horizontalLayout_5.addWidget(self.label_12)
        self.label_7 = QtWidgets.QLabel(self.widget_doy)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_5.addWidget(self.label_7)
        self.lineEdit_year = QtWidgets.QLineEdit(self.widget_doy)
        self.lineEdit_year.setObjectName("lineEdit_year")
        self.horizontalLayout_5.addWidget(self.lineEdit_year)
        self.label_4 = QtWidgets.QLabel(self.widget_doy)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_5.addWidget(self.label_4)
        self.lineEdit_doy = QtWidgets.QLineEdit(self.widget_doy)
        self.lineEdit_doy.setObjectName("lineEdit_doy")
        self.horizontalLayout_5.addWidget(self.lineEdit_doy)
        spacerItem7 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem7)
        self.verticalLayout_3.addWidget(self.widget_doy)
        self.widget_leapSecond = QtWidgets.QWidget(self.widget_6)
        self.widget_leapSecond.setEnabled(True)
        self.widget_leapSecond.setObjectName("widget_leapSecond")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget_leapSecond)
        self.horizontalLayout_2.setContentsMargins(-1, 0, -1, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_13 = QtWidgets.QLabel(self.widget_leapSecond)
        self.label_13.setEnabled(True)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_2.addWidget(self.label_13)
        self.label_leapSecond = QtWidgets.QLabel(self.widget_leapSecond)
        self.label_leapSecond.setEnabled(True)
        self.label_leapSecond.setObjectName("label_leapSecond")
        self.horizontalLayout_2.addWidget(self.label_leapSecond)
        spacerItem8 = QtWidgets.QSpacerItem(617, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem8)
        self.verticalLayout_3.addWidget(self.widget_leapSecond)
        self.verticalLayout_2.addWidget(self.widget_6)
        self.widget_2 = QtWidgets.QWidget(self.widget)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget_2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem9 = QtWidgets.QSpacerItem(227, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem9)
        self.pushButton_convertTime = QtWidgets.QPushButton(self.widget_2)
        self.pushButton_convertTime.setObjectName("pushButton_convertTime")
        self.horizontalLayout.addWidget(self.pushButton_convertTime)
        spacerItem10 = QtWidgets.QSpacerItem(226, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem10)
        self.verticalLayout_2.addWidget(self.widget_2)
        self.verticalLayout.addWidget(self.widget)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "时间转换工具"))
        self.pushButton_getCurrentTime.setText(_translate("Dialog", "获取当前时间"))
        self.lineEdit_getCurrentTime.setPlaceholderText(_translate("Dialog", "YY/MM/dd hh:mm:ss"))
        self.lineEdit_getCurrentTime_Unix.setPlaceholderText(_translate("Dialog", "UnixTime: second"))
        self.label_2.setText(_translate("Dialog", "选择输入时间类型："))
        self.comboBox_timeType.setItemText(0, _translate("Dialog", "Unix时间戳"))
        self.comboBox_timeType.setItemText(1, _translate("Dialog", "北京时间"))
        self.comboBox_timeType.setItemText(2, _translate("Dialog", "UTC时间"))
        self.comboBox_timeType.setItemText(3, _translate("Dialog", "GPS时间"))
        self.comboBox_timeType.setItemText(4, _translate("Dialog", "周内秒"))
        self.comboBox_timeType.setItemText(5, _translate("Dialog", "年积日"))
        self.label_8.setText(_translate("Dialog", "【Unix时间戳】"))
        self.lineEdit_unixTime.setPlaceholderText(_translate("Dialog", "second"))
        self.label_17.setText(_translate("Dialog", "【北京时间】  "))
        self.dateTimeEdit_pkTime.setDisplayFormat(_translate("Dialog", "yyyy/M/d H:mm:ss"))
        self.label_14.setText(_translate("Dialog", "【UTC时间】  "))
        self.dateTimeEdit_utcTime.setDisplayFormat(_translate("Dialog", "yyyy/M/d H:mm:ss"))
        self.label_16.setText(_translate("Dialog", "【GPS时间】  "))
        self.dateTimeEdit_gpsTime.setDisplayFormat(_translate("Dialog", "yyyy/M/d H:mm:ss"))
        self.label_11.setText(_translate("Dialog", "【周内秒】    "))
        self.label_6.setText(_translate("Dialog", "周"))
        self.label_3.setText(_translate("Dialog", "周内秒"))
        self.lineEdit_gpsItow.setPlaceholderText(_translate("Dialog", "second"))
        self.label_12.setText(_translate("Dialog", "【年积日】    "))
        self.label_7.setText(_translate("Dialog", "年"))
        self.label_4.setText(_translate("Dialog", "年积日"))
        self.label_13.setText(_translate("Dialog", "【跳秒】      "))
        self.label_leapSecond.setText(_translate("Dialog", "18"))
        self.pushButton_convertTime.setText(_translate("Dialog", "转换"))
