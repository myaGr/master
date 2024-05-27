import datetime

from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import Qt
import UI.ui_Dialog_convert_time
from Utils import timeExchangeMgr
import global_var


# 时间转换工具窗口
class DialogConvertTime(QDialog, UI.ui_Dialog_convert_time.Ui_Dialog):
    def __init__(self):
        super(DialogConvertTime, self).__init__()
        self.setupUi(self)

        self.pushButton_getCurrentTime.clicked.connect(self.getCurrentTime)  # 获取当前时间
        self.comboBox_timeType.currentIndexChanged.connect(self.on_clicked_select_time_type)  # 选择时间类
        self.pushButton_convertTime.clicked.connect(self.time_convert)  # 时间转换

        print(global_var.get_value("TEST_DATE"))

    def getCurrentTime(self):
        current_time = timeExchangeMgr.setCurrentTimer(datetype="date", format='%Y/%m/%d %H:%M:%S')
        self.lineEdit_getCurrentTime.setText(str(current_time))
        current_unix_time = timeExchangeMgr.setCurrentTimer(datetype="unix")
        self.lineEdit_getCurrentTime_Unix.setText(str(int(current_unix_time)))

    def on_clicked_select_time_type(self):
        # self.widget_unixTime.setEnabled(False)
        # self.widget_pkTime.setEnabled(False)
        # self.widget_utcTime.setEnabled(False)
        # self.widget_gpsTime.setEnabled(False)
        # self.widget_itows.setEnabled(False)
        # self.widget_doy.setEnabled(False)
        # self.widget_leapSecond.setEnabled(False)

        if self.comboBox_timeType.currentText() == "Unix时间戳":
            self.widget_unixTime.setEnabled(True)
        elif self.comboBox_timeType.currentText() == "北京时间":
            self.widget_pkTime.setEnabled(True)
        elif self.comboBox_timeType.currentText() == "UTC时间":
            self.widget_utcTime.setEnabled(True)
        elif self.comboBox_timeType.currentText() == "GPS时间":
            self.widget_gpsTime.setEnabled(True)
        elif self.comboBox_timeType.currentText() == "周内秒":
            self.widget_itows.setEnabled(True)
        elif self.comboBox_timeType.currentText() == "年积日":
            self.widget_doy.setEnabled(True)

    def time_convert(self):
        unixTime, pk_time, utc_time, gps_time, gps_week, gps_sec, year, day, leapSecond = 0, 0, 0, 0, 0, 0, 0, 0, 0
        try:
            if self.comboBox_timeType.currentText() == "Unix时间戳":
                unixTime = float(self.lineEdit_unixTime.text())
                utc_time = timeExchangeMgr.unix2Utc(unixTime, unit="s")
                pk_time = timeExchangeMgr.unix2date(unixTime, unit="s")
                gps_time = timeExchangeMgr.unix2Gps(unixTime, unit="s")
                gps_week, gps_sec = timeExchangeMgr.utc2Gps(str(utc_time))
                year, day = timeExchangeMgr.timedate2doy(pk_time)
            elif self.comboBox_timeType.currentText() == "北京时间":
                pk_time = self.dateTimeEdit_pkTime.dateTime()
                pk_time_str = pk_time.toString(Qt.ISODate)
                pk_time_str = pk_time_str.replace("T", " ")
                unixTime = timeExchangeMgr.date2Unix(pk_time_str)
                utc_time = timeExchangeMgr.unix2Utc(unixTime, unit="s")
                gps_time = timeExchangeMgr.unix2Gps(unixTime, unit="s")
                gps_week, gps_sec = timeExchangeMgr.utc2Gps(str(utc_time))
                pk_time = timeExchangeMgr.unix2date(unixTime, unit="s")
                year, day = timeExchangeMgr.timedate2doy(pk_time)
            elif self.comboBox_timeType.currentText() == "UTC时间":
                utc_time = self.dateTimeEdit_utcTime.dateTime()
                utc_time_str = utc_time.toString(Qt.ISODate)
                utc_time_str = utc_time_str.replace("T", " ")
                unixTime = timeExchangeMgr.utc2Unix(utc_time_str)
                utc_time = timeExchangeMgr.unix2Utc(unixTime, unit="s")
                pk_time = timeExchangeMgr.unix2date(unixTime, unit="s")
                gps_time = timeExchangeMgr.unix2Gps(unixTime, unit="s")
                gps_week, gps_sec = timeExchangeMgr.utc2Gps(str(utc_time))
                year, day = timeExchangeMgr.timedate2doy(pk_time)
            elif self.comboBox_timeType.currentText() == "GPS时间":
                gps_time = self.dateTimeEdit_gpsTime.dateTime()
                gps_time_str = gps_time.toString(Qt.ISODate)
                gps_time_str = gps_time_str.replace("T", " ")
                unixTime = timeExchangeMgr.gpsTime2Unix(gps_time_str)
                utc_time = timeExchangeMgr.unix2Utc(unixTime, unit="s")
                pk_time = timeExchangeMgr.unix2date(unixTime, unit="s")
                gps_time = timeExchangeMgr.unix2Gps(unixTime, unit="s")
                gps_week, gps_sec = timeExchangeMgr.utc2Gps(str(utc_time))
                year, day = timeExchangeMgr.timedate2doy(pk_time)
            elif self.comboBox_timeType.currentText() == "周内秒":
                gps_week, gps_sec = float(self.lineEdit_gpsWeek.text()), float(self.lineEdit_gpsItow.text())
                unixTime = timeExchangeMgr.gps2Unix(gps_week, gps_sec)
                utc_time = timeExchangeMgr.unix2Utc(unixTime, unit="s")
                pk_time = timeExchangeMgr.unix2date(unixTime, unit="s")
                gps_time = timeExchangeMgr.unix2Gps(unixTime, unit="s")
                year, day = timeExchangeMgr.timedate2doy(pk_time)
            elif self.comboBox_timeType.currentText() == "年积日":
                year, day = int(self.lineEdit_year.text()), int(self.lineEdit_doy.text())
                pk_time = timeExchangeMgr.doy2timedate(year, day)
                unixTime = timeExchangeMgr.date2Unix(pk_time)
                utc_time = timeExchangeMgr.unix2Utc(unixTime, unit="s")
                pk_time = timeExchangeMgr.unix2date(unixTime, unit="s")
                gps_time = timeExchangeMgr.unix2Gps(unixTime, unit="s")
                gps_week, gps_sec = timeExchangeMgr.utc2Gps(str(utc_time))

            leapSecond = timeExchangeMgr.cal_leapSecond("unixTime", unixTime)
        except:
            unixTime, pk_time, utc_time, gps_time, gps_week, gps_sec, year, day, leapSecond = 0, 0, 0, 0, 0, 0, 0, 0, 0

        self.lineEdit_unixTime.setText(str(unixTime))
        self.dateTimeEdit_pkTime.setDateTime(pk_time)
        self.dateTimeEdit_utcTime.setDateTime(utc_time)
        self.dateTimeEdit_gpsTime.setDateTime(gps_time)
        self.lineEdit_gpsWeek.setText(str(int(gps_week)))
        self.lineEdit_gpsItow.setText(str(gps_sec))
        self.lineEdit_year.setText(str(year))
        self.lineEdit_doy.setText(str(day))
        self.label_leapSecond.setText(str(leapSecond))
