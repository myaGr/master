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
        self.dateEdit_utcDate_1.setDate(global_var.get_value("TEST_DATE"))
        self.dateTimeEdit_utcTime_3.setDate(global_var.get_value("TEST_DATE"))
        self.pushButton_utcDate2gpsWeek.clicked.connect(self.utcDate2gpsWeek)  # UTC日期转GPS周
        self.pushButton_gpsWeek2utcDate.clicked.connect(self.gpsWeek2utcDate)  # GPS周转UTC日期
        self.pushButton_utc2itow.clicked.connect(self.utc2Itow)  # utc转gps周内秒
        self.pushButton_gps2utc.clicked.connect(self.gps2utc)  # gps时间转utc

    def utcDate2gpsWeek(self):
        utc_date = self.dateEdit_utcDate_1.date()
        utc_date = utc_date.toString(Qt.ISODate)
        gps_week = timeExchangeMgr.date2Gpsweek(utc_date)
        self.lineEdit_gpsWeek_1.setText(str(int(gps_week)))

    def gpsWeek2utcDate(self):
        gpsWeek = int(self.lineEdit_gpsWeek_2.text())
        utc_date = timeExchangeMgr.gps2UTC(gpsWeek, 0)
        self.lineEdit_utcDate_2.setText(str(utc_date.date()))

    def utc2Itow(self):
        utc_time = self.dateTimeEdit_utcTime_3.dateTime()
        utc_time = utc_time.toString(Qt.ISODate)
        utc_time = utc_time.replace("T", " ")
        gps_week, gps_sec = timeExchangeMgr.utc2Gps(utc_time)
        self.lineEdit_itow_3.setText(str(gps_sec))

    def gps2utc(self):
        gps_sec = float(self.lineEdit_itow_4.text())
        gps_week = int(self.lineEdit_gpsWeek_4.text())
        utc_time = timeExchangeMgr.gps2UTC(gps_week, gps_sec)
        self.lineEdit_utcTime_4.setText(str(utc_time))


