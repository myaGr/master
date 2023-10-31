from PyQt5.QtWidgets import QDialog
import UI.ui_Dialog_cal_distiance
from Utils.dataStatistics import DataStatistics
from Utils import posDMSExchangeMgr
import numpy as np
import math


# 距离计算工具窗口
class DialogCalDistance(QDialog, UI.ui_Dialog_cal_distiance.Ui_Dialog):
    def __init__(self):
        super(DialogCalDistance, self).__init__()
        self.setupUi(self)

        self.pushButton_dd2dm.clicked.connect(self.dd2dm)
        self.pushButton_dm2dd.clicked.connect(self.dm2dd)
        self.pushButton_dd2dms.clicked.connect(self.dd2dms)
        self.pushButton_dms2dd.clicked.connect(self.dms2dd)
        self.pushButton_2distance.clicked.connect(self.cal_distance)  # 已知2点计算距离

    def dd2dm(self):
        dd = self.lineEdit_dd_1.text()
        dm = posDMSExchangeMgr.d2dm(float(dd))
        self.lineEdit_dm_1.setText(str(dm))

    def dm2dd(self):
        dm = self.lineEdit_dm_2.text()
        dd = posDMSExchangeMgr.dm2d(float(dm))
        self.lineEdit_dd_2.setText(str(dd))

    def dd2dms(self):
        dd = self.lineEdit_dd_3.text()
        d, m, s = posDMSExchangeMgr.dd2ddmmss(float(dd))
        self.lineEdit_d_3.setText(str(d))
        self.lineEdit_m_3.setText(str(m))
        self.lineEdit_s_3.setText(str(s))

    def dms2dd(self):
        d = self.lineEdit_d_4.text()
        m = self.lineEdit_m_4.text()
        s = self.lineEdit_s_4.text()
        dd = posDMSExchangeMgr.ddmmss2dd(float(d), float(m), float(s))
        self.lineEdit_dd_4.setText(str(dd))

    def cal_distance(self):
        # 两个坐标点之间距离计算
        lat, lon, height = np.array([float(self.lineEdit_lat2.text())]), np.array([float(self.lineEdit_lon2.text())]), np.array([0])
        # 以第一个为坐标为原点
        pos0 = [float(self.lineEdit_lat1.text()), float(self.lineEdit_lon1.text()), 0]
        coordinate = DataStatistics().pos_covert(lat, lon, height, pos0)   # 地理坐标系转平面坐标坐标系
        north, east, ground = coordinate[0][0], coordinate[1][0], coordinate[2][0]
        distance = math.sqrt(north ** 2 + east ** 2)
        self.lineEdit_distance.setText(str(distance))


if __name__ == "__main__":
    pos0 = [float(31.218219383), float(121.312532067), 0]
    lat, lon, height = np.array([31.218219383]), np.array([121.312532067]), np.array([0])
    coordinate = DataStatistics().pos_covert(lat, lon, height, pos0)  # 坐标转换
    north, east, ground = coordinate[0][0], coordinate[1][0], coordinate[2][0]
    print(math.sqrt(north ** 2 + east ** 2))


