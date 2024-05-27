from PyQt5.QtWidgets import QDialog
from PyQt5 import QtCore
import UI.ui_Dialog_gnss_multi_plot


# 绘图设置
class DialogGnssMultiPlot(QDialog, UI.ui_Dialog_gnss_multi_plot.Ui_Dialog):
    my_signal = QtCore.pyqtSignal(list)

    def __init__(self):
        super(DialogGnssMultiPlot, self).__init__()
        self.setupUi(self)
        self.signal_list = []
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下

    def on_accepted(self):
        if self.checkBox_timeGap.isChecked():
            self.signal_list.append("历元间隔分布图")
        if self.checkBox_ageDiff.isChecked():
            self.signal_list.append("差分龄期统计图")
        if self.checkBox_gpsQualityPosError.isChecked():
            self.signal_list.append("解状态水平误差序列图")
        if self.checkBox_posLateralError.isChecked():
            self.signal_list.append("位置横向误差历元分布图")
        if self.checkBox_posLongitudinalError.isChecked():
            self.signal_list.append("位置纵向误差历元分布图")
        if self.checkBox_posHorizontalError.isChecked():
            self.signal_list.append("位置水平误差历元分布图")
        if self.checkBox_elevationError.isChecked():
            self.signal_list.append("高程误差历元分布图")
        if self.checkBox_velError.isChecked():
            self.signal_list.append("速度误差(前向)历元分布图")
        if self.checkBox_headingError.isChecked():
            self.signal_list.append("姿态误差(航向)历元分布图")
        if self.checkBox_doubleheadingError.isChecked():
            self.signal_list.append("双天线航向误差历元分布图")

        self.my_signal.emit(self.signal_list)
