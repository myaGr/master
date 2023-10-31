from PyQt5.QtWidgets import QDialog
from PyQt5 import QtCore
import UI.ui_Dialog_set_plot_view


# 绘图设置
class DialogSetPlot(QDialog, UI.ui_Dialog_set_plot_view.Ui_Dialog):
    my_signal = QtCore.pyqtSignal(dict)

    def __init__(self):
        super(DialogSetPlot, self).__init__()
        self.setupUi(self)
        self.signal_dict = {}
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下

    def on_accepted(self):
        # 绘图时间范围
        if self.comboBox_cutTimeType.currentText() == "gps周内秒":
            cut_time_type = "gps"
        else:
            cut_time_type = "utc"

        # INS 画图时间类型
        if self.radioButton_insItow.isChecked():
            ins_plot_time_type = "gps周内秒"
        else:
            ins_plot_time_type = "计时时间（秒）"

        try:
            ins_freq_sample_num = int(self.lineEdit_freq_sample_num.text())
        except Exception as e:
            print('ins 抽稀倍数输入有误： ', e)
            ins_freq_sample_num = 10

        # gnss 画图时间类型
        if self.radioButton_gpsUtc.isChecked():
            gnss_plot_time_type = "utc时间"
        elif self.radioButton_gpsItow.isChecked():
            gnss_plot_time_type = "gps周内秒"
        else:
            gnss_plot_time_type = "计时时间（秒）"

        # gnss 画图解状态
        gnss_plot_flags = {}
        if self.checkBox_fixed.isChecked():
            gnss_plot_flags[4] = '固定解'
        if self.checkBox_float.isChecked():
            gnss_plot_flags[5] = '浮点解'
        if self.checkBox_pseduo.isChecked():
            gnss_plot_flags[2] = '差分解'
        if self.checkBox_single.isChecked():
            gnss_plot_flags[1] = '单点解'
        if self.checkBox_other.isChecked():
            gnss_plot_flags[0] = '其他'

        self.signal_dict = {"time_type": cut_time_type,
                            "time": [self.lineEdit_startTime.text(), self.lineEdit_endTime.text()],
                            "ins_plot_time_type": ins_plot_time_type,
                            'ins_plot_flags': {4: '固定解', 5: '浮点解', 2: '差分解', 1: '单点解', 0: '其他'},
                            'ins_plot_freq': ins_freq_sample_num,
                            "gnss_plot_time_type": gnss_plot_time_type,
                            "gnss_plot_flags": gnss_plot_flags
                            }
        print(self.signal_dict)
        self.my_signal.emit(self.signal_dict)
