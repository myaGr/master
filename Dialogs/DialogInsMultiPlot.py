from PyQt5.QtWidgets import QDialog
from PyQt5 import QtCore
import UI.ui_Dialog_ins_multi_plot


# 绘图设置
class DialogInsMultiPlot(QDialog, UI.ui_Dialog_ins_multi_plot.Ui_Dialog):
    my_signal = QtCore.pyqtSignal(list)

    def __init__(self):
        super(DialogInsMultiPlot, self).__init__()
        self.setupUi(self)
        self.signal_list = []
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下

    def on_accepted(self):
        if self.checkBox_timeGap.isChecked():
            self.signal_list.append("历元间隔分布图")
        if self.checkBox_vel_bias.isChecked():
            self.signal_list.append("速度偏差统计")
        if self.checkBox_yaw_bias.isChecked():
            self.signal_list.append("航向偏差统计")
        if self.checkBox_pos_horizontal_bias.isChecked():
            self.signal_list.append("位置水平误差")
        if self.checkBox_pos_lateral_bias.isChecked():
            self.signal_list.append("位置横向误差")
        if self.checkBox_pos_longitudinal_bias.isChecked():
            self.signal_list.append("位置纵向误差")

        if self.checkBox_vel_error_f.isChecked():
            self.signal_list.append("前向速度对比")
        if self.checkBox_vel_error_r.isChecked():
            self.signal_list.append("右向速度对比")
        if self.checkBox_vel_error_d.isChecked():
            self.signal_list.append("下向速度对比")
        if self.checkBox_vel_error_n.isChecked():
            self.signal_list.append("北向速度对比")
        if self.checkBox_vel_error_e.isChecked():
            self.signal_list.append("东向速度对比")
        if self.checkBox_vel_error_g.isChecked():
            self.signal_list.append("地向速度对比")
        if self.checkBox_vel_n.isChecked():
            self.signal_list.append("北向速度")
        if self.checkBox_vel_e.isChecked():
            self.signal_list.append("东向速度")
        if self.checkBox_vel_g.isChecked():
            self.signal_list.append("地向速度")
        if self.checkBox_vel_f.isChecked():
            self.signal_list.append("前向速度")
        if self.checkBox_vel_r.isChecked():
            self.signal_list.append("右向速度")
        if self.checkBox_vel_d.isChecked():
            self.signal_list.append("下向速度")
        if self.checkBox_mile_f.isChecked():
            self.signal_list.append("前向里程")
        if self.checkBox_mile_r.isChecked():
            self.signal_list.append("右向里程")
        if self.checkBox_mile_d.isChecked():
            self.signal_list.append("下向里程")

        if self.checkBox_lon.isChecked():
            self.signal_list.append("经度")
        if self.checkBox_lat.isChecked():
            self.signal_list.append("纬度")
        if self.checkBox_h.isChecked():
            self.signal_list.append("高度")
        if self.checkBox_lon_error.isChecked():
            self.signal_list.append("经度对比")
        if self.checkBox_lat_error.isChecked():
            self.signal_list.append("纬度对比")
        if self.checkBox_h_error.isChecked():
            self.signal_list.append("高度对比")
        if self.checkBox_roll.isChecked():
            self.signal_list.append("Roll")
        if self.checkBox_pitch.isChecked():
            self.signal_list.append("Pitch")
        if self.checkBox_yaw.isChecked():
            self.signal_list.append("Yaw")
        if self.checkBox_roll_error.isChecked():
            self.signal_list.append("Roll对比")
        if self.checkBox_yaw_error.isChecked():
            self.signal_list.append("Pitch对比")
        if self.checkBox_pitch_error.isChecked():
            self.signal_list.append("Yaw对比")

        if self.checkBox_Pp.isChecked():
            self.signal_list.append("Pp")
        if self.checkBox_Pv.isChecked():
            self.signal_list.append("Pv")
        if self.checkBox_Patt.isChecked():
            self.signal_list.append("Patt")
        if self.checkBox_Pba.isChecked():
            self.signal_list.append("Pba")
        if self.checkBox_Pbg.isChecked():
            self.signal_list.append("Pbg")
        if self.checkBox_Palign.isChecked():
            self.signal_list.append("Palign")
        if self.checkBox_Plbbg.isChecked():
            self.signal_list.append("Plbbg")
        if self.checkBox_Plbbc.isChecked():
            self.signal_list.append("Plbbc")
        if self.checkBox_Pkws.isChecked():
            self.signal_list.append("Pkws")
        if self.checkBox_PAtttg.isChecked():
            self.signal_list.append("PAtttg")

        if self.checkBox_Xp.isChecked():
            self.signal_list.append("Xp")
        if self.checkBox_Xv.isChecked():
            self.signal_list.append("Xv")
        if self.checkBox_Xatt.isChecked():
            self.signal_list.append("Xatt")
        if self.checkBox_Xba.isChecked():
            self.signal_list.append("Xba")
        if self.checkBox_Xbg.isChecked():
            self.signal_list.append("Xbg")
        if self.checkBox_Xalign.isChecked():
            self.signal_list.append("Xalign")
        if self.checkBox_Xlbbg.isChecked():
            self.signal_list.append("Xlbbg")
        if self.checkBox_Xlbbc.isChecked():
            self.signal_list.append("Xlbbc")
        if self.checkBox_Xkws.isChecked():
            self.signal_list.append("Xkws")
        if self.checkBox_XAtttg.isChecked():
            self.signal_list.append("XAtttg")

        if self.checkBox_Ba.isChecked():
            self.signal_list.append("Ba")
        if self.checkBox_Bg.isChecked():
            self.signal_list.append("Bg")
        if self.checkBox_Atttb.isChecked():
            self.signal_list.append("Atttb")
        if self.checkBox_Lttg.isChecked():
            self.signal_list.append("Lttg")
        if self.checkBox_Lttc.isChecked():
            self.signal_list.append("Lttc")
        if self.checkBox_Kws.isChecked():
            self.signal_list.append("Kws")
        if self.checkBox_Atttg.isChecked():
            self.signal_list.append("Atttg")
        if self.checkBox_Ltgc.isChecked():
            self.signal_list.append("Ltgc")

        self.my_signal.emit(self.signal_list)
