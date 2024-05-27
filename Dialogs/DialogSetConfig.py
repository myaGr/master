from PyQt5.QtWidgets import QDialog
from PyQt5 import QtCore
import UI.ui_Dialog_set_config


# 绘图设置
class DialogSetConfig(QDialog, UI.ui_Dialog_set_config.Ui_Dialog):
    config_signal = QtCore.pyqtSignal(dict)

    def __init__(self):
        super(DialogSetConfig, self).__init__()
        self.setupUi(self)
        self.bpos = [0, 0, 0]

        self.radioButton_value.clicked.connect(self.on_clicked_set_value)
        self.radioButton_time.clicked.connect(self.on_clicked_set_time)

        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下

    def on_clicked_set_value(self):
        self.widget_value.setEnabled(True)
        self.widget_time.setEnabled(False)

    def on_clicked_set_time(self):
        self.widget_value.setEnabled(False)
        self.widget_time.setEnabled(True)

    def on_accepted(self):
        emit_info = {}
        if self.radioButton_value.isChecked():
            self.bpos = [float(self.lineEdit_BposX.text()),
                         float(self.lineEdit_BposY.text()),
                         float(self.lineEdit_BposZ.text())]
            emit_info = {"bpos": self.bpos}

        elif self.radioButton_time.isChecked():
            time_list = [self.lineEdit_BposStart.text(),
                         self.lineEdit_BposEnd.text(),
                         self.comboBox_TimeType.currentText()]
            emit_info = {"time": time_list}

        emit_info["posture_bpox"] = [float(self.lineEdit_posture_Bpos_roll.text()),
                                                     float(self.lineEdit_posture_Bpos_pitch.text()),
                                                     float(self.lineEdit_posture_Bpos_heading.text())]

        self.config_signal.emit(emit_info)

