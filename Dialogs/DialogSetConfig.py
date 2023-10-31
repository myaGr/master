from PyQt5.QtWidgets import QDialog
from PyQt5 import QtCore
import UI.ui_Dialog_set_config


# 绘图设置
class DialogSetConfig(QDialog, UI.ui_Dialog_set_config.Ui_Dialog):
    config_signal = QtCore.pyqtSignal(list)

    def __init__(self):
        super(DialogSetConfig, self).__init__()
        self.setupUi(self)
        self.bpos = [0, 0, 0]
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下

    def on_accepted(self):
        self.bpos = [float(self.lineEdit_BposX.text()),
                     float(self.lineEdit_BposY.text()),
                     float(self.lineEdit_BposZ.text())]

        self.config_signal.emit(self.bpos)

