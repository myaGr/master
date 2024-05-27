from PyQt5.QtWidgets import QDialog
from PyQt5 import QtWidgets, QtCore
import UI.ui_Dialog_bddb2mat
import os


# 绘图设置
class DialogBddb2Mat(QDialog, UI.ui_Dialog_bddb2mat.Ui_Dialog):
    my_signal = QtCore.pyqtSignal(dict)

    def __init__(self):
        super(DialogBddb2Mat, self).__init__()
        self.setupUi(self)
        self.signal_dict = {
            'data_info': [],
            'data_analysis_flag': {'ins': True, 'gps': True, 'vehicle': True, 'imu': True,
                                   'ins2': False, 'imu2': False, 'sync': False,
                                   'sat': False, 'sat2': False, 'ZeroBias': False, 'EKFhandle_type': False,
                                   'InsPl': False, 'GnssPl': False, 'x_status_ef': False, 'history_pos_4e': False
                                   }
        }
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下
        self.pushButton_selectBDDB.clicked.connect(self.selectFiles)  # OK按钮按下

    def selectFiles(self):
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileNames(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        self.lineEdit_bddbPath.setText(';'.join(fileName))

        # filePath, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
        #                                       "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        # self.lineEdit_bddbPath.setText(filePath)

    def on_accepted(self):

        self.signal_dict['data_info'] = list(self.lineEdit_bddbPath.text().split(';'))

        if self.checkBox_gps.isChecked():
            self.signal_dict['data_analysis_flag']['gps'] = True
        if self.checkBox_vehicle.isChecked():
            self.signal_dict['data_analysis_flag']['vehicle'] = True
        if self.checkBox_imu.isChecked():
            self.signal_dict['data_analysis_flag']['imu'] = True
        if self.checkBox_ins2.isChecked():
            self.signal_dict['data_analysis_flag']['ins2'] = True
        if self.checkBox_imu2.isChecked():
            self.signal_dict['data_analysis_flag']['imu2'] = True
        if self.checkBox_sync.isChecked():
            self.signal_dict['data_analysis_flag']['sync'] = True
        if self.checkBox_zeroBias.isChecked():
            self.signal_dict['data_analysis_flag']['ZeroBias'] = True
        if self.checkBox_EKFhandle.isChecked():
            self.signal_dict['data_analysis_flag']['EKFhandle_type'] = True
        if self.checkBox_sat.isChecked():
            self.signal_dict['data_analysis_flag']['sat'] = True
        if self.checkBox_sat2.isChecked():
            self.signal_dict['data_analysis_flag']['sat2'] = True
        if self.checkBox_insPL.isChecked():
            self.signal_dict['data_analysis_flag']['InsPl'] = True
        if self.checkBox_gnssPL.isChecked():
            self.signal_dict['data_analysis_flag']['GnssPl'] = True
        if self.checkBox_his_pos.isChecked():
            self.signal_dict['data_analysis_flag']['history_pos_4e'] = True
        if self.checkBox_xStatus.isChecked():
            self.signal_dict['data_analysis_flag']['x_status_ef'] = True

        print(self.signal_dict)
        self.my_signal.emit(self.signal_dict)
