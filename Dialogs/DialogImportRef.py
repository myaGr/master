import os
import time

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog

import UI.ui_Dialog_import_ref
import global_var
from Dialogs.DialogSetCsv import DialogSetCsv


# 导入参考文件窗口
class DialogImportRef(QDialog, UI.ui_Dialog_import_ref.Ui_Dialog):
    my_signal = QtCore.pyqtSignal(dict)

    def __init__(self):
        super(DialogImportRef, self).__init__()
        self.setupUi(self)
        self.dateEdit_testDate.setDate(global_var.get_value("TEST_DATE"))
        self.csv_file_path = None
        self.signal_dict = {}
        self.csv_dict = {}
        self.split_way = None

        self.comboBox_testType.currentIndexChanged.connect(self.on_clicked_select_test_type)  # 选择参考类型按钮按下
        self.comboBox_staticRefType.currentIndexChanged.connect(self.on_clicked_select_test_type)  # 选择静态测试参考类型按钮按下
        self.comboBox_getRefType.currentIndexChanged.connect(self.on_clicked_select_ref_type)  # 选择参考文件类型按下
        self.pushBtn_selectRef.clicked.connect(self.on_clicked_select_ref_file)  # 选择参考文件按钮按下
        # self.pushButton_setCsv.clicked.connect(lambda: self.open_dialog(DialogSetCsv(self.csv_file_path)))  # 打开csv配置窗口
        self.pushButton_setCsv.clicked.connect(lambda: self.open_dialog_import_csv())  # 打开csv配置窗口

        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下
        # self.buttonBox.rejected.connect(self.rejected_info)

    # 选择测试类型按钮按下
    def on_clicked_select_test_type(self):
        if self.comboBox_testType.currentText() == "静态":
            self.comboBox_staticRefType.setEnabled(True)
            self.groupBox_refCoordinate.setEnabled(True)
            if self.comboBox_staticRefType.currentText() == "参考点坐标":
                self.groupBox_refFile.setEnabled(False)
                self.widget_refCalType.setEnabled(True)
            elif self.comboBox_staticRefType.currentText() == "参考文件":
                self.groupBox_refCoordinate.setEnabled(False)
                self.groupBox_refFile.setEnabled(True)
                self.widget_refCalType.setEnabled(True)
        elif self.comboBox_testType.currentText() == "动态":
            self.comboBox_staticRefType.setEnabled(False)
            self.groupBox_refCoordinate.setEnabled(False)
            self.widget_refCalType.setEnabled(False)
            self.groupBox_refFile.setEnabled(True)
        elif self.comboBox_testType.currentText() == "无参考":
            self.comboBox_staticRefType.setEnabled(False)
            self.groupBox_refCoordinate.setEnabled(False)
            self.groupBox_refFile.setEnabled(False)

    # 选择参考文件类型按钮按下
    def on_clicked_select_ref_type(self):
        if self.comboBox_getRefType.currentText() == "CSV自定义":
            self.pushButton_setCsv.setEnabled(True)
        else:
            self.pushButton_setCsv.setEnabled(False)

    # 选择参考文件按钮按下
    def on_clicked_select_ref_file(self):
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        self.lineEdit_getRefFile.setText(fileName)
        self.csv_file_path = fileName

    # 导入csv配置文件窗口
    def open_dialog_import_csv(self):
        dialog = DialogSetCsv(self.csv_file_path)
        dialog.csv_signal.connect(self.update_csv_info)
        dialog.exec_()  # 显示对话框并等待用户响应

    def update_csv_info(self, signal_dict):
        self.csv_dict = signal_dict

    def on_accepted(self):
        test_type = self.comboBox_testType.currentText()
        static_ref_type = self.comboBox_staticRefType.currentText()
        TEST_DATE = self.dateEdit_testDate.date()
        global_var.set_value("TEST_DATE", TEST_DATE)
        freq = int(self.comboBox_getFre.currentText()[:-2]) if "HZ" in self.comboBox_getFre.currentText() else None

        if test_type == "静态" and static_ref_type == "参考点坐标":
            self.signal_dict = {"test_type": test_type,
                                "static_ref_type": static_ref_type,
                                "date_time": TEST_DATE.toString(Qt.ISODate),  # 获取测试日期
                                "latitude": self.lineEdit_latitude.text(),
                                "longitude": self.lineEdit_longitude.text(),
                                "ellHeight": self.lineEdit_height.text(),
                                'bpox': [0, 0, 0]
                                }
        elif test_type == "静态" and static_ref_type == "参考文件":
            self.signal_dict = {"test_type": test_type,
                                "static_ref_type": static_ref_type,
                                "date_time": TEST_DATE.toString(Qt.ISODate),  # 获取测试日期
                                "ref_cal_Type": self.comboBox_refCalType.currentText(),
                                "file_path": self.lineEdit_getRefFile.text(),
                                "file_type": self.comboBox_getRefType.currentText(),
                                "file_freq": freq,
                                "csv_dict": self.csv_dict,
                                'bpox': [0, 0, 0]
                                }
        elif test_type == "动态":
            self.signal_dict = {"test_type": test_type,
                                "static_ref_type": None,
                                "date_time": TEST_DATE.toString(Qt.ISODate),  # 获取测试日期,
                                "ref_cal_Type": self.comboBox_refCalType.currentText(),
                                "file_path": self.lineEdit_getRefFile.text(),
                                "file_type": self.comboBox_getRefType.currentText(),
                                "file_freq": freq,
                                "csv_dict": self.csv_dict,
                                'bpox': [0, 0, 0]
                                }
        else:
            self.signal_dict = {"test_type": test_type,
                                "date_time": TEST_DATE.toString(Qt.ISODate)  # 获取测试日期
                                }
        self.my_signal.emit(self.signal_dict)
