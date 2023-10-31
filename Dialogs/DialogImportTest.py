from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog

import UI.ui_Dialog_import_test
import global_var
import os
from Dialogs.DialogSetCsv import DialogSetCsv


# 导入测试文件窗口
class DialogImportTest(QDialog, UI.ui_Dialog_import_test.Ui_Dialog):
    my_signal = QtCore.pyqtSignal(list)

    def __init__(self):
        super(DialogImportTest, self).__init__()
        self.setupUi(self)
        self.csv_dict = {}
        self.signal_dict = {}
        self.csv_file_path = None

        self.ins_widget_add = []
        self.gnss_widget_add = []

        # global TEST_DATE
        self.dateEdit_testDate.setDate(global_var.get_value("TEST_DATE"))

        # 信号关联
        self.comboBox_algorithmType.currentIndexChanged.connect(self.on_clicked_select_algorithm_type)  # 选择测试类型按钮按下
        self.addTestFile.clicked.connect(self.on_clicked_add_test_file)  # 选择测试类型按钮按下

        # INS
        self.pushBtn_SelectIns.clicked.connect(self.on_clicked_select_ins_file)  # 选择参考文件按钮按下

        # GNSS
        self.pushBtn_SelectGnss.clicked.connect(self.on_clicked_select_gnss_file)  # 选择参考文件按钮按下
        self.comboBox_GetGnssType.currentIndexChanged.connect(lambda: self.on_clicked_select_test_type(None))  # 选择测试文件类型按下
        self.pushButton_setCsv.clicked.connect(lambda: self.open_dialog_import_csv())  # 打开csv配置窗口

        self.buttonBox.accepted.connect(self.on_accepted)    # OK按钮按下
        # self.buttonBox.rejected.connect(self.on_rejected)

    # 导入csv配置文件窗口
    def open_dialog_import_csv(self, gnss_widget=None):
        if not gnss_widget:
            dialog = DialogSetCsv(self.csv_file_path)
        else:
            dialog = DialogSetCsv(gnss_widget['csv_file_path'], gnss_widget)
        dialog.csv_signal.connect(self.update_csv_info)
        dialog.exec_()  # 显示对话框并等待用户响应

    def update_csv_info(self, signal_dict, not_in_widget):
        if not_in_widget:
            self.csv_dict = signal_dict

    def on_accepted(self):
        algorithm_type = self.comboBox_algorithmType.currentText()
        if algorithm_type == "INS":
            print(len(self.ins_widget_add))
            self.signal_dict = [
                    {"algorithm_type": algorithm_type,
                     "date_time": None,
                     "file_path": self.lineEdit_GetInsFile.text(),
                     "file_type": self.comboBox_GetInsType.currentText(),
                     "csv_dict": self.csv_dict,
                     'bpox': [0, 0, 0]}
                ]
            for ins_widget in self.ins_widget_add:
                self.signal_dict.append({
                    "algorithm_type": algorithm_type,
                    "date_time": None,
                    "file_path": ins_widget['lineEdit_GetInsFile'].text(),
                    "file_type": ins_widget['comboBox_GetInsType'].currentText(),
                    "csv_dict": self.csv_dict,
                    'bpox': [0, 0, 0]
                })
        else:
            TEST_DATE = self.dateEdit_testDate.date()
            global_var.set_value("TEST_DATE", TEST_DATE)
            print(len(self.gnss_widget_add))
            self.signal_dict = [{"algorithm_type": algorithm_type,
                                "date_time": TEST_DATE.toString(Qt.ISODate),  # 获取测试日期,
                                "file_path": self.lineEdit_GetGnssFile.text(),
                                "file_type": self.comboBox_GetGnssType.currentText(),
                                "csv_dict": self.csv_dict,
                                'bpox': [0, 0, 0]}]
            for gnss_widget in self.gnss_widget_add:
                self.signal_dict.append({
                    "algorithm_type": algorithm_type,
                    "date_time": TEST_DATE.toString(Qt.ISODate),  # 获取测试日期,
                    "file_path": gnss_widget['lineEdit_GetGnssFile'].text(),
                    "file_type": gnss_widget['comboBox_GetGnssType'].currentText(),
                    "csv_dict": gnss_widget['csv_dict'],
                    'bpox': [0, 0, 0]
                })

        self.my_signal.emit(self.signal_dict)

    def on_clicked_select_algorithm_type(self):
        if self.comboBox_algorithmType.currentText() == "INS":
            self.groupBox_ins.setEnabled(True)
            self.widget_3.setEnabled(True)
            self.groupBox_gnss.setEnabled(False)
        elif self.comboBox_algorithmType.currentText() == "GNSS":
            self.groupBox_ins.setEnabled(False)
            self.widget_4.setEnabled(True)
            self.widget_5.setEnabled(True)
            self.groupBox_gnss.setEnabled(True)

    def on_clicked_add_test_file(self):
        if self.comboBox_algorithmType.currentText() == "INS":
            ins_widget = {'lineEdit_GetInsFile':None, 'comboBox_GetInsType':'', 'widget':None}
            self.ins_widget_add.append(ins_widget)

            ins_widget['widget'] = QtWidgets.QWidget(self.groupBox_ins)
            ins_widget['widget'].setObjectName("widget_3")
            horizontalLayout_2 = QtWidgets.QHBoxLayout(ins_widget['widget'])
            horizontalLayout_2.setObjectName("horizontalLayout_2")
            pushBtn_SelectIns = QtWidgets.QPushButton(ins_widget['widget'])
            pushBtn_SelectIns.setObjectName("pushBtn_SelectIns")
            pushBtn_SelectIns.setText("选择文件")

            ins_widget['lineEdit_GetInsFile'] = QtWidgets.QLineEdit(ins_widget['widget'])
            ins_widget['lineEdit_GetInsFile'].setObjectName("lineEdit_GetInsFile")
            pushBtn_SelectIns.clicked.connect(lambda: self.on_clicked_select_ins_file(ins_widget['lineEdit_GetInsFile']))
            horizontalLayout_2.addWidget(pushBtn_SelectIns)
            horizontalLayout_2.addWidget(ins_widget['lineEdit_GetInsFile'])

            ins_widget['comboBox_GetInsType'] = QtWidgets.QComboBox(ins_widget['widget'])
            ins_widget['comboBox_GetInsType'].setObjectName("comboBox_GetInsType")
            ins_widget['comboBox_GetInsType'].addItem("导远自定义格式")
            ins_widget['comboBox_GetInsType'].addItems({"北云明文格式"})
            horizontalLayout_2.addWidget(ins_widget['comboBox_GetInsType'])

            toolButton_delet = QtWidgets.QPushButton(ins_widget['widget'])
            toolButton_delet.setObjectName("toolButton_delet")
            toolButton_delet.setText("删除")
            toolButton_delet.clicked.connect(lambda: self.delete_ins_widget(ins_widget))
            horizontalLayout_2.addWidget(toolButton_delet)
            #
            self.verticalLayout_2.addWidget(ins_widget['widget'])
        elif self.comboBox_algorithmType.currentText() == "GNSS":
            gnss_widget = {'lineEdit_GetGnssFile':None, 'comboBox_GetGnssType':'','pushButton_setCsv':None , 'csv_file_path':'', 'csv_dict':{}, 'widget':None}
            self.gnss_widget_add.append(gnss_widget)

            gnss_widget['widget'] = QtWidgets.QWidget(self.groupBox_gnss)
            gnss_widget['widget'].setObjectName("widget_5")
            horizontalLayout_4 = QtWidgets.QHBoxLayout(gnss_widget['widget'])
            horizontalLayout_4.setObjectName("horizontalLayout_4")
            pushBtn_SelectGnss = QtWidgets.QPushButton(gnss_widget['widget'])
            pushBtn_SelectGnss.setObjectName("pushBtn_SelectGnss")
            pushBtn_SelectGnss.setText("选择文件")

            gnss_widget['lineEdit_GetGnssFile'] = QtWidgets.QLineEdit(gnss_widget['widget'])
            gnss_widget['lineEdit_GetGnssFile'].setObjectName("lineEdit_GetGnssFile")
            pushBtn_SelectGnss.clicked.connect(lambda: self.on_clicked_select_gnss_file(widget=gnss_widget))
            horizontalLayout_4.addWidget(pushBtn_SelectGnss)
            horizontalLayout_4.addWidget(gnss_widget['lineEdit_GetGnssFile'])

            gnss_widget['comboBox_GetGnssType'] = QtWidgets.QComboBox(gnss_widget['widget'])
            gnss_widget['comboBox_GetGnssType'].setObjectName("comboBox_GetGnssType")
            gnss_widget['comboBox_GetGnssType'].addItem("NMEA")
            gnss_widget['comboBox_GetGnssType'].addItems({"CSV自定义", "导远自定义格式-GPS"})
            gnss_widget['comboBox_GetGnssType'].currentIndexChanged.connect(lambda: self.on_clicked_select_test_type(gnss_widget))
            horizontalLayout_4.addWidget(gnss_widget['comboBox_GetGnssType'])

            gnss_widget['pushButton_setCsv'] = QtWidgets.QPushButton(gnss_widget['widget'])
            gnss_widget['pushButton_setCsv'].setObjectName("pushButton_setCsv")
            gnss_widget['pushButton_setCsv'].setText("配置CSV")
            gnss_widget['pushButton_setCsv'].setEnabled(False)
            gnss_widget['pushButton_setCsv'].clicked.connect(lambda: self.open_dialog_import_csv(gnss_widget=gnss_widget))
            horizontalLayout_4.addWidget(gnss_widget['pushButton_setCsv'])

            toolButton_delet = QtWidgets.QPushButton(gnss_widget['widget'])
            toolButton_delet.setObjectName("toolButton_delet")
            toolButton_delet.setText("删除")
            toolButton_delet.clicked.connect(lambda: self.delete_gnss_widget(gnss_widget))
            horizontalLayout_4.addWidget(toolButton_delet)
            #
            self.verticalLayout_4.addWidget(gnss_widget['widget'])

    def delete_ins_widget(self, ins_widget):
        self.verticalLayout_2.removeWidget(ins_widget['widget'])
        ins_widget['widget'].deleteLater()
        self.ins_widget_add.pop(self.ins_widget_add.index(ins_widget))

    def delete_gnss_widget(self, gnss_widget):
        self.verticalLayout_4.removeWidget(gnss_widget['widget'])
        gnss_widget['widget'].deleteLater()
        self.gnss_widget_add.pop(self.gnss_widget_add.index(gnss_widget))

    # 选择INS文件按钮按下
    def on_clicked_select_ins_file(self, line_edit=None):
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        if not line_edit:
            self.lineEdit_GetInsFile.setText(fileName)
        else:
            line_edit.setText(fileName)

    # 选择GNSS文件按钮按下
    def on_clicked_select_gnss_file(self, widget=None):
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件

        if not widget:
            self.lineEdit_GetGnssFile.setText(fileName)
            self.csv_file_path = fileName
        else:
            widget['lineEdit_GetGnssFile'].setText(fileName)
            widget['csv_file_path'] = fileName

    # 选择测试文件类型按钮按下
    def on_clicked_select_test_type(self, widget):
        if not widget:
            if self.comboBox_GetGnssType.currentText() == "CSV自定义":
                self.pushButton_setCsv.setEnabled(True)
            else:
                self.pushButton_setCsv.setEnabled(False)
        else:
            if widget['comboBox_GetGnssType'].currentText() == "CSV自定义":
                widget['pushButton_setCsv'].setEnabled(True)
            else:
                widget['pushButton_setCsv'].setEnabled(False)

