from PyQt5.QtWidgets import QDialog
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QStandardItemModel, QStandardItem
import UI.ui_Dialog_set_csv


# 配置自定义CSV格式
class DialogSetCsv(QDialog, UI.ui_Dialog_set_csv.Ui_Dialog):
    csv_signal = QtCore.pyqtSignal(dict, bool)

    def __init__(self, filepath, widget=None):
        super(DialogSetCsv, self).__init__()
        self.setupUi(self)
        self.filepath = filepath
        self.split_way = ","
        self.protocol_Type = {}
        self.pushButton_previewCsv.clicked.connect(lambda: self.preview_file())  # 预览按钮按下
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下
        self.widget = widget if widget else None

    def check_split_way(self):
        if self.radioButton_splitByComma.isChecked():
            self.split_way = ","
        elif self.radioButton_splitBySpace.isChecked():
            self.split_way = " "

    def preview_file(self):
        self.check_split_way()
        model = QStandardItemModel(100, 30)
        if self.filepath in ['', None]:
            print("no file")
            return
        with open(self.filepath, 'r', encoding='GB2312', errors='ignore') as f:
            for i in range(100):
                line = f.readline().strip()
                # line_list = line.split()
                line_list = line.replace('\n', '').split(self.split_way)
                while '' in line_list:
                    line_list.remove('')
                for j in range(len(line_list)):
                    model.setItem(i, j, QStandardItem(line_list[j]))
        self.tableView_csvPreview.setModel(model)

    def on_accepted(self):
        self.protocol_Type['time'] = [int(self.spinBox_time.text()) - 1, self.comboBox_time.currentText()]
        self.protocol_Type['latitude'] = [int(self.spinBox_lat.text()) - 1, self.comboBox_latlon.currentText()]
        self.protocol_Type['longitude'] = [int(self.spinBox_lon.text()) - 1, self.comboBox_latlon.currentText()]
        self.protocol_Type['height'] = [int(self.spinBox_hight.text()) - 1, self.comboBox_hight.currentText()]
        self.protocol_Type['velocity'] = []
        self.protocol_Type['heading'] = []

        signal_dict = {"protocol_type": self.protocol_Type, "split_way": self.split_way}

        if self.widget:
            self.widget['csv_dict'] = signal_dict
            self.csv_signal.emit(signal_dict, False)
        else:
            self.csv_signal.emit(signal_dict, True)

