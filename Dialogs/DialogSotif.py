from PyQt5.QtWidgets import QDialog
from PyQt5 import QtWidgets, QtCore
import UI.ui_Dialog_sotif
from Utils import visual_integrity
from Utils.dataMatPlot import PlotData
import matplotlib.pyplot as plt
import pandas as pd
import threading
import os


# 绘图设置
class DialogSotif(QDialog, UI.ui_Dialog_sotif.Ui_Dialog):

    def __init__(self):
        super(DialogSotif, self).__init__()
        self.setupUi(self)
        self.pushButton_selectBDDB.clicked.connect(self.selectFiles)
        self.pushButton_exportCsv.clicked.connect(self.export_csv)  # OK按钮按下
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下

    def selectFiles(self):
        filePath, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(), "Feather Files(*.feather)")  # 打开文件对话框，选择文件
        self.lineEdit_PLPath.setText(filePath)

    def export_csv(self):
        def save2csv(df, path):
            with open(path, "w") as file:
                df.to_csv(file)
        if self.lineEdit_PLPath.text():
            pl_df = pd.read_feather(self.lineEdit_PLPath.text())  # 读取完好性数据
            output_path = self.lineEdit_PLPath.text() + ".csv"
            t = threading.Thread(target=save2csv, args=(pl_df, output_path))
            t.start()

    def on_accepted(self):
        # 开始完好性统计
        pl_df = pd.read_feather(self.lineEdit_PLPath.text())  # 读取完好性数据
        # 统计字段名称
        if self.checkBox_pos_horizontal_pl.isChecked():
            try:
                pl, pe = 'pos_horizontal_pl', 'pos_horizontal_pe'
                pepl_list = visual_integrity.integrity_series(pl_df, pl, pe)  # 单份数据的时间序列图统计
                self.plot_time_series(pepl_list, pl.split("_")[1])
            except Exception as e:
                self.output_msg("horizontal 统计失败，原因:" + str(e) + "\n")
        if self.checkBox_pos_lateral_pl.isChecked():
            try:
                pl, pe = 'pos_lateral_pl', 'pos_lateral_pe'
                pepl_list = visual_integrity.integrity_series(pl_df, pl, pe)  # 单份数据的时间序列图统计
                self.plot_time_series(pepl_list, pl.split("_")[1])
            except Exception as e:
                self.output_msg("lateral 统计失败，原因:" + str(e) + "\n")
        if self.checkBox_pos_longitudinal_pl.isChecked():
            try:
                pl, pe = 'pos_longitudinal_pl', 'pos_longitudinal_pe'
                pepl_list = visual_integrity.integrity_series(pl_df, pl, pe)  # 单份数据的时间序列图统计
                self.plot_time_series(pepl_list, pl.split("_")[1])
            except Exception as e:
                self.output_msg("longitudinal 统计失败，原因:" + str(e) + "\n")

        if self.checkBox_pos_vertical_pl.isChecked():
            try:
                pl, pe = 'pos_vertical_pl', 'pos_vertical_pe'
                pepl_list = visual_integrity.integrity_series(pl_df, pl, pe)  # 单份数据的时间序列图统计
                self.plot_time_series(pepl_list, pl.split("_")[1])
            except Exception as e:
                self.output_msg("vertical 统计失败，原因:" + str(e) + "\n")

        plt.show()
        self.close()  # 关闭

    # 时间序列图
    def plot_time_series(self, pepl_list, title):
        obj = PlotData()
        obj.fig.suptitle(title + " " + pepl_list['title'])
        obj.ax1 = plt.subplot(111)
        for df in pepl_list['value']:
            x = [item[0] for item in df['data']]
            y = [item[1] for item in df['data']]
            obj.PlotData(obj.ax1, x, y, df['name'], scatter=True if df['type'] == "scatter" else False)
        obj.ShowPlotFormat('time', 'meter')
        return obj

    def output_msg(self, msg):
        print(msg)




