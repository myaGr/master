import os
import time

from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog, QMenu, QTableWidgetItem

import UI.ui_Dialog_import_scenes


# 场景配置窗口
class DialogImportScenes(QDialog, UI.ui_Dialog_import_scenes.Ui_Dialog):
    my_signal = QtCore.pyqtSignal(list)

    def __init__(self):
        super(DialogImportScenes, self).__init__()
        self.setupUi(self)
        self.add_scenes = []  # 测试场景

        self.tableWidget_addScene.setContextMenuPolicy(Qt.CustomContextMenu)  # 允许打开上下文菜单
        self.tableWidget_addScene.customContextMenuRequested.connect(self.open_submenu)  # 绑定事件

        #  信号关联
        self.pushBtn_addScene.clicked.connect(self.onClickedAddScene)  # 配置场景按钮按下
        self.buttonBox.accepted.connect(self.on_accepted)  # OK按钮按下

    def add_scene(self, rowNum):
        self.tableWidget_addScene.insertRow(rowNum)  # 插入一行新行表格

    def delete_scene(self, rowNum):
        self.tableWidget_addScene.removeRow(rowNum)  # 删除当前行

    def open_submenu(self, pos):
        for i in self.tableWidget_addScene.selectionModel().selection().indexes():
            rowNum = i.row()
        submenu = QMenu()
        submenu.addAction("往上插入一行", lambda: self.add_scene(rowNum))
        submenu.addAction("往下插入一行", lambda: self.add_scene(rowNum + 1))
        submenu.addAction("删除本行数据", lambda: self.delete_scene(rowNum))

        globalPos = self.tableWidget_addScene.viewport().mapToGlobal(pos)  # 转换坐标系
        submenu.exec(globalPos)

    # 配置场景按钮按下
    def onClickedAddScene(self):
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        if not os.path.isfile(fileName):
            self.outputMsg(fileName + "文件不存在...")
            return
        try:
            self.add_scenes = []
            with open(fileName, 'r', encoding='utf-8', errors='ignore') as fileObj:
                data = fileObj.readlines()
                self.tableWidget_addScene.setRowCount(len(data))

                i = 0
                for line in data:
                    line = line.strip().split(",")
                    # print(line[0] + "_" + line[-1] + "： " + line[2] + " - " + line[3] + " (" + line[1] + "时间)")
                    self.tableWidget_addScene.setItem(i, 0, QTableWidgetItem(line[0]))
                    self.tableWidget_addScene.setItem(i, 1, QTableWidgetItem(line[4]))
                    self.tableWidget_addScene.setItem(i, 2, QTableWidgetItem(line[1]))
                    self.tableWidget_addScene.setItem(i, 3, QTableWidgetItem(line[2]))
                    self.tableWidget_addScene.setItem(i, 4, QTableWidgetItem(line[3]))
                    i = i + 1

        except Exception as e:
            self.outputMsg("场景读取失败，失败原因:" + str(e))

    def on_accepted(self):
        row = self.tableWidget_addScene.rowCount()  # 获取当前表格共有多少行
        if row > 0:
            for i in range(row):
                try:
                    scene = {"id": self.tableWidget_addScene.item(i, 0).text(),
                             "name": self.tableWidget_addScene.item(i, 1).text(),
                             "time_type": self.tableWidget_addScene.item(i, 2).text(),
                             "time": [self.tableWidget_addScene.item(i, 3).text(),
                                      self.tableWidget_addScene.item(i, 4).text()]}
                    self.add_scenes.append(scene)
                except Exception as e:
                    self.outputMsg(str(i + 1) + "场景异常：" + str(e))
        self.my_signal.emit(self.add_scenes)

    # 更新消息框内容
    @staticmethod
    def outputMsg(msg_str):  # textBrowser打印消息
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        print(msg)
