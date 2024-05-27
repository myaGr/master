from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QVBoxLayout, QMainWindow, QWidget, QMenu, QTreeWidgetItem, QMessageBox
from PyQt5.QtCore import Qt, QObject
from PyQt5.QtGui import *

import global_var
from UI.ui_MainWindow import Ui_MainWindow
from Dialogs.DialogImportRef import DialogImportRef
from Dialogs.DialogImportTest import DialogImportTest
from Dialogs.DialogImportScenes import DialogImportScenes
from Dialogs.DialogSetPlot import DialogSetPlot
from Dialogs.DialogConvertTime import DialogConvertTime
from Dialogs.DialogCalDistance import DialogCalDistance
from Dialogs.DialogSetConfig import DialogSetConfig
from Dialogs.DialogBddb2Mat import DialogBddb2Mat
from Dialogs.DialogSotif import DialogSotif
from Dialogs.DialogWireshark import DialogWireshark
from Dialogs.DialogGnssMultiPlot import DialogGnssMultiPlot
from Dialogs.DialogInsMultiPlot import DialogInsMultiPlot
from Utils.dataStatistics import DataStatistics, gnssErrorCal
from Standardize import dataParse, dataPreProcess
from Utils import gnssStatistics, gnssPlot, insStatistic, insPlot, df2Nmea
from Parse.HexDataParse import HexDataParse

import pyqtgraph as pg
import pandas as pd
import numpy as np
import threading
import datetime
import typing
import time
import math
import uuid
import os

import webbrowser
import tkinter.messagebox as msgbox
import requests
import json


# 工具主窗口
class MainWindow(QMainWindow, Ui_MainWindow):
    refInfo = QtCore.pyqtSignal(dict)
    gnssInfo = QtCore.pyqtSignal(dict)
    insInfo = QtCore.pyqtSignal(dict)
    bddbInfo = QtCore.pyqtSignal(dict)
    signalInfo = QtCore.pyqtSignal(dict)

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.date_time = None
        self.ref_data = {}  # 参考数据
        self.test_data = {}  # 单份测试数据
        self.gnss_test_data = []  # gnss测试数据集合
        self.ins_test_data = []  # ins测试数据集合
        self.scene_list = []  # 场景数据集合
        self.color_list = ["b", "g", "r", "darkturquoise", "m", "k", "saddlebrown"]  # 暂设定7种颜色
        self.color_list_gps = ["royalblue", "lime", "coral", "c", "violet", "grey", "chocolate"]  # GPS暂设定7种颜色
        self.set_config_data = None
        self.label = ""

        # 公用QT线程计数
        self.thread_i = 0
        self.thread_common = {}

        # 绘图配置初始化
        self.plot_config = {'time_type': 'gps', 'time': ['0', '0'],
                            'ins_plot_time_type': '计时时间（秒）', 'gnss_plot_time_type': 'utc时间',
                            'ins_plot_flags': {4: '固定解', 5: '浮点解', 2: '差分解', 1: '单点解', 0: '其他'},
                            'ins_plot_freq': 1,
                            'gnss_plot_flags': {4: '固定解', 5: '浮点解', 2: '差分解', 1: '单点解', 0: '其他'}}

        # loadUi("MainWindow.ui", self)
        self.setWindowTitle(global_var.get_value("TOOL_VERSION") + " Powered by 算法部 ")  # 设置工具标题内容

        # 下拉菜单
        # 1. 文件
        self.action_importRef.triggered.connect(lambda: self.open_dialog_import_ref())  # 点击导入参考数据
        self.action_importTest.triggered.connect(lambda: self.open_dialog_import_test())  # 点击导入测试数据
        self.action_importScenes.triggered.connect(lambda: self.open_dialog_import_scenes())  # 点击打开场景配置

        # 2. 视图
        self.action_setPlotView.triggered.connect(lambda: self.open_dialog_set_plot())  # 点击打开绘图配置

        # 2.1 INS 单图绘制
        self.action_loc_error.triggered.connect(lambda: self.ins_data_plot("位置偏差(水平、横向、纵向)历元分布图"))
        self.action_neg_vel_error.triggered.connect(lambda: self.ins_data_plot("速度误差(北东地)历元分布图"))
        self.action_frd_vel_error.triggered.connect(lambda: self.ins_data_plot("速度误差(前右下)历元分布图"))
        self.action_wheel_speed.triggered.connect(lambda: self.ins_data_plot("轮速历元分布图"))
        self.action_vel_wheelSpeed.triggered.connect(lambda: self.ins_data_plot("速度轮速对比历元分布图"))
        self.action_pos_error.triggered.connect(lambda: self.ins_data_plot("姿态误差(横滚、俯仰、航向)历元分布图"))
        self.action_coor_error.triggered.connect(lambda: self.ins_data_plot("坐标误差(经纬高)历元分布图"))
        self.action_mile_error.triggered.connect(lambda: self.ins_data_plot("里程(前右下)历元分布图"))
        self.action_acc_xyz.triggered.connect(lambda: self.ins_data_plot("加表(XYZ)历元分布图"))
        self.action_grox_xyz.triggered.connect(lambda: self.ins_data_plot("陀螺(XYZ)历元分布图"))

        self.action_ins_by_time.triggered.connect(lambda: self.ins_data_plot("历元间隔分布图"))
        self.action_timeGap_acc_grox.triggered.connect(lambda: self.ins_data_plot("加表陀螺历元间隔分布图"))
        self.action_timeGap_gps_imu.triggered.connect(lambda: self.ins_data_plot("相邻GPS-IMU时间间隔图"))
        self.action_traj.triggered.connect(lambda: self.ins_data_plot("同步数据轨迹图"))
        self.action_kalman_p.triggered.connect(lambda: self.ins_data_plot("Kalman.P历元分布图"))
        self.action_kalman_x.triggered.connect(lambda: self.ins_data_plot("Kalman.X历元分布图"))
        self.action_bias_by_time.triggered.connect(lambda: self.ins_data_plot("零偏历元分布图"))
        self.action_gps_status_precent.triggered.connect(lambda: self.ins_data_plot("GPS解状态占比饼状图"))
        self.action_errorbias.triggered.connect(lambda: self.ins_data_plot("统计误差分布图"))
        self.action_gps_error.triggered.connect(lambda: self.ins_data_plot("GPS偏差对比图"))
        self.action_INS_plot_all.triggered.connect(lambda: self.ins_data_plot("ALL"))

        # 2.2 INS 多图绘制
        self.actionINS_multiPlot.triggered.connect(lambda: self.open_dialog_ins_data_multiPlot())

        # 2.3 GNSS 单图绘制
        self.actionGNSS_gpsQuality.triggered.connect(lambda: self.gnss_data_plot("解状态占比饼状图"))
        self.actionGNSS_timeGap.triggered.connect(lambda: self.gnss_data_plot("历元间隔分布图"))
        self.actionGNSS_ageDiff.triggered.connect(lambda: self.gnss_data_plot("差分龄期统计图"))
        self.actionGNSS_gpsQualityPosError.triggered.connect(lambda: self.gnss_data_plot("解状态水平误差序列图"))
        self.actionGNSS_posError.triggered.connect(lambda: self.gnss_data_plot("位置误差(横向、纵向、水平)历元分布图"))
        self.actionGNSS_velError.triggered.connect(lambda: self.gnss_data_plot("速度误差(前向)历元分布图"))
        self.actionGNSS_angleError.triggered.connect(lambda: self.gnss_data_plot("姿态误差(航向)历元分布图"))
        self.actionGNSS_cdfError.triggered.connect(lambda: self.gnss_data_plot("统计误差CDF分布图"))
        self.action_GNSS_plot_all.triggered.connect(lambda: self.gnss_data_plot("ALL"))

        # 2.4 GNSS 多图绘制
        self.actionGNSS_multiPlot.triggered.connect(lambda: self.open_dialog_gnss_data_multiPlot())

        # 3. 统计
        self.action_staristics_GNSS.triggered.connect(lambda: self.on_clicked_save_excel("GNSS"))  # 输出GNSS统计
        self.action_staristics_INS.triggered.connect(lambda: self.on_clicked_save_excel("INS"))  # 输出GNSS统计

        # 4. 小工具
        self.action_convertTime.triggered.connect(lambda: self.open_dialog(DialogConvertTime()))  # 点击导间转换工具
        self.action_calDistiance.triggered.connect(lambda: self.open_dialog(DialogCalDistance()))  # 点击距离转换工具
        self.action_INS_mat.triggered.connect(lambda: self.open_dialog_bddb2mat())  # 点击 bddb2mat
        self.action_sotif_statistics.triggered.connect(lambda: self.open_dialog(DialogSotif()))  # Sotif统计窗口
        # self.action_pcpng2bin.triggered.connect(lambda: self.open_dialog(DialogWireshark()))  # 类似Wireshark
        self.action_pcpng2bin.triggered.connect(lambda: self.open_dialog_pcapng2bin())

        # 5.帮助
        self.action_introduction.triggered.connect(lambda: self.open_help_web())
        self.action_refresh.triggered.connect(lambda: self.download_new_version())

        #  绘图模块（pyqtgraph）
        win = pg.GraphicsLayoutWidget(show=True)  # 创建绘图区域
        win.setBackground('w')  # 绘图区域背景白色
        self.label = pg.LabelItem(justify='right')  # 右上角添加标签
        win.addItem(self.label)
        self.widget_plot = win.addPlot(row=1, col=0)  # 添加一个按钮和一个PlotWidget
        self.widget_plot.setAspectLocked(True)  # 设置横纵坐标长度一致

        centralWidget = QWidget()  # MainWindow的主体内容要通过一个widget包裹在一起，通过self.setCentralWidget设置
        main_layout = QVBoxLayout()
        main_layout.addWidget(win)
        centralWidget.setLayout(main_layout)
        self.setCentralWidget(centralWidget)  # 应用上述布局
        self.pushButton.clicked.connect(self.update_data_plot)  # 为[绘图]按钮注册一个回调函数

        #  数据窗口
        self.treeWidget_dataManage.setContextMenuPolicy(Qt.CustomContextMenu)  # 允许打开上下文菜单
        self.treeWidget_dataManage.customContextMenuRequested.connect(self.open_submenu)  # 绑定事件

        # 解析参考文件QT子线程
        self.thread_parse_ref = ParseRef(self)  # 参考数据解析, QT子线程实例化
        self.refInfo.connect(self.thread_parse_ref.getInfo)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
        self.thread_parse_ref.updated.connect(self.output_msg)  # B类（QT子线程）发送信号， 调用A类(主线程)的槽函数
        self.thread_parse_ref.ref_signal.connect(self.receive_ref_dataframe)  # 更新主界面

        # 解析GNSS数据QT子线程
        self.thread_compare_gnss = CompareMainGnss(self)  # 测试数据解析, QT子线程实例化
        self.gnssInfo.connect(self.thread_compare_gnss.getInfo)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
        self.thread_compare_gnss.updated.connect(self.output_msg)  # B类（QT子线程）发送信号， 调用A类(主线程)的槽函数
        self.thread_compare_gnss.test_signal.connect(self.receive_gnss_dataframe)  # 更新主界面

        # 解析INS数据QT子线程
        self.thread_compare_ins = CompareMainIns(self)  # 测试数据解析, QT子线程实例化
        self.insInfo.connect(self.thread_compare_ins.getInfo)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
        self.thread_compare_ins.updated.connect(self.output_msg)  # B类（QT子线程）发送信号， 调用A类(主线程)的槽函数
        self.thread_compare_ins.test_signal.connect(self.receive_ins_dataframe)  # 更新主界面

        # 解析INS数据转bddb
        self.thread_bddb2mat = AnalysisBddb(self)  # 测试数据解析, QT子线程实例化
        self.bddbInfo.connect(self.thread_bddb2mat.getInfo)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
        self.thread_bddb2mat.updated.connect(self.output_msg)

    def closeEvent(self, event):
        reply = QMessageBox.question(self, '警告', "是否确认退出工具?", QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)

        if reply == QMessageBox.Yes:
            for test_data in self.gnss_test_data + self.ins_test_data:
                for key in test_data.keys():
                    if '_path' in key and key != 'file_path' and os.path.exists(test_data[key]):
                        os.remove(test_data[key])
                if os.path.exists('./' + test_data["file_name"]):
                    os.rmdir('./' + test_data["file_name"])
            event.accept()
        else:
            event.ignore()

    def open_dialog(self, dialog):
        result = dialog.exec_()  # 显示对话框并等待用户响应
        if result:
            self.output_msg(str(result))

    # open
    def open_help_web(self):
        try:
            url = 'https://asensing.feishu.cn/docx/COJddHjGHolZ60xEDUGcCbEInHh'
            webbrowser.open_new_tab(url)
            self.output_msg('已打开软件说明书: ' + url)
        except Exception as e:
            self.output_msg('无法打开软件说明书')
            self.output_msg(str(e))

    def download_new_version(self):
        def checkVersion(now_exe_name):
            """
            @author: liqianwen
            :param now_exe_name:
            :return:
            """
            url = 'http://10.1.135.105/solutions/tools/version_test/' + now_exe_name
            # param = {'tool':now_exe_name}
            param = {}
            res_info = request_get(url, param)
            if 'fileName' in res_info.keys():
                yes_download = msgbox.askyesno(title='版本更新',
                                               message='最新版本为 %s ，是否更新？\n（若使用此版本，请选否）' % res_info[
                                                   'fileName'])
                if yes_download:
                    download_url = 'http://10.1.135.105/solutions/download/tools/' + res_info['fileName']
                    res = requests.get(download_url, param)
                    self.output_msg('工具下载中：' + download_url)
                    with open(res_info['fileName'], 'wb') as code:
                        code.write(res.content)

                    self.output_msg('提示：最新工具已下载。')
            else:
                # msgbox.askyesno(title='版本更新', message=res_info.message)
                msgbox.showinfo('Failed to check', res_info['message'])

        def request_get(url_get, param_get):
            """
            @author: liqianwen
            :param url_get:
            :param param_get:
            :return:
            """
            fails = 0
            text = {}
            while True:
                try:
                    if fails >= 5:
                        print('网络连接出现问题, 无法检查当前版本')
                        text['message'] = "网络连接失败,无法检查当前版本。"
                        return text

                    ret = requests.get(url=url_get, params=param_get, timeout=10)
                    if ret.status_code == 200:
                        text = json.loads(ret.text)
                        return text
                    else:
                        fails += 1
                        continue
                except Exception as e:
                    fails += 1
                    print('网络连接出现问题', e)
                    text['message'] = '网络连接失败,无法检查当前版本。'
            return text

        # 版本检测
        tool_version = global_var.get_value("TOOL_VERSION")
        try:
            checkVersion(tool_version + '.exe')
        except Exception as e:
            self.output_msg(str(e))

    # 导入参考文件窗口
    def open_dialog_import_ref(self):
        dialog = DialogImportRef()
        dialog.my_signal.connect(self.parse_ref_data)
        dialog.exec_()  # 显示对话框并等待用户响应

    # 导入测试文件窗口
    def open_dialog_import_test(self):
        dialog = DialogImportTest()
        dialog.my_signal.connect(self.parse_test_data)
        dialog.exec_()  # 显示对话框并等待用户响应

    # 导入场景配置文件窗口
    def open_dialog_import_scenes(self):
        dialog = DialogImportScenes()
        dialog.my_signal.connect(self.parse_scenes)
        dialog.exec_()  # 显示对话框并等待用户响应

    # 配置统计绘图参数
    def open_dialog_set_plot(self):
        dialog = DialogSetPlot()
        dialog.my_signal.connect(self.set_plot_config)
        dialog.exec_()  # 显示对话框并等待用户响应

    # 配置GNSS综合误差图
    def open_dialog_gnss_data_multiPlot(self):
        dialog = DialogGnssMultiPlot()
        dialog.my_signal.connect(self.gnss_multi_plot)
        dialog.exec_()  # 显示对话框并等待用户响应

    # 配置INS综合误差图
    def open_dialog_ins_data_multiPlot(self):
        dialog = DialogInsMultiPlot()
        dialog.my_signal.connect(self.ins_multi_plot)
        dialog.exec_()  # 显示对话框并等待用户响应

    # 配置杆臂值窗口
    def open_dialog_set_config(self):
        dialog = DialogSetConfig()
        dialog.config_signal.connect(self.set_config_bpox)
        dialog.exec_()  # 显示对话框并等待用户响应

    # 配置mat生成
    def open_dialog_bddb2mat(self):
        dialog = DialogBddb2Mat()
        dialog.my_signal.connect(self.parse_bddb2mat)
        dialog.exec_()  # 显示对话框并等待用户响应

    def open_dialog_pcapng2bin(self):
        self.output_msg('请注意，本机需安装wireshark才可使用该功能， 否则生成文件为空。'
                        '\n如出现tshark.exe弹窗，属于正常情况，待数据分析完会自动关闭，请勿提前手动关闭窗口。'
                        '\nWireshark安装包可通过网址下载：http://10.1.135.105/solutions/download/tools/Wireshark-4.2.3-x64.exe')
        dialog = DialogWireshark()
        result = dialog.exec_()  # 显示对话框并等待用户响应
        if dialog.messages:
            self.output_msg(str(dialog.messages))

    # 更新消息框内容
    def output_msg(self, msg_str):  # textBrowser打印消息
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + str(msg_str)
        self.textBrowser_printLog.append(msg)
        if '【导入参考数据】' in msg_str or '【导入INS测试数据】' in msg_str or '【导入GNSS测试数据】' in msg_str:
            self.action_importRef.setEnabled(False)
            self.action_importTest.setEnabled(False)
        if '【数据导入完毕】' in msg_str or '【导入失败】' in msg_str:
            self.action_importRef.setEnabled(True)
            self.action_importTest.setEnabled(True)

    # 更新数据管理列表
    def update_data_manage(self, file_name, data_type):
        if data_type == "参考数据":
            n = self.treeWidget_dataManage.topLevelItemCount()  # 获取根节点数量
            for i in range(n - 1, -1, -1):
                item = self.treeWidget_dataManage.topLevelItem(i)  # 循环获取根节点, 删除原有参考数据
                item.removeChild(item)
            self.gnss_test_data = []  # 初始化测试数据
            self.ins_test_data = []
        if file_name:
            root = QTreeWidgetItem(self.treeWidget_dataManage)
            root.setText(0, file_name)
            # root.setCheckState(0, Qt.Checked)
            child1 = QTreeWidgetItem(root)
            child1.setText(0, data_type)

    # 数据管理列表右键菜单栏操作
    def open_submenu(self, pos):
        for index in self.treeWidget_dataManage.selectionModel().selection().indexes():
            data = index.data()
            row = index.parent().row()
            if row == -1:
                row = index.row()
            else:
                data = index.parent().data()
            item_to_select = self.treeWidget_dataManage.topLevelItem(row)  # 选中该索引
            self.treeWidget_dataManage.setCurrentItem(item_to_select)
            print("Selected item is: ", data)

        submenu = QMenu()
        submenu.addAction("配置杆臂值", lambda: self.on_action_set_config(data))
        submenu.addAction("导出csv", lambda: self.on_action_save2csv_clicked(data))
        submenu.addAction("转为nmea", lambda: self.on_action_save2nmea_clicked(data))
        submenu.addAction("删除数据", lambda: self.on_action_delete_data_clicked(data))

        globalPos = self.treeWidget_dataManage.viewport().mapToGlobal(pos)  # 转换坐标系
        submenu.exec(globalPos)

    # 打开配置杆臂值窗口
    def on_action_set_config(self, data):
        self.set_config_data = data
        self.open_dialog_set_config()

    # 计算杆臂值
    def cal_bpox(self, sync_df, config_signal):
        utcDate = sync_df["utcDate"][0]  # 获取UTC日期
        start = sync_df['unixTime'][0]  # 初始化开始时间
        end = sync_df['unixTime'][len(sync_df['unixTime']) - 1]  # 初始化结束时间
        try:
            # 统一时间类型为unix时间
            start, end = dataPreProcess.time_to_unix([config_signal[0], config_signal[1]], config_signal[2], utcDate,
                                                     start, end)
            # 根据指定时间段筛选
            sync_df = sync_df[(sync_df['unixTime'] >= start) & (sync_df['unixTime'] <= end)].copy()
            # 杆臂小于5厘米忽略
            bpox_x = sync_df["longitudinal_error"].median() if abs(sync_df["longitudinal_error"].median()) > 0.05 else 0
            bpox_y = sync_df["horizontal_error"].median() if abs(sync_df["horizontal_error"].median()) > 0.05 else 0
            bpox_z = sync_df["horizontal_error"].median() if abs(sync_df["horizontal_error"].median()) > 0.05 else 0
            bpox = [bpox_x, bpox_y, bpox_z]
            # 计算双天线航向固定偏差
            bpox_heading = 0
            if "double_heading_error" in sync_df:
                bpox_heading = sync_df["double_heading_error"].median() if not math.isnan(sync_df["double_heading_error"].median()) else 0
            return bpox, bpox_heading
        except Exception as e:
            self.output_msg("【提示】杆臂值计算失败， 失败原因：" + str(e))

    # 补偿杆臂值计算
    def set_config_bpox(self, config_signal):
        if self.ref_data["file_name"] == self.set_config_data:
            self.output_msg("【注意】参考数据，不支持配置杆臂值")
            return
        if self.ref_data["test_type"] == '静态':
            self.output_msg("【注意】静态测试，不支持配置杆臂值")
            return
        for test_data in self.gnss_test_data:
            if test_data["file_name"] == self.set_config_data:
                if "time" in config_signal:  # 输入的静态时间
                    self.output_msg("【配置杆臂值】" + test_data["file_name"] + "静态时间段为：" + str(
                        [config_signal["time"][0], config_signal["time"][1]]) + ", 时间类型为：" + config_signal["time"][2])
                    test_data["bpox"], test_data["bpox_heading"] = self.cal_bpox(test_data["sync_df"], config_signal["time"])
                    self.output_msg("计算杆臂值，距离为：" + str(test_data["bpox"]) + ", 双天线航向为：" + str(
                        test_data["bpox_heading"]))
                elif "bpos" in config_signal:  # 输入杆臂值
                    test_data["bpox"] = config_signal["bpos"]
                    test_data["bpox_heading"] = 0
                    self.output_msg("【配置杆臂值】" + test_data["file_name"] + "已配置为：" + str(config_signal["bpos"]))
                # 解析GNSS数据QT子线程
                self.output_msg("【更新测试数据】" + test_data["file_name"] + "补偿杆臂后误差计算。")
                test_data["sync_df"] = gnssErrorCal(test_data["sync_df"], test_data["bpox"], test_data["bpox_heading"])
                self.output_msg("【更新完毕】" + "\n")
        for test_data in self.ins_test_data:
            # self.output_msg("INS暂仅支持姿态杆臂值配置")
            if test_data["file_name"] == self.set_config_data:
                if "bpos" in config_signal:  # 输入杆臂值:
                    test_data["bpox"] = config_signal["bpos"]
                    # self.output_msg("【配置杆臂值】" + test_data["file_name"] + "已配置为：" + str(config_signal["bpos"]))
                if "posture_bpox" in config_signal:  # [0,0,0],  # roll pitch yaw
                    test_data["posture_bpox"] = config_signal["posture_bpox"]
                    self.output_msg("【配置姿态杆臂值】" + test_data["file_name"] + "已配置为：" + str(config_signal["posture_bpox"]))
            elif test_data["file_name"] + "【gps】" == self.set_config_data:
                if "bpos" in config_signal:  # 输入杆臂值:
                    test_data["bpox_gps"] = config_signal["bpos"]
                    # self.output_msg("【配置杆臂值】" + test_data["file_name"] + "【gps】" + "已配置为：" + str(config_signal["bpos"]))
                if "posture_bpox" in config_signal:  # [0,0,0],  # roll pitch yaw
                    test_data["posture_bpox_gps"] = config_signal["posture_bpox"]
                    self.output_msg("【配置姿态杆臂值】" + test_data["file_name"] + "【gps】" + "已配置为：" + str(config_signal["posture_bpox"]))

    # 导出csv
    def on_action_save2csv_clicked(self, data):
        def save2csv(df, path, sync_df=None, sync_path=None):
            # QT 线程初始化
            self.thread_i = self.thread_i + 1
            self.thread_common[self.thread_i] = CommonQtThread(self)
            self.signalInfo.connect(self.thread_common[self.thread_i].getInfo)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
            self.thread_common[self.thread_i].updated.connect(self.output_msg)

            signal_dict = {
                "function_type": "df2Csv",
                "output_df": df,
                "output_path": path,
                "output_sync_df": sync_df,
                "output_sync_path": sync_path
            }
            self.signalInfo.emit(signal_dict)  # 传递子线程参数
            self.thread_common[self.thread_i].start()  # 子线程开始执行

        if self.ref_data:
            if 'file_name' in self.ref_data and "df" in self.ref_data:
                if self.ref_data["file_name"] == data:
                    output_path = self.ref_data["file_path"] + ".csv"
                    save2csv(self.ref_data["df"], output_path)

        for test_data in self.gnss_test_data + self.ins_test_data:
            if test_data["file_name"] == data:
                output_path = test_data["file_path"] + ".csv"
                output_path_sync = test_data["file_path"] + "_sync.csv"  # 同步数据导出
                if 'sync_df' in test_data:
                    save2csv(test_data["df"], output_path, sync_df=test_data["sync_df"], sync_path=output_path_sync)
                else:
                    save2csv(test_data["df"], output_path)

            if test_data["file_name"] + "【gps】" == data:
                output_path = test_data["file_path"] + "【gps】.csv"
                output_path_sync = test_data["file_path"] + "_sync【gps】.csv"
                if 'sync_df_gps' in test_data:
                    save2csv(test_data["df"], output_path, sync_df=test_data["sync_df_gps"], sync_path=output_path_sync)
                else:
                    save2csv(test_data["df"], output_path)

    # 输出nmea
    def on_action_save2nmea_clicked(self, data):
        def save2Nmea(df, path):
            # QT 线程初始化
            self.thread_i = self.thread_i + 1
            self.thread_common[self.thread_i] = CommonQtThread(self)
            self.signalInfo.connect(self.thread_common[self.thread_i].getInfo)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
            self.thread_common[self.thread_i].updated.connect(self.output_msg)

            signal_dict = {
                "function_type": "df2Nmea",
                "output_df": df,
                "output_path": path
            }
            self.signalInfo.emit(signal_dict)  # 传递子线程参数
            self.thread_common[self.thread_i].start()  # 子线程开始执行

        if self.ref_data:
            if self.ref_data["file_name"] == data and "df" in self.ref_data:
                output_path = self.ref_data["file_path"] + ".nmea"
                save2Nmea(self.ref_data["df"], output_path)

        for test_data in self.gnss_test_data + self.ins_test_data:
            if test_data["file_name"] == data:
                output_path = test_data["file_path"] + ".nmea"
                save2Nmea(test_data["df"], output_path)

            if test_data["file_name"] + "【gps】" == data:
                output_path = test_data["file_path"] + "【gps】.nmea"
                save2Nmea(test_data["df_gps"], output_path)

    # 删除数据
    def on_action_delete_data_clicked(self, data):
        n = self.treeWidget_dataManage.topLevelItemCount()  # 获取根节点数量
        for i in range(0, n):
            item = self.treeWidget_dataManage.topLevelItem(i)  # 循环获取根节点
            if item.text(0) == data:
                item.removeChild(item)  # 从数据管理列表删除
                if item.child(0).text(0) == "参考数据":  # 删除对应参数
                    self.ref_data = {}
                else:
                    for test_data in self.gnss_test_data:
                        if test_data["file_name"] == data:
                            for key in test_data.keys():
                                if '_path' in key and key != 'file_path' and os.path.exists(test_data[key]):
                                    os.remove(test_data[key])
                            self.gnss_test_data.remove(test_data)
                    for test_data in self.ins_test_data:
                        if test_data["file_name"] == data:
                            for key in test_data.keys():
                                if '_path' in key and key != 'file_path' and os.path.exists(test_data[key]):
                                    os.remove(test_data[key])
                            self.ins_test_data.remove(test_data)
                        if test_data["file_name"] + "【gps】" == data:
                            test_data.pop("df_gps")
                            if "sync_df_gps" in test_data:
                                test_data.pop("sync_df_gps")
                            if "sync_gps_ins" in test_data:
                                test_data.pop("sync_gps_ins")
                            if "df" not in test_data:
                                self.ins_test_data.remove(test_data)
                self.output_msg("【删除数据】已删除：" + data + "\n")

    # 参考数据解析
    def parse_ref_data(self, signal_dict):
        if signal_dict['test_type'] == "无参考":
            self.ref_data = signal_dict
            self.output_msg("测试类型为：" + signal_dict['test_type'] + "【注意】目前仅支持INS与自身GPS对比")
            self.update_data_manage(None, "参考数据")  # 更新数据列表
            self.update_data_plot()  # 更新数据画图
        elif signal_dict["static_ref_type"] == "参考点坐标":  # 静态测试
            self.refInfo.emit(signal_dict)  # 传递子线程参数
            self.ref_data = signal_dict
            self.output_msg("【导入参考数据】")
            self.output_msg("测试类型为：" + signal_dict['test_type'] + "测试， 参考纬度(°)：" + signal_dict["latitude"]
                            + "，经度(°)：" + signal_dict["longitude"] + "，高度(米)：" + signal_dict["ellHeight"] + "。")
            self.ref_data["file_name"] = str(
                [signal_dict["latitude"], signal_dict["longitude"], signal_dict["ellHeight"]])
            # 更新数据列表
            self.update_data_manage(self.ref_data["file_name"], "参考数据")
            # 更新数据画图
            self.update_data_plot()
            self.output_msg("【数据导入完毕】\n")
        else:
            self.date_time = signal_dict["date_time"]
            self.refInfo.emit(signal_dict)  # 传递子线程参数
            self.thread_parse_ref.start()  # 子线程开始执行
            self.ref_data = signal_dict

    # 测试数据解析
    def parse_test_data(self, signal_dict):
        exit_gnss_test, exit_ins_test = False, False
        self.thread_compare_gnss.test_data_all = []
        self.thread_compare_ins.test_data_all = []
        # print(signal_dict)
        for test_info in signal_dict:
            if "file_name" in self.ref_data:
                if os.path.basename(test_info["file_path"]) == self.ref_data["file_name"]:
                    self.output_msg("【导入失败】注意：无法导入2份同名的文件，请修改文件名!")
                    return
            for test_data in self.gnss_test_data + self.ins_test_data:
                if os.path.basename(test_info["file_path"]) == test_data["file_name"]:
                    self.output_msg("【导入失败】注意：无法导入2份同名的文件，请修改文件名!")
                    return

            if test_info['algorithm_type'] == 'GNSS':
                exit_gnss_test = True
                if self.ref_data == {}:
                    self.output_msg("【注意】请先选择参考数据！GNSS仅支持和参考对比")
                    return
                test_info["test_type"] = self.ref_data["test_type"]
                if self.ref_data["test_type"] == "静态":  # 静态
                    test_info["ref_data"] = [self.ref_data["latitude"], self.ref_data["longitude"],
                                             self.ref_data["ellHeight"]]
                else:  # 动态
                    self.date_time = test_info["date_time"]
                    # 暂存为feather格式
                    feather_id = uuid.uuid3(uuid.NAMESPACE_URL, self.ref_data["file_path"])
                    feather_path = './ref_' + str(feather_id) + '.feather'
                    self.ref_data["df"].to_feather(feather_path)
                    test_info["ref_data"] = self.ref_data["file_name"]
                    test_info["ref_feather_path"] = feather_path

                test_info["file_name"] = os.path.basename(test_info["file_path"])
                self.gnssInfo.emit(test_info)  # 传递子线程参数
            elif test_info['algorithm_type'] == 'INS':
                exit_ins_test = True
                if self.ref_data == {}:
                    test_info["test_type"] = '无参考'
                    test_info["date_time"] = str(datetime.date.today())
                    self.output_msg("【注意】未指定参考数据和测试时间，默认：无参考，测试日期：" + test_info["date_time"])
                    # self.insInfo.emit(test_info)  # 传递子线程参数
                    # self.thread_compare_ins.start()  # 子线程开始执行
                elif self.ref_data["test_type"] == "无参考":
                    test_info["test_type"] = '无参考'
                    test_info["date_time"] = self.ref_data["date_time"]
                    # self.insInfo.emit(test_info)  # 传递子线程参数
                    # self.thread_compare_ins.start()  # 子线程开始执行
                elif self.ref_data["test_type"] == "动态":
                    test_info["test_type"] = self.ref_data["test_type"]
                    test_info["date_time"] = self.ref_data["date_time"]
                    test_info["ref_data"] = self.ref_data["file_name"]
                    # 暂存为feather格式
                    feather_id = uuid.uuid3(uuid.NAMESPACE_URL, self.ref_data["file_path"])
                    feather_path = './ref_' + str(feather_id) + '.feather'
                    self.ref_data["df"].to_feather(feather_path)
                    test_info["ref_feather_path"] = feather_path
                elif self.ref_data["test_type"] == "静态":
                    self.output_msg("【注意】INS暂不支持静态测试")
                    return
                else:
                    self.output_msg(self.ref_data["test_type"] + "待开发")
                    return

                test_info["file_name"] = os.path.basename(test_info["file_path"])
                print(test_info)
                self.insInfo.emit(test_info)  # 传递子线程参数
        if exit_gnss_test:
            self.thread_compare_gnss.start()  # 子线程开始执行
        if exit_ins_test:
            self.thread_compare_ins.start()  # 子线程开始执行

    def parse_bddb2mat(self, signal_dict):
        if len(signal_dict['data_info']) == 0:
            self.output_msg("未选择文件")

        self.bddbInfo.emit(signal_dict)  # 传递子线程参数
        self.thread_bddb2mat.start()  # 子线程开始执行

    # 场景数据解析
    def parse_scenes(self, scene_list):
        self.output_msg("【导入场景信息】")
        try:
            for scene in scene_list:
                self.output_msg(scene["id"] + "_" + scene["name"] + "： " +
                                scene["time"][0] + " - " + scene["time"][1] + " (" + scene["time_type"] + "时间)")
            self.output_msg("【场景导入完毕】\n")
            self.scene_list = scene_list
        except Exception as e:
            self.output_msg("【场景导入失败】异常原因：" + str(e))

    # 更新参考数解析结果
    def receive_ref_dataframe(self, json_data):
        try:
            # 更新参考数据
            self.ref_data["feather_path"] = json_data
            self.ref_data["df"] = pd.read_feather(json_data)
            os.remove(json_data)  # 删除临时数据

            if self.ref_data["test_type"] == "动态":  # 静态参考文件
                self.ref_data["file_name"] = os.path.basename(self.ref_data["file_path"])
                # f = os.path.splitext(file)[0] # 去后缀
            elif self.ref_data["test_type"] == "静态":  # 静态参考文件的情况
                self.ref_data["file_name"] = self.ref_data["file_name"] + "【" + self.ref_data["ref_cal_Type"] + "】"
                if self.ref_data["ref_cal_Type"] == "均值":
                    self.ref_data["latitude"] = self.ref_data["df"]['latitude'].mean()
                    self.ref_data["longitude"] = self.ref_data["df"]['longitude'].mean()
                    self.ref_data["ellHeight"] = self.ref_data["df"]['ellHeight'].mean()
                else:
                    self.ref_data["latitude"] = self.ref_data["df"]['latitude'].median()
                    self.ref_data["longitude"] = self.ref_data["df"]['longitude'].median()
                    self.ref_data["ellHeight"] = self.ref_data["df"]['ellHeight'].median()
                self.ref_data.pop('df')  # 释放内存
                self.output_msg("参考数据计算方法：" + self.ref_data["ref_cal_Type"] +
                                "。 参考纬度：" + str(self.ref_data["latitude"]) +
                                "，经度：" + str(self.ref_data["longitude"]) +
                                "，高度：" + str(self.ref_data["ellHeight"]) + "。")
            else:
                pass
            # 更新数据画图
            self.update_data_plot()
            # 更新数据列表
            self.update_data_manage(self.ref_data["file_name"], "参考数据")
        except Exception as e:
            self.output_msg("【导入失败】异常原因：" + str(e))

    # 更新gnss测试数解析结果
    def receive_gnss_dataframe(self, test_info):
        try:
            self.test_data = test_info
            # 更新gnss数据i
            if test_info['df_path'] != '':
                self.test_data["df"] = pd.read_feather(test_info['df_path'])
            if test_info["sync_df_path"] != '':
                self.test_data["sync_df"] = pd.read_feather(test_info["sync_df_path"])
            self.gnss_test_data.append(self.test_data)  # 测试数据集合
            # 更新数据列表
            self.update_data_manage(self.test_data["file_name"], "gnss测试数据")
            # 更新数据画图
            self.update_data_plot()
            self.test_data = None
        except Exception as e:
            self.output_msg("【导入失败】异常原因：" + str(e))

    # 更新ins测试数解析结果
    def receive_ins_dataframe(self, test_info):
        self.output_msg('该INS数据解析完成，数据接收中...')
        try:
            self.test_data = test_info
            # 更新ins数据
            if self.test_data['df_path'] != "":
                self.test_data["df"] = pd.read_feather(self.test_data['df_path'])
            else:
                self.test_data["df"] = pd.DataFrame()
            # os.remove(json_data)
            if self.test_data['sync_df_path'] != "":
                self.test_data["sync_df"] = pd.read_feather(self.test_data['sync_df_path'])

                scene_dict = {
                    'name': '0_全程_all_' + '.'.join(self.test_data['file_path'].split('/')[-1].split('.')[:-1])
                    , 'percent': 1, 'data': self.test_data["sync_df"]}
                diff_info = self.ins_cal_error_diffs([scene_dict])[0]
                if 'cal_info' in diff_info.keys():
                    for i in ['utcTime', 'velocity_x', 'velocity']:
                        if i in diff_info['cal_info'].keys():
                            diff_info['cal_info'].pop(i)
                    self.test_data["sync_df"] = pd.concat([self.test_data["sync_df"], pd.DataFrame(diff_info['cal_info'])], axis=1)

                # os.remove(json_data_sync)

            if self.test_data['df_gps_path'] != "":
                self.test_data["df_gps"] = pd.read_feather(self.test_data['df_gps_path'])
                # os.remove(self.test_data['df_gps_path'])
            if self.test_data['sync_df_gps_path'] != "":
                self.test_data["sync_df_gps"] = pd.read_feather(self.test_data['sync_df_gps_path'])

                scene_dict = {
                    'name': '0_全程_all_' + '.'.join(self.test_data['file_path'].split('/')[-1].split('.')[:-1]),
                    'percent': 1, 'data': self.test_data["sync_df_gps"]}
                diff_info = self.ins_cal_error_diffs([scene_dict], data_type='sync_df_gps')[0]
                if 'cal_info' in diff_info.keys():
                    for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime', 'velocity_x', 'velocity']:
                        if i in diff_info['cal_info'].keys():
                            diff_info['cal_info'].pop(i)
                    self.test_data["sync_df_gps"] = pd.concat(
                        [self.test_data["sync_df_gps"], pd.DataFrame(diff_info['cal_info'])], axis=1)
            # os.remove(self.test_data['df_gps_path'])

            if self.test_data['df_pdata_path'] != "":
                # df_name = '_'.join(self.test_data['df_pdata_path'].split('_')[-2:])
                if self.test_data['df_pdata_path'].split('.')[-1] == 'npy':
                    self.test_data['df_pdata'] = np.load(self.test_data['df_pdata_path'], allow_pickle=True).item()
                # os.remove(json_data_path)

            if self.test_data['df_imu_path'] != "":
                self.test_data["df_imu"] = pd.read_feather(self.test_data['df_imu_path'])
            if self.test_data['df_vehicle_path'] != "":
                self.test_data["df_vehicle"] = pd.read_feather(self.test_data['df_vehicle_path'])

            if self.test_data['sync_gps_ins_path'] != "":
                self.test_data["sync_gps_ins"] = pd.read_feather(self.test_data['sync_gps_ins_path'])

                # 后轮轴中心补PData里的杆臂值
                scene_dict = {'name': '0_全程_all_' + '.'.join(self.test_data['file_path'].split('/')[-1].split('.')[:-1]),
                                     'percent': 1, 'data': self.test_data["sync_gps_ins"]}
                lbgc = [0, 0, 0]
                if self.test_data["output_pos"] == "后轮轴中心" and 'df_pdata' in self.test_data.keys():
                    if len(self.test_data['df_pdata']['Lbgc']) > 0:
                        lbgc = self.test_data['df_pdata']['Lbgc'][0]
                        lbgc = [-x for x in lbgc]  # 注意： lbgc为在车体坐标系下， 天线到后轮轴中心的距离，如需把天线位置补偿到后轮轴中心，补偿值取负数
                        self.output_msg("【补偿杆臂值】输出位置为后轮轴中心，自动补偿杆臂Lbgc= " + str(lbgc))
                diff_info = self.ins_cal_error_diffs([scene_dict], data_type='sync_gps_ins', lbgc=lbgc)[0]

                if 'cal_info' in diff_info.keys():
                    for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime', 'velocity_x', 'velocity']:
                        if i in diff_info['cal_info'].keys():
                            diff_info['cal_info'].pop(i)
                    self.test_data["sync_gps_ins"] = pd.concat([self.test_data["sync_gps_ins"], pd.DataFrame(diff_info['cal_info'])], axis=1)

            self.ins_test_data.append(self.test_data)  # 测试数据集合

            # 更新数据列表
            if self.test_data['df_path'] != "":
                self.update_data_manage(self.test_data["file_name"], "ins测试数据")
            if self.test_data['df_gps_path'] != "":
                self.update_data_manage(self.test_data["file_name"] + "【gps】", "ins测试数据")
            # 更新数据画图
            self.update_data_plot()

            self.test_data = None
        except Exception as e:
            self.output_msg("【导入失败】异常原因：" + str(e))

    # 更新绘图效果
    def update_data_plot(self):
        self.widget_plot.clear()  # 清除当前画图
        self.widget_plot.addLegend()  # 添加图例
        i = 0
        pos0 = None
        if 'test_type' not in self.ref_data.keys():
            for test_data in self.ins_test_data:
                if "df" in test_data and len(test_data["df"]) > 0:
                    if not pos0:
                        pos0 = [test_data["df"]["latitude"][0], test_data["df"]["longitude"][0],
                                test_data["df"]["ellHeight"][0]]
                    test_coords = DataStatistics().pos_covert(test_data["df"]["latitude"],
                                                              test_data["df"]["longitude"],
                                                              test_data["df"]["ellHeight"], pos0)
                    self.widget_plot.plot(test_coords[1], test_coords[0], pen=self.color_list[i], name=test_data["file_name"])
                if "df_gps" in test_data:
                    test_coords = DataStatistics().pos_covert(test_data["df_gps"]["latitude"],
                                                              test_data["df_gps"]["longitude"],
                                                              test_data["df_gps"]["ellHeight"], pos0)

                    self.widget_plot.plot(test_coords[1], test_coords[0],
                                          pen=pg.mkPen(self.color_list_gps[i], style=QtCore.Qt.DashLine),
                                          name=test_data["file_name"] + "【gps】")
                i = i + 1
                if i == len(self.color_list):
                    i = 0
        elif self.ref_data["test_type"] == "动态" and 'df' in self.ref_data.keys():
            # 统一初始位置
            if len(self.ref_data["df"]) == 0:
                return
            pos0 = [self.ref_data["df"]["latitude"][0], self.ref_data["df"]["longitude"][0],
                    self.ref_data["df"]["ellHeight"][0]]
            # 参考数据绘制轨迹图
            if "df" in self.ref_data:
                ref_coords = DataStatistics().pos_covert(self.ref_data["df"]["latitude"],
                                                         self.ref_data["df"]["longitude"],
                                                         self.ref_data["df"]["ellHeight"], pos0)
                self.widget_plot.plot(ref_coords[1], ref_coords[0], pen=pg.mkPen("orange", width=2),
                                      name=self.ref_data["file_name"])
            # GNSS 或 INS测试数据绘制轨迹图
            for test_data in self.gnss_test_data + self.ins_test_data:
                if "df" in test_data and len(test_data['df']) > 0:
                    test_coords = DataStatistics().pos_covert(test_data["df"]["latitude"],
                                                              test_data["df"]["longitude"],
                                                              test_data["df"]["ellHeight"], pos0)
                    self.widget_plot.plot(test_coords[1], test_coords[0], pen=self.color_list[i], name=test_data["file_name"])
                if "df_gps" in test_data and len(test_data['df_gps']) > 0:
                    test_coords = DataStatistics().pos_covert(test_data["df_gps"]["latitude"],
                                                              test_data["df_gps"]["longitude"],
                                                              test_data["df_gps"]["ellHeight"], pos0)

                    self.widget_plot.plot(test_coords[1], test_coords[0],
                                          pen=pg.mkPen(self.color_list_gps[i], style=QtCore.Qt.DashLine),
                                          name=test_data["file_name"] + "【gps】")
                i = i + 1
                if i == len(self.color_list):
                    i = 0

        elif self.ref_data["test_type"] == "静态":  # 静态绘图
            for test_data in self.gnss_test_data:  # 测试数据绘制轨迹图
                # 计算参考位置（初始位置）
                lenArr = len(test_data["sync_df"]['longitude'])
                lon0 = np.sum(np.array(test_data["sync_df"]['longitude_x'])) / lenArr
                lat0 = np.sum(np.array(test_data["sync_df"]['latitude_x'])) / lenArr
                high0 = np.sum(np.array(test_data["sync_df"]['ellHeight_x'])) / lenArr
                pos0 = [lat0, lon0, high0]
                # 地理坐标系转平面坐标系
                coordinate = DataStatistics().pos_covert(test_data["sync_df"]['latitude'],
                                                         test_data["sync_df"]['longitude'],
                                                         test_data["sync_df"]['ellHeight'],
                                                         pos0)
                results = DataStatistics.pos_error_cal(coordinate)

                # 保存过程值
                test_data["sync_df"]["north"], test_data["sync_df"]["east"], test_data["sync_df"]["ground"] = \
                    coordinate[0], coordinate[1], coordinate[2]
                test_data["sync_df"]["horizontal_error"] = results["horizontal_error"]
                test_data["sync_df"]["elevation_error"] = test_data["sync_df"]['ellHeight'] - high0

                # 计算水平误差与偏差角
                radius = results["horizontal_error"]
                theta = results["theta"]

                # Transform to cartesian and plot
                x = radius * np.cos(theta)
                y = radius * np.sin(theta)
                self.widget_plot.plot(x, y, pen=self.color_list[i], name=test_data["file_name"],
                                      symbol='o', symbolBrush=self.color_list[i], symbolSize=5)
                i = i + 1
                if i == len(self.color_list):
                    i = 0

            # 设置靶心图刻度
            rad_list = [0.05, 0.1, 0.2, 0.3, 0.5, 1, 2, 3, 5]

            # 设置靶心图样式
            self.widget_plot.addLegend()  # 添加图例
            self.widget_plot.addLine(x=0, pen=(200, 200, 200))
            self.widget_plot.addLine(y=0, pen=(200, 200, 200))

            # 东南西北标识
            text_N = pg.TextItem(html='N', border='w', anchor=(0.5, 1))
            self.widget_plot.addItem(text_N)
            text_N.setPos(0, rad_list[-1])
            text_E = pg.TextItem(html='E', border='w', anchor=(-0.5, 0.5))
            self.widget_plot.addItem(text_E)
            text_E.setPos(rad_list[-1], 0)
            text_S = pg.TextItem(html='S', border='w', anchor=(0.5, 0))
            self.widget_plot.addItem(text_S)
            text_S.setPos(0, rad_list[-1] * -1)
            text_W = pg.TextItem(html='W', border='w', anchor=(1, 0.5))
            self.widget_plot.addItem(text_W)
            text_W.setPos(rad_list[-1] * -1, 0)

            # 靶心圆圈
            for r in rad_list:
                circle = pg.QtWidgets.QGraphicsEllipseItem(-r, -r, r * 2, r * 2)
                circle.setPen(pg.mkPen(pen=(200, 200, 200)))
                self.widget_plot.addItem(circle)

                # 靶心距离标记
                text_circle = pg.TextItem(text=str(r), anchor=(0.5, 0.5), color=(180, 180, 180))
                self.widget_plot.addItem(text_circle)
                text_circle.setPos(r / math.sqrt(2), -r / math.sqrt(2))

        # 将区域添加到图形并将其与图形区域链接
        region = pg.LinearRegionItem()
        region.setZValue(10)

        # 十字线
        vLine = pg.InfiniteLine(angle=90, movable=False)
        hLine = pg.InfiniteLine(angle=0, movable=False)
        self.widget_plot.addItem(vLine, ignoreBounds=True)
        self.widget_plot.addItem(hLine, ignoreBounds=True)

        # 鼠标移动
        def mouseMoved(evt):
            pos = evt
            if self.widget_plot.sceneBoundingRect().contains(pos):
                mousePoint = vb.mapSceneToView(pos)
                self.label.setText("<span style='font-size: 12pt'>x=%0.3f,   <span style='color: red'>y1=%0.3f</span> "
                                   % (mousePoint.x(), mousePoint.y()))
                vLine.setPos(mousePoint.x())
                hLine.setPos(mousePoint.y())

        vb = self.widget_plot.vb
        self.widget_plot.scene().sigMouseMoved.connect(mouseMoved)

    # 设置统计绘图参数
    def set_plot_config(self, plot_config_dict):
        self.output_msg("【配置绘图参数】")
        try:
            if plot_config_dict["time"] == ['0', '0']:
                self.output_msg("绘图时间范围为：全程")
            else:
                self.output_msg(
                    "绘图时间范围为：" + str(plot_config_dict["time"]) + str(plot_config_dict['time_type']) + "时间")
            if self.ins_test_data:
                self.output_msg("INS绘图显示时间类型为：" + plot_config_dict["ins_plot_time_type"])
                self.output_msg("INS绘图抽稀倍数有：" + str(plot_config_dict["ins_plot_freq"]))
            if self.gnss_test_data:
                self.output_msg("GNSS绘图显示时间类型为：" + plot_config_dict["gnss_plot_time_type"])
                self.output_msg("GNSS绘图显示解状态有：" + str(list(plot_config_dict["gnss_plot_flags"].values())))
            self.plot_config = plot_config_dict
        except Exception as e:
            self.output_msg("【参数配置失败】异常原因：" + str(e))
            return
        self.output_msg("【参数配置完毕】\n")

    # gnss绘图（单图）
    def gnss_data_plot(self, plot_title):
        polt_list = ["解状态占比饼状图", "历元间隔分布图", "差分龄期统计图", "解状态水平误差序列图"
            , "位置误差(横向、纵向、水平)历元分布图", "速度误差(前向)历元分布图"
            , "姿态误差(航向)历元分布图", "统计误差CDF分布图"] if plot_title == 'ALL' else [plot_title]

        if len(self.gnss_test_data) == 0:
            self.output_msg("【绘图失败】 无GNSS数据可绘图")
            return
        plotObj = gnssPlot.gnssDataPlot()
        plotObj.plot_config = self.plot_config
        plotObj.gnss_test_data = self.gnss_test_data
        for plot_title in polt_list:
            self.output_msg("【开始绘图】" + plot_title)
            error_msg = plotObj.gnss_start_plot(plot_title)  # 画统计图流程
        if error_msg:
            self.output_msg("【绘图失败】" + error_msg)

    # 设置gnss综合误差图
    def gnss_multi_plot(self, plot_list):
        if len(self.gnss_test_data) == 0:
            self.output_msg("【绘图失败】 无GNSS数据可绘图")
            return
        self.output_msg("【开始绘图】GNSS综合误差绘图：" + str(plot_list))
        plotObj = gnssPlot.gnssDataPlot()
        plotObj.plot_config = self.plot_config
        plotObj.gnss_test_data = self.gnss_test_data
        error_msg = plotObj.gnss_multi_plot(plot_list)  # 画统计图流程
        if error_msg:
            self.output_msg("【绘图失败】" + error_msg)

    # 选择保存统计文件路径按钮按下
    def on_clicked_save_excel(self, algorithm_type):
        fileName, fileType = QtWidgets.QFileDialog.getSaveFileName(self, "保存文件", os.getcwd(),
                                                                   "Excel File (*.xlsx)")  # 打开文件对话框，选择文件
        if algorithm_type == 'GNSS':
            self.output_msg("【GNSS统计开始】")
            if not self.gnss_test_data:
                self.output_msg("【统计失败】无GNSS数据")
                return
            self.export_gnss_statistics(fileName)
        elif algorithm_type == 'INS':
            self.output_msg("【INS统计开始】")
            if not self.ins_test_data:
                self.output_msg("【统计失败】无INS数据")
                return
            self.export_ins_statistics(fileName)

    # 输出GNSS精度统计表
    def export_gnss_statistics(self, export_file_path):
        self.output_msg("【统计开始】")
        # 1. 场景分类
        statistics_all_scenes = []  # 汇总所有数据的场景信息
        for test_data in self.gnss_test_data:
            try:
                self.output_msg("开始场景分类:" + test_data["file_name"])
                # 每个场景的信息由{索引name、 数据data、 时间段time 和 占比percent} 组成
                scenes = dataPreProcess.sceneClassify(test_data, self.scene_list)
                statistics_all_scenes.extend(scenes)
            except Exception as e:
                self.output_msg("数据" + test_data["file_name"] + "场景分类失败,失败原因: 数据异常，" + str(e))

        # 2. 精度统计
        reportObj = gnssStatistics.GnssReport()
        gps_flag = {"all": "所有解", "fixed": "固定解", "float": "浮点解", "pseduo": "伪距解", "single": "单点解"}
        self.output_msg("开始生成统计表")
        for scene in statistics_all_scenes:
            try:
                items = scene["name"].split("_", 3)
                items[2] = gps_flag[items[2]]
                if "all" in scene["name"]:
                    self.output_msg("统计场景：" + scene["name"] + "， 统计历元数：" + str(len(scene["data"])))
                if scene["data"].empty:  # 跳过场景分类后无数据的情况
                    self.output_msg("【注意】无数据场景：" + scene["name"])
                statiObj = gnssStatistics.GnssStatistics()
                statiObj.percent = scene["percent"]
                statiObj.percent_ksxt = scene["percent_ksxt"]
                statiObj.scene_name = scene["name"]
                statiObj.SyncRefGpsData = scene["data"]
                statiObj.dataStatistics()  # 开始统计
                if self.ref_data['test_type'] == '动态':
                    reportObj.horizontal_error[scene["name"]] = statiObj.horizontal_error  # 水平误差
                    reportObj.longitudinal_error[scene["name"]] = statiObj.longitudinal_error  # 纵向误差
                    reportObj.lateral_error[scene["name"]] = statiObj.lateral_error  # 横向误差
                    reportObj.height_error[scene["name"]] = statiObj.height_error  # 高程误差
                    reportObj.velocity_error[scene["name"]] = statiObj.velocity_error  # 速度误差
                    reportObj.heading_error[scene["name"]] = statiObj.heading_error  # 航向误差
                    reportObj.hdop_sats[scene["name"]] = statiObj.hdop_sats  # Hdop和可用卫星数
                    reportObj.double_heading_error[scene["name"]] = statiObj.double_heading_error  # 双天线航向误差
                elif self.ref_data['test_type'] == '静态':
                    reportObj.horizontal_error[scene["name"]] = statiObj.horizontal_error  # 水平误差
                    reportObj.height_error[scene["name"]] = statiObj.height_error  # 高程误差
                    reportObj.hdop_sats[scene["name"]] = statiObj.hdop_sats  # Hdop和可用卫星数
            except Exception as e:
                self.output_msg("【注意】" + scene["name"] + "，精度统计失败，失败原因:" + str(e))

        # 3. 输出统计表
        try:
            reportObj.reportPath = export_file_path
            reportObj.saveStatisticToExcel()
            self.output_msg("【统计完成】统计表输出路径：" + reportObj.reportPath + "\n")
        except Exception as e:
            self.output_msg("【统计失败】失败原因:" + str(e))

    # ins绘图（单图）
    def ins_data_plot(self, plot_title):
        polt_list = ["位置偏差(水平、横向、纵向)历元分布图", "速度误差(北东地)历元分布图",
                     "速度误差(前右下)历元分布图", "轮速历元分布图", "速度轮速对比历元分布图",
                     "姿态误差(横滚、俯仰、航向)历元分布图", "坐标误差(经纬高)历元分布图",
                     "里程(前右下)历元分布图", "加表(XYZ)历元分布图", "陀螺(XYZ)历元分布图",
                     "历元间隔分布图", "加表陀螺历元间隔分布图", '相邻GPS-IMU时间间隔图',
                     "同步数据轨迹图", "Kalman.P历元分布图", "Kalman.X历元分布图",
                     "零偏历元分布图", "GPS解状态占比饼状图", "统计误差分布图", "GPS偏差对比图"] if plot_title == 'ALL' else [plot_title]

        if len(self.ins_test_data) == 0:
            self.output_msg("【绘图失败】 无INS数据可绘图")
            return
        plotObj = insPlot.insDataPlot()
        plotObj.plot_config = self.plot_config
        plotObj.ins_test_data = self.ins_test_data
        for plot_title in polt_list:
            self.output_msg("【开始绘图】" + plot_title)
            error_msg = plotObj.ins_start_plot(plot_title)  # 画统计图流程
        if error_msg:
            self.output_msg("【绘图失败】" + error_msg)

    # 设置ins综合误差图
    def ins_multi_plot(self, plot_list):
        if len(self.ins_test_data) == 0:
            self.output_msg("【绘图失败】 无INS数据可绘图")
            return
        self.output_msg("【开始绘图】INS综合误差绘图：" + str(plot_list))
        plotObj = insPlot.insDataPlot()
        plotObj.plot_config = self.plot_config
        plotObj.ins_test_data = self.ins_test_data
        self.output_msg(plotObj.ins_multi_plot(plot_list))  # 画统计图流程

    # 计算相关参数
    def ins_cal_error_diffs(self, all_scenes_data, data_type='sync_df', lbgc=[0, 0, 0]):
        """

        :param lbgc:  后轮轴中心补偿PData中的lbgc
        :param all_scenes_data: [{'name': '', 'data': {}, 'percent':1}]
        :param data_type:
        :return:
        """
        if len(all_scenes_data) == 0:
            self.output_msg('【导入失败】ins相关数据列表空，无法统计')
            return all_scenes_data
        else:
            statistics_info = [{'base_info': {}, 'test_info': {}, 'cal_info': {}, 'statistic_info': {}} for i in
                               range(len(all_scenes_data))]
            precision_val_names = ['horizontal_error', 'longitudinal_error', 'lateral_error', 'elevation_error'
                , 'north_vel_diff', 'east_vel_diff', 'ground_vel_diff', 'forward_vel_diff'
                , 'right_vel_diff', 'downward_vel_diff', 'velocity_diff'
                , 'roll_diff', 'pitch_diff', 'heading_diff']
        for scene in all_scenes_data:
            error_scene_index = []
            try:
                scene_index = all_scenes_data.index(scene)
                if data_type == 'sync_df':
                    self.output_msg("ref和INS同步数据相关信息：")
                    if len(scene['data']) == 0:
                        self.output_msg(scene['name'] + ' 无符合的数据，无法统计！')
                        all_scenes_data[scene_index].update(statistics_info[scene_index])
                        continue
                    else:
                        self.output_msg(scene['name'] + ' 的ref和INS同步数据统计中...')

                    ################################### 速度 ###################################
                    # Ref北东地速度转换成前右下坐标系下的速度
                    statistics_info[scene_index]['base_info'].update(insStatistic.change_val_name(scene['data']
                                                                                                  , ['NorthVelocity_x', 'EastVelocity_x', 'GroundVelocity_x']
                                                                                                  , ['north_vel', 'east_vel', 'ground_vel']))
                    statistics_info[scene_index]['base_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['base_info'], scene['data']['heading_x']))
                    # INS北东地速度转换成前右下坐标系下的速度
                    statistics_info[scene_index]['test_info'].update(insStatistic.change_val_name(scene['data']
                                                                                                  , ['NorthVelocity', 'EastVelocity', 'GroundVelocity']
                                                                                                  , ['north_vel', 'east_vel', 'ground_vel']))
                    statistics_info[scene_index]['test_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['test_info'], scene['data']['heading_x']))
                    # 前右下里程计算
                    statistics_info[scene_index]['test_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['test_info'], scene['data']['gpsItow']))
                    statistics_info[scene_index]['base_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['base_info'], scene['data']['gpsItow']))

                    statistics_info[scene_index]['base_info']['velocity'] = scene['data']['velocity_x']
                    statistics_info[scene_index]['test_info']['velocity'] = scene['data']['velocity']
                    ################################### 姿态 ###################################
                    pos0 = [scene['data']['latitude_x'][0], scene['data']['longitude_x'][0], scene['data']['ellHeight_x'][0]]
                    statistics_info[scene_index]['test_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x']))
                    statistics_info[scene_index]['base_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x'],
                                                val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))
                    statistics_info[scene_index]['cal_info'].update(
                        insStatistic.pos_error_cal(statistics_info[scene_index]['test_info'], statistics_info[scene_index]['base_info'], yaw=scene['data']['heading_x']))

                    ################################### 计算差值 ###################################
                    statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(
                        statistics_info[scene_index]['base_info'], statistics_info[scene_index]['test_info']
                        , df1_val_name=["north_vel", "east_vel", "ground_vel", 'forward_vel', 'right_vel', 'downward_vel', 'velocity']
                        , df2_val_name=["north_vel", "east_vel", "ground_vel", 'forward_vel', 'right_vel', 'downward_vel', 'velocity']))
                    statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data'],
                                                                                          df1_val_name=["roll_x", "pitch_x", "heading_x"],
                                                                                          df2_val_name=["roll", "pitch", "heading"]))

                    ################################### 记录相关值 ###################################
                    for i in ["forward_vel", "right_vel", "downward_vel", 'north_vel', 'east_vel', 'ground_vel', 'velocity',
                              "n_axis", "e_axis", "d_axis", "forward_mile", "right_mile", "downward_mile"]:
                        statistics_info[scene_index]['cal_info'][i + '_x'] = statistics_info[scene_index]['base_info'][i]
                        statistics_info[scene_index]['cal_info'][i] = statistics_info[scene_index]['test_info'][i]
                    ################################### 精度统计 ###################################
                    statistics_info[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                    for key in statistics_info[scene_index]['cal_info']:
                        if key in precision_val_names:
                            statistics_info[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info[scene_index]['cal_info'], key, percent=scene['percent'])

                elif data_type == 'sync_df_gps':
                    if len(scene['data']) == 0:
                        # self.output_msg(scene['name'] + ' 无符合的数据，无法统计！')
                        all_scenes_data[scene_index].update(statistics_info[scene_index])
                        continue
                    else:
                        # self.output_msg(scene['name'] + ' 的ref和GPS同步数据统计中...')
                        pass
                    ################################### 速度 ###################################
                    statistics_info[scene_index]['base_info'].update(insStatistic.change_val_name(scene['data']
                                                                                                  , ['NorthVelocity_x', 'EastVelocity_x', 'GroundVelocity_x']
                                                                                                  , ['north_vel', 'east_vel', 'ground_vel']))
                    statistics_info[scene_index]['base_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['base_info'], scene['data']['heading_x']))
                    statistics_info[scene_index]['test_info'].update(insStatistic.speed_hv2neg(scene['data'], scene['data']['TrackAngle']))
                    statistics_info[scene_index]['test_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['test_info'], scene['data']['heading_x']))

                    statistics_info[scene_index]['test_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['test_info'], scene['data']['gpsItow']))  # 前右下里程计算
                    statistics_info[scene_index]['base_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['base_info'], scene['data']['gpsItow']))  # 前右下里程计算
                    ################################### 姿态 ###################################
                    pos0 = [scene['data']['latitude_x'][0], scene['data']['longitude_x'][0], scene['data']['ellHeight_x'][0]]
                    statistics_info[scene_index]['test_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x']))
                    statistics_info[scene_index]['base_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x'], val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))
                    statistics_info[scene_index]['cal_info'].update(insStatistic.pos_error_cal(statistics_info[scene_index]['test_info'], statistics_info[scene_index]['base_info'], yaw=scene['data']['heading_x']))
                    ################################### 解状态转换 ###################################
                    statistics_info[scene_index]['cal_info'].update(insStatistic.gps_flags_transfer(scene['data']))

                    ################################### 计算差值 ###################################
                    statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(statistics_info[scene_index]['base_info'], statistics_info[scene_index]['test_info']))
                    statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data'], df1_val_name=["pitch_x", "heading_x", "velocity_x"], df2_val_name=["pitch", "heading", "velocity"]))
                    ################################### 记录相关值 ###################################
                    for i in ["forward_vel", "right_vel", "downward_vel", 'north_vel', 'east_vel', 'ground_vel', "n_axis", "e_axis", "d_axis", "forward_mile", "right_mile", "downward_mile"]:
                        statistics_info[scene_index]['cal_info'][i+'_x'] = statistics_info[scene_index]['base_info'][i]
                        statistics_info[scene_index]['cal_info'][i] = statistics_info[scene_index]['test_info'][i]

                    ################################### 精度统计 ###################################
                    statistics_info[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                    for key in statistics_info[scene_index]['cal_info'].keys():
                        if key in precision_val_names:
                            statistics_info[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info[scene_index]['cal_info'], key, percent=scene['percent'])
                elif data_type == 'sync_gps_ins':
                    if len(scene['data']) == 0:
                        self.output_msg(scene['name'] + ' 无符合的数据，无法统计！')
                        all_scenes_data[scene_index].update(all_scenes_data[scene_index])
                        continue
                    else:
                        self.output_msg(scene['name'] + ' 的GPS和INS同步数据统计中...')
                    ################################### 速度 ###################################
                    statistics_info[scene_index]['base_info'].update(insStatistic.speed_hv2neg(scene['data'], scene['data']['TrackAngle_x'], speed_name=['HSpd_x', 'velocity_x']))
                    statistics_info[scene_index]['base_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['base_info'], scene['data']['heading_x']))
                    statistics_info[scene_index]['test_info'].update(insStatistic.change_val_name(scene['data']
                                                                                                                                        , ['NorthVelocity', 'EastVelocity', 'GroundVelocity']
                                                                                                                                        , ['north_vel', 'east_vel', 'ground_vel']))

                    statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(statistics_info[scene_index]['base_info'], statistics_info[scene_index]['test_info']))  # GPS-INS 北、东、地速度差
                    statistics_info[scene_index]['test_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['test_info'], scene['data']['heading_x']))
                    statistics_info[scene_index]['test_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['test_info'], scene['data']['gpsItow']))  # 前右下里程计算
                    statistics_info[scene_index]['base_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['base_info'], scene['data']['gpsItow']))  # 前右下里程计算

                    ################################### 姿态 ###################################
                    pos0 = [scene['data']['latitude_x'][0], scene['data']['longitude_x'][0], scene['data']['ellHeight_x'][0]]
                    statistics_info[scene_index]['test_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading']))
                    statistics_info[scene_index]['base_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading'], bpos=lbgc, val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))

                    statistics_info[scene_index]['cal_info'].update(insStatistic.pos_error_cal(
                        statistics_info[scene_index]['test_info'], statistics_info[scene_index]['base_info'], yaw=scene['data']['heading']))
                    ################################### 解状态转换 ###################################
                    statistics_info[scene_index]['cal_info'].update(insStatistic.gps_flags_transfer(scene['data'], flags_pos_name='flagsPos_x'))

                    ################################### 计算差值 ###################################
                    statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data'], df1_val_name=["pitch_x", "heading_x", "velocity_x"], df2_val_name=["pitch", "heading", "velocity"]))
                    ################################### 记录对应数据, 4plot ###################################
                    for i in ["forward_vel", "right_vel", "downward_vel", 'north_vel', 'east_vel', 'ground_vel', "n_axis", "e_axis", "d_axis", "forward_mile", "right_mile", "downward_mile"]:
                        if i in statistics_info[scene_index]['base_info'].keys() and i in statistics_info[scene_index]['test_info'].keys():
                            statistics_info[scene_index]['cal_info'][i+'_x'] = statistics_info[scene_index]['base_info'][i]
                            statistics_info[scene_index]['cal_info'][i] = statistics_info[scene_index]['test_info'][i]

                    ################################### 精度统计 ###################################
                    statistics_info[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                    for key in statistics_info[scene_index]['cal_info']:
                        if key in precision_val_names:
                            statistics_info[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info[scene_index]['cal_info'], key, percent=scene['percent'])

                else:
                    self.output_msg('INS同步数据中无该类型：' + str(data_type))

                ################################### 存到对应场景信息中 ###################################
                all_scenes_data[scene_index].update(statistics_info[scene_index])
            except Exception as e:
                self.output_msg(all_scenes_data[scene_index]['name'], ' 场景无法解析统计:')
                self.output_msg(e)
                error_scene_index.append(scene_index)
                continue

        # delete error scene data
        for index in error_scene_index:
            all_scenes_data.pop(index)

        return all_scenes_data

    # 输出INS精度统计表 ——  中间层，用来控制不同统计函数得出结果
    def export_ins_statistics(self, export_file_path):

        for test_data_o in self.ins_test_data:
            statistics_all_scenes_ins = []  # 汇总所有数据的场景信息
            statistics_all_scenes_gps = []  # 汇总所有数据的场景信息
            statistics_all_scenes_insgps = []  # 汇总所有数据的场景信息
            # reportObj = insStatistic.insReport()
            # report_path = '.'.join(test_data["file_path"].split(".")[:-1])

            test_data = test_data_o.copy()
            try:
                if 'posture_bpox' in test_data.keys():
                    for df_name in ['sync_df', 'sync_gps_ins']:
                        if df_name in test_data.keys():
                            for val_name in ['roll', 'pitch', 'heading']:
                                val_index = ['roll', 'pitch', 'heading'].index(val_name)
                                if val_name in test_data[df_name].keys():
                                    test_data[df_name][val_name] -= test_data['posture_bpox'][val_index]
                                else:
                                    continue
                if 'posture_bpox_gps' in test_data.keys():
                    if 'sync_df_gps' in test_data.keys():
                        for val_name in ['roll', 'pitch', 'heading']:
                            val_index = ['roll', 'pitch', 'heading'].index(val_name)
                            if val_name in test_data['sync_df_gps'].keys():
                                test_data['sync_df_gps'][val_name] -= test_data['posture_bpox_gps'][val_index]
                            else:
                                continue
                    if 'sync_gps_ins' in test_data.keys():
                        for val_name in ['roll', 'pitch', 'heading']:
                            val_index = ['roll', 'pitch', 'heading'].index(val_name)
                            if val_name+'_x' in test_data['sync_gps_ins'].keys():
                                test_data['sync_gps_ins'][val_name+'_x'] -= test_data['posture_bpox_gps'][val_index]
                            else:
                                continue
            except Exception as e:
                self.output_msg('统计姿态杆臂补偿失败，失败数据为：', val_name, '\n原因为：', e)

            # 1. 场景分类 #######################################################
            self.output_msg("开始场景分类:" + test_data["file_name"])
            # 每个场景的信息由{索引name、 数据data、 时间段time 和 占比percent} 组成
            if 'sync_df' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list)
                    statistics_all_scenes_ins.extend(scenes)
                except Exception as e:
                    self.output_msg(
                        "数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            if 'sync_df_gps' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list, classify_data_name='sync_df_gps')
                    statistics_all_scenes_gps.extend(scenes)
                except Exception as e:
                    self.output_msg(
                        "数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            if 'sync_gps_ins' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list, classify_data_name='sync_gps_ins')
                    statistics_all_scenes_insgps.extend(scenes)
                except Exception as e:
                    self.output_msg(
                        "数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            # 2. 计算相关参数, 精度统计 #######################################################
            # 速度计算：位置、姿态、neg xyz速度
            if len(statistics_all_scenes_ins) > 0:
                statistics_all_scenes_ins = self.ins_cal_error_diffs(statistics_all_scenes_ins)
                ### 存储为feather
                # info_dict = statistics_all_scenes_ins[0]['cal_info'].copy()
                # for i in ['utcTime']:
                #     info_dict.pop(i)
                # self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path + '_ref同步ins.feather'))
                info_dict = None
            else:
                self.output_msg('无数据，不进行统计。')

            if len(statistics_all_scenes_gps) > 0:
                statistics_all_scenes_gps = self.ins_cal_error_diffs(statistics_all_scenes_gps, data_type='sync_df_gps')
                # info_dict = statistics_all_scenes_gps[0]['cal_info'].copy()
                # for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime']:
                #     info_dict.pop(i)
                # self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path + '_ref同步gps.feather'))
                info_dict = None
            else:
                self.output_msg('无数据，不进行统计。')

            if len(statistics_all_scenes_insgps) > 0:
                lbgc = [0, 0, 0]
                if test_data["output_pos"] == "后轮轴中心" and 'df_pdata' in test_data.keys():
                    if len(test_data['df_pdata']['Lbgc']) > 0:
                        lbgc = test_data['df_pdata']['Lbgc'][0]

                statistics_all_scenes_insgps = self.ins_cal_error_diffs(statistics_all_scenes_insgps,
                                                                        data_type='sync_gps_ins', lbgc=lbgc)
                # info_dict = statistics_all_scenes_gps[0]['cal_info'].copy()
                # for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime']:
                #     info_dict.pop(i)
                # self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path + '_gps同步ins.feather'))
                info_dict = None
            else:
                self.output_msg('无数据，不进行统计。')

            # 3. 输出统计表 #######################################################
            reportObj = insStatistic.insReport()
            reportObj.reportPath = export_file_path
            self.output_msg("【统计结果输出中】文件路径：" + reportObj.reportPath)
            if len(statistics_all_scenes_ins) != 0:
                self.output_msg(reportObj.statistic2excel(statistics_all_scenes_ins))
            # if len(statistics_all_scenes_gps) != 0:
            #     self.output_msg(
            #         reportObj.statistic2excel(statistics_all_scenes_gps, data_type='基准与GPS数据同步统计结果'))
            if len(statistics_all_scenes_insgps) != 0:
                self.output_msg(
                    reportObj.statistic2excel(statistics_all_scenes_insgps, data_type='GPS帧与INS帧同步统计结果'))

        self.output_msg("【统计完成】")


# QT子线程: 解析参考数据
class ParseRef(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）
    ref_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent: typing.Optional[QObject] = ...):
        super().__init__(parent)
        self.ref_data = None  # 参考数据

    def getInfo(self, signal_dict):
        self.ref_data = signal_dict

    def run(self):
        # 1.参考数据解析
        self.output_msg("【导入参考数据】")
        try:
            self.output_msg(
                "开始解析：" + self.ref_data["file_path"] + ", 数据类型：" + self.ref_data["file_type"] + ", 请等待...")
            self.output_msg(
                "已设置测试日期： " + self.ref_data["date_time"] + ", 测试类型： " + self.ref_data["test_type"])
            if dataParse.allDataParse(self.ref_data, self.ref_data["date_time"], freq=self.ref_data["file_freq"]) == -1:
                self.output_msg("【导入失败】参考数据无解析结果, 请检查数据类型" + "\n")
                return
            self.sendDataframe()
            self.output_msg("【数据导入完毕】\n")
        except Exception as e:
            self.output_msg("【导入失败】参考数据解析失败,失败原因: 数据异常，" + str(e) + "\n")
        # 释放内存
        self.ref_data = {}

    # 更新消息框内容
    def output_msg(self, msg_str):
        self.updated.emit(msg_str)  # 推送消息回主线程

    def sendDataframe(self):
        feather_id = uuid.uuid3(uuid.NAMESPACE_URL, self.ref_data["file_path"])
        feather_path = './ref_' + str(feather_id) + '.feather'
        self.ref_data["df"].to_feather(feather_path)  # 暂存为feather格式
        self.ref_signal.emit(feather_path)  # 输入feather路径


# QT子线程: 解析和同步GNSS数据
class CompareMainGnss(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）
    test_signal = QtCore.pyqtSignal(dict)

    def __init__(self, parent: typing.Optional[QObject] = ...):
        super().__init__(parent)
        self.ref_data = {}
        self.test_data_all = []  # 参考数据
        self.test_data = None  # 参考数据

    def getInfo(self, signal_dict):
        self.test_data_all.append(signal_dict)

    def run(self):
        self.output_msg("【导入GNSS测试数据】")
        for test_info in self.test_data_all:

            # 静态
            if test_info["test_type"] == "静态":
                self.ref_data["latitude"] = float(test_info["ref_data"][0])
                self.ref_data["longitude"] = float(test_info["ref_data"][1])
                self.ref_data["ellHeight"] = float(test_info["ref_data"][2])
            # 动态
            else:
                self.ref_data["df"] = pd.read_feather(test_info["ref_feather_path"])
                # os.remove(test_info["ref_feather_path"])  # 删除临时数据

            self.decodeGnssData(test_info)

        self.output_msg("【数据导入完毕】\n")

    def decodeGnssData(self, test_info):
        self.test_data = test_info.copy()
        # 1. 测试数据解析
        try:
            self.output_msg("开始解析：" + self.test_data["file_path"] + ", 数据类型：" + self.test_data[
                "file_type"] + ", 请等待......")
            self.output_msg("已设置测试日期： " + self.test_data["date_time"])
            if dataParse.allDataParse(self.test_data, self.test_data["date_time"]) == -1:
                self.output_msg("【导入失败】无解析结果, 无法绘图统计, 请检查数据格式！" + "\n")
                return
            if 'df' not in self.test_data.keys():
                self.output_msg("【导入失败】该文件解析有误" + "\n")
                return
            self.output_msg("数据解析完毕")
        except Exception as e:
            self.output_msg("【导入失败】测试数据解析失败,失败原因: 数据异常，" + str(e) + "\n")
            return

        # 2. 时间同步
        try:
            self.output_msg("开始时间同步, 同步参考数据为:" + str(self.test_data["ref_data"]))
            if self.test_data["test_type"] == "动态":
                sync_data = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df"])
            else:
                sync_data = dataPreProcess.nullRefDataFun(self.ref_data, self.test_data["df"])
            if len(sync_data[sync_data.keys()[0]]) == 0:
                self.output_msg("【导入失败】没有时间同步的结果,请检查参考数据对应的测试日期!")
                return
            self.test_data["sync_df"] = sync_data
            self.output_msg("时间同步完毕")
        except Exception as e:
            self.output_msg("【导入失败】时间同步失败,失败原因: 数据异常，" + str(e) + "\n")
            return

        # 3. 误差计算
        try:
            self.output_msg("开始误差统计")
            if self.test_data["test_type"] == "动态":
                bpox = self.test_data['bpox']  # 测试数据杆臂值
                self.test_data["sync_df"] = gnssErrorCal(sync_data, bpox)
            self.output_msg("误差统计完毕")
        except Exception as e:
            self.output_msg("【导入失败】同步数据统计误差计算失败，原因: 数据异常，" + str(e) + "\n")
            return

        # 传递解析数据
        self.sendDataframe(test_info=test_info)

    # 更新消息框内容
    def output_msg(self, msg_str):
        self.updated.emit(msg_str)  # 推送消息回主线程

    def sendDataframe(self, test_info=None):
        feather_id = uuid.uuid3(uuid.NAMESPACE_URL, self.test_data["file_path"])
        feather_path = './gnss_' + str(feather_id) + '.feather'
        sync_feather_path = './sync_gnss_' + str(feather_id) + '.feather'
        if 'df' in self.test_data.keys():
            self.test_data["df"].to_feather(feather_path)  # 暂存为feather格式
        else:
            feather_path = ''
        if 'sync_df' in self.test_data.keys():
            self.test_data["sync_df"].to_feather(sync_feather_path)
        else:
            sync_feather_path = ''

        # 输出feather路径
        if test_info:
            test_info['df_path'] = feather_path
            test_info["sync_df_path"] = sync_feather_path
            self.test_signal.emit(test_info)  # 输出feather路径

            # 释放内存
            self.test_data = {}
            self.ref_data = {}


# QT子线程: 解析和同步INS数据
class CompareMainIns(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）
    test_signal = QtCore.pyqtSignal(dict)
    mutex = QtCore.QMutex()

    def __init__(self, parent: typing.Optional[QObject] = ...):
        super().__init__(parent)
        self.ref_data = {}
        self.test_data_all = []  # 参考数据
        self.test_data = None  # 参考数据

    def getInfo(self, signal_dict):
        # 测试数据
        self.test_data_all.append(signal_dict)

    def run(self):
        self.output_msg("【导入INS测试数据】")
        for test_info in self.test_data_all:
            if 'ref_feather_path' in test_info:
                # 参考数据【已解析】
                self.ref_data["df"] = pd.read_feather(test_info["ref_feather_path"])
            else:
                pass

            if self.decodeInsData(test_info):
                if test_info['date_time'] != self.test_data['date_time']:
                    test_info['date_time'] = self.test_data['date_time']
                    print("change date time in test info:" + self.test_data['date_time'])

                self.sendDataframe(test_info=test_info)  # 传递解析数据
            else:
                continue

        # 释放内存
        self.ref_data = {}
        self.output_msg("【数据导入完毕】\n")

    def decodeInsData(self, test_info):
        self.test_data = test_info.copy()
        # 1. 测试数据解析
        if test_info['file_type'] == 'feather':
            for file_name in os.listdir('./' + self.test_data["file_name"]):
                if 'df_pdata' in file_name:
                    self.test_data["df_pdata_path"] = './' + self.test_data["file_name"] + '/' + file_name
                elif 'gps_' == file_name[:4]:
                    self.test_data["df_gps"] = pd.read_feather('./' + self.test_data["file_name"] + '/' + file_name)
                elif 'ins_' == file_name[:4]:
                    self.test_data["df"] = pd.read_feather('./' + self.test_data["file_name"] + '/' + file_name)
        else:
            try:
                self.output_msg("开始解析 ：" + self.test_data["file_path"] + "，数据类型：" + self.test_data["file_type"]
                                + "，输出位置：" + self.test_data["output_pos"] + "， 请等待......")
                self.output_msg("已设置测试日期： " + self.test_data["date_time"])
                insParseOutput = dataParse.allDataParse(self.test_data, self.test_data["date_time"])
                if insParseOutput == -1:
                    self.output_msg("【导入失败】ins帧无解析结果， 无法绘图统计， 请检查数据格式！" + "\n")
                    return False
                elif type(insParseOutput) == str:
                    self.output_msg(insParseOutput)
                    # 修改参考数据中输入错误的时间参数
                    if '【注意】gps帧时间为：' in insParseOutput and "df" in self.ref_data.keys():
                        try:
                            gps_date = insParseOutput.split('【注意】gps帧时间为：')[-1].split('与设定时间不符')[0]
                            self.output_msg('\n PS. 基准数据的测试日期设置与此测试数据的GPS帧内日期不一致！ 参考时间设置为：' + str(
                                    self.test_data['date_time']) + '，GPS帧内时间（年月日字段）为：' + gps_date + '。\n请确认时间后重新导入参考数据/测试数据。')
                            self.test_data['date_time'] = gps_date
                            if 'df' in self.ref_data.keys():
                                return False
                        except Exception as e:
                            self.output_msg('基准的Unix时间修改失败:\n')
                            self.output_msg(e)
                            return False
                self.output_msg("INS测试数据解析完毕。")
            except Exception as e:
                self.output_msg("【导入失败】测试数据解析失败，失败原因: 数据异常，" + str(e) + "\n")
                return False

        # 2. 时间同步
        if "df" in self.test_data.keys() and "df" in self.ref_data.keys():
            try:
                self.output_msg("ins数据开始时间同步， 同步参考数据为:" + str(self.test_data["ref_data"]))
                sync_data = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df"])
                if len(sync_data[sync_data.keys()[0]]) == 0:
                    self.output_msg("【导入失败】没有ins帧时间同步的结果，请检查参考数据对应的测试日期!" + "\n")
                    return False
                else:
                    self.test_data["sync_df"] = sync_data
                    self.output_msg("ins帧与参考数据时间同步完成 ")
            except Exception as e:
                self.output_msg("【导入失败】ins帧时间同步失败，失败原因: 数据异常，" + str(e) + "\n")
                return False
        else:
            self.output_msg('ref与ins同步中，无ref参考数据同步')

        if "df_gps" in self.test_data.keys() and "df" in self.ref_data.keys():
            try:
                self.output_msg("gps数据开始时间同步， 同步参考数据为:" + str(self.test_data["ref_data"]))
                sync_data_gps = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df_gps"])
                if len(sync_data_gps[sync_data_gps.keys()[0]]) == 0:
                    self.output_msg("【注意】没有gps帧时间同步的结果，请检查参考数据对应的测试日期!")
                else:
                    self.test_data["sync_df_gps"] = sync_data_gps
                    self.output_msg("gps帧与参考数据时间同步完成 ")
            except Exception as e:
                self.output_msg("gps帧时间同步失败，失败原因: 数据异常，" + str(e))
        else:
            self.output_msg('ref与gps同步中，无ref参考数据同步')

        if "df_gps" in self.test_data.keys() and "df" in self.test_data.keys():
            try:
                self.output_msg("测试数据 ins帧与自身gps帧 开始时间同步")
                sync_data_gpsins = dataPreProcess.timeSynchronize(self.test_data["df_gps"], self.test_data["df"])
                if len(sync_data_gpsins[sync_data_gpsins.keys()[0]]) == 0:
                    self.output_msg("【注意】没有gps帧与ins帧时间同步的结果,请检查参考数据对应的测试日期!")
                else:
                    self.test_data["sync_gps_ins"] = sync_data_gpsins
                    self.output_msg("ins与自身gps时间同步完成")
            except Exception as e:
                self.output_msg("gps帧时间同步失败，失败原因: 数据异常，" + str(e))

        return True

    # 更新消息框内容
    def output_msg(self, msg_str):
        self.updated.emit(msg_str)  # 推送消息回主线程

    # 输出解析结果
    def sendDataframe(self, test_info=None):
        if not self.test_data:
            print('self.test_data in sendDataframe is None')
            self.output_msg('解析错误！')
            return

        if not os.path.exists('./' + self.test_data["file_name"]):
            os.makedirs('./' + self.test_data["file_name"])
            print('./' + self.test_data["file_name"])

        feather_id = uuid.uuid3(uuid.NAMESPACE_URL, self.test_data["file_path"])
        feather_path = './' + self.test_data["file_name"] + '/ins_' + str(feather_id) + '.feather'
        sync_feather_path = './' + self.test_data["file_name"] + '/sync_ref-ins_' + str(feather_id) + '.feather'
        pdata_npy_path = None
        if 'df' in self.test_data.keys():
            self.test_data["df"].to_feather(feather_path)  # 暂存为feather格式
        else:
            feather_path = ''

        if "sync_df" in self.test_data.keys():
            self.test_data["sync_df"].to_feather(sync_feather_path)
        else:
            sync_feather_path = ""

        if "df_gps" in self.test_data.keys():
            gps_feather_path = './' + self.test_data["file_name"] + '/gps_' + str(feather_id) + '.feather'
            self.test_data["df_gps"].to_feather(gps_feather_path)  # 暂存为feather格式
        else:
            gps_feather_path = ""

        if "sync_df_gps" in self.test_data.keys():
            gps_sync_feather_path = './' + self.test_data["file_name"] + '/sync_ref-gps_' + str(feather_id) + '.feather'
            self.test_data["sync_df_gps"].to_feather(gps_sync_feather_path)
        else:
            gps_sync_feather_path = ""

        if "sync_gps_ins" in self.test_data.keys():
            gpsins_sync_feather_path = './' + self.test_data["file_name"] + '/sync_gps-ins_' + str(
                feather_id) + '.feather'
            self.test_data["sync_gps_ins"].to_feather(gpsins_sync_feather_path)
        else:
            gpsins_sync_feather_path = ""

        if "df_pdata" in self.test_data.keys():
            pdata_npy_path = './' + self.test_data["file_name"] + '/df_pdata_' + str(feather_id) + '.npy'
            np.save(pdata_npy_path, self.test_data["df_pdata"])
        elif "df_pdata_path" in self.test_data.keys():
            pdata_npy_path = self.test_data["df_pdata_path"]
        else:
            pdata_npy_path = ""

        if "df_imu" in self.test_data.keys():
            imu_feather_path = './' + self.test_data["file_name"] + '/df_imu_' + str(feather_id) + '.feather'
            self.test_data["df_imu"].to_feather(imu_feather_path)  # 暂存为feather格式
        elif "df_imu_path" in self.test_data.keys():
            imu_feather_path = self.test_data["df_imu_path"]
        else:
            imu_feather_path = ""

        if "df_vehicle" in self.test_data.keys():
            vehicle_feather_path = './' + self.test_data["file_name"] + '/df_vehicle_' + str(feather_id) + '.feather'
            self.test_data["df_vehicle"].to_feather(vehicle_feather_path)  # 暂存为feather格式
        elif "df_vehicle_path" in self.test_data.keys():
            vehicle_feather_path = self.test_data["df_vehicle_path"]
        else:
            vehicle_feather_path = ""

        # 输出feather路径
        if test_info:
            test_info['df_path'] = feather_path
            test_info["df_gps_path"] = gps_feather_path
            test_info["sync_df_path"] = sync_feather_path
            test_info["sync_df_gps_path"] = gps_sync_feather_path
            test_info["sync_gps_ins_path"] = gpsins_sync_feather_path
            test_info["df_pdata_path"] = pdata_npy_path
            test_info["df_imu_path"] = imu_feather_path
            test_info["df_vehicle_path"] = vehicle_feather_path
            self.test_signal.emit(test_info)

        self.test_data = None


# QT子线程: INS转bddb
class AnalysisBddb(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）
    mutex = QtCore.QMutex()

    def __init__(self, parent: typing.Optional[QObject] = ...):
        super().__init__(parent)
        self.data_analysis_flag = {'ins': True, 'gps': True, 'vehicle': True, 'imu': True,
                                   'ins2': False, 'imu2': False, 'sync': False,
                                   'sat': False, 'sat2': False, 'ZeroBias': False, 'EKFhandle_type': False,
                                   'InsPl': False, 'GnssPl': False
                                   }
        self.data_info = []  # 以后可能是[filepath1, filepath2...]

    def getInfo(self, signal_dict):
        # 测试数据
        self.data_analysis_flag = signal_dict['data_analysis_flag']
        self.data_info = signal_dict['data_info']
        if len(self.data_info) == 0:
            self.output_msg('没有数据')
            return

    def run(self):
        self.output_msg("【导入BDDB数据】")
        for test_info in self.data_info:
            self.decodeInsData(test_info)

        # 释放内存
        self.output_msg("【BDDB转mat完毕】\n")

    def decodeInsData(self, test_path):
        # 1. 测试数据解析
        try:
            self.output_msg("开始解析 ：" + test_path + "， 请等待......")

            obj = HexDataParse()
            obj.data_analysis_flag = self.data_analysis_flag
            obj.filePath = test_path
            obj.startParseFileHexData()
            obj.saveDataToDF()

            self.output_msg('帧数情况：')
            # check frequency
            obj.checkFreq()
            obj.outputDataFrameStatsResult()
            self.output_msg(obj.full_info)

            self.output_msg('MAT文件生成中...')
            obj.saveAllDataToMatFile()
            dir_path = os.path.split(test_path)
            self.output_msg("【MAT文件生成成功】" + dir_path[0] + '\\' + dir_path[1][:-4] + '_GPSINSData.mat')
        except Exception as e:
            self.output_msg("【MAT文件生成失败】测试数据解析失败，失败原因: 数据异常，" + str(e) + "\n")
            return

    # 更新消息框内容
    def output_msg(self, msg_str):
        self.updated.emit(msg_str)  # 推送消息回主线程


# QT子线程: 对已解析文件转换成 csv、nmea数据等
class CommonQtThread(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）
    mutex = QtCore.QMutex()

    def __init__(self, parent: typing.Optional[QObject] = ...):
        super().__init__(parent)
        self.signal_dict = {"function_type": None}

    def getInfo(self, signal_dict):
        self.signal_dict = signal_dict

    def run(self):
        if self.signal_dict["function_type"] == "df2Nmea":
            df, path = self.signal_dict["output_df"], self.signal_dict["output_path"]  # 先提取信号，避免串线程
            self.output_msg("【nmea导出开始】导出路径为：" + path + "，请等待...")
            df2Nmea.saveDfToNmea(df, path)
            self.output_msg("【nmea导出完毕】导出路径为：" + path)
        elif self.signal_dict["function_type"] == "df2Csv":
            # with open(self.signal_dict["output_path"], "w") as file:
            #     self.signal_dict["output_df"].to_csv(file)
            df, path, sync_df, sync_path = self.signal_dict["output_df"], self.signal_dict["output_path"], \
                                           self.signal_dict["output_sync_df"], self.signal_dict["output_sync_path"]
            self.output_msg("【csv导出开始】导出路径为：" + path + "，请等待...")
            df.to_csv(path)
            self.output_msg("【csv导出完毕】导出路径为：" + path)
            if sync_path is not None:
                self.output_msg("【csv导出开始】同步数据导出路径为：" + path + "，请等待...")
                sync_df.to_csv(sync_path)
                self.output_msg("【csv导出完毕】同步数据导出路径为：" + path)

    # 更新消息框内容
    def output_msg(self, msg_str):
        self.updated.emit(msg_str)  # 推送消息回主线程
