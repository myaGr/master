from PyQt5 import QtWidgets
from PyQt5.QtCore import QCoreApplication

from ui_MainWindow_DataAnalysis import Ui_MainWindow  # UI界面
from HexDataParse import HexDataParse  # 导入自定义的类，数据解析
from Parse100CData import Parse100CData  # 100C数据解析
from DataPreProcess import DataPreProcess
from PlotGpsInsSyncData import PlotGpsInsRawSyncData  # 画图统计
from threading import Thread  # 多线程
import os.path
import time
import numpy as np
import ui_another_window_actions

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.inscont = 1
        self.path = None
        self.types = []
        self.inspath = None
        self.refpath = None
        self.inspaths = []
        self.name_list = []
        self.GpsNum = None
        self.StarTime = None
        self.EndTime = None
        self.setupUi(self)
        self.InsBpos = {}
        self.GpsBpos = {}
        self.InsDataDF = {}
        self.GpsDataDF = {}
        self.ins100cdf = {}
        self.InsDataDFInter = None
        self.SyncInsGpsData = None
        self.SyncRefInsData = None
        self.SyncRefGpsData = None
        # 新增测试数据
        self.widget_add = {}
        self.horizontalLayout_add = {}
        self.label_add = {}
        self.lineEdit_GetInsFile_add = {}
        self.label_add = {}
        self.lineEdit_GetInsFile_add = {}
        self.pushBtn_SelectIns_add = {}
        self.toolButton_add = {}
        # 实例化数据解析对象
        self.HexDataParseObj = HexDataParse()
        self.Parse100CDataObj = Parse100CData()
        self.DataPreProcess = DataPreProcess()
        self.PlotGpsInsRawSyncDataObj = PlotGpsInsRawSyncData()
        # 信号关联
        self.pushBtn_SelectFile.clicked.connect(self.onClickedSelectFile)  # 选择文件按钮按下
        self.pushBtn_StartParse1.clicked.connect(self.onClickedStartParseIns)  # 开始解析按钮按下
        self.pushBtn_StartPlot1.clicked.connect(self.onClickedStartPlotInsGps)  # 开始画图按钮按下
        self.pushBtn_ClearMsg1.clicked.connect(self.onClickedClearMsg1)  # 清空消息按钮按下

        self.pushBtn_SelectIns.clicked.connect(lambda: self.onClickedSelectINSFile('1'))  # 选择INS文件按钮按下
        self.pushBtn_SelectRef.clicked.connect(self.onClickedSelectRefFile)  # 选择参考文件按钮按下
        self.pushBtn_StartParse2.clicked.connect(self.onClickedStartParseInsRef)  # 开始解析按钮按下
        self.pushBtn_StartPlot2.clicked.connect(self.onClickedStartPlotInsRef)  # 开始画图按钮按下
        self.pushBtn_ClearMsg2.clicked.connect(self.onClickedClearMsg2)  # 清空消息按钮按下\
        self.pushBtn_SelectIns_2.clicked.connect(self.add)  # 添加数据按钮按下

        # # 绑定槽函数
        self.toolButton.clicked.connect(lambda: self.get_config('1')) #参数配置按钮按下


    #'''添加一份测试数据的组件'''
    def add(self):

        self.inscont += 1
        n = str(self.inscont)
        self.widget_add[n] = QtWidgets.QWidget(self.widget_10)
        self.widget_add[n].setObjectName("widget_6")

        self.horizontalLayout_add[n] = QtWidgets.QHBoxLayout(self.widget_add[n])
        self.horizontalLayout_add[n].setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout_add[n].setObjectName("horizontalLayout_6")

        self.label_add[n] = QtWidgets.QLabel(self.widget_add[n])
        self.label_add[n].setObjectName("label")
        self.label_add[n].setText("测试文件" + n + "  ：")
        self.horizontalLayout_add[n].addWidget(self.label_add[n])

        self.lineEdit_GetInsFile_add[n] = QtWidgets.QLineEdit(self.widget_add[n])
        self.lineEdit_GetInsFile_add[n].setObjectName("lineEdit_GetInsFile")
        self.horizontalLayout_add[n].addWidget(self.lineEdit_GetInsFile_add[n])

        self.pushBtn_SelectIns_add[n] = QtWidgets.QPushButton(self.widget_add[n])
        self.pushBtn_SelectIns_add[n].setObjectName("pushBtn_SelectIns")
        self.pushBtn_SelectIns_add[n].setText("选择文件")
        self.horizontalLayout_add[n].addWidget(self.pushBtn_SelectIns_add[n])
        self.pushBtn_SelectIns_add[n].clicked.connect(lambda: self.onClickedSelectINSFile(n))  # 选择INS文件按钮按下

        self.toolButton_add[n] = QtWidgets.QToolButton(self.widget_add[n])
        self.toolButton_add[n].setObjectName("toolButton")
        self.toolButton_add[n].setText("...")
        self.horizontalLayout_add[n].addWidget(self.toolButton_add[n])
        self.verticalLayout.addWidget(self.widget_add[n])

        self.toolButton_add[n].clicked.connect(lambda: self.get_config(n))

    # 打开配置参数窗口
    def get_config(self, n):
        """点击相应按钮，跳转到第二个界面"""
        # 实例化第二个界面的后端类，并对第二个界面进行显示
        # 通过派生新类去访问类

        self.config_window = ui_another_window_actions.AnotherWindowActions()
        self.config_window.show()
        self.config_window.saveDataButton.clicked.connect(lambda: self.save_config_data(n))
        self.config_window.saveDataButton.clicked.connect(self.config_window.close)

    # 读取杆臂值
    def save_config_data(self, n):
        self.InsBpos[n] = [float(self.config_window.lineEdit_InsBposX.text()),
                           float(self.config_window.lineEdit_InsBposY.text()),
                           float(self.config_window.lineEdit_InsBposZ.text())]
        self.GpsBpos[n] = [float(self.config_window.lineEdit_GpsBposX.text()),
                           float(self.config_window.lineEdit_GpsBposY.text()),
                           float(self.config_window.lineEdit_GpsBposZ.text())]
        # self.config_window.lineEdit_InsBposX.setText(str(self.InsBpos[n][0]))

    # 关闭画图按键功能
    def setPlotBtnFlase(self):
        self.pushBtn_StartPlot1.setEnabled(False)
        self.pushBtn_StartPlot2.setEnabled(False)

    # INSGPS 对比，选择文件按钮按下
    def onClickedSelectFile(self):
        self.setPlotBtnFlase()
        fileslist = ''
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(), "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        fileslist = fileslist + fileName
        self.lineEdit_GetFile.setText(fileslist)

    # 选择INS文件按钮按下
    def onClickedSelectINSFile(self, n):
        self.setPlotBtnFlase()
        fileslist = ''
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(), "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        fileslist = fileslist + fileName
        if int(n) == 1:
            self.lineEdit_GetInsFile.setText(fileslist)
        elif int(n) > 1:
            self.lineEdit_GetInsFile_add[n].setText(fileslist)
        self.InsBpos[n] = [0, 0, 0]
        self.GpsBpos[n] = [0, 0, 0]

    # 选择参考文件按钮按下
    def onClickedSelectRefFile(self):
        self.setPlotBtnFlase()
        fileslist = ''
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(), "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        fileslist = fileslist + fileName
        self.lineEdit_GetRefFile.setText(fileslist)

    # 清空消息按钮按下
    def onClickedClearMsg1(self):
        self.textBrowser_showmsg1.clear()  # 清空textBrowser_showMsg里的消息

    def onClickedClearMsg2(self):
        self.textBrowser_showmsg2.clear()  # 清空textBrowser_showMsg里的消息

    # 开始解析按钮按下
    def onClickedStartParseIns(self):
        self.setPlotBtnFlase()
        self.path = self.lineEdit_GetFile.text()  # 获取文件路径
        if not os.path.isfile(self.path):
            self.outputMsg1(self.path + "文件不存在...")
            return
        self.checkOutputDataType()  # 获取生成文件类型
        self.createDataParseThread()  # 创建数据解析线程，开始数据解析

    # INS和100C对比，开始解析按钮按下
    def onClickedStartParseInsRef(self):
        self.setPlotBtnFlase()
        self.inspaths = []
        self.inspath = self.lineEdit_GetInsFile.text()  # 获取INS文件路径
        self.refpath = self.lineEdit_GetRefFile.text()  # 获取参考文件路径
        self.inspaths.append(self.inspath)
        for n in self.lineEdit_GetInsFile_add.keys():   # 获取增加INS文件路径
            file_add = self.lineEdit_GetInsFile_add[n].text()
            self.inspaths.append(file_add)
        for file in self.inspaths:
            if not os.path.isfile(file):
                self.outputMsg2(file + "文件不存在...")
                return
        if not os.path.isfile(self.refpath):
            self.outputMsg2(self.refpath + "文件不存在...")
            return
        self.createDataAnanlyseThread()  # 创建数据解析线程，开始数据解析

    # 线程回调函数，数据解析
    def dataParseThreadFunc(self):
        try:
            self.HexDataParseObj.filePath = self.path
            self.outputMsg1("开始解析：" + self.path)
            self.HexDataParseObj.startParseFileHexData()  # 开始数据解析
            self.HexDataParseObj.saveDataToDF()
            self.outputMsg1("解析完成。")
            self.outputDataFrameStatsResult()  # 输出数据帧统计结果

            if "csv" in self.types:
                self.outputMsg1("生成Csv文件...")
                self.HexDataParseObj.SaveAllDataToCsvFile()
            if "mat" in self.types:
                self.outputMsg1("生成Mat文件...")
                self.HexDataParseObj.startSaveAllDataToMatFile()

            # 时间插值
            self.outputMsg2("开始参考数据时间插值...")
            InsDataDFfilter = self.DataPreProcess.Datafilter(self.HexDataParseObj.InsDataDF, [0, 0])  # 过滤无效数据
            self.InsDataDFInter = self.DataPreProcess.Datainterpolation(InsDataDFfilter)  # 时间插值
            # 时间同步
            self.outputMsg1("开始INS和GPS时间同步...")
            self.SyncInsGpsData = self.DataPreProcess.Timesynchronize(self.InsDataDFInter, self.HexDataParseObj.GpsDataDF, 'time', 'itow_pos')
            self.outputMsg1("时间同步完成，可生成统计图。")
            self.pushBtn_StartPlot1.setEnabled(True)
        except Exception as e:
            self.outputMsg1(self.path + "文件解析失败...")
            self.outputMsg1('失败原因:' + str(e))

    def dataAnalysisThreadFunc(self):
        try:
            # 100C 数据解析
            self.outputMsg2("开始解析：" + self.refpath)
            self.Parse100CDataObj.filepath = self.refpath
            ref_flag = self.Parse100CDataObj.save100Ctodf()  # 开始参考数据解析
            if ref_flag:
                self.outputMsg2("缺少字段：" + str(ref_flag) + ", 解析失败。")
                return
            if len(self.Parse100CDataObj.ins100cdf) == 0:
                self.outputMsg2("数据无效，解析失败。")
                return

            # INS 数据解析
            self.name_list = []
            for file in self.inspaths:
                self.HexDataParseObj.filePath = file
                file_name = file.split('/')[-1].split('.')[0]
                self.name_list.append(file_name)
                try:
                    # 数据解析
                    self.outputMsg2("开始解析：" + file)
                    self.HexDataParseObj.startParseFileHexData()  # 开始INS数据解析
                    self.HexDataParseObj.saveDataToDF()
                    if self.HexDataParseObj.InsDataDF is None:
                        self.outputMsg2("数据无效，解析失败。")
                        return
                    self.outputMsg2("数据解析完成。")
                    # 数据保存
                    self.InsDataDF[file_name] = self.HexDataParseObj.InsDataDF
                    self.HexDataParseObj.InsDataDF = None
                    self.GpsDataDF[file_name] = self.HexDataParseObj.GpsDataDF
                    self.HexDataParseObj.GpsDataDF = None
                except Exception as e:
                    self.inspaths.pop(file_name)
                    self.name_list.pop(file_name)

                    self.outputMsg2(file_name + "解析失败...")
                    self.outputMsg2('失败原因:' + str(e))
                    continue

            # 获取开始和结束时间
            self.GetStartEndTime()

            # 获取GPS显示数量
            self.checkGpsNum()
            if self.GpsNum > len(self.name_list):
                self.outputMsg2("参数无效： 显示GPS数量超过测试文件数量，请重新配置")
                return
            else:
                for i in range(self.GpsNum):
                    self.PlotGpsInsRawSyncDataObj.gps_flag[self.name_list[i]] = 1
                print("gps_flag:", self.PlotGpsInsRawSyncDataObj.gps_flag)

            # 时间插值
            self.outputMsg2("开始参考数据时间插值...")
            self.ins100cdf = self.DataPreProcess.Datainterpolation(self.Parse100CDataObj.ins100cdf)  # 时间插值
            # 时间同步
            self.PlotGpsInsRawSyncDataObj.SyncRefInsData = {}
            for file_name in self.name_list:
                try:
                    self.outputMsg2(file_name + ": INS和参考数据时间同步...")
                    self.PlotGpsInsRawSyncDataObj.SyncRefInsData[file_name] = self.DataPreProcess.Timesynchronize(self.ins100cdf, self.InsDataDF[file_name], 'time', 'time')
                    if self.PlotGpsInsRawSyncDataObj.gps_flag[file_name]:
                        self.outputMsg2(file_name + ": GPS和参考数据时间同步...")
                        self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name] = self.DataPreProcess.Timesynchronize(self.ins100cdf, self.GpsDataDF[file_name], 'time', 'itow_pos')
                except Exception as e:
                    self.outputMsg2(file_name + ": 时间同步失败...")
                    self.outputMsg2('失败原因:' + str(e))
                    continue
            self.outputMsg2("时间同步完成，可生成统计图。")
            self.pushBtn_StartPlot2.setEnabled(True)

        except Exception as e:
            self.outputMsg2("统计失败...")
            self.outputMsg2('失败原因:' + str(e))

    # 开始画图按钮按下
    def onClickedStartPlotInsGps(self):
        self.outputMsg1("开始INS和GPS统计画图...")
        self.PlotGpsInsRawSyncDataObj.InsDataDF = self.HexDataParseObj.InsDataDF
        self.PlotGpsInsRawSyncDataObj.GpsDataDF = self.HexDataParseObj.GpsDataDF
        self.PlotGpsInsRawSyncDataObj.PDataDict = self.HexDataParseObj.PDataDict
        self.PlotGpsInsRawSyncDataObj.SyncDataDF = self.HexDataParseObj.SyncDataDF
        self.PlotGpsInsRawSyncDataObj.VehicleDataDF = self.HexDataParseObj.VehicleDataDF
        self.PlotGpsInsRawSyncDataObj.ImuDataDF = self.HexDataParseObj.ImuDataDF
        self.PlotGpsInsRawSyncDataObj.SyncInsGpsData = self.SyncInsGpsData
        self.checkOutputTimeType1()  # 获取显示时间类型
        try:
            print("开始INS和GPS统计画图...")
            self.PlotGpsInsRawSyncDataObj.PlotGpsInsRawSyncData()
        except Exception as e:
            self.outputMsg2("画图失败...")
            self.outputMsg2('失败原因:' + str(e))

    def onClickedStartPlotInsRef(self):
        # 配置杆臂值
        for n in self.GpsBpos.keys():
            self.PlotGpsInsRawSyncDataObj.bpos_refins[self.name_list[int(n) - 1]] = np.array([[0, 0, 0], self.InsBpos[n]])
            self.PlotGpsInsRawSyncDataObj.bpos_refgps[self.name_list[int(n) - 1]] = np.array([[0, 0, 0], self.GpsBpos[n]])
            # print("bpos_refins:", self.PlotGpsInsRawSyncDataObj.bpos_refins)
            # print("bpos_refgps:", self.PlotGpsInsRawSyncDataObj.bpos_refgps)
            # self.outputMsg2('file_name: ' + str(self.name_list[int(n) - 1]))
            # self.outputMsg2("INS杆臂值: " + self.PlotGpsInsRawSyncDataObj.bpos_refins[self.name_list[int(n) - 1]])
            # self.outputMsg2("GPS杆臂值: " + self.PlotGpsInsRawSyncDataObj.bpos_refins[self.name_list[int(n) - 1]])

        # 获取显示时间类型
        self.checkOutputTimeType2()
        print("second_of_week:", self.PlotGpsInsRawSyncDataObj.second_of_week)

        self.outputMsg2("开始INS和参考对比统计画图...")
        try:
            self.PlotGpsInsRawSyncDataObj.PlotRefGpsInsSyncData(os.getcwd())
            self.outputMsg2("统计结果已生成：" + os.getcwd() + '\statistic.xlsx')
        except Exception as e:
            self.outputMsg2("画图失败...")
            self.outputMsg2('失败原因:' + str(e))
            self.outputMsg2('若想绘制其余文件统计，请再次点击按钮【开始画图】！')


    # 获取生成文件类型
    def checkOutputDataType(self):
        if self.checkBox_csv.isChecked():
            self.types.append("csv")
        if self.checkBox_mat.isChecked():
            self.types.append("mat")

    # 获取显示时间类型
    def checkOutputTimeType1(self):
        if self.radioButton_time.isChecked():
            self.PlotGpsInsRawSyncDataObj.second_of_week = False
        elif self.radioButton_tow.isChecked():
            self.PlotGpsInsRawSyncDataObj.second_of_week = True

    def checkOutputTimeType2(self):
        if self.radioButton_time2.isChecked():
            self.PlotGpsInsRawSyncDataObj.second_of_week = False
        elif self.radioButton_tow2.isChecked():
            self.PlotGpsInsRawSyncDataObj.second_of_week = True

    # 获取GPS显示数量
    def checkGpsNum(self):
        self.PlotGpsInsRawSyncDataObj.gps_flag = dict.fromkeys(self.name_list, 0)
        self.GpsNum = self.showGPS_spinBox.value()


    # 获取开始和结束时间
    def GetStartEndTime(self):
        self.DataPreProcess.t = [0, 0]
        if self.lineEdit_StarTime.text():
            self.DataPreProcess.t[0] = float(self.lineEdit_StarTime.text())
        if self.lineEdit_EndTime.text():
            self.DataPreProcess.t[1] = float(self.lineEdit_EndTime.text())

    # 创建数据解析线程
    def createDataParseThread(self):
        t = Thread(target=self.dataParseThreadFunc)  # 创建一个线程
        t.setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
        t.start()  # 启动线程

    def createDataAnanlyseThread(self):
        t = Thread(target=self.dataAnalysisThreadFunc)  # 创建一个线程
        t.setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
        t.start()  # 启动线程

    # 输出提示消息
    def outputMsg1(self, msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        self.textBrowser_showmsg1.append(msg)

    def outputMsg2(self, msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        self.textBrowser_showmsg2.append(msg)

    # 输出数据帧统计结果
    def outputDataFrameStatsResult(self):
        msg = "IMU数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['imuFrameHeadNum_bdbd0a']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['imuDataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['imuCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['imuCheckErrIndex'])
        self.outputMsg1(msg)

        msg = "GPS数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['gpsFrameHeadNum_bdbd10']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['gpsDataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['gpsCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['gpsCheckErrIndex'])
        self.outputMsg1(msg)

        msg = "车辆数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['vehicleFrameHeadNum_bdbd20']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['vehicleDataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['vehicleCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['vehicleCheckErrIndex'])
        self.outputMsg1(msg)

        msg = "INS数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['insFrameHeadNum_bdbd0b']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['insDataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['insCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['insCheckErrIndex'])
        self.outputMsg1(msg)

        msg = "同步时间数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['syncFrameHeadNum_bdbd0c']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['syncDataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['syncCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['syncCheckErrIndex'])
        self.outputMsg1(msg)

        msg = "IMU2数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['imu2FrameHeadNum_bdbd2a']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['imu2DataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['imu2CheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['imu2CheckErrIndex'])
        self.outputMsg1(msg)

        msg = "INS2数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['ins2FrameHeadNum_bdbd1b']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['ins2DataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['ins2CheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['ins2CheckErrIndex'])
        self.outputMsg1(msg)

        msg = "四轮转角数据帧：纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['FourWSFrameHeadNum_bdbd30']) + "，总帧数量:" + str(
            self.HexDataParseObj.dataFrameStats['FourWSDataNum']) + "，错误帧数量:" + str(
            len(self.HexDataParseObj.dataFrameStats['FourWSCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.HexDataParseObj.dataFrameStats['FourWSCheckErrIndex'])
        self.outputMsg1(msg)
