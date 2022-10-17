from PyQt5 import QtWidgets
from ui_MainWindow_DataAnalysis import Ui_MainWindow  # UI界面
from HexDataParse import HexDataParse  # 导入自定义的类，数据解析
from Parse100CData import Parse100CData  # 100C数据解析
from PlotGpsInsSyncData import PlotGpsInsRawSyncData  # 画图统计
from threading import Thread  # 多线程
import os.path
import time


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.path = None
        self.types = []
        self.inspath = None
        self.refpath = None
        self.setupUi(self)
        self.SyncInsGpsData = None
        self.SyncRefInsData = None
        self.SyncRefGpsData = None
        # 实例化数据解析对象
        self.HexDataParseObj = HexDataParse()
        self.Parse100CDataObj = Parse100CData()
        self.PlotGpsInsRawSyncDataObj = PlotGpsInsRawSyncData()
        # 信号关联
        self.pushBtn_SelectFile.clicked.connect(self.onClickedSelectFile)  # 选择文件按钮按下
        self.pushBtn_StartParse1.clicked.connect(self.onClickedStartParseIns)  # 开始解析按钮按下
        self.pushBtn_StartPlot1.clicked.connect(self.onClickedStartPlotInsGps)  # 开始画图按钮按下
        self.pushBtn_ClearMsg1.clicked.connect(self.onClickedClearMsg1)  # 清空消息按钮按下

        self.pushBtn_SelectIns.clicked.connect(self.onClickedSelectINSFile)  # 选择INS文件按钮按下
        self.pushBtn_SelectRef.clicked.connect(self.onClickedSelectRefFile)  # 选择参考文件按钮按下
        self.pushBtn_StartParse2.clicked.connect(self.onClickedStartParseInsRef)  # 开始解析按钮按下
        self.pushBtn_StartPlot2.clicked.connect(self.onClickedStartPlotInsRef)  # 开始画图按钮按下
        self.pushBtn_ClearMsg2.clicked.connect(self.onClickedClearMsg2)  # 清空消息按钮按下


    # 关闭画图按键功能
    def setPlotBtnFlase(self):
        self.pushBtn_StartPlot1.setEnabled(False)
        self.pushBtn_StartPlot2.setEnabled(False)

    # 选择文件按钮按下
    def onClickedSelectFile(self):
        self.setPlotBtnFlase()
        fileslist = ''
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        fileslist = fileslist + fileName
        self.lineEdit_GetFile.setText(fileslist)

    def onClickedSelectINSFile(self):
        self.setPlotBtnFlase()
        fileslist = ''
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        fileslist = fileslist + fileName
        self.lineEdit_GetInsFile.setText(fileslist)

    def onClickedSelectRefFile(self):
        self.setPlotBtnFlase()
        fileslist = ''
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
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

    def onClickedStartParseInsRef(self):
        self.setPlotBtnFlase()
        self.inspath = self.lineEdit_GetInsFile.text()  # 获取文件路径
        self.refpath = self.lineEdit_GetRefFile.text()  # 获取文件路径
        if not os.path.isfile(self.inspath):
            self.outputMsg2(self.inspath + "文件不存在...")
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
            self.outputMsg1("开始INS和GPS时间同步...")
            self.SyncInsGpsData = self.HexDataParseObj.Timesynchronize(self.HexDataParseObj.InsDataDF,
                                                                       self.HexDataParseObj.GpsDataDF, 'time', 'itow_pos')
            self.outputMsg1("时间同步完成，可生成统计图。")
            self.pushBtn_StartPlot1.setEnabled(True)

        except ValueError as e:
            self.outputMsg1(self.path + "文件解析失败...")
            self.outputMsg1('ValueError:' + str(e))

    def dataAnalysisThreadFunc(self):
        try:
            self.HexDataParseObj.filePath = self.inspath
            self.Parse100CDataObj.filepath = self.refpath
            self.outputMsg2("开始解析：" + self.inspath)
            self.HexDataParseObj.startParseFileHexData()  # 开始数据解析
            self.HexDataParseObj.saveDataToDF()
            self.outputMsg2("开始解析：" + self.refpath)
            self.Parse100CDataObj.save100Ctodf()
            self.outputMsg2("解析完成。")
            self.outputMsg2("INS和参考数据时间同步...")
            self.PlotGpsInsRawSyncDataObj.SyncRefInsData = self.HexDataParseObj.Timesynchronize(
                self.Parse100CDataObj.ins100cdf, self.HexDataParseObj.InsDataDF, 'time', 'time')
            self.outputMsg2("GPS和参考数据时间同步...")
            self.PlotGpsInsRawSyncDataObj.SyncRefGpsData = self.HexDataParseObj.Timesynchronize(
                self.Parse100CDataObj.ins100cdf, self.HexDataParseObj.GpsDataDF, 'time', 'itow_pos')
            self.outputMsg2("时间同步完成，可生成统计图。")
            self.pushBtn_StartPlot2.setEnabled(True)
        except ValueError as e:
            self.outputMsg2("文件统计失败...")
            self.outputMsg2('ValueError:' + str(e))

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
        self.PlotGpsInsRawSyncDataObj.PlotGpsInsRawSyncData()

    def onClickedStartPlotInsRef(self):
        self.outputMsg2("开始INS和参考对比统计画图...")
        self.PlotGpsInsRawSyncDataObj.PlotRefGpsInsSyncData(self.inspath)
        self.outputMsg2("统计结果已生成：" + self.inspath[:-4] + '_statistic.xlsx')

    # 获取生成文件类型
    def checkOutputDataType(self):
        if self.checkBox_csv.isChecked():
            self.types.append("csv")
        if self.checkBox_mat.isChecked():
            self.types.append("mat")

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
