from PyQt5 import QtWidgets
from ui_MainWindow import Ui_MainWindow  # UI界面
from threading import Thread  # 多线程
from HexDataParse import HexDataParse  # 导入自定义的类，数据解析
import os.path
import time


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)

        # 实例化数据解析对象
        self.hexDataParseObj = HexDataParse()

        # 信号关联
        self.pushBtn_startDataParse.clicked.connect(self.onClickedStartDataParse)  # 开始解析按钮按下
        self.pushBtn_clearMsg.clicked.connect(self.onClickedClearMsg)  # 清空消息按钮按下
        self.pushBtn_selectFile.clicked.connect(self.onClickedSelectFile)  # 选择文件按钮按下

    # 选择文件按钮按下
    def onClickedSelectFile(self):
        fileslist = ''
        fileNames, fileType = QtWidgets.QFileDialog.getOpenFileNames(self, "选择文件", os.getcwd(), "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件

        for fileName in fileNames:
            fileslist = fileslist + fileName + ';'
        self.lineEdit_filePath.setText(fileslist)



    # 清空消息按钮按下
    def onClickedClearMsg(self):
        self.textBrowser_showMsg.clear()  # 清空textBrowser_showMsg里的消息

    # 开始解析按钮按下响应函数
    def onClickedStartDataParse(self):

        fullpaths = self.lineEdit_filePath.text()  # 获取文件路径
        self.pathlist = fullpaths.split(';')[0:-1]

        for path in self.pathlist:
            if not os.path.isfile(path):
                self.outputMsg(path + "文件不存在...")
                return
        self.createDataParseThread()  # 创建数据解析线程，开始数据解析


    # 线程回调函数，数据解析
    def dataParseThreadFunc(self):
        for path in self.pathlist:
            self.hexDataParseObj.filePath = path
            self.outputMsg("开始解析：" + self.hexDataParseObj.filePath)
            self.hexDataParseObj.startParseFileHexData()  # 开始数据解析
            self.outputMsg("解析完成...")
            self.outputMsg("开始存成.mat文件...")
            self.hexDataParseObj.startSaveAllDataToMatFile()  # 开始将数据存成.mat文件
            self.outputMsg("存成.mat文件完成...")
            # self.outputDataFrameStatsResult()  # 输出数据帧统计结果
        self.outputMsg("所有数据解析结束")



        # self.outputMsg("开始打包数据...")
        # try:
        #     self.hexDataParseObj.PackAllData()  #打包数据
        # except Exception as e:
        #     print("数据打包失败", e)
        # self.outputMsg("数据打包结束...")



    # 创建数据解析线程
    def createDataParseThread(self):
        t = Thread(target=self.dataParseThreadFunc)  # 创建一个线程
        t.setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
        t.start()  # 启动线程

    # 输出提示消息
    def outputMsg(self, msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        self.textBrowser_showMsg.append(msg)

    # 输出数据帧统计结果
    def outputDataFrameStatsResult(self):
        msg = "IMU数据帧：纯帧头数量:" + str(self.hexDataParseObj.dataFrameStats['imuFrameHeadNum_bdbd0a']) + "，总帧数量:" + str(
            self.hexDataParseObj.dataFrameStats['imuDataNum']) + "，错误帧数量:" + str(
            len(self.hexDataParseObj.dataFrameStats['imuCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.hexDataParseObj.dataFrameStats['imuCheckErrIndex'])
        self.outputMsg(msg)

        msg = "GPS数据帧：纯帧头数量:" + str(self.hexDataParseObj.dataFrameStats['gpsFrameHeadNum_bdbd10']) + "，总帧数量:" + str(
            self.hexDataParseObj.dataFrameStats['gpsDataNum']) + "，错误帧数量:" + str(
            len(self.hexDataParseObj.dataFrameStats['gpsCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.hexDataParseObj.dataFrameStats['gpsCheckErrIndex'])
        self.outputMsg(msg)

        msg = "车辆数据帧：纯帧头数量:" + str(self.hexDataParseObj.dataFrameStats['vehicleFrameHeadNum_bdbd20']) + "，总帧数量:" + str(
            self.hexDataParseObj.dataFrameStats['vehicleDataNum']) + "，错误帧数量:" + str(
            len(self.hexDataParseObj.dataFrameStats['vehicleCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.hexDataParseObj.dataFrameStats['vehicleCheckErrIndex'])
        self.outputMsg(msg)

        msg = "INS数据帧：纯帧头数量:" + str(self.hexDataParseObj.dataFrameStats['insFrameHeadNum_bdbd0b']) + "，总帧数量:" + str(
            self.hexDataParseObj.dataFrameStats['insDataNum']) + "，错误帧数量:" + str(
            len(self.hexDataParseObj.dataFrameStats['insCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.hexDataParseObj.dataFrameStats['insCheckErrIndex'])
        self.outputMsg(msg)

        msg = "同步时间数据帧：纯帧头数量:" + str(self.hexDataParseObj.dataFrameStats['syncFrameHeadNum_bdbd0c']) + "，总帧数量:" + str(
            self.hexDataParseObj.dataFrameStats['syncDataNum']) + "，错误帧数量:" + str(
            len(self.hexDataParseObj.dataFrameStats['syncCheckErrIndex'])) + "，错误帧索引下标:" + str(
            self.hexDataParseObj.dataFrameStats['syncCheckErrIndex'])
        self.outputMsg(msg)
