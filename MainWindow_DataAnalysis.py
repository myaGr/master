from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QDate, Qt, QObject
import webbrowser
from PyQt5.QtCore import QCoreApplication

from view.ui_MainWindow_DataAnalysis import Ui_MainWindow  # UI界面
from func.HexDataParse import HexDataParse  # 导入自定义的类，数据解析
from func.Parse100CData import Parse100CData  # 100C数据解析
from func.DataPreProcess import DataPreProcess
from func.PlotGpsInsSyncData import PlotGpsInsRawSyncData  # 画图统计
from threading import Thread  # 多线程
import os.path
import time
import typing
import numpy as np
from view import ui_another_window_actions  # 配置杆臂值输入 + 场景输入
from func.canLoader import canLoader


# QT子线程
class mainFunctionAnalysicSingleFile(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）

    # 参数初始化
    def __init__(self, parent: typing.Optional[QObject] = ...):
        super().__init__(parent)
        self.HexDataParseObj = None
        self.Parse100CDataObj = None
        self.DataPreProcess = None
        self.PlotGpsInsRawSyncDataObj = None

        self.file_path = None
        self.types = []

        self.SyncInsGpsData = None

        self.InsDataDFInter = None
        self.SyncRefInsData = None
        self.SyncRefGpsData = None

    def get_info(self, data_info):
        self.file_path = data_info[0]
        self.types = data_info[1]

    def outputMsg1(self, msg_str):
        self.updated.emit(msg_str)

    def run(self):
        self.HexDataParseObj = HexDataParse()
        self.PlotGpsInsRawSyncDataObj = PlotGpsInsRawSyncData()
        self.DataPreProcess = DataPreProcess()
        try:
            self.HexDataParseObj.filePath = self.file_path
            self.outputMsg1("开始解析：" + self.file_path)
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

            # 时间同步
            self.outputMsg1("开始INS和GPS时间同步...")
            self.SyncInsGpsData = self.DataPreProcess.timeSynchronize(self.HexDataParseObj.InsDataDF,
                                                                      self.HexDataParseObj.GpsDataDF, 'time',
                                                                      'itow_pos')
            self.outputMsg1("【绘图】时间同步完成，可生成统计图。")
        except Exception as e:
            self.outputMsg1(self.file_path + "文件解析失败...")
            self.outputMsg1('失败原因:' + str(e))

    def plot_insgps(self, second_of_week=False):
        self.outputMsg1("开始INS和GPS统计画图...")
        self.PlotGpsInsRawSyncDataObj.InsDataDF = self.HexDataParseObj.InsDataDF
        self.PlotGpsInsRawSyncDataObj.GpsDataDF = self.HexDataParseObj.GpsDataDF
        self.PlotGpsInsRawSyncDataObj.PDataDict = self.HexDataParseObj.PDataDict
        self.PlotGpsInsRawSyncDataObj.SyncDataDF = self.HexDataParseObj.SyncDataDF
        self.PlotGpsInsRawSyncDataObj.VehicleDataDF = self.HexDataParseObj.VehicleDataDF
        self.PlotGpsInsRawSyncDataObj.ImuDataDF = self.HexDataParseObj.ImuDataDF
        self.PlotGpsInsRawSyncDataObj.SyncInsGpsData = self.SyncInsGpsData
        # 繪圖橫坐標為周内秒: 默認False
        self.PlotGpsInsRawSyncDataObj.second_of_week = second_of_week

        try:
            print("轨迹生图..")
            self.PlotGpsInsRawSyncDataObj.plot_raw_path()
        except Exception as e:
            self.outputMsg1("轨迹生图失败...")
            self.outputMsg1('失败原因:' + str(e))

        try:
            print("开始INS和GPS统计画图...")
            self.PlotGpsInsRawSyncDataObj.PlotGpsInsRawSyncData()
            self.outputMsg1(self.PlotGpsInsRawSyncDataObj.msg_info)
            self.PlotGpsInsRawSyncDataObj.msg_info = ''
        except Exception as e:
            self.outputMsg1("画图失败...")
            self.outputMsg1('失败原因:' + str(e))

    def outputDataFrameStatsResult(self):
        try:
            msg = "IMU数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['imuFrameHeadNum_bdbd0a']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['imuDataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['imuCheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['imuCheckErrIndex'])))
            self.outputMsg1(msg)

            msg = "GPS数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['gpsFrameHeadNum_bdbd10']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['gpsDataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['gpsCheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['gpsCheckErrIndex'])))
            self.outputMsg1(msg)

            msg = "车辆数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['vehicleFrameHeadNum_bdbd20']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['vehicleDataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['vehicleCheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['vehicleCheckErrIndex'])))
            self.outputMsg1(msg)

            msg = "INS数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['insFrameHeadNum_bdbd0b']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['insDataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['insCheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['insCheckErrIndex'])))
            self.outputMsg1(msg)

            msg = "同步时间数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['syncFrameHeadNum_bdbd0c']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['syncDataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['syncCheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['syncCheckErrIndex'])))
            self.outputMsg1(msg)

            msg = "IMU2数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['imu2FrameHeadNum_bdbd2a']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['imu2DataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['imu2CheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['imu2CheckErrIndex'])))
            self.outputMsg1(msg)

            msg = "INS2数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['ins2FrameHeadNum_bdbd1b']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['ins2DataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['ins2CheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['ins2CheckErrIndex'])))
            self.outputMsg1(msg)

            msg = "四轮转角数据帧：纯帧头数量:" + str(
                self.HexDataParseObj.dataFrameStats['FourWSFrameHeadNum_bdbd30']) + "，总帧数量:" + str(
                self.HexDataParseObj.dataFrameStats['FourWSDataNum']) + "，错误帧数量:" + str(
                len(self.HexDataParseObj.dataFrameStats['FourWSCheckErrIndex'])) + "，错误帧索引下标:" + str(
                list(set(self.HexDataParseObj.dataFrameStats['FourWSCheckErrIndex'])))
            self.outputMsg1(msg)

        except Exception as e:
            self.outputMsg1('显示有误')


class mainFunctionInsCompare(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）

    # 参数初始化
    def __init__(self, parent: typing.Optional[QObject] = ...):
        super().__init__(parent)
        self.HexDataParseObj = None
        self.Parse100CDataObj = None
        self.DataPreProcess = None
        self.PlotGpsInsRawSyncDataObj = None

        self.refpath = None
        self.inspaths = None
        self.name_list = []
        self.time_dir = {}
        self.plot_scene_time = []
        self.plot_scene = '全程'
        self.GpsNum = None
        self.InsBpos = None
        self.GpsBpos = None
        self.ref_type = '100C'

        self.InsDataDF = {}
        self.GpsDataDF = {}

    def get_info(self, data_info):
        self.refpath = data_info[0]
        self.ref_type = data_info[1]
        self.inspaths = data_info[2]
        self.time_dir = data_info[3]
        self.plot_scene_time = data_info[4]  # 画图场景，将赋值给 self.DataPreProcess.t
        self.plot_scene = data_info[5]
        self.GpsNum = data_info[6]
        self.InsBpos = data_info[7]
        self.GpsBpos = data_info[8]

    def outputMsg2(self, msg_str):
        self.updated.emit(msg_str)

    def run(self):
        self.HexDataParseObj = HexDataParse()
        self.PlotGpsInsRawSyncDataObj = PlotGpsInsRawSyncData()
        self.DataPreProcess = DataPreProcess()
        self.Parse100CDataObj = Parse100CData()

        ##################################### 1、解析基准数据 #####################################
        try:
            # ps. self.Parse100CDataObj.ins100cdf 会被重置
            self.outputMsg2("开始解析：" + self.refpath)
            self.Parse100CDataObj.filepath = self.refpath
            ref_file_name = self.refpath.split('/')[-1].split('.')[0]

            ##################################### POS320
            if self.ref_type == '320':
                self.outputMsg2(ref_file_name + "为 POS320 设备输出格式。")
                try:
                    ref_flag = self.Parse100CDataObj.save_320_to_df()  # 开始参考数据解析
                except Exception as e:
                    print(e)
                    self.outputMsg2("尝试解析另一种 POS320 设备输出格式。")
                    ref_flag = self.Parse100CDataObj.save_320_to_df_1()  # 开始参考数据解析
            ##################################### 100C
            elif self.ref_type == '100C':
                self.outputMsg2(ref_file_name + "为 100C 设备输出格式。")
                ref_flag = self.Parse100CDataObj.save100Ctodf()  # 开始参考数据解析
            ##################################### NMEA
            elif self.ref_type == 'NMEA':
                self.outputMsg2(ref_file_name + "为 NMEA 设备输出格式。")
                self.outputMsg2(ref_file_name + "为 NMEA 设备输出格式。\n 该格式待重构版本开发。")

                # ref_flag = self.Parse100CDataObj.save_nmea_to_df()  # 开始参考数据解析
            else:
                self.outputMsg2("尚未兼容基准文件格式：" + self.ref_type)
                # self.clear_handinput_scenes()
                return
            # 判断是否存在缺少的字段
            print(ref_flag)
            if type(ref_flag) == str:
                self.outputMsg2(ref_flag)
            elif ref_flag:
                self.outputMsg2("缺少字段：" + str(ref_flag) + ", 解析失败。")
                return
            if len(self.Parse100CDataObj.ins100cdf) == 0:
                self.outputMsg2("数据无效，解析失败。")
                return
        except Exception as e:
            self.outputMsg2("基准数据解析失败...")
            self.outputMsg2('失败原因:' + str(e))

        # 解析测试数据
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
                if self.HexDataParseObj.InsDataDF is not None:
                    self.outputMsg2("数据解析完成。")
                    # 数据保存
                    self.InsDataDF[file_name] = self.HexDataParseObj.InsDataDF
                    self.HexDataParseObj.InsDataDF = None
                    self.GpsDataDF[file_name] = self.HexDataParseObj.GpsDataDF
                    self.HexDataParseObj.GpsDataDF = None
                else:
                    self.outputMsg2("数据无效，解析失败。")
                    continue
            except Exception as e:
                self.inspaths.pop(file_name)
                self.name_list.pop(file_name)

                self.outputMsg2(file_name + "解析失败...")
                self.outputMsg2('失败原因:' + str(e))
                continue

        ##################################### 2、时间同步 #####################################
        try:
            self.PlotGpsInsRawSyncDataObj.SyncRefInsData = {}
            self.PlotGpsInsRawSyncDataObj.SyncRefGpsData = {}
            for file_name in self.name_list:
                try:
                    self.outputMsg2(file_name + ": INS和参考数据时间同步...")
                    self.PlotGpsInsRawSyncDataObj.SyncRefInsData[file_name] = self.DataPreProcess.timeSynchronize(
                        self.Parse100CDataObj.ins100cdf, self.InsDataDF[file_name], 'time', 'time')
                    # if self.PlotGpsInsRawSyncDataObj.gps_flag[file_name]:
                    self.outputMsg2(file_name + ": GPS和参考数据时间同步...")
                    self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name] = self.DataPreProcess.timeSynchronize(
                        self.Parse100CDataObj.ins100cdf, self.GpsDataDF[file_name], 'time', 'itow_pos')
                    if len(self.PlotGpsInsRawSyncDataObj.SyncRefInsData[file_name]) == 0 or len(
                            self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name]) == 0:
                        self.outputMsg2('时间同步失败：数据都被过滤了，请检查数据中的IMUstatus和flagsPos值！')
                except Exception as e:
                    self.outputMsg2(file_name + ": 时间同步失败...")
                    self.outputMsg2('失败原因:' + str(e))
                    continue
        except Exception as e:
            self.outputMsg2("时间同步失败...")
            self.outputMsg2('失败原因:' + str(e))

        # 3、分时段统计， time_dir仅被使用于此
        if len(list(self.time_dir.keys())) >= 1:
            for scene in self.time_dir.keys():
                try:
                    time_arrange = self.time_dir[scene]['time_arrange']
                    scene_dscribe = str(self.time_dir[scene]['scene_num']) + '_' + self.time_dir[scene]['scene']
                    # 统计部分场景中，先把所有GPS的值都置上
                    self.PlotGpsInsRawSyncDataObj.gps_flag = dict.fromkeys(self.name_list, 1)
                    self.PlotGpsInsRawSyncDataObj.iniInsGpsBpos()

                    # if self.time_dir[scene]['scene'] != '全程':
                    self.outputMsg2('统计时间范围为%s的数据, 为场景：%s' % (str(time_arrange), scene_dscribe))
                    static_msg_info = self.PlotGpsInsRawSyncDataObj.dataPreStatistics(time_arrange=time_arrange)
                    self.PlotGpsInsRawSyncDataObj.gen_statistics_xlsx(os.getcwd(),
                                                                      time_arrange=time_arrange,
                                                                      scene=scene_dscribe)
                    self.outputMsg2(static_msg_info)
                    static_msg_info = ''
                except Exception as e:
                    self.outputMsg2(scene + ": 场景统计失败...")
                    self.outputMsg2('失败原因:' + str(e))
                    continue

            self.outputMsg2("场景统计完成！")

        # 4、绘图参数配置
        try:
            # 获取绘图的开始和结束时间
            self.DataPreProcess.t = self.plot_scene_time
            print('time_range:', self.DataPreProcess.t)
            self.DataPreProcess.scene = self.plot_scene
            self.outputMsg2('绘制时间段设为：%s' % str(self.DataPreProcess.t))

            # 【僅爲畫圖】如果仅有的画图时间段（plot_scene_time）不是全程（[0,0]），需要时间二次同步
            if self.DataPreProcess.t[0] != 0 or self.DataPreProcess.t[1] != 0:
                self.outputMsg2('【绘图】同步时间段为%s的数据：' % str(self.DataPreProcess.t))
                self.PlotGpsInsRawSyncDataObj.SyncRefInsData = {}
                self.PlotGpsInsRawSyncDataObj.SyncRefGpsData = {}
                self.PlotGpsInsRawSyncDataObj.SyncRefInsData_all = {}
                self.PlotGpsInsRawSyncDataObj.SyncRefGpsData_all = {}
                for file_name in self.name_list:
                    try:
                        self.outputMsg2(file_name + "【绘图】: INS和参考数据时间同步，同步的时间段为: " + str(self.plot_scene_time))
                        self.PlotGpsInsRawSyncDataObj.SyncRefInsData[file_name] = self.DataPreProcess.timeSynchronize(
                            self.Parse100CDataObj.ins100cdf, self.InsDataDF[file_name], 'time', 'time')
                        # if self.PlotGpsInsRawSyncDataObj.gps_flag[file_name]:
                        self.outputMsg2(file_name + "【绘图】: GPS和参考数据时间同步，同步的时间段为: " + str(self.plot_scene_time))
                        self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name] = self.DataPreProcess.timeSynchronize(
                            self.Parse100CDataObj.ins100cdf, self.GpsDataDF[file_name], 'time', 'itow_pos')
                    except Exception as e:
                        self.outputMsg2(file_name + "【绘图】: 时间同步失败，即将绘制全程图，同步失败的时间段为: " + str(self.plot_scene_time))
                        self.outputMsg2('失败原因:' + str(e))
                        continue

            # 获取绘图中 GPS 显示数量
            self.PlotGpsInsRawSyncDataObj.gps_flag = dict.fromkeys(self.name_list, 0)
            if self.GpsNum > len(self.name_list):
                self.outputMsg2("参数无效： 显示GPS数量超过测试文件数量，请重新配置")
                return
            else:
                for i in range(self.GpsNum):
                    self.PlotGpsInsRawSyncDataObj.gps_flag[self.name_list[i]] = 1
                print("gps_flag:", self.PlotGpsInsRawSyncDataObj.gps_flag)

            self.outputMsg2("【绘图】时间同步完成，可生成图。")
            # self.pushBtn_StartPlot2.setEnabled(True)

        except Exception as e:
            self.outputMsg2("绘图参数配置失败...")
            self.outputMsg2('失败原因:' + str(e))

    def set_gps_bpos(self):
        # 配置杆臂值
        for n in self.GpsBpos.keys():
            self.PlotGpsInsRawSyncDataObj.bpos_refins[self.name_list[int(n) - 1]] = np.array(
                [[0, 0, 0], self.InsBpos[n]])
            self.PlotGpsInsRawSyncDataObj.bpos_refgps[self.name_list[int(n) - 1]] = np.array(
                [[0, 0, 0], self.GpsBpos[n]])

    def plot_insgps_compare(self, second_of_week=False):
        self.outputMsg2("开始INS和GPS统计画图...")
        self.outputMsg2('将绘制场景%s时间段为%s的统计图' % (self.DataPreProcess.scene, str(self.DataPreProcess.t)))
        self.set_gps_bpos()

        self.outputMsg2("开始INS和参考对比统计画图...")

        self.PlotGpsInsRawSyncDataObj.InsDataDF = self.InsDataDF
        self.PlotGpsInsRawSyncDataObj.GpsDataDF = self.GpsDataDF
        self.PlotGpsInsRawSyncDataObj.rtkDataDF = self.Parse100CDataObj.ins100cdf
        try:
            print("轨迹生图..")
            self.PlotGpsInsRawSyncDataObj.plot_raw_path(type='ref')
        except Exception as e:
            self.outputMsg1("轨迹生图失败...")
            self.outputMsg1('失败原因:' + str(e))

        try:
            self.PlotGpsInsRawSyncDataObj.second_of_week = second_of_week
            msg_info = self.PlotGpsInsRawSyncDataObj.PlotRefGpsInsSyncData(os.getcwd()
                                                                           , scene=self.DataPreProcess.scene
                                                                           , time_arrange=str(self.DataPreProcess.t))
            self.outputMsg2(msg_info)
            self.outputMsg2("统计结果已生成：" + os.getcwd() + '\statistic.xlsx')
        except Exception as e:
            self.outputMsg2("画图失败...")
            self.outputMsg2('失败原因:' + str(e))
            self.outputMsg2('若想绘制其余文件统计，请再次点击按钮【开始画图】！')


class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    push_info_1 = QtCore.pyqtSignal(list)  # 信号类变量（数组）
    push_info_2 = QtCore.pyqtSignal(list)  # 信号类变量（数组）

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.dbc_paths = None
        self.dbcs_path_list = None
        self.blfs_path_list = None
        self.dbc_path = None
        self.blf_path = None

        self.path = None
        self.types = []

        self.inspath = None
        self.refpath = None
        self.inspaths = []
        self.inscont = 1
        self.scene_cont = 0
        self.GpsNum = None
        self.setupUi(self)
        self.InsBpos = {}
        self.GpsBpos = {}
        self.ref_type = '100C'
        self.time_dir = {}
        self.time_dir_config = {}
        self.config_window, self.scene_config_window = None, None
        # 新增测试数据
        self.scene_widget_add = {}
        self.horizontalLayout_scene_add = {}
        self.label_scene_add = {}
        self.label_start_add = {}
        self.label_end_add = {}
        self.lineEdit_scene_add = {}
        self.lineEdit_start_add = {}
        self.lineEdit_end_add = {}

        self.widget_add = {}
        self.horizontalLayout_add = {}
        self.label_add = {}
        self.lineEdit_GetInsFile_add = {}
        self.pushBtn_SelectIns_add = {}
        self.toolButton_add = {}

        # 信号关联
        ############ 功能一：自身对比部分
        self.pushBtn_SelectFile.clicked.connect(self.onClickedSelectFile)  # 选择文件按钮按下
        self.pushBtn_StartParse1.clicked.connect(self.onClickedStartParseIns)  # 开始解析按钮按下
        self.pushBtn_StartPlot1.clicked.connect(self.onClickedStartPlotInsGps)  # 开始画图按钮按下
        # QT子线程信号关联
        self.thread_1 = mainFunctionAnalysicSingleFile(self)  # QT子线程实例化
        self.push_info_1.connect(self.thread_1.get_info)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
        self.thread_1.updated.connect(self.outputMsg1)  # B类（QT子线程）发送信号， 调用A类(主线程)的槽函数
        self.pushBtn_ClearMsg1.clicked.connect(self.onClickedClearMsg1)  # 清空消息按钮按下

        ############ 功能二：与参考数据对比部分
        self.pushBtn_SelectIns.clicked.connect(lambda: self.onClickedSelectINSFile('1'))  # 选择INS文件按钮按下
        self.pushBtn_SelectRef.clicked.connect(self.onClickedSelectRefFile)  # 选择参考文件按钮按下
        self.pushBtn_StartParse2.clicked.connect(self.onClickedStartParseInsRef)  # 开始解析按钮按下
        self.pushBtn_StartPlot2.clicked.connect(self.onClickedStartPlotInsRef)  # 开始画图按钮按下
        self.pushBtn_ClearMsg2.clicked.connect(self.onClickedClearMsg2)  # 清空消息按钮按下\
        self.pushBtn_SelectIns_2.clicked.connect(self.add)  # 添加文件按钮按下
        self.clear_test_file.clicked.connect(self.clear_test_file_record)  # 清空文件按钮按下

        self.pushBtn_addScene.clicked.connect(self.add_scene)  # 添加场景按钮按下
        self.importScene.clicked.connect(self.onClickedSelectFile_scene)  # 导入场景按钮按下

        # QT子线程信号关联
        self.thread_2 = mainFunctionInsCompare(self)  # QT子线程实例化
        self.push_info_2.connect(self.thread_2.get_info)  # A类(主线程)发送信号， 调用B类（QT子线程）的槽函数
        self.thread_2.updated.connect(self.outputMsg2)  # B类（QT子线程）发送信号， 调用A类(主线程)的槽函数

        # # 绑定槽函数
        self.toolButton.clicked.connect(lambda: self.get_config('1'))  # 参数配置按钮按下

        ############ 功能三：CAN解析
        self.dbc_files_dict = {}
        self.blf_files_dict = {}
        self.pushBtn_getDBCtext.clicked.connect(self.on_clicked_select_dbc_file)
        self.pushBtn_GetFiletext.clicked.connect(self.on_clicked_select_blf_file)
        self.pushBtn_decode.clicked.connect(self.on_clicked_start_analysis_blf)

        # HELP
        self.botton_help.clicked.connect(self.open_help_web)

    ##################################### INS和与自身对比 #####################################
    # 开始解析按钮按下
    def onClickedStartParseIns(self):
        self.setPlotBtnFlase()

        self.path = self.lineEdit_GetFile.text()  # 获取文件路径
        if not os.path.isfile(self.path):
            self.outputMsg1(self.path + "文件不存在...")
            return
        self.checkOutputDataType()  # 获取生成文件类型

        self.push_info_1.emit([self.path, self.types])
        self.thread_1.start()
        # self.pushBtn_StartPlot1.setEnabled(True)
        # self.createDataParseThread()  # 创建数据解析线程，开始数据解析

    # 开始画图按钮按下
    def onClickedStartPlotInsGps(self):
        # global second_of_week
        # if self.radioButton_time.isChecked():
        #     second_of_week = False
        # elif self.radioButton_tow.isChecked():
        #     second_of_week = True

        second_of_week = True if self.radioButton_tow.isChecked() else False
        self.thread_1.plot_insgps(second_of_week=second_of_week)

    # 获取生成文件类型
    def checkOutputDataType(self):
        if self.checkBox_csv.isChecked():
            self.types.append("csv")
        if self.checkBox_mat.isChecked():
            self.types.append("mat")

    # 输出提示消息
    def outputMsg1(self, msg_str):
        if '开始解析：' in msg_str:
            self.pushBtn_StartPlot1.setEnabled(False)
        if '【绘图】时间同步完成，可生成统计图' in msg_str:
            self.pushBtn_StartPlot1.setEnabled(True)
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        self.textBrowser_showmsg1.append(msg)

    # 清空消息按钮按下
    def onClickedClearMsg1(self):
        self.textBrowser_showmsg1.clear()  # 清空textBrowser_showMsg里的消息
        self.thread_1.PlotGpsInsRawSyncDataObj.close_plot()

    ##################################### INS和100C对比，开始解析按钮按下 #####################################

    '''add scene'''

    def add_scene(self):
        self.scene_cont += 1
        n = str(self.scene_cont)
        self.scene_widget_add[n] = QtWidgets.QWidget(self.widget_16)
        self.scene_widget_add[n].setObjectName("widget_11")

        self.horizontalLayout_scene_add[n] = QtWidgets.QHBoxLayout(self.scene_widget_add[n])
        self.horizontalLayout_scene_add[n].setContentsMargins(0, 6, 0, 6)
        self.horizontalLayout_scene_add[n].setObjectName("horizontalLayout_11")
        print(self.horizontalLayout_scene_add[n])

        self.label_scene_add[n] = QtWidgets.QLabel(self.scene_widget_add[n])
        self.label_scene_add[n].setObjectName("label")
        self.label_scene_add[n].setText("场景 ：")
        self.horizontalLayout_scene_add[n].addWidget(self.label_scene_add[n])

        self.lineEdit_scene_add[n] = QtWidgets.QLineEdit(self.scene_widget_add[n])
        self.lineEdit_scene_add[n].setText("")
        self.lineEdit_scene_add[n].setObjectName("lineEdit_scene")
        self.horizontalLayout_scene_add[n].addWidget(self.lineEdit_scene_add[n])

        self.label_start_add[n] = QtWidgets.QLabel(self.scene_widget_add[n])
        self.label_start_add[n].setObjectName("label_7")
        self.label_start_add[n].setText("开始(周内秒, s) ：")
        self.horizontalLayout_scene_add[n].addWidget(self.label_start_add[n])

        self.lineEdit_start_add[n] = QtWidgets.QLineEdit(self.scene_widget_add[n])
        self.lineEdit_start_add[n].setText("")
        self.lineEdit_start_add[n].setObjectName("lineEdit_StarTime")
        self.horizontalLayout_scene_add[n].addWidget(self.lineEdit_start_add[n])

        self.label_end_add[n] = QtWidgets.QLabel(self.scene_widget_add[n])
        self.label_end_add[n].setObjectName("label_8")
        self.label_end_add[n].setText("结束(周内秒, s) ：")
        self.horizontalLayout_scene_add[n].addWidget(self.label_end_add[n])

        self.lineEdit_end_add[n] = QtWidgets.QLineEdit(self.scene_widget_add[n])
        self.lineEdit_end_add[n].setText("")
        self.lineEdit_end_add[n].setObjectName("lineEdit_EndTime")
        self.horizontalLayout_scene_add[n].addWidget(self.lineEdit_end_add[n])

        self.verticalLayout_4.addWidget(self.scene_widget_add[n])

    # '''添加一份测试数据的组件'''
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

    def clear_test_file_record(self):
        key_list = list(self.widget_add.keys())
        for key in key_list:
            self.verticalLayout.removeWidget(self.widget_add[key])
            self.widget_add.pop(key)
            self.horizontalLayout_add.pop(key)
            self.label_add.pop(key)
            self.lineEdit_GetInsFile_add.pop(key)
            self.pushBtn_SelectIns_add.pop(key)
            self.toolButton_add.pop(key)
        self.inscont = 1
        self.refpath = ''
        self.inspath = ''
        self.lineEdit_GetInsFile.setText('')
        self.lineEdit_GetRefFile.setText('')
        self.time_dir = {}
        self.time_dir_config = {}
        self.GpsBpos = {}
        self.InsBpos = {}

    # 打开配置参数窗口
    def get_config(self, n):
        """点击相应按钮，跳转到第二个界面"""
        # 实例化第二个界面的后端类，并对第二个界面进行显示
        # 通过派生新类去访问类

        self.config_window = ui_another_window_actions.AnotherWindowActions()
        self.config_window.show()
        self.config_window.saveDataButton.clicked.connect(lambda: self.save_config_data(n))
        self.config_window.saveDataButton.clicked.connect(self.config_window.close)

    # 打开 导入场景 窗口
    def onClickedSelectFile_scene(self):
        """
        @author: liqianwen
        @version: for 数据解析分析工具 V2.0.1
        """
        key_list = list(self.scene_widget_add.keys())
        for key in key_list:
            self.verticalLayout_4.removeWidget(self.scene_widget_add[key])
            self.scene_widget_add.pop(key)
            self.horizontalLayout_scene_add.pop(key)
            self.label_scene_add.pop(key)
            self.label_start_add.pop(key)
            self.label_end_add.pop(key)
            self.lineEdit_scene_add.pop(key)
            self.lineEdit_start_add.pop(key)
            self.lineEdit_end_add.pop(key)
        self.lineEdit_scene.setText('')
        self.lineEdit_StarTime.setText('')
        self.lineEdit_EndTime.setText('')

        output_info = ''
        time_dir = {}
        self.setPlotBtnFlase()
        config_file_path = ''
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        config_file_path = config_file_path + fileName
        self.scene_cont = 0
        if not os.path.isfile(config_file_path):
            self.outputMsg2('已清空场景, 文件' + config_file_path + "文件不存在...")
            config_file_path = ''
            self.time_dir_config = {}
        else:
            try:
                with open(config_file_path, 'r', encoding='utf-8') as file_t:
                    for line in file_t:
                        self.scene_cont += 1
                        time_dir[str(self.scene_cont)] = {'scene_num': int(line.split(',')[0])
                            , 'scene': line.split(',')[-1][:-1]
                            , 'time_arrange': [float(line.split(',')[2]), float(line.split(',')[3])]}
                        have_all_scene = 1 if time_dir[str(self.scene_cont)]['time_arrange'] != [0,0] or have_all_scene else 0
                if have_all_scene:
                    self.scene_cont += 1
                    time_dir[str(self.scene_cont)] = {'scene_num': 0
                        , 'scene': '全程'
                        , 'time_arrange': [0, 0]}

                output_info += '场景配置文件解析结果如下:\n'
                for key in time_dir.keys():
                    output_info += '  场景 %s 的时间范围：' % time_dir[key]['scene'] + str(
                        time_dir[key]['time_arrange']) + '\n'
                output_info += 'PS. 全程的时间范围默认为：[0, 0]。\n'
                output_info += '请确认信息，若有误请检查场景配置文件!\n'
                self.outputMsg2(output_info)
                self.time_dir_config = time_dir
            except Exception as e:
                output_info += '场景配置文件解析有误:\n'
                output_info += 'Exception: \n'
                output_info += e
                self.outputMsg2(output_info)
                print(e)

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

    # INSGPS 对比，一份测试文件按钮按下
    def onClickedSelectFile(self):
        self.setPlotBtnFlase()
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        self.lineEdit_GetFile.setText(fileName)

    # 选择INS文件按钮按下，多份测试文件
    def onClickedSelectINSFile(self, n):
        self.setPlotBtnFlase()
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        if int(n) == 1:
            self.lineEdit_GetInsFile.setText(fileName)
        elif int(n) > 1:
            self.lineEdit_GetInsFile_add[n].setText(fileName)
        self.InsBpos[n] = [0, 0, 0]
        self.GpsBpos[n] = [0, 0, 0]

    # 选择参考文件按钮按下
    def onClickedSelectRefFile(self):
        self.setPlotBtnFlase()
        fileName, fileType = QtWidgets.QFileDialog.getOpenFileName(self, "选择文件", os.getcwd(),
                                                                   "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        self.lineEdit_GetRefFile.setText(fileName)

    def onClickedStartParseInsRef(self):
        self.setPlotBtnFlase()

        # reset some value
        self.inspaths = []
        ##################################### 取文件、判断是否为有效地址 #####################################
        self.inspath = self.lineEdit_GetInsFile.text()  # 获取INS文件路径
        self.refpath = self.lineEdit_GetRefFile.text()  # 获取参考文件路径
        self.inspaths.append(self.inspath)
        for n in self.lineEdit_GetInsFile_add.keys():  # 获取增加INS文件路径
            file_add = self.lineEdit_GetInsFile_add[n].text()
            self.inspaths.append(file_add)
        empty_file_index = []
        for i in range(len(self.inspaths)):
            file = self.inspaths[i]
            if not os.path.isfile(file):
                self.outputMsg2(file + "文件不存在...")
                empty_file_index.append(i)
                continue
        # delete empty files
        for index in empty_file_index:
            self.inspaths.pop(index)
        if not os.path.isfile(self.refpath):
            self.outputMsg2(self.refpath + "文件不存在...")
            return

        # 获取基准数据的数据类型
        self.ref_type = self.select_type.currentText()
        self.outputMsg2(self.refpath + "为 %s 文件" % self.ref_type)

        ##################################### 获取需要解析的场景信息 #####################################
        self.time_dir = self.time_dir_config.copy()
        print(self.time_dir)
        self.scene_cont = len(self.time_dir.keys())
        # 场景中有填写东西
        if self.lineEdit_scene.text() != '':
            self.scene_cont += 1
            try:
                self.time_dir[str(self.scene_cont)] = {'scene_num': '手动填写'
                    , 'scene': self.lineEdit_scene.text()
                    , 'time_arrange': [float(self.lineEdit_StarTime.text()), float(self.lineEdit_EndTime.text())]}
            except Exception as e:
                self.outputMsg2(self.lineEdit_scene.text() + ' 场景输入错误，默认为全程！')
                self.time_dir[str(self.scene_cont)] = {'scene_num': '手动填写'
                    , 'scene': self.lineEdit_scene.text()
                    , 'time_arrange': [0, 0]}
        else:
            pass
        for n in self.lineEdit_scene_add.keys():  # 获取增加INS文件路径
            self.scene_cont += 1
            try:
                self.time_dir[str(self.scene_cont)] = {'scene_num': '手动填写'
                    , 'scene': self.lineEdit_scene_add[n].text()
                    , 'time_arrange': [float(self.lineEdit_start_add[n].text()),
                                       float(self.lineEdit_end_add[n].text())]}
            except Exception as e:
                self.outputMsg2(self.lineEdit_scene_add[n].text() + ' 场景输入错误，默认为全程！')
                self.time_dir[str(self.scene_cont)] = {'scene_num': '手动填写'
                    , 'scene': self.lineEdit_scene_add[n].text()
                    , 'time_arrange': [0, 0]}
        # 场景中没有填写东西，默认统计绘制全局 --> 在绘图前判断
        if len(list(self.time_dir)) == 0:
            self.scene_cont += 1
            self.time_dir[str(self.scene_cont)] = {'scene_num': 0, 'scene': '全程', 'time_arrange': [0, 0]}
        # 打印场景信息
        if self.time_dir:
            output_info = '即将解析的场景:\n\n'
            for key in self.time_dir.keys():
                output_info += '  场景 %s 的时间范围：' % self.time_dir[key]['scene'] + str(
                    self.time_dir[key]['time_arrange']) + '\n'
            self.outputMsg2(output_info)

        ##################################### 判断画图的时间段 #####################################
        plot_time_range = [0, 0]
        plot_scene = '全程'
        # 1、有config.txt但文件内里没有东西--画全程
        # 2、没有config，也没写场景--画全程
        # 3、没有config，有一条记录--画该记录
        if not self.scene_widget_add and 1 == len(self.time_dir):
            if self.lineEdit_StarTime.text():
                # 有写一条记录
                plot_scene = self.lineEdit_scene.text()
                plot_time_range[0] = float(self.lineEdit_StarTime.text())
            if self.lineEdit_EndTime.text():
                plot_time_range[1] = float(self.lineEdit_EndTime.text())

            # 没有config，有一条记录
            if plot_time_range[0] == 0 and plot_time_range[1] == 0:
                for val in self.time_dir.values():
                    plot_time_range[0] = val['time_arrange'][0]
                    plot_time_range[1] = val['time_arrange'][1]
        # 获取GPS显示数量
        GpsNum = self.showGPS_spinBox.value()

        ##################################### 开启子线程 #####################################
        self.push_info_2.emit([self.refpath, self.ref_type, self.inspaths, self.time_dir, plot_time_range, plot_scene, GpsNum, self.InsBpos, self.GpsBpos])
        self.thread_2.start()
        # self.pushBtn_StartPlot2.setEnabled(True)
        # self.createDataAnanlyseThread()  # 创建数据解析线程，开始数据解析

    # 画图
    def onClickedStartPlotInsRef(self):
        # 获取显示时间类型
        self.checkOutputTimeType2()

        self.thread_2.plot_insgps_compare(second_of_week=self.checkOutputTimeType2())

    def checkOutputTimeType2(self):
        if self.radioButton_time2.isChecked():
            # self.PlotGpsInsRawSyncDataObj.second_of_week = False
            return False
        elif self.radioButton_tow2.isChecked():
            # self.PlotGpsInsRawSyncDataObj.second_of_week = True
            return True

    def clear_handinput_scenes(self):
        try:
            # 场景统计完成，清空已有场景
            # 单条场景
            if len(list(self.time_dir.keys())) == 1:
                # 此条场景是手动输入
                if self.lineEdit_scene.text() != '':
                    self.scene_cont = 0
                    self.time_dir = {}
                # 此条场景是通过场景文件输入
                elif self.time_dir[list(self.time_dir.keys())[0]]['time_arrange'] == [0, 0]:
                    print('1条场景，且此条场景值为[0,0]')
                    self.scene_cont = 0
                    self.time_dir = {}
                else:
                    print('1条场景，且此条场景通过场景文件输入')
            else:
                # 手动输入场景不止一条
                print('手动输入场景不止一条')
                if len(list(self.scene_widget_add.keys())) > 0:
                    for key in list(self.scene_widget_add.keys()):
                        print(key)
                        print('删除场景记录：' + str(self.time_dir[str(self.scene_cont)]))
                        self.time_dir.pop(str(self.scene_cont))
                        self.scene_cont -= 1
                # 清除最上方的场景
                if self.lineEdit_scene.text() != '':
                    print('删除场景记录：' + str(self.time_dir[str(self.scene_cont)]))
                    self.time_dir.pop(str(self.scene_cont))
                    self.scene_cont -= 1
            print(self.time_dir)
        except Exception as e:
            print(e)
            self.outputMsg2('场景输入有误，请点击清除场景键')

    def outputMsg2(self, msg_str):
        if '开始解析：' in msg_str:
            self.pushBtn_StartPlot2.setEnabled(False)
        if '【绘图】时间同步完成，可生成图' in msg_str:
            self.pushBtn_StartPlot2.setEnabled(True)
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        self.textBrowser_showmsg2.append(msg)

    def onClickedClearMsg2(self):
        self.textBrowser_showmsg2.clear()  # 清空textBrowser_showMsg里的消息
        self.thread_2.PlotGpsInsRawSyncDataObj.close_plot()

    ##################################### CAN解析 #####################################
    def create_can_ananlyse_thread(self):
        t = Thread(target=self.can_analysis_func)  # 创建一个线程
        t.setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
        t.start()  # 启动线程

    def output_can_msg(self, msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        self.textBrowser_can.append(msg)

    def on_clicked_select_dbc_file(self):
        self.dbc_files_dict = {}
        self.setPlotBtnFlase()
        # files_list = ''
        files_list, fileType = QtWidgets.QFileDialog.getOpenFileNames(self, "选择文件", os.getcwd(),
                                                                      "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        # files_list = files_list + file_name
        self.lineEdit_getDBCtext.setText(str(files_list))
        self.output_can_msg('\n'.join(["选择dbc文件： "] + files_list))
        self.dbcs_path_list = files_list

    def on_clicked_select_blf_file(self):
        self.blf_files_dict = {}
        self.setPlotBtnFlase()
        # files_list = ''
        files_list, fileType = QtWidgets.QFileDialog.getOpenFileNames(self, "选择文件", os.getcwd(),
                                                                      "All Files(*);;Text Files(*.txt)")  # 打开文件对话框，选择文件
        # files_list = files_list + file_name
        self.lineEdit_GetFiletext.setText(str(files_list))
        self.output_can_msg('\n'.join(["选择blf/asc文件： "] + files_list))
        self.blfs_path_list = files_list

    def on_clicked_start_analysis_blf(self):
        self.dbc_paths = []
        self.dbc_path = self.lineEdit_getDBCtext.text()  # 获取dbc文件路径
        self.blf_path = self.lineEdit_GetFiletext.text()  # 获取参考文件路径
        self.dbc_paths.append(self.dbc_path)

        for file in self.dbcs_path_list:
            if not os.path.isfile(file):
                self.output_can_msg(file + "文件不存在...")
                return
        for file in self.blfs_path_list:
            if not os.path.isfile(file):
                self.output_can_msg(file + "文件不存在...")
                return

        for dbc_path in self.dbcs_path_list:
            key = os.path.abspath(dbc_path).split(os.sep)[-1]
            self.dbc_files_dict[key] = dbc_path

        for blf_path in self.blfs_path_list:
            key = os.path.abspath(blf_path).split(os.sep)[-1]
            self.blf_files_dict[key] = blf_path
        # self.outputMsg2("hahaha...")

        self.create_can_ananlyse_thread()  # 创建数据解析线程，开始数据解析

    def can_analysis_func(self):
        try:
            # 100C 数据解析
            self.output_can_msg("开始解析, 请稍等...")
            loader = canLoader(self.blf_files_dict, self.dbc_files_dict)
            signals_value_db_dict, msg_info = loader.main()
            self.output_can_msg(msg_info)
            self.output_can_msg("解析完成！")

        except Exception as e:
            self.output_can_msg("解析失败...")
            self.output_can_msg('失败原因:' + str(e))

    ##################################### 使用说明 #####################################
    def open_help_web(self):
        try:
            url = 'https://asensing.feishu.cn/docx/doxcn0rUDcvflKKm5wPmmdisKab '
            webbrowser.open_new_tab(url)
            self.output_can_msg('已打开软件说明书')
            self.outputMsg1('已打开软件说明书')
            self.outputMsg2('已打开软件说明书')
        except Exception as e:
            self.output_can_msg('无法打开软件说明书')
            self.outputMsg1('无法打开软件说明书')
            self.outputMsg2('无法打开软件说明书')
            self.output_can_msg(e)
