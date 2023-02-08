import numpy as np
import os
from func.HexDataParse import HexDataParse
from func.PlotGpsInsSyncData import PlotGpsInsRawSyncData
from func.Parse100CData import Parse100CData
from func.DataPreProcess import DataPreProcess
import time
from PyQt5 import QtCore


class StatisticAndPlot:
    """
    测试功能：与自身对比 + 与基准数据对比
    @author: wenzixuan liqianwen
    @data: 2022-11-01
    @version: For version_1.3.2
    """

    def __init__(self):
        # 实例化数据解析对象
        self.InsDataDFInter = None
        self.SyncInsGpsData = None
        self.SyncRefInsData = None
        self.SyncRefGpsData = None
        self.SyncRefGpsINSData = None
        self.HexDataParseObj = HexDataParse()
        self.Parse100CDataObj = Parse100CData()
        self.DataPreProcess = DataPreProcess()
        self.PlotGpsInsRawSyncDataObj = PlotGpsInsRawSyncData(ref_type='100C')

        self.InsDataDF = {}
        self.GpsDataDF = {}
        self.PDataDict = {}
        self.SyncDataDF = {}
        self.VehicleDataDF = {}
        self.ImuDataDF = {}
        self.ins100cdf = {}

    def GetParseInsData(self, gen_file_type=None):
        print(time.strftime('%H:%M:%S', time.localtime()), "开始解析数据: ", self.HexDataParseObj.filePath)
        self.HexDataParseObj.startParseFileHexData()  # 开始数据解析
        print(time.strftime('%H:%M:%S', time.localtime()), "解析完成...")
        self.HexDataParseObj.saveDataToDF()
        if "csv" in gen_file_type:
            print("存成Csv文件...")
            self.HexDataParseObj.SaveAllDataToCsvFile()
        if "mat" in gen_file_type:
            print("存成Mat文件...")
            self.HexDataParseObj.startSaveAllDataToMatFile()
        print(time.strftime('%H:%M:%S', time.localtime()), "数据保存完成")

    def InsDataAnalysis(self):
        print(time.strftime('%H:%M:%S', time.localtime()), "开始参考数据时间插值...")
        # InsDataDFfilter = self.DataPreProcess.Datafilter(self.HexDataParseObj.InsDataDF, [0, 0])  # 过滤无效数据
        # self.InsDataDFInter = self.DataPreProcess.Datainterpolation(InsDataDFfilter)  # 时间插值
        print(time.strftime('%H:%M:%S', time.localtime()), "INS和GPS时间同步...")
        self.SyncInsGpsData = self.DataPreProcess.timeSynchronize(self.HexDataParseObj.InsDataDF,
                                                                  self.HexDataParseObj.GpsDataDF, 'time', 'itow_pos')
        print(time.strftime('%H:%M:%S', time.localtime()), "INS和GPS统计画图开始...")
        self.PlotGpsInsRawSyncDataObj.InsDataDF = self.HexDataParseObj.InsDataDF
        self.PlotGpsInsRawSyncDataObj.GpsDataDF = self.HexDataParseObj.GpsDataDF
        self.PlotGpsInsRawSyncDataObj.PDataDict = self.HexDataParseObj.PDataDict
        self.PlotGpsInsRawSyncDataObj.SyncDataDF = self.HexDataParseObj.SyncDataDF
        self.PlotGpsInsRawSyncDataObj.VehicleDataDF = self.HexDataParseObj.VehicleDataDF
        self.PlotGpsInsRawSyncDataObj.ImuDataDF = self.HexDataParseObj.ImuDataDF
        self.PlotGpsInsRawSyncDataObj.SyncInsGpsData = self.SyncInsGpsData
        self.PlotGpsInsRawSyncDataObj.PlotGpsInsRawSyncData()

    def Get100CData(self):
        print(time.strftime('%H:%M:%S', time.localtime()), "开始解析数据: ", self.Parse100CDataObj.filepath)
        #  判断是否为100C参考数据
        ref_file_name = self.Parse100CDataObj.filepath.split('/')[-1].split('.')[0]
        if '100C' in ref_file_name:
            print(ref_file_name + '为100C格式输出数据')
            ref_flag = self.Parse100CDataObj.save100Ctodf()  # 开始参考数据解析
        else:
            print(ref_file_name + '为POS320格式输出数据')
            ref_flag = self.Parse100CDataObj.save_320_to_df()
        if ref_flag:
            print("缺少字段：" + str(ref_flag) + ", 解析失败。")
            return
        if len(self.Parse100CDataObj.ins100cdf) == 0:
            print("数据无效，解析失败。")
            return

    def InsRefStatistics(self, t):
        self.DataPreProcess.t = t
        # print(time.strftime('%H:%M:%S', time.localtime()), "INS和参考数据时间同步...")
        # self.PlotGpsInsRawSyncDataObj.SyncRefInsData = self.DataPreProcess.timeSynchronize(
        #     obj.Parse100CDataObj.ins100cdf, obj.HexDataParseObj.InsDataDF, 'time', 'time')
        # print(time.strftime('%H:%M:%S', time.localtime()), "GPS和参考数据时间同步...")
        # self.PlotGpsInsRawSyncDataObj.SyncRefGpsData = self.DataPreProcess.timeSynchronize(
        #     obj.Parse100CDataObj.ins100cdf, obj.HexDataParseObj.GpsDataDF, 'time', 'itow_pos')
        # print(time.strftime('%H:%M:%S', time.localtime()), "INS和参考对比统计画图开始...")
        # self.PlotGpsInsRawSyncDataObj.PlotRefGpsInsSyncData(self.HexDataParseObj.filePath)


class mainFunctionAnalysicSingleFile(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)  # 信号类变量（字符串）

    # 参数初始化
    def __init__(self, parent=None):
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
        # self.updated.emit(msg_str)
        print(msg_str)

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
        except Exception as e:
            self.outputMsg1(self.file_path + "文件解析失败...")
            self.outputMsg1('失败原因:' + str(e))

        try:
            if "csv" in self.types:
                self.outputMsg1("生成Csv文件...")
                self.HexDataParseObj.SaveAllDataToCsvFile()
            if "mat" in self.types:
                self.outputMsg1("生成Mat文件...")
                self.HexDataParseObj.startSaveAllDataToMatFile()
        except Exception as e:
            self.outputMsg1("文件有误，无法生成csv/mat文件！")

        # 时间同步
        self.outputMsg1("开始INS和GPS时间同步...")
        try:
            self.SyncInsGpsData = self.DataPreProcess.timeSynchronize(self.HexDataParseObj.InsDataDF,
                                                                      self.HexDataParseObj.GpsDataDF, 'time',
                                                                      'itow_pos')
            self.outputMsg1("时间同步完成，可生成统计图。")
        except Exception as e:
            self.outputMsg1("时间同步失败。")
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
            print("开始INS和GPS统计画图...")
            msg_info = self.PlotGpsInsRawSyncDataObj.PlotGpsInsRawSyncData()
            self.outputMsg1(msg_info)
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
    def __init__(self, parent=None):
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
        # self.time_dir = {'1': {'scene' : '内环高架上', 'scene_num': 2, 'time_arrange': [0.0, 281800.0]}, '2': {'scene': '林荫道', 'scene_num': 4, 'time_arrange': [281810.0, 282000.0]}, '3': {'scene': '高楼多径', 'scene_num': 5, 'time_arrange': [282010.0, 282200.0]}, '4': {'scene': '隧道', 'scene_num': 71, 'time_arrange': [282210.0, 0.0]}, '5': {'scene': '全程', 'scene_num': 0, 'time_arrange': [0, 0]}}
        self.plot_scene_time = data_info[4]  # 画图场景，将赋值给 self.DataPreProcess.t
        self.plot_scene = data_info[5]
        self.GpsNum = data_info[6]
        self.InsBpos = data_info[7]
        self.GpsBpos = data_info[8]

    def outputMsg2(self, msg_str):
        print(msg_str)
        # self.updated.emit(msg_str)

    def run(self):
        self.HexDataParseObj = HexDataParse()
        self.PlotGpsInsRawSyncDataObj = PlotGpsInsRawSyncData()
        self.DataPreProcess = DataPreProcess()
        self.Parse100CDataObj = Parse100CData()

        # 1、解析基准数据
        try:
            # ps. self.Parse100CDataObj.ins100cdf 会被重置
            self.outputMsg2("开始解析：" + self.refpath)
            self.Parse100CDataObj.filepath = self.refpath
            ref_file_name = self.refpath.split('/')[-1].split('.')[0]
            if self.ref_type == '320':
                self.outputMsg2(ref_file_name + "为 POS320 设备输出格式。")
                try:
                    ref_flag = self.Parse100CDataObj.save_320_to_df()  # 开始参考数据解析
                except Exception as e:
                    print(e)
                    self.outputMsg2("尝试解析另一种 POS320 设备输出格式。")
                    ref_flag = self.Parse100CDataObj.save_320_to_df_1()  # 开始参考数据解析
            elif self.ref_type == '100C':
                self.outputMsg2(ref_file_name + "为 100C 设备输出格式。")
                ref_flag = self.Parse100CDataObj.save100Ctodf()  # 开始参考数据解析
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

        # 2、时间同步
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

            self.outputMsg2("场景统计完成。")

        # 4、绘图参数配置
        try:
            # 获取绘图的开始和结束时间
            self.DataPreProcess.t = self.plot_scene_time
            print('time_arange:', self.DataPreProcess.t)
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
                        self.outputMsg2(file_name + "【绘图】: INS和参考数据时间同步，同步失败的时间段为: ",self.plot_scene_time)
                        self.PlotGpsInsRawSyncDataObj.SyncRefInsData[file_name] = self.DataPreProcess.timeSynchronize(
                            self.Parse100CDataObj.ins100cdf, self.InsDataDF[file_name], 'time', 'time')
                        # if self.PlotGpsInsRawSyncDataObj.gps_flag[file_name]:
                        self.outputMsg2(file_name + "【绘图】: GPS和参考数据时间同步，同步失败的时间段为: ",self.plot_scene_time)
                        self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name] = self.DataPreProcess.timeSynchronize(
                            self.Parse100CDataObj.ins100cdf, self.GpsDataDF[file_name], 'time', 'itow_pos')
                    except Exception as e:
                        self.outputMsg2(file_name + "【绘图】: 时间同步失败，即将绘制全程图，同步失败的时间段为: ",self.plot_scene_time)
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


if __name__ == "__main__":
    # # 1. INS数据解析与自身GPS对比画图
    # obj = StatisticAndPlot()
    # obj.HexDataParseObj.filePath = r"D:\Files\dbFiles\12302.txt"
    # # obj.HexDataParseObj.filePath = "C:/Users/wenzixuan/Downloads/lqw/111.txt"
    # # types = ["csv"]
    # types = []
    # obj.PlotGpsInsRawSyncDataObj.second_of_week = True
    # obj.GetParseInsData(types)
    # obj.InsDataAnalysis()

    def analyze_single_file():
        thread_1 = mainFunctionAnalysicSingleFile()

        # 正常文件
        # path = r'D:\Files\test\dbFiles\test6_320\12311-1114-紧组合.txt'
        # 错误文件
        # path = r'D:/Files/test/dbFiles/test2/100/config1.txt'
        # 内容有误
        # path = r'D:\Files\test\dbFiles\test7_errordata\202212 08152309911\INS570D_0108_main_linux_didi_20221205.txt'
        path = r'D:\Downloads\12311-0118-imu-2.txt'
        # types = ['csv', 'mat']
        types = []

        thread_1.get_info([path, types])
        thread_1.run()
        thread_1.plot_insgps()


    def compare_test():
        thread_2 = mainFunctionInsCompare()

        # reset some value
        # inspaths = [r'D:\Downloads\algo_bin.log']
        # inspaths = [r'D:/Files/test/dbFiles/test2/100/config1.txt']

        # inspaths = [r'D:/Files/test/dbFiles/test2/100/12311-0928测试案例3.txt']
        # refpath = 'D:/Files/test/dbFiles/test2/100/POS后轮轴_100C_test.txt'

        inspaths = [r'D:\Files\test\dbFiles\test1\test1_LogINS.txt']
        refpath = r'D:\Files\test\dbFiles\test1\100C_test.txt'

        # inspaths = [r'D:\Files\test\dbFiles\test6_320\12311-1114-紧组合.txt'
        #             ,r'D:\Files\test\dbFiles\test6_320\12311-1114-松组合.txt'
        #             ]
        # refpath = r'D:\Files\test\dbFiles\test6_320\1114到后轴320.txt'

        # inspaths = [r'D:\Downloads\12311-1114-紧组合.txt'
        #     ,r'D:\Downloads\12311-1114-松组合.txt'
        #             ]
        # refpath = r'D:\Downloads\1114到后轴320.txt'

        # # 320 error data
        # inspaths = [r'D:\Files\test\dbFiles\test9_320\12311-1227-载波.txt']
        # refpath = r'D:\Files\test\dbFiles\test9_320\后轮轴中心320.txt'

        # 320 error data
        # inspaths = [r'D:\Files\test\dbFiles\test10_320\algo_bin.log']
        # refpath = r'D:\Files\test\dbFiles\test10_320\pos320.txt'

        # another 320 base filename
        # refpath = r'D:\Files\test\dbFiles\another320.txt'
        # refpath = r'D:\Files\test\dbFiles\test11_another320\1高速pos320基准.txt'
        # inspaths = [r'D:\Files\test\dbFiles\test11_another320\AG-2022-05-17_033012_LogINS_截取.txt']

        for file in inspaths:
            if not os.path.isfile(file):
                print(file + "文件不存在...")
                return
        if not os.path.isfile(refpath):
            print(refpath + "文件不存在...")
            return

        # 获取基准数据的数据类型
        ref_type = '100C'  # '100C' '320'

        #### 获取需要解析的场景信息 ###
        time_dir = {'1': {'scene_num': 0, 'scene': '全程', 'time_arrange': [0, 0]}}

        plot_time_range = [0,0]
        # plot_time_range = [203643, 203645]
        # plot_time_range = [202707, 202708]
        plot_scene = '全程'
        InsBpos = {'1': [0, 0, 0]}
        GpsBpos = {'1': [0, 0, 0]}
        # 获取GPS显示数量
        GpsNum = 1

        thread_2.get_info(
            [refpath, ref_type, inspaths, time_dir, plot_time_range, plot_scene, GpsNum,
             InsBpos, GpsBpos])
        thread_2.run()
        thread_2.plot_insgps_compare()


    compare_test()
    # analyze_single_file()
    print('over')


