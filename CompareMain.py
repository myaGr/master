import numpy as np
import os
import pandas as pd
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
        self.InsDataDF = None
        self.GpsDataDF = None

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
            self.InsDataDF = self.HexDataParseObj.InsDataDF
            self.GpsDataDF = self.HexDataParseObj.GpsDataDF
            self.outputMsg1("解析完成。")

            self.outputDataFrameStatsResult()  # 输出数据帧统计结果
        except Exception as e:
            self.outputMsg1(self.file_path + "文件解析失败...")
            self.outputMsg1('失败原因:' + str(e))

        try:
            if "csv" in self.types:
                self.outputMsg1("生成Csv文件...")
                self.HexDataParseObj.SaveAllDataToCsvFile()
                self.outputMsg1("已生成")
            if "mat" in self.types:
                self.outputMsg1("生成Mat文件...")
                self.HexDataParseObj.startSaveAllDataToMatFile()
                self.outputMsg1("已生成")
        except Exception as e:
            self.outputMsg1("文件有误，无法生成csv/mat文件！")

        # 时间同步
        self.outputMsg1("开始INS和GPS时间同步...")
        try:
            self.InsDataDF = self.InsDataDF.replace(np.nan,0)
            self.GpsDataDF = self.GpsDataDF.replace(np.nan,0)
            self.SyncInsGpsData = self.DataPreProcess.timeSynchronize(self.InsDataDF,
                                                                      self.GpsDataDF,
                                                                      'time', 'itow_pos')
            self.outputMsg1("【绘图】时间同步完成，可生成统计图。")
        except Exception as e:
            self.outputMsg1("时间同步失败。")
            self.outputMsg1('失败原因:' + str(e))

    def plot_insgps(self, second_of_week=False):
        self.outputMsg1("开始INS和GPS统计画图...")
        self.PlotGpsInsRawSyncDataObj.InsDataDF = self.InsDataDF
        self.PlotGpsInsRawSyncDataObj.GpsDataDF = self.GpsDataDF
        self.PlotGpsInsRawSyncDataObj.PDataDict = self.HexDataParseObj.PDataDict
        self.PlotGpsInsRawSyncDataObj.SyncDataDF = self.HexDataParseObj.SyncDataDF
        self.PlotGpsInsRawSyncDataObj.VehicleDataDF = self.HexDataParseObj.VehicleDataDF
        self.PlotGpsInsRawSyncDataObj.ImuDataDF = self.HexDataParseObj.ImuDataDF
        self.PlotGpsInsRawSyncDataObj.SyncInsGpsData = self.SyncInsGpsData
        # 繪圖橫坐標為周内秒: 默認False
        self.PlotGpsInsRawSyncDataObj.second_of_week = second_of_week

        self.HexDataParseObj.clear_cache()

        try:
            print("轨迹生图..")
            self.PlotGpsInsRawSyncDataObj.plot_raw_path()
        except Exception as e:
            self.outputMsg1("轨迹生图失败...")
            self.outputMsg1('失败原因:' + str(e))

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

            msg = "30卫星数据帧：" \
                  "\n    纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['satelliteFrameHeadNum_bdbd30']) \
                  + "，\n    总帧数量:" + str(self.HexDataParseObj.dataFrameStats['satelliteDataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.HexDataParseObj.dataFrameStats['satelliteCheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.HexDataParseObj.dataFrameStats['satelliteCheckErrIndex']))) \
                  + "。\n\n"
            self.outputMsg1(msg)

            msg = "31卫星数据帧：" \
                  "\n    纯帧头数量:" + str(self.HexDataParseObj.dataFrameStats['satellite2FrameHeadNum_bdbd31']) \
                  + "，\n    总帧数量:" + str(self.HexDataParseObj.dataFrameStats['satellite2DataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.HexDataParseObj.dataFrameStats['satellite2CheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.HexDataParseObj.dataFrameStats['satellite2CheckErrIndex']))) \
                  + "。\n\n"
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
        self.test_type = []

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
        self.test_type = data_info[9]

    def outputMsg2(self, msg_str):
        print(msg_str)
        # self.updated.emit(msg_str)

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
            ##################################### liuzhiqiang  华测
            elif self.ref_type == '华测特制':
                self.outputMsg2(ref_file_name + "为 刘志强版本 设备输出格式。")
                self.outputMsg2("基准格式为：gps周、gps周内秒、纬度、经度、大地高、定位状态"
                                "、使用卫星数、东向标准差、北向标准差、高程标准差、协方差、协方差、协方差"
                                "、位置精度因子、静态动态状态、东向速度、北向速度、天向速度、横滚角、俯仰角、航向角")
                ref_flag = self.Parse100CDataObj.save_lzq_to_df()  # 开始参考数据解析
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
        self.outputMsg2('测试数据类型：'+str(self.test_type))
        self.name_list = []
        for file in self.inspaths:
            file_name = file.split('/')[-1].split('.')[0]
            self.name_list.append(file_name)
            try:
                # 数据解析
                if self.test_type[self.inspaths.index(file)] == '导远自定义':
                    self.HexDataParseObj.filePath = file
                    self.outputMsg2("开始解析导远自定义测试文件：" + file)
                    self.HexDataParseObj.startParseFileHexData()  # 开始INS数据解析
                    self.HexDataParseObj.saveDataToDF()
                    if self.HexDataParseObj.InsDataDF is not None:
                        self.outputMsg2("数据解析完成。")
                        # 数据保存
                        self.InsDataDF[file_name] = self.HexDataParseObj.InsDataDF
                        self.HexDataParseObj.InsDataDF = None
                        self.GpsDataDF[file_name] = self.HexDataParseObj.GpsDataDF
                        self.HexDataParseObj.GpsDataDF = None
                        # # TODO: 刘志强要求删减，仅在  华测特制  有效
                        # self.InsDataDF[file_name]['roll'] = self.InsDataDF[file_name]['roll'] + 0.200680469
                        # self.InsDataDF[file_name]['pitch'] = self.InsDataDF[file_name]['pitch'] + 0.333099707
                        # self.InsDataDF[file_name]['yaw'] = self.InsDataDF[file_name]['yaw'] - 2.847532422
                    else:
                        self.outputMsg2("数据无效，解析失败。")
                        continue
                else:
                    self.outputMsg2("开始解析北云测试文件：" + file)
                    self.InsDataDF[file_name] = self.Parse100CDataObj.beiYunDataToDataframe(file)
                    self.GpsDataDF[file_name] = {'Lon':[], 'LonStd':[], 'Lat':[], 'LatStd':[], 'hMSL':[], 'hMSLStd':[], 'gpsFix':[], 'flags':[],
                                               'HSpd':[], 'TrackAngle':[], 'VSpd':[], 'LatencyVel':[], 'BaseLineLength':[], 'Heading':[],
                                               'HeadingStd':[], 'Pitch':[], 'PitchStd':[], 'year':[], 'month':[], 'day':[], 'hour':[],
                                               'minute':[], 'second':[], 'itow_pos':[], 'itow_vel':[], 'itow_heading':[], 'RecMsg':[],
                                               'numSV':[], 'flagsPos':[], 'ts':[]}
                    self.GpsDataDF[file_name] = pd.DataFrame(self.GpsDataDF[file_name])
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
                    if len(self.PlotGpsInsRawSyncDataObj.SyncRefInsData[file_name]) == 0:
                        self.outputMsg2(file_name + ': INS和参考数据时间同步失败 (数据都被过滤了，请检查数据中的IMUstatus和flagsPos值！)')
                    else:
                        self.outputMsg2(file_name + ': INS和参考数据时间同步完成')

                    if self.test_type[self.name_list.index(file_name)] == '导远自定义':
                        self.outputMsg2(file_name + ": GPS和参考数据时间同步...")
                        self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name] = self.DataPreProcess.timeSynchronize(
                            self.Parse100CDataObj.ins100cdf, self.GpsDataDF[file_name], 'time', 'itow_pos')
                        if len(self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name]) == 0:
                            self.outputMsg2(file_name + ': GPS和参考数据时间同步失败 （数据不存在或是数据都被过滤了，请检查数据中的IMUstatus和flagsPos值！)')
                        else:
                            self.outputMsg2(file_name + ': GPS和参考数据时间同步完成')
                    else:
                        self.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name] = {}
                        self.outputMsg2(file_name + '非自定义数据，默认无GPS数据 不进行同步GPS操作')
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

                    self.outputMsg2('统计时间范围为%s的数据, 为场景：%s' % (str(time_arrange), scene_dscribe))
                    self.PlotGpsInsRawSyncDataObj.iniInsGpsBpos()

                    self.PlotGpsInsRawSyncDataObj.gps_flag = dict.fromkeys(self.name_list, 0)
                    static_msg_info = self.PlotGpsInsRawSyncDataObj.dataPreStatistics(time_arrange=time_arrange, test_type=self.test_type)

                    self.PlotGpsInsRawSyncDataObj.gen_statistics_xlsx(os.getcwd(), time_arrange=time_arrange, scene=scene_dscribe)

                    self.outputMsg2(static_msg_info)
                    static_msg_info = ''
                    self.outputMsg2("场景 %s 统计完成！" % scene)
                except Exception as e:
                    self.outputMsg2("场景 %s 统计失败..." % scene)
                    self.outputMsg2('失败原因:' + str(e))
                    continue

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

                        if self.test_type[self.name_list.index(file_name)] == '导远自定义':
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
                    if self.test_type[i] == '导远自定义':
                        self.PlotGpsInsRawSyncDataObj.gps_flag[self.name_list[i]] = 1
                    else:
                        continue
                print("gps_flag:", self.PlotGpsInsRawSyncDataObj.gps_flag)

            self.outputMsg2("【绘图】时间同步完成，可生成图。")

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
            self.outputMsg2("轨迹生图失败...")
            self.outputMsg2('失败原因:' + str(e))

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
        # path = r'D:/Files/test/dbFiles/test2/100/12311-0928测试案例3.txt'
        path = r'D:\Files\test\dbFiles\北云\3\02-tight#2.txt'
        # 错误文件
        # path = r'D:/Files/test/dbFiles/test2/100/config1.txt'
        # 内容有误
        # path = r'D:\Files\test\dbFiles\test7_errordata\202212 08152309911\INS570D_0108_main_linux_didi_20221205.txt'
        # path = r'D:\Downloads\ins_2022_12_25_10_41_27.log'
        types = ['csv','mat']
        # types = [ ]

        thread_1.get_info([path, types])
        thread_1.run()
        thread_1.plot_insgps()

    def compare_test():
        thread_2 = mainFunctionInsCompare()

        # # 刘志强 华测
        # refpath = r'D:\Files\test\dbFiles\刘志强特殊数据\1\test_COM5-1.txt'
        # inspaths = [r'D:\Files\test\dbFiles\刘志强特殊数据\1\2023.06.16.txt']
        # refpath = r'D:\Files\test\dbFiles\刘志强特殊数据\20230617-20230620\真值参考数据\真值\0619\test_COM5_2-1.txt'
        # inspaths = [r'D:\Files\test\dbFiles\刘志强特殊数据\20230617-20230620\570D测试数据\2023.06.19\2023.06.19.txt']

        # 测试：北云 华测
        # refpath = r'E:\Downloads\到测试天线自定义格式-0717PM.txt'
        # inspaths = [r'E:\Downloads\北云-原数据-0714PM.DAT']
        # refpath = r'D:\Files\test\dbFiles\北云\2\到测试天线自定义-0710PM.txt'
        # inspaths = [r'D:\Files\test\dbFiles\北云\2\北云-原数据-0710PM.DAT']
        # refpath = r'D:\Files\test\dbFiles\北云\3\到测试天线自定义-0711PM.txt'
        # inspaths = [
            # r'D:\Files\test\dbFiles\北云\3\03-INS570D-原数据-0711PM.txt',
            #         r'D:\Files\test\dbFiles\北云\3\北云-原数据-0711PM.DAT']

        refpath = r'D:\Files\test\dbFiles\北云\3\到测试天线自定义-0711PM.txt'
        inspaths = [r'D:\Files\test\dbFiles\北云\3\02-tight#2.txt',
                    r'D:\Files\test\dbFiles\北云\3\北云-原数据-0711PM.DAT']

        # inspaths = [
        #             r'D:\Files\test\dbFiles\test13_pos\4\INS20230423102502169\001_1号_M级5711D.txt',
        #             r'D:\Files\test\dbFiles\test13_pos\4\INS20230423102502169\001_2号_M级5711D.txt'
        #             ]
        # refpath = r'D:\Files\test\dbFiles\test13_pos\4\INS20230423102502169\320.txt'

        # inspaths = [r'D:\Downloads\INS2023021715243554\INS_0217.log'
        #             ,r'D:\Downloads\INS2023021715243554\INS_0217_main.txt'
        #             ]
        # refpath = r'D:\Downloads\INS2023021715243554\100c_0217.txt'

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
        ref_type = '320'  # '100C' '320' 华测特制
        # 获取测试数据的数据类型
        test_type = ['导远自定义', '北云']  # '导远自定义' '北云'

        #### 获取需要解析的场景信息 ###
        time_dir = {'1': {'scene_num': 0, 'scene': '全程', 'time_arrange': [0, 0]}}
        # time_dir = {'1': {'scene_num': 1, 'scene': '延安高架上', 'time_arrange': [198524.0, 199799.0]},
        #             '2': {'scene_num': 2, 'scene': '林荫道', 'time_arrange': [200822.0, 201189.0]},
        #             '3': {'scene_num': 3, 'scene': '延安高架下', 'time_arrange': [201200.0, 201795.0]},
        #             '4': {'scene_num': 4, 'scene': '南京路', 'time_arrange': [201992.0, 203319.0]},
        #             '5': {'scene_num': 0, 'scene': '全程', 'time_arrange': [0, 0]}}

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
             InsBpos, GpsBpos, test_type])
        thread_2.run()
        thread_2.plot_insgps_compare()

    def compare_multi_file():
        ref_file_dict = [r'D:\Files\test\dbFiles\test13_pos\bigFile\INS20230412173845494\0308RCGS.txt',
                         r'D:\Files\test\dbFiles\test13_pos\bigFile\INS20230412174147325\0309RCGS.txt',
                         r'D:\Files\test\dbFiles\test13_pos\bigFile\INS20230412174411198\POS320.txt',]
        ref_type_dict = ['320','320','320',]
        test_file_dict = {0:r'D:\Files\test\dbFiles\test13_pos\bigFile\INS20230412173845494\大通.txt',
                          1:r'D:\Files\test\dbFiles\test13_pos\bigFile\INS20230412174147325\大通.txt',
                          2:r'D:\Files\test\dbFiles\test13_pos\bigFile\INS20230412174411198\大通.txt',}
        thread_dict = {}

        start_time = time.time()
        for i in test_file_dict.keys():
            this_file_start = time.time()
            print('第 %d 趟开始时间：%d s' % (i, this_file_start))
            ### 获取文件
            thread_dict[i] = mainFunctionInsCompare()
            inspaths = [test_file_dict[i]]
            refpath = ref_file_dict[i]
            for file in inspaths:
                if not os.path.isfile(file):
                    print(file + "文件不存在...")
                    return
            if not os.path.isfile(refpath):
                print(refpath + "文件不存在...")
                return

            ### 获取基准数据的数据类型
            ref_type = ref_type_dict[i]

            ### 获取需要解析的场景信息 ###
            time_dir = {'1': {'scene_num': 0, 'scene': '全程', 'time_arrange': [0, 0]}}
            plot_time_range = [0, 0]
            plot_scene = '全程'
            InsBpos = {'1': [0, 0, 0]}
            GpsBpos = {'1': [0, 0, 0]}
            ### 获取GPS显示数量
            GpsNum = 1

            ### 运行
            thread_dict[i].get_info(
                [refpath, ref_type, inspaths, time_dir, plot_time_range, plot_scene, GpsNum,
                 InsBpos, GpsBpos])
            thread_dict[i].run()
            print('第 %d 趟解析用时：%d s' % (i, time.time()-this_file_start))
            # thread_dict[i].plot_insgps_compare()
            # print('第 %d 趟结束画图用时：%d s' % (i, time.time()-this_file_start))


        print('总共用时：%d s' % time.time() - start_time)

    compare_test()
    # compare_multi_file()
    # analyze_single_file()
    print('over')


