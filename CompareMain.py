import numpy as np
import os
import pandas as pd
from func.HexDataParse import HexDataParse
from func.PlotGpsInsSyncData import PlotGpsInsRawSyncData
from func.Parse100CData import Parse100CData
from func.DataPreProcess import DataPreProcess
import time


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
        print(time.strftime('%H:%M:%S', time.localtime()), "INS和参考数据时间同步...")
        self.PlotGpsInsRawSyncDataObj.SyncRefInsData = self.DataPreProcess.timeSynchronize(
            obj.Parse100CDataObj.ins100cdf, obj.HexDataParseObj.InsDataDF, 'time', 'time')
        print(time.strftime('%H:%M:%S', time.localtime()), "GPS和参考数据时间同步...")
        self.PlotGpsInsRawSyncDataObj.SyncRefGpsData = self.DataPreProcess.timeSynchronize(
            obj.Parse100CDataObj.ins100cdf, obj.HexDataParseObj.GpsDataDF, 'time', 'itow_pos')
        print(time.strftime('%H:%M:%S', time.localtime()), "INS和参考对比统计画图开始...")
        self.PlotGpsInsRawSyncDataObj.PlotRefGpsInsSyncData(self.HexDataParseObj.filePath)


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

    # 2. INS与参考数据对比画图
    obj = StatisticAndPlot()
    file_list = [
        # 'D:/Files/test/dbFiles/test2/320/12311-0927_test.txt',
        r"D:\Files\test\dbFiles\test2\100\12311-0928_test.txt",
        # r"D:\Files\test\dbFiles\test2\100\12311-0928_test2.txt"
    ]
    # obj.Parse100CDataObj.filepath = r"D:\Downloads\POS320后轮轴中心.txt"
    # obj.Parse100CDataObj.filepath = 'D:/Files/test/dbFiles/test2/320/POS320后轮轴中心_test.txt'
    obj.Parse100CDataObj.filepath = r'D:\Files\test\dbFiles\test2\100\POS后轮轴_100C_test.txt'

    # 最后一个得是 全程 t = [0,0]
    scenes_config_path = r'D:\Files\test\dbFiles\test2\100\config1.txt'
    time_dict = {}
    scene_cont = 0
    with open(scenes_config_path, 'r', encoding='utf-8') as file_t:
        for line in file_t:
            scene_cont += 1
            time_dict[str(scene_cont)] = {'scene_num': int(line.split(',')[0])
                , 'scene': line.split(',')[-1][:-1]
                , 'time_arrange': [float(line.split(',')[2]), float(line.split(',')[3])]}
        scene_cont += 1
        time_dict[str(scene_cont)] = {'scene_num': 0
            , 'scene': '全程'
            , 'time_arrange': [0, 0]}

    obj.DataPreProcess.t = [0, 0]  # 默认[0,0]
    types = []  # ["csv", "mat"]
    # 解析基准数据
    obj.Get100CData()

    name_list = []
    for file in file_list:
        obj.HexDataParseObj.filePath = file
        obj.GetParseInsData(types)
        file_name = file.split('\\')[-1].split('.')[0]
        name_list.append(file_name)

        obj.InsDataDF[file_name] = obj.HexDataParseObj.InsDataDF
        obj.HexDataParseObj.InsDataDF = None
        obj.GpsDataDF[file_name] = obj.HexDataParseObj.GpsDataDF
        obj.HexDataParseObj.GpsDataDF = None

    # 时间同步
    for file_name in name_list:
        print(time.strftime('%H:%M:%S', time.localtime()), "INS和参考数据时间同步...")
        obj.PlotGpsInsRawSyncDataObj.SyncRefInsData[file_name] = obj.DataPreProcess.timeSynchronize(
            obj.Parse100CDataObj.ins100cdf, obj.InsDataDF[file_name], 'time', 'time')
        # if obj.PlotGpsInsRawSyncDataObj.gps_flag[file_name]:
        print(time.strftime('%H:%M:%S', time.localtime()), "GPS和参考数据时间同步...")
        obj.PlotGpsInsRawSyncDataObj.SyncRefGpsData[file_name] = obj.DataPreProcess.timeSynchronize(
            obj.Parse100CDataObj.ins100cdf, obj.GpsDataDF[file_name], 'time', 'itow_pos')

    # TODO: 数据统计
    for scene in time_dict.keys():
        time_arrange = time_dict[scene]['time_arrange']
        scene_dscribe = str(time_dict[scene]['scene_num']) + '_' + time_dict[scene]['scene']
        obj.PlotGpsInsRawSyncDataObj.gps_flag = dict.fromkeys(name_list, 1)
        obj.PlotGpsInsRawSyncDataObj.iniInsGpsBpos()

        if time_dict[scene]['scene'] != '全程':
            print('统计时间范围为%s的数据, 是为场景：%s' % (str(time_arrange), scene_dscribe))
            obj.PlotGpsInsRawSyncDataObj.dataPreStatistics(time_arrange=time_arrange)
            obj.PlotGpsInsRawSyncDataObj.gen_statistics_xlsx(os.getcwd(), time_arrange=time_arrange,
                                                             scene=scene_dscribe)

    # Attention: 配置 GPS 显示 here
    #  TODO
    obj.PlotGpsInsRawSyncDataObj.gps_flag = dict.fromkeys(name_list, 0)
    obj.PlotGpsInsRawSyncDataObj.gps_flag[name_list[0]] = 1
    obj.PlotGpsInsRawSyncDataObj.second_of_week = True

    # 画图
    print(time.strftime('%H:%M:%S', time.localtime()), "INS和参考对比统计画图开始...")
    try:
        obj.PlotGpsInsRawSyncDataObj.PlotRefGpsInsSyncData(os.getcwd(), ref_type='100C')
    except Exception as e:
        print("画图失败...")
        print('ValueError:' + str(e))
    print(time.strftime('%H:%M:%S', time.localtime()), 'over')
