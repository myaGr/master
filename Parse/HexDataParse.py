# -*- coding: UTF-8 -*-
# @Author   : lqw
# @Date     : 2023/12/04

import re  # 正则表达式
import struct  # 十六进制数据解析
import pandas as pd
import os  # 处理文件路径、文件名、创建文件夹等
# from memory_profiler import profile  # 内存占用分析
from scipy.io import savemat  # 存成.mat文件格式
import numpy as np
import csv
import time
import math


class HexDataParse(object):
    def __init__(self):
        # 数据解析
        self.filePath = ""  # 数据文件路径
        self.fileHexData = ""  # 存储从文件里读取的十六进制数据

        self.full_info = ""
        self.imu_freq = 200
        self.vehicle_freq = 50
        self.gps_freq = 10
        self.freq_info = {}

        self.sysTimeFactor = 4000  # 系统时间系数，IMU数据、INS数据里会用到，570D除以1000，个别版本(234平台和377平台)是除以4000\

        self.flags = ['ins', 'gps', 'vehicle', 'imu', 'ins2', 'imu2', 'sync', 'ZeroBias', 'EKFhandle_type', 'sat', 'sat2', 'InsPl', 'GnssPl', 'x_status_ef', 'history_pos_4e']
        self.data_analysis_flag = {'ins': True, 'gps': True, 'vehicle': True, 'imu': True
                                   , 'ins2': False, 'imu2': False,  'sync': False, 'ZeroBias': False, 'EKFhandle_type': False, 'sat': False, 'sat2': False
                                   , 'InsPl': False, 'GnssPl': False, 'x_status_ef': False, 'history_pos_4e': False}

        self.imuDataFrame = None  # IMU数据帧，列表数据类型，元素是十六进制字符串
        self.imuData = pd.DataFrame()  # 解析后的IMU数据，pandas数据类型，这样占用内存更小，由于pandas数据结构添加一行数据效率很低，所以先以字典数据类型收集好数据，然后转成pandas数据结构进行存储

        self.gpsDataFrame = None  # GPS数据帧
        self.gpsData = pd.DataFrame()
        self.bd_len_10 = '{134}'
        self.bd_len_0B = '{110}'

        self.vehicleDataFrame = None  # 车辆数据帧
        self.vehicleData = pd.DataFrame()

        self.insDataFrame = None  # INS数据帧
        self.insData = pd.DataFrame()

        self.syncDataFrame = None  # 同步时间数据帧
        self.syncData = pd.DataFrame()

        self.imu2DataFrame = None  # 2Aimu数据帧
        self.imu2Data = pd.DataFrame()

        self.ins2DataFrame = None  # 1Bins数据
        self.ins2Data = pd.DataFrame()

        self.satelliteDataFrame = None  # 正常：卫星帧  570D：四轮转向帧=30
        self.satelliteData = pd.DataFrame()
        self.satellite2DataFrame = None  # 正常：卫星帧2  570D：四轮转向帧=30
        self.satellite2Data = pd.DataFrame()

        self.EKFhandle_typeFrame = None  # 紧组合调试1
        self.EKFhandle_typeData = pd.DataFrame()
        self.historyPosDataFrame = None
        self.historyPosData = pd.DataFrame()
        self.xStatusDataFrame = None
        self.xStatusData = pd.DataFrame()

        self.zeroBiasDataFrame = None  # 常值零偏估算
        self.zeroBiasData = pd.DataFrame()

        self.InsPlDataFrame = None
        self.InsPlData = pd.DataFrame()
        self.GnssPlDataFrame = None
        self.GnssPlData = pd.DataFrame()

        self.dataFrameStats = {'imuDataNum': 0, 'imuCheckErrIndex': [], 'gpsDataNum': 0, 'gpsCheckErrIndex': [],
                               'vehicleDataNum': 0, 'vehicleCheckErrIndex': [], 'insDataNum': 0, 'insCheckErrIndex': [],
                               'syncDataNum': 0, 'syncCheckErrIndex': [], 'imuFrameHeadNum_bdbd0a': 0,
                               'gpsFrameHeadNum_bdbd10': 0, 'vehicleFrameHeadNum_bdbd20': 0,
                               'insFrameHeadNum_bdbd0b': 0,
                               'syncFrameHeadNum_bdbd0c': 0, 'imu2DataNum': 0, 'imu2CheckErrIndex': [],
                               'ins2DataNum': 0, 'ins2CheckErrIndex': [],
                               'imu2FrameHeadNum_bdbd2a': 0, 'ins2FrameHeadNum_bdbd1b': 0,
                               'satelliteDataNum': 0, 'satelliteCheckErrIndex': [], 'satelliteFrameHeadNum_bdbd30': 0,
                               'satellite2DataNum': 0, 'satellite2CheckErrIndex': [], 'satellite2FrameHeadNum_bdbd31': 0,
                               'InsPlDataNum': 0, 'InsPlErrIndex': [], 'InsPlFrameHeadNum_bdbd0d': 0,
                               'GnssPlDataNum': 0, 'GnssPlErrIndex': [], 'GnssPlFrameHeadNum_bdbd0e': 0

                               }  # DataNum：数据个数，CheckErr：校验错误数据帧的序列号(下标)，FrameHeadNum：帧头数，只匹配帧头得出

        # 数据存储
        self.saveFolderPath = ""  # 存储文件夹路径，初始值要是“”，后面会用此作为判断条件，表明是否使用自定义文件夹
        self.frameHeadStrLst = None  # 各帧头字符串列表
        self.gpsInsTimeIndex = []  # GPS数据系统时间，使用上一帧INS时间，此处记录的是INS时间数据的下标
        self.vehicleInsTimeIndex = []  # 车辆数据系统时间
        self.syncInsTimeIndex = []  # 同步时间数据系统时间
        self.imu2TimeIndex = []  # IMU2时间数据的下标
        self.ins2TimeIndex = []  # INS2时间数据系统时间
        # self.FourWSTimeIndex = None  # 四轮转向时间数据系统时间
        self.satelliteTimeIndex = []  # 四轮转向时间数据系统时间
        self.satellite2TimeIndex = []
        self.EKFhandle_typeIndex = []  # ekfhandle时间数据系统时间
        self.zeroBiasIndex = []  # 常值零偏系统时间

        self.SyncInsGpsData = None  # Gps和INS同步数据
        self.PDataDict = {}  # 轮询表DataFrame结构
        self.SyncDataDF = pd.DataFrame()  # Gps和INS同步数据DataFrame结构
        self.VehicleDataDF = pd.DataFrame()  # 车辆步数据DataFrame结构
        self.GpsDataDF = pd.DataFrame()  # Gps数据DataFrame结构
        self.ImuDataDF = pd.DataFrame()  # IMU数据DataFrame结构
        self.InsDataDF = pd.DataFrame()  # INS数据DataFrame结构
        self.Ins2DataDF = pd.DataFrame()  # odom数据DataFrame结构
        self.InsPlDataDF = pd.DataFrame()  #
        self.GnssPlDataDF = pd.DataFrame()  #

    # 变量初始化
    def variableInit(self):
        self.dataFrameStats['imuDataNum'] = 0
        self.dataFrameStats['imuCheckErrIndex'] = []
        self.dataFrameStats['gpsDataNum'] = 0
        self.dataFrameStats['gpsCheckErrIndex'] = []
        self.dataFrameStats['vehicleDataNum'] = 0
        self.dataFrameStats['vehicleCheckErrIndex'] = []
        self.dataFrameStats['insDataNum'] = 0
        self.dataFrameStats['insCheckErrIndex'] = []
        self.dataFrameStats['syncDataNum'] = 0
        self.dataFrameStats['syncCheckErrIndex'] = []
        self.dataFrameStats['imu2DataNum'] = 0
        self.dataFrameStats['imu2CheckErrIndex'] = []
        self.dataFrameStats['ins2DataNum'] = 0
        self.dataFrameStats['ins2CheckErrIndex'] = []

        self.dataFrameStats['satelliteDataNum'] = 0
        self.dataFrameStats['satelliteCheckErrIndex'] = []
        self.dataFrameStats['satellite2DataNum'] = 0
        self.dataFrameStats['satellite2CheckErrIndex'] = []

        self.dataFrameStats['EKFhandle_typeDataNum'] = 0
        self.dataFrameStats['EKFhandle_typeCheckErrIndex'] = []
        self.dataFrameStats['x_status_data_num'] = 0
        self.dataFrameStats['x_status_ErrIndex'] = []
        self.dataFrameStats['history_pos_data_num'] = 0
        self.dataFrameStats['history_pos_ErrIndex'] = []
        self.dataFrameStats['zeroBiasNum'] = 0
        self.dataFrameStats['zeroBiasErrIndex'] = []

        self.dataFrameStats['InsPlErrIndex'] = []
        self.dataFrameStats['InsPlNum'] = 0
        self.dataFrameStats['GnssPlErrIndex'] = []
        self.dataFrameStats['GnssPlNum'] = 0

        for key in self.flags:
            if key not in self.data_analysis_flag.keys():
                self.data_analysis_flag[key] = False

    # 读取文件数据，以十六进制形式
    def readFileData(self):
        with open(self.filePath, 'rb') as f:
            self.fileHexData = f.read().hex()  # hex()函数：bytes转hex字符串

        # self.fileHexData = self.fileHexData[:int(len(self.fileHexData)/5)]  # 开发测试用

    # 异或校验
    # hexstr：一帧数据包，十六进制字符串，包括末尾校验位
    # 返回值：校验没问题：返回True，校验有问题：返回False
    def myXORCheck(self, hexstr):
        a = 0
        for i in range(len(hexstr))[::2]:
            a ^= int(hexstr[i:i + 2], 16)

        if a == 0:
            return True
        else:
            return False

    #################################### parse data ####################################
    # 解析一帧IMU数据
    # dataHexStr：一帧数据包，十六进制字符串
    # sysTimeFactor：系统时间系数，570D除以1000，个别版本(234平台和377平台)是除以4000
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    def parseIMUDataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次为陀螺仪XYZ轴、加速度XYZ轴、温度、系统时间
        data = list(struct.unpack('<6fhI', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
        data[6] *= 200.0 / 32768.0  # 温度
        data[7] /= sysTimeFactor  # 系统时间
        return data

    # 解析IMU数据帧
    def parseIMUData(self):
        dataDic = {"GyroX": [], "GyroY": [], "GyroZ": [], "AccX": [], "AccY": [], "AccZ": [], "temperature": [],
                   "time": []}
        sorteddataDic = ["GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ", "temperature", "time"]

        for dataPkg in self.imuDataFrame:

            data = self.parseIMUDataOneFrame(dataPkg, self.sysTimeFactor)

            i = 0
            for key in dataDic:
                dataDic[sorteddataDic[i]].append(data[i])
                i += 1

        self.imuDataFrame = None  # 释放内存
        self.imuData = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    # 找到GPS数据帧
    # def findGPSDataFrame(self):
    #     self.gpsDataFrame = re.findall(r'bddb10.{134}', self.fileHexData)  # GPS数据帧头是BD DB 10，包括帧头一共70个字节

    # 解析一帧GPS数据
    # dataHexStr：一帧数据包，十六进制字符串
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    def parseGPSDataOneFrame(self, dataHexStr):
        # if self.myXORCheck(dataHexStr):
        #     encode = '<iHiHihH2B4B9hH4BH3I3BhB'
        # elif self.myXORCheck(dataHexStr[:-8]):
        #     encode = '<iHiHihH2B4B9hH4BH3I2B'
        #     dataHexStr = dataHexStr[:-8]
        if len(dataHexStr[6:]) == 210:
            encode = '<iHiHihH2B4B9hH4BH3I3B2IB5h2B2q'
        elif len(dataHexStr[6:]) == 176:
            encode = '<iHiHihH2B4B9hH4BH3I3B2IB5hB'
        elif len(dataHexStr[6:]) == 152:
            encode = '<iHiHihH2B4B9hH4BH3I3B2I'
        elif len(dataHexStr[6:]) == 142:
            encode = '<iHiHihH2B4B9hH4BH3I4Bh'
            # # 原74字节版本，（包括81字节的）已弃用
            # encode = '<iHiHihH2B4B9hH4BH3I3BhB'
        elif len(dataHexStr[6:]) == 134:
            encode = '<iHiHihH2B4B9hH4BH3I2B'  #32个
        else:
            return []
        data = list(struct.unpack(encode, bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

        utc_year = (int(bytes.fromhex(dataHexStr[2 * 3:-2])[44:46][::-1].hex(), 16) & 0x3f)  # 年
        gps_week = (int(bytes.fromhex(dataHexStr[2 * 3:-2])[44:46][::-1].hex(), 16) >> 6)  # GPS周

        data[0] *= 1e-7  # 经度
        data[1] *= 1e-3
        data[2] *= 1e-7
        data[3] *= 1e-3
        data[4] *= 1e-3
        data[5] *= 1e-3

        data[6] *= 1e-3

        data[13] *= 1e-2  # 水平速度
        data[14] *= 1e-2
        data[15] *= 1e-2
        data[16] *= 1e-3
        data[17] *= 1e-3
        data[18] *= 1e-2
        data[19] *= 1e-3
        data[20] *= 1e-3
        if data[20] > 180:
            data[20] -= 360
        data[21] *= 1e-3
        data[22] = utc_year + 2000
        data[27] *= 1e-3  # 秒
        data[28] *= 1e-3  # tsPos
        data[29] *= 1e-3  # tsVel
        data[30] *= 1e-3  # tsHeading

        if len(dataHexStr[6:]) == 142:
            # 校验位：69-70双字节校验
            data.pop(-3)
            data.pop(-2)
            # aveSNR
            data[-1] *= 1e-2
        elif len(data) > 33:
            data[34] *= 2.5e-4  # week_seconds
            # 校验位
            data.pop(33)

        if len(data) > 36:
            data[37] *= 1e-2  # GDOP
            data[38] *= 1e-2  # PDOP
            data[39] *= 1e-2  # HDOP
            data[40] *= 1e-2  # HTDOP
            data[41] *= 1e-2  # TDOP
            # 校验位
            data.pop(36)
        if len(data) > 43:
            data[44] *= 1e-8  # 高精纬度lat
            data[45] *= 1e-8  # 高精经度lon
            # 校验位
            data.pop(43)

        gps_week += 2000 if gps_week > 0 else 0
        data.append(gps_week)

        return data

    # 解析GPS数据
    def parseGPSData(self):
        if self.bd_len_10 in ['{142}',  '{134}']:
            dataDic = {'Lon': [], 'LonStd': [], 'Lat': [], 'LatStd': [], 'hMSL': [], 'hMSLStd': [], 'SAcc': [],
                       'RtkAge': [], 'gpsFix': [], 'flagsPos': [], 'flagsVel': [], 'flagsHeading': [], 'flagsTime': [],
                       'HSpd': [], 'TrackAngle': [], 'VSpd': [], 'LatencyVel': [], 'BaseLineLength': [],
                       'Heading': [], 'HeadingStd': [], 'Pitch': [], 'PitchStd': [], 'year': [], 'month': [],
                       'day': [], 'hour': [], 'minute': [], 'second': [], 'itow_pos': [], 'itow_vel': [],
                       'itow_heading': [], 'RecMsg': [], 'numSV': [], "aveSNR": [], 'gpsWeek': []}
            # # 原74字节版本，（包括81字节的）已弃用
            # dataDic = {'Lon': [], 'LonStd': [], 'Lat': [], 'LatStd': [], 'hMSL': [], 'hMSLStd': [], 'SAcc': [],
            #            'RtkAge': [], 'gpsFix': [], 'flagsPos': [], 'flagsVel': [], 'flagsHeading': [], 'flagsTime': [],
            #            'HSpd': [], 'TrackAngle': [], 'VSpd': [], 'LatencyVel': [], 'BaseLineLength': [],
            #            'Heading': [], 'HeadingStd': [], 'Pitch': [], 'PitchStd': [], 'year': [], 'month': [],
            #            'day': [], 'hour': [], 'minute': [], 'second': [], 'itow_pos': [], 'itow_vel': [],
            #            'itow_heading': [], 'RecMsg': [], 'numSV': [], "DOP": [], "sub_sat_num": [], 'gpsWeek': []}
        else:
            dataDic = {'Lon': [], 'LonStd': [], 'Lat': [], 'LatStd': [], 'hMSL': [], 'hMSLStd': [], 'SAcc': [],
                       'RtkAge': [], 'gpsFix': [], 'flagsPos': [], 'flagsVel': [], 'flagsHeading': [], 'flagsTime': [],
                       'HSpd': [], 'TrackAngle': [], 'VSpd': [], 'LatencyVel': [], 'BaseLineLength': [],
                       'Heading': [], 'HeadingStd': [], 'Pitch': [], 'PitchStd': [], 'year': [], 'month': [],
                       'day': [], 'hour': [], 'minute': [], 'second': [], 'itow_pos': [], 'itow_vel': [],
                       'itow_heading': [], 'RecMsg': [], 'numSV': [], 'week_seconds': [], 'gpsWeek': [],
                      "GDOP": [], "PDOP": [], "HDOP": [], "HTDOP": [], "TDOP": [], "sub_sat_num": [],
                      'Lat_high_precision': [],  'Lon_high_precision': [], }
        error_index = []

        for dataPkg in self.gpsDataFrame:

            data = self.parseGPSDataOneFrame(dataPkg)
            if not data:
                self.dataFrameStats['gpsCheckErrIndex'].append(self.gpsDataFrame.index(dataPkg))
                error_index.append(self.gpsDataFrame.index(dataPkg))
                continue

            for key in list(dataDic.keys())[:-1]:  # 因最后的GPS week是通过之前的值算出来的
                if list(dataDic.keys()).index(key) < len(data) - 1:
                    dataDic[key].append(data[list(dataDic.keys()).index(key)])
                else:
                    dataDic[key].append(None)
            dataDic['gpsWeek'].append(data[-1])

        self.gpsInsTimeIndex = np.delete(self.gpsInsTimeIndex, error_index)
        self.gpsDataFrame = None  # 释放内存
        self.gpsData = pd.DataFrame(dataDic)

    # 解析一帧车辆数据
    # dataHexStr：一帧数据包，十六进制字符串
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    def parseVehicleDataOneFrame(self, dataHexStr):
        # 数据依次为 按照协议顺序
        data = list(struct.unpack('<2HIB2HIBBIhIB', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

        data[0] *= 0.00863
        data[1] *= 0.00863
        data[4] *= 0.00863
        data[5] *= 0.00863

        data[2] *= 1e-3   # 前轮时间
        data[6] *= 1e-3   # 后轮时间
        data[9] *= 1e-3
        data[10] *= 0.1
        data[11] *= 1e-3

        data.pop(7)  # 删除元素，从后往前删
        data.pop(3)

        return data

    # 解析车辆数据
    def parseVehicleData(self):
        dataDic = {"WheelSpeedFrontLeft": [], "WheelSpeedFrontRight": [], "WheelSpeedFrontTime": [],
                   "WheelSpeedBackLeft": [], "WheelSpeedBackRight": [], "WheelSpeedBackTime": [],
                   "gear": [], "gearTime": [], "WheelAngle": [], "WheelAngleTime": [], "flag": []}  # gear:挡位
        sorteddataDic = ["WheelSpeedFrontLeft", "WheelSpeedFrontRight", "WheelSpeedFrontTime", "WheelSpeedBackLeft",
                         "WheelSpeedBackRight", "WheelSpeedBackTime", "gear", "gearTime", "WheelAngle",
                         "WheelAngleTime", "flag"]  # gear:挡位

        for dataPkg in self.vehicleDataFrame:

            data = self.parseVehicleDataOneFrame(dataPkg)

            i = 0
            for key in dataDic:
                dataDic[sorteddataDic[i]].append(data[i])
                i += 1

        self.vehicleDataFrame = None  # 释放内存
        self.vehicleData = pd.DataFrame(dataDic)

    # 找到INS数据帧
    # def findINSDataFrame(self):
    #     self.insDataFrame = re.findall(r'bddb0b.{110}', self.fileHexData)  # INS数据帧头是BD DB 0B，包括帧头一共58个字节

    # 解析一帧INS数据
    # dataHexStr：一帧数据包，十六进制字符串
    # sysTimeFactor：系统时间系数，570D除以1000，个别版本(234平台和377平台)是除以4000
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    def parseINSDataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次为 按照协议顺序
        # 不定长 110 或 120
        # if self.myXORCheck(dataHexStr):
        #     encode = '<9h3i3hBI2B3hI2BI'
        # elif self.myXORCheck(dataHexStr[:-10]):
        #     encode = '<9h3i3hBI2B3hIB'
        #     dataHexStr = dataHexStr[:-10]
        # else:
        #     return []
        # 定长 110
        if len(dataHexStr[6:]) == 110:
            encode = '<9h3i3hBI2B3hIB'
        elif len(dataHexStr[6:]) == 120:
            encode = '<9h3i3hBI2B3hI2BI'
        else:
            return []
        data = list(struct.unpack(encode, bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

        data[0] *= 360.0 / 32768.0  # roll，横滚角，unit:deg
        data[1] *= 360.0 / 32768.0
        data[2] *= 360.0 / 32768.0
        data[3] *= 300.0 / 32768.0  # 陀螺仪X轴，unit:deg
        data[4] *= 300.0 / 32768.0
        data[5] *= 300.0 / 32768.0
        data[6] *= 12.0 / 32768.0  # 加速度X轴，unit:g
        data[7] *= 12.0 / 32768.0
        data[8] *= 12.0 / 32768.0
        data[9] *= 1e-7  # 纬度，unit:deg
        data[10] *= 1e-7  # unit:deg
        data[11] *= 1e-3  # unit:m
        data[12] *= 100.0 / 32768.0  # 北向速度，unit:m/s
        data[13] *= 100.0 / 32768.0
        data[14] *= 100.0 / 32768.0
        data[22] /= sysTimeFactor

        if len(data) > 24:
            # 校验位
            data.pop(-2)

        return data

    # 解析INS数据
    def parseINSData(self):
        dataDic = {"roll": [], "pitch": [], "yaw": [], "GyroX": [], "GyroY": [], "GyroZ": [], "AccX": [],
                   "AccY": [], "AccZ": [], "lat": [], "lon": [], "height": [], "NorthVelocity": [],
                   "EastVelocity": [], "GroundVelocity": [], "IMUstatus": [], "LEKFstatus": [], "GPSstatus": [],
                   "DebugBin": [], "PData1": [], "PData2": [], "PData3": [], "time": [], "Ptype": [], "gpsWeek": []}
        sorteddataDic = ["roll", "pitch", "yaw", "GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ", "lat", "lon",
                         "height", "NorthVelocity", "EastVelocity", "GroundVelocity", "IMUstatus", "LEKFstatus",
                         "GPSstatus", "DebugBin", "PData1", "PData2", "PData3", "time", "Ptype", "gpsWeek"]  #

        for dataPkg in self.insDataFrame:

            data = self.parseINSDataOneFrame(dataPkg, self.sysTimeFactor)
            if not data:
                self.dataFrameStats['insCheckErrIndex'].append(self.insDataFrame.index(dataPkg))
                continue

            for index in range(len(sorteddataDic)):
                if index < len(data):
                    dataDic[sorteddataDic[index]].append(data[index])
                else:
                    dataDic[sorteddataDic[index]].append(None)

        self.insDataFrame = None
        self.insData = pd.DataFrame(dataDic)

    def parseInsPlDataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次为 按照协议顺序
        encode = '<15H2BI'
        data = list(struct.unpack(encode, bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

        data[0] *= 1e-2  # 纵向pl unit:m
        data[1] *= 1e-2  # 横向pl unit:m
        data[2] *= 1e-2  # 水平pl unit:m
        data[3] *= 1e-2  # 高程pl unit:m
        data[4] *= 1e-2  # 北向pl unit:m
        data[5] *= 1e-2  # 东向pl unit:m

        data[6] *= 1e-2  # 纵向速度pl unit:m/s
        data[7] *= 1e-2  # 横向速度pl unit:m/s
        data[8] *= 1e-2  # 水平速度pl unit:m/s
        data[9] *= 1e-2  # 高程速度pl unit:m/s
        data[10] *= 1e-2  # 北向速度pl unit:m/s
        data[11] *= 1e-2  # 东向速度pl unit:m/s

        data[12] *= 1e-2  # 横滚角姿态pl unit:deg
        data[13] *= 1e-2  # 俯仰角姿态pl unit:deg
        data[14] *= 1e-2  # 航向角姿态pl unit:deg
        # data[15] -- 组合状态
        # data[16] -- GNSS解状态
        data[17] *= 1e-3  # 时间

        # if len(data) > 24:
        #     # 校验位
        #     data.pop(-2)

        return data

    # 解析INS完好性数据
    def parseInsPlData(self):
        dataDic = {'pos_longitudinal_pl': [], 'pos_lateral_pl': [], 'pos_horizontal_pl': [], 'pos_vertical_pl': [], 'pos_north_pl': [], 'pos_east_pl': []
                         , 'vel_longitudinal_pl': [], 'vel_lateral_pl': [], 'vel_horizontal_pl': [], 'vel_vertical_pl': [], 'vel_north_pl': [], 'vel_east_pl': []
                         , 'roll_pl': [], 'pitch_pl': [], 'yaw_pl': [], 'status':[], 'gps_flag_pos': [], 'time':[]}
        for dataPkg in self.InsPlDataFrame:

            data = self.parseInsPlDataOneFrame(dataPkg)

            for key in list(dataDic.keys()):
                dataDic[key].append(data[list(dataDic.keys()).index(key)])

        self.InsPlDataFrame = None
        self.InsPlData = pd.DataFrame(dataDic)

    def parseGnssPlDataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次为 按照协议顺序
        encode = '<3HI'
        try:
            data = list(struct.unpack(encode, bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

            data[0] *= 1e-2  # 纵向pl unit:m
            data[1] *= 1e-2  # 横向pl unit:m
            data[2] *= 1e-2  # 稿pl unit:m

            data[3] *= 1e-3  # 时间
        except Exception as e:
            print(e)
            return None

        return data

    # 解析Gnss完好性数据
    def parseGnssPlData(self):
        dataDic = {'pos_lat_pl': [], 'pos_lon_pl': [], 'pos_vertical_pl': [],  'time':[]}
        for dataPkg in self.GnssPlDataFrame:
            data = self.parseGnssPlDataOneFrame(dataPkg)
            if not data:
                self.dataFrameStats['GnssPlErrIndex'].append(self.GnssPlDataFrame.index(dataPkg))

            for key in list(dataDic.keys()):
                dataDic[key].append(data[list(dataDic.keys()).index(key)])

        self.GnssPlDataFrame = None
        self.GnssPlData = pd.DataFrame(dataDic)

    # 找到同步时间数据帧
    # def findSyncDataFrame(self):
    #     self.syncDataFrame = re.findall(r'bddb0c.{18}', self.fileHexData)  # 同步时间数据帧头是BD DB 0C，包括帧头一共12个字节

    # 解析一帧同步时间数据
    # dataHexStr：一帧数据包，十六进制字符串
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    def parseSyncDataOneFrame(self, dataHexStr):
        # 数据依次为 按照协议顺序
        data = list(struct.unpack('<2I', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

        data[0] *= 1e-3
        data[1] *= 1e-3

        return data

    # 解析同步时间数据
    def parseSyncData(self):
        dataDic = {"imuTime": [], "gpsTime": []}
        sorteddataDic = ["imuTime", "gpsTime"]

        for dataPkg in self.syncDataFrame:

            data = self.parseSyncDataOneFrame(dataPkg)

            i = 0
            for key in dataDic:
                dataDic[sorteddataDic[i]].append(data[i])
                i += 1

        self.syncDataFrame = None
        self.syncData = pd.DataFrame(dataDic)

        # 找到IMU2数据帧

    # def findIMU2DataFrame(self):
    #     self.imuDataFrame = re.findall(r'bddb2a.{62}', self.fileHexData)  # IMU数据帧头是BD DB 0A，包括帧头一共34个字节，一个字节用2个十六进制数表示

    # 解析一帧IMU2数据
    # dataHexStr：一帧数据包，十六进制字符串
    # sysTimeFactor：系统时间系数，570D除以1000，个别版本(234平台和377平台)是除以4000
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列

    def parseIMU2DataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次为陀螺仪XYZ轴、加速度XYZ轴、温度、系统时间
        data = list(struct.unpack('<6fhI', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
        data[6] *= 200.0 / 32768.0  # 温度
        data[7] /= sysTimeFactor  # 系统时间
        return data

        # 解析IMU2数据帧

    def parseIMU2Data(self):
        dataDic = {"GyroX": [], "GyroY": [], "GyroZ": [], "AccX": [], "AccY": [], "AccZ": [], "temperature": [],
                   "time": []}

        for dataPkg in self.imu2DataFrame:

            data = self.parseIMU2DataOneFrame(dataPkg, self.sysTimeFactor)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1

        self.imu2DataFrame = None  # 释放内存
        self.imu2Data = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

        # 找到INS2数据帧

    def findINS2DataFrame(self):
        self.imuDataFrame = re.findall(r'bddb1b.{132}', self.fileHexData)  # INS数据帧头是BD DB 1B，包括帧头一共69个字节，一个字节用2个十六进制数表示

        # 解析一帧INS2数据
        # dataHexStr：一帧数据包，十六进制字符串
        # sysTimeFactor：系统时间系数，570D除以1000，个别版本(234平台和377平台)是除以4000
        # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列

    def parseINS2DataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次当前姿态与初始时刻姿态做差转出的四元数,odom坐标系下的x,y,z,odom坐标系下的线速度（前，左，上),
        # x y z轴分速度的合速,自车坐标系下的三轴角速度、加速度，系统状态，传感器状态，轮询位三字节，轮询标志位，时间
        data = list(struct.unpack('<7i10hBi3h2BI', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
        data[0] *= 1e-9  # 四元数
        data[1] *= 1e-9  # 四元数
        data[2] *= 1e-9  # 四元数
        data[3] *= 1e-9  # 四元数
        data[4] *= 1e-3  # odom坐标系下的x,y,z
        data[5] *= 1e-3
        data[6] *= 1e-3
        data[7] *= 100.0 / 32768.0  # odom坐标系下的线速度，方向同position（前，左，上)，以及 x y z轴分速度的合速度
        data[8] *= 100.0 / 32768.0
        data[9] *= 100.0 / 32768.0
        data[10] *= 100.0 / 32768.0
        data[11] *= 300.0 / 32768.0  # 车坐标系下的角速度
        data[12] *= 300.0 / 32768.0
        data[13] *= 300.0 / 32768.0
        data[14] *= 12.0 / 32768.0  # 车坐标系下的加速度
        data[15] *= 12.0 / 32768.0
        data[16] *= 12.0 / 32768.0
        data[24] /= sysTimeFactor  # 系统时间

        return data

        # 解析INS2数据帧

    def parseINS2Data(self):
        dataDic = {"Quaternion1": [], "Quaternion2": [], "Quaternion3": [], "Quaternion4": [],
                   "OdomX": [], "OdomY": [], "OdomZ": [],
                   "OdomVX": [], "OdomVY": [], "OdomVZ": [], "OdomVM": [],
                   "OdomAngular_VX": [], "OdomAngular_VY": [], "OdomAngular_VZ": [],
                   "OdomAcceleration_X": [], "OdomAcceleration_Y": [], "OdomAcceleration_Z": [],
                   "LiOdometryStatus": [], "sensor_status": [],
                   "temp_P0": [], "temp_P1": [], "temp_P2": [], "Index": [], "frame_id": [], "time": []
                   }
        for dataPkg in self.ins2DataFrame:

            data = self.parseINS2DataOneFrame(dataPkg, self.sysTimeFactor)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1

        self.ins2DataFrame = None  # 释放内存
        self.ins2Data = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    # @staticmethod
    def parseSatelliteDataOneFrame(self, dataHexStr):
        base_info = list(struct.unpack('<2HI2BHIHIHIHIHIHIH2B', bytes.fromhex(dataHexStr[2 * 3:2 * 53])))
        sat_info = []

        if base_info[-1] > 40:
            return base_info, sat_info, base_info[-1]
        if len(dataHexStr) < 6 + 100 + base_info[-1] * 148 + 2:
            print(len(dataHexStr))
            return base_info, sat_info, base_info[-1]

        if not self.myXORCheck(dataHexStr[:6 + 100 + base_info[-1] * 148 + 2]):
            # self.dataFrameStats['satelliteCheckErrIndex'].append(
            #     self.dataFrameStats['satelliteDataNum']+len(self.dataFrameStats['satelliteCheckErrIndex']))
            return base_info, sat_info, base_info[-1]

        sat_encode = '<' + base_info[-1] * '4Bih2H15i'
        if len(dataHexStr) > base_info[-1] * 148 + 106 + 2:
            sat_info = list(struct.unpack(sat_encode, bytes.fromhex(dataHexStr[2 * 53:base_info[-1] * 148 + 106])))
        else:
            return base_info, sat_info, base_info[-1]

        base_info[2] /= 1000  # GPS second of week (s)
        base_info[-3] /= 1000  # epoch interval"(s)
        base_info[5] = base_info[5] + float('0.' + str(base_info[6]))  # lat
        base_info[7] = base_info[7] + float('0.' + str(base_info[8]))  # lon
        base_info[9] = base_info[9] + float('0.' + str(base_info[10]))  # height
        base_info[11] = base_info[11] + float('0.' + str(base_info[12]))  # lat
        base_info[13] = base_info[13] + float('0.' + str(base_info[14]))  # lon
        base_info[15] = base_info[15] + float('0.' + str(base_info[16]))  # height
        base_info_pop_index = [16, 14, 12, 10, 8, 6]
        for index in base_info_pop_index:
            base_info.pop(index)

        for i in range(base_info[-1]):
            # 一帧卫星解出来的长度有 23 位
            sat_info[i * 23 + 4] /= 1e4  # epoch_diff
            sat_info[i * 23 + 5] /= 1e3  # epoch_residuals
            sat_info[i * 23 + 6] /= 1e2  # height_angle
            sat_info[i * 23 + 7] /= 1e2  # azimuth_angle
            sat_info[i * 23 + 8] /= 1e7  # x_unit_vector
            sat_info[i * 23 + 9] /= 1e7  # y_unit_vector
            sat_info[i * 23 + 10] /= 1e7  # z_unit_vector

            sat_info[i * 23 + 11] = sat_info[i * 23 + 11] + sat_info[i * 23 + 12] / 1e8  # cur_sat_pos0
            sat_info[i * 23 + 13] = sat_info[i * 23 + 13] + sat_info[i * 23 + 14] / 1e8  # cur_sat_pos0
            sat_info[i * 23 + 15] = sat_info[i * 23 + 15] + sat_info[i * 23 + 16] / 1e8  # cur_sat_pos0
            sat_info[i * 23 + 17] = sat_info[i * 23 + 17] + sat_info[i * 23 + 18] / 1e8  # cur_sat_pos0
            sat_info[i * 23 + 19] = sat_info[i * 23 + 19] + sat_info[i * 23 + 20] / 1e8  # cur_sat_pos0
            sat_info[i * 23 + 21] = sat_info[i * 23 + 21] + sat_info[i * 23 + 22] / 1e8  # cur_sat_pos0

        sat_info_pop_index = [22, 20, 18, 16, 14, 12]
        for i in range(base_info[-1] - 1, -1, -1):
            for j in sat_info_pop_index:
                sat_info.pop(i * 23 + j)

        return base_info, sat_info, base_info[-1]

        # 解析轮转角数据帧

    def parseSatelliteData(self):
        dataDic = {"message_len": [], "gps_week": [], "gps_week_s": [], "last_status": [], "current_status": [],
                   "last_lat": [], "last_lon": [], "last_height": [], "current_lat": [], "current_lon": [],
                   "current_height": [], "epoch_interval": [], "epoch_mode": [], "satellite_num": []
                   }
        sat_info_dict = {}
        satellite_info = ["satellite_system", "PRN", "dB", "effective", "epoch_diff", "epoch_residuals",
                          "height_angle", "azimuth_angle", "x_unit_vector", "y_unit_vector", "z_unit_vector",
                          "cur_sat_pos0", "cur_sat_pos1", "cur_sat_pos2", "cur_sat_vel0", "cur_sat_vel1",
                          "cur_sat_vel2"]
        # 固定帧长 108=2*3(bddb30) + 2*50(固定字段) + 2*1(校验位)
        # sat_num_max = int((max([len(dataPkg) for dataPkg in self.satelliteDataFrame]) - 108) / 148)
        sat_num_max = 40

        for dataPkg in self.satelliteDataFrame:
            try:
                base_info, sat_info, sat_num = self.parseSatelliteDataOneFrame(dataPkg)
                if not base_info:
                    self.dataFrameStats['satelliteCheckErrIndex'].append(self.satelliteDataFrame.index(dataPkg))
                    continue
                else:
                    self.dataFrameStats['satelliteDataNum'] += 1

            except Exception as e:
                print('无法解析该帧：' + str(e))
                self.dataFrameStats['satelliteCheckErrIndex'].append(self.satelliteDataFrame.index(dataPkg))
                continue

            if len(base_info) == len(dataDic) and len(sat_info) == sat_num * len(satellite_info):
                for key in dataDic.keys():
                    dataDic[key].append(base_info[list(dataDic.keys()).index(key)])

                for sat in range(sat_num_max):
                    for key in satellite_info:
                        if 'sat' + str(sat) + '_' + key not in sat_info_dict.keys():
                            sat_info_dict['sat' + str(sat) + '_' + key] = []

                        if sat < sat_num:
                            # try:
                            sat_info_dict['sat' + str(sat) + '_' + key].append(sat_info[sat * len(satellite_info) + satellite_info.index(key)])
                            # except Exception as e:
                            #     sat_info_dict['sat' + str(sat) + '_' + key].append(None)
                        else:
                            sat_info_dict['sat' + str(sat) + '_' + key].append(None)

        for key in sat_info_dict.keys():
            dataDic[key] = sat_info_dict[key]
        self.satelliteDataFrame = None  # 释放内存
        self.satelliteData = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少
        self.dataFrameStats['satelliteDataNum'] += len(self.dataFrameStats['satelliteCheckErrIndex'])

    def parseSatellite2DataOneFrame(self, dataHexStr):
        base_info = list(struct.unpack('<HIBHIHIHIHIHIHIHIB', bytes.fromhex(dataHexStr[2 * 3:2 * 53])))

        if base_info[-1] > 40:
            return base_info, [], 0
        if len(dataHexStr) < 6 + 100 + base_info[-1] * 200 + 2:
            return base_info, [], 0

        if not self.myXORCheck(dataHexStr[:6 + 100 + base_info[-1] * 200 + 2]):
            # self.dataFrameStats['satelliteCheckErrIndex'].append(
            #     self.dataFrameStats['satelliteDataNum']+len(self.dataFrameStats['satelliteCheckErrIndex']))
            return base_info, [], 0

        sat_encode = '<' + base_info[-1] * '4B8i2H15i'
        sat_info = list(struct.unpack(sat_encode, bytes.fromhex(dataHexStr[2 * 53:base_info[-1] * 200 + 106])))

        base_info[1] /= 1000  # GPS second of week (s)
        base_info[3] = base_info[3] + float('0.' + str(base_info[4]))  # lat
        base_info[5] = base_info[5] + float('0.' + str(base_info[6]))  # lon
        base_info[7] = base_info[7] + float('0.' + str(base_info[8]))  # height
        base_info[9] = base_info[9] + float('0.' + str(base_info[10]))  # receiver_clock_diff0
        base_info[11] = base_info[11] + float('0.' + str(base_info[12]))  # receiver_clock_diff1
        base_info[13] = base_info[13] + float('0.' + str(base_info[14]))  # receiver_clock_diff2
        base_info[15] = base_info[15] + float('0.' + str(base_info[16]))  # receiver_clock_diff3
        base_info_pop_index = [16, 14, 12, 10, 8, 6, 4]
        for index in base_info_pop_index:
            base_info.pop(index)

        for i in range(base_info[-1]):
            sat_info[i * 29 + 12] /= 1e2  # height_angle
            sat_info[i * 29 + 13] /= 1e2  # azimuth_angle
            sat_info[i * 29 + 14] /= 1e7  # x_unit_vector
            sat_info[i * 29 + 15] /= 1e7  # y_unit_vector
            sat_info[i * 29 + 16] /= 1e7  # z_unit_vector

            sat_info[i * 29 + 4] = sat_info[i * 29 + 4] + sat_info[i * 23 + 5] / 1e8  # pseudo_range
            sat_info[i * 29 + 6] = sat_info[i * 29 + 6] + sat_info[i * 23 + 7] / 1e8  # pseudo_noise
            sat_info[i * 29 + 8] = sat_info[i * 29 + 8] + sat_info[i * 23 + 9] / 1e8  # doppler
            sat_info[i * 29 + 10] = sat_info[i * 29 + 10] + sat_info[i * 23 + 11] / 1e8  # doppler_noise

            sat_info[i * 29 + 17] = sat_info[i * 29 + 17] + sat_info[i * 23 + 18] / 1e8  # cur_sat_pos0
            sat_info[i * 29 + 19] = sat_info[i * 29 + 19] + sat_info[i * 23 + 20] / 1e8  # cur_sat_pos1
            sat_info[i * 29 + 21] = sat_info[i * 29 + 21] + sat_info[i * 23 + 22] / 1e8  # cur_sat_pos2
            sat_info[i * 29 + 23] = sat_info[i * 29 + 23] + sat_info[i * 23 + 24] / 1e8  # cur_sat_vel0
            sat_info[i * 29 + 25] = sat_info[i * 29 + 25] + sat_info[i * 23 + 26] / 1e8  # cur_sat_vel1
            sat_info[i * 29 + 27] = sat_info[i * 29 + 27] + sat_info[i * 23 + 28] / 1e8  # cur_sat_vel2

        sat_info_pop_index = [28, 26, 24, 22, 20, 18, 11, 9, 7, 5]
        for i in range(base_info[-1] - 1, -1, -1):
            for j in sat_info_pop_index:
                sat_info.pop(i * 29 + j)

        return base_info, sat_info, base_info[-1]

        # 解析轮转角数据帧

    def parseSatellite2Data(self):
        dataDic = {"message_len": [], "gps_week_s": [], "current_status": [], "current_lat": [], "current_lon": [],
                   "current_height": [], "receiver_clock_diff0": [], "receiver_clock_diff1": [],
                   "receiver_clock_diff2": [], "receiver_clock_diff3": [], "satellite_num": []
                   }
        satellite_info = ["satellite_system", "PRN", "dB", "effective", "pseudo_range", "pseudo_noise", "doppler",
                          "doppler_noise", "height_angle", "azimuth_angle", "x_unit_vector", "y_unit_vector",
                          "z_unit_vector", "cur_sat_pos0", "cur_sat_pos1", "cur_sat_pos2", "cur_sat_vel0",
                          "cur_sat_vel1", "cur_sat_vel2"]

        # 固定帧长 108=2*3(bddb30) + 2*50(固定字段) + 2*1(校验位)
        sat_num_max = int((max([len(dataPkg) for dataPkg in self.satellite2DataFrame]) - 108) / 200)
        sat_info_dict = {}

        for dataPkg in self.satellite2DataFrame:
            try:
                base_info, sat_info, sat_num = self.parseSatellite2DataOneFrame(dataPkg)
                if not base_info:
                    self.dataFrameStats['satellite2CheckErrIndex'].append(self.satellite2DataFrame.index(dataPkg))
                    continue
                else:
                    self.dataFrameStats['satellite2DataNum'] += 1
            except Exception as e:
                print('无法解析该帧：' + str(e))
                self.dataFrameStats['satellite2CheckErrIndex'].append(self.satellite2DataFrame.index(dataPkg))
                continue

            if len(base_info) == len(dataDic) and len(sat_info) == sat_num * len(satellite_info):
                for key in dataDic.keys():
                    dataDic[key].append(base_info[list(dataDic.keys()).index(key)])

                for sat in range(sat_num_max):
                    for key in satellite_info:
                        if 'sat' + str(sat) + '_' + key not in sat_info_dict.keys():
                            sat_info_dict['sat' + str(sat) + '_' + key] = []

                        if sat < sat_num:
                            sat_info_dict['sat' + str(sat) + '_' + key].append(sat_info[sat * len(satellite_info) + satellite_info.index(key)])
                        else:
                            sat_info_dict['sat' + str(sat) + '_' + key].append(None)
            else:
                continue

        for key in sat_info_dict.keys():
            dataDic[key] = sat_info_dict[key]
        self.satellite2DataFrame = None  # 释放内存
        self.satellite2Data = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少
        self.dataFrameStats['satellite2DataNum'] += len(self.dataFrameStats['satellite2CheckErrIndex'])

    # 解析EE帧数据
    def parseEKFhandle_typeOneFrame(self, dataHexStr):
        data = list(struct.unpack('<2I4B', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
        data[0] /= 4000
        return data

    def parseEKFhandle_typeData(self):
        dataDic = {"SysTime": [], "Perrorflag": [], "Obs_Valid_flag": [], "Ekf_update_taskstate": [],
                   "Ekf_update_algostate": [], "Rnum": []
                   }
        for dataPkg in self.EKFhandle_typeFrame:
            data = self.parseEKFhandle_typeOneFrame(dataPkg)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1

        self.EKFhandle_typeFrame = None  # 释放内存
        self.EKFhandle_typeData = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    def zeroBiasDataOneFrame(self, dataHexStr):
        data = list(struct.unpack('<3fB', bytes.fromhex(dataHexStr[2 * 3:-2])))
        data[0] *= 180 / math.pi  # 弧度转角度
        data[1] *= 180 / math.pi
        data[2] *= 180 / math.pi
        return data

    # 估算常值零偏的协议
    def parseZeroBiasData(self):
        dataDic = {'bgx': [], 'bgy': [], 'bgz': [], 'ValidFlag': []}
        for dataPkg in self.zeroBiasDataFrame:
            data = self.zeroBiasDataOneFrame(dataPkg)
            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1
        self.zeroBiasDataFrame = None  # 释放内存
        self.zeroBiasData = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    def parse_x_status_data_one_frame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次为 按照协议顺序
        encode = '<I154f'
        try:
            data0 = list(struct.unpack(encode, bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位
            data = [data0[0] * 2.50e-3, [], [], []] # 系统时间 unit:s
            data[1] = data0[1:8]
            data[2] = data0[8:15]
            data[3] = data0[15:]
        except Exception as e:
            print(e)
            return None

        return data

    # 解析 X状态量异常调试信息数据协议（EF）
    def parse_x_status_data(self):
        dataDic = {'sys_time': [], 'Z': [], 'R': [],  'K':[]}
        for dataPkg in self.xStatusDataFrame:
            data = self.parse_x_status_data_one_frame(dataPkg)
            if not data:
                self.dataFrameStats['x_status_ErrIndex'].append(self.xStatusDataFrame.index(dataPkg))
            else:
                for key in list(dataDic.keys()):
                    dataDic[key].append(data[list(dataDic.keys()).index(key)])

        self.xStatusDataFrame = None
        dataDic['sys_time'] = pd.DataFrame(dataDic['sys_time'], columns=['sys_time'])
        dataDic['Z'], dataDic['R'], dataDic['K'] = pd.DataFrame(dataDic['Z']), pd.DataFrame(dataDic['R']), pd.DataFrame(dataDic['K'])
        self.xStatusData = dataDic

    def parse_history_pos_data_one_frame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次为 按照协议顺序
        encode = '<h3i'
        try:
            data = list(struct.unpack(encode, bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

            data[0] *= 360.0 / 32768.0  # 航向
            data[1] *= 1e-7  # 纬度
            data[2] *= 1e-7  # 经度
            data[3] *= 1e-3  # 高
        except Exception as e:
            print(e)
            return None

        return data

    # 解析 历史位置记忆
    def parse_history_pos_data(self):
        dataDic = {'yaw': [], 'lat': [], 'lon': [], 'alt': []}
        for dataPkg in self.historyPosDataFrame:
            data = self.parse_history_pos_data_one_frame(dataPkg)
            if not data:
                self.dataFrameStats['history_pos_ErrIndex'].append(self.historyPosDataFrame.index(dataPkg))

            for key in list(dataDic.keys()):
                dataDic[key].append(data[list(dataDic.keys()).index(key)])

        self.historyPosDataFrame = None
        self.historyPosData = pd.DataFrame(dataDic)

    # 开始数据解包，总体整合
    # @profile  # 内存分析修饰器，添加这句代码，表明对此函数进行内存分析，内存分析结果会打印输出
    def startParseFileHexData(self):
        self.variableInit()  # 变量初始化
        self.readFileData()
        print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始统计各帧头个数统计")
        self.dataFrameHeadNumStats()  # 各数据帧头个数统计
        print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "找到所有数据帧、获得INS时间下标")
        self.findAllDataFrame()  # 找到所有数据帧、获得INS时间下标
        self.fileHexData = None  # 释放内存

        print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析INS")
        self.parseINSData()
        print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析INS2")
        self.parseINS2Data()
        print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析GPS")
        self.parseGPSData()
        if self.data_analysis_flag['imu']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析IMU")
            self.parseIMUData()
            self.parseIMU2Data()
        if self.data_analysis_flag['vehicle']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析轮速")
            self.parseVehicleData()
        if self.data_analysis_flag['sync']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析同步数据")
            self.parseSyncData()
        if self.data_analysis_flag['EKFhandle_type']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析EKF")
            self.parseEKFhandle_typeData()
        if self.data_analysis_flag['ZeroBias']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析零偏")
            self.parseZeroBiasData()
        if self.data_analysis_flag['InsPl']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析INS完好性")
            self.parseInsPlData()
        if self.data_analysis_flag['GnssPl']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析GNSS完好性")
            self.parseGnssPlData()
        if self.data_analysis_flag['x_status_ef']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析X状态量异常调试信息数据协议")
            self.parse_x_status_data()
        if self.data_analysis_flag['history_pos_4e']:
            print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + "开始解析历史位置记忆")
            self.parse_history_pos_data()
        if self.data_analysis_flag['sat']:
            try:
                print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + '开始解析frame30 ')
                self.parseSatelliteData()
                print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + 'frame30 解析完毕')
            except Exception as e:
                print(e)
        if self.data_analysis_flag['sat2']:
            try:
                print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + '开始解析frame31')
                self.parseSatellite2Data()
                print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + 'frame31 解析完毕')
            except Exception as e:
                print(e)

        # 开启多线程，分别解析各类数据。由于python存在GIL锁，所以python多线程是假多线程，对于CPU计算类，不能提高执行速度，对于I/O操作类，可以提高执行速度
        # t = Thread(target=self.parseIMUData)  # 创建一个线程
        # t.setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
        # t.start()  # 启动线程
        # t.join()  # 等待线程终止，要不然一直挂起

    #################################### save type ####################################

    # @profile  # 内存分析修饰器，添加这句代码，表明对此函数进行内存分析，内存分析结果会打印输出
    def saveDataToDF(self):
        # IMU数据
        self.ImuDataDF = self.imuData
        self.ImuDataDF['gps_ts'] = [self.gpsData['itow_pos'][i] if i < len(self.gpsData) else None
                                for i in self.imuGpsTimeIndex]

        # GPS数据
        self.GpsDataDF = self.gpsData
        self.GpsDataDF['ts'] = [self.insData['time'][i] if i < len(self.insData) else None
                                for i in self.gpsInsTimeIndex]
        self.GpsDataDF['imu_ts'] = [self.imuData['time'][i] if i < len(self.imuData) else None
                                for i in self.gpsImuTimeIndex] if 'time' in self.imuData.keys() else [None]*len(self.GpsDataDF)
        self.GpsDataDF['RecMsgBin'] = self.Unit2Bin(self.gpsData['RecMsg'], 8)

        # vehicle数据
        if self.data_analysis_flag['vehicle']:
            self.VehicleDataDF = self.vehicleData
            self.VehicleDataDF['ts'] = [self.insData['time'][i] if i < len(self.insData) else None
                                        for i in self.vehicleInsTimeIndex]

        # INS数据
        self.InsDataDF = self.insData
        self.InsDataDF['IMUstatusBin'] = self.Unit2Bin(self.insData['IMUstatus'], 8)
        self.InsDataDF['LEKFstatusBin'] = self.Unit2Bin(self.insData['LEKFstatus'], 32)
        self.InsDataDF['GPSstatusBin'] = self.Unit2Bin(self.insData['GPSstatus'], 8)

        # INS2数据
        if self.data_analysis_flag['ins2']:
            self.Ins2DataDF = self.ins2Data
            self.Ins2DataDF['ts'] = [self.insData['time'][i] if i < len(self.insData) else None
                                     for i in self.ins2TimeIndex]

        # Sync数据
        if self.data_analysis_flag['sync']:
            self.SyncDataDF = self.syncData
            self.SyncDataDF['ts'] = [self.insData['time'][i] if i < len(self.insData) else None
                                     for i in self.syncInsTimeIndex]

        # Pdat(字典)
        self.PDataDict = {}
        PData = self.savePDataToMatFile()
        for key in PData:
            self.PDataDict[key] = PData[key].tolist() if type(PData[key]) != list else PData[key]
        # PDataDF = pd.DataFrame.from_dict(self.PDataDict)  # All arrays must be of the same length

    # 将数据存成pkl二进制序列化文件，比csv文件占用存储空间更少
    # data_df：要存的数据，pandas数据类型
    # fileName：文件名，不包括文件后缀名和文件路径
    def dataSavePklFile(self, data_df, fileName):
        newFilePath = os.path.dirname(self.filePath) + '\\' + fileName + '.pkl'
        data_df.to_pickle(newFilePath)

    # 将数据存成csv文件
    # data_df：要存的数据，pandas数据类型
    # fileName：文件名，不包括文件后缀名和文件路径
    def dataSaveCsvFile(self, data_df, fileName):
        newFilePath = os.path.dirname(self.filePath) + '\\SaveDataTest\\' + fileName + '.csv'
        data_df.to_csv(newFilePath, index=False)

    # 开始将GPSINS数据存成.cvs格式
    def SaveAllDataToCsvFile(self):
        dir_path = os.path.split(self.filePath)
        folderPath = dir_path[0] + '/' + dir_path[1][:-4] + '_CsvData'
        if not os.path.exists(folderPath):
            os.mkdir(folderPath)
        self.ImuDataDF.to_csv(folderPath + '/ImuData.csv')
        # # 客户端还需需屏蔽 g a LEKFstatus LEKFstatusBin
        # #  'g': np.array([self.insData['GyroX'], self.insData['GyroY'], self.insData['GyroZ']]).T,
        # #  'a': np.array([self.insData['AccX'], self.insData['AccY'], self.insData['AccZ']]).T,
        # self.InsDataDF = self.InsDataDF.drop(['LEKFstatus', 'LEKFstatusBin','GyroX','GyroY','GyroZ','AccX','AccY','AccZ'], axis=1)
        self.InsDataDF.to_csv(folderPath + '/InsData.csv')
        self.Ins2DataDF.to_csv(folderPath + '/Ins2Data.csv')
        self.GpsDataDF.to_csv(folderPath + '/GpsData.csv')
        self.VehicleDataDF.to_csv(folderPath + '/vehicleData.csv')
        self.SyncDataDF.to_csv(folderPath + '/SyncData.csv')
        self.satelliteData.to_csv(folderPath + '/satelliteData.csv')
        self.satellite2Data.to_csv(folderPath + '/satellite2Data.csv')
        self.EKFhandle_typeData.to_csv(folderPath + '/EKFhandle_typeData.csv')
        self.InsPlData.to_csv(folderPath + '/INS_PL_Data.csv')
        self.GnssPlData.to_csv(folderPath + '/GNSS_PL_Data.csv')

        self.historyPosData.to_csv(folderPath + '/history_Pos_Data.csv')
        writer = pd.ExcelWriter(folderPath + '/x_status_Data.xlsx')
        self.xStatusData['sys_time'].to_excel(writer, sheet_name='sys_time')
        self.xStatusData['Z'].to_excel(writer, sheet_name='Z')
        self.xStatusData['R'].to_excel(writer, sheet_name='R')
        self.xStatusData['K'].to_excel(writer, sheet_name='K')
        writer.save()
        writer.close()

    # 开始将数据存成.mat文件格式，总体整合
    def saveAllDataToMatFile(self):
        veh = self.saveVehicleDataToMatFile() if self.data_analysis_flag['vehicle'] else {}
        sync = self.saveSyncDataToMatFile() if self.data_analysis_flag['vehicle'] else {}
        raw = self.saveImuDataToMatFile() if self.data_analysis_flag['imu'] else {}
        gps = self.saveGpsDataToMatFile() if self.data_analysis_flag['gps'] else {}
        ins = self.saveInsDataToMatFile()

        pdata = self.savePDataToMatFile()
        Imu2Dta = self.saveImu2DataToMatFile() if self.data_analysis_flag['imu2'] else {}
        Ins2Data = self.saveIns2DataToMatFile() if self.data_analysis_flag['ins2'] else {}
        SatelliteData = self.saveSatelliteDataToMatFile() if self.data_analysis_flag['sat'] else {}
        Satellite2Data = self.saveSatellite2DataToMatFile() if self.data_analysis_flag['sat2'] else {}
        EKFhandle_typeData = self.saveEkfHandleDataToMatFile() if self.data_analysis_flag['EKFhandle_type'] else {}
        ZeroBiasData = self.saveZeroBiasDataToMatFile() if self.data_analysis_flag['ZeroBias'] else {}
        InsPlData = self.saveInsPlDataDataToMatFile() if self.data_analysis_flag['InsPl'] else {}
        GnssPlData = self.saveGnssPlDataDataToMatFile() if self.data_analysis_flag['GnssPl'] else {}

        history_pos_data = self.save_history_pos_data2mat() if self.data_analysis_flag['history_pos_4e'] else {}
        x_status_data = self.save_x_status_data2mat() if self.data_analysis_flag['x_status_ef'] else {}

        ins['Pdata'] = pdata
        matDic = {'GPSINSData': {'RawData': raw, 'SyncData': sync, 'GPSData': gps, 'INSData': ins,
                                 'EnvisionCanData': veh, 'Imu2Dta': Imu2Dta, 'Ins2Data': Ins2Data,
                                 'SatelliteData': SatelliteData, 'Satellite2Data': Satellite2Data,
                                 'EKFhandle_type': EKFhandle_typeData, 'ZeroBiasData': ZeroBiasData,
                                 'inr': InsPlData, 'GNSS_PL': GnssPlData,
                                 'history_pos':history_pos_data, 'x_status':x_status_data
                                 }}
        dir_path = os.path.split(self.filePath)
        newFilePath = dir_path[0] + '/' + dir_path[1][:-4] + '_GPSINSData.mat'
        print(time.strftime('%H:%M:%S', time.localtime()), "保存路径为：", newFilePath)
        savemat(newFilePath, matDic, do_compression=True)  # 存成.mat文件格式

    # 将数据存成.mat文件格式
    # IMU数据
    def saveImuDataToMatFile(self):
        matDic = dict.fromkeys(['g', 'a', 'temp', 'ts'], [])
        if len(self.imuData) == 0:
            return matDic
        matDic = {'g': np.array([self.imuData['GyroX'], self.imuData['GyroY'], self.imuData['GyroZ']]).T,
                  'a': np.array([self.imuData['AccX'], self.imuData['AccY'], self.imuData['AccZ']]).T,
                  'temp': np.array([self.imuData['temperature']]).T, 'ts': np.array([self.imuData['time']]).T}
        return matDic

    # GPS数据
    def saveGpsDataToMatFile(self):
        mat_key = ['Lon', 'LonStd', 'Lat', 'LatStd', 'hMSL', 'hMSLStd', 'SAcc', 'RtkAge'
            , 'gpsFix', 'HSpd', 'Trk', 'VSpd', 'LatencyVel', 'BaseLineLength'
            , 'heading', 'cAcc', 'pitch', 'pitchStd', 'RecMsg', 'RecMsgBin'
            , 'NumSV', 't', 'gpsWeek', 'itow_pos', 'itow_vel', 'itow_heading'
            , 'ts', 'flagsPos', 'flagsVel', 'flagsHeading', 'flagsTime']
        if len(self.gpsData) == 0:
            return dict.fromkeys(mat_key, [])
        gpsInsTime = [self.insData['time'][i] if i < len(self.insData) else 0 for i in self.gpsInsTimeIndex]
        # gpsInsTime = [self.insData['time'][i] for i in self.gpsInsTimeIndex] if len(self.insData) != 0 else []

        matDic = {'Lon': np.array([self.gpsData['Lon']]).T, 'LonStd': np.array([self.gpsData['LonStd']]).T,
                  'Lat': np.array([self.gpsData['Lat']]).T, 'LatStd': np.array([self.gpsData['LatStd']]).T,
                  'hMSL': np.array([self.gpsData['hMSL']]).T, 'hMSLStd': np.array([self.gpsData['hMSLStd']]).T,
                  'SAcc': np.array([self.gpsData['SAcc']]).T,
                  'RtkAge': np.array([self.gpsData['RtkAge']]).T,
                  'gpsFix': np.float64(np.array([self.gpsData['gpsFix']]).T),
                  'HSpd': np.array([self.gpsData['HSpd']]).T, 'Trk': np.array([self.gpsData['TrackAngle']]).T,
                  'VSpd': np.array([self.gpsData['VSpd']]).T, 'LatencyVel': np.array([self.gpsData['LatencyVel']]).T,
                  'BaseLineLength': np.array([self.gpsData['BaseLineLength']]).T,
                  'heading': np.array([self.gpsData['Heading']]).T, 'cAcc': np.array([self.gpsData['HeadingStd']]).T,
                  'pitch': np.array([self.gpsData['Pitch']]).T, 'pitchStd': np.array([self.gpsData['PitchStd']]).T,
                  'RecMsg': np.float64(np.array([self.gpsData['RecMsg']]).T),
                  'RecMsgBin': self.Unit2Bin(self.gpsData['RecMsg'], 8),
                  'NumSV': np.float64(np.array([self.gpsData['numSV']]).T),
                  't': np.array([self.gpsData['year'], self.gpsData['month'], self.gpsData['day'], self.gpsData['hour'],
                                 self.gpsData['minute'], self.gpsData['second']]).T,
                  'gpsWeek': np.array([self.gpsData['gpsWeek']]).T,
                  'itow_pos': np.array([self.gpsData['itow_pos']]).T,
                  'itow_vel': np.array([self.gpsData['itow_vel']]).T,
                  'itow_heading': np.array([self.gpsData['itow_heading']]).T,
                  'ts': np.array([gpsInsTime]).T,
                  'aveSNR': np.array([self.gpsData['aveSNR']]).T if self.gpsData['aveSNR'][0] is not None else np.array([[0] * len(self.gpsData['aveSNR'])]),

                  # 'flags': np.float64(np.array([self.gpsData['flags']]).T),  # 转成下面的:
                  'flagsPos': np.float64(np.array([self.gpsData['flagsPos']]).T),
                  'flagsVel': np.float64(np.array([self.gpsData['flagsVel']]).T),
                  'flagsHeading': np.float64(np.array([self.gpsData['flagsHeading']]).T),
                  'flagsTime': np.float64(np.array([self.gpsData['flagsTime']]).T)
                  }

        return matDic

    # 车辆数据
    def saveVehicleDataToMatFile(self):
        if len(self.vehicleData) == 0 or not self.data_analysis_flag['vehicle']:
            return dict.fromkeys(['ts', 'tsWheelAngle', 'WheelAngle', 'VehSpdDriL', 'VehSpdDriR', 'tsVehSpdDri',
                                  'VehSpdNonDriL', 'VehSpdNonDriR', 'tsVehSpdNonDri', 'Shifter', 'tsShifter', 'flag']
                                 , [])

        # vehicleInsTime = [self.insData['time'][i] for i in self.vehicleInsTimeIndex] if len(self.insData) != 0 else []
        vehicleInsTime = [self.insData['time'][i] if i < len(self.insData) else None for i in self.vehicleInsTimeIndex]
        VehSpdDriL = np.array([self.vehicleData['WheelSpeedFrontLeft']]).T
        VehSpdDriR = np.array([self.vehicleData['WheelSpeedFrontRight']]).T
        VehSpdNonDriL = np.array([self.vehicleData['WheelSpeedBackLeft']]).T
        vehicleData = np.array([self.vehicleData['WheelSpeedBackRight']]).T

        matDic = {'ts': np.array([vehicleInsTime]).T,
                  'tsWheelAngle': np.array([self.vehicleData['WheelAngleTime']]).T,
                  'WheelAngle': np.array([self.vehicleData['WheelAngle']]).T,
                  'VehSpdDriL': VehSpdDriL.astype(np.float64),
                  'VehSpdDriR': VehSpdDriR.astype(np.float64),
                  'tsVehSpdDri': np.array([self.vehicleData['WheelSpeedFrontTime']]).T,
                  'VehSpdNonDriL': VehSpdNonDriL.astype(np.float64),
                  'VehSpdNonDriR': vehicleData.astype(np.float64),
                  'tsVehSpdNonDri': np.array([self.vehicleData['WheelSpeedBackTime']]).T,
                  'Shifter': np.float64(np.array([self.vehicleData['gear']]).T),
                  'tsShifter': np.array([self.vehicleData['gearTime']]).T,
                  'flag': np.float64(np.array([self.vehicleData['flag']]).T)}
        return matDic

    # INS数据
    def saveInsDataToMatFile(self):
        mat_keys = ['angle', 'g', 'a', 'pos', 'v', 'IMUstatus'
            , 'IMUstatusBin', 'LEKFstatus', 'LEKFstatusBin'
            , 'GPSstatus', 'GPSstatusBin', 'DebugBin', 'P', 'ts', 'Ptype']
        if len(self.insData) == 0:
            return dict.fromkeys(mat_keys, [])
        gpsWeek = np.array([self.insData['gpsWeek']]).T if self.insData['gpsWeek'][0] != None else np.array([[0]*len(self.insData['gpsWeek'])])
        matDic = {'angle': np.array([self.insData['roll'], self.insData['pitch'], self.insData['yaw']]).T,
                  'g': np.array([self.insData['GyroX'], self.insData['GyroY'], self.insData['GyroZ']]).T,  # 客户端需屏蔽
                  'a': np.array([self.insData['AccX'], self.insData['AccY'], self.insData['AccZ']]).T,  # 客户端需屏蔽
                  'pos': np.array([self.insData['lat'], self.insData['lon'], self.insData['height']]).T,
                  'v': np.array(
                      [self.insData['NorthVelocity'], self.insData['EastVelocity'], self.insData['GroundVelocity']]).T,
                  'IMUstatus': np.float64(np.array([self.insData['IMUstatus']]).T),
                  'IMUstatusBin': self.Unit2Bin(self.insData['IMUstatus'], 8),
                  'LEKFstatus': np.float64(np.array([self.insData['LEKFstatus']]).T),  # 客户端需屏蔽
                  'LEKFstatusBin': self.Unit2Bin(self.insData['LEKFstatus'], 32),  #客户端需屏蔽
                  'GPSstatus': np.float64(np.array([self.insData['GPSstatus']]).T),
                  'GPSstatusBin': self.Unit2Bin(self.insData['GPSstatus'], 8),
                  'DebugBin': self.Unit2Bin(self.insData['DebugBin'], 8),
                  'P': np.float64(np.array([self.insData['PData1'], self.insData['PData2'], self.insData['PData3']]).T),
                  'ts': np.array([self.insData['time']]).T, 'Ptype': np.float64(np.array([self.insData['Ptype']]).T),
                  'gpsWeek': gpsWeek}
        return matDic

    # 同步时间数据
    def saveSyncDataToMatFile(self):
        if len(self.syncData) == 0:
            return dict.fromkeys(['timu', 'tgps', 'ts'], [])
        # if len(self.insData) != 0:
        #     syncInsTime = [self.insData['time'][i] for i in self.syncInsTimeIndex]
        syncInsTime = [self.insData['time'][i] if i < len(self.insData) else None for i in self.syncInsTimeIndex]

        matDic = {'timu': np.float64(np.array([self.syncData['imuTime']]).T),
                  'tgps': np.float64(np.array([self.syncData['gpsTime']]).T),
                  'ts': np.float64(np.array([syncInsTime]).T)}
        return matDic

    # IMU2数据
    def saveImu2DataToMatFile(self):
        if len(self.imu2Data) == 0:
            return dict.fromkeys(['g', 'a', 'temp', 'ts'], [])
        # 为保证生成mat文件中的值均为double格式，在类似temp的值中转换成 np.float64
        matDic = {'g': np.float64(np.array([self.imu2Data['GyroX'], self.imu2Data['GyroY'], self.imu2Data['GyroZ']]).T),
                  'a': np.float64(np.array([self.imu2Data['AccX'], self.imu2Data['AccY'], self.imu2Data['AccZ']]).T),
                  'temp': np.float64(np.array([self.imu2Data['temperature']]).T),
                  'ts': np.array([self.imu2Data['time']]).T}
        return matDic

    # INS数据
    def saveIns2DataToMatFile(self):
        if len(self.ins2Data) == 0:
            return dict.fromkeys(['Quaternion', 'Odom_Position', 'Odom_V', 'Odom_AngV', 'Odom_Acc'
                                     , 'LiOdometryStatu', 'sensor_status', 'temp_P', 'Index', 'frame_id', 'time']
                                 , [])
        # 为保证生成mat文件中的值均为double格式，在类似status的值中转换成 np.float64
        matDic = {'Quaternion': np.array(
            [self.ins2Data['Quaternion1'], self.ins2Data['Quaternion2'], self.ins2Data['Quaternion3'],
             self.ins2Data['Quaternion4']]).T,
                  'Odom_Position': np.array([self.ins2Data['OdomX'], self.ins2Data['OdomY'], self.ins2Data['OdomZ']]).T,
                  'Odom_V': np.array([self.ins2Data['OdomVX'], self.ins2Data['OdomVY'], self.ins2Data['OdomVZ'],
                                      self.ins2Data['OdomVM']]).T,
                  'Odom_AngV': np.array([self.ins2Data['OdomAngular_VX'], self.ins2Data['OdomAngular_VY'],
                                         self.ins2Data['OdomAngular_VZ']]).T,
                  'Odom_Acc': np.array([self.ins2Data['OdomAcceleration_X'], self.ins2Data['OdomAcceleration_Y'],
                                        self.ins2Data['OdomAcceleration_Z']]).T,
                  'LiOdometryStatu': np.float64(np.array([self.ins2Data['LiOdometryStatus']]).T),
                  'sensor_status': np.float64(np.array([self.ins2Data['sensor_status']]).T),
                  'temp_P': np.array([self.ins2Data['temp_P0'], self.ins2Data['temp_P1'], self.ins2Data['temp_P2']]).T,
                  'Index': np.float64(np.array([self.ins2Data['Index']]).T),
                  'frame_id': np.float64(np.array([self.ins2Data['frame_id']]).T),
                  'ts': np.array([self.Ins2DataDF['ts']]).T,
                  'time': np.array([self.ins2Data['time']]).T
                  }
        return matDic

    def saveSatelliteDataToMatFile(self):
        matDic = dict.fromkeys(['base_info', 'sat_info'], [])
        if not self.satelliteData.empty:
            matDic = {
                'base_info': np.float64(np.array([self.satelliteData[i] for i in list(self.satelliteData.keys())[:14]]).T),
                'sat_info': np.float64(np.array([self.satelliteData[i] for i in list(self.satelliteData.keys())[14:]]).T)}
        return matDic

    def saveSatellite2DataToMatFile(self):
        matDic = dict.fromkeys(['base_info', 'sat_info'], [])
        if not self.satellite2Data.empty:
            matDic = {
                'base_info': np.float64(np.array([self.satellite2Data[i] for i in list(self.satellite2Data.keys())[:11]]).T),
                'sat_info': np.float64(np.array([self.satellite2Data[i] for i in list(self.satellite2Data.keys())[11:]]).T)}
        return matDic

    # 紧组合EE数据
    def saveEkfHandleDataToMatFile(self):
        if len(self.EKFhandle_typeData) == 0:
            return dict.fromkeys(
                ['SysTime', 'Perrorflag', 'Obs_Valid_flag', 'Ekf_update_taskstate', 'Ekf_update_algostate', 'Rnum']
                , [])
        matDic = {'SysTime': np.array([self.EKFhandle_typeData['SysTime']]).T,
                  'Perrorflag': self.Unit2Bin(self.EKFhandle_typeData['Perrorflag'], 32),
                  'Obs_Valid_flag': np.array([self.EKFhandle_typeData['Obs_Valid_flag']]).T,
                  'Ekf_update_taskstate': np.array([self.EKFhandle_typeData['Ekf_update_taskstate']]).T,
                  'Ekf_update_algostate': np.array([self.EKFhandle_typeData['Ekf_update_algostate']]).T,
                  'Rnum': np.array([self.EKFhandle_typeData['Rnum']]).T}
        return matDic

    # 零偏数据
    def saveZeroBiasDataToMatFile(self):
        if len(self.zeroBiasData) == 0:
            return dict.fromkeys(['bgx', 'bgy', 'bgz', 'ValidFlag'], [])
        matDic = {'bgx': np.array([self.zeroBiasData['bgx']]).T, 'bgy': np.array([self.zeroBiasData['bgy']]).T,
                  'bgz': np.array([self.zeroBiasData['bgz']]).T,
                  'ValidFlag': np.array([self.zeroBiasData['ValidFlag']]).T}
        return matDic

    # 轮循表数据
    def savePDataToMatFile(self):
        mat_key = ['Xp', 'Xv', 'Xatt', 'Xba', 'Xbg', 'Align', 'Xalign', 'Lbbg', 'Xlbbg', 'Lbbc', 'Xlbbc', 'Ba'
            , 'Bg', 'Atttg', 'XAtttg', 'Lbgc', 'Attbg', 'StdAtttb', 'Dpos', 'Dvel', 'Tsover', 'Tsover2'
            , 'DParkingpos', 'Pp', 'Pv', 'Patt', 'Pba', 'Pbg', 'Palign', 'Plbbg', 'Plbbc', 'PAtttg'
            , 'Pkws', 'StdAtttg', 'SUIDAOTIME', 'StdLttg', 'PpT', 'PvT', 'PattT', 'PbaT', 'PbgT', 'XpT'
            , 'XvT', 'XattT', 'XbaT', 'XbgT', 'AlignT', 'XalignT', 'PalignT', 'TimeGPS2IMUT'
            , 'CalibrationFlagT', 'TimeLekfUpdateT', 'LbbgT', 'XlbbgT', 'PlbbgT', 'LbbcT', 'XlbbcT'
            , 'PlbbcT', 'BaT', 'BgT', 'TempT', 'Gmean_Gvar_AvarT', 'AtttgT', 'PAtttgT', 'XAtttgT', 'KwsT'
            , 'PkwsT', 'XkwsT', 'LbgcT', 'AttbgT', 'TrisQualityT', 'nStateT', 'Flag34T', 'State35T'
            , 'StdAtttbT', 'StdAtttgT', 'SUIDAOTIMET', 'StdLttgT', 'StdKwsT', 'TimeT', 'DposT', 'DvelT'
            , 'VelbT', 'TsoverT', 'Tsover2T', 'RposT', 'stdPosT', 'DposHpPpHT', 'TimeEdlayT'
            , 'DParkingposT', 'TimeGPS2IMU', 'CalibrationFlag', 'TimeLekfUpdate', 'Temp', 'TimeNoW'
            , 'TimeNoRTKINT', 'Gmean', 'TrisAbsmaxdposPspeed', 'TrisQuality', 'TrisMaxdpos', 'StateGPS'
            , 'StateKWS', 'Ppostemp', 'State35', 'DposHorizontal', 'temptime', 'TimeNoPOS', 'tsOverRTKInt'
            , 'tsOverVelb', 'Ra1', 'rabs', 'DposHpPpH', 'TimeGPS2IMUPos', 'TimeGPS2IMUHeading', 'TimeGPS2IMUCarVel'
            , 'Avar', 'Gvar', 'Kws', 'Xkws', 'Flag34', 'StdKws', 'Velb', 'Rpos', 'stdPos']
        matDic = dict.fromkeys(mat_key, [])
        if len(self.insData) == 0:
            return matDic

        p = np.array([self.insData['PData1'], self.insData['PData2'], self.insData['PData3'], self.insData['Ptype'],
                      self.insData['time']]).T

        unite_key = ['Xp', 'Xv', 'Xatt', 'Xba', 'Xbg', 'Align', 'Xalign', 'Lbbg', 'Xlbbg', 'Lbbc', 'Xlbbc', 'Ba', 'Bg',
                     'Atttg', 'XAtttg', 'Lbgc', 'Attbg', 'StdAtttb', 'Dpos', 'Dvel', 'Tsover', 'Tsover2', 'DParkingpos',
                     'Rvel3', 'DParkingvel', 'Attng']
        unite_num = [5, 6, 7, 8, 9, 10, 12, 14, 16, 17, 19, 21, 20, 24, 26, 30, 31, 36, 44, 45, 47, 48, 54, 55, 56, 59]
        unite_form = [1 / 1000.0, 1 / 32768, 1 / 32768 * 360.0 / 100, 1 / 32768 * 0.1 / 100, 1 / 32768.0 / 100,
                      1 / 32768.0 * 360, 1 / 32768.0 * 360.0 / 100, 1.00E-03, 1.00E-04, 1.00E-03, 1.00E-04, 0.1 / 32768,
                      1 / 32768, 1 / 32768.0 * 360, 1 / 32768.0 * 360 / 100, 1.00E-03, 1 / 32768.0 * 360, 1, 1, 1e-3,
                      1.00E-03, 1.00E-03, 1, 1.00E-02, 1.00E-03, 1 / 32768.0 * 360]
        for i in range(len(unite_key)):
            matDic[unite_key[i]] = p[np.where(p[:, 3] == unite_num[i])][:, 0:3] * unite_form[i]

        exp_key = ['Pp', 'Pv', 'Patt', 'Pba', 'Pbg', 'Palign', 'Plbbg', 'Plbbc', 'PAtttg', 'Pkws', 'StdAtttg',
                   'SUIDAOTIME', 'StdLttg']
        exp_num = [0, 1, 2, 3, 4, 11, 15, 18, 25, 28, 40, 41, 42]
        for i in range(len(exp_key)):
            matDic[exp_key[i]] = np.exp(p[np.where(p[:, 3] == exp_num[i])][:, 0:3] / 100)

        time_key = ['PpT', 'PvT', 'PattT', 'PbaT', 'PbgT', 'XpT', 'XvT', 'XattT', 'XbaT', 'XbgT', 'AlignT', 'XalignT',
                    'PalignT', 'TimeGPS2IMUT', 'CalibrationFlagT', 'TimeLekfUpdateT', 'LbbgT', 'XlbbgT', 'PlbbgT',
                    'LbbcT', 'XlbbcT', 'PlbbcT', 'BaT', 'BgT', 'TempT', 'Gmean_Gvar_AvarT', 'AtttgT', 'PAtttgT',
                    'XAtttgT', 'KwsT', 'PkwsT', 'XkwsT', 'LbgcT', 'AttbgT', 'TrisQualityT', 'nStateT', 'Flag34T',
                    'State35T', 'StdAtttbT', 'StdAtttgT', 'SUIDAOTIMET', 'StdLttgT', 'StdKwsT', 'TimeT', 'DposT',
                    'DvelT', 'VelbT', 'TsoverT', 'Tsover2T', 'RposT', 'stdPosT', 'DposHpPpHT', 'TimeEdlayT',
                    'DParkingposT', 'RvelT3', 'DParkingvelT', 'TimeEdlayParkingT', 'cntErrorT', 'AttngT', 'cnt98T',
                    'cnt99T']
        time_num = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 11, 13, 13, 13, 14, 16, 15, 17, 19, 18, 21, 20, 22, 23, 24,
                    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 40, 41, 42, 43, 49, 44, 45, 46, 47, 48, 52, 53, 50,
                    51, 54, 55, 56, 57, 58, 59, 98, 99]
        for i in range(len(time_key)):
            matDic[time_key[i]] = np.array([p[np.where(p[:, 3] == time_num[i])][:, 4]]).T

        single_key = ['TimeGPS2IMU', 'CalibrationFlag', 'TimeLekfUpdate', 'Temp', 'TimeNoW', 'TimeNoRTKINT', 'Gmean',
                      'TrisAbsmaxdposPspeed', 'TrisQuality', 'TrisMaxdpos', 'StateGPS', 'StateKWS', 'Ppostemp',
                      'State35', 'DposHorizontal', 'temptime', 'TimeNoPOS', 'tsOverRTKInt', 'tsOverVelb', 'Ra1', 'rabs',
                      'DposHpPpH', 'TimeGPS2IMUPos', 'TimeGPS2IMUHeading', 'TimeGPS2IMUCarVel', 'TimeParking2IMUPos3',
                      'TimeParking2IMUVel3', 'cntError', 'cntUnknown', 'tempcnt', 'cntCount0', 'cntReset', 'cntByte',
                      'cntError', 'cntPerHz']
        single_num = [13, 13, 13, 22, 22, 22, 23, 32, 32, 32, 33, 33, 33, 35, 35, 35, 49, 49, 49, 50, 50, 50, 51, 51,
                      51, 57, 57, 58, 58, 58, 98, 98, 99, 99, 99]
        single_id = [0, 1, 2, 0, 1, 2, 0, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 0, 1, 2, 0, 1, 0,
                     1, 2]
        single_form = [1.00E-03, 1, 1.00E-05, 200.0 / 32768, 1.00E-03, 1, 1 / 100.0, 0.001, 1, 0.001, 1, 1, 0.01, 1,
                       0.01, 1, 1, 1.00E-03, 1.00E-03, 1.00E-05, 1.00E-01, 1.00E-02, 1.00E-03, 1.00E-03, 1.00E-03,
                       1.00E-03, 1.00E-03, 1, 1, 1, 1, 1, 1, 1, 1]
        for i in range(len(single_key)):
            matDic[single_key[i]] = np.array(
                [p[np.where(p[:, 3] == single_num[i])][:, single_id[i]] * single_form[i]]).T

        matDic['Avar'] = np.array([np.exp(p[np.where(p[:, 3] == 23)][:, 2] / 1000)]).T
        matDic['Gvar'] = np.exp(p[np.where(p[:, 3] == 23)][:, 1] / 1000)
        matDic['Kws'] = np.array([[p[np.where(p[:, 3] == 27)][:, 0] * 1e-5, p[np.where(p[:, 3] == 27)][:, 1] * 1e-5,
                                   p[np.where(p[:, 3] == 27)][:, 2] * 1e-3]]).T
        matDic['Xkws'] = np.array([[p[np.where(p[:, 3] == 29)][:, 0] * 1e-5, p[np.where(p[:, 3] == 29)][:, 1] * 1e-5,
                                    p[np.where(p[:, 3] == 29)][:, 2] * 1e-3]]).T
        matDic['Flag34'] = np.round(p[np.where(p[:, 3] == 34)][:, 0:3])
        matDic['StdKws'] = np.array(
            [[np.exp(p[np.where(p[:, 3] == 43)][:, 0] / 100), np.exp(p[np.where(p[:, 3] == 43)][:, 1] / 100)]]).T
        matDic['Velb'] = p[np.where(p[:, 3] == 46)][:, 0:2] * 1e-3
        matDic['Rpos'] = np.array([[p[np.where(p[:, 3] == 52)][:, 0] * 1e-10, p[np.where(p[:, 3] == 52)][:, 1] * 1e-10,
                                    p[np.where(p[:, 3] == 52)][:, 2] * 1e-3]]).T
        matDic['stdPos'] = np.array([[p[np.where(p[:, 3] == 53)][:, 0] * 1e-2, p[np.where(p[:, 3] == 53)][:, 1] * 1e-2,
                                      p[np.where(p[:, 3] == 53)][:, 2] * 1e-3]]).T

        # 删除无值的项
        rm = []
        for key in matDic:
            if len(matDic[key]) == 0:
                rm.append(key)
        for i in rm:
            matDic.pop(i)
        return matDic

    def saveInsPlDataDataToMatFile(self):
        if len(self.InsPlData) == 0:
            return dict.fromkeys(['pos_pl', 'vel_pl', 'att_pl', 'status', 'time'], [])
        matDic = {'pos_pl': np.array([self.InsPlData['pos_longitudinal_pl'], self.InsPlData['pos_lateral_pl'], self.InsPlData['pos_horizontal_pl']
                                         , self.InsPlData['pos_vertical_pl'], self.InsPlData['pos_north_pl'], self.InsPlData['pos_east_pl']]).T,
                  'vel_pl': np.array([self.InsPlData['vel_longitudinal_pl'], self.InsPlData['vel_lateral_pl'], self.InsPlData['vel_horizontal_pl']
                                         , self.InsPlData['vel_vertical_pl'], self.InsPlData['vel_north_pl'], self.InsPlData['vel_east_pl']]).T,
                  'att_pl': np.array([self.InsPlData['roll_pl'], self.InsPlData['pitch_pl'], self.InsPlData['yaw_pl']]).T,
                  'status': np.array([self.InsPlData['status']]).T,'time': np.array([self.InsPlData['time']]).T}
        return matDic

    def saveGnssPlDataDataToMatFile(self):
        if len(self.GnssPlData) == 0:
            return dict.fromkeys(['pos_pl'], [])
        matDic = {'pos_pl': np.array([self.GnssPlData['time']
                                         , self.GnssPlData['pos_lat_pl'], self.GnssPlData['pos_lon_pl'], self.GnssPlData['pos_vertical_pl']]).T}
        return matDic

    def save_history_pos_data2mat(self):
        if len(self.historyPosData) == 0:
            return dict.fromkeys(['yaw', 'lat', 'lon', 'alt'], [])
        matDic = {'yaw': np.array([self.historyPosData['yaw']]).T
            , 'lat': np.array([self.historyPosData['lat']]).T, 'lon': np.array([self.historyPosData['lon']]).T, 'alt': np.array([self.historyPosData['alt']]).T}
        return matDic

    def save_x_status_data2mat(self):
        if len(self.xStatusData) == 0:
            return dict.fromkeys(['sys_time', 'Z', 'R',  'K'], [])
        matDic = {'sys_time': np.array([self.xStatusData['sys_time']]).T
            , 'Z': np.array([self.xStatusData['Z']]).T, 'R': np.array([self.xStatusData['R']]).T, 'K': np.array([self.xStatusData['K']]).T}
        return matDic

    #################################### analyze data ####################################
    def findHexLength(self, head, len_lists):
        """
        检查帧的长短
        :param head: 'bddb10'
        :param len_lists: [最大值, 次之, ...] 除去 bddbxx 六位的长度
        :return: '{142}''
        """
        if not len_lists:
            return '{0}'
        len_count_dict = dict.fromkeys(len_lists, 0)

        frames = re.findall(r''+head+'.{'+str(len_lists[0])+'}', self.fileHexData)
        for frame in frames:
            this_head = frame[:6]
            if this_head == head:
                for hex_len in len_lists:
                    if self.myXORCheck(frame[:hex_len+6]):
                        len_count_dict[hex_len] += 1
                        break
                    else:
                        continue

        bd_length = '{'+str(max(len_count_dict, key=lambda x: len_count_dict[x]))+'}'

        return bd_length

    # 找到所有数据帧
    def findAllDataFrame(self):
        frame0A, frame10, frame20, frame0B, frame0C, frame2A, frame1B, frame30, frame31, frameEE, frame4E, frameEF, frame6E, frame0D, frame0E = [], [], [], [], [], [], [], [], [], [], [], [], [], [], []
        flag, flagGps, flagImu = True, True, True
        indexIns, indexGps, indexImu = 0, 0, 0
        self.gpsInsTimeIndex, self.gpsImuTimeIndex, self.vehicleInsTimeIndex, self.syncInsTimeIndex, self.imuGpsTimeIndex = [], [], [], [], []

        # 10帧 检查长短
        self.bd_len_10 = self.findHexLength('bddb10', [142, 134])
        self.bd_len_0B = self.findHexLength('bddb0b', [120, 110])
        # frames = re.findall(r'bddb10.{142}', self.fileHexData)
        # long_db, short_db = 0, 0
        # for frame in frames:
        #     head = frame[:6]
        #     if head == 'bddb10':
        #         if self.myXORCheck(frame):
        #             long_db += 1
        #         elif self.myXORCheck(frame[:-8]):
        #             short_db += 1
        #         else:
        #             continue
        # bd_len = '{142}' if long_db > short_db else '{134}'
        re_find = r'bddb0a.{62}|bddb20.{62}|bddb0c.{18}|bddb2a.{62}|bddb1b.{132}|bddbee.{26}|bddb4e.{30}|bddb6e.{28}|bddb0d.{74}|bddb0e.{22}|bddbef.{1242}'
        frames = re.findall(
            # r'bddb0a.{62}|bddb20.{62}|bddb0b.{120}|bddb0c.{18}|bddb2a.{62}|bddb1b.{132}|bddbee.{26}|bddb6e.{28}',
            'bddb10.' + self.bd_len_10+'|bddb0b.' + self.bd_len_0B + '|' + re_find,
            self.fileHexData)
        for frame in frames:
            head = frame[:6]
            if head == 'bddb0a':
                if not self.myXORCheck(frame):  # 数据校验有问题
                    self.dataFrameStats['imuCheckErrIndex'].append(
                        self.dataFrameStats['imuDataNum'] + len(self.dataFrameStats['imuCheckErrIndex']))  # 统计校验有问题的数据帧的序列号(下标)
                    continue
                indexImu += 1
                if flagImu:  # 首次进入
                    indexImu = 0
                    flagImu = False
                self.imuGpsTimeIndex.append(indexGps)
                self.dataFrameStats['imuDataNum'] += 1
                frame0A.append(frame)
            elif head == 'bddb20':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['vehicleCheckErrIndex'].append(
                        self.dataFrameStats['vehicleDataNum'] + len(self.dataFrameStats['vehicleCheckErrIndex']))
                    continue
                self.dataFrameStats['vehicleDataNum'] += 1
                self.vehicleInsTimeIndex.append(indexIns)
                frame20.append(frame)
            elif head == 'bddb0b':
                if not self.myXORCheck(frame) and not self.myXORCheck(frame[:-10]):
                    self.dataFrameStats['insCheckErrIndex'].append(
                        self.dataFrameStats['insDataNum'] + len(self.dataFrameStats['insCheckErrIndex']))
                    continue
                indexIns += 1
                if flag:  # 首次进入
                    indexIns = 0
                    flag = False
                self.dataFrameStats['insDataNum'] += 1
                frame0B.append(frame)
            elif head == 'bddb0c':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['syncCheckErrIndex'].append(
                        self.dataFrameStats['syncDataNum'] + len(self.dataFrameStats['syncCheckErrIndex']))
                    continue
                self.dataFrameStats['syncDataNum'] += 1
                self.syncInsTimeIndex.append(indexIns)
                frame0C.append(frame)
            elif head == 'bddb1b':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['ins2CheckErrIndex'].append(
                        self.dataFrameStats['ins2DataNum'] + len(self.dataFrameStats['ins2CheckErrIndex']))
                    continue
                self.ins2TimeIndex.append(indexIns)
                self.dataFrameStats['ins2DataNum'] += 1
                frame1B.append(frame)
            elif head == 'bddb2a':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['imu2CheckErrIndex'].append(
                        self.dataFrameStats['imu2DataNum'] + len(self.dataFrameStats['imu2CheckErrIndex']))
                    continue
                self.dataFrameStats['imu2DataNum'] += 1
                frame2A.append(frame)
            elif head == 'bddbee':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['EKFhandle_typeCheckErrIndex'].append(
                        self.dataFrameStats['EKFhandle_typeDataNum_bdbdEE'])
                    continue
                self.dataFrameStats['EKFhandle_typeDataNum'] += 1
                frameEE.append(frame)
            elif head == 'bddbef':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['x_status_ErrIndex'].append(
                        self.dataFrameStats['x_status_bdbdEF'] + len(self.dataFrameStats['x_status_ErrIndex']))
                    continue
                self.dataFrameStats['x_status_data_num'] += 1
                frameEF.append(frame)
            elif head == 'bddb4e':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['history_pos_ErrIndex'].append(
                        self.dataFrameStats['history_pos_bdbd4E'] + len(self.dataFrameStats['history_pos_ErrIndex']))
                    continue
                self.dataFrameStats['history_pos_data_num'] += 1
                frame4E.append(frame)
            elif head == 'bddb6e':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['zeroBiasErrIndex'].append(self.dataFrameStats['zeroBiasNum'])
                    continue
                self.dataFrameStats['zeroBiasNum'] += 1
                frame6E.append(frame)
            elif head == 'bddb10':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['gpsCheckErrIndex'].append(
                        self.dataFrameStats['gpsDataNum'] + len(self.dataFrameStats['gpsCheckErrIndex']))
                    continue
                indexGps += 1
                if flagGps:  # 首次进入
                    indexGps = 0
                    flagGps = False
                self.gpsInsTimeIndex.append(indexIns)
                self.gpsImuTimeIndex.append(indexImu)
                self.dataFrameStats['gpsDataNum'] += 1
                frame10.append(frame)
            elif head == 'bddb0d':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['InsPlErrIndex'].append(self.dataFrameStats['InsPlNum'])
                    continue
                self.dataFrameStats['InsPlNum'] += 1
                frame0D.append(frame)
            elif head == 'bddb0e':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['GnssPlErrIndex'].append(self.dataFrameStats['GnssPlNum'])
                    continue
                self.dataFrameStats['GnssPlNum'] += 1
                frame0E.append(frame)

        # 30 31帧
        # # method 0
        # frame_sat = re.findall(r'bddb30.{4838}|bddb31.{6502}', self.fileHexData)
        # for frame in frame_sat:
        #     head = frame[:6]
        #     if head == 'bddb30':
        #         if not self.myXORCheck(frame[:-4]):
        #             self.dataFrameStats['satelliteCheckErrIndex'].append(
        #                 self.dataFrameStats['satelliteDataNum']+len(self.dataFrameStats['satelliteCheckErrIndex']))
        #             continue
        #         self.dataFrameStats['satelliteDataNum'] += 1
        #         frame31.append(frame[:-4])
        #     if head == 'bddb31':
        #         if not self.myXORCheck(frame[:-4]):
        #             self.dataFrameStats['satellite2CheckErrIndex'].append(
        #                 self.dataFrameStats['satellite2DataNum']+len(self.dataFrameStats['satellite2CheckErrIndex']))
        #             continue
        #         self.dataFrameStats['satellite2DataNum'] += 1
        #         frame31.append(frame[:-4])
        # method 1
        if self.data_analysis_flag['sat']:
            for i in range(len(self.fileHexData[:-6]) - 6028):
                if self.fileHexData[i:i + 6] == 'bddb30':
                    frame30.append(self.fileHexData[i:i + 6028])
            self.dataFrameStats['satelliteDataNum'] = len(frame30)
        if self.data_analysis_flag['sat2']:
            for i in range(len(self.fileHexData[:-6]) - 8108):
                if self.fileHexData[i:i + 6] == 'bddb31':
                    frame31.append(self.fileHexData[i:i + 8108])
            self.dataFrameStats['satellite2DataNum'] = len(frame31)

        self.imuDataFrame = frame0A
        self.gpsDataFrame = frame10
        self.vehicleDataFrame = frame20
        self.insDataFrame = frame0B
        self.syncDataFrame = frame0C
        self.imu2DataFrame = frame2A
        self.ins2DataFrame = frame1B
        self.satelliteDataFrame = frame30
        self.satellite2DataFrame = frame31
        self.EKFhandle_typeFrame = frameEE
        self.historyPosDataFrame = frame4E
        self.xStatusDataFrame = frameEF
        self.zeroBiasDataFrame = frame6E
        self.InsPlDataFrame = frame0D
        self.GnssPlDataFrame = frame0E
        # 减少内存占用
        self.gpsInsTimeIndex = np.array(self.gpsInsTimeIndex)
        self.vehicleInsTimeIndex = np.array(self.vehicleInsTimeIndex)
        self.syncInsTimeIndex = np.array(self.syncInsTimeIndex)
        self.imu2TimeIndex = np.array(self.imu2TimeIndex)
        self.ins2TimeIndex = np.array(self.ins2TimeIndex)
        self.satelliteTimeIndex = np.array(self.satelliteTimeIndex)
        self.satellite2TimeIndex = np.array(self.satellite2TimeIndex)
        self.EKFhandle_typeIndex = np.array(self.EKFhandle_typeIndex)
        self.zeroBiasIndex = np.array(self.zeroBiasIndex)
        # 统计不同帧的长度
        self.dataFrameStats['imuDataTotalNum'] = self.dataFrameStats['imuDataNum'] + len(
            self.dataFrameStats['imuCheckErrIndex'])
        self.dataFrameStats['gpsDataTotalNum'] = self.dataFrameStats['gpsDataNum'] + len(
            self.dataFrameStats['gpsCheckErrIndex'])
        self.dataFrameStats['vehicleDataTotalNum'] = self.dataFrameStats['vehicleDataNum'] + len(
            self.dataFrameStats['vehicleCheckErrIndex'])
        self.dataFrameStats['insDataTotalNum'] = self.dataFrameStats['insDataNum'] + len(
            self.dataFrameStats['insCheckErrIndex'])
        self.dataFrameStats['syncDataTotalNum'] = self.dataFrameStats['syncDataNum'] + len(
            self.dataFrameStats['syncCheckErrIndex'])
        self.dataFrameStats['imu2DataTotalNum'] = self.dataFrameStats['imu2DataNum'] + len(
            self.dataFrameStats['imu2CheckErrIndex'])
        self.dataFrameStats['ins2DataTotalNum'] = self.dataFrameStats['ins2DataNum'] + len(
            self.dataFrameStats['ins2CheckErrIndex'])
        self.dataFrameStats['satelliteDataTotalNum'] = self.dataFrameStats['satelliteDataNum'] + len(
            self.dataFrameStats['satelliteCheckErrIndex'])
        self.dataFrameStats['satellite2DataTotalNum'] = self.dataFrameStats['satellite2DataNum'] + len(
            self.dataFrameStats['satellite2CheckErrIndex'])
        self.dataFrameStats['InsPlTotalNum'] = self.dataFrameStats['InsPlNum'] + len(
            self.dataFrameStats['InsPlErrIndex'])
        self.dataFrameStats['GnssPlTotalNum'] = self.dataFrameStats['GnssPlNum'] + len(
            self.dataFrameStats['GnssPlErrIndex'])
        self.dataFrameStats['x_status_data_total_num'] = self.dataFrameStats['x_status_data_num'] + len(
            self.dataFrameStats['x_status_ErrIndex'])
        self.dataFrameStats['history_pos_data_total_num'] = self.dataFrameStats['history_pos_data_num'] + len(
            self.dataFrameStats['history_pos_ErrIndex'])

    # 各帧头数量统计
    def dataFrameHeadNumStats(self):
        self.dataFrameStats['imuFrameHeadNum_bdbd0a'] = len(re.findall(r'bddb0a', self.fileHexData))
        self.dataFrameStats['gpsFrameHeadNum_bdbd10'] = len(re.findall(r'bddb10', self.fileHexData))
        self.dataFrameStats['vehicleFrameHeadNum_bdbd20'] = len(re.findall(r'bddb20', self.fileHexData))
        self.dataFrameStats['insFrameHeadNum_bdbd0b'] = len(re.findall(r'bddb0b', self.fileHexData))
        self.dataFrameStats['syncFrameHeadNum_bdbd0c'] = len(re.findall(r'bddb0c', self.fileHexData))
        self.dataFrameStats['imu2FrameHeadNum_bdbd2a'] = len(re.findall(r'bddb2a', self.fileHexData))
        self.dataFrameStats['ins2FrameHeadNum_bdbd1b'] = len(re.findall(r'bddb1b', self.fileHexData))
        self.dataFrameStats['satelliteFrameHeadNum_bdbd30'] = len(re.findall(r'bddb30', self.fileHexData))
        self.dataFrameStats['satellite2FrameHeadNum_bdbd31'] = len(re.findall(r'bddb31', self.fileHexData))
        self.dataFrameStats['EKFhandle_typeDataNum_bdbdEE'] = len(re.findall(r'bddbee', self.fileHexData))
        self.dataFrameStats['x_status_bdbdEF'] = len(re.findall(r'bddbef', self.fileHexData))
        self.dataFrameStats['history_pos_bdbd4E'] = len(re.findall(r'bddb4e', self.fileHexData))
        self.dataFrameStats['ZeroBiasDataNum_bdbd6E'] = len(re.findall(r'bddb6e', self.fileHexData))
        self.dataFrameStats['InsFlFrameHeadNum_bdbd0D'] = len(re.findall(r'bddb0d', self.fileHexData))
        self.dataFrameStats['GnssFlFrameHeadNum_bdbd0E'] = len(re.findall(r'bddb0e', self.fileHexData))

    # 所有数据存为DataFrame数格式
    def clear_cache(self):
        # PS: 为减少内存占用， 批量统计时不会用到的指标不保存到变量
        self.imuData = pd.DataFrame()  # 释放内存!
        self.ImuDataDF = pd.DataFrame()
        self.vehicleData = pd.DataFrame()
        self.VehicleDataDF = pd.DataFrame()
        self.syncData = pd.DataFrame()  # 释放内存!
        self.SyncDataDF = pd.DataFrame()
        self.satelliteData = pd.DataFrame()
        self.satellite2Data = pd.DataFrame()
        self.insData = pd.DataFrame()
        self.InsDataDF = pd.DataFrame()
        self.gpsData = pd.DataFrame()
        self.GpsDataDF = pd.DataFrame()

        self.frameHeadStrLst = []  # 各帧头字符串列表
        self.gpsInsTimeIndex = []  # GPS数据系统时间，使用上一帧INS时间，此处记录的是INS时间数据的下标
        self.vehicleInsTimeIndex = []  # 车辆数据系统时间
        self.syncInsTimeIndex = []  # 同步时间数据系统时间
        self.imu2TimeIndex = []  # IMU2时间数据的下标
        self.ins2TimeIndex = []  # INS2时间数据系统时间
        self.satelliteTimeIndex = []  # 四轮转向时间数据系统时间
        self.satellite2TimeIndex = []
        self.EKFhandle_typeIndex = []  # ekfhandle时间数据系统时间
        self.zeroBiasIndex = None  # 常值零偏系统时间

    # 十进整数制转为二进制列表
    def Unit2Bin(self, num, unit):
        alllist = []
        for i in num:
            tobin = bin(i)[2:].rjust(unit, '0')
            binlist = list(map(float, reversed(list(tobin))))
            alllist.append(binlist)
        return alllist

    def dict2csv(self, dic, filename):
        """
        将字典写入csv文件，要求字典的值长度一致。
        :param dic: the dict to csv
        :param filename: the name of the csv file
        :return: None
        """
        file = open(filename, 'w', encoding='utf-8', newline='')
        csv_writer = csv.DictWriter(file, fieldnames=list(dic.keys()))
        csv_writer.writeheader()
        for i in range(len(dic[list(dic.keys())[0]])):  # 将字典逐行写入csv
            dic1 = {key: dic[key][i] for key in dic.keys()}
            csv_writer.writerow(dic1)
        file.close()

    # 打印输出数据帧统计结果
    def printDataFrameStatsResult(self):
        print("IMU数据帧：", "纯帧头数量:", self.dataFrameStats['imuFrameHeadNum_bdbd0a'], "总帧数量:",
              self.dataFrameStats['imuDataNum'], "错误帧数量:",
              len(self.dataFrameStats['imuCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['imuCheckErrIndex'][:10])
        print("GPS数据帧：", "纯帧头数量:", self.dataFrameStats['gpsFrameHeadNum_bdbd10'], "总帧数量:",
              self.dataFrameStats['gpsDataNum'], "错误帧数量:",
              len(self.dataFrameStats['gpsCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['gpsCheckErrIndex'][:10])
        print("车辆数据帧：", "纯帧头数量:", self.dataFrameStats['vehicleFrameHeadNum_bdbd20'], "总帧数量:",
              self.dataFrameStats['vehicleDataNum'], "错误帧数量:",
              len(self.dataFrameStats['vehicleCheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['vehicleCheckErrIndex'][:10])
        print("INS数据帧：", "纯帧头数量:", self.dataFrameStats['insFrameHeadNum_bdbd0b'], "总帧数量:",
              self.dataFrameStats['insDataNum'], "错误帧数量:",
              len(self.dataFrameStats['insCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['insCheckErrIndex'][:10])
        print("同步时间数据帧：", "纯帧头数量:", self.dataFrameStats['syncFrameHeadNum_bdbd0c'], "总帧数量:",
              self.dataFrameStats['syncDataNum'], "错误帧数量:",
              len(self.dataFrameStats['syncCheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['syncCheckErrIndex'][:10])
        print("IMU2数据帧：", "纯帧头数量:", self.dataFrameStats['imu2FrameHeadNum_bdbd2a'], "总帧数量:",
              self.dataFrameStats['imu2DataNum'], "错误帧数量:",
              len(self.dataFrameStats['imu2CheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['imu2CheckErrIndex'][:10])
        print("INS2数据帧：", "纯帧头数量:", self.dataFrameStats['ins2FrameHeadNum_bdbd1b'], "总帧数量:",
              self.dataFrameStats['ins2DataNum'], "错误帧数量:",
              len(self.dataFrameStats['ins2CheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['ins2CheckErrIndex'][:10])
        print("Satellite数据帧：", "纯帧头数量:", self.dataFrameStats['satelliteFrameHeadNum_bdbd30'], "总帧数量:",
              self.dataFrameStats['satelliteDataNum'], "错误帧数量:",
              len(self.dataFrameStats['satelliteCheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['satelliteCheckErrIndex'][:10])
        print("Satellite31数据帧：", "纯帧头数量:", self.dataFrameStats['satellite2DataTotalNum'], "总帧数量:",
              self.dataFrameStats['satellite2DataNum'], "错误帧数量:",
              len(self.dataFrameStats['satellite2CheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['satellite2CheckErrIndex'][:10])
        print("EE数据帧：", "EE纯帧头数量:", self.dataFrameStats['EKFhandle_typeDataNum_bdbdEE'], "总帧数量:",
              self.dataFrameStats['EKFhandle_typeDataNum'], "错误帧数量:",
              len(self.dataFrameStats['EKFhandle_typeCheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['EKFhandle_typeCheckErrIndex'][:10])
        print("EF数据帧：", "EF纯帧头数量:", self.dataFrameStats['x_status_bdbdEF'], "总帧数量:",
              self.dataFrameStats['x_status_data_num'], "错误帧数量:",
              len(self.dataFrameStats['x_status_ErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['x_status_ErrIndex'][:10])
        print("4E数据帧：", "4E纯帧头数量:", self.dataFrameStats['history_pos_bdbd4E'], "总帧数量:",
              self.dataFrameStats['history_pos_data_num'], "错误帧数量:",
              len(self.dataFrameStats['history_pos_ErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['history_pos_ErrIndex'][:10])
        print("常值零偏数据帧：", "6E纯帧头数量:", self.dataFrameStats['ZeroBiasDataNum_bdbd6E'], "总帧数量:",
              self.dataFrameStats['zeroBiasNum'], "错误帧数量:",
              len(self.dataFrameStats['zeroBiasErrIndex']), "错误帧索引下标:", self.dataFrameStats['zeroBiasErrIndex'][:10])
        print("INS完好性数据帧：", "0D纯帧头数量:", self.dataFrameStats['InsFlFrameHeadNum_bdbd0D'], "总帧数量:",
              self.dataFrameStats['InsPlNum'], "错误帧数量:",
              len(self.dataFrameStats['InsPlErrIndex']), "错误帧索引下标:", self.dataFrameStats['InsPlErrIndex'][:10])
        print("GNSS完好性数据帧：", "0E纯帧头数量:", self.dataFrameStats['GnssFlFrameHeadNum_bdbd0E'], "总帧数量:",
              self.dataFrameStats['GnssPlNum'], "错误帧数量:",
              len(self.dataFrameStats['GnssPlErrIndex']), "错误帧索引下标:", self.dataFrameStats['GnssPlErrIndex'][:10])

    @staticmethod
    def checkDataFreq(df, timeName='', freq=200):
        """
        计算不同数据
        其中，IMU默认200hz 轮速默认50hz或100hz GPS默认10hz或5hz
        :param timeName: 时间字段名称
        :param df: 解析出来的帧 dataframe
        :param freq: 数据应为频率
        :return:
        """
        time_diff_dict = {}
        dataLen = 0

        if len(df) == 0:
            time_diff_dict['帧数统计'] = '无该帧信息'
            return dataLen, time_diff_dict
        if timeName not in df.keys():
            time_diff_dict['帧数统计'] = '无'+str(timeName)+'信息'
            return dataLen, time_diff_dict
        try:
            duration = df.iloc[-1][timeName] - df.iloc[0][timeName]
            dataLen = duration * freq
            time_diff = np.array(np.diff(df[timeName]))

            gap_time = 1 / freq
            time_diff_dict[
                '与前一帧间隔时长在 [%s s,%s s]的帧数' % (str(gap_time * 0.95), str(gap_time * 1.05))] = len1 = len(
                time_diff[np.where((time_diff > gap_time * 0.95) & (time_diff < gap_time * 1.05))])
            time_diff_dict[
                '与前一帧间隔时长在 [%s s,%s s]的帧数' % (str(gap_time * 0.5), str(gap_time * 0.95))] = len2 = len(
                time_diff[np.where((time_diff >= gap_time * 0.5) & (time_diff < gap_time * 0.95))])
            time_diff_dict[
                '与前一帧间隔时长在 [%s s,%s s]的帧数' % (str(gap_time * 1.05), str(gap_time * 1.5))] = len3 = len(
                time_diff[np.where((time_diff > gap_time * 1.05) & (time_diff <= gap_time * 1.5))])
            time_diff_dict['其他帧间隔时长的帧数'] = len(time_diff) - len1 - len2 - len3
        except Exception as e:
            time_diff_dict['帧数统计'] = str(e)

        return dataLen, time_diff_dict

    def checkFreq(self):
        try:
            self.full_info = "开始检查频率是否有误："
            imu_actual_len, self.freq_info['imu不同时间间隔帧数'] = self.checkDataFreq(self.ImuDataDF, timeName='time', freq=self.imu_freq)
            self.freq_info['imu数据长度'] = imu_actual_len + len(self.dataFrameStats['imuCheckErrIndex'])

            vehicle_actual_len, self.freq_info['vehicle不同时间间隔帧数'] = self.checkDataFreq(self.VehicleDataDF, timeName='ts', freq=self.vehicle_freq)
            self.freq_info['vehicle数据长度'] = vehicle_actual_len + len(self.dataFrameStats['vehicleCheckErrIndex'])

            gps_actual_len, self.freq_info['gps不同时间间隔帧数'] = self.checkDataFreq(self.GpsDataDF, timeName='ts', freq=self.gps_freq)
            self.freq_info['gps数据长度'] = gps_actual_len + len(self.dataFrameStats['gpsCheckErrIndex'])
            self.full_info += "\n【检查完成】。"

            self.outputDataFrameStatsResult()  # 输出数据帧统计结果
        except Exception as e:
            self.full_info += "文件频率检测失败..."
            self.full_info += '失败原因:' + str(e)

    def outputDataFrameStatsResult(self):
        try:
            msg = "IMU数据帧" \
                  "：\n    纯帧头数量:" + str(self.dataFrameStats['imuFrameHeadNum_bdbd0a']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['imuDataTotalNum']) \
                  + "，\n    " + str(self.imu_freq) + "Hz频率下应该有帧数：" + str(self.freq_info['imu数据长度']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['imuCheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['imuCheckErrIndex']))) \
                  + "，\n    不同时间间隔帧数:\n        " + str(self.freq_info['imu不同时间间隔帧数']) \
                  + "。\n\n"
            self.full_info = msg

            msg = "GPS数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['gpsFrameHeadNum_bdbd10']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['gpsDataTotalNum']) \
                  + "，\n    " + str(self.gps_freq) + "Hz频率下应该有帧数：" + str(self.freq_info['gps数据长度']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['gpsCheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['gpsCheckErrIndex']))) \
                  + "，\n    不同时间间隔帧数:\n        " + str(self.freq_info['gps不同时间间隔帧数']) \
                  + "。\n\n"
            self.full_info += msg

            msg = "车辆数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['vehicleFrameHeadNum_bdbd20']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['vehicleDataTotalNum']) \
                  + "，\n    " + str(self.vehicle_freq) + "Hz频率下应该有帧数：" + str(self.freq_info['vehicle数据长度']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['vehicleCheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['vehicleCheckErrIndex']))) \
                  + "，\n    不同时间间隔帧数:\n        " + str(self.freq_info['vehicle不同时间间隔帧数']) \
                  + "。\n\n"
            self.full_info += msg

            msg = "INS数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['insFrameHeadNum_bdbd0b']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['insDataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['insCheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['insCheckErrIndex']))) \
                  + "。\n\n"
            self.full_info += msg

            msg = "同步时间数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['syncFrameHeadNum_bdbd0c']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['syncDataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['syncCheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['syncCheckErrIndex']))) \
                  + "。\n\n"
            self.full_info += msg

            msg = "IMU2数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['imu2FrameHeadNum_bdbd2a']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['imu2DataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['imu2CheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['imu2CheckErrIndex']))) \
                  + "。\n\n"
            self.full_info += msg

            msg = "INS2数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['ins2FrameHeadNum_bdbd1b']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['ins2DataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['ins2CheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['ins2CheckErrIndex']))) \
                  + "。\n\n"
            self.full_info += msg

            msg = "30卫星数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['satelliteFrameHeadNum_bdbd30']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['satelliteDataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['satelliteCheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['satelliteCheckErrIndex']))) \
                  + "。\n\n"
            self.full_info += msg

            msg = "31卫星数据帧：" \
                  "\n    纯帧头数量:" + str(self.dataFrameStats['satellite2FrameHeadNum_bdbd31']) \
                  + "，\n    总帧数量:" + str(self.dataFrameStats['satellite2DataTotalNum']) \
                  + "，\n    错误帧数量:" + str(len(self.dataFrameStats['satellite2CheckErrIndex'])) \
                  + "，\n    错误帧索引下标:" + str(list(set(self.dataFrameStats['satellite2CheckErrIndex']))) \
                  + "。\n\n"
            self.full_info += msg

            self.full_info += '\nPS. 采用奇偶校验帧是否错误。'

        except Exception as e:
            self.full_info += 'bddb数据解析数据显示有误'
            self.full_info += str(e)


if __name__ == "__main__":
    obj = HexDataParse()
    # obj.filePath = r"D:\Files\test\dbFiles\test1\test1_LogINS.txt" #10帧是134
    # obj.filePath = r'D:\Files\test\dbFiles\test6_320\12311-1114-紧组合.txt'  # 142长的10帧
    # obj.filePath = r'D:\Files\test\dbFiles\test1\test1_LogINS.txt'
    obj.filePath = r"E:\Downloads\fault_bddb_Li_test_wenzixuan_20240518172611473_ins_bddb_xh_0518_2_ins_2.edited"
    obj.data_analysis_flag = {'ins': True, 'gps': True, 'vehicle': False, 'imu': False
        , 'ins2': False, 'imu2': False, 'sync': False
        , 'sat': True, 'sat2': True, 'ZeroBias': False, 'EKFhandle_type': False
        , 'InsPl': False, 'GnssPl': False, 'x_status_ef': False, 'history_pos_4e': False}
    # types = ["csv", "mat"]
    types = ["csv", "mat"]

    print(time.strftime('%H:%M:%S', time.localtime()), "开始解析数据: ", obj.filePath)
    obj.startParseFileHexData()  # 开始数据解析
    print(time.strftime('%H:%M:%S', time.localtime()), "解析完成...")
    obj.printDataFrameStatsResult()
    obj.saveDataToDF()
    if "csv" in types:
        print("存成Csv文件...")
        obj.SaveAllDataToCsvFile()
    if "mat" in types:
        print("存成Mat文件...")
        obj.saveAllDataToMatFile()
    print(time.strftime('%H:%M:%S', time.localtime()), "数据保存完成")
