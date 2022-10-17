# -*- coding: UTF-8 -*-

import re  # 正则表达式
import struct  # 十六进制数据解析
import pandas as pd
import os  # 处理文件路径、文件名、创建文件夹等
# from memory_profiler import profile  # 内存占用分析
import subprocess
from scipy.io import savemat  # 存成.mat文件格式
import numpy as np
import csv
import time


class HexDataParse(object):
    """
    解析二进制文件
    @Author: fsw
    @Date: 2022/9/23 [只读前10000个字节]
    """

    def __init__(self):
        # 数据解析
        self.filePath = ""  # 数据文件路径
        self.fileHexData = ""  # 存储从文件里读取的十六进制数据

        self.sysTimeFactor = 4000  # 系统时间系数，IMU数据、INS数据里会用到，570D除以1000，个别版本(234平台和377平台)是除以4000

        self.imuDataFrame = None  # IMU数据帧，列表数据类型，元素是十六进制字符串
        self.imuData = None  # 解析后的IMU数据，pandas数据类型，这样占用内存更小，由于pandas数据结构添加一行数据效率很低，所以先以字典数据类型收集好数据，然后转成pandas数据结构进行存储

        self.gpsDataFrame = None  # GPS数据帧
        self.gpsData = None

        self.vehicleDataFrame = None  # 车辆数据帧
        self.vehicleData = None

        self.insDataFrame = None  # INS数据帧
        self.insData = None

        self.syncDataFrame = None  # 同步时间数据帧
        self.syncData = None

        self.imu2DataFrame = None  # 2Aimu数据帧
        self.imu2Data = None

        self.ins2DataFrame = None  # 1Bins数据
        self.ins2Data = None

        self.FourWSDataFrame = None  # 四轮转向帧
        self.FourWSData = None

        self.dataFrameStats = {'imuDataNum': 0, 'imuCheckErrIndex': [], 'gpsDataNum': 0, 'gpsCheckErrIndex': [],
                               'vehicleDataNum': 0, 'vehicleCheckErrIndex': [], 'insDataNum': 0, 'insCheckErrIndex': [],
                               'syncDataNum': 0, 'syncCheckErrIndex': [], 'imuFrameHeadNum_bdbd0a': 0,
                               'gpsFrameHeadNum_bdbd10': 0, 'vehicleFrameHeadNum_bdbd20': 0,
                               'insFrameHeadNum_bdbd0b': 0,
                               'syncFrameHeadNum_bdbd0c': 0, 'imu2DataNum': 0, 'imu2CheckErrIndex': [],
                               'ins2DataNum': 0, 'ins2CheckErrIndex': [],
                               'imu2FrameHeadNum_bdbd2a': 0, 'ins2FrameHeadNum_bdbd1b': 0,
                               'FourWSDataNum': 0, 'FourWSCheckErrIndex': [],
                               'FourWSFrameHeadNum_bdbd30': 0

                               }  # DataNum：数据个数，CheckErr：校验错误数据帧的序列号(下标)，FrameHeadNum：帧头数，只匹配帧头得出

        # 数据存储
        self.saveFolderPath = ""  # 存储文件夹路径，初始值要是“”，后面会用此作为判断条件，表明是否使用自定义文件夹
        self.frameHeadStrLst = None  # 各帧头字符串列表
        self.gpsInsTimeIndex = None  # GPS数据系统时间，使用上一帧INS时间，此处记录的是INS时间数据的下标
        self.vehicleInsTimeIndex = None  # 车辆数据系统时间
        self.syncInsTimeIndex = None  # 同步时间数据系统时间
        self.imu2TimeIndex = None  # IMU2时间数据的下标
        self.ins2TimeIndex = None  # INS2时间数据系统时间
        self.FourWSTimeIndex = None  # 四轮转向时间数据系统时间
        self.SyncInsGpsData = None  # Gps和INS同步数据
        self.PDataDict = None  # 轮询表DataFrame结构
        self.SyncDataDF = None  # Gps和INS同步数据DataFrame结构
        self.VehicleDataDF = None  # 车辆步数据DataFrame结构
        self.GpsDataDF = None  # Gps数据DataFrame结构
        self.ImuDataDF = None  # IMU数据DataFrame结构
        self.InsDataDF = None  # INS数据DataFrame结构

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
        self.dataFrameStats['FourWSDataNum'] = 0
        self.dataFrameStats['FourWSCheckErrIndex'] = []

    # 读取文件数据，以十六进制形式
    def readFileData(self):
        with open(self.filePath, 'rb') as f:
            self.fileHexData = f.read().hex()  # hex()函数：bytes转hex字符串

            print('截断前10000个字节， 仅用于生成杆臂值！')
            self.fileHexData = self.fileHexData[:100000] if len(self.fileHexData) > 100000 else self.fileHexData

    # 异或校验
    # hexstr：一帧数据包，十六进制字符串，包括末尾校验位
    # 返回值：校验没问题：返回True，校验有问题：返回False
    @staticmethod
    def myXORCheck(hexstr):
        a = 0
        for i in range(len(hexstr))[::2]:
            a ^= int(hexstr[i:i + 2], 16)

        if a == 0:
            return True
        else:
            return False

    # 找到IMU数据帧
    # def findIMUDataFrame(self):
    #     self.imuDataFrame = re.findall(r'bddb0a.{62}', self.fileHexData)  # IMU数据帧头是BD DB 0A，包括帧头一共34个字节，一个字节用2个十六进制数表示

    # 解析一帧IMU数据
    # dataHexStr：一帧数据包，十六进制字符串
    # sysTimeFactor：系统时间系数，570D除以1000，个别版本(234平台和377平台)是除以4000
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    @staticmethod
    def parseIMUDataOneFrame(dataHexStr, sysTimeFactor=4000):
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
            # if not self.myXORCheck(dataPkg):  # 数据校验有问题
            #     self.dataFrameStats['imuCheckErrIndex'].append(self.dataFrameStats['imuDataNum'])  # 统计校验有问题的数据帧的序列号(下标)
            #     self.dataFrameStats['imuDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['imuDataNum'] += 1

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
        # 数据依次为 按照协议顺序
        data = list(struct.unpack('<iHiHiH2I9hH4BH3I2B', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

        data[0] *= 1e-7  # 经度
        data[1] *= 1e-3
        data[2] *= 1e-7
        data[3] *= 1e-3
        data[4] *= 1e-3
        data[5] *= 1e-3
        data[8] *= 1e-2  # 水平速度
        data[9] *= 1e-2
        data[10] *= 1e-2
        data[11] *= 1e-3
        data[12] *= 1e-3
        data[13] *= 1e-2
        if data[13] > 180:
            data[13] -= 360
        data[14] *= 1e-3
        data[15] *= 1e-3
        data[16] *= 1e-3
        data[22] *= 1e-3  # 秒
        data[23] *= 1e-3
        data[24] *= 1e-3
        data[25] *= 1e-3
        data.append(data[7] % 256)

        return data

    # 解析GPS数据
    def parseGPSData(self):
        dataDic = {'Lon': [], 'LonStd': [], 'Lat': [], 'LatStd': [], 'hMSL': [], 'hMSLStd': [], 'gpsFix': [],
                   'flags': [], 'HSpd': [], 'TrackAngle': [], 'VSpd': [], 'LatencyVel': [], 'BaseLineLength': [],
                   'Heading': [], 'HeadingStd': [], 'Pitch': [], 'PitchStd': [], 'year': [], 'month': [],
                   'day': [], 'hour': [], 'minute': [], 'second': [], 'itow_pos': [], 'itow_vel': [],
                   'itow_heading': [], 'RecMsg': [], 'numSV': [], 'flagsPos': []}
        sorteddataDic = ['Lon', 'LonStd', 'Lat', 'LatStd', 'hMSL', 'hMSLStd', 'gpsFix', 'flags', 'HSpd', 'TrackAngle',
                         'VSpd', 'LatencyVel', 'BaseLineLength', 'Heading', 'HeadingStd', 'Pitch', 'PitchStd', 'year',
                         'month', 'day', 'hour', 'minute', 'second', 'itow_pos', 'itow_vel', 'itow_heading', 'RecMsg',
                         'numSV', 'flagsPos']

        for dataPkg in self.gpsDataFrame:
            # if not self.myXORCheck(dataPkg):
            #     self.dataFrameStats['gpsCheckErrIndex'].append(self.dataFrameStats['gpsDataNum'])
            #     self.dataFrameStats['gpsDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['gpsDataNum'] += 1

            data = self.parseGPSDataOneFrame(dataPkg)

            i = 0
            for _ in dataDic:
                dataDic[sorteddataDic[i]].append(data[i])
                i += 1

        self.gpsDataFrame = None  # 释放内存
        self.gpsData = pd.DataFrame(dataDic)

    # 找到车辆数据帧
    # def findVehicleDataFrame(self):
    #     self.vehicleDataFrame = re.findall(r'bddb20.{62}', self.fileHexData)  # 车辆数据帧头是BD DB 20，包括帧头一共34个字节

    # 解析一帧车辆数据
    # dataHexStr：一帧数据包，十六进制字符串
    # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    def parseVehicleDataOneFrame(self, dataHexStr):
        # 数据依次为 按照协议顺序
        data = list(struct.unpack('<2HIB2HIBBIhIB', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

        data[2] *= 1e-3
        data[6] *= 1e-3
        data[9] *= 1e-3
        data[10] *= 0.0625
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
            # if not self.myXORCheck(dataPkg):
            #     self.dataFrameStats['vehicleCheckErrIndex'].append(self.dataFrameStats['vehicleDataNum'])
            #     self.dataFrameStats['vehicleDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['vehicleDataNum'] += 1

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
        data = list(struct.unpack('<9h3i3hBIH3hIB', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位

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
        data[-2] /= sysTimeFactor

        return data

    # 解析INS数据
    def parseINSData(self):
        dataDic = {"roll": [], "pitch": [], "yaw": [], "GyroX": [], "GyroY": [], "GyroZ": [], "AccX": [],
                   "AccY": [], "AccZ": [], "lat": [], "lon": [], "height": [], "NorthVelocity": [],
                   "EastVelocity": [], "GroundVelocity": [], "IMUstatus": [], "LEKFstatus": [], "GPSstatus": [],
                   "PData1": [], "PData2": [], "PData3": [], "time": [], "Ptype": []}
        sorteddataDic = ["roll", "pitch", "yaw", "GyroX", "GyroY", "GyroZ", "AccX", "AccY", "AccZ", "lat", "lon",
                         "height", "NorthVelocity", "EastVelocity", "GroundVelocity", "IMUstatus", "LEKFstatus",
                         "GPSstatus", "PData1", "PData2", "PData3", "time", "Ptype"]

        for dataPkg in self.insDataFrame:
            # if not self.myXORCheck(dataPkg):  # 数据校验有问题
            #     self.dataFrameStats['insCheckErrIndex'].append(self.dataFrameStats['insDataNum'])
            #     self.dataFrameStats['insDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['insDataNum'] += 1

            data = self.parseINSDataOneFrame(dataPkg, self.sysTimeFactor)

            i = 0
            for key in dataDic:
                dataDic[sorteddataDic[i]].append(data[i])
                i += 1

        self.insDataFrame = None
        self.insData = pd.DataFrame(dataDic)

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
            # if not self.myXORCheck(dataPkg):
            #     self.dataFrameStats['syncCheckErrIndex'].append(self.dataFrameStats['syncDataNum'])
            #     self.dataFrameStats['syncDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['syncDataNum'] += 1

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
            # if not self.myXORCheck(dataPkg):  # 数据校验有问题
            #     self.dataFrameStats['imuCheckErrIndex'].append(self.dataFrameStats['imuDataNum'])  # 统计校验有问题的数据帧的序列号(下标)
            #     self.dataFrameStats['imuDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['imuDataNum'] += 1

            data = self.parseIMU2DataOneFrame(dataPkg, self.sysTimeFactor)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1

        self.imu2DataFrame = None  # 释放内存
        self.imu2Data = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

        # 找到INS2数据帧

        # 找到INS2数据帧

    def findINS2DataFrame(self):
        self.imuDataFrame = re.findall(r'bddb1b.{132}', self.fileHexData)  # INS数据帧头是BD DB 1B，包括帧头一共69个字节，一个字节用2个十六进制数表示

        # 解析一帧INS2数据
        # dataHexStr：一帧数据包，十六进制字符串
        # sysTimeFactor：系统时间系数，570D除以1000，个别版本(234平台和377平台)是除以4000
        # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列

    @staticmethod
    def parseINS2DataOneFrame(dataHexStr, sysTimeFactor=4000):
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
                   "OdomX": [], "OdomY": [], "OdomZ": [], "OdomVX": [],
                   "OdomVY": [], "OdomVZ": [], "OdomVM": [], "OdomAngular_VX": [],
                   "OdomAngular_VY": [], "OdomAngular_VZ": [], "OdomAcceleration_X": [], "OdomAcceleration_Y": [],
                   "OdomAcceleration_Z": [], "LiOdometryStatus": [], "sensor_status": [], "temp_P0": [],
                   "temp_P1": [], "temp_P2": [], "Index": [], "frame_id": [], "time": []
                   }
        for dataPkg in self.ins2DataFrame:
            # if not self.myXORCheck(dataPkg):  # 数据校验有问题
            #     self.dataFrameStats['imuCheckErrIndex'].append(self.dataFrameStats['imuDataNum'])  # 统计校验有问题的数据帧的序列号(下标)
            #     self.dataFrameStats['imuDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['imuDataNum'] += 1

            data = self.parseINS2DataOneFrame(dataPkg, self.sysTimeFactor)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1

        self.ins2Datafram = None  # 释放内存
        self.ins2Data = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    @staticmethod
    def parseFourWSDataOneFrame(dataHexStr):
        data = list(struct.unpack('<2hI2hI', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
        data[0] *= 0.005  # 左前轮转角
        data[1] *= 0.005  # 右前轮转角
        data[2] *= 0.001  # 前轮转角时间
        data[3] *= 0.005  # 左后轮转角
        data[4] *= 0.005  # 右后轮转角
        data[5] *= 0.001  # 后轮转角时间
        return data

        # 解析轮转角数据帧

    def parseFourWSData(self):
        dataDic = {"WheelFl": [], "WheelFr": [], "tsWheelF_s": [], "WheelRl": [],
                   "WheelRr": [], "tsWheelR_s": []
                   }
        for dataPkg in self.FourWSDataFrame:
            # if not self.myXORCheck(dataPkg):  # 数据校验有问题
            #     self.dataFrameStats['imuCheckErrIndex'].append(self.dataFrameStats['imuDataNum'])  # 统计校验有问题的数据帧的序列号(下标)
            #     self.dataFrameStats['imuDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['imuDataNum'] += 1

            data = self.parseFourWSDataOneFrame(dataPkg)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1

        self.FourWSDatafram = None  # 释放内存
        self.FourWSData = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    #
    # def findIMU3DataFrame(self):
    #     self.imu3DataFrame = re.findall(r'bddb2a.{38}', self.fileHexData)  # IMU数据帧头是BD DB 0A，包括帧头一共34个字节，一个字节用2个十六进制数表示
    #
    #     # 解析一帧IMU2数据
    #     # dataHexStr：一帧数据包，十六进制字符串
    #     # sysTimeFactor：系统时间系数，570D除以1000，个别版本(234平台和377平台)是除以4000
    #     # 返回值：解析后的数据，列表数据类型，数据按照协议顺序排列
    #
    # def parseIMU3DataOneFrame(self, dataHexStr):
    #     # 数据依次为陀螺仪XYZ轴、加速度XYZ轴、温度、系统时间
    #     data = list(struct.unpack('<9I', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
    #     return data
    #
    #     # 解析IMU2数据帧
    #
    # def parseIMU3Data(self):
    #     dataDic = {"GyroX": [], "GyroY": [], "GyroZ": [], "AccX": [], "AccY": [], "AccZ": [], "temperature1": [], "temperature2": []
    #     , "temperature3": []}
    #
    #     for dataPkg in self.imu3DataFrame:
    # if not self.myXORCheck(dataPkg):  # 数据校验有问题
    #     self.dataFrameStats['imuCheckErrIndex'].append(self.dataFrameStats['imuDataNum'])  # 统计校验有问题的数据帧的序列号(下标)
    #     self.dataFrameStats['imuDataNum'] += 1
    #     continue
    #
    # self.dataFrameStats['imuDataNum'] += 1
    #
    #     data = self.parseIMU3DataOneFrame(dataPkg, self.sysTimeFactor)
    #
    #     i = 0
    #     for key in dataDic:
    #         dataDic[key].append(data[i])
    #         i += 1
    #
    # self.imu3DataFrame = None  # 释放内存
    # self.imu3Data = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    # 找到INS2数据帧

    # 找到INS2数据帧

    # 开始数据解包，总体整合
    # @profile  # 内存分析修饰器，添加这句代码，表明对此函数进行内存分析，内存分析结果会打印输出
    def startParseFileHexData(self):
        self.variableInit()  # 变量初始化

        self.readFileData()

        # self.findIMUDataFrame()
        # self.findGPSDataFrame()
        # self.findVehicleDataFrame()
        # self.findINSDataFrame()
        # self.findSyncDataFrame()
        #
        # self.findFrameHead()  # 找到各帧头，获取INS时间下标用

        self.dataFrameHeadNumStats()  # 各数据帧头个数统计

        self.findAllDataFrame()  # 找到所有数据帧、获得INS时间下标

        self.fileHexData = None  # 释放内存

        self.parseIMUData()
        self.parseGPSData()
        self.parseVehicleData()
        self.parseINSData()
        self.parseSyncData()
        self.parseIMU2Data()
        self.parseINS2Data()
        self.parseFourWSData()
        # self.generateDataInsTimeIndex()  # 生成INS时间下标，要放到self.parseINSData()函数执行后，因为要用到self.dataFrameStats['insCheckErrIndex']

        # 开启多线程，分别解析各类数据。由于python存在GIL锁，所以python多线程是假多线程，对于CPU计算类，不能提高执行速度，对于I/O操作类，可以提高执行速度
        # t = Thread(target=self.parseIMUData)  # 创建一个线程
        # t.setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
        # t.start()  # 启动线程
        # t.join()  # 等待线程终止，要不然一直挂起

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

    # 找到各帧头
    def findFrameHead(self):
        frames = re.findall(
            r'bddb0a.{62}|bddb10.{134}|bddb20.{62}|bddb0b.{110}|bddb0c.{18}|bddb2a.{62}|bddb1b.{132}|bddb30.{36}|bddb30.{38}',
            self.fileHexData)
        self.frameHeadStrLst = [frame[:6] for frame in frames]

    # 各帧头数量统计
    def dataFrameHeadNumStats(self):
        self.dataFrameStats['imuFrameHeadNum_bdbd0a'] = len(re.findall(r'bddb0a', self.fileHexData))
        self.dataFrameStats['gpsFrameHeadNum_bdbd10'] = len(re.findall(r'bddb10', self.fileHexData))
        self.dataFrameStats['vehicleFrameHeadNum_bdbd20'] = len(re.findall(r'bddb20', self.fileHexData))
        self.dataFrameStats['insFrameHeadNum_bdbd0b'] = len(re.findall(r'bddb0b', self.fileHexData))
        self.dataFrameStats['syncFrameHeadNum_bdbd0c'] = len(re.findall(r'bddb0c', self.fileHexData))
        self.dataFrameStats['imu2FrameHeadNum_bdbd2a'] = len(re.findall(r'bddb2a', self.fileHexData))
        self.dataFrameStats['ins2FrameHeadNum_bdbd1b'] = len(re.findall(r'bddb1b', self.fileHexData))
        self.dataFrameStats['FourWSFrameHeadNum_bdbd30'] = len(re.findall(r'bddb30', self.fileHexData))

    # 生成GPS、车辆、同步时间数据的INS数据的下标
    def generateDataInsTimeIndex(self):
        self.gpsInsTimeIndex = []
        self.vehicleInsTimeIndex = []
        self.syncInsTimeIndex = []

        frameIndex = [0, 0, 0, 0]  # 数据帧索引
        i = 0  # 解析后的值索引
        flag = True
        for head in self.frameHeadStrLst:
            if head == 'bddb0b':  # INS帧
                if frameIndex[0] not in self.dataFrameStats['insCheckErrIndex']:
                    i += 1
                    if not flag:  # 首次进入
                        continue
                    flag = False
                    i = 0
                frameIndex[0] += 1
            elif head == 'bddb10':  # GPS帧
                if frameIndex[1] not in self.dataFrameStats['gpsCheckErrIndex']:
                    self.gpsInsTimeIndex.append(i)
                frameIndex[1] += 1
            elif head == 'bddb20':  # 车辆帧
                if frameIndex[2] not in self.dataFrameStats['vehicleCheckErrIndex']:
                    self.vehicleInsTimeIndex.append(i)
                frameIndex[2] += 1
            elif head == 'bddb0c':  # 同步时间帧
                if frameIndex[3] not in self.dataFrameStats['syncCheckErrIndex']:
                    self.syncInsTimeIndex.append(i)
                frameIndex[3] += 1

        # 减少内存占用
        self.frameHeadStrLst = None
        self.gpsInsTimeIndex = np.array(self.gpsInsTimeIndex)
        self.vehicleInsTimeIndex = np.array(self.vehicleInsTimeIndex)
        self.syncInsTimeIndex = np.array(self.syncInsTimeIndex)

        # 将数据存成DataFrame格式

    # 所有数据存为DataFrame数格式
    def saveDataToDF(self):
        # IMU数据
        self.ImuDataDF = self.imuData

        # GPS数据
        self.GpsDataDF = self.gpsData
        self.GpsDataDF['ts'] = [self.insData['time'][i] for i in self.gpsInsTimeIndex]
        self.GpsDataDF['RecMsgBin'] = self.Unit2Bin(self.gpsData['RecMsg'], 8)
        self.GpsDataDF['flagsPos'] = np.array([self.gpsData['flags']]).T % 256
        self.GpsDataDF['flagsVel'] = (np.array([self.gpsData['flags']]).T / 256) % 256
        self.GpsDataDF['flagsHeading'] = (np.array([self.gpsData['flags']]).T / 256 / 256) % 256
        self.GpsDataDF['flagsTime'] = (np.array([self.gpsData['flags']]).T / 256 / 256 / 256) % 256

        # vehicle数据
        self.VehicleDataDF = self.vehicleData
        self.VehicleDataDF['ts'] = [self.insData['time'][i] for i in self.vehicleInsTimeIndex]

        # INS数据
        self.InsDataDF = self.insData
        self.InsDataDF['IMUstatusBin'] = self.Unit2Bin(self.insData['IMUstatus'], 8)
        self.InsDataDF['LEKFstatusBin'] = self.Unit2Bin(self.insData['LEKFstatus'], 32)
        self.InsDataDF['GPSstatusBin'] = self.Unit2Bin(self.insData['GPSstatus'], 8)

        # Sync数据
        self.SyncDataDF = self.syncData
        self.SyncDataDF['ts'] = [self.insData['time'][i] for i in self.syncInsTimeIndex]

        # Pdat(字典)
        self.PDataDict = {}
        PData = self.savePDataToMatFile()
        for key in PData:
            self.PDataDict[key] = PData[key].tolist()

    # 将数据存成.mat文件格式
    # IMU数据
    def saveImuDataToMatFile(self):
        matDic = {'g': np.array([self.imuData['GyroX'], self.imuData['GyroY'], self.imuData['GyroZ']]).T,
                  'a': np.array([self.imuData['AccX'], self.imuData['AccY'], self.imuData['AccZ']]).T,
                  'temp': np.array([self.imuData['temperature']]).T, 'ts': np.array([self.imuData['time']]).T}
        return matDic

        # if self.saveFolderPath == "":
        #     folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        # else:
        #     folderPath = self.saveFolderPath
        # if not os.path.exists(folderPath):
        #     os.makedirs(folderPath)
        # newFilePath = folderPath + '/ImuData.mat'
        # savemat(newFilePath, matDic)  # 存成.mat文件格式

    # GPS数据
    def saveGpsDataToMatFile(self):
        gpsInsTime = [self.insData['time'][i] for i in self.gpsInsTimeIndex]

        matDic = {'Lon': np.array([self.gpsData['Lon']]).T, 'LonStd': np.array([self.gpsData['LonStd']]).T,
                  'Lat': np.array([self.gpsData['Lat']]).T, 'LatStd': np.array([self.gpsData['LatStd']]).T,
                  'hMSL': np.array([self.gpsData['hMSL']]).T, 'hMSLStd': np.array([self.gpsData['hMSLStd']]).T,
                  'gpsFix': np.array([self.gpsData['gpsFix']]).T, 'flags': np.array([self.gpsData['flags']]).T,
                  'HSpd': np.array([self.gpsData['HSpd']]).T, 'Trk': np.array([self.gpsData['TrackAngle']]).T,
                  'VSpd': np.array([self.gpsData['VSpd']]).T, 'LatencyVel': np.array([self.gpsData['LatencyVel']]).T,
                  'BaseLineLength': np.array([self.gpsData['BaseLineLength']]).T,
                  'heading': np.array([self.gpsData['Heading']]).T, 'cAcc': np.array([self.gpsData['HeadingStd']]).T,
                  'pitch': np.array([self.gpsData['Pitch']]).T, 'pitchStd': np.array([self.gpsData['PitchStd']]).T,
                  'RecMsg': np.array([self.gpsData['RecMsg']]).T, 'RecMsgBin': self.Unit2Bin(self.gpsData['RecMsg'], 8),
                  'NumSV': np.array([self.gpsData['numSV']]).T,
                  't': np.array([self.gpsData['year'], self.gpsData['month'], self.gpsData['day'], self.gpsData['hour'],
                                 self.gpsData['minute'], self.gpsData['second']]).T,
                  'itow_pos': np.array([self.gpsData['itow_pos']]).T,
                  'itow_vel': np.array([self.gpsData['itow_vel']]).T,
                  'itow_heading': np.array([self.gpsData['itow_heading']]).T, 'ts': np.array([gpsInsTime]).T,
                  'flagsPos': (np.array([self.gpsData['flags']]).T % 256),
                  'flagsVel': (np.array([self.gpsData['flags']]).T / 256) % 256,
                  'flagsHeading': (np.array([self.gpsData['flags']]).T / 256 / 256) % 256,
                  'flagsTime': (np.array([self.gpsData['flags']]).T / 256 / 256 / 256) % 256}

        return matDic

        # if self.saveFolderPath == "":
        #     folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        # else:
        #     folderPath = self.saveFolderPath
        # if not os.path.exists(folderPath):
        #     os.makedirs(folderPath)
        # newFilePath = folderPath + '/GpsData.mat'
        #
        # savemat(newFilePath, matDic)  # 存成.mat文件格式

    # 车辆数据
    def saveVehicleDataToMatFile(self):
        vehicleInsTime = [self.insData['time'][i] for i in self.vehicleInsTimeIndex]
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
                  'Shifter': np.array([self.vehicleData['gear']]).T,
                  'tsShifter': np.array([self.vehicleData['gearTime']]).T,
                  'flag': np.array([self.vehicleData['flag']]).T}
        return matDic

        # if self.saveFolderPath == "":
        #     folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        # else:
        #     folderPath = self.saveFolderPath
        # if not os.path.exists(folderPath):
        #     os.makedirs(folderPath)
        # newFilePath = folderPath + '/VehicleData.mat'
        #
        # savemat(newFilePath, matDic)  # 存成.mat文件格式

    # INS数据
    def saveInsDataToMatFile(self):
        matDic = {'angle': np.array([self.insData['roll'], self.insData['pitch'], self.insData['yaw']]).T,
                  'g': np.array([self.insData['GyroX'], self.insData['GyroY'], self.insData['GyroZ']]).T,
                  'a': np.array([self.insData['AccX'], self.insData['AccY'], self.insData['AccZ']]).T,
                  'pos': np.array([self.insData['lat'], self.insData['lon'], self.insData['height']]).T,
                  'v': np.array(
                      [self.insData['NorthVelocity'], self.insData['EastVelocity'], self.insData['GroundVelocity']]).T,
                  'IMUstatus': np.array([self.insData['IMUstatus']]).T,
                  'IMUstatusBin': self.Unit2Bin(self.insData['IMUstatus'], 8),
                  'LEKFstatus': np.array([self.insData['LEKFstatus']]).T,
                  'LEKFstatusBin': self.Unit2Bin(self.insData['LEKFstatus'], 32),
                  'GPSstatus': np.array([self.insData['GPSstatus']]).T,
                  'GPSstatusBin': self.Unit2Bin(self.insData['GPSstatus'], 8),
                  'P': np.array([self.insData['PData1'], self.insData['PData2'], self.insData['PData3']]).T,
                  'ts': np.array([self.insData['time']]).T, 'Ptype': np.array([self.insData['Ptype']]).T}
        return matDic
        # if self.saveFolderPath == "":
        #     folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        # else:
        #     folderPath = self.saveFolderPath
        # if not os.path.exists(folderPath):
        #     os.makedirs(folderPath)
        # newFilePath = folderPath + '/InsData.mat'
        #
        # savemat(newFilePath, matDic)  # 存成.mat文件格式

    # 同步时间数据
    def saveSyncDataToMatFile(self):
        syncInsTime = [self.insData['time'][i] for i in self.syncInsTimeIndex]

        matDic = {'timu': np.array([self.syncData['imuTime']]).T, 'tgps': np.array([self.syncData['gpsTime']]).T,
                  'ts': np.array([syncInsTime]).T}
        return matDic

        # if self.saveFolderPath == "":
        #     folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        # else:
        #     folderPath = self.saveFolderPath
        # if not os.path.exists(folderPath):
        #     os.makedirs(folderPath)
        # newFilePath = folderPath + '/SyncData.mat'
        #
        # savemat(newFilePath, matDic)  # 存成.mat文件格式

        # IMU2数据

    def saveImu2DataToMatFile(self):
        matDic = {'g': np.array([self.imu2Data['GyroX'], self.imu2Data['GyroY'], self.imu2Data['GyroZ']]).T,
                  'a': np.array([self.imu2Data['AccX'], self.imu2Data['AccY'], self.imu2Data['AccZ']]).T,
                  'temp': np.array([self.imu2Data['temperature']]).T,
                  'ts': np.array([self.imu2Data['time']]).T}
        return matDic

    def saveIns2DataToMatFile(self):
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
                  'LiOdometryStatu': np.array([self.ins2Data['LiOdometryStatus']]).T,
                  'sensor_status': np.array([self.ins2Data['sensor_status']]).T,
                  'temp_P': np.array([self.ins2Data['temp_P0'], self.ins2Data['temp_P1'], self.ins2Data['temp_P2']]).T,
                  'Index': np.array([self.ins2Data['Index']]).T,
                  'frame_id': np.array([self.ins2Data['frame_id']]).T,
                  'time': np.array([self.ins2Data['time']]).T
                  }
        return matDic

    def saveFourWSDataToMatFile(self):
        matDic = {'WheelF': np.array(
            [self.FourWSData['tsWheelF_s'], self.FourWSData['WheelFl'], self.FourWSData['WheelFr']]).T,
                  'WheelR': np.array(
                      [self.FourWSData['tsWheelR_s'], self.FourWSData['WheelRl'], self.FourWSData['WheelRr']]).T}
        return matDic

    # 轮循表数据
    def savePDataToMatFile(self):
        matDic = {}
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

    # 十进整数制转为二进制列表
    @staticmethod
    def Unit2Bin(num, unit):
        alllist = []
        for i in num:
            tobin = bin(i)[2:].rjust(unit, '0')
            binlist = list(map(int, reversed(list(tobin))))
            alllist.append(binlist)
        return alllist

    # 开始将数据存成.mat文件格式，总体整合
    def startSaveAllDataToMatFile(self):
        # self.saveImuDataToMatFile()
        # self.saveGpsDataToMatFile()
        # self.saveVehicleDataToMatFile()
        # self.saveInsDataToMatFile()
        # self.saveSyncDataToMatFile()
        veh = self.saveVehicleDataToMatFile()
        sync = self.saveSyncDataToMatFile()
        raw = self.saveImuDataToMatFile()
        gps = self.saveGpsDataToMatFile()
        ins = self.saveInsDataToMatFile()
        pdata = self.savePDataToMatFile()
        Imu2Dta = self.saveImu2DataToMatFile()
        Ins2Data = self.saveIns2DataToMatFile()
        FourWSData = self.saveFourWSDataToMatFile()
        ins['Pdata'] = pdata
        matDic = {'GPSINSData': {'GPSData': gps, 'INSData': ins, 'RawData': raw, 'SyncData': sync,
                                 'EnvisionCanData': veh, 'Imu2Dta': Imu2Dta, 'FourWSData': FourWSData,
                                 'Ins2Data': Ins2Data
                                 }}
        d_path = os.path.split(self.filePath)
        newFilePath = d_path[0] + '/' + d_path[1][:-4] + '_GPSINSData.mat'
        savemat(newFilePath, matDic, do_compression=True)  # 存成.mat文件格式

    @staticmethod
    def dict2csv(dic, filename):
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

    # 开始将GPSINS数据存成.cvs格式
    def SaveAllDataToCsvFile(self):
        dir_p = os.path.split(self.filePath)
        folderPath = dir_p[0] + '/' + dir_p[1][:-4] + '_CsvData'
        if not os.path.exists(folderPath):
            os.mkdir(folderPath)
        self.InsDataDF.to_csv(folderPath + '/InsData.csv')
        self.GpsDataDF.to_csv(folderPath + '/GpsData.csv')
        self.VehicleDataDF.to_csv(folderPath + '/vehicleData.csv')
        self.SyncDataDF.to_csv(folderPath + '/SyncData.csv')
        # self.dict2csv(self.PDataDict, folderPath + '/PDataDict.csv')

    # 打印输出数据帧统计结果
    def printDataFrameStatsResult(self):
        print("IMU数据帧：", "纯帧头数量:", self.dataFrameStats['imuFrameHeadNum_bdbd0a'], "总帧数量:",
              self.dataFrameStats['imuDataNum'], "错误帧数量:",
              len(self.dataFrameStats['imuCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['imuCheckErrIndex'])
        print("GPS数据帧：", "纯帧头数量:", self.dataFrameStats['gpsFrameHeadNum_bdbd10'], "总帧数量:",
              self.dataFrameStats['gpsDataNum'], "错误帧数量:",
              len(self.dataFrameStats['gpsCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['gpsCheckErrIndex'])
        print("车辆数据帧：", "纯帧头数量:", self.dataFrameStats['vehicleFrameHeadNum_bdbd20'], "总帧数量:",
              self.dataFrameStats['vehicleDataNum'], "错误帧数量:",
              len(self.dataFrameStats['vehicleCheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['vehicleCheckErrIndex'])
        print("INS数据帧：", "纯帧头数量:", self.dataFrameStats['insFrameHeadNum_bdbd0b'], "总帧数量:",
              self.dataFrameStats['insDataNum'], "错误帧数量:",
              len(self.dataFrameStats['insCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['insCheckErrIndex'])
        print("同步时间数据帧：", "纯帧头数量:", self.dataFrameStats['syncFrameHeadNum_bdbd0c'], "总帧数量:",
              self.dataFrameStats['syncDataNum'], "错误帧数量:",
              len(self.dataFrameStats['syncCheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['syncCheckErrIndex'])
        print("IMU2数据帧：", "纯帧头数量:", self.dataFrameStats['imu2FrameHeadNum_bdbd2a'], "总帧数量:",
              self.dataFrameStats['imu2DataNum'], "错误帧数量:",
              len(self.dataFrameStats['imu2CheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['imu2CheckErrIndex'])
        print("INS2数据帧：", "纯帧头数量:", self.dataFrameStats['ins2FrameHeadNum_bdbd1b'], "总帧数量:",
              self.dataFrameStats['ins2DataNum'], "错误帧数量:",
              len(self.dataFrameStats['ins2CheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['ins2CheckErrIndex'])
        print("FourWS数据帧：", "纯帧头数量:", self.dataFrameStats['FourWSFrameHeadNum_bdbd30'], "总帧数量:",
              self.dataFrameStats['FourWSDataNum'], "错误帧数量:",
              len(self.dataFrameStats['FourWSCheckErrIndex']), "错误帧索引下标:",
              self.dataFrameStats['FourWSCheckErrIndex'])

    # 找到所有数据帧
    def findAllDataFrame(self):
        frame0A = []
        frame10 = []
        frame20 = []
        frame0B = []
        frame0C = []
        frame2A = []
        frame1B = []
        frame30 = []
        flag = True
        indexIns = 0
        self.gpsInsTimeIndex = []
        self.vehicleInsTimeIndex = []
        self.syncInsTimeIndex = []
        frames = re.findall(
            r'bddb0a.{62}|bddb10.{134}|bddb20.{62}|bddb0b.{110}|bddb0c.{18}|bddb2a.{62}|bddb1b.{132}|bddb30.{34}|bddb06.{38}',
            self.fileHexData)
        for frame in frames:
            head = frame[:6]
            if head == 'bddb0a':
                if not self.myXORCheck(frame):  # 数据校验有问题
                    self.dataFrameStats['imuCheckErrIndex'].append(
                        self.dataFrameStats['imuDataNum'])  # 统计校验有问题的数据帧的序列号(下标)
                    continue
                self.dataFrameStats['imuDataNum'] += 1
                frame0A.append(frame)
            elif head == 'bddb10':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['gpsCheckErrIndex'].append(self.dataFrameStats['gpsDataNum'])
                    continue
                self.dataFrameStats['gpsDataNum'] += 1
                self.gpsInsTimeIndex.append(indexIns)
                frame10.append(frame)
            elif head == 'bddb20':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['vehicleCheckErrIndex'].append(self.dataFrameStats['vehicleDataNum'])
                    continue
                self.dataFrameStats['vehicleDataNum'] += 1
                self.vehicleInsTimeIndex.append(indexIns)
                frame20.append(frame)
            elif head == 'bddb0b':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['insCheckErrIndex'].append(self.dataFrameStats['insDataNum'])
                    continue
                indexIns += 1
                if not flag:
                    pass
                else:  # 首次进入
                    indexIns = 0
                    flag = False
                self.dataFrameStats['insDataNum'] += 1
                frame0B.append(frame)
            elif head == 'bddb0c':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['syncCheckErrIndex'].append(self.dataFrameStats['syncDataNum'])
                    continue
                self.dataFrameStats['syncDataNum'] += 1
                self.syncInsTimeIndex.append(indexIns)
                frame0C.append(frame)
            elif head == 'bddb30':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['FourWSCheckErrIndex'].append(self.dataFrameStats['FourWSDataNum'])
                    continue
                self.dataFrameStats['FourWSDataNum'] += 1
                frame30.append(frame)
            elif head == 'bddb1b':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['ins2CheckErrIndex'].append(self.dataFrameStats['ins2DataNum'])
                    continue
                self.dataFrameStats['ins2DataNum'] += 1
                frame1B.append(frame)
            elif head == 'bddb2a':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['imu2CheckErrIndex'].append(self.dataFrameStats['imu2DataNum'])
                    continue
                self.dataFrameStats['imu2DataNum'] += 1
                frame2A.append(frame)

        self.imuDataFrame = frame0A
        self.gpsDataFrame = frame10
        self.vehicleDataFrame = frame20
        self.insDataFrame = frame0B
        self.syncDataFrame = frame0C
        self.imu2DataFrame = frame2A
        self.ins2DataFrame = frame1B
        self.FourWSDataFrame = frame30
        # 减少内存占用
        self.gpsInsTimeIndex = np.array(self.gpsInsTimeIndex)
        self.vehicleInsTimeIndex = np.array(self.vehicleInsTimeIndex)
        self.syncInsTimeIndex = np.array(self.syncInsTimeIndex)
        self.imu2TimeIndex = np.array(self.imu2TimeIndex)
        self.ins2TimeIndex = np.array(self.ins2TimeIndex)
        self.FourWSTimeIndex = np.array(self.FourWSTimeIndex)


if __name__ == "__main__":
    # original_file_path = r"D:\Files\dbFiles\12302.txt" D:\Downloads\ins.log
    # main_path = r'D:\Files\pyFiles\DataParse_0822\main.exe'

    main_path = input("请输入 仿真程序 路径（.exe）[注意：本exe需放在仿真程序同一文件夹下]：")
    original_file_path = input("请输入 仿真文件 路径（.txt或.log）：")

    main_name = main_path.split('\\')[-1][:-4]
    simu_file_name = original_file_path[:-4] + '_' + main_name + '_simu.txt'
    config_path = '\\'.join(main_path.split('\\')[:-1] + ['config.txt'])

    if os.path.exists(config_path):
        os.remove(config_path)

    try:
        # Compute bpos
        obj = HexDataParse()
        obj.filePath = original_file_path
        print(time.strftime('%H:%M:%S', time.localtime()), "开始解析数据: ", obj.filePath)
        obj.startParseFileHexData()  # 开始数据解析
        print(time.strftime('%H:%M:%S', time.localtime()), "解析完成...")
        obj.saveDataToDF()

        Lbbg = obj.PDataDict['Lbbg'][0]
        Lbbc = obj.PDataDict['Lbbc'][0]
        Atttg = obj.PDataDict['Atttg'][0]
        Align = obj.PDataDict['Align'][0]
        print('杆臂值为：\nLbbg: %s\nLbbc: %s\nAtttg: %s\nAlign: %s\n' % (str(Lbbg), str(Lbbc), str(Atttg), str(Align)))
    except Exception as e:
        print(e)
        print('无法解析杆臂值')
        print('程序将在 1min 后退出 ！')
        time.sleep(60)

    # generate config.txt
    with open(config_path, encoding='utf-8', mode='w+') as config_file:
        O1 = 'Original =' + original_file_path
        S1 = 'Simulation =' + simu_file_name
        config_file.writelines(O1 + "\n")
        config_file.writelines(S1 + "\n")

        config_file.writelines('ltg=' + ';'.join([str(i) for i in Lbbg]) + "\n")
        config_file.writelines('ltb=' + ';'.join([str(i) for i in Lbbc]) + "\n")
        config_file.writelines('atg=' + ';'.join([str(i) for i in Atttg]) + "\n")
        config_file.writelines('atb=' + ';'.join([str(i) for i in Align]) + "\n")

        config_file.close()
    print(time.strftime('%H:%M:%S', time.localtime()), "获得杆臂值于config文件(请勿手动删除)： %s" % config_path)

    print(time.strftime('%H:%M:%S', time.localtime()), "开始仿真...")
    # res = os.system(exe_path)
    rc, out = subprocess.getstatusoutput(main_path)
    print(time.strftime('%H:%M:%S', time.localtime()), "仿真完成...")
    print(rc)
    print('*' * 10)
    print(out)

    print('程序将在 30s 后退出 ！')
    time.sleep(30)
    print('haha')
