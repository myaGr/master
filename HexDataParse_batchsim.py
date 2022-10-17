# -*- coding: UTF-8 -*-
# @Author   : fsw
# @Date     : 2022/2/25

import re  # 正则表达式
import struct  # 十六进制数据解析
import pandas as pd
import os  # 处理文件路径、文件名、创建文件夹等
# from memory_profiler import profile  # 内存占用分析
from scipy.io import savemat  # 存成.mat文件格式
import numpy as np
from threading import Thread  # 多线程
from pathlib import Path
import subprocess
import time
import threading
#import mulitiprocess


class HexDataParse(object):
    def __init__(self):
        # 数据解析
        Thread.__init__(self)
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

        self.imu2DataFrame=None  #2Aimu数据帧
        self.imu2Data=None

        self.ins2DataFrame=None   #1Bins数据
        self.ins2Data=None

        self.SFDataFrame = None  # 0ESF数据
        self.SFData = None

        self.dataFrameStats = {'imuDataNum': 0, 'imuCheckErrIndex': [], 'gpsDataNum': 0, 'gpsCheckErrIndex': [],
                               'vehicleDataNum': 0, 'vehicleCheckErrIndex': [], 'insDataNum': 0, 'insCheckErrIndex': [],
                               'syncDataNum': 0, 'syncCheckErrIndex': [], 'imuFrameHeadNum_bdbd0a': 0,
                               'gpsFrameHeadNum_bdbd10': 0, 'vehicleFrameHeadNum_bdbd20': 0,
                               'insFrameHeadNum_bdbd0b': 0,
                               'syncFrameHeadNum_bdbd0c': 0,'imu2DataNum':0,'imu2CheckErrIndex':[],
                               'ins2DataNum':0,'ins2CheckErrIndex':[],
                               'imu2FrameHeadNum_bdbd2a': 0, 'ins2FrameHeadNum_bdbd1b': 0,
                                'SFDataNum':0, 'SFCheckErrIndex': [],
                               'SFFrameHeadNum_bdbd0e': 0
                               }  # DataNum：数据个数，CheckErr：校验错误数据帧的序列号(下标)，FrameHeadNum：帧头数，只匹配帧头得出

        # 数据存储
        self.saveFolderPath = ""  # 存储文件夹路径，初始值要是“”，后面会用此作为判断条件，表明是否使用自定义文件夹
        self.frameHeadStrLst = None  # 各帧头字符串列表
        self.gpsInsTimeIndex = None  # GPS数据系统时间，使用上一帧INS时间，此处记录的是INS时间数据的下标
        self.vehicleInsTimeIndex = None  # 车辆数据系统时间
        self.syncInsTimeIndex = None  # 同步时间数据系统时间
        self.imu2TimeIndex = None  # IMU2时间数据的下标
        self.ins2TimeIndex = None  # INS2时间数据系统时间

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

    # 读取文件数据，以十六进制形式
    def readFileData(self):
        with open(self.filePath, 'rb') as f:
            self.fileHexData = f.read().hex()  # hex()函数：bytes转hex字符串

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

    # 找到IMU数据帧
    def findIMUDataFrame(self):
        self.imuDataFrame = re.findall(r'bddb0a.{62}', self.fileHexData)  # IMU数据帧头是BD DB 0A，包括帧头一共34个字节，一个字节用2个十六进制数表示

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
                dataDic[key].append(data[i])
                i += 1

        self.imuDataFrame = None  # 释放内存
        self.imuData = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

    # 找到GPS数据帧
    def findGPSDataFrame(self):
        self.gpsDataFrame = re.findall(r'bddb10.{132}', self.fileHexData)  # GPS数据帧头是BD DB 10，包括帧头一共70个字节

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

        return data

    # 解析GPS数据
    def parseGPSData(self):
        dataDic = {'Lon': [], 'LonStd': [], 'Lat': [], 'LatStd': [], 'hMSL': [], 'hMSLStd': [], 'gpsFix': [],
                   'flags': [], 'HSpd': [], 'TrackAngle': [], 'VSpd': [], 'LatencyVel': [], 'BaseLineLength': [],
                   'heading': [], 'headingStd': [], 'pitch': [], 'pitchStd': [], 'year': [], 'month': [],
                   'day': [], 'hour': [], 'minute': [], 'second': [], 'itow_pos': [], 'itow_vel': [],
                   'itow_heading': [], 'RecMsg': [], 'numSV': []}

        for dataPkg in self.gpsDataFrame:
            # if not self.myXORCheck(dataPkg):
            #     self.dataFrameStats['gpsCheckErrIndex'].append(self.dataFrameStats['gpsDataNum'])
            #     self.dataFrameStats['gpsDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['gpsDataNum'] += 1

            data = self.parseGPSDataOneFrame(dataPkg)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i+= 1

        self.gpsDataFrame = None  # 释放内存
        self.gpsData = pd.DataFrame(dataDic)

    # 找到车辆数据帧
    def findVehicleDataFrame(self):
        self.vehicleDataFrame = re.findall(r'bddb20.{60}', self.fileHexData)  # 车辆数据帧头是BD DB 20，包括帧头一共34个字节

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
                dataDic[key].append(data[i])
                i += 1

        self.vehicleDataFrame = None  # 释放内存
        self.vehicleData = pd.DataFrame(dataDic)

    # 找到INS数据帧
    def findINSDataFrame(self):
        self.insDataFrame = re.findall(r'bddb0b.{110}', self.fileHexData)  # INS数据帧头是BD DB 0B，包括帧头一共58个字节

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
                dataDic[key].append(data[i])
                i += 1

        self.insDataFrame = None
        self.insData = pd.DataFrame(dataDic)

    # 找到同步时间数据帧
    def findSyncDataFrame(self):
        self.syncDataFrame = re.findall(r'bddb0c.{18}', self.fileHexData)  # 同步时间数据帧头是BD DB 0C，包括帧头一共12个字节

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
                dataDic[key].append(data[i])
                i += 1

        self.syncDataFrame = None
        self.syncData = pd.DataFrame(dataDic)

    # 找到IMU2数据帧
    def findIMU2DataFrame(self):
        self.imuDataFrame = re.findall(r'bddb2a.{62}', self.fileHexData)  # IMU数据帧头是BD DB 0A，包括帧头一共34个字节，一个字节用2个十六进制数表示

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

    def parseINS2DataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次当前姿态与初始时刻姿态做差转出的四元数,odom坐标系下的x,y,z,odom坐标系下的线速度（前，左，上),
        # x y z轴分速度的合速,自车坐标系下的三轴角速度、加速度，系统状态，传感器状态，轮询位三字节，轮询标志位，时间
        data = list(struct.unpack('<7i10hBi3h2BI', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
        data[0] *= 1e-9 #四元数
        data[1] *= 1e-9 #四元数
        data[2] *= 1e-9 #四元数
        data[3] *= 1e-9 #四元数
        data[4] *= 1e-3 #odom坐标系下的x,y,z
        data[5] *= 1e-3
        data[6] *= 1e-3
        data[7] *= 100.0 / 32768.0 #odom坐标系下的线速度，方向同position（前，左，上)，以及 x y z轴分速度的合速度
        data[8] *= 100.0 / 32768.0
        data[9] *= 100.0 / 32768.0
        data[10] *= 100.0 / 32768.0
        data[11] *= 300.0 / 32768.0     # 车坐标系下的角速度
        data[12] *= 300.0 / 32768.0
        data[13] *= 300.0 / 32768.0
        data[14] *= 12.0 / 32768.0      # 车坐标系下的加速度
        data[15] *= 12.0 / 32768.0
        data[16] *= 12.0 / 32768.0
        data[24] /= sysTimeFactor       # 系统时间

        return data

        # 解析INS2数据帧

    def parseINS2Data(self):
        dataDic = {"Quaternion1": [], "Quaternion2": [], "Quaternion3": [], "Quaternion4": [],
                   "OdomX": [], "OdomY": [], "OdomZ": [],  "OdomVX": [],
                   "OdomVY": [], "OdomVZ": [], "OdomVM": [], "OdomAngular_VX": [],
                   "OdomAngular_VY": [], "OdomAngular_VZ": [],"OdomAcceleration_X": [], "OdomAcceleration_Y": [],
                   "OdomAcceleration_Z": [],"LiOdometryStatus":[],"sensor_status":[],"temp_P0":[],
                   "temp_P1":[],"temp_P2":[],"Index":[], "frame_id":[],"time":[]
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

    # 找到SF数据帧
    def parseSFDataOneFrame(self, dataHexStr, sysTimeFactor=4000):
        # 数据依次当前姿态与初始时刻姿态做差转出的四元数,odom坐标系下的x,y,z,odom坐标系下的线速度（前，左，上),
        # x y z轴分速度的合速,自车坐标系下的三轴角速度、加速度，系统状态，传感器状态，轮询位三字节，轮询标志位，时间
        data = list(struct.unpack('<18h', bytes.fromhex(dataHexStr[2 * 3:-2])))  # 跳过帧头和末尾的校验位，<是小端，>是大端
        data[0] *= 360.0 / 32768.0 # 姿态 DiffAtt_deg
        data[1] *= 360.0 / 32768.0 # 姿态
        data[2] *= 360.0 / 32768.0 # 姿态
        data[3] *= 100.0 / 32768.0 # 速度  DiffVel_NED_mps
        data[4] *= 100.0 / 32768.0 # 速度
        data[5] *= 100.0 / 32768.0 # 速度
        data[6] *= 100.0 / 32768.0 # 位置  DiffPos_m
        data[7] *= 100.0 / 32768.0 # 位置
        data[8] *= 100.0 / 32768.0 # 位置
        data[9] *= 360.0 / 32768.0 # 姿态DiffAtt_deg_pureins
        data[10] *= 360.0 / 32768.0  # 姿态DiffAtt_deg_pureins
        data[11] *= 360.0 / 32768.0   # 姿态DiffAtt_deg_pureins
        data[12] *= 100.0 / 32768.0   # 速度  DiffVel_NED_mps_pureins
        data[13] *= 100.0 / 32768.0   # 速度
        data[14] *= 100.0 / 32768.0   # 速度
        data[15] *= 100.0 / 32768.0  # 位置  DiffPos_m_pureins
        data[16] *= 100.0 / 32768.0  # 位置
        data[17] *= 100.0 / 32768.0  # 位置

        return data

        # 解析SF数据帧

    def parseSFData(self):
        dataDic = {"DiffAtt1": [], "DiffAtt2": [], "DiffAtt3": [],
                   "DiffV1": [], "DiffV2": [], "DiffV3": [],
                   "DiffPos1": [], "DiffPos2": [], "DiffPos3": [],
                   "DiffAtt1_p": [], "DiffAtt2_p": [], "DiffAtt3_p": [],
                   "DiffV1_p": [], "DiffV2_p": [], "DiffV3_p": [],
                   "DiffPos1_p": [], "DiffPos2_p": [], "DiffPos3_p": []
                   }
        for dataPkg in self.SFDataFrame:
            # if not self.myXORCheck(dataPkg):  # 数据校验有问题
            #     self.dataFrameStats['imuCheckErrIndex'].append(self.dataFrameStats['imuDataNum'])  # 统计校验有问题的数据帧的序列号(下标)
            #     self.dataFrameStats['imuDataNum'] += 1
            #     continue
            #
            # self.dataFrameStats['imuDataNum'] += 1

            data = self.parseSFDataOneFrame(dataPkg, self.sysTimeFactor)

            i = 0
            for key in dataDic:
                dataDic[key].append(data[i])
                i += 1

        self.SFDatafram = None  # 释放内存
        self.SFData = pd.DataFrame(dataDic)  # 以pandas数据结构来存储数据，占用内存空间更少

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
        self.parseSFData()

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
        frames = re.findall(r'bddb0a.{62}|bddb10.{134}|bddb20.{62}|bddb0b.{110}|bddb0c.{18}|bddb2a.{62}|bddb1b.{132}|bddb0e.{74}', self.fileHexData)
        self.frameHeadStrLst = [frame[:6] for frame in frames]

    # 各帧头数量统计
    def dataFrameHeadNumStats(self):
        self.dataFrameStats['imuFrameHeadNum_bdbd0a'] = len(re.findall(r'bddb0a', self.fileHexData))
        self.dataFrameStats['gpsFrameHeadNum_bdbd10'] = len(re.findall(r'bddb10', self.fileHexData))
        self.dataFrameStats['vehicleFrameHeadNum_bdbd20'] = len(re.findall(r'bddb20', self.fileHexData))
        self.dataFrameStats['insFrameHeadNum_bdbd0b'] = len(re.findall(r'bddb0b', self.fileHexData))
        self.dataFrameStats['syncFrameHeadNum_bdbd0c'] = len(re.findall(r'bddb0c', self.fileHexData))
        self.dataFrameStats['ins2FrameHeadNum_bdbd1b'] = len(re.findall(r'bddb1b', self.fileHexData))
        self.dataFrameStats['imu2FrameHeadNum_bdbd2a'] = len(re.findall(r'bddb2a', self.fileHexData))
        self.dataFrameStats['SFFrameHeadNum_bdbd0e'] = len(re.findall(r'bddb0e', self.fileHexData))

    # 生成GPS、车辆、同步时间数据的INS数据的下标
    def generateDataInsTimeIndex(self):
        self.gpsInsTimeIndex = []
        self.vehicleInsTimeIndex = []
        self.syncInsTimeIndex = []

        frameIndex = [0, 0, 0, 0,0,0,0]  # 数据帧索引
        i = 0  # 解析后的值索引
        flag = True
        for head in self.frameHeadStrLst:
            if head == 'bddb0b':  # INS帧
                if frameIndex[0] not in self.dataFrameStats['insCheckErrIndex']:
                    i += 1
                    if flag == True:  # 首次进入
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
            elif head == 'bddb2a':  # IMU2帧
                if frameIndex[4] not in self.dataFrameStats['imu2CheckErrIndex']:
                    self.imu2TimeIndex.append(i)
                frameIndex[4] += 1
            elif head == 'bddb1b':  # INS2帧
                if frameIndex[5] not in self.dataFrameStats['ins2CheckErrIndex']:
                    self.ins2TimeIndex.append(i)
                frameIndex[5] += 1
            elif head == 'bddb0e':  # SF帧
                if frameIndex[6] not in self.dataFrameStats['SFCheckErrIndex']:
                    self.SFTimeIndex.append(i)
                frameIndex[6] += 1

        # 减少内存占用
        self.frameHeadStrLst = None
        self.gpsInsTimeIndex = np.array(self.gpsInsTimeIndex)
        self.vehicleInsTimeIndex = np.array(self.vehicleInsTimeIndex)
        self.imu2TimeIndex = np.array(self.imu2TimeIndex)
        self.ins2TimeIndex = np.array(self.ins2TimeIndex)


    # 将数据存成.mat文件格式
    # IMU数据
    def saveImuDataToMatFile(self):
        matDic = {'g': np.array([self.imuData['GyroX'], self.imuData['GyroY'], self.imuData['GyroZ']]).T,
                  'a': np.array([self.imuData['AccX'], self.imuData['AccY'], self.imuData['AccZ']]).T,
                  'temp': np.array([self.imuData['temperature']]).T, 'ts': np.array([self.imuData['time']]).T}

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/ImuData.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

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
                  'heading': np.array([self.gpsData['heading']]).T, 'cAcc': np.array([self.gpsData['headingStd']]).T,
                  'pitch': np.array([self.gpsData['pitch']]).T, 'pitchStd': np.array([self.gpsData['pitchStd']]).T,
                  'RecMsg': np.array([self.gpsData['RecMsg']]).T, 'NumSV': np.array([self.gpsData['numSV']]).T,
                  't': np.array([self.gpsData['year'], self.gpsData['month'], self.gpsData['day'], self.gpsData['hour'],
                                 self.gpsData['minute'], self.gpsData['second']]).T,
                  'itow_pos': np.array([self.gpsData['itow_pos']]).T,
                  'itow_vel': np.array([self.gpsData['itow_vel']]).T,
                  'itow_heading': np.array([self.gpsData['itow_heading']]).T,
                  'ts': np.array([gpsInsTime]).T}

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/GpsData.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

    # 车辆数据
    def saveVehicleDataToMatFile(self):
        vehicleInsTime = [self.insData['time'][i] for i in self.vehicleInsTimeIndex]

        matDic = {'ts': np.array([vehicleInsTime]).T,
                  'tsWheelAngle': np.array([self.vehicleData['WheelAngleTime']]).T,
                  'WheelAngle': np.array([self.vehicleData['WheelAngle']]).T,
                  'VehSpdDriL': np.array([self.vehicleData['WheelSpeedFrontLeft']]).T,
                  'VehSpdDriR': np.array([self.vehicleData['WheelSpeedFrontRight']]).T,
                  'tsVehSpdDri': np.array([self.vehicleData['WheelSpeedFrontTime']]).T,
                  'VehSpdNonDriL': np.array([self.vehicleData['WheelSpeedBackLeft']]).T,
                  'VehSpdNonDriR': np.array([self.vehicleData['WheelSpeedBackRight']]).T,
                  'tsVehSpdNonDri': np.array([self.vehicleData['WheelSpeedBackTime']]).T,
                  'Shifter': np.array([self.vehicleData['gear']]).T,
                  'tsShifter': np.array([self.vehicleData['gearTime']]).T,
                  'flag': np.array([self.vehicleData['flag']]).T
                  }

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/VehicleData.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

    # INS数据
    def saveInsDataToMatFile(self):
        matDic = {'angle': np.array([self.insData['roll'], self.insData['pitch'], self.insData['yaw']]).T,
                  'g': np.array([self.insData['GyroX'], self.insData['GyroY'], self.insData['GyroZ']]).T,
                  'a': np.array([self.insData['AccX'], self.insData['AccY'], self.insData['AccZ']]).T,
                  'pos': np.array([self.insData['lat'], self.insData['lon'], self.insData['height']]).T,
                  'v': np.array(
                      [self.insData['NorthVelocity'], self.insData['EastVelocity'], self.insData['GroundVelocity']]).T,
                  'IMUstatus': np.array([self.insData['IMUstatus']]).T,
                  'LEKFstatus': np.array([self.insData['LEKFstatus']]).T,
                  'GPSstatus': np.array([self.insData['GPSstatus']]).T,
                  'P': np.array([self.insData['PData1'], self.insData['PData2'], self.insData['PData3']]).T,
                  'ts': np.array([self.insData['time']]).T, 'Ptype': np.array([self.insData['Ptype']]).T}

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/InsData.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

    # 同步时间数据
    def saveSyncDataToMatFile(self):
        syncInsTime = [self.insData['time'][i] for i in self.syncInsTimeIndex]

        matDic = {'timu': np.array([self.syncData['imuTime']]).T, 'tgps': np.array([self.syncData['gpsTime']]).T,
                  'ts': np.array([syncInsTime]).T}

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/SyncData.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

        # IMU2数据
    def saveImu2DataToMatFile(self):
        matDic = {'g': np.array([self.imu2Data['GyroX'], self.imu2Data['GyroY'], self.imu2Data['GyroZ']]).T,
                  'a': np.array([self.imu2Data['AccX'], self.imu2Data['AccY'], self.imu2Data['AccZ']]).T,
                  'temp': np.array([self.imu2Data['temperature']]).T, 'ts': np.array([self.imu2Data['time']]).T}

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/Imu2Data.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

    def saveIns2DataToMatFile(self):

        matDic = {'Quaternion': np.array([self.ins2Data['Quaternion1'], self.ins2Data['Quaternion2'], self.ins2Data['Quaternion3'], self.ins2Data['Quaternion4']]).T,
                  'Odom_Position':np.array([self.ins2Data['OdomX'], self.ins2Data['OdomY'], self.ins2Data['OdomZ']]).T,
                  'Odom_V':np.array([self.ins2Data['OdomVX'], self.ins2Data['OdomVY'], self.ins2Data['OdomVZ'],self.ins2Data['OdomVM']]).T,
                  'Odom_AngV': np.array([self.ins2Data['OdomAngular_VX'], self.ins2Data['OdomAngular_VY'], self.ins2Data['OdomAngular_VZ']]).T,
                  'Odom_Acc': np.array([self.ins2Data['OdomAcceleration_X'], self.ins2Data['OdomAcceleration_Y'],self.ins2Data['OdomAcceleration_Z']]).T,
                  'LiOdometryStatu': np.array([self.ins2Data['LiOdometryStatus']]).T,
                  'sensor_status': np.array([self.ins2Data['sensor_status']]).T,
                  'temp_P':np.array([self.ins2Data['temp_P0'],self.ins2Data['temp_P1'],self.ins2Data['temp_P2']]).T,
                  'Index': np.array([self.ins2Data['Index']]).T,
                  'frame_id': np.array([self.ins2Data['frame_id']]).T,
                  'time': np.array([self.ins2Data['time']]).T
                  }

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/Ins2Data.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

    def saveSFDataToMatFile(self):

        # "DiffAtt1": [], "DiffAtt2": [], "DiffAtt3": [],
        # "DiffV1": [], "DiffV2": [], "DiffV3": [],
        # "DiffPos1": [], "DiffPos2": [], "DiffPos3": [],
        # "DiffAtt1_p": [], "DiffAtt2_p": [], "DiffAtt3_p": [],
        # "DiffV1_p": [], "DiffV2_p": [], "DiffV3_p": [],
        # "DiffPos1_p": [], "DiffPos2_p": [], "DiffPos3_p": []

        matDic = {'DiffAtt1': np.array([self.SFData['DiffAtt1'], self.SFData['DiffAtt2'], self.SFData['DiffAtt3']]).T,
                  'DiffV1': np.array([self.SFData['DiffV1'], self.SFData['DiffV2'], self.SFData['DiffV3']]).T,
                  'DiffPos1': np.array([self.SFData['DiffPos1'], self.SFData['DiffPos2'], self.SFData['DiffPos3']]).T,
                  'DiffAtt2': np.array([self.SFData['DiffAtt1_p'], self.SFData['DiffAtt2_p'], self.SFData['DiffAtt3_p']]).T,
                  'DiffV2': np.array([self.SFData['DiffV1_p'], self.SFData['DiffV2_p'], self.SFData['DiffV3_p']]).T,
                  'DiffPos2': np.array([self.SFData['DiffPos1_p'], self.SFData['DiffPos2_p'], self.SFData['DiffPos3_p']]).T
                  }

        if self.saveFolderPath == "":
            folderPath = os.path.dirname(self.filePath) + '/Unpack_' + os.path.basename(self.filePath).split('.')[0]
        else:
            folderPath = self.saveFolderPath
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        newFilePath = folderPath + '/SFData.mat'

        savemat(newFilePath, matDic)  # 存成.mat文件格式

    # 开始将数据存成.mat文件格式，总体整合
    def startSaveAllDataToMatFile(self):
        self.saveImuDataToMatFile()
        self.saveGpsDataToMatFile()
        self.saveVehicleDataToMatFile()
        self.saveInsDataToMatFile()
        self.saveSyncDataToMatFile()
        self.saveImu2DataToMatFile()
        self.saveIns2DataToMatFile()
        self.saveSFDataToMatFile()
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
              len(self.dataFrameStats['vehicleCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['vehicleCheckErrIndex'])
        print("INS数据帧：", "纯帧头数量:", self.dataFrameStats['insFrameHeadNum_bdbd0b'], "总帧数量:",
              self.dataFrameStats['insDataNum'], "错误帧数量:",
              len(self.dataFrameStats['insCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['insCheckErrIndex'])
        print("同步时间数据帧：", "纯帧头数量:", self.dataFrameStats['syncFrameHeadNum_bdbd0c'], "总帧数量:",
              self.dataFrameStats['syncDataNum'], "错误帧数量:",
              len(self.dataFrameStats['syncCheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['syncCheckErrIndex'])
        print("IMU2数据帧：", "纯帧头数量:", self.dataFrameStats['imu2FrameHeadNum_bdbd2a'], "总帧数量:",
              self.dataFrameStats['imu2DataNum'], "错误帧数量:",
              len(self.dataFrameStats['imu2CheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['imu2CheckErrIndex'])
        print("INS2数据帧：", "纯帧头数量:", self.dataFrameStats['ins2FrameHeadNum_bdbd1b'], "总帧数量:",
              self.dataFrameStats['ins2DataNum'], "错误帧数量:",
              len(self.dataFrameStats['ins2CheckErrIndex']), "错误帧索引下标:", self.dataFrameStats['ins2CheckErrIndex'])

    # 找到所有数据帧
    def findAllDataFrame(self):
        frame0A = []
        frame10 = []
        frame20 = []
        frame0B = []
        frame0C = []
        frame2A = []
        frame1B = []
        frame0E = []
        flag = True
        indexIns = 0
        self.gpsInsTimeIndex = []
        self.vehicleInsTimeIndex = []
        self.syncInsTimeIndex = []
        frames = re.findall(r'bddb0a.{62}|bddb10.{134}|bddb20.{62}|bddb0b.{110}|bddb0c.{18}|bddb2a.{62}|bddb1b.{132}|bddb0e.{74}', self.fileHexData)
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
                if flag == True:  # 首次进入
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
            elif head == 'bddb2a':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['imu2CheckErrIndex'].append(self.dataFrameStats['imu2DataNum'])
                    continue
                self.dataFrameStats['imu2DataNum'] += 1
                frame2A.append(frame)
            elif head == 'bddb1b':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['ins2CheckErrIndex'].append(self.dataFrameStats['ins2DataNum'])
                    continue
                self.dataFrameStats['ins2DataNum'] += 1
                frame1B.append(frame)
            elif head == 'bddb0e':
                if not self.myXORCheck(frame):
                    self.dataFrameStats['SFCheckErrIndex'].append(self.dataFrameStats['SFDataNum'])
                    continue
                self.dataFrameStats['SFDataNum'] += 1
                frame0E.append(frame)

        self.imuDataFrame = frame0A
        self.gpsDataFrame = frame10
        self.vehicleDataFrame = frame20
        self.insDataFrame = frame0B
        self.syncDataFrame = frame0C
        self.imu2DataFrame = frame2A
        self.ins2DataFrame = frame1B
        self.SFDataFrame = frame0E
        # 减少内存占用
        self.gpsInsTimeIndex = np.array(self.gpsInsTimeIndex)
        self.vehicleInsTimeIndex = np.array(self.vehicleInsTimeIndex)
        self.syncInsTimeIndex = np.array(self.syncInsTimeIndex)
        self.imu2TimeIndex = np.array(self.imu2TimeIndex)
        self.ins2TimeIndex = np.array(self.ins2TimeIndex)

    #创建多线程数据解析函数
    def ParseThreadFunc(self):
        print("开始数据解析...")
        self.startParseFileHexData()
        print("数据解析完成...")
        print("开始存成mat文件...")
        self.startSaveAllDataToMatFile()
        print("存成mat文件完成...")
        self.printDataFrameStatsResult()  # 打印数据帧统计结果

    # 创建数据解析线程
    def createDataParseThread(self):
        t = Thread(target=self.ParseThreadFunc)  # 创建一个线程
        t.setDaemon(True)  # 设置守护线程，如果进程结束，会自动去结束线程
        t.start()  # 启动线程

def getconfig1(configall,filepath,filename):
    f = open(configall, encoding='utf-8', mode='r')
    file = f.readlines()
    #f2= open('config.txt', encoding='utf-8', mode='w')
    outfile=filepath+'//config.txt'
    with open(outfile, encoding='utf-8', mode='w') as f2:
        for i in range(len(file)):
            if re.search(filename,file[i]):
                confige=file[i:i+6]
                #print(confige)
                f2.writelines(confige)
                break

def getconfig2(Larm_path,filepath,filename): #
    confige= []
    outfile=filepath+'//'+'config.txt'
    #f2= open('config.txt', encoding='utf-8', mode='w')
    f = open(Larm_path, encoding='utf-8', mode='r')
    Larm = f.readlines()
    if filename.find('.txt') >= 0 or filename.find('.log') >= 0 or filename.find('.INS') >= 0 or filename.find('.dat') >= 0:
        with open(outfile, encoding='utf-8', mode='w+') as f2:
            for i in range(len(Larm)):
                if re.search("ltg",Larm[i]):
                    O1='Original ='+filepath+'\\data\\'+filename
                    S1='Simulation =' + filepath + '\\data\\' + filename[:-4] +'_sim.txt'
                    f2.writelines(O1+"\n")
                    f2.writelines(S1+"\n")
                    f2.writelines(Larm[i:i+4])
                break


def getconfig3(Larm_path,filepath,filename): #
    confige= []
    outfile=filepath+'//'+'config.txt'
    #f2= open('config.txt', encoding='utf-8', mode='w')
    f = open(Larm_path, encoding='utf-8', mode='r')
    Larm = f.readlines()
    str1=filename[:filename.index(".")]
    if filename.find('.txt') >= 0 or filename.find('.log') >= 0 or filename.find('.INS') >= 0 or filename.find('.dat') >= 0:
        with open(outfile, encoding='utf-8', mode='w+') as f2:
            for i in range(len(Larm)):
                if re.search(str1,Larm[i]):
                    O1='Original ='+filepath+'\\data\\'+filename
                    S1='Simulation =' + filepath + '\\data\\' + filename[:-4] +'_sim.txt'
                    f2.writelines(O1+"\n")
                    f2.writelines(S1+"\n")
                    f2.writelines(Larm[i+1:i+5])
                    break


def getconfig4(Larm_path,filepath,filename): #
    confige= []
    outfile=filepath+'//'+'config.txt'
    #f2= open('config.txt', encoding='utf-8', mode='w')
    f = open(Larm_path, encoding='utf-8', mode='r')
    Larm = f.readlines()
    str1=filename[:filename.index(".")]
    if filename.find('.txt') >= 0 or filename.find('.log') >= 0 or filename.find('.INS') >= 0 or filename.find('.dat') >= 0:
        with open(outfile, encoding='utf-8', mode='w+') as f2:
            for i in range(len(Larm)):
                if re.search(str1,Larm[i]):
                    O1='Original ='+filepath+'\\data\\'+filename
                    S1='Simulation =' + filepath + '\\data\\' + filename[:-4] +'_sim_error.txt'
                    S2 = 'Simulation_output =' + filepath + '\\data\\' + filename[:-4] + '_sim.txt'
                    f2.writelines(O1+"\n")
                    f2.writelines(S1+"\n")
                    f2.writelines(S2 + "\n")
                    f2.writelines(Larm[i+1:i+5])
                    break

def getconfig5(Larm_path,filepath,filename): #
    confige= []
    outfile = filepath+'//'+'config.txt'
    #f2= open('config.txt', encoding='utf-8', mode='w')
    f = open(Larm_path, encoding='utf-8', mode='r')
    Larm = f.readlines()
    if filename.find('.txt') >= 0 or filename.find('.log') >= 0 or filename.find('.INS') >= 0 or filename.find('.dat') >= 0:
        with open(outfile, encoding='utf-8', mode='w+') as f2:
            for i in range(len(Larm)):
                if re.search("ltg",Larm[i]):
                    O1 = 'Original ='+filepath+'\\data\\'+filename
                    S1 = 'Simulation =' + filepath + '\\data\\' + filename[:-4] + '_sim_error.txt'
                    S2 = 'Simulation_output =' + filepath + '\\data\\' + filename[:-4] + '_sim.txt'
                    f2.writelines(O1+"\n")
                    f2.writelines(S1+"\n")
                    f2.writelines(S2 + "\n")
                    f2.writelines(Larm[i:i+4])



def Parsone(Path):
    obj.filePath = path
    obj.ParseThreadFunc()



if __name__ == "__main__":
    filePath1 = "./test_1.INS"
    filePath2 = "./test12302sim.txt"
    obj = HexDataParse()
    obj1 = HexDataParse()
    obj2 = HexDataParse()
    obj1.filePath = filePath2
    obj2.filePath = "./test_1.INS"
    DirPath = input("请输入文件路径（路径下需包含data数据存放文件夹、配置文件、仿真可执行程序）：")
    threads = []
    if Path(DirPath).is_dir():
        Lif = input("请选择杆臂值是配置方式：\n"
                    "1:单独配置，修改congfigall\n"
                    "2:统一配置，修改Larm_config\n"
                    "3：统一配置，输入其他杆臂值路径\n"
                    )
    else:
        print("路径错误，请检查...")
        #exit()
    # print(Lif)
    # print(type(Lif))
    if Lif == "2":
        Larm_path=DirPath +'//Larm_config.txt'
        #print(Larm_path)
    elif Lif == "3":
        Larm_path=input("请输入其他杆臂值路径:")
    datapath = DirPath +'//data'
    configall_path = DirPath +'//configall.txt'
    exe_path = DirPath+'//main.exe'
    print(exe_path)

    #DirPath="E:\\02工作文件\\数据帧解包\\数据帧解包工具20220419\\源码2\\alldir"

    Datapath_list = os.listdir(datapath)
    N = len(Datapath_list)

    #DirList=os.listdir(DirPath)
    #path1=DirPath+"\\"+DirList[0]
    #仿真：1
    for i in range(N):
        if Lif == "1":
            getconfig4(configall_path, DirPath, Datapath_list[i])
        else:
            getconfig5(Larm_path, DirPath, Datapath_list[i])
        # r_v = os.system(exe_path)
        # print(r_v)
        #open(DirPath)
        main =DirPath+'//'+"main.exe"
        #main = "main.exe"
        time.sleep(0.5)
        if os.path.exists(main):
            print("开始仿真...")
            #res = os.system(exe_path)
            rc, out = subprocess.getstatusoutput(main)
            print("仿真完成...")
            print(rc)
            print('*' * 10)
            print(out)
            path_sim = Datapath_list[i][:-4] +'_sim.txt'
            Datapath_list.append(path_sim)

    for i in range(N,len(Datapath_list)):
        if Datapath_list[i].find('.txt')>=0 or Datapath_list[i].find('.log')>=0 or Datapath_list[i].find('.INS')>=0 or Datapath_list[i].find('.dat')>=0:
            path=datapath+"\\"+Datapath_list[i]
            #print(path)
            #t1=threading.Thread(target=Parsone,args=(path))
            Parsone(path)
            #t1.start()
        else:
            break












    # obj2.startParseFileHexData()
    # obj2.startSaveAllDataToMatFile()
    # obj2.printDataFrameStatsResult()
    # obj1.startParseFileHexData()
    # obj1.startSaveAllDataToMatFile()
    # obj1.printDataFrameStatsResult()
    #obj1.ParseThreadFunc()
    # obj1.set
    # obj1.createDataParseThread()
    # try:
    #     obj2.createDataParseThread()
    # except:
    #     print("Error: unable to start thread")

    # obj = HexDataParse()
    # obj = HexDataParse()
    # obj.filePath = "./pboxHE.txt"
    # print("开始数据解析...")
    # obj.startParseFileHexData()
    # print("数据解析完成...")
    # print("开始存成mat文件...")
    # obj.startSaveAllDataToMatFile()
    # print("存成mat文件完成...")



    #obj.printDataFrameStatsResult()  # 打印数据帧统计结果

    # obj.saveImuDataToMatFile()

    # obj.dataSaveCsvFile(obj.imuData, "IMU数据")
    # obj.dataSaveCsvFile(obj.gpsData, "GPS数据")
    # obj.dataSaveCsvFile(obj.vehicleData, "车辆数据")
    # obj.dataSaveCsvFile(obj.insData, "INS数据")
    # obj.dataSaveCsvFile(obj.syncData, "同步时间数据")

    # obj.readFileData()

    # obj.findSyncDataFrame()
    # obj.parseSyncData()
    # obj.dataSaveCsvFile(obj.syncData, "同步时间数据")

    # obj.findINSDataFrame()
    # obj.parseINSData()
    # obj.dataSaveCsvFile(obj.insData,"INS数据")

    # obj.findVehicleDataFrame()
    # obj.parseVehicleData()
    # obj.dataSaveCsvFile(obj.vehicleData, "车辆数据")

    # obj.findGPSDataFrame()
    # obj.parseGPSData()
    # obj.dataSaveCsvFile(obj.gpsData, "GPS数据")

    # obj.findIMUDataFrame()
    # obj.parseIMUData()
    # obj.dataSavePklFile(obj.imuData, "原始IMU数据")
    # obj.dataSaveCsvFile(obj.imuData, "原始IMU数据")
