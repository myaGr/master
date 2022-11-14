import numpy as np
import math


class DataStatistics:
    def __init__(self, parent=None):
        self.error = {}
        self.REFSpeed = {}
        self.mileage = {}
        self.GPSSpeed = {}
        self.INSSpeed = {}
        self.GpsInsSpeedDiff = {}
        self.VehSpd = {}
        self.Pos = {}
        self.bpos = np.array([[0, 0, 0], [0, 0, 0]])
        self.statisticslist = {}
        self.statistics_list_loc = {}
        self.statistics_list_pos = {}
        self.statisticsgpsflag = {}
        self.error_statistic = {}
        self.error_statistic_loc = {}
        self.error_statistic_pos = {}

    # InsGps速度转换，北东地坐标系转前右下坐标系
    # 参数： 时间同步数据，同步数据类型：（1）GPS和INS同步数据 （2）参考数据和INS同步数据
    def SpeedCalculation(self, InsGpsSyncData, type):
        if type == 1:  # GPS和INS同步数据
            sinyaw = np.sin(np.array(InsGpsSyncData["yaw"]) / 180 * math.pi)
            cosyaw = np.cos(np.array(InsGpsSyncData["yaw"]) / 180 * math.pi)
            sinheading = np.sin(np.array(InsGpsSyncData['TrackAngle']) / 180 * math.pi)
            cosheading = np.cos(np.array(InsGpsSyncData['TrackAngle']) / 180 * math.pi)

            # GPS 北、东、地速度
            self.GPSSpeed["NorthVelocity"] = np.array(InsGpsSyncData['HSpd']) * cosheading
            self.GPSSpeed["EastVelocity"] = np.array(InsGpsSyncData['HSpd']) * sinheading
            self.GPSSpeed["GroundVelocity"] = np.array(InsGpsSyncData['VSpd'])
            # INS 北、东、地速度
            self.INSSpeed["NorthVelocity"] = InsGpsSyncData['NorthVelocity']
            self.INSSpeed["EastVelocity"] = InsGpsSyncData['EastVelocity']
            self.INSSpeed["GroundVelocity"] = -InsGpsSyncData['GroundVelocity']
            # GPS-INS 北、东、地速度差
            self.GpsInsSpeedDiff["NorthVelocity"] = self.GPSSpeed["NorthVelocity"] - self.INSSpeed["NorthVelocity"]
            self.GpsInsSpeedDiff["EastVelocity"] = self.GPSSpeed["EastVelocity"] - self.INSSpeed["EastVelocity"]
            self.GpsInsSpeedDiff["GroundVelocity"] = self.GPSSpeed["GroundVelocity"] - self.INSSpeed["GroundVelocity"]
            # INS北东地速度转换成前右下坐标系下的速度
            self.INSSpeed["ForwardVelocity"] = cosyaw * self.INSSpeed["NorthVelocity"] + sinyaw * self.INSSpeed[
                "EastVelocity"]
            self.INSSpeed["RightVelocity"] = - sinyaw * self.INSSpeed["NorthVelocity"] + cosyaw * self.INSSpeed[
                "EastVelocity"]
            self.INSSpeed["DownwardVelocity"] = self.INSSpeed["GroundVelocity"]
            # 里程
            instime_diff_list = np.diff(InsGpsSyncData['time'])
            self.mileage["ForwardVelocity"] = np.cumsum(
                instime_diff_list * np.array(self.INSSpeed["ForwardVelocity"][1:]))
            self.mileage["RightVelocity"] = np.cumsum(instime_diff_list * np.array(self.INSSpeed["RightVelocity"][1:]))
            self.mileage["DownwardVelocity"] = np.cumsum(
                instime_diff_list * np.array(self.INSSpeed["DownwardVelocity"][1:]))

        if type == 2:  # 参考数据和INS同步数据
            sinyaw = np.sin(np.array(InsGpsSyncData["yaw_x"]) / 180 * math.pi)
            cosyaw = np.cos(np.array(InsGpsSyncData["yaw_x"]) / 180 * math.pi)
            # Ref 北、东、地速度
            self.REFSpeed["NorthVelocity"] = InsGpsSyncData['NorthVelocity_x']
            self.REFSpeed["EastVelocity"] = InsGpsSyncData['EastVelocity_x']
            self.REFSpeed["GroundVelocity"] = InsGpsSyncData['GroundVelocity_x']
            # INS 北、东、地速度
            self.INSSpeed["NorthVelocity"] = InsGpsSyncData['NorthVelocity_y']
            self.INSSpeed["EastVelocity"] = InsGpsSyncData['EastVelocity_y']
            self.INSSpeed["GroundVelocity"] = InsGpsSyncData['GroundVelocity_y']
            # Ref北东地速度转换成前右下坐标系下的速度
            self.REFSpeed["ForwardVelocity"] = cosyaw * self.REFSpeed["NorthVelocity"] + sinyaw * self.REFSpeed[
                "EastVelocity"]
            self.REFSpeed["RightVelocity"] = - sinyaw * self.REFSpeed["NorthVelocity"] + cosyaw * self.REFSpeed[
                "EastVelocity"]
            self.REFSpeed["DownwardVelocity"] = self.REFSpeed["GroundVelocity"]
            # INS北东地速度转换成前右下坐标系下的速度
            self.INSSpeed["ForwardVelocity"] = cosyaw * self.INSSpeed["NorthVelocity"] + sinyaw * self.INSSpeed[
                "EastVelocity"]
            self.INSSpeed["RightVelocity"] = - sinyaw * self.INSSpeed["NorthVelocity"] + cosyaw * self.INSSpeed[
                "EastVelocity"]
            self.INSSpeed["DownwardVelocity"] = self.INSSpeed["GroundVelocity"]

    # 四轮轮速计算
    def ForwardSpeedCalculation(self, PData, VehicleData):
        KwsMean = np.mean(PData["Kws"], axis=0)
        self.VehSpd["DriR_L"] = VehicleData["WheelSpeedFrontLeft"] * KwsMean[0]
        self.VehSpd["DriR_R"] = VehicleData["WheelSpeedFrontRight"] * KwsMean[1]
        self.VehSpd["NonDriR_L"] = VehicleData["WheelSpeedBackLeft"] * KwsMean[0]
        self.VehSpd["NonDriR_R"] = VehicleData["WheelSpeedBackRight"] * KwsMean[1]

    ## 位置坐标转换：
    # 参数： 时间同步数据，杆臂值，同步数据类型：（1）GPS和INS同步数据 （2）参考数据和INS同步数据
    # 步骤1. 地理坐标系转平面坐标坐标系
    # 步骤2. 北东地（平面）坐标系转 前右下（平面）坐标系
    def PosErrorCalculation(self, InsGpsSyncData, bpos, pos0, type):
        global yaw
        self.Pos["pos0"] = pos0
        if type == 1:  # GPS和INS同步数据
            self.Pos["lat"] = np.array([InsGpsSyncData['lat'], InsGpsSyncData['Lat']])
            self.Pos["lon"] = np.array([InsGpsSyncData['lon'], InsGpsSyncData['Lon']])
            self.Pos["h"] = np.array([InsGpsSyncData['height'], InsGpsSyncData['hMSL']])
            yaw = np.array(InsGpsSyncData["yaw"])
        elif type == 2:  # 参考数据和INS同步数据
            self.Pos["lat"] = np.array([InsGpsSyncData['lat_x'], InsGpsSyncData['lat_y']])
            self.Pos["lon"] = np.array([InsGpsSyncData['lon_x'], InsGpsSyncData['lon_y']])
            self.Pos["h"] = np.array([InsGpsSyncData['height_x'], InsGpsSyncData['height_y']])
            yaw = np.array(InsGpsSyncData["yaw_x"])

        # 1. 地理坐标系转平面坐标坐标系
        # (1) 基础公式计算
        R = 6378137.0  # 地球长半轴
        f = 1 / 298.257223563  # 地球椭球扁率
        e2 = f * (2 - f)  # 椭球偏心率的平方
        self.Pos["L"] = self.Pos["lat"] * math.pi / 180
        self.Pos["sinL"] = np.sin(self.Pos["L"])
        self.Pos["cosL"] = np.cos(self.Pos["L"])
        self.Pos["temp"] = np.sqrt(1 - e2 * self.Pos["sinL"] * self.Pos["sinL"])
        self.Pos["temp3"] = self.Pos["temp"] * self.Pos["temp"] * self.Pos["temp"]
        self.Pos["Rn"] = R * (1 - e2) / self.Pos["temp3"]  # 子午面曲率半径
        self.Pos["Re"] = R / self.Pos["temp"]  # 横向曲率半径
        self.Pos["RnhINS"] = self.Pos["Rn"] + self.Pos["h"]
        self.Pos["RehINS"] = (self.Pos["Re"] + self.Pos["h"]) * self.Pos["cosL"]
        # (2) 经纬度单位转为米
        self.Pos["Lat_Y"] = (self.Pos["lat"] - self.Pos["pos0"][0]) / 180 * math.pi * self.Pos["RnhINS"]
        self.Pos["Lon_X"] = (self.Pos["lon"] - self.Pos["pos0"][1]) / 180 * math.pi * self.Pos["RehINS"]
        self.Pos["Height_Z"] = self.Pos["h"]
        # 2. 载体坐标系（前右下）转地理坐标系（北东地）
        # (1) 北东地坐标系下杆臂值校正:
        sin3 = np.sin(yaw / 180 * math.pi)
        cos3 = np.cos(yaw / 180 * math.pi)
        pos_err = [[cos3 * bpos[0, 0] - sin3 * bpos[0, 1], sin3 * bpos[0, 0] + cos3 * bpos[0, 1], - bpos[0, 2]],
                   [cos3 * bpos[1, 0] - sin3 * bpos[1, 1], sin3 * bpos[1, 0] + cos3 * bpos[1, 1], - bpos[1, 2]]]

        # (2) 北东地坐标系下坐标校正，补偿固定偏差:
        self.Pos["Lat_Y"][0] = self.Pos["Lat_Y"][0] - pos_err[0][0]
        self.Pos["Lon_X"][0] = self.Pos["Lon_X"][0] - pos_err[0][1]
        self.Pos["Height_Z"][0] = self.Pos["Height_Z"][0] - pos_err[0][2]
        self.Pos["Lat_Y"][1] = self.Pos["Lat_Y"][1] - pos_err[1][0]
        self.Pos["Lon_X"][1] = self.Pos["Lon_X"][1] - pos_err[1][1]
        self.Pos["Height_Z"][1] = self.Pos["Height_Z"][1] - pos_err[1][2]
        self.Pos["PosXError"] = cos3 * (self.Pos["Lat_Y"][0] - self.Pos["Lat_Y"][1]) + sin3 * (
                    self.Pos["Lon_X"][0] - self.Pos["Lon_X"][1])  # 横向偏差
        self.Pos["PosYError"] = -sin3 * (self.Pos["Lat_Y"][0] - self.Pos["Lat_Y"][1]) + cos3 * (
                    self.Pos["Lon_X"][0] - self.Pos["Lon_X"][1])  # 纵向偏差
        self.Pos["PosXYError"] = np.sqrt(
            self.Pos["PosXError"] * self.Pos["PosXError"] + self.Pos["PosYError"] * self.Pos["PosYError"])  # 水平偏差

    # GPS解状态转换
    def Gpsflagstransfer(self, InsGpsSyncData):
        flagsPos = InsGpsSyncData['flagsPos']
        self.Pos["GpsNone"] = [self.Pos["Lat_Y"][1][np.where(flagsPos == 0)],
                               self.Pos["Lon_X"][1][np.where(flagsPos == 0)]]
        self.Pos["GpsSigle"] = [self.Pos["Lat_Y"][1][np.where(flagsPos == 16)],
                                self.Pos["Lon_X"][1][np.where(flagsPos == 16)]]
        self.Pos["GpsDiff"] = [self.Pos["Lat_Y"][1][np.where(flagsPos == 17)],
                               self.Pos["Lon_X"][1][np.where(flagsPos == 17)]]
        self.Pos["GpsFloat"] = [self.Pos["Lat_Y"][1][np.where(flagsPos == 34)],
                                self.Pos["Lon_X"][1][np.where(flagsPos == 34)]]
        self.Pos["GpsFix"] = [self.Pos["Lat_Y"][1][np.where((flagsPos == 48) | (flagsPos == 49) | (flagsPos == 50))],
                              self.Pos["Lon_X"][1][np.where((flagsPos == 48) | (flagsPos == 49) | (flagsPos == 50))]]
        self.Pos["GpsFlags"] = np.array(flagsPos)
        self.Pos["GpsFlags"][np.where(flagsPos == 0)] = 0
        self.Pos["GpsFlags"][np.where(flagsPos == 16)] = 1
        self.Pos["GpsFlags"][np.where(flagsPos == 17)] = 2
        self.Pos["GpsFlags"][np.where(flagsPos == 34)] = 5
        self.Pos["GpsFlags"][np.where((flagsPos == 48) | (flagsPos == 49) | (flagsPos == 50))] = 4

    # 计算数组的最大值和1\2\3倍sigma
    @staticmethod
    def sigma_err_cal(datalist):
        sigma_error = [0, 0, 0, 0]
        sigma = [0.6827, 0.9544, 0.9974];
        sorteddata = sorted(abs(datalist))
        sigma_error[0] = sorteddata[math.ceil(len(sorteddata) * sigma[0]) - 1]  # 1倍sigame
        sigma_error[1] = sorteddata[math.ceil(len(sorteddata) * sigma[1]) - 1]  # 2倍sigame
        sigma_error[2] = sorteddata[math.ceil(len(sorteddata) * sigma[2]) - 1]  # 3倍sigame
        sigma_error[3] = sorteddata[-1]  # 最大值
        return sigma_error

    # 统计INS精度
    def StatisticSyncData(self, SyncData, name):
        items = ['名称', '统计帧数', 'RMS外符合精度', '<0.02偏差占比', '<0.05偏差占比', '<0.1偏差占比', '<0.2偏差占比',
                 '<0.5偏差占比', '<1偏差占比', '<1.5偏差占比', '<2偏差占比',
                 '位置偏差1σ', '位置偏差2σ', '位置偏差3σ', '位置偏差最大', '横滚偏差1σ', '横滚偏差2σ', '横滚偏差3σ',
                 '横滚偏差最大',
                 '俯仰偏差1σ', '俯仰偏差2σ', '俯仰偏差3σ', '俯仰偏差最大', '航向偏差1σ', '航向偏差2σ', '航向偏差3σ',
                 '航向偏差最大',
                 '速度偏差1σ', '速度偏差2σ', '速度偏差3σ', '速度偏差最大', 'X速度偏差1σ', 'X速度偏差2σ', 'X速度偏差3σ',
                 'X速度偏差最大',
                 'Y速度偏差1σ', 'Y速度偏差2σ', 'Y速度偏差3σ', 'Y速度偏差最大', 'Z速度偏差1σ', 'Z速度偏差2σ',
                 'Z速度偏差3σ', 'Z速度偏差最大',
                 '高程偏差1σ', '高程偏差2σ', '高程偏差3σ', '高程偏差最大']
        percentage = [0.02, 0.05, 0.1, 0.2, 0.5, 1, 1.5, 2]
        error = ["pos", "roll", "pitch", "yaw", "vel", "vel_x", "vel_y", "vel_z", "height"]
        self.error["pos"] = self.Pos["PosXYError"]
        self.error["height"] = self.Pos["Height_Z"][0] - self.Pos["Height_Z"][1]

        self.statisticslist[items[0]] = [name]
        self.statisticslist[items[1]] = [len(SyncData)]
        self.statisticslist[items[2]] = [round(np.sqrt(sum(self.error["pos"] * self.error["pos"]) / len(SyncData)), 4)]
        for i in range(len(percentage)):
            self.statisticslist[items[i + 3]] = [
                round(len(self.error["pos"][np.where(self.error["pos"] < percentage[i])]) / len(SyncData), 4)]

        for i in range(len(error)):
            sigma_err = self.sigma_err_cal(self.error[error[i]])
            self.statisticslist[items[10 + i * 4 + 1]] = [round(sigma_err[0], 4)]
            self.statisticslist[items[10 + i * 4 + 2]] = [round(sigma_err[1], 4)]
            self.statisticslist[items[10 + i * 4 + 3]] = [round(sigma_err[2], 4)]
            self.statisticslist[items[10 + i * 4 + 4]] = [round(sigma_err[3], 4)]
            self.error_statistic[error[i]] = sigma_err
        return self.statisticslist

    def statistic_sync_positions(self, SyncData, name, time_range='[0,0]', scene='全程'):
        items = ['名称', '时间范围', '场景', '统计帧数',
                 '横滚偏差1σ', '横滚偏差2σ', '横滚偏差3σ', '横滚偏差最大',
                 '俯仰偏差1σ', '俯仰偏差2σ', '俯仰偏差3σ', '俯仰偏差最大',
                 '航向偏差1σ', '航向偏差2σ', '航向偏差3σ', '航向偏差最大',
                 '速度偏差1σ', '速度偏差2σ', '速度偏差3σ', '速度偏差最大',
                 'X速度偏差1σ', 'X速度偏差2σ', 'X速度偏差3σ', 'X速度偏差最大',
                 'Y速度偏差1σ', 'Y速度偏差2σ', 'Y速度偏差3σ', 'Y速度偏差最大',
                 'Z速度偏差1σ', 'Z速度偏差2σ', 'Z速度偏差3σ', 'Z速度偏差最大']
        error = ["roll", "pitch", "yaw", "vel", "vel_x", "vel_y", "vel_z"]

        self.statistics_list_pos[items[0]] = [name]
        self.statistics_list_pos[items[1]] = [time_range]
        self.statistics_list_pos[items[2]] = [scene]
        self.statistics_list_pos[items[3]] = [len(SyncData)]

        for i in range(len(error)):
            sigma_err = self.sigma_err_cal(self.error[error[i]])
            self.statistics_list_pos[items[3 + i * 4 + 1]] = [round(sigma_err[0], 4)]
            self.statistics_list_pos[items[3 + i * 4 + 2]] = [round(sigma_err[1], 4)]
            self.statistics_list_pos[items[3 + i * 4 + 3]] = [round(sigma_err[2], 4)]
            self.statistics_list_pos[items[3 + i * 4 + 4]] = [round(sigma_err[3], 4)]
            self.error_statistic[error[i]] = sigma_err
        return self.statistics_list_pos

    def statistic_sync_locations(self, SyncData, name, time_range='[0,0]', scene='全程'):
        items = ['名称', '时间范围', '场景', '统计帧数', 'RMS外符合精度',
                 '<0.02偏差占比', '<0.05偏差占比', '<0.1偏差占比', '<0.2偏差占比',
                 '<0.5偏差占比', '<1偏差占比', '<1.5偏差占比', '<2偏差占比',
                 '位置偏差1σ', '位置偏差2σ', '位置偏差3σ', '位置偏差最大',
                 '高程偏差1σ', '高程偏差2σ', '高程偏差3σ', '高程偏差最大',
                 '横向偏差1σ', '横向偏差2σ', '横向偏差3σ', '横向偏差最大',
                 '纵向偏差1σ', '纵向偏差2σ', '纵向偏差3σ', '纵向偏差最大']
        percentage = [0.02, 0.05, 0.1, 0.2, 0.5, 1, 1.5, 2]
        error = ["pos", "height", "PosXError", "PosYError"]
        self.error["pos"] = self.Pos["PosXYError"]
        self.error["height"] = self.Pos["Height_Z"][0] - self.Pos["Height_Z"][1]
        self.error["PosXError"] = self.Pos["PosXError"]
        self.error["PosYError"] = self.Pos["PosYError"]

        self.statistics_list_loc[items[0]] = [name]
        self.statistics_list_loc[items[1]] = [time_range]
        self.statistics_list_loc[items[2]] = [scene]
        self.statistics_list_loc[items[3]] = [len(SyncData)]
        self.statistics_list_loc[items[4]] = [round(np.sqrt(sum(self.error["pos"] * self.error["pos"]) / len(SyncData)), 4)]
        for i in range(len(percentage)):
            self.statistics_list_loc[items[i + 5]] = [
                round(len(self.error["pos"][np.where(self.error["pos"] < percentage[i])]) / len(SyncData), 4)]

        for i in range(len(error)):
            sigma_err = self.sigma_err_cal(self.error[error[i]])
            self.statistics_list_loc[items[12 + i * 4 + 1]] = [round(sigma_err[0], 4)]
            self.statistics_list_loc[items[12 + i * 4 + 2]] = [round(sigma_err[1], 4)]
            self.statistics_list_loc[items[12 + i * 4 + 3]] = [round(sigma_err[2], 4)]
            self.statistics_list_loc[items[12 + i * 4 + 4]] = [round(sigma_err[3], 4)]
            self.error_statistic_loc[error[i]] = sigma_err
        return self.statistics_list_loc

    # 统计GPS各种解状态的占比
    def StatisticGpsFlag(self, SyncRefGpsData, name):
        items = ["名称", "统计帧数", "固定", "浮点", "差分", "单点", "None"]
        gps_len = len(SyncRefGpsData)
        none_len = gps_len - len(self.Pos["GpsFix"][0]) - len(self.Pos["GpsFloat"][0]) - len(
            self.Pos["GpsDiff"][0]) - len(self.Pos["GpsSigle"][0])
        self.statisticsgpsflag[items[0]] = [name]
        self.statisticsgpsflag[items[1]] = [gps_len]
        self.statisticsgpsflag[items[2]] = [round(len(self.Pos["GpsFix"][0]) / gps_len, 4)]
        self.statisticsgpsflag[items[3]] = [round(len(self.Pos["GpsFloat"][0]) / gps_len, 4)]
        self.statisticsgpsflag[items[4]] = [round(len(self.Pos["GpsDiff"][0]) / gps_len, 4)]
        self.statisticsgpsflag[items[5]] = [round(len(self.Pos["GpsSigle"][0]) / gps_len, 4)]
        self.statisticsgpsflag[items[6]] = [round(none_len / gps_len, 4)]
        return self.statisticsgpsflag


if __name__ == "__main__":
    obj = DataStatistics()
    data = np.array([1, -2, 3, -4, 5, 6, -7, 8, 9, -10])
    obj.sigma_err_cal(data)
