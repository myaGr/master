import matplotlib.pyplot as plt
import numpy as np
from PlotData import PlotGpsInsData
from matplotlib.pyplot import MultipleLocator
from DataStatistics import DataStatistics
import pandas as pd
import math

plt.rc("font", family='MicroSoft YaHei', weight='bold')


class PlotGpsInsRawSyncData:
    """
    plot INS数据与自身GPS对比画图 & INS数据与参考对比画图
    @author: wenzixuan liqianwen
    @version: for analyse db files
    @date: 2022-09-21
    """

    def __init__(self):
        self.bpos_gpsins = np.array([[0, 0, 0], [0, 0, 0]])
        self.bpos_refins = {}
        self.bpos_refgps = {}
        self.InsDataDF = None
        self.PDataDict = None
        self.SyncDataDF = None
        self.VehicleDataDF = None
        self.GpsDataDF = None
        self.ImuDataDF = None
        self.SyncInsGpsData = None
        self.SyncRefInsData = {}
        self.SyncRefGpsData = {}
        self.SyncRefGpsINSData = {}
        self.RefInsData = {}
        self.RefGpsData = {}
        self.gps_flag = None
        self.second_of_week = True
        self.time0_set = 0


    def PlotGpsInsRawSyncData(self):
        """
        INS数据与自身GPS对比画图
        :param second_of_week 是否采用周内秒画图
        :return: 10 picture
        """
        bpos = self.bpos_gpsins
        InsData = self.InsDataDF
        PData = self.PDataDict
        SyncData = self.SyncDataDF
        VehicleData = self.VehicleDataDF
        GpsData = self.GpsDataDF
        ImuData = self.ImuDataDF
        InsGpsSyncData = self.SyncInsGpsData
        DataStatiObj = DataStatistics()  # 统计计算
        DataStatiObj.SpeedCalculation(InsGpsSyncData, 1)  # 计算北东地和前右下速度
        DataStatiObj.ForwardSpeedCalculation(PData, VehicleData)  # 四轮轮速计算
        DataStatiObj.PosErrorCalculation(InsGpsSyncData, bpos, 1)  # 位置坐标转换
        DataStatiObj.Gpsflagstransfer(InsGpsSyncData)  # GPS解状态转换

        InsData_pre, ImuData_pre, GpsData_pre = InsData, ImuData, GpsData

        # if not self.second_of_week:
        #     InsData, ImuData, GpsData, InsGpsSyncData, VehicleData, PData = flit_time_file(InsData, ImuData, GpsData, InsGpsSyncData, VehicleData, PData)

        # todo
        self.checkTimeType1()  # 显示时间类型

        # figure1: DataDiff
        Plotobj1 = PlotGpsInsData()
        Plotobj1.fig.suptitle('DataDiff')
        Plotobj1.ax1 = plt.subplot(3, 2, 1)
        Plotobj1.ax1.set_title('0A')
        Plotobj1.PlotData(Plotobj1.ax1, ImuData['time'][:-1] - self.time0_set, np.diff(ImuData_pre['time']), '0A_diff')
        Plotobj1.ShowPlotFormat('', 'unit:s')
        Plotobj1.ax2 = plt.subplot(3, 2, 3)
        Plotobj1.ax2.set_title('0B')
        Plotobj1.PlotData(Plotobj1.ax2, InsData['time'][:-1] - self.time0_set, np.diff(InsData_pre['time']), '0B_diff')
        Plotobj1.ShowPlotFormat('', 'unit:s')
        Plotobj1.ax3 = plt.subplot(3, 2, 5)
        Plotobj1.ax3.set_title('0C')
        Plotobj1.PlotData(Plotobj1.ax3, SyncData['ts'][:-1] - self.time0_set, np.diff(SyncData['imuTime']), '0C_imuTime_diff')
        Plotobj1.PlotData(Plotobj1.ax3, SyncData['ts'][:-1] - self.time0_set, np.diff(SyncData['gpsTime']), '0C_gpsTime_diff')
        Plotobj1.PlotData(Plotobj1.ax3, SyncData['ts'][:-1] - self.time0_set, np.diff(SyncData['ts']), '0C_ts_diff')
        Plotobj1.ShowPlotFormat('', 'unit:s')
        Plotobj1.ax4 = plt.subplot(3, 2, 2)
        Plotobj1.ax4.set_title('10_bestpos')
        Plotobj1.PlotData(Plotobj1.ax4, GpsData['ts'][:-1] - self.time0_set, np.diff(GpsData_pre['itow_pos']), '10_bestpos_diff')
        Plotobj1.ShowPlotFormat('', 'unit:s')
        Plotobj1.ax5 = plt.subplot(3, 2, 4)
        Plotobj1.ax5.set_title('10_bestvel')
        Plotobj1.PlotData(Plotobj1.ax5, GpsData['ts'][:-1] - self.time0_set, np.diff(GpsData_pre['itow_vel']), '10_bestvel_diff')
        Plotobj1.ShowPlotFormat('', 'unit:s')
        Plotobj1.ax6 = plt.subplot(3, 2, 6)
        Plotobj1.ax6.set_title('10_heading')
        Plotobj1.PlotData(Plotobj1.ax6, GpsData['ts'][:-1] - self.time0_set, np.diff(GpsData_pre['itow_heading']), '10_heading_diff')
        Plotobj1.ShowPlotFormat('', 'unit:s')

        # figure2: Att
        Plotobj2 = PlotGpsInsData()
        Plotobj2.fig.suptitle('Angle')
        Plotobj2.ax1 = plt.subplot(2, 1, 1)
        Plotobj2.ax1.set_title('yaw')
        Plotobj2.PlotData(Plotobj2.ax1, GpsData['itow_heading'] - self.time0_set, GpsData['Heading'], 'GPSYaw')
        if 'headingStd' in GpsData.keys():
            Plotobj2.PlotData(Plotobj2.ax1, GpsData['itow_pos'] - self.time0_set, GpsData['headingStd'], 'HeadingAccuracy')
        Plotobj2.PlotData(Plotobj2.ax1, InsData['time'] - self.time0_set, InsData['yaw'], 'INSYaw')
        Plotobj2.PlotData(Plotobj2.ax1, GpsData['itow_vel'] - self.time0_set, GpsData['TrackAngle'], 'GPSTrackAngle')
        Plotobj2.ShowPlotFormat('', 'unit:deg')
        Plotobj2.ax2 = plt.subplot(2, 1, 2)
        Plotobj2.ax2.set_title('att')
        GPSYaw_INSYaw = angele_standardization(np.array(InsGpsSyncData['Heading'] - InsGpsSyncData['yaw']))
        GPSTrackAngle_INSYaw = angele_standardization(np.array(InsGpsSyncData['TrackAngle'] - InsGpsSyncData['yaw']))
        Plotobj2.PlotData(Plotobj2.ax2, InsData['time'] - self.time0_set, InsData['roll'], 'INSRoll')
        Plotobj2.PlotData(Plotobj2.ax2, InsData['time'] - self.time0_set, InsData['pitch'], 'INSPitch')
        Plotobj2.PlotData(Plotobj2.ax2, GpsData['itow_heading'] - self.time0_set, GpsData['Pitch'], 'GPSPitch')
        Plotobj2.PlotData(Plotobj2.ax2, InsGpsSyncData['time'] - self.time0_set, GPSYaw_INSYaw, 'GPSYaw-INSYaw')
        Plotobj2.PlotData(Plotobj2.ax2, InsGpsSyncData['time'] - self.time0_set, GPSTrackAngle_INSYaw, 'GPSTrackAngle-INSYaw')
        Plotobj2.ShowPlotFormat('', 'unit:deg')

        # figure3: Velocity
        Plotobj3 = PlotGpsInsData()
        Plotobj3.fig.suptitle('Speed')
        Plotobj3.ax1 = plt.subplot(3, 2, 1)
        Plotobj3.ax1.set_title('North_Speed')
        Plotobj3.PlotData(Plotobj3.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.GPSSpeed["NorthVelocity"], 'GPS')
        Plotobj3.PlotData(Plotobj3.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.INSSpeed["NorthVelocity"], 'INS')
        Plotobj3.PlotData(Plotobj3.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.GpsInsSpeedDiff["NorthVelocity"],
                          'GPS-INS')
        Plotobj3.ShowPlotFormat('', 'unit:m/s')
        Plotobj3.ax2 = plt.subplot(3, 2, 3)
        Plotobj3.ax2.set_title('East_Speed')
        Plotobj3.PlotData(Plotobj3.ax2, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.GPSSpeed["EastVelocity"], 'GPS')
        Plotobj3.PlotData(Plotobj3.ax2, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.INSSpeed["EastVelocity"], 'INS')
        Plotobj3.PlotData(Plotobj3.ax2, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.GpsInsSpeedDiff["EastVelocity"], 'GPS-INS')
        Plotobj3.ShowPlotFormat('', 'unit:m/s')
        Plotobj3.ax3 = plt.subplot(3, 2, 5)
        Plotobj3.ax3.set_title('Ground_Speed')
        Plotobj3.PlotData(Plotobj3.ax3, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.GPSSpeed["GroundVelocity"], 'GPS')
        Plotobj3.PlotData(Plotobj3.ax3, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.INSSpeed["GroundVelocity"], 'INS')
        Plotobj3.PlotData(Plotobj3.ax3, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.GpsInsSpeedDiff["GroundVelocity"],
                          'GPS-INS')
        Plotobj3.ShowPlotFormat('', 'unit:m/s')
        Plotobj3.ax4 = plt.subplot(3, 2, 2)
        Plotobj3.ax4.set_title('Forward_Speed_VBX')
        Plotobj3.PlotData(Plotobj3.ax4, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.INSSpeed["ForwardVelocity"], 'INS')
        Plotobj3.PlotData(Plotobj3.ax4, InsGpsSyncData['time'] - self.time0_set, InsGpsSyncData['HSpd'], 'GPS')
        Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["DriR_L"], 'DriR_L')
        Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["DriR_R"], 'DriR_R')
        Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["NonDriR_L"], 'NonDriR_L')
        Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["NonDriR_R"], 'NonDriR_R')
        Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, VehicleData["gear"], 'Shifter')
        Plotobj3.ShowPlotFormat('', 'unit:m/s')
        Plotobj3.ax5 = plt.subplot(3, 2, 4)
        Plotobj3.ax5.set_title('Right_Speed_VBY')
        Plotobj3.PlotData(Plotobj3.ax5, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.INSSpeed["RightVelocity"], 'INS')
        Plotobj3.ShowPlotFormat('', 'unit:m/s')
        Plotobj3.ax6 = plt.subplot(3, 2, 6)
        Plotobj3.ax6.set_title('Downward_Speed_VBZ')
        Plotobj3.PlotData(Plotobj3.ax6, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.INSSpeed["DownwardVelocity"], 'INS')
        Plotobj3.ShowPlotFormat('', 'unit:m/s')

        # figure4: Pos
        Plotobj4 = PlotGpsInsData()
        Plotobj4.fig.suptitle('Pos')
        Plotobj4.ax1 = plt.subplot(3, 1, 1)
        Plotobj4.ax1.set_title('Latitude')
        Plotobj4.PlotData(Plotobj4.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lat_Y"][1], 'GPS')
        Plotobj4.PlotData(Plotobj4.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lat_Y"][0], 'INS')
        Plotobj4.PlotData(Plotobj4.ax1, InsGpsSyncData['time'] - self.time0_set,
                          DataStatiObj.Pos["Lat_Y"][1] - DataStatiObj.Pos["Lat_Y"][0], 'GPS-INS')
        Plotobj4.ShowPlotFormat('', 'unit:meter')
        Plotobj4.ax2 = plt.subplot(3, 1, 2)
        Plotobj4.ax2.set_title('Longitude')
        Plotobj4.PlotData(Plotobj4.ax2, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lon_X"][1], 'GPS')
        Plotobj4.PlotData(Plotobj4.ax2, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lon_X"][0], 'INS')
        Plotobj4.PlotData(Plotobj4.ax2, InsGpsSyncData['time'] - self.time0_set,
                          DataStatiObj.Pos["Lon_X"][1] - DataStatiObj.Pos["Lon_X"][0], 'GPS-INS')
        Plotobj4.ShowPlotFormat('', 'unit:meter')
        Plotobj4.ax3 = plt.subplot(3, 1, 3)
        Plotobj4.ax3.set_title('Altitude')
        Plotobj4.PlotData(Plotobj4.ax3, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Height_Z"][1], 'GPS')
        Plotobj4.PlotData(Plotobj4.ax3, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Height_Z"][0], 'INS')
        Plotobj4.PlotData(Plotobj4.ax3, InsGpsSyncData['time'] - self.time0_set,
                          DataStatiObj.Pos["Height_Z"][1] - DataStatiObj.Pos["Height_Z"][0], 'GPS-INS')
        Plotobj4.ShowPlotFormat('', 'unit:meter')

        # figure5: Pos Error
        Plotobj5 = PlotGpsInsData()
        Plotobj5.fig.suptitle('Pos Error')
        Plotobj5.ax1 = plt.subplot(111)
        Plotobj5.ax1.set_title('GPS-INS (under C coordinate system)')
        Plotobj5.PlotData(Plotobj5.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["PosXError"], '横向偏差gcx')
        Plotobj5.PlotData(Plotobj5.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["PosYError"], '纵向偏差gcy')
        Plotobj5.PlotData(Plotobj5.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["PosXYError"], '水平偏差')
        Plotobj5.ShowPlotFormat('', 'unit:meter')
        Plotobj5.ax1.set_ylim(-5, 5)
        Plotobj5.ax2 = Plotobj5.ax1.twinx()  # 双坐标轴，右轴为GPS解状态
        Plotobj5.ax2.plot(InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["GpsFlags"],
                          label='1: 单点\n2: 差分\n4: 固定\n5: 浮点', color='r', linestyle="", marker='*')
        Plotobj5.ax2.set_ylim(0.5, 5.5)
        Plotobj5.ax2.yaxis.set_major_locator(MultipleLocator(1))
        plt.legend(loc='lower right')

        # figure6: Mileage
        Plotobj6 = PlotGpsInsData()
        Plotobj6.fig.suptitle('Mileage ')
        Plotobj6.ax1 = plt.subplot(111)
        Plotobj6.ax1.set_title('里程')
        Plotobj6.PlotData(Plotobj6.ax1, InsGpsSyncData['time'][1:] - self.time0_set, DataStatiObj.mileage['ForwardVelocity'], '里程(前)')
        Plotobj6.PlotData(Plotobj6.ax1, InsGpsSyncData['time'][1:] - self.time0_set, DataStatiObj.mileage['RightVelocity'], '里程(右)')
        Plotobj6.PlotData(Plotobj6.ax1, InsGpsSyncData['time'][1:] - self.time0_set, DataStatiObj.mileage["DownwardVelocity"],
                          '里程(下)')
        Plotobj6.ShowPlotFormat('', 'unit:meter')

        # figure7: Car Path
        Plotobj7 = PlotGpsInsData()
        Plotobj7.fig.suptitle('Car Path')
        Plotobj7.ax1 = plt.subplot(111)
        Plotobj7.ax1.set_title('行车轨迹')
        labels = list(InsGpsSyncData['time'] - self.time0_set)
        Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["Lon_X"][1], DataStatiObj.Pos["Lat_Y"][1], None, None,
                            'silver', '-', '')
        Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["Lon_X"][0], DataStatiObj.Pos["Lat_Y"][0], None, None, 'b',
                            '-', '')
        Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["Lon_X"][0], DataStatiObj.Pos["Lat_Y"][0], 'INS', labels,
                            'b', '', '.')
        Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsSigle"][1], DataStatiObj.Pos["GpsSigle"][0], 'Gps_Sigle',
                            labels, 'lightcoral', '', '.')
        Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsDiff"][1], DataStatiObj.Pos["GpsDiff"][0], 'Gps_Diff',
                            labels, 'deepskyblue', '', '.')
        Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsFloat"][1], DataStatiObj.Pos["GpsFloat"][0], 'Gps_Float',
                            labels, 'orange', '', '.')
        Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsFix"][1], DataStatiObj.Pos["GpsFix"][0], 'Gps_Fix',
                            labels, 'limegreen', '', '.')
        Plotobj7.ShowPlotFormat('经度 unit:meter', '纬度 unit:meter')

        # figure8: Error零偏
        Plotobj8 = PlotGpsInsData()
        Plotobj8.fig.suptitle('Error零偏')
        Plotobj8.ax1 = plt.subplot(4, 2, 1)
        Plotobj8.ax1.set_title('ba')
        Plotobj8.PlotData(Plotobj8.ax1, PData['BaT'] - self.time0_set, np.array(PData['Ba'])[:, 0], 'ax')
        Plotobj8.PlotData(Plotobj8.ax1, PData['BaT'] - self.time0_set, np.array(PData['Ba'])[:, 1], 'ay')
        Plotobj8.PlotData(Plotobj8.ax1, PData['BaT'] - self.time0_set, np.array(PData['Ba'])[:, 2], 'az')
        Plotobj8.ShowPlotFormat('', 'unit:g')
        Plotobj8.ax2 = plt.subplot(4, 2, 2)
        Plotobj8.ax2.set_title('bg')
        Plotobj8.PlotData(Plotobj8.ax2, PData['BgT'] - self.time0_set, np.array(PData['Bg'])[:, 0], 'gx')
        Plotobj8.PlotData(Plotobj8.ax2, PData['BgT'] - self.time0_set, np.array(PData['Bg'])[:, 1], 'gy')
        Plotobj8.PlotData(Plotobj8.ax2, PData['BgT'] - self.time0_set, np.array(PData['Bg'])[:, 2], 'gz')
        Plotobj8.ShowPlotFormat('', 'unit:deg/s')
        Plotobj8.ax3 = plt.subplot(4, 2, 3)
        Plotobj8.ax3.set_title('Atttb')
        Plotobj8.PlotData(Plotobj8.ax3, PData['AlignT'] - self.time0_set, np.array(PData['Align'])[:, 0], 'Roll')
        Plotobj8.PlotData(Plotobj8.ax3, PData['AlignT'] - self.time0_set, np.array(PData['Align'])[:, 1], 'Pitch')
        Plotobj8.PlotData(Plotobj8.ax3, PData['AlignT'] - self.time0_set, np.array(PData['Align'])[:, 2], 'Yaw')
        Plotobj8.ShowPlotFormat('', 'unit:deg')
        Plotobj8.ax4 = plt.subplot(4, 2, 4)
        Plotobj8.ax4.set_title('Lttg')
        Plotobj8.PlotData(Plotobj8.ax4, PData['LbbgT'] - self.time0_set, np.array(PData['Lbbg'])[:, 0], 'x')
        Plotobj8.PlotData(Plotobj8.ax4, PData['LbbgT'] - self.time0_set, np.array(PData['Lbbg'])[:, 1], 'y')
        Plotobj8.PlotData(Plotobj8.ax4, PData['LbbgT'] - self.time0_set, np.array(PData['Lbbg'])[:, 2], 'z')
        Plotobj8.ShowPlotFormat('', 'unit:m')
        Plotobj8.ax5 = plt.subplot(4, 2, 5)
        Plotobj8.ax5.set_title('Lttc')
        Plotobj8.PlotData(Plotobj8.ax5, PData['LbbcT'] - self.time0_set, np.array(PData['Lbbc'])[:, 0], 'x')
        Plotobj8.PlotData(Plotobj8.ax5, PData['LbbcT'] - self.time0_set, np.array(PData['Lbbc'])[:, 1], 'y')
        Plotobj8.PlotData(Plotobj8.ax5, PData['LbbcT'] - self.time0_set, np.array(PData['Lbbc'])[:, 2], 'z')
        Plotobj8.ShowPlotFormat('', 'unit:m')
        Plotobj8.ax6 = plt.subplot(4, 2, 6)
        Plotobj8.ax6.set_title('Kws')
        Plotobj8.PlotData(Plotobj8.ax6, PData['KwsT'] - self.time0_set, np.array(PData['Kws'])[:, 0], 'left')
        Plotobj8.PlotData(Plotobj8.ax6, PData['KwsT'] - self.time0_set, np.array(PData['Kws'])[:, 1], 'right')
        Plotobj8.ShowPlotFormat('', 'unit:m/puls')
        Plotobj8.ax7 = plt.subplot(4, 2, 7)
        Plotobj8.ax7.set_title('Atttg')
        Plotobj8.PlotData(Plotobj8.ax7, PData['AtttgT'] - self.time0_set, np.array(PData['Atttg'])[:, 0], 'Roll')
        Plotobj8.PlotData(Plotobj8.ax7, PData['AtttgT'] - self.time0_set, np.array(PData['Atttg'])[:, 1], 'Pitch')
        Plotobj8.PlotData(Plotobj8.ax7, PData['AtttgT'] - self.time0_set, np.array(PData['Atttg'])[:, 2], 'Yaw')
        Atttg_len = min(len(PData['Atttg']), len(PData['Align']))
        Plotobj8.PlotData(Plotobj8.ax7, PData['AlignT'][:Atttg_len] - self.time0_set,
                          np.array(PData['Atttg'])[:Atttg_len, 2] - np.array(PData['Align'])[:Atttg_len, 2],
                          'Attbg-Atttg(Yaw)')
        Plotobj8.ShowPlotFormat('', 'unit:deg')
        Plotobj8.ax8 = plt.subplot(4, 2, 8)
        Plotobj8.ax8.set_title('Ltgc')
        Plotobj8.PlotData(Plotobj8.ax8, PData['LbgcT'] - self.time0_set, np.array(PData['Lbgc'])[:, 0], 'x')
        Plotobj8.PlotData(Plotobj8.ax8, PData['LbgcT'] - self.time0_set, np.array(PData['Lbgc'])[:, 1], 'y')
        Plotobj8.PlotData(Plotobj8.ax8, PData['LbgcT'] - self.time0_set, np.array(PData['Lbgc'])[:, 2], 'z')
        Plotobj8.PlotData(Plotobj8.ax8, PData['KwsT'] - self.time0_set, np.array(PData['Kws'])[:, 0], 'lcc2')
        Plotobj8.ShowPlotFormat('', 'unit:m')

        # figure9: Kalman.P
        Plotobj9 = PlotGpsInsData()
        Plotobj9.fig.suptitle('Kalman.P')
        Plotobj9.ax1 = plt.subplot(5, 2, 1)
        Plotobj9.ax1.set_title('Pp')
        Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set, np.array(PData['Pp'])[:, 0], 'posLat')
        Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set, np.array(PData['Pp'])[:, 1], 'posLon')
        Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set, np.array(PData['Pp'])[:, 2], 'posH')
        Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set, np.array(PData['Pp'])[:, 0] + np.array(PData['Pp'])[:, 1],
                          'posHorizontal')
        Plotobj9.ShowPlotFormat('', 'unit:m', 0.8)
        Plotobj9.ax2 = plt.subplot(5, 2, 2)
        Plotobj9.ax2.set_title('Pv')
        Plotobj9.PlotData(Plotobj9.ax2, PData['PvT'] - self.time0_set, np.array(PData['Pv'])[:, 0], 'vlat')
        Plotobj9.PlotData(Plotobj9.ax2, PData['PvT'] - self.time0_set, np.array(PData['Pv'])[:, 1], 'vlon')
        Plotobj9.PlotData(Plotobj9.ax2, PData['PvT'] - self.time0_set, np.array(PData['Pv'])[:, 2], 'vh')
        Plotobj9.ShowPlotFormat('', 'unit:m/s', 0.8)
        Plotobj9.ax3 = plt.subplot(5, 2, 3)
        Plotobj9.ax3.set_title('Patt')
        Plotobj9.PlotData(Plotobj9.ax3, PData['PattT'] - self.time0_set, np.array(PData['Patt'])[:, 0], 'roll')
        Plotobj9.PlotData(Plotobj9.ax3, PData['PattT'] - self.time0_set, np.array(PData['Patt'])[:, 1], 'pitch')
        Plotobj9.PlotData(Plotobj9.ax3, PData['PattT'] - self.time0_set, np.array(PData['Patt'])[:, 2], 'yaw')
        Plotobj9.ShowPlotFormat('', 'unit:deg', 0.8)
        Plotobj9.ax4 = plt.subplot(5, 2, 4)
        Plotobj9.ax4.set_title('Pba')
        Plotobj9.PlotData(Plotobj9.ax4, PData['PbaT'] - self.time0_set, np.array(PData['Pba'])[:, 0], 'aex')
        Plotobj9.PlotData(Plotobj9.ax4, PData['PbaT'] - self.time0_set, np.array(PData['Pba'])[:, 1], 'aey')
        Plotobj9.PlotData(Plotobj9.ax4, PData['PbaT'] - self.time0_set, np.array(PData['Pba'])[:, 2], 'aez')
        Plotobj9.ShowPlotFormat('', 'unit:g', 0.8)
        Plotobj9.ax5 = plt.subplot(5, 2, 5)
        Plotobj9.ax5.set_title('Pbg')
        Plotobj9.PlotData(Plotobj9.ax5, PData['PbgT'] - self.time0_set, np.array(PData['Pbg'])[:, 0], 'gex')
        Plotobj9.PlotData(Plotobj9.ax5, PData['PbgT'] - self.time0_set, np.array(PData['Pbg'])[:, 1], 'gey')
        Plotobj9.PlotData(Plotobj9.ax5, PData['PbgT'] - self.time0_set, np.array(PData['Pbg'])[:, 2], 'gez')
        Plotobj9.ShowPlotFormat('', 'unit:deg/s', 0.8)
        Plotobj9.ax6 = plt.subplot(5, 2, 6)
        Plotobj9.ax6.set_title('Palign')
        Plotobj9.PlotData(Plotobj9.ax6, PData['PalignT'] - self.time0_set, np.array(PData['Palign'])[:, 0], 'PCtbRollError')
        Plotobj9.PlotData(Plotobj9.ax6, PData['PalignT'] - self.time0_set, np.array(PData['Palign'])[:, 1], 'PCtbPitchError')
        Plotobj9.PlotData(Plotobj9.ax6, PData['PalignT'] - self.time0_set, np.array(PData['Palign'])[:, 2], 'PCtbYawError')
        Plotobj9.ShowPlotFormat('', 'unit:deg', 0.8)
        Plotobj9.ax7 = plt.subplot(5, 2, 7)
        Plotobj9.ax7.set_title('Plbbg')
        Plotobj9.PlotData(Plotobj9.ax7, PData['PlbbgT'] - self.time0_set, np.array(PData['Plbbg'])[:, 0], 'LttgX')
        Plotobj9.PlotData(Plotobj9.ax7, PData['PlbbgT'] - self.time0_set, np.array(PData['Plbbg'])[:, 1], 'LttgY')
        Plotobj9.PlotData(Plotobj9.ax7, PData['PlbbgT'] - self.time0_set, np.array(PData['Plbbg'])[:, 2], 'LttgZ')
        Plotobj9.ShowPlotFormat('', 'unit:m', 0.8)
        Plotobj9.ax8 = plt.subplot(5, 2, 8)
        Plotobj9.ax8.set_title('Plbbc')
        Plotobj9.PlotData(Plotobj9.ax8, PData['PlbbcT'] - self.time0_set, np.array(PData['Plbbc'])[:, 0], 'LttcX')
        Plotobj9.PlotData(Plotobj9.ax8, PData['PlbbcT'] - self.time0_set, np.array(PData['Plbbc'])[:, 1], 'LttcY')
        Plotobj9.PlotData(Plotobj9.ax8, PData['PlbbcT'] - self.time0_set, np.array(PData['Plbbc'])[:, 2], 'LttcZ')
        Plotobj9.PlotData(Plotobj9.ax8, PData['PkwsT'] - self.time0_set, np.array(PData['Pkws'])[:, 2], 'Lcc2')
        Plotobj9.ShowPlotFormat('', 'unit:m', 0.8)
        Plotobj9.ax9 = plt.subplot(5, 2, 9)
        Plotobj9.ax9.set_title('Pkws')
        Plotobj9.PlotData(Plotobj9.ax9, PData['PkwsT'] - self.time0_set, np.array(PData['Pkws'])[:, 1], 'left')
        Plotobj9.PlotData(Plotobj9.ax9, PData['PkwsT'] - self.time0_set, np.array(PData['Pkws'])[:, 2], 'right')
        Plotobj9.ShowPlotFormat('', 'unit:m/puls', 0.8)
        Plotobj9.ax10 = plt.subplot(5, 2, 10)
        Plotobj9.ax10.set_title('PAtttg')
        Plotobj9.PlotData(Plotobj9.ax10, PData['PAtttgT'] - self.time0_set, np.array(PData['PAtttg'])[:, 0], 'RollError')
        Plotobj9.PlotData(Plotobj9.ax10, PData['PAtttgT'] - self.time0_set, np.array(PData['PAtttg'])[:, 1], 'PitchError')
        Plotobj9.PlotData(Plotobj9.ax10, PData['PAtttgT'] - self.time0_set, np.array(PData['PAtttg'])[:, 2], 'YawError')
        Plotobj9.ShowPlotFormat('', 'unit:deg', 0.8)

        # figure10: Kalman.X
        Plotobj10 = PlotGpsInsData()
        Plotobj10.fig.suptitle('Kalman.X')
        Plotobj10.ax1 = plt.subplot(5, 2, 1)
        Plotobj10.ax1.set_title('Xp')
        Plotobj10.PlotData(Plotobj10.ax1, PData['XpT'] - self.time0_set, np.array(PData['Xp'])[:, 0], 'Xlat')
        Plotobj10.PlotData(Plotobj10.ax1, PData['XpT'] - self.time0_set, np.array(PData['Xp'])[:, 1], 'Xlon')
        Plotobj10.PlotData(Plotobj10.ax1, PData['XpT'] - self.time0_set, np.array(PData['Xp'])[:, 2], 'Xh')
        Plotobj10.ShowPlotFormat('', 'unit:m', 0.8)
        Plotobj10.ax2 = plt.subplot(5, 2, 2)
        Plotobj10.ax2.set_title('Xv')
        Plotobj10.PlotData(Plotobj10.ax2, PData['XvT'] - self.time0_set, np.array(PData['Xv'])[:, 0], 'Xvn')
        Plotobj10.PlotData(Plotobj10.ax2, PData['XvT'] - self.time0_set, np.array(PData['Xv'])[:, 1], 'Xve')
        Plotobj10.PlotData(Plotobj10.ax2, PData['XvT'] - self.time0_set, np.array(PData['Xv'])[:, 2], 'Xvd')
        Plotobj10.ShowPlotFormat('', 'unit:m/s', 0.8)
        Plotobj10.ax3 = plt.subplot(5, 2, 3)
        Plotobj10.ax3.set_title('Xatt')
        Plotobj10.PlotData(Plotobj10.ax3, PData['XattT'] - self.time0_set, np.array(PData['Xatt'])[:, 0], 'Xroll')
        Plotobj10.PlotData(Plotobj10.ax3, PData['XattT'] - self.time0_set, np.array(PData['Xatt'])[:, 1], 'Xpitch')
        Plotobj10.PlotData(Plotobj10.ax3, PData['XattT'] - self.time0_set, np.array(PData['Xatt'])[:, 2], 'Xyaw')
        Plotobj10.ShowPlotFormat('', 'unit:deg', 0.8)
        Plotobj10.ax4 = plt.subplot(5, 2, 4)
        Plotobj10.ax4.set_title('Xba')
        Plotobj10.PlotData(Plotobj10.ax4, PData['XbaT'] - self.time0_set, np.array(PData['Xba'])[:, 0], 'Xbax')
        Plotobj10.PlotData(Plotobj10.ax4, PData['XbaT'] - self.time0_set, np.array(PData['Xba'])[:, 1], 'Xbay')
        Plotobj10.PlotData(Plotobj10.ax4, PData['XbaT'] - self.time0_set, np.array(PData['Xba'])[:, 2], 'Xbaz')
        Plotobj10.ShowPlotFormat('', 'unit:g', 0.8)
        Plotobj10.ax5 = plt.subplot(5, 2, 5)
        Plotobj10.ax5.set_title('Xbg')
        Plotobj10.PlotData(Plotobj10.ax5, PData['XbgT'] - self.time0_set, np.array(PData['Xbg'])[:, 0], 'Xbgx')
        Plotobj10.PlotData(Plotobj10.ax5, PData['XbgT'] - self.time0_set, np.array(PData['Xbg'])[:, 1], 'Xbgy')
        Plotobj10.PlotData(Plotobj10.ax5, PData['XbgT'] - self.time0_set, np.array(PData['Xbg'])[:, 2], 'Xbgz')
        Plotobj10.ShowPlotFormat('', 'uunit:deg/s', 0.8)
        Plotobj10.ax6 = plt.subplot(5, 2, 6)
        Plotobj10.ax6.set_title('Xalign')
        Plotobj10.PlotData(Plotobj10.ax6, PData['XalignT'] - self.time0_set, np.array(PData['Xalign'])[:, 0], 'XalignX')
        Plotobj10.PlotData(Plotobj10.ax6, PData['XalignT'] - self.time0_set, np.array(PData['Xalign'])[:, 1], 'XalignY')
        Plotobj10.PlotData(Plotobj10.ax6, PData['XalignT'] - self.time0_set, np.array(PData['Xalign'])[:, 2], 'XalignZ')
        Plotobj10.ShowPlotFormat('', 'unit:deg', 0.8)
        Plotobj10.ax7 = plt.subplot(5, 2, 7)
        Plotobj10.ax7.set_title('Xlbbg')
        Plotobj10.PlotData(Plotobj10.ax7, PData['XlbbgT'] - self.time0_set, np.array(PData['Xlbbg'])[:, 0], 'XlbbgX')
        Plotobj10.PlotData(Plotobj10.ax7, PData['XlbbgT'] - self.time0_set, np.array(PData['Xlbbg'])[:, 1], 'XlbbgY')
        Plotobj10.PlotData(Plotobj10.ax7, PData['XlbbgT'] - self.time0_set, np.array(PData['Xlbbg'])[:, 2], 'XlbbgZ')
        Plotobj10.ShowPlotFormat('', 'unit:m', 0.8)
        Plotobj10.ax8 = plt.subplot(5, 2, 8)
        Plotobj10.ax8.set_title('Xlbbc')
        Plotobj10.PlotData(Plotobj10.ax8, PData['XlbbcT'] - self.time0_set, np.array(PData['Xlbbc'])[:, 0], 'XlttcX')
        Plotobj10.PlotData(Plotobj10.ax8, PData['XlbbcT'] - self.time0_set, np.array(PData['Xlbbc'])[:, 1], 'XlttcY')
        Plotobj10.PlotData(Plotobj10.ax8, PData['XlbbcT'] - self.time0_set, np.array(PData['Xlbbc'])[:, 2], 'XlttcZ')
        Plotobj10.ShowPlotFormat('', 'unit:m', 0.8)
        Plotobj10.ax9 = plt.subplot(5, 2, 9)
        Plotobj10.ax9.set_title('Xkws')
        Plotobj10.PlotData(Plotobj10.ax9, PData['XkwsT'] - self.time0_set, np.array(PData['Xkws'])[:, 0], 'left')
        Plotobj10.PlotData(Plotobj10.ax9, PData['XkwsT'] - self.time0_set, np.array(PData['Xkws'])[:, 1], 'right')
        Plotobj10.ShowPlotFormat('', 'unit:m/puls', 0.8)
        Plotobj10.ax10 = plt.subplot(5, 2, 10)
        Plotobj10.ax10.set_title('XAtttg')
        Plotobj10.PlotData(Plotobj10.ax10, PData['XAtttgT'] - self.time0_set, np.array(PData['XAtttg'])[:, 0], 'RollError')
        Plotobj10.PlotData(Plotobj10.ax10, PData['XAtttgT'] - self.time0_set, np.array(PData['XAtttg'])[:, 1], 'PitchError')
        Plotobj10.PlotData(Plotobj10.ax10, PData['XAtttgT'] - self.time0_set, np.array(PData['XAtttg'])[:, 2], 'YawError')
        Plotobj10.ShowPlotFormat('', 'unit:m', 0.8)

        plt.show()  # 显示所有图像

    # InsGps杆臂值初始化
    def iniInsGpsBpos(self):
        if self.bpos_refins == {}:
            self.bpos_refins = dict.fromkeys(list(self.SyncRefInsData.keys()), np.array([[0, 0, 0], [0, 0, 0]]))
        if self.bpos_refgps == {}:
            self.bpos_refgps = dict.fromkeys(list(self.SyncRefInsData.keys()), np.array([[0, 0, 0], [0, 0, 0]]))

    # 检查Gpsflag参数是否传入正确，否则默认只画第一个数据的gps
    def checkGpsFlag(self):
        if self.gps_flag is None:
            print('gps_flag is None')
            self.gps_flag = dict.fromkeys(list(self.SyncRefInsData.keys()), 0)
            self.gps_flag[list(self.SyncRefInsData.keys())[0]] = 1
        elif len(self.SyncRefInsData.keys()) != len(self.gps_flag.keys()):
            print('only the first file gps info!')
            self.gps_flag = dict.fromkeys(list(self.SyncRefInsData.keys()), 0)
            self.gps_flag[list(self.SyncRefInsData.keys())[0]] = 1

    # INS和GPS指标数据绘图预处理
    def dataPreStatistics(self):
        for file_name in list(self.SyncRefInsData.keys()):
            try:
                SyncRefInsData = self.SyncRefInsData[file_name]
                self.RefInsData[file_name] = DataStatistics()  # INS和参考数据统计计算
                self.RefInsData[file_name].SpeedCalculation(SyncRefInsData, 2)  # 计算北东地和前右下速度
                self.RefInsData[file_name].PosErrorCalculation(SyncRefInsData, self.bpos_refins[file_name], 2)

                if self.gps_flag[file_name]:
                    SyncRefGpsData = self.SyncRefGpsData[file_name]
                    self.RefGpsData[file_name] = DataStatistics()  # GPSs和参考数据统计计算
                    self.RefGpsData[file_name].PosErrorCalculation(SyncRefGpsData, self.bpos_refgps[file_name], 1)
                    self.RefGpsData[file_name].Gpsflagstransfer(SyncRefGpsData)  # GPS解状态转换
            except Exception as e:
                print(e)
                self.SyncRefInsData.pop(file_name)
                self.SyncRefGpsData.pop(file_name)
                self.gps_flag.pop(file_name)
                if file_name in self.RefInsData.keys():
                    self.RefInsData.pop(file_name)
                if file_name in self.RefGpsData.keys():
                    self.RefGpsData.pop(file_name)

    # 显示时间的方式，second_of_week为显示True周内秒， 为显示Fasle第一组数据的同步的第一个时间
    def checkTimeType2(self):
        if not self.second_of_week:
            self.time0_set = self.SyncRefInsData[list(self.SyncRefInsData.keys())[0]]['time_x'][0]

    def checkTimeType1(self):
        if not self.second_of_week:
            self.time0_set = self.InsDataDF['time'][0]
        else:
            self.time0_set = self.InsDataDF['time'][0] - self.InsDataDF['time'][0]

    # INS数据与参考对比画图
    def PlotRefGpsInsSyncData(self, filePath):
        """

        @author: wenzixuan liqianwen
        :param second_of_week:
        :param gps_flag: {'file1':1, 'file2':0, ...} record 1 means show the gps
        :param filePath: saving path
        """

        if 0 == len(self.SyncRefInsData.keys()):
            return print('no file is required to show !!!')

        # todo
        self.iniInsGpsBpos()    # 杆臂值初始化
        self.checkGpsFlag()   # Gps绘图数初始化
        self.dataPreStatistics()  # 统计指标数据预处理
        self.checkTimeType2()  # 显示时间类型

        color_list = ['r', 'b', 'g', 'yellow', 'silver', 'lightcoral', 'deepskyblue', 'gold', 'limegreen', 'pink']

        # figure1: time differ
        Plotobj1 = PlotGpsInsData()
        Plotobj1.fig.suptitle('时间同步精度')
        Plotobj1.ax1 = plt.subplot(111)
        for file_name in self.SyncRefInsData.keys():
            Plotobj1.PlotData(Plotobj1.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set, self.SyncRefInsData[file_name]['time_x'] - self.SyncRefInsData[file_name]['time_y'], str(file_name + '_Ref-Ins'))
            if self.gps_flag[file_name]:
                Plotobj1.PlotData(Plotobj1.ax1, self.SyncRefGpsData[file_name]['time'] - self.time0_set, self.SyncRefGpsData[file_name]['time'] - self.SyncRefGpsData[file_name]['itow_pos'], str(file_name + '_Ref-Gps'))
        Plotobj1.ShowPlotFormat('', 'unit:s')

        # figure2: Car Path
        Plotobj2 = PlotGpsInsData()
        Plotobj2.fig.suptitle('全局路径')
        Plotobj2.ax1 = plt.subplot(111)
        i = 0
        for file_name in self.SyncRefInsData.keys():
            # 参考和INS
            if 0 == i:
                lat, lon = self.flit_coordinate(self.RefInsData[file_name].Pos["Lat_Y"][0].tolist(),
                                                self.RefInsData[file_name].Pos["Lon_X"][0].tolist())
                Plotobj2.PlotDataXY(Plotobj2.ax1, lon, lat, None, None, color_list[i], '-', '')
                Plotobj2.PlotDataXY(Plotobj2.ax1, lon, lat, 'Ref100C', list(self.SyncRefInsData[file_name]['time_x'] - self.time0_set), color_list[i], '', '.')

            lat, lon = self.flit_coordinate(self.RefInsData[file_name].Pos["Lat_Y"][1].tolist(),self.RefInsData[file_name].Pos["Lon_X"][1].tolist())
            Plotobj2.PlotDataXY(Plotobj2.ax1, lon, lat, None, None, color_list[i + 1], '-', '')
            Plotobj2.PlotDataXY(Plotobj2.ax1, lon, lat, str(file_name + '_Ins'), list(self.SyncRefInsData[file_name]['time_y']-self.time0_set), color_list[i + 1], '', '.')

            # GPS
            if self.gps_flag[file_name]:
                labels = list(self.SyncRefGpsData[file_name]['time'] - self.time0_set)
                Plotobj2.PlotDataXY(Plotobj2.ax1, self.RefGpsData[file_name].Pos["Lon_X"][1],
                                    self.RefGpsData[file_name].Pos["Lat_Y"][1],
                                    None, None, None, '-', '')
                Plotobj2.PlotDataXY(Plotobj2.ax1, self.RefGpsData[file_name].Pos["GpsSigle"][1],
                                    self.RefGpsData[file_name].Pos["GpsSigle"][0], str(file_name + '_Gps_Sigle'),
                                    labels,
                                    None, '', '.')
                Plotobj2.PlotDataXY(Plotobj2.ax1, self.RefGpsData[file_name].Pos["GpsDiff"][1],
                                    self.RefGpsData[file_name].Pos["GpsDiff"][0], str(file_name + '_Gps_Diff'), labels,
                                    None, '', '.')
                Plotobj2.PlotDataXY(Plotobj2.ax1, self.RefGpsData[file_name].Pos["GpsFloat"][1],
                                    self.RefGpsData[file_name].Pos["GpsFloat"][0], str(file_name + '_Gps_Float'),
                                    labels,
                                    None, '', '.')
                Plotobj2.PlotDataXY(Plotobj2.ax1, self.RefGpsData[file_name].Pos["GpsFix"][1],
                                    self.RefGpsData[file_name].Pos["GpsFix"][0],
                                    str(file_name + '_Gps_Fix'), labels, None, '', '.')
            i += 2
        Plotobj2.ShowPlotFormat('经度 unit:meter', '纬度 unit:meter')

        # figure3: Pos
        Plotobj3 = PlotGpsInsData()
        Plotobj3.fig.suptitle('位置')
        Plotobj3.ax1 = plt.subplot(3, 2, 1)
        Plotobj3.ax1.set_title('纬度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj3.PlotData(Plotobj3.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].Pos["Lat_Y"][0], 'Ref100C')
            Plotobj3.PlotData(Plotobj3.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["Lat_Y"][1], str(file_name + '_INS'))
            if self.gps_flag[file_name]:
                Plotobj3.PlotData(Plotobj3.ax1, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["Lat_Y"][1], str(file_name + '_GPS'))
            i += 1
        Plotobj3.ShowPlotFormat('', 'unit:m')
        Plotobj3.ax2 = plt.subplot(3, 2, 3)
        Plotobj3.ax2.set_title('经度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj3.PlotData(Plotobj3.ax2, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].Pos["Lon_X"][0], 'Ref100C')
            Plotobj3.PlotData(Plotobj3.ax2, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["Lon_X"][1], str(file_name + '_INS'))
            if self.gps_flag[file_name]:
                Plotobj3.PlotData(Plotobj3.ax2, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["Lon_X"][1], str(file_name + '_GPS'))
            i += 1
        Plotobj3.ShowPlotFormat('', 'unit:m')
        Plotobj3.ax3 = plt.subplot(3, 2, 5)
        Plotobj3.ax3.set_title('高度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj3.PlotData(Plotobj3.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].Pos["Height_Z"][0], 'Ref100C')
            Plotobj3.PlotData(Plotobj3.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["Height_Z"][1], str(file_name + '_INS'))
            if self.gps_flag[file_name]:
                Plotobj3.PlotData(Plotobj3.ax3, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["Height_Z"][1], str(file_name + '_GPS'))
            i += 1
        Plotobj3.ShowPlotFormat('', 'unit:m')
        Plotobj3.ax5 = plt.subplot(3, 2, 2)
        Plotobj3.ax5.set_title('纬度对比')
        for file_name in self.SyncRefInsData.keys():
            Plotobj3.PlotData(Plotobj3.ax5, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["Lon_X"][0] - self.RefInsData[file_name].Pos["Lon_X"][1],
                              str(file_name + '_Ref-Ins'))
            if self.gps_flag[file_name]:
                Plotobj3.PlotData(Plotobj3.ax5, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["Lon_X"][0] - self.RefGpsData[file_name].Pos["Lon_X"][
                                      1],
                                  str(file_name + '_Ref-Gps'))
        Plotobj3.ShowPlotFormat('', 'unit:m')
        Plotobj3.ax4 = plt.subplot(3, 2, 4)
        Plotobj3.ax4.set_title('经度对比')
        for file_name in self.SyncRefInsData.keys():
            Plotobj3.PlotData(Plotobj3.ax4, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["Lat_Y"][0] - self.RefInsData[file_name].Pos["Lat_Y"][1],
                              str(file_name + '_Ref-Ins'))
            if self.gps_flag[file_name]:
                Plotobj3.PlotData(Plotobj3.ax4, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["Lat_Y"][0] - self.RefGpsData[file_name].Pos["Lat_Y"][
                                      1],
                                  str(file_name + '_Ref-Gps'))
        Plotobj3.ShowPlotFormat('', 'unit:m')
        Plotobj3.ax6 = plt.subplot(3, 2, 6)
        Plotobj3.ax6.set_title('高度对比')
        for file_name in self.SyncRefInsData.keys():
            Plotobj3.PlotData(Plotobj3.ax6, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["Height_Z"][0] -
                              self.RefInsData[file_name].Pos["Height_Z"][1], str(file_name + '_Ref-Ins'))
            if self.gps_flag[file_name]:
                Plotobj3.PlotData(Plotobj3.ax6, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["Height_Z"][0] -
                                  self.RefGpsData[file_name].Pos["Height_Z"][1], str(file_name + '_Ref-Gps'))
        Plotobj3.ShowPlotFormat('', 'unit:m')

        # figure4: Pos Compare
        Plotobj4 = PlotGpsInsData()
        Plotobj4.fig.suptitle('横纵偏差(载体坐标系下)')
        Plotobj4.ax11 = plt.subplot(2, 1, 1)
        Plotobj4.ax11.set_title('横向偏差')
        for file_name in self.SyncRefInsData.keys():
            Plotobj4.PlotData(Plotobj4.ax11, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["PosXError"], str(file_name + '_Ref-Ins'))
            if self.gps_flag[file_name]:
                Plotobj4.PlotData(Plotobj4.ax11, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["PosXError"], str(file_name + '_Ref-Gps'))
        Plotobj4.ShowPlotFormat('', 'unit:m')
        if plt.ylim()[0] < -10 or plt.ylim()[1] > 10:
            Plotobj4.ax11.set_ylim(-10, 10)
        Plotobj4.ax12 = Plotobj4.ax11.twinx()
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if self.gps_flag[file_name]:
                Plotobj4.ax12.plot(self.SyncRefGpsData[file_name]['time'] - self.time0_set, self.RefGpsData[file_name].Pos["GpsFlags"],
                                   label=file_name + '\n1: 单点\n2: 差分\n4: 固定\n5: 浮点', color=color_list[i],
                                   linestyle="", marker='*')
            i += 1
        Plotobj4.ax12.set_ylim(0.5, 5.5)
        Plotobj4.ax12.yaxis.set_major_locator(MultipleLocator(1))
        plt.legend(loc='lower right')

        Plotobj4.ax21 = plt.subplot(2, 1, 2)
        Plotobj4.ax21.set_title('纵向偏差')
        for file_name in self.SyncRefInsData.keys():
            Plotobj4.PlotData(Plotobj4.ax21, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["PosYError"], str(file_name + '_Ref-Ins'))
            if self.gps_flag[file_name]:
                Plotobj4.PlotData(Plotobj4.ax21, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["PosYError"], str(file_name + '_Ref-Gps'))
        Plotobj4.ShowPlotFormat('', 'unit:m')
        if plt.ylim()[0] < -10 or plt.ylim()[1] > 10:
            Plotobj4.ax11.set_ylim(-10, 10)
        Plotobj4.ax22 = Plotobj4.ax21.twinx()
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if self.gps_flag[file_name]:
                Plotobj4.ax22.plot(self.SyncRefGpsData[file_name]['time'] - self.time0_set, self.RefGpsData[file_name].Pos["GpsFlags"],
                                   label=file_name + '\n1: 单点\n2: 差分\n4: 固定\n5: 浮点', color=color_list[i],
                                   linestyle="", marker='*')
            i += 1
        Plotobj4.ax22.set_ylim(0.5, 5.5)
        Plotobj4.ax22.yaxis.set_major_locator(MultipleLocator(1))
        plt.legend(loc='lower right')

        # figure5: Pos Err
        Plotobj5 = PlotGpsInsData()
        Plotobj5.ax1 = plt.subplot(111)
        Plotobj5.fig.suptitle('水平偏差(载体坐标系下)')
        for file_name in self.SyncRefInsData.keys():
            Plotobj5.PlotData(Plotobj5.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].Pos["PosXYError"], str(file_name + '_Ref-Ins'))
            if self.gps_flag[file_name]:
                Plotobj5.PlotData(Plotobj5.ax1, self.SyncRefGpsData[file_name]['time'] - self.time0_set,
                                  self.RefGpsData[file_name].Pos["PosXYError"], str(file_name + '_Ref-Gps'))
        Plotobj5.ShowPlotFormat('', 'unit:m')
        if plt.ylim()[1] > 10:
            Plotobj5.ax1.set_ylim(0, 10)
        Plotobj5.ax2 = Plotobj5.ax1.twinx()
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if self.gps_flag[file_name]:
                Plotobj5.ax2.plot(self.SyncRefGpsData[file_name]['time'] - self.time0_set, self.RefGpsData[file_name].Pos["GpsFlags"],
                                  label=file_name + '\n1: 单点\n2: 差分\n4: 固定\n5: 浮点', color=color_list[i],
                                  linestyle="", marker='*')
            i += 1
        Plotobj5.ax2.set_ylim(0.5, 5.5)
        Plotobj5.ax2.yaxis.set_major_locator(MultipleLocator(1))
        plt.legend(loc='lower right')

        # figure6: Att
        Plotobj6 = PlotGpsInsData()
        Plotobj6.fig.suptitle('姿态')
        Plotobj6.ax1 = plt.subplot(3, 2, 1)
        Plotobj6.ax1.set_title('Roll')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj6.PlotData(Plotobj6.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.SyncRefInsData[file_name]["roll_x"], 'Ref100C')
            Plotobj6.PlotData(Plotobj6.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.SyncRefInsData[file_name]["roll_y"], str(file_name + '_Ins'))
            i += 1
        Plotobj6.ShowPlotFormat('', 'unit:deg')
        Plotobj6.ax2 = plt.subplot(3, 2, 2)
        Plotobj6.ax2.set_title('Roll对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["roll"] = self.SyncRefInsData[file_name]["roll_x"] - \
                                                       self.SyncRefInsData[file_name]["roll_y"]
            Plotobj6.PlotData(Plotobj6.ax2, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["roll"], str(file_name) + '_Ref-Ins')
        Plotobj6.ShowPlotFormat('', 'unit:deg')
        Plotobj6.ax3 = plt.subplot(3, 2, 3)
        Plotobj6.ax3.set_title('Pitch')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj6.PlotData(Plotobj6.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.SyncRefInsData[file_name]["pitch_x"], 'Ref100C')
            Plotobj6.PlotData(Plotobj6.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.SyncRefInsData[file_name]["pitch_y"], str(file_name) + '_Ins')
            i += 1
        Plotobj6.ShowPlotFormat('', 'unit:deg')
        Plotobj6.ax4 = plt.subplot(3, 2, 4)
        Plotobj6.ax4.set_title('Pitch对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["pitch"] = self.SyncRefInsData[file_name]["pitch_x"] - \
                                                        self.SyncRefInsData[file_name]["pitch_y"]
            Plotobj6.PlotData(Plotobj6.ax4, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["pitch"], str(file_name) + '_Ref-Ins')
        Plotobj6.ShowPlotFormat('', 'unit:deg')
        Plotobj6.ax5 = plt.subplot(3, 2, 5)
        Plotobj6.ax5.set_title('Yaw')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj6.PlotData(Plotobj6.ax5, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.SyncRefInsData[file_name]["yaw_x"], 'Ref100C')
            Plotobj6.PlotData(Plotobj6.ax5, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.SyncRefInsData[file_name]["yaw_y"], str(file_name) + '_Ins')
            i += 1
        Plotobj6.ShowPlotFormat('', 'unit:deg')
        Plotobj6.ax6 = plt.subplot(3, 2, 6)
        Plotobj6.ax6.set_title('Yaw对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["yaw"] = angele_standardization(
                np.array(self.SyncRefInsData[file_name]["yaw_x"] - self.SyncRefInsData[file_name]["yaw_y"]))
            Plotobj6.PlotData(Plotobj6.ax6, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["yaw"], str(file_name) + '_Ref-Ins')
        Plotobj6.ShowPlotFormat('', 'unit:deg')

        # figure7: NED-Speed
        Plotobj7 = PlotGpsInsData()
        Plotobj7.fig.suptitle('速度（导航坐标系）')
        Plotobj7.ax1 = plt.subplot(3, 2, 1)
        Plotobj7.ax1.set_title('北向速度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj7.PlotData(Plotobj7.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].REFSpeed["NorthVelocity"], 'Ref100C')
            Plotobj7.PlotData(Plotobj7.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].INSSpeed["NorthVelocity"], str(file_name) + '_Ins')
            i += 1
        Plotobj7.ShowPlotFormat('', 'unit:m/s')
        Plotobj7.ax2 = plt.subplot(3, 2, 2)
        Plotobj7.ax2.set_title('北向速度对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["vel_n"] = self.RefInsData[file_name].REFSpeed["NorthVelocity"] - \
                                                        self.RefInsData[file_name].INSSpeed["NorthVelocity"]
            Plotobj7.PlotData(Plotobj7.ax2, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["vel_n"], str(file_name) + '_Ref-Ins')
        Plotobj7.ShowPlotFormat('', 'unit:m/s')
        Plotobj7.ax3 = plt.subplot(3, 2, 3)
        Plotobj7.ax3.set_title('东向速度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj7.PlotData(Plotobj7.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].REFSpeed["EastVelocity"], 'Ref100C')
            Plotobj7.PlotData(Plotobj7.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].INSSpeed["EastVelocity"], str(file_name) + '_Ins')
            i += 1
        Plotobj7.ShowPlotFormat('', 'unit:m/s')
        Plotobj7.ax4 = plt.subplot(3, 2, 4)
        Plotobj7.ax4.set_title('东向速度对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["vel_e"] = self.RefInsData[file_name].REFSpeed["EastVelocity"] - \
                                                        self.RefInsData[file_name].INSSpeed["EastVelocity"]
            Plotobj7.PlotData(Plotobj7.ax4, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["vel_e"], str(file_name) + '_Ref-Ins')
        Plotobj7.ShowPlotFormat('', 'unit:m/s')
        Plotobj7.ax5 = plt.subplot(3, 2, 5)
        Plotobj7.ax5.set_title('地向速度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj7.PlotData(Plotobj7.ax5, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].REFSpeed["GroundVelocity"], 'Ref100C')
            Plotobj7.PlotData(Plotobj7.ax5, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].INSSpeed["GroundVelocity"], str(file_name) + '_Ins')
            i += 1
        Plotobj7.ShowPlotFormat('', 'unit:m/s')
        Plotobj7.ax6 = plt.subplot(3, 2, 6)
        Plotobj7.ax6.set_title('地向速度对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["vel_g"] = self.RefInsData[file_name].REFSpeed["GroundVelocity"] - \
                                                        self.RefInsData[file_name].INSSpeed["GroundVelocity"]
            Plotobj7.PlotData(Plotobj7.ax6, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["vel_g"], str(file_name) + '_Ref-Ins')
        Plotobj7.ShowPlotFormat('', 'unit:m/s')

        # figure8: XYZ-Speed
        Plotobj8 = PlotGpsInsData()
        Plotobj8.fig.suptitle('速度（载体坐标系）')
        Plotobj8.ax1 = plt.subplot(3, 2, 1)
        Plotobj8.ax1.set_title('前向速度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj8.PlotData(Plotobj8.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].REFSpeed["ForwardVelocity"], 'Ref100C')
            Plotobj8.PlotData(Plotobj8.ax1, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].INSSpeed["ForwardVelocity"], str(file_name) + '_Ins')
            i += 1
        Plotobj8.ShowPlotFormat('', 'unit:m/s')
        Plotobj8.ax2 = plt.subplot(3, 2, 2)
        Plotobj8.ax2.set_title('前向速度对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["vel_x"] = self.RefInsData[file_name].REFSpeed["ForwardVelocity"] - \
                                                        self.RefInsData[file_name].INSSpeed["ForwardVelocity"]
            Plotobj8.PlotData(Plotobj8.ax2, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["vel_x"], str(file_name) + '_Ref-Ins')
        Plotobj8.ShowPlotFormat('', 'unit:m/s')
        Plotobj8.ax3 = plt.subplot(3, 2, 3)
        Plotobj8.ax3.set_title('右向速度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj8.PlotData(Plotobj8.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].REFSpeed["RightVelocity"], 'Ref100C')
            Plotobj8.PlotData(Plotobj8.ax3, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].INSSpeed["RightVelocity"], str(file_name) + '_Ins')
            i += 1
        Plotobj8.ShowPlotFormat('', 'unit:m/s')
        Plotobj8.ax4 = plt.subplot(3, 2, 4)
        Plotobj8.ax4.set_title('右向速度对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["vel_y"] = self.RefInsData[file_name].REFSpeed["RightVelocity"] - \
                                                        self.RefInsData[file_name].INSSpeed["RightVelocity"]
            Plotobj8.PlotData(Plotobj8.ax4, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["vel_y"], str(file_name) + '_Ref-Ins')
        Plotobj8.ShowPlotFormat('', 'unit:m/s')
        Plotobj8.ax5 = plt.subplot(3, 2, 5)
        Plotobj8.ax5.set_title('下向速度')
        i = 0
        for file_name in self.SyncRefInsData.keys():
            if 0 == i:
                Plotobj8.PlotData(Plotobj8.ax5, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                                  self.RefInsData[file_name].REFSpeed["DownwardVelocity"], 'Ref100C')
            Plotobj8.PlotData(Plotobj8.ax5, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].INSSpeed["DownwardVelocity"], str(file_name) + '_Ins')
            i += 1
        Plotobj8.ShowPlotFormat('', 'unit:m/s')
        Plotobj8.ax6 = plt.subplot(3, 2, 6)
        Plotobj8.ax6.set_title('下向速度对比')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["vel_z"] = self.RefInsData[file_name].REFSpeed["DownwardVelocity"] - \
                                                        self.RefInsData[file_name].INSSpeed["DownwardVelocity"]
            Plotobj8.PlotData(Plotobj8.ax6, self.SyncRefInsData[file_name]['time_x'] - self.time0_set,
                              self.RefInsData[file_name].error["vel_z"], str(file_name) + '_Ref-Ins')
        Plotobj8.ShowPlotFormat('', 'unit:m/s')

        # figure9: Statistics 精度统计
        Plotobj9 = PlotGpsInsData()
        Plotobj9.fig.suptitle('精度统计')
        # 水平偏差统计
        Plotobj9.ax1 = plt.subplot(3, 1, 1)
        Plotobj9.ax1.set_title('水平偏差统计')
        for file_name in self.SyncRefInsData.keys():
            bin_edges, cdf_error = DataCDF(self.RefInsData[file_name].Pos["PosXYError"])
            Plotobj9.PlotData(Plotobj9.ax1, bin_edges, cdf_error, str(file_name) + '_Ins')
            if self.gps_flag[file_name]:
                bin_edges, cdf_error = DataCDF(self.RefGpsData[file_name].Pos["PosXYError"])
                Plotobj9.PlotData(Plotobj9.ax1, bin_edges, cdf_error, str(file_name) + '_Gps')
        Plotobj9.ax1.set_ylim(0, 1)
        Plotobj9.ax1.set_xlim(0, 0.5)
        Plotobj9.ShowPlotFormat('unit:m', '占比：%', 0.6)
        # 航向偏差统计
        Plotobj9.ax2 = plt.subplot(3, 1, 2)
        Plotobj9.ax2.set_title('航向偏差统计')
        for file_name in self.SyncRefInsData.keys():
            bin_edges, cdf_error = DataCDF(abs(self.RefInsData[file_name].error["yaw"]))
            Plotobj9.PlotData(Plotobj9.ax2, bin_edges, cdf_error, str(file_name) + '_Ins')
        Plotobj9.ax2.set_ylim(0, 1)
        Plotobj9.ax2.set_xlim(0, 0.5)
        Plotobj9.ShowPlotFormat('unit:deg', '占比：%', 0.6)
        # 速度偏差统计
        Plotobj9.ax3 = plt.subplot(3, 1, 3)
        Plotobj9.ax3.set_title('速度偏差统计')
        for file_name in self.SyncRefInsData.keys():
            self.RefInsData[file_name].error["vel"] = np.sqrt(
                pow(self.RefInsData[file_name].error["vel_x"], 2) + pow(self.RefInsData[file_name].error["vel_y"], 2))
            bin_edges, cdf_error = DataCDF(self.RefInsData[file_name].error["vel"])
            Plotobj9.PlotData(Plotobj9.ax3, bin_edges, cdf_error, str(file_name) + '_Ins')
        Plotobj9.ax3.set_ylim(0, 1)
        Plotobj9.ax3.set_xlim(0, 0.5)
        Plotobj9.ShowPlotFormat('unit:m/s', '占比：%', 0.6)

        savedata = []
        statisticsgpsflag_all = {}
        gps_filename = []
        statisticsgpsflag = []
        for name, value in self.gps_flag.items():
            if value:
                gps_filename.append(name)

        for file_name in self.SyncRefInsData.keys():
            statisticslist = self.RefInsData[file_name].StatisticSyncData(self.SyncRefInsData[file_name],
                                                                          file_name + "_Ins")  # INS数据与参考对比统计
            if self.gps_flag[file_name]:
                statisticsgpsflag = self.RefGpsData[file_name].StatisticGpsFlag(self.SyncRefGpsData[file_name],
                                                                                file_name + "_gps")  # GPS解状态统计
            if file_name in gps_filename:
                statisticsgpsflag_all[file_name] = statisticsgpsflag
            if not savedata:
                savedata = [statisticslist, statisticsgpsflag]
            else:
                for item in statisticslist:
                    savedata[0][item].append(statisticslist[item][0])
                for item in statisticsgpsflag:
                    savedata[1][item].append(statisticsgpsflag[item][0])
        SaveStatisticToExcel(savedata, filePath)

        if statisticsgpsflag_all:
            flags = ['固定', '浮点', '差分', '单点', 'None']
            Plotobj10 = plt.figure(figsize=(10, 10))
            plt.suptitle('GPS解状态占比')
            if sum(list(self.gps_flag.values())) in [0, 1]:
                file_name = gps_filename[0] if len(gps_filename) == 1 else file_name
                sub = plt.subplot(1, 1, 1)
                sub.set_title(file_name)
                items = []
                percent = []
                for flag in flags:
                    if statisticsgpsflag_all[file_name][flag][0] != 0:
                        items.append(flag)
                for item in items:
                    percent.append(statisticsgpsflag_all[file_name][item][0])
                sub.pie(percent, labels=items, autopct='%1.2f%%')
            else:
                sub_pic_num = sum(self.gps_flag.values())
                if math.ceil(pow(sub_pic_num, 0.5)) * (math.ceil(pow(sub_pic_num, 0.5)) - 1) < sub_pic_num:
                    row, col = math.ceil(pow(sub_pic_num, 0.5)), math.ceil(pow(sub_pic_num, 0.5))
                else:
                    row, col = math.ceil(pow(sub_pic_num, 0.5)), (math.ceil(pow(sub_pic_num, 0.5)) - 1)

                for i in range(sub_pic_num):
                    file_name = gps_filename[i]
                    sub = plt.subplot(col, row, i + 1)
                    sub.set_title(file_name)
                    sub.axis('equal')
                    items = []
                    percent = []
                    for flag in flags:
                        if statisticsgpsflag_all[file_name][flag][0] != 0:
                            items.append(flag)
                    for item in items:
                        percent.append(statisticsgpsflag_all[file_name][item][0])

                    sub.pie(percent, labels=items, autopct='%1.2f%%')
            Plotobj10.tight_layout()

        plt.ioff()
        plt.show()  # 显示所有图像
        return

    @staticmethod
    def flit_coordinate(lat, lon):
        if len(lat) == 0 or len(lon) == 0:
            return [], []
        i = 0
        while i < min(len(lon), len(lat)):
            if abs(lat[i]) < 1 or abs(lon[i]) < 1:
                lat.pop(i)
                lon.pop(i)
            # if abs(lat[i]) > 2560000 or abs(lon[i]) > 12633960:
            #     lat.pop(i)
            #     lon.pop(i)
            i += 1
        return lat, lon


def SaveStatisticToExcel(savedata, filePath):
    print("filePath:  ", filePath + '/statistic.xlsx')
    writer = pd.ExcelWriter(filePath + '/statistic.xlsx')

    data_1 = pd.DataFrame.from_dict(savedata[0])
    data_2 = pd.DataFrame.from_dict(savedata[1])
    data_1.to_excel(writer, 'INS精度统计')  # 不要行和列的标签
    data_2.to_excel(writer, 'GPS解状态统计')
    writer.save()


def angele_standardization(angle):
    # 所有角度统一到正负180°之间, 输入angel为array
    angle[np.where(angle > 180)] = angle[np.where(angle > 180)] - 360
    angle[np.where(angle < -180)] = angle[np.where(angle < -180)] + 360
    return angle


def DataCDF(Data):
    hist, bin_edges = np.histogram(Data, bins=len(Data))
    cdf = np.cumsum(hist / sum(hist))
    return bin_edges[1:], cdf


def flit_time_files(ins_data_dict, gps_data_dict, gps_flag):
    for file_name in ins_data_dict.keys():
        if gps_flag[file_name]:
            time = [0]
            for i in range(1, len(gps_data_dict[file_name]['time'])):
                time.append(
                    gps_data_dict[file_name]['time'][i] - gps_data_dict[file_name]['time'][0])
            gps_data_dict[file_name]['time'] = time

        time_x = [0]
        for i in range(1, len(ins_data_dict[file_name]['time_x'])):
            time_x.append(
                ins_data_dict[file_name]['time_x'][i] - ins_data_dict[file_name]['time_x'][0])
        ins_data_dict[file_name]['time_x'] = time_x


def flit_time_file(InsData, ImuData, GpsData, InsGpsSyncData, VehicleData, PData):
    time = [0]
    for i in range(1, len(InsData['time'])):
        time.append(InsData['time'][i] - InsData['time'][0])
    InsData['time'] = time
    time = [0]
    for i in range(1, len(ImuData['time'])):
        time.append(ImuData['time'][i] - ImuData['time'][0])
    ImuData['time'] = time
    time = [0]
    for i in range(1, len(GpsData['ts'])):
        time.append(GpsData['ts'][i] - GpsData['ts'][0])
    GpsData['ts'] = time
    # time = [0]
    # for i in range(1, len(SyncData['ts'])):
    #     time.append(SyncData['ts'][i]-SyncData['ts'][0])
    # SyncData['ts'] = time
    time = [0]
    for i in range(1, len(InsGpsSyncData['time'])):
        time.append(InsGpsSyncData['time'][i] - InsGpsSyncData['time'][0])
    InsGpsSyncData['time'] = time
    time = [0]
    for i in range(1, len(VehicleData['ts'])):
        time.append(VehicleData['ts'][i] - VehicleData['ts'][0])
    VehicleData['ts'] = time

    time = [0]
    for i in range(1, len(GpsData['itow_heading'])):
        time.append(GpsData['itow_heading'][i] - GpsData['itow_heading'][0])
    GpsData['itow_heading'] = time
    time = [0]
    for i in range(1, len(GpsData['itow_pos'])):
        time.append(GpsData['itow_pos'][i] - GpsData['itow_pos'][0])
    GpsData['itow_pos'] = time
    time = [0]
    for i in range(1, len(GpsData['itow_vel'])):
        time.append(GpsData['itow_vel'][i] - GpsData['itow_vel'][0])
    GpsData['itow_vel'] = time

    items = ['BaT', 'BgT', 'AlignT', 'LbbgT', 'LbbcT', 'KwsT', 'AtttgT', 'LbgcT'
        , 'PpT', 'PvT', 'PattT', 'PbaT', 'PbgT', 'PalignT', 'PlbbgT', 'PlbbcT', 'PkwsT', 'PAtttgT'
        , 'XpT', 'XvT', 'XattT', 'XbaT', 'XbgT', 'XalignT', 'XlbbgT', 'XlbbcT', 'XkwsT', 'XAtttgT']
    for item in items:
        try:
            if type(PData[item][0]) != list:
                time = [0]
                for i in range(1, len(PData[item])):
                    time.append(PData[item][i] - PData[item][0])
                PData[item] = time
            else:
                time = [[0]]
                for i in range(1, len(PData[item])):
                    time.append([PData[item][i][0] - PData[item][0][0]])
                PData[item] = time
        except Exception as e:
            print(PData[item][:10])
            print(e)
    return InsData, ImuData, GpsData, InsGpsSyncData, VehicleData, PData

# if __name__ == "__main__":
# #     HexDataParse()
# #     # hexDataParseObj.filePath = "E://PycharmProjects/DataParse/12302.txt"
