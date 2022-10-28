import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Cursor
import mplcursors
from matplotlib.pyplot import MultipleLocator
from func.DataStatistics import DataStatistics
import pandas as pd
import math

plt.rc("font", family='MicroSoft YaHei', weight='bold')


class PlotGpsInsData:
    def __init__(self, parent=None):
        super().__init__()
        # 实例化数据解析对象，默认1个figure只有1个图像
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.window.showMaximized()

    def call_back(self, event):
        # 使用鼠标滚轮放大/缩小图像的X轴
        axtemp = event.inaxes
        x_min, x_max = axtemp.get_xlim()
        scale = (x_max - x_min) / 10
        if event.button == 'up':
            axtemp.set(xlim=(x_min + scale, x_max - scale))
            print('up')
        elif event.button == 'down':
            axtemp.set(xlim=(x_min - scale, x_max + scale))
            print('down')
        self.fig.canvas.draw_idle()  # 绘图动作实时反映在图像上

    def call_back_xy(self, event):
        # 使用鼠标滚轮等比例放大/缩小图像的XY轴
        axtemp = event.inaxes
        x_min, x_max = axtemp.get_xlim()
        y_min, y_max = axtemp.get_ylim()
        xscale = (x_max - x_min) / 10
        yscale = (y_max - y_min) / (x_max - x_min) * (x_max - x_min) / 10
        if event.button == 'up':
            axtemp.set(xlim=(x_min + xscale, x_max - xscale))
            axtemp.set(ylim=(y_min + yscale, y_max - yscale))
            print('up')
        elif event.button == 'down':
            axtemp.set(xlim=(x_min - xscale, x_max + xscale))
            axtemp.set(ylim=(y_min - yscale, y_max + yscale))
            print('down')
        self.fig.canvas.draw_idle()  # 绘图动作实时反映在图像上

    def PlotData(self, ax, x, y, label):
        # 线性图绘图
        # ax:绘图, x：自变量, y：因变量, label：因变量标签
        lines = ax.plot(x, y, label=label, linewidth=1, alpha=0.7)
        self.fig.canvas.mpl_connect('scroll_event', self.call_back)
        mplcursors.cursor(lines, multiple=True)  # 数据游标

    def PlotDataXY(self, ax, x, y, label, labels, color, linestyle, marker):
        # 绘制等比例图 + 游标显示标签信息
        # ax:绘图, x：自变量, y：因变量, label：因变量标签, labels:其他标签, linestyle:线条格式, marker：
        ax.axis('equal')
        ax = plt.gca()
        if color is not None:
            lines = ax.plot(x, y, label=label, linewidth=1, linestyle=linestyle, marker=marker, color=color, alpha=0.7)
        else:
            lines = ax.plot(x, y, label=label, linewidth=1, linestyle=linestyle, marker=marker, alpha=0.7)
        self.fig.canvas.mpl_connect('scroll_event', self.call_back_xy)
        if labels:
            cursor = mplcursors.cursor(lines, hover=True)  # 数据游标
            cursor.connect("add", lambda sel: sel.annotation.set_text(labels[sel.target.index]))

    def ShowPlotFormat(self, xlabel, ylabel, hspace=0.5):
        # 统一绘图样式设置
        # 设置（xlabel：X坐标轴名称，ylabel：Y坐标轴名称）
        plt.rc("font", family='MicroSoft YaHei', weight='bold')
        plt.subplots_adjust(hspace=hspace)
        plt.autoscale(enable=True, axis='y')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid()
        plt.legend(loc='best')


class PlotGpsInsRawSyncData:
    """
    plot INS数据与自身GPS对比画图 & INS数据与参考对比画图
    @author: wenzixuan liqianwen
    @version: for analyse db files
    @date: 2022-10-12
    """

    def __init__(self, ref_type='100C'):
        self.time0_set = 0
        self.gps_flag = None
        self.second_of_week = True
        self.ref_type = ref_type
        self.color_list = ['r', 'g', 'b', 'c', 'm', 'grey']
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

    def PlotGpsInsRawSyncData(self):
        """
        INS数据与自身GPS对比画图
        :return: 10 picture
        """

        def plot_data_diff(name='DataDiff'):
            Plotobj1 = PlotGpsInsData()
            Plotobj1.fig.suptitle(name)
            Plotobj1.ax1 = plt.subplot(3, 2, 1)
            Plotobj1.ax1.set_title('0A')
            Plotobj1.PlotData(Plotobj1.ax1, ImuData['time'][:-1] - self.time0_set, np.diff(ImuData['time']),
                              '0A_diff')
            Plotobj1.ShowPlotFormat('', 'unit:s')
            Plotobj1.ax2 = plt.subplot(3, 2, 3)
            Plotobj1.ax2.set_title('0B')
            Plotobj1.PlotData(Plotobj1.ax2, InsData['time'][:-1] - self.time0_set, np.diff(InsData['time']),
                              '0B_diff')
            Plotobj1.ShowPlotFormat('', 'unit:s')
            Plotobj1.ax3 = plt.subplot(3, 2, 5)
            Plotobj1.ax3.set_title('0C')
            Plotobj1.PlotData(Plotobj1.ax3, SyncData['ts'][:-1] - self.time0_set, np.diff(SyncData['imuTime']),
                              '0C_imuTime_diff')
            Plotobj1.PlotData(Plotobj1.ax3, SyncData['ts'][:-1] - self.time0_set, np.diff(SyncData['gpsTime']),
                              '0C_gpsTime_diff')
            Plotobj1.PlotData(Plotobj1.ax3, SyncData['ts'][:-1] - self.time0_set, np.diff(SyncData['ts']), '0C_ts_diff')
            Plotobj1.ShowPlotFormat('', 'unit:s')
            Plotobj1.ax4 = plt.subplot(3, 2, 2)
            Plotobj1.ax4.set_title('10_bestpos')
            Plotobj1.PlotData(Plotobj1.ax4, GpsData['ts'][:-1] - self.time0_set, np.diff(GpsData['itow_pos']),
                              '10_bestpos_diff')
            Plotobj1.ShowPlotFormat('', 'unit:s')
            Plotobj1.ax5 = plt.subplot(3, 2, 4)
            Plotobj1.ax5.set_title('10_bestvel')
            Plotobj1.PlotData(Plotobj1.ax5, GpsData['ts'][:-1] - self.time0_set, np.diff(GpsData['itow_vel']),
                              '10_bestvel_diff')
            Plotobj1.ShowPlotFormat('', 'unit:s')
            Plotobj1.ax6 = plt.subplot(3, 2, 6)
            Plotobj1.ax6.set_title('10_heading')
            Plotobj1.PlotData(Plotobj1.ax6, GpsData['ts'][:-1] - self.time0_set, np.diff(GpsData['itow_heading']),
                              '10_heading_diff')
            Plotobj1.ShowPlotFormat('', 'unit:s')

        def plot_attitude(name='Angle'):
            Plotobj2 = PlotGpsInsData()
            Plotobj2.fig.suptitle(name)
            Plotobj2.ax1 = plt.subplot(2, 1, 1)
            Plotobj2.ax1.set_title('yaw')
            Plotobj2.PlotData(Plotobj2.ax1, GpsData['itow_heading'] - self.time0_set, GpsData['Heading'], 'GPSYaw')
            if 'headingStd' in GpsData.keys():
                Plotobj2.PlotData(Plotobj2.ax1, GpsData['itow_pos'] - self.time0_set, GpsData['headingStd'],
                                  'HeadingAccuracy')
            Plotobj2.PlotData(Plotobj2.ax1, InsData['time'] - self.time0_set, InsData['yaw'], 'INSYaw')
            Plotobj2.PlotData(Plotobj2.ax1, GpsData['itow_vel'] - self.time0_set, GpsData['TrackAngle'],
                              'GPSTrackAngle')
            Plotobj2.ShowPlotFormat('', 'unit:deg')
            Plotobj2.ax2 = plt.subplot(2, 1, 2)
            Plotobj2.ax2.set_title('att')
            GPSYaw_INSYaw = angele_standardization(np.array(InsGpsSyncData['Heading'] - InsGpsSyncData['yaw']))
            GPSTrackAngle_INSYaw = angele_standardization(
                np.array(InsGpsSyncData['TrackAngle'] - InsGpsSyncData['yaw']))
            Plotobj2.PlotData(Plotobj2.ax2, InsData['time'] - self.time0_set, InsData['roll'], 'INSRoll')
            Plotobj2.PlotData(Plotobj2.ax2, InsData['time'] - self.time0_set, InsData['pitch'], 'INSPitch')
            Plotobj2.PlotData(Plotobj2.ax2, GpsData['itow_heading'] - self.time0_set, GpsData['Pitch'], 'GPSPitch')
            Plotobj2.PlotData(Plotobj2.ax2, InsGpsSyncData['time'] - self.time0_set, GPSYaw_INSYaw, 'GPSYaw-INSYaw')
            Plotobj2.PlotData(Plotobj2.ax2, InsGpsSyncData['time'] - self.time0_set, GPSTrackAngle_INSYaw,
                              'GPSTrackAngle-INSYaw')
            Plotobj2.ShowPlotFormat('', 'unit:deg')

        def plot_velocity(name='Speed'):
            Plotobj3 = PlotGpsInsData()
            Plotobj3.fig.suptitle(name)
            Plotobj3.ax1 = plt.subplot(3, 2, 1)
            Plotobj3.ax1.set_title('North_Speed')
            Plotobj3.PlotData(Plotobj3.ax1, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.GPSSpeed["NorthVelocity"], 'GPS')
            Plotobj3.PlotData(Plotobj3.ax1, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.INSSpeed["NorthVelocity"], 'INS')
            Plotobj3.PlotData(Plotobj3.ax1, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.GpsInsSpeedDiff["NorthVelocity"],
                              'GPS-INS')
            Plotobj3.ShowPlotFormat('', 'unit:m/s')
            Plotobj3.ax2 = plt.subplot(3, 2, 3)
            Plotobj3.ax2.set_title('East_Speed')
            Plotobj3.PlotData(Plotobj3.ax2, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.GPSSpeed["EastVelocity"], 'GPS')
            Plotobj3.PlotData(Plotobj3.ax2, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.INSSpeed["EastVelocity"], 'INS')
            Plotobj3.PlotData(Plotobj3.ax2, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.GpsInsSpeedDiff["EastVelocity"], 'GPS-INS')
            Plotobj3.ShowPlotFormat('', 'unit:m/s')
            Plotobj3.ax3 = plt.subplot(3, 2, 5)
            Plotobj3.ax3.set_title('Ground_Speed')
            Plotobj3.PlotData(Plotobj3.ax3, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.GPSSpeed["GroundVelocity"], 'GPS')
            Plotobj3.PlotData(Plotobj3.ax3, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.INSSpeed["GroundVelocity"], 'INS')
            Plotobj3.PlotData(Plotobj3.ax3, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.GpsInsSpeedDiff["GroundVelocity"],
                              'GPS-INS')
            Plotobj3.ShowPlotFormat('', 'unit:m/s')
            Plotobj3.ax4 = plt.subplot(3, 2, 2)
            Plotobj3.ax4.set_title('Forward_Speed_VBX')
            Plotobj3.PlotData(Plotobj3.ax4, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.INSSpeed["ForwardVelocity"], 'INS')
            Plotobj3.PlotData(Plotobj3.ax4, InsGpsSyncData['time'] - self.time0_set, InsGpsSyncData['HSpd'], 'GPS')
            Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["DriR_L"], 'DriR_L')
            Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["DriR_R"], 'DriR_R')
            Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["NonDriR_L"],
                              'NonDriR_L')
            Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, DataStatiObj.VehSpd["NonDriR_R"],
                              'NonDriR_R')
            Plotobj3.PlotData(Plotobj3.ax4, VehicleData['ts'] - self.time0_set, VehicleData["gear"], 'Shifter')
            Plotobj3.ShowPlotFormat('', 'unit:m/s')
            Plotobj3.ax5 = plt.subplot(3, 2, 4)
            Plotobj3.ax5.set_title('Right_Speed_VBY')
            Plotobj3.PlotData(Plotobj3.ax5, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.INSSpeed["RightVelocity"], 'INS')
            Plotobj3.ShowPlotFormat('', 'unit:m/s')
            Plotobj3.ax6 = plt.subplot(3, 2, 6)
            Plotobj3.ax6.set_title('Downward_Speed_VBZ')
            Plotobj3.PlotData(Plotobj3.ax6, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.INSSpeed["DownwardVelocity"], 'INS')
            Plotobj3.ShowPlotFormat('', 'unit:m/s')

        def plot_position(name='Pos'):
            Plotobj4 = PlotGpsInsData()
            Plotobj4.fig.suptitle(name)
            Plotobj4.ax1 = plt.subplot(3, 1, 1)
            Plotobj4.ax1.set_title('Latitude')
            Plotobj4.PlotData(Plotobj4.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lat_Y"][1],
                              'GPS')
            Plotobj4.PlotData(Plotobj4.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lat_Y"][0],
                              'INS')
            Plotobj4.PlotData(Plotobj4.ax1, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.Pos["Lat_Y"][1] - DataStatiObj.Pos["Lat_Y"][0], 'GPS-INS')
            Plotobj4.ShowPlotFormat('', 'unit:meter')
            Plotobj4.ax2 = plt.subplot(3, 1, 2)
            Plotobj4.ax2.set_title('Longitude')
            Plotobj4.PlotData(Plotobj4.ax2, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lon_X"][1],
                              'GPS')
            Plotobj4.PlotData(Plotobj4.ax2, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Lon_X"][0],
                              'INS')
            Plotobj4.PlotData(Plotobj4.ax2, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.Pos["Lon_X"][1] - DataStatiObj.Pos["Lon_X"][0], 'GPS-INS')
            Plotobj4.ShowPlotFormat('', 'unit:meter')
            Plotobj4.ax3 = plt.subplot(3, 1, 3)
            Plotobj4.ax3.set_title('Altitude')
            Plotobj4.PlotData(Plotobj4.ax3, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Height_Z"][1],
                              'GPS')
            Plotobj4.PlotData(Plotobj4.ax3, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["Height_Z"][0],
                              'INS')
            Plotobj4.PlotData(Plotobj4.ax3, InsGpsSyncData['time'] - self.time0_set,
                              DataStatiObj.Pos["Height_Z"][1] - DataStatiObj.Pos["Height_Z"][0], 'GPS-INS')
            Plotobj4.ShowPlotFormat('', 'unit:meter')

        def plot_pos_error(name='Pos Error'):
            Plotobj5 = PlotGpsInsData()
            Plotobj5.fig.suptitle(name)
            Plotobj5.ax1 = plt.subplot(111)
            Plotobj5.ax1.set_title('GPS-INS (under C coordinate system)')
            Plotobj5.PlotData(Plotobj5.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["PosXError"],
                              '横向偏差gcx')
            Plotobj5.PlotData(Plotobj5.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["PosYError"],
                              '纵向偏差gcy')
            Plotobj5.PlotData(Plotobj5.ax1, InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["PosXYError"],
                              '水平偏差')
            Plotobj5.ShowPlotFormat('', 'unit:meter')
            Plotobj5.ax1.set_ylim(-5, 5)
            Plotobj5.ax2 = Plotobj5.ax1.twinx()  # 双坐标轴，右轴为GPS解状态
            Plotobj5.ax2.plot(InsGpsSyncData['time'] - self.time0_set, DataStatiObj.Pos["GpsFlags"],
                              label='1: 单点\n2: 差分\n4: 固定\n5: 浮点', color='r', linestyle="", marker='*')
            Plotobj5.ax2.set_ylim(0.5, 5.5)
            Plotobj5.ax2.yaxis.set_major_locator(MultipleLocator(1))

            plt.legend(loc='lower right')

        def plot_mileage(name='Mileage'):
            Plotobj6 = PlotGpsInsData()
            Plotobj6.fig.suptitle(name)
            Plotobj6.ax1 = plt.subplot(111)
            Plotobj6.ax1.set_title('里程')
            Plotobj6.PlotData(Plotobj6.ax1, InsGpsSyncData['time'][1:] - self.time0_set,
                              DataStatiObj.mileage['ForwardVelocity'], '里程(前)')
            Plotobj6.PlotData(Plotobj6.ax1, InsGpsSyncData['time'][1:] - self.time0_set,
                              DataStatiObj.mileage['RightVelocity'], '里程(右)')
            Plotobj6.PlotData(Plotobj6.ax1, InsGpsSyncData['time'][1:] - self.time0_set,
                              DataStatiObj.mileage["DownwardVelocity"],
                              '里程(下)')
            Plotobj6.ShowPlotFormat('', 'unit:meter')

        def plot_path(name='Car Path'):
            Plotobj7 = PlotGpsInsData()
            Plotobj7.fig.suptitle(name)
            Plotobj7.ax1 = plt.subplot(111)
            Plotobj7.ax1.set_title('行车轨迹')
            labels = list(InsGpsSyncData['time'] - self.time0_set)
            Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["Lon_X"][1], DataStatiObj.Pos["Lat_Y"][1], None, None,
                                'silver', '-', '')
            Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["Lon_X"][0], DataStatiObj.Pos["Lat_Y"][0], None, None,
                                'b',
                                '-', '')
            Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["Lon_X"][0], DataStatiObj.Pos["Lat_Y"][0], 'INS', labels,
                                'b', '', '.')
            Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsSigle"][1], DataStatiObj.Pos["GpsSigle"][0],
                                'Gps_Sigle',
                                labels, 'lightcoral', '', '.')
            Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsDiff"][1], DataStatiObj.Pos["GpsDiff"][0],
                                'Gps_Diff',
                                labels, 'deepskyblue', '', '.')
            Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsFloat"][1], DataStatiObj.Pos["GpsFloat"][0],
                                'Gps_Float',
                                labels, 'orange', '', '.')
            Plotobj7.PlotDataXY(Plotobj7.ax1, DataStatiObj.Pos["GpsFix"][1], DataStatiObj.Pos["GpsFix"][0], 'Gps_Fix',
                                labels, 'limegreen', '', '.')
            Plotobj7.ShowPlotFormat('经度 unit:meter', '纬度 unit:meter')

        def plot_zero_error(name='Error零偏'):
            Plotobj8 = PlotGpsInsData()
            Plotobj8.fig.suptitle(name)
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

        def plot_kalman_p(name='Kalman.P'):
            Plotobj9 = PlotGpsInsData()
            Plotobj9.fig.suptitle(name)
            Plotobj9.ax1 = plt.subplot(5, 2, 1)
            Plotobj9.ax1.set_title('Pp')
            Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set, np.array(PData['Pp'])[:, 0], 'posLat')
            Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set, np.array(PData['Pp'])[:, 1], 'posLon')
            Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set, np.array(PData['Pp'])[:, 2], 'posH')
            Plotobj9.PlotData(Plotobj9.ax1, PData['PpT'] - self.time0_set,
                              np.array(PData['Pp'])[:, 0] + np.array(PData['Pp'])[:, 1],
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
            Plotobj9.PlotData(Plotobj9.ax6, PData['PalignT'] - self.time0_set, np.array(PData['Palign'])[:, 0],
                              'PCtbRollError')
            Plotobj9.PlotData(Plotobj9.ax6, PData['PalignT'] - self.time0_set, np.array(PData['Palign'])[:, 1],
                              'PCtbPitchError')
            Plotobj9.PlotData(Plotobj9.ax6, PData['PalignT'] - self.time0_set, np.array(PData['Palign'])[:, 2],
                              'PCtbYawError')
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
            Plotobj9.PlotData(Plotobj9.ax10, PData['PAtttgT'] - self.time0_set, np.array(PData['PAtttg'])[:, 0],
                              'RollError')
            Plotobj9.PlotData(Plotobj9.ax10, PData['PAtttgT'] - self.time0_set, np.array(PData['PAtttg'])[:, 1],
                              'PitchError')
            Plotobj9.PlotData(Plotobj9.ax10, PData['PAtttgT'] - self.time0_set, np.array(PData['PAtttg'])[:, 2],
                              'YawError')
            Plotobj9.ShowPlotFormat('', 'unit:deg', 0.8)

        def plot_kalman_x(name='Kalman.X'):
            Plotobj10 = PlotGpsInsData()
            Plotobj10.fig.suptitle(name)
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
            Plotobj10.PlotData(Plotobj10.ax6, PData['XalignT'] - self.time0_set, np.array(PData['Xalign'])[:, 0],
                               'XalignX')
            Plotobj10.PlotData(Plotobj10.ax6, PData['XalignT'] - self.time0_set, np.array(PData['Xalign'])[:, 1],
                               'XalignY')
            Plotobj10.PlotData(Plotobj10.ax6, PData['XalignT'] - self.time0_set, np.array(PData['Xalign'])[:, 2],
                               'XalignZ')
            Plotobj10.ShowPlotFormat('', 'unit:deg', 0.8)
            Plotobj10.ax7 = plt.subplot(5, 2, 7)
            Plotobj10.ax7.set_title('Xlbbg')
            Plotobj10.PlotData(Plotobj10.ax7, PData['XlbbgT'] - self.time0_set, np.array(PData['Xlbbg'])[:, 0],
                               'XlbbgX')
            Plotobj10.PlotData(Plotobj10.ax7, PData['XlbbgT'] - self.time0_set, np.array(PData['Xlbbg'])[:, 1],
                               'XlbbgY')
            Plotobj10.PlotData(Plotobj10.ax7, PData['XlbbgT'] - self.time0_set, np.array(PData['Xlbbg'])[:, 2],
                               'XlbbgZ')
            Plotobj10.ShowPlotFormat('', 'unit:m', 0.8)
            Plotobj10.ax8 = plt.subplot(5, 2, 8)
            Plotobj10.ax8.set_title('Xlbbc')
            Plotobj10.PlotData(Plotobj10.ax8, PData['XlbbcT'] - self.time0_set, np.array(PData['Xlbbc'])[:, 0],
                               'XlttcX')
            Plotobj10.PlotData(Plotobj10.ax8, PData['XlbbcT'] - self.time0_set, np.array(PData['Xlbbc'])[:, 1],
                               'XlttcY')
            Plotobj10.PlotData(Plotobj10.ax8, PData['XlbbcT'] - self.time0_set, np.array(PData['Xlbbc'])[:, 2],
                               'XlttcZ')
            Plotobj10.ShowPlotFormat('', 'unit:m', 0.8)
            Plotobj10.ax9 = plt.subplot(5, 2, 9)
            Plotobj10.ax9.set_title('Xkws')
            Plotobj10.PlotData(Plotobj10.ax9, PData['XkwsT'] - self.time0_set, np.array(PData['Xkws'])[:, 0], 'left')
            Plotobj10.PlotData(Plotobj10.ax9, PData['XkwsT'] - self.time0_set, np.array(PData['Xkws'])[:, 1], 'right')
            Plotobj10.ShowPlotFormat('', 'unit:m/puls', 0.8)
            Plotobj10.ax10 = plt.subplot(5, 2, 10)
            Plotobj10.ax10.set_title('XAtttg')
            Plotobj10.PlotData(Plotobj10.ax10, PData['XAtttgT'] - self.time0_set, np.array(PData['XAtttg'])[:, 0],
                               'RollError')
            Plotobj10.PlotData(Plotobj10.ax10, PData['XAtttgT'] - self.time0_set, np.array(PData['XAtttg'])[:, 1],
                               'PitchError')
            Plotobj10.PlotData(Plotobj10.ax10, PData['XAtttgT'] - self.time0_set, np.array(PData['XAtttg'])[:, 2],
                               'YawError')
            Plotobj10.ShowPlotFormat('', 'unit:m', 0.8)

        bpos = self.bpos_gpsins
        InsData = self.InsDataDF
        PData = self.PDataDict
        SyncData = self.SyncDataDF
        VehicleData = self.VehicleDataDF
        GpsData = self.GpsDataDF
        ImuData = self.ImuDataDF
        InsGpsSyncData = self.SyncInsGpsData
        pos0 = np.array([InsGpsSyncData['lat'][1], InsGpsSyncData['lon'][1], InsGpsSyncData['height'][1]])  # 轨迹初始点计算
        DataStatiObj = DataStatistics()  # 统计计算
        DataStatiObj.SpeedCalculation(InsGpsSyncData, 1)  # 计算北东地和前右下速度
        DataStatiObj.ForwardSpeedCalculation(PData, VehicleData)  # 四轮轮速计算
        DataStatiObj.PosErrorCalculation(InsGpsSyncData, bpos, pos0, 1)  # 位置坐标转换
        DataStatiObj.Gpsflagstransfer(InsGpsSyncData)  # GPS解状态转换

        self.checkTimeType1()

        # figure1: DataDiff
        plot_data_diff()
        # figure2: Att
        plot_attitude()
        # figure3: Velocity
        plot_velocity()
        # figure4: Pos
        plot_position()
        # figure5: Pos Error
        plot_pos_error()
        # figure6: Mileage
        plot_mileage()
        # figure7: Car Path
        plot_path()
        # figure8: Error零偏
        plot_zero_error()
        # figure9: Kalman.P
        plot_kalman_p()
        # figure10: Kalman.X
        plot_kalman_x()

        plt.show()  # 显示所有图像

    # INS数据与参考对比画图
    def PlotRefGpsInsSyncData(self, filePath, ref_type='100C'):
        """

        @author: wenzixuan liqianwen
        :param filePath: saving path

        注意：有时plot函数不要包裹太多层
        """

        def plot_add_nonius(line, labels=None, show_formate='xy', hover=False, multiple=True):
            """添加游标"""
            cursor = mplcursors.cursor(line, hover=hover, multiple=multiple)
            if 'time' == show_formate:
                if labels:
                    cursor.connect("add", lambda sel: sel.annotation.set_text(
                        'time:{}s'.format(labels[math.floor(sel.target.index)])))
            elif 'xy' == show_formate:
                cursor.connect("add", lambda sel: sel.annotation.set_text(
                    'x:{}\ny:{}'.format(sel.target[0], sel.target[1])))
            else:
                cursor.connect("add", lambda sel: sel.annotation.set_text(sel.target))

            return cursor

        def plot_time_diff(ref_ins_data, ref_gps_data, name='时间同步精度'):
            fig = plt.figure(figsize=(8, 6))
            ax0 = fig.add_subplot(111)
            for f_name in ref_ins_data.keys():
                ax0.plot(ref_ins_data[f_name]['time_x'] - self.time0_set,
                         ref_ins_data[f_name]['time_x'] - ref_ins_data[f_name]['time_y'],
                         label=str(f_name + '_Ref-Ins'),
                         linewidth=1, alpha=0.7)

                if self.gps_flag[f_name]:
                    ax0.plot(ref_gps_data[f_name]['time'] - self.time0_set,
                             ref_gps_data[f_name]['time'] - ref_gps_data[f_name]['itow_pos'],
                             label=str(f_name + '_Ref-Gps'), linewidth=1, alpha=0.7)

            ax0 = self.set_ax(ax0, '', 'unit:s', name)
            cursor_0 = Cursor(ax0, horizOn=False, vertOn=True, useblit=False, color='grey', linewidth=1, linestyle='--')
            return cursor_0

        def plot_path(RefGpsData, RefInsData, ref_ins_data_before, ref_gps_data_before, name='全局路径'):
            cursor_1 = []
            fig = plt.figure(figsize=(8, 6))
            ax1 = fig.add_subplot(111)
            index = 0
            for f_name in self.gps_flag.keys():
                # 参考和INS
                if 0 == index:
                    ax1.plot(RefInsData[f_name].Pos["Lon_X"][0].tolist(),
                             RefInsData[f_name].Pos["Lat_Y"][0].tolist()
                             , label='Ref' + self.ref_type, linestyle='-', marker='', color=self.color_list[index], alpha=0.7)
                    line1 = ax1.plot(RefInsData[f_name].Pos["Lon_X"][0].tolist(),
                                     RefInsData[f_name].Pos["Lat_Y"][0].tolist()
                                     , linestyle='', marker='.',
                                     color=self.color_list[index], alpha=0.7)
                    labels1 = list(ref_ins_data_before[f_name]['time_x'] - self.time0_set)
                    cursor_1.append(plot_add_nonius(line1, labels=labels1, show_formate='time'))

                ax1.plot(RefInsData[f_name].Pos["Lon_X"][1].tolist(),
                         RefInsData[f_name].Pos["Lat_Y"][1].tolist()
                         , label=str(f_name + '_Ins'), linestyle='-', marker='', color=self.color_list[index + 1], alpha=0.7)
                line2 = ax1.plot(RefInsData[f_name].Pos["Lon_X"][1].tolist(),
                                 RefInsData[f_name].Pos["Lat_Y"][1].tolist()
                                 , linestyle='', marker='.',
                                 color=self.color_list[index + 1], alpha=0.7)
                labels2 = list(ref_ins_data_before[f_name]['time_y'] - self.time0_set)
                cursor_1.append(plot_add_nonius(line2, labels=labels2, show_formate='time'))

                # GPS
                if self.gps_flag[f_name]:
                    labels3 = list(ref_gps_data_before[f_name]['time'])
                    ax1.plot(RefGpsData[f_name].Pos["Lon_X"][1], RefGpsData[f_name].Pos["Lat_Y"][1]
                             , linestyle='-', marker='', alpha=0.3)
                    line3 = ax1.plot(RefGpsData[f_name].Pos["Lon_X"][1], RefGpsData[f_name].Pos["Lat_Y"][1]
                                     , linestyle='', marker='.', alpha=0.7)
                    cursor_1.append(
                        plot_add_nonius(line3, labels=labels3, show_formate='time', hover=True, multiple=False))

                    ax1.plot(RefGpsData[f_name].Pos["GpsSigle"][1], RefGpsData[f_name].Pos["GpsSigle"][0]
                             , label=str(f_name + '_Gps_Sigle'), linestyle='', marker='.', color='lightcoral',
                             alpha=0.6)
                    ax1.plot(RefGpsData[f_name].Pos["GpsDiff"][1], RefGpsData[f_name].Pos["GpsDiff"][0]
                             , label=str(f_name + '_Gps_Diff'), linestyle='', marker='.', color='deepskyblue',
                             alpha=0.6)
                    ax1.plot(RefGpsData[f_name].Pos["GpsFloat"][1],
                             RefGpsData[f_name].Pos["GpsFloat"][0]
                             , label=str(f_name + '_Gps_Float'), linestyle='', marker='.', color='orange', alpha=0.6)
                    ax1.plot(RefGpsData[f_name].Pos["GpsFix"][1], RefGpsData[f_name].Pos["GpsFix"][0]
                             , label=str(f_name + '_Gps_Fix'), linestyle='', marker='.', color='limegreen', alpha=0.6)

                index += 1

            ax1 = self.set_ax(ax1, '经度 unit:meter', '纬度 unit:meter', name)
            ax1.axis('equal')

            # cursor_1 = Cursor(ax1, horizOn=False, vertOn=True, useblit=False, color='grey', linewidth=1, linestyle='--')
            return cursor_1

        def plot_position(ref_ins_data, ref_gps_data, ref_ins_data_before, ref_gps_data_before, name='位置'):
            cursor_2 = []
            fig = plt.figure()
            fig.suptitle(name)

            ax21 = fig.add_subplot(321)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax21.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].Pos["Lat_Y"][0],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                line = ax21.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].Pos["Lat_Y"][1],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax21.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                     ref_gps_data[f_name].Pos["Lat_Y"][1],
                                     label=str(f_name + '_GPS'), linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
            ax21 = self.set_ax(ax21, '', 'unit:m', '纬度')

            ax22 = fig.add_subplot(323)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax22.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].Pos["Lon_X"][0],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                line = ax22.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].Pos["Lon_X"][1],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax22.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                     ref_gps_data[f_name].Pos["Lon_X"][1],
                                     label=str(f_name + '_GPS'), linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
            ax22 = self.set_ax(ax22, '', 'unit:m', '经度')

            ax23 = fig.add_subplot(325)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax23.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].Pos["Height_Z"][0],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                line = ax23.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].Pos["Height_Z"][1],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax23.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                     ref_gps_data[f_name].Pos["Height_Z"][1],
                                     label=str(f_name + '_GPS'), linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
            ax23 = self.set_ax(ax23, '', 'unit:m', '高度')

            ax24 = fig.add_subplot(322)
            for f_name in ref_ins_data.keys():
                line = ax24.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].Pos["Lon_X"][0] - ref_ins_data[f_name].Pos["Lon_X"][1],
                                 label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax24.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                     ref_gps_data[f_name].Pos["Lon_X"][0] - ref_gps_data[f_name].Pos["Lon_X"][1],
                                     label=str(f_name + '_Ref-GPS'), linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
            ax24 = self.set_ax(ax24, '', 'unit:m', '纬度对比')

            ax25 = fig.add_subplot(324)
            for f_name in ref_ins_data.keys():
                line = ax25.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].Pos["Lat_Y"][0] - ref_ins_data[f_name].Pos["Lat_Y"][1],
                                 label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax25.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                     ref_gps_data[f_name].Pos["Lat_Y"][0] - ref_gps_data[f_name].Pos["Lat_Y"][1],
                                     label=str(f_name + '_Ref-GPS'), linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
            ax25 = self.set_ax(ax25, '', 'unit:m', '经度对比')

            ax26 = fig.add_subplot(326)
            for f_name in ref_ins_data.keys():
                line = ax26.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].Pos["Height_Z"][0] - ref_ins_data[f_name].Pos["Height_Z"][1],
                                 label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_2.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax26.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                     ref_gps_data[f_name].Pos["Height_Z"][0] - ref_gps_data[f_name].Pos["Height_Z"][1],
                                     label=str(f_name + '_Ref-GPS'), linewidth=1, alpha=0.7)
                    cursor_2.append(plot_add_nonius(line, show_formate='xy'))
            ax26 = self.set_ax(ax26, '', 'unit:m', '高度对比')

            fig.subplots_adjust(hspace=0.7)
            fig.canvas.manager.window.showMaximized()
            return cursor_2

        def plot_pos_compare(ref_ins_data, ref_gps_data, ref_ins_data_before, ref_gps_data_before,
                             name='横纵偏差(载体坐标系下)'):
            cursor_3 = []
            fig = plt.figure()
            fig.suptitle(name)

            ax311 = fig.add_subplot(211)
            for f_name in ref_ins_data.keys():
                line = ax311.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                  ref_ins_data[f_name].Pos["PosXError"],
                                  label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_3.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax311.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                      ref_gps_data[f_name].Pos["PosXError"],
                                      label=str(f_name + '_Ref-Gps'), linewidth=1, alpha=0.7)
                    cursor_3.append(plot_add_nonius(line, show_formate='xy'))
            ax311 = self.set_ax(ax311, '', 'unit:m', '横向偏差')
            # 限制纵坐标
            if plt.ylim()[0] < -5 or plt.ylim()[1] > 2:
                ax311.set_ylim(-5, 5)

            ax312 = ax311.twinx()
            for f_name in ref_ins_data.keys():
                if self.gps_flag[f_name]:
                    index = list(ref_ins_data.keys()).index(f_name)
                    line = ax312.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                      ref_gps_data[f_name].Pos["GpsFlags"],
                                      label=f_name + '\n1: 单点\n2: 差分\n4: 固定\n5: 浮点',
                                      color=self.color_list[index],
                                      linestyle="", marker='*')
                    cursor_3.append(plot_add_nonius(line, show_formate='xy'))
            ax312.set_ylim(0, 6)
            ax312.yaxis.set_major_locator(MultipleLocator(1))
            ax312.legend(loc='lower right')

            ########
            ax321 = fig.add_subplot(212)
            for f_name in ref_ins_data.keys():
                line = ax321.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                  ref_ins_data[f_name].Pos["PosYError"],
                                  label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_3.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax321.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                      ref_gps_data[f_name].Pos["PosYError"],
                                      label=str(f_name + '_Ref-Gps'), linewidth=1, alpha=0.7)
                    cursor_3.append(plot_add_nonius(line, show_formate='xy'))
            ax321 = self.set_ax(ax321, '', 'unit:m', '纵向偏差')
            # 限制纵坐标
            if plt.ylim()[0] < -5 or plt.ylim()[1] > 2:
                ax321.set_ylim(-5, 5)

            ax322 = ax321.twinx()
            for f_name in ref_ins_data.keys():
                if self.gps_flag[f_name]:
                    index = list(ref_ins_data.keys()).index(f_name)
                    line = ax322.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                      ref_gps_data[f_name].Pos["GpsFlags"],
                                      label=f_name + '\n1: 单点\n2: 差分\n4: 固定\n5: 浮点',
                                      color=self.color_list[index],
                                      linestyle="", marker='*')
                    cursor_3.append(plot_add_nonius(line, show_formate='xy'))
            ax322.set_ylim(0, 6)
            ax322.yaxis.set_major_locator(MultipleLocator(1))
            ax322.legend(loc='lower right')

            fig.subplots_adjust(hspace=0.7)
            fig.canvas.manager.window.showMaximized()
            return cursor_3

        def plot_pos_error(ref_ins_data, ref_gps_data, ref_ins_data_before, ref_gps_data_before,
                           name='水平偏差(载体坐标系下)'):
            cursor_4 = []
            fig = plt.figure()
            fig.suptitle(name)
            ax4 = fig.add_subplot(111)
            for f_name in ref_ins_data.keys():
                line = ax4.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                ref_ins_data[f_name].Pos["PosXYError"],
                                label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_4.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    line = ax4.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                    ref_gps_data[f_name].Pos["PosXYError"],
                                    label=str(f_name + '_Ref-Gps'), linewidth=1, alpha=0.7)
                    cursor_4.append(plot_add_nonius(line, show_formate='xy'))
            ax4 = self.set_ax(ax4, '', 'unit:m', '')
            # 限制纵坐标
            if plt.ylim()[1] > 10:
                ax4.set_ylim(0, 10)

            ax42 = ax4.twinx()
            for f_name in ref_ins_data.keys():
                if self.gps_flag[f_name]:
                    index = list(ref_ins_data.keys()).index(f_name)
                    line = ax42.plot(ref_gps_data_before[f_name]['time'] - self.time0_set,
                                     ref_gps_data[f_name].Pos["GpsFlags"]
                                     , label=f_name + '\n1: 单点\n2: 差分\n4: 固定\n5: 浮点',
                                     color=self.color_list[index],
                                     linestyle="", marker='*')
                    cursor_4.append(plot_add_nonius(line, show_formate='xy'))
            ax42.set_ylim(0, 6)
            ax42.yaxis.set_major_locator(MultipleLocator(1))
            ax42.legend(loc='lower right')
            fig.canvas.manager.window.showMaximized()
            return cursor_4

        def plot_attitude(ref_ins_data, ref_ins_data_before, name='姿态'):
            cursor_5 = []
            fig = plt.figure()
            fig.suptitle(name)

            ax51 = fig.add_subplot(321)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax51.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data_before[f_name]["roll_x"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_5.append(plot_add_nonius(line, show_formate='xy'))
                line = ax51.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data_before[f_name]["roll_y"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_5.append(plot_add_nonius(line, show_formate='xy'))
            ax51 = self.set_ax(ax51, '', 'unit:deg', 'Roll')

            ax52 = fig.add_subplot(322)
            for f_name in ref_ins_data.keys():
                line = ax52.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["roll"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_5.append(plot_add_nonius(line, show_formate='xy'))
            ax52 = self.set_ax(ax52, '', 'unit:deg', 'Roll对比')

            ax53 = fig.add_subplot(323)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax53.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data_before[f_name]["pitch_x"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_5.append(plot_add_nonius(line, show_formate='xy'))
                line = ax53.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data_before[f_name]["pitch_y"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_5.append(plot_add_nonius(line, show_formate='xy'))
            ax53 = self.set_ax(ax53, '', 'unit:deg', 'Pitch')

            ax54 = fig.add_subplot(324)
            for f_name in ref_ins_data.keys():
                line = ax54.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["pitch"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_5.append(plot_add_nonius(line, show_formate='xy'))
            ax54 = self.set_ax(ax54, '', 'unit:deg', 'Pitch对比')

            ax55 = fig.add_subplot(325)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax55.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name]["yaw_x"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_5.append(plot_add_nonius(line, show_formate='xy'))
                line = ax55.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name]["yaw_y"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_5.append(plot_add_nonius(line, show_formate='xy'))
            ax55 = self.set_ax(ax55, '', 'unit:deg', 'yaw')

            ax56 = fig.add_subplot(326)
            for f_name in ref_ins_data.keys():
                line = ax56.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["yaw"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_5.append(plot_add_nonius(line, show_formate='xy'))
            ax56 = self.set_ax(ax56, '', 'unit:deg', 'yaw对比')

            fig.subplots_adjust(hspace=0.7)
            fig.canvas.manager.window.showMaximized()
            return cursor_5

        def plot_ned_speed(ref_ins_data, ref_ins_data_before, name='速度（导航坐标系）'):
            cursor_6 = []
            fig = plt.figure()
            fig.suptitle(name)

            ax61 = fig.add_subplot(321)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax61.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].REFSpeed["NorthVelocity"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_6.append(plot_add_nonius(line, show_formate='xy'))
                line = ax61.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].INSSpeed["NorthVelocity"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_6.append(plot_add_nonius(line, show_formate='xy'))
            ax61 = self.set_ax(ax61, '', 'unit:m/s', '北向速度')

            ax62 = fig.add_subplot(322)
            for f_name in ref_ins_data.keys():
                line = ax62.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["vel_n"],
                                 label=str(f_name + '_Ref-INS'), linewidth=1, alpha=0.7)
                cursor_6.append(plot_add_nonius(line, show_formate='xy'))
            ax62 = self.set_ax(ax62, '', 'unit:m/s', '北向速度对比')

            ax63 = fig.add_subplot(323)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax63.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].REFSpeed["EastVelocity"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_6.append(plot_add_nonius(line, show_formate='xy'))
                line = ax63.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].INSSpeed["EastVelocity"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_6.append(plot_add_nonius(line, show_formate='xy'))
            ax63 = self.set_ax(ax63, '', 'unit:m/s', '东向速度')

            ax64 = fig.add_subplot(324)
            for f_name in ref_ins_data.keys():
                line = ax64.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["vel_e"],
                                 label=str(f_name + '_Ref-INS'), linewidth=1, alpha=0.7)
                cursor_6.append(plot_add_nonius(line, show_formate='xy'))
            ax64 = self.set_ax(ax64, '', 'unit:m/s', '东向速度对比')

            ax65 = fig.add_subplot(325)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax65.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].REFSpeed["GroundVelocity"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_6.append(plot_add_nonius(line, show_formate='xy'))
                line = ax65.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].INSSpeed["GroundVelocity"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_6.append(plot_add_nonius(line, show_formate='xy'))
            ax65 = self.set_ax(ax65, '', 'unit:m/s', '地向速度')

            ax66 = fig.add_subplot(326)
            for f_name in ref_ins_data.keys():
                line = ax66.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["vel_g"],
                                 label=str(f_name + '_Ref-INS'), linewidth=1, alpha=0.7)
                cursor_6.append(plot_add_nonius(line, show_formate='xy'))
            ax66 = self.set_ax(ax66, '', 'unit:m/s', '地向速度对比')

            fig.subplots_adjust(hspace=0.7)
            fig.canvas.manager.window.showMaximized()
            return cursor_6

        def plot_xyz_speed(ref_ins_data, ref_ins_data_before, name='速度（载体坐标系）'):
            cursor_7 = []
            fig = plt.figure()
            fig.suptitle(name)

            ax71 = fig.add_subplot(321)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax71.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].INSSpeed["ForwardVelocity"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_7.append(plot_add_nonius(line, show_formate='xy'))
                line = ax71.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].INSSpeed["ForwardVelocity"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_7.append(plot_add_nonius(line, show_formate='xy'))
            ax71 = self.set_ax(ax71, '', 'unit:m/s', '前向速度')

            ax72 = fig.add_subplot(322)
            for f_name in ref_ins_data.keys():
                line = ax72.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["vel_x"],
                                 label=str(f_name + '_Ref-INS'), linewidth=1, alpha=0.7)
                cursor_7.append(plot_add_nonius(line, show_formate='xy'))
            ax72 = self.set_ax(ax72, '', 'unit:m/s', '前向速度对比')

            ax73 = fig.add_subplot(323)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax73.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].REFSpeed["RightVelocity"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_7.append(plot_add_nonius(line, show_formate='xy'))
                line = ax73.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].INSSpeed["RightVelocity"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_7.append(plot_add_nonius(line, show_formate='xy'))
            ax73 = self.set_ax(ax73, '', 'unit:m/s', '右向速度')

            ax74 = fig.add_subplot(324)
            for f_name in ref_ins_data.keys():
                line = ax74.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["vel_y"],
                                 label=str(f_name + '_Ref-INS'), linewidth=1, alpha=0.7)
                cursor_7.append(plot_add_nonius(line, show_formate='xy'))
            ax74 = self.set_ax(ax74, '', 'unit:m/s', '右向速度对比')

            ax75 = fig.add_subplot(325)
            for f_name in ref_ins_data.keys():
                index = list(ref_ins_data.keys()).index(f_name)
                if 0 == index:  # only plant the first Ref100C
                    line = ax75.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                     ref_ins_data[f_name].REFSpeed["DownwardVelocity"],
                                     label='Ref' + self.ref_type, linewidth=1, alpha=0.7)
                    cursor_7.append(plot_add_nonius(line, show_formate='xy'))
                line = ax75.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].INSSpeed["DownwardVelocity"],
                                 label=str(f_name + '_INS'), linewidth=1, alpha=0.7)
                cursor_7.append(plot_add_nonius(line, show_formate='xy'))
            ax75 = self.set_ax(ax75, '', 'unit:m/s', '下向速度')

            ax76 = fig.add_subplot(326)
            for f_name in ref_ins_data.keys():
                line = ax76.plot(ref_ins_data_before[f_name]['time_x'] - self.time0_set,
                                 ref_ins_data[f_name].error["vel_z"],
                                 label=str(f_name + '_Ref-INS'), linewidth=1, alpha=0.7)
                cursor_7.append(plot_add_nonius(line, show_formate='xy'))
            ax76 = self.set_ax(ax76, '', 'unit:m/s', '下向速度对比')

            fig.subplots_adjust(hspace=0.7)
            fig.canvas.manager.window.showMaximized()
            return cursor_7

        def plot_precision_statistics(ref_ins_data, ref_gps_data, name='精度统计'):
            cursor_8 = []
            fig = plt.figure()
            fig.suptitle(name)

            ax81 = fig.add_subplot(311)
            for f_name in ref_ins_data.keys():
                bin_edges, cdf_error = DataCDF(ref_ins_data[f_name].Pos["PosXYError"])
                line = ax81.plot(bin_edges, cdf_error, label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_8.append(plot_add_nonius(line, show_formate='xy'))
                if self.gps_flag[f_name]:
                    bin_edges, cdf_error = DataCDF(ref_gps_data[f_name].Pos["PosXYError"])
                    line = ax81.plot(bin_edges, cdf_error, label=str(f_name + '_Ref-Gps'), linewidth=1, alpha=0.7)
                    cursor_8.append(plot_add_nonius(line, show_formate='xy'))
            ax81 = self.set_ax(ax81, 'unit:m', '占比：%', '水平偏差统计', x_lim=[0, 0.5], y_lim=[0, 1])

            ########
            ax82 = fig.add_subplot(312)
            for f_name in ref_ins_data.keys():
                bin_edges, cdf_error = DataCDF(abs(ref_ins_data[f_name].error["yaw"]))
                line = ax82.plot(bin_edges, cdf_error, label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_8.append(plot_add_nonius(line, show_formate='xy'))
            ax82 = self.set_ax(ax82, '', 'unit:m', '航向偏差统计', x_lim=[0, 0.5], y_lim=[0, 1])

            ########
            ax83 = fig.add_subplot(313)
            for f_name in ref_ins_data.keys():
                bin_edges, cdf_error = DataCDF(ref_ins_data[f_name].error["vel"])
                line = ax83.plot(bin_edges, cdf_error, label=str(f_name + '_Ref-Ins'), linewidth=1, alpha=0.7)
                cursor_8.append(plot_add_nonius(line, show_formate='xy'))
            ax83 = self.set_ax(ax83, '', 'unit:m', '速度偏差统计', x_lim=[0, 0.5], y_lim=[0, 1])

            fig.subplots_adjust(hspace=0.5)
            fig.canvas.manager.window.showMaximized()
            return cursor_8

        def plot_gps_situation(statistic_gps_all, name='GPS解状态占比'):
            # 画GPS解状态
            flags = ['固定', '浮点', '差分', '单点', 'None']
            gps_filename = []

            # if
            if 0 == len(self.gps_flag.keys()):
                return print('No required GPS data!')

            for f_name, value in self.gps_flag.items():
                if value:
                    gps_filename.append(f_name)
            plt.suptitle(name)

            if sum(list(self.gps_flag.values())) in [0, 1]:
                file_name = gps_filename[0] if len(gps_filename) == 1 else list(self.gps_flag.keys())[0]
                sub = plt.subplot(1, 1, 1)
                sub.set_title(file_name)
                items = []
                percent = []
                for flag in flags:
                    if statistic_gps_all[file_name][flag][0] != 0:
                        items.append(flag)
                for item in items:
                    percent.append(statistic_gps_all[file_name][item][0])
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
                        if statistic_gps_all[file_name][flag][0] != 0:
                            items.append(flag)
                    for item in items:
                        percent.append(statistic_gps_all[file_name][item][0])

                    sub.pie(percent, labels=items, autopct='%1.2f%%')

            # fig.canvas.manager.window.showMaximized()

        def gen_statistics_xlsx():
            savedata = []
            statistic_gps_all = {}
            statistics_ins, statistics_gps = [], []

            for file_name in self.gps_flag.keys():
                # INS数据与参考对比统计
                statistics_ins = self.RefInsData[file_name].StatisticSyncData(self.SyncRefInsData[file_name],
                                                                              file_name + "_Ins")
                # GPS解状态统计
                if self.gps_flag[file_name]:
                    statistics_gps = self.RefGpsData[file_name].StatisticGpsFlag(self.SyncRefGpsData[file_name],
                                                                                 file_name + "_gps")
                    statistic_gps_all[file_name] = statistics_gps

                if not savedata:
                    savedata = [statistics_ins, statistics_gps]
                else:
                    for item in statistics_ins:
                        savedata[0][item].append(statistics_ins[item][0])
                    for item in statistics_gps:
                        savedata[1][item].append(statistics_gps[item][0])
            SaveStatisticToExcel(savedata, filePath)

            return statistic_gps_all

        if 0 == len(self.SyncRefInsData.keys()):
            return print('no file required to show !!!')

        self.ref_type = ref_type
        ########################### 预处理 ###########################
        self.iniInsGpsBpos()  # 杆臂值初始化
        self.checkGpsFlag()  # Gps绘图数初始化
        self.dataPreStatistics()  # 统计指标数据预处理
        self.checkTimeType2()  # 显示时间类型

        ########################## 生成统计表格 ##########################
        statistics_gps_all = gen_statistics_xlsx()

        ########################## 开始画图 ##########################
        cursors = []
        try:
            plot_gps_situation(statistics_gps_all)
        except Exception as e:
            print('Cannot plot gps解状态')
            print(e)
        try:
            cursors.append(plot_time_diff(self.SyncRefInsData, self.SyncRefGpsData))
        except Exception as e:
            print('Cannot plot time_diff')
            print(e)
        try:
            cursors.append(plot_path(self.RefGpsData, self.RefInsData, self.SyncRefInsData, self.SyncRefGpsData))
        except Exception as e:
            print('Cannot plot path')
            print(e)
        try:
            cursors.append(plot_position(self.RefInsData, self.RefGpsData, self.SyncRefInsData, self.SyncRefGpsData))
        except Exception as e:
            print('Cannot plot position')
            print(e)
        try:
            cursors.append(plot_pos_compare(self.RefInsData, self.RefGpsData, self.SyncRefInsData, self.SyncRefGpsData))
        except Exception as e:
            print('Cannot plot pos_compare')
            print(e)
        try:
            cursors.append(plot_pos_error(self.RefInsData, self.RefGpsData, self.SyncRefInsData, self.SyncRefGpsData))
        except Exception as e:
            print('Cannot plot pos_error')
            print(e)

        if self.ref_type == '100C':
            try:
                cursors.append(plot_attitude(self.RefInsData, self.SyncRefInsData))
            except Exception as e:
                print('Cannot plot attitude')
                print(e)
            try:
                cursors.append(plot_ned_speed(self.RefInsData, self.SyncRefInsData))
            except Exception as e:
                print('Cannot plot ned_speed')
                print(e)
            try:
                cursors.append(plot_xyz_speed(self.RefInsData, self.SyncRefInsData))
            except Exception as e:
                print('Cannot plot xyz_speed')
                print(e)
        try:
            cursors.append(plot_precision_statistics(self.RefInsData, self.RefGpsData))
        except Exception as e:
            print('Cannot plot precision_statistics')
            print(e)

        # cursors.append(plot_precision_statistics(self.RefInsData, self.RefGpsData))

        plt.show()  # 显示所有图像
        return

    @staticmethod
    def set_ax(ax, x_label='', y_label='', name='', x_lim=None, y_lim=None):
        ax.autoscale(enable=True, axis='y')
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.grid()
        ax.legend(loc='best')
        ax.set_title(name)

        if x_lim:
            try:
                ax.set_xlim(x_lim[0], x_lim[1])
            except Exception as e:
                print('Cannot set x_lim')
        if y_lim:
            try:
                ax.set_ylim(y_lim[0], y_lim[1])
            except Exception as e:
                print('Cannot set y_lim')

        return ax

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
        data1 = self.SyncRefInsData[list(self.SyncRefInsData.keys())[0]]
        pos0 = np.array([data1['lat_x'][1], data1['lon_x'][1], data1['height_x'][1]])  # 统一计算初始点
        for file_name in list(self.SyncRefInsData.keys()):
            try:
                SyncRefInsData = self.SyncRefInsData[file_name]
                self.RefInsData[file_name] = DataStatistics()  # INS和参考数据统计计算
                self.RefInsData[file_name].SpeedCalculation(SyncRefInsData, 2)  # 计算北东地和前右下速度
                self.RefInsData[file_name].PosErrorCalculation(SyncRefInsData, self.bpos_refins[file_name], pos0, 2)
                # 原画图内生成部分
                self.RefInsData[file_name].error["yaw"] = angele_standardization(
                    np.array(self.SyncRefInsData[file_name]["yaw_x"] - self.SyncRefInsData[file_name]["yaw_y"]))
                self.RefInsData[file_name].error["pitch"] = self.SyncRefInsData[file_name]["pitch_x"] - \
                                                            self.SyncRefInsData[file_name]["pitch_y"]
                self.RefInsData[file_name].error["roll"] = self.SyncRefInsData[file_name]["roll_x"] - \
                                                           self.SyncRefInsData[file_name]["roll_y"]
                self.RefInsData[file_name].error["vel_n"] = self.RefInsData[file_name].REFSpeed["NorthVelocity"] - \
                                                            self.RefInsData[file_name].INSSpeed["NorthVelocity"]
                self.RefInsData[file_name].error["vel_e"] = self.RefInsData[file_name].REFSpeed["EastVelocity"] - \
                                                            self.RefInsData[file_name].INSSpeed["EastVelocity"]
                self.RefInsData[file_name].error["vel_g"] = self.RefInsData[file_name].REFSpeed["GroundVelocity"] - \
                                                            self.RefInsData[file_name].INSSpeed["GroundVelocity"]
                self.RefInsData[file_name].error["vel_x"] = self.RefInsData[file_name].REFSpeed["ForwardVelocity"] - \
                                                            self.RefInsData[file_name].INSSpeed["ForwardVelocity"]
                self.RefInsData[file_name].error["vel_y"] = self.RefInsData[file_name].REFSpeed["RightVelocity"] - \
                                                            self.RefInsData[file_name].INSSpeed["RightVelocity"]
                self.RefInsData[file_name].error["vel_z"] = self.RefInsData[file_name].REFSpeed["DownwardVelocity"] - \
                                                            self.RefInsData[file_name].INSSpeed["DownwardVelocity"]
                self.RefInsData[file_name].error["vel"] = np.sqrt(
                    pow(self.RefInsData[file_name].error["vel_x"], 2) + pow(self.RefInsData[file_name].error["vel_y"],
                                                                            2))
                if self.gps_flag[file_name]:
                    SyncRefGpsData = self.SyncRefGpsData[file_name]
                    self.RefGpsData[file_name] = DataStatistics()  # GPSs和参考数据统计计算
                    self.RefGpsData[file_name].PosErrorCalculation(SyncRefGpsData, self.bpos_refgps[file_name], pos0, 1)
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

    # 用于与自身对比
    def checkTimeType1(self):
        if not self.second_of_week:
            self.time0_set = self.InsDataDF['time'][0]
        else:
            self.time0_set = self.InsDataDF['time'][0] - self.InsDataDF['time'][0]

    # 显示时间的方式(用于与基准数据对比)，second_of_week为显示True周内秒， 为显示Fasle第一组数据的同步的第一个时间
    def checkTimeType2(self):
        if not self.second_of_week:
            self.time0_set = self.SyncRefInsData[list(self.SyncRefInsData.keys())[0]]['time_x'][0]
        else:
            self.time0_set = self.SyncRefInsData[list(self.SyncRefInsData.keys())[0]]['time_x'][0] - self.SyncRefInsData[list(self.SyncRefInsData.keys())[0]]['time_x'][0]


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
