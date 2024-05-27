from Utils.dataMatPlot import PlotData
from Standardize.dataPreProcess import scenesDifferByTime
from Utils import  insStatistic
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from Utils.dataStatistics import DataStatistics

plt.rc("font", family='MicroSoft YaHei', weight='bold')


class insDataPlot:
    def __init__(self):
        self.plot_config = {'time_type': 'gps', 'time': ['0', '0'],
                            'ins_plot_time_type': '计时时间（秒）', 'gnss_plot_time_type': 'utc时间',
                            'ins_plot_freq': 1,
                            'ins_plot_flags': {4: '固定解 Fix', 5: '浮点解 Float', 2: '差分解 Diff', 1: '单点解 Single', 0: '其他 Else'},
                            }
        self.ins_test_data = []
        self.english_map = {'INS综合误差历元分布图':'INS Comprehensive Error Epoch Distribution'
            , 'GPS解状态占比饼状图':'GPS Solution Status Percentage'
            , '历元间隔分布图':'Epoch Interval Distribution'
            , '相邻IMU-GPS时间间隔图':'Epoch Interval Distribution of Neighboring IMU & GPS'
            , '相邻GPS-IMU时间间隔图':'Epoch Interval Distribution of Neighboring GPS & IMU'
            , '加表陀螺历元间隔分布图':'Epoch Interval Distribution of Acc & Gyro'
            , '速度误差(北东地)历元分布图':'Epoch Distribution of Velocity Error (North, East, Down)'
            , '速度误差(前右下)历元分布图':'Epoch Distribution of Velocity Error (Front, Right, Down)'
            , '里程(前右下)历元分布图':'Epoch Distribution of Mileage (Front, Right, Down)'
            , '坐标误差(经纬高)历元分布图':'Epoch Distribution of Coordinate (Longitude, Latitude, Height)'
            , '位置偏差(水平、横向、纵向)历元分布图':'Epoch Distribution of Positional Deviation (Horizontal, Lateral, Longitudinal)'
            , '姿态误差(横滚、俯仰、航向)历元分布图':'Epoch Distribution of Attitude Error (Roll, Pitch, Heading)'
            , '统计误差分布图':'Statistical Error Distribution', '同步数据轨迹图':'Synchronize Trajectory'
            , 'Kalman.P历元分布图':'Epoch Distribution of Kalman.P'
            , 'Kalman.X历元分布图':'Epoch Distribution of Kalman.X'
            , '零偏历元分布图':'Epoch Distribution of Bias'
            , 'GPS偏差对比图':'GPS Error Comparison'
            , '加表(XYZ)历元分布图':'Epoch Distribution of Acceleration (XYZ)'
            , '陀螺(XYZ)历元分布图':'Epoch Distribution of Gyro (XYZ)'
            , '轮速历元分布图':'Epoch Distribution of Wheel Speed', '速度轮速对比历元分布图':'Epoch Distribution of Wheel Speed and Velocity'

            , '北向速度':'North Velocity', '北向速度对比':'North Velocity Comparison', '东向速度':'East Velocity', '东向速度对比':'East Velocity Comparison', '地向速度':'Down Velocity', '地向速度对比':'Down Velocity Comparison'
            , '前向速度':'Front Velocity', '前向速度对比':'Front Velocity Comparison', '右向速度':'Right Velocity', '右向速度对比':'Right Velocity Comparison', '下向速度':'Down Velocity', '下向速度对比':'Down Velocity Comparison'
            , '前向里程': 'Front Mileage', '右向里程': 'Right Mileage', '下向里程': 'Down Mileage'
            , '纬度':'Latitude', '纬度对比':'Latitude Comparison', '经度':'Longitude', '经度对比':'Longitude Comparison', '高度':'Height', '高度对比':'Height Comparison'
            , '经度 实际与估计偏差对比':'Longitude Error Comparison', '纬度 实际与估计偏差对比':'Latitude Error Comparison', '高度 实际与估计偏差对比':'Height Error Comparison'
            , '位置水平误差':'Horizontal Position Error', '位置纵向误差':'Longitudinal Position Error', '位置横向误差':'Lateral Position Error'
            , 'Roll对比':'Roll Comparison', 'Pitch对比':'Pitch Comparison', 'Yaw对比':'Yaw / Heading Comparison'
            , '航向偏差统计':'Yaw / Heading Comparison', '水平速度偏差统计':'Velocity Comparison'
            , '加表x轴': 'Acc X', '加表y轴': 'Acc Y', '加表z轴': 'Acc Z'
            , '陀螺x轴': 'Gyro X', '陀螺y轴': 'Gyro Y', '陀螺z轴': 'Gyro Z'
            , '左前轮轮速': 'Wheel Speed (Front Left)', '右前轮轮速': 'Wheel Speed (Front Right)'
            , '左后轮轮速': 'Wheel Speed (Back Left)', '右后轮轮速': 'Wheel Speed (Back Right)'
            , }
        self.color_list = ["royalblue", "lime", "coral", "c", "violet", "grey", "chocolate"]

    def set_polt_data(self, test_data, df_name='sync_df', time_type=None):
        time_type = time_type if time_type else self.plot_config['ins_plot_time_type']

        time_cut = [
            {'time_type': self.plot_config['time_type'], 'time': self.plot_config['time'], 'id': "", 'name': ""}]
        if df_name not in test_data.keys() or len(test_data[df_name]) == 0:
            return pd.DataFrame(), np.array([])
        # 限制画图范围
        if self.plot_config['time'] != ['0', '0'] and type(test_data[df_name]) == pd.DataFrame:
            cut_data = scenesDifferByTime(test_data[df_name], time_cut, test_data["file_name"])
            plot_data0 = cut_data[0]['data'].copy()
        else:
            plot_data0 = test_data[df_name].copy()

        # 抽稀倍数
        try:
            plot_data = plot_data0[::self.plot_config['ins_plot_freq']]
            plot_data = plot_data.reset_index(drop=True)
        except Exception as e:
            print('画图抽稀失败，失败文件类型为：', type(plot_data0), '\n原因为：', e)
            plot_data = plot_data0

        # posture data补偿杆臂
        try:
            if 'bpox' in test_data.keys():
                pass
            if 'posture_bpox' in test_data.keys():
                if df_name in ['sync_df', 'sync_gps_ins']:
                    for val_name in ['roll', 'pitch', 'heading']:
                        val_index = ['roll', 'pitch', 'heading'].index(val_name)
                        if val_name in plot_data.keys():
                            plot_data[val_name] -= test_data['posture_bpox'][val_index]
                        else:
                            continue
            if 'posture_bpox_gps' in test_data.keys():
                if df_name == 'sync_df_gps':
                    for val_name in ['roll', 'pitch', 'heading']:
                        val_index = ['roll', 'pitch', 'heading'].index(val_name)
                        if val_name in plot_data.keys():
                            plot_data[val_name] -= test_data['posture_bpox_gps'][val_index]
                        else:
                            continue
                if df_name == 'sync_gps_ins':
                    for val_name in ['roll', 'pitch', 'heading']:
                        val_index = ['roll', 'pitch', 'heading'].index(val_name)
                        if val_name+'_x' in plot_data.keys():
                            plot_data[val_name+'_x'] -= test_data['posture_bpox_gps'][val_index]
                        else:
                            continue
        except Exception as e:
            print('画图姿态杆臂补偿失败，失败数据为：', val_name, '\n原因为：', e)

        # 画图时间轴类型
        if time_type == 'utc时间':
            plot_time = pd.to_datetime(plot_data['utcDate'].astype('str') + ' ' + plot_data['utcTime'].astype('str'))
        elif time_type == '计时时间（秒）':
            plot_time = plot_data['gpsItow'] - plot_data['gpsItow'][0]
        elif 'gpsItow' in plot_data.keys():  # GPS周内秒
            plot_time = plot_data['gpsItow']
        return plot_data, plot_time

    @staticmethod
    def DataCDF(Data):
        hist, bin_edges = np.histogram(Data, bins=len(Data))
        cdf = np.cumsum(hist / sum(hist))
        return bin_edges[1:], cdf

    def plot_gps_quality(self, title="GPS解状态占比饼状图"):
        try:
            plotObj = PlotData()
            add_name = '\n'+self.english_map[title] if title in self.english_map.keys() else ''
            plotObj.fig.suptitle(title+add_name)
            num = len(self.ins_test_data)
            for i in range(num):
                plotObj.ax = plt.subplot(1, num, i + 1)
                plotObj.ax.set_title(self.ins_test_data[i]['file_name'])
                if 'df_gps' in self.ins_test_data[i]:
                    plot_data, plot_time = self.set_polt_data(self.ins_test_data[i], df_name='df_gps')
                    if 'gpsQuality' in plot_data:
                        gpsQuality = {}
                        length = len(plot_data['gpsQuality'])
                        count = plot_data['gpsQuality'].value_counts()
                        # 计算解状态的百分比
                        for k, v in self.plot_config['ins_plot_flags'].items():
                            try:
                                gpsQuality[v] = count[k] / length
                            except KeyError:
                                gpsQuality[v] = 0
                        plt.pie(np.array(list(gpsQuality.values())), labels=list(gpsQuality.keys()), autopct='%.2f%%')  # labels--设置饼图标签 autopct--格式化输出百分比
                # elif 'sync_gps_ins' in self.ins_test_data[i]:
                #     plot_data, plot_time = self.set_polt_data(self.ins_test_data[i], df_name='sync_gps_ins')
                #     if 'flags_pos' in plot_data:
                #         gpsQuality = {}
                #         length = len(plot_data['flags_pos'])
                #         count = plot_data['flags_pos'].value_counts()
                #         if 0 not in count.index:
                #             count.loc[0] = 0
                #             for i in count.index:
                #                 if i not in [0,1,2,3,4,5]:
                #                     count[0] += count[i]
                #         # 计算解状态的百分比
                #         for k, v in self.plot_config['ins_plot_flags'].items():
                #             try:
                #                 gpsQuality[v] = count[k] / length
                #             except KeyError:
                #                 gpsQuality[v] = 0
                #         plt.pie(np.array(list(gpsQuality.values())), labels=list(gpsQuality.keys()), autopct='%.2f%%')
            plotObj.ShowPlotFormat('', 'unit: s')
            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + r"\n"

    def plot_time_gap(self, plotObj, ax, title="历元间隔分布图", title_on=True):

        name_map = {'历元间隔分布图': {'df':'unixTime','df_gps':'unixTime'},
                                '加表陀螺历元间隔分布图': {'df_vehicle':'gearTime','df_imu':'time'},}

        subtitle = title
        if title_on:
            add_name = '\n'+self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
            ax.set_title(subtitle+add_name)

        info = ''

        try:
            for test_data in self.ins_test_data:
                for df_name in list(name_map[subtitle].keys()):  # 不同数据集
                    index = list(name_map[subtitle].keys()).index(df_name)  # 默认一个数据只需要
                    df_type = df_name.split('_')[-1] if df_name.split('_')[-1] != 'df' else 'ins'
                    if df_name in test_data:
                        if name_map[subtitle][df_name] in test_data[df_name].keys():
                            if name_map[subtitle][df_name] == 'unixTime':
                                plot_data, plot_time = self.set_polt_data(test_data, df_name=df_name)
                            else:
                                plot_time = test_data[df_name][name_map[subtitle][df_name]]

                            time_differ = list(np.diff(plot_time))
                            plotObj.PlotPoint(ax, plot_time[1:], time_differ, test_data['file_name']+'_'+df_type, color=self.color_list[index], markersize=5)
                        else:
                            info += '数据 '+df_type+' 中不存在字段'+name_map[subtitle][df_name]+'; '
                    else:
                        info += '不存在数据 '+df_type+'; '

                # if 'df' in test_data.keys():
                #     plot_data, plot_time = self.set_polt_data(test_data, df_name='df')
                #     time_differ = list(np.diff(plot_data['unixTime']))
                #     plotObj.PlotData(ax, plot_time[1:], time_differ, test_data['file_name']+'_ins')
                # elif 'df_gps' in test_data.keys():
                #     plot_data, plot_time = self.set_polt_data(test_data, df_name='df_gps')
                #     time_differ = list(np.diff(plot_data['unixTime']))
                #     plotObj.PlotData(ax, plot_time[1:], time_differ, test_data['file_name']+'_gps')
                # else:
                #     return '既INS帧，又无GPS帧数据。无法绘制历元间隔分布图。'
            plotObj.ShowPlotFormat('', 'unit: s')
            return info
        except Exception as e:
            return title + "生成失败原因:" + str(e) + r"\n"

    def plot_traj(self, plotObj, ax, title="同步数据轨迹图"):
        try:
            ref_flag = True
            color_map = {'single':'lightcoral', 'pseduo':'deepskyblue', 'float':'orange', 'fixed':'limegreen'}

            for test_data in self.ins_test_data:
                if 'sync_df' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    time_labels = list(plot_data['unixTime'])
                    if 'n_axis' in plot_data.keys() and 'e_axis' in plot_data.keys():
                        plotObj.PlotData(ax, plot_data['e_axis'], plot_data['n_axis'], test_data['file_name']+'_ins', labels=time_labels, labels_format='time:{}s')
                    if ref_flag:
                        if 'n_axis_x' in plot_data.keys() and 'e_axis_x' in plot_data.keys():
                            plotObj.PlotData(ax, plot_data['e_axis_x'], plot_data['n_axis_x'], 'ref', labels=time_labels)
                        ref_flag = False
                if 'sync_df_gps' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_df_gps')
                    if 'n_axis' in plot_data.keys() and 'e_axis' in plot_data.keys():
                        # plotObj.PlotData(ax, plot_data['e_axis'], plot_data['n_axis'], test_data['file_name']+'_gps', labels=time_labels, labels_format='time:{}s')
                        plotObj.PlotPoint(ax, plot_data['e_axis'], plot_data['n_axis'], test_data['file_name']+'_gps', marker='.', color='grey', labels=time_labels, labels_format='time:{}s')
                        if 'flags_pos' in plot_data.keys():
                            coor_dict = insStatistic.gps_flags_transfer(plot_data, flags_pos_name='flags_pos', lat_name='n_axis', lon_name='e_axis')
                            for key, color in color_map.items():
                                plotObj.PlotPoint(ax, coor_dict[key][1], coor_dict[key][0], test_data['file_name'] + '_gps_' + key, marker='.', color=color)
                    if ref_flag:
                        if 'n_axis_x' in plot_data.keys() and 'e_axis_x' in plot_data.keys():
                            plotObj.PlotData(ax, plot_data['e_axis_x'], plot_data['n_axis_x'], 'ref', labels=time_labels, labels_format='time:{}s')
                        ref_flag = False
                elif 'sync_gps_ins' in test_data:                                                           # 没有ref 但是有ins帧和GPS帧
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_gps_ins')
                    if 'n_axis' in plot_data.keys() and 'e_axis' in plot_data.keys():
                        plotObj.PlotData(ax, plot_data['e_axis'], plot_data['n_axis'], test_data['file_name']+'_ins', labels=time_labels, labels_format='time:{}s')
                    if 'n_axis_x' in plot_data.keys() and 'e_axis_x' in plot_data.keys():
                        # plotObj.PlotData(ax, plot_data['e_axis_x'], plot_data['n_axis_x'], test_data['file_name']+'_gps', labels=time_labels, labels_format='time:{}s')
                        plotObj.PlotPoint(ax, plot_data['e_axis_x'], plot_data['n_axis_x'], test_data['file_name']+'_gps', marker='.', color='grey', labels=time_labels, labels_format='time:{}s')
                        if 'flags_pos' in plot_data.keys():
                            coor_dict = insStatistic.gps_flags_transfer(plot_data, flags_pos_name='flags_pos', lat_name='n_axis_x', lon_name='e_axis_x')
                            for key, color in color_map.items():
                                plotObj.PlotPoint(ax, coor_dict[key][1], coor_dict[key][0], test_data['file_name'] + '_gps_' + key, marker='.', color=color)

            plotObj.ShowPlotFormat('', 'unit: m')
            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + r"\n"

    def plot_val(self, plotObj, ax, title="速度"):
        try:
            sync_name_map = {
                        '北向速度': 'north_vel', '东向速度': 'east_vel', '地向速度': 'ground_vel'
                        # '北向速度': 'NorthVelocity', '东向速度': 'EastVelocity', '地向速度': 'GroundVelocity'
                        , '前向速度': 'forward_vel', '右向速度': 'right_vel', '下向速度': 'downward_vel', '水平速度': 'velocity'
                        , '经度': 'longitude', '纬度':'latitude', '高度':'ellHeight'
                        , 'Roll': 'roll', 'Pitch':'pitch', 'Yaw':'heading'
                        , '前向里程':'forward_mile', '右向里程':'right_mile', '下向里程':'downward_mile'
                        # , ''
            }
            imu_name_map = {
                '陀螺x轴':'GyroX', '陀螺y轴':'GyroY', '陀螺z轴':'GyroZ', '加表x轴':'AccX', '加表y轴':'AccY', '加表z轴':'AccZ',
            }
            vehicle_name_map = {'左前轮轮速':'WheelSpeedFrontLeft', '右前轮轮速':'WheelSpeedFrontRight'
                , '左后轮轮速':'WheelSpeedBackLeft', '右后轮轮速':'WheelSpeedBackRight',}
            # unit_map = {'速度':'unit: m/s', '位置':'unit: m', '经纬度':'unit: m', '其他':'unit: '}
            ref_flag = True

            subtitle = title
            add_name = '\n'+self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
            ax.set_title(subtitle+add_name)

            for test_data in self.ins_test_data:
                if subtitle in sync_name_map.keys():     # 同步数据
                    if 'sync_df' in test_data:
                        plot_data, plot_time = self.set_polt_data(test_data)
                        if sync_name_map[subtitle] in plot_data.keys():
                            plotObj.PlotData(ax, plot_time, plot_data[sync_name_map[subtitle]], test_data['file_name']+'_ins')
                        if ref_flag:
                            if sync_name_map[subtitle]+'_x' in plot_data.keys():
                                plotObj.PlotData(ax, plot_time, plot_data[sync_name_map[subtitle] + '_x'], 'ref')
                            ref_flag = False
                    if 'sync_df_gps' in test_data:
                        plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_df_gps')
                        if sync_name_map[subtitle] in plot_data.keys():
                            plotObj.PlotData(ax, plot_time, plot_data[sync_name_map[subtitle]], test_data['file_name']+'_gps')
                        if ref_flag:
                            if sync_name_map[subtitle]+'_x' in plot_data.keys():
                                plotObj.PlotData(ax, plot_time, plot_data[sync_name_map[subtitle] + '_x'], 'ref')
                            ref_flag = False
                    elif 'sync_gps_ins' in test_data:                                                           # 没有ref 但是有ins帧和GPS帧
                        plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_gps_ins')
                        if sync_name_map[subtitle] in plot_data.keys():
                            plotObj.PlotData(ax, plot_time, plot_data[sync_name_map[subtitle]], test_data['file_name']+'_ins')
                        if sync_name_map[subtitle]+'_x' in plot_data.keys():
                            plotObj.PlotData(ax, plot_time, plot_data[sync_name_map[subtitle] + '_x'], test_data['file_name']+'_gps')
                if subtitle in imu_name_map.keys():
                    if 'df_imu' in test_data:
                        if imu_name_map[subtitle] in test_data['df_imu'].keys():
                            plotObj.PlotData(ax, test_data['df_imu']['time'], test_data['df_imu'][imu_name_map[subtitle]], test_data['file_name']+'_imu')
                if subtitle in vehicle_name_map.keys():
                    if 'df_vehicle' in test_data:
                        if vehicle_name_map[subtitle] in test_data['df_vehicle'].keys():
                            plotObj.PlotData(ax, test_data['df_vehicle']['gearTime'], test_data['df_vehicle'][vehicle_name_map[subtitle]], test_data['file_name']+'_vehicle')

            if '速' in title:
                plotObj.ShowPlotFormat('', 'unit: m/s')
            elif '经度' in title or '纬度' in title:  # 经纬度
                plotObj.ShowPlotFormat('', 'unit: m')
            elif '度' in title or '里程' in title:
                plotObj.ShowPlotFormat('', 'unit: m')
            elif '陀螺' in title:
                plotObj.ShowPlotFormat('', 'unit: °/s')
            elif '加表' in title:
                plotObj.ShowPlotFormat('', 'unit: g')
            else:
                plotObj.ShowPlotFormat('','unit: °')

            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + "\n"

    def plot_val_compare(self, plotObj, ax, title="速度轮速对比历元分布图", title_on=True):
        try:
            name_map = {"速度轮速对比历元分布图":{'df_vehicle': {'左前轮轮速': 'WheelSpeedFrontLeft', '右前轮轮速': 'WheelSpeedFrontRight'
                                                                                                 , '左后轮轮速': 'WheelSpeedBackLeft', '右后轮轮速': 'WheelSpeedBackRight'},
                                                                              'sync_df': {'前向速度': 'forward_vel'},
                                                                              'sync_df_gps': {'前向速度': 'forward_vel'}, 'sync_gps_ins': {'前向速度': 'forward_vel'}}}
            ref_flag = []

            if title not in name_map.keys():
                return title + "生成失败原因: plot_val_compare函数内无该图设置" + "\n"

            if title_on:
                subtitle = title
                add_name = '\n'+self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                ax.set_title(subtitle+add_name)

            for test_data in self.ins_test_data:
                for df_name in name_map[title]:
                    if df_name not in test_data.keys():
                        continue
                    elif 'sync_' in df_name:  # ['sync_df', 'sync_df_gps', 'sync_gps_ins']
                        suffix = '_ins' if df_name in ['sync_df', 'sync_gps_ins'] else '_gps'
                        suffix_x = '_ref' if df_name in ['sync_df', 'sync_df_gps'] else '_gps'

                        plot_data, plot_time = self.set_polt_data(test_data, df_name=df_name, time_type='gps周内秒')
                        for val_name in name_map[title][df_name].keys():   # '前向速度'...
                            # test
                            if name_map[title][df_name][val_name] in plot_data.keys() and str(test_data['file_name'] + val_name + suffix) not in ref_flag:
                                plotObj.PlotData(ax, plot_time, plot_data[name_map[title][df_name][val_name]], test_data['file_name'] + val_name + suffix)
                                ref_flag.append(test_data['file_name'] + val_name + suffix)
                            # base
                            if name_map[title][df_name][val_name]+'_x' in plot_data.keys() and str(test_data['file_name'] + val_name + suffix_x) not in ref_flag:
                                plotObj.PlotData(ax, plot_time, plot_data[name_map[title][df_name][val_name]+'_x'], test_data['file_name'] + val_name + suffix_x)
                                ref_flag.append(test_data['file_name'] + val_name + suffix_x)
                    else:  # [df_imu, df_vehicle]
                        time_name = 'gearTime' if df_name == 'df_vehicle' else 'time'
                        for val_name in name_map[title][df_name].keys():   # '左前轮轮速'。。。
                            if name_map[title][df_name][val_name] in test_data[df_name].keys():
                                plotObj.PlotData(ax, test_data[df_name][time_name], test_data[df_name][name_map[title][df_name][val_name]]
                                                 , test_data['file_name'] + val_name + df_name[2:])

            if '速' in title:
                plotObj.ShowPlotFormat('', 'unit: m/s')
            elif '度' in title or '里程' in title:
                plotObj.ShowPlotFormat('', 'unit: m')
            elif '陀螺' in title:
                plotObj.ShowPlotFormat('', 'unit: °/s')
            elif '加表' in title:
                plotObj.ShowPlotFormat('', 'unit: m/s^2')
            else:
                plotObj.ShowPlotFormat('','unit: °')

            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + "\n"

    def plot_val_diff(self, plotObj, ax, title="速度对比", title_on=True):
        try:
            name_map = {'北向速度对比': 'north_vel', '东向速度对比': 'east_vel', '地向速度对比': 'ground_vel'
                        , '前向速度对比': 'forward_vel', '右向速度对比': 'right_vel', '下向速度对比': 'downward_vel', '水平速度对比': 'velocity'
                        , '经度对比': 'longitude', '纬度对比':'latitude', '高度对比':'ellHeight'
                        , '经度 实际与估计偏差对比': 'longitude', '纬度 实际与估计偏差对比':'latitude', '高度 实际与估计偏差对比':'ellHeight'
                        , '经度 实际与估计偏差对比std': 'LonStd', '纬度 实际与估计偏差对比std':'LatStd', '高度 实际与估计偏差对比std':'hMSLStd'
                        , 'Roll对比': 'roll', 'Pitch对比':'pitch', 'Yaw对比':'heading', '航向偏差统计':'heading', '水平速度偏差统计':'velocity'}
            imu_name_map = {'相邻IMU-GPS时间间隔图': ['time', 'gps_ts']}
            gps_name_map = {'相邻GPS-IMU时间间隔图': ['gpsItow', 'imu_ts']}

            subtitle = title
            if title_on:
                add_name = '\n'+self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                ax.set_title(subtitle+add_name)

            for test_data in self.ins_test_data:
                if title in name_map.keys():
                    if 'sync_df' in test_data:
                        plot_data, plot_time = self.set_polt_data(test_data)
                        if name_map[subtitle] in plot_data.keys() and name_map[subtitle]+'_x' in plot_data.keys():
                            if subtitle == '航向偏差统计':
                                bin_edges, cdf_error = self.DataCDF(abs(DataStatistics().angele_standardization(np.array(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))))
                                plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name']+'_ref-ins')
                            elif '偏差统计' in subtitle:
                                bin_edges, cdf_error = self.DataCDF(abs(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))
                                plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name']+'_ref-ins')
                            elif 'heading' in name_map[subtitle]:
                                plotObj.PlotData(ax
                                                 , plot_time, DataStatistics().angele_standardization(np.array(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))
                                                 , test_data['file_name']+'_ref-ins')
                            else:
                                plotObj.PlotData(ax
                                                 , plot_time, plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]
                                                 , test_data['file_name']+'_ref-ins')
                    if 'sync_df_gps' in test_data:
                        plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_df_gps')
                        if name_map[subtitle] in plot_data.keys() and name_map[subtitle]+'_x' in plot_data.keys():
                            if subtitle == '航向偏差统计':
                                bin_edges, cdf_error = self.DataCDF(abs(DataStatistics().angele_standardization(np.array(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))))
                                plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name']+'_ref-gps')
                            elif '偏差统计' in subtitle:
                                bin_edges, cdf_error = self.DataCDF(abs(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))
                                plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name']+'_ref-gps')
                            elif 'heading' in name_map[subtitle]:
                                plotObj.PlotData(ax
                                                 , plot_time, DataStatistics().angele_standardization(np.array(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))
                                                 , test_data['file_name']+'_ref-gps')
                            else:
                                plotObj.PlotData(ax
                                                 , plot_time, plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]
                                                 , test_data['file_name']+'_ref-gps')
                            if '实际与估计偏差对比std' in subtitle:
                                gps_data, gps_time = self.set_polt_data(test_data, df_name='df_gps')
                                plotObj.PlotData(ax, gps_time, gps_data[name_map[subtitle+'std']]
                                                  , test_data['file_name']+'_gps std')
                    if 'sync_gps_ins' in test_data:                                                           # 没有ref 但是有ins帧和GPS帧
                        plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_gps_ins')
                        if name_map[subtitle] in plot_data.keys() and name_map[subtitle]+'_x' in plot_data.keys():
                            if subtitle == '航向偏差统计':
                                bin_edges, cdf_error = self.DataCDF(abs(DataStatistics().angele_standardization(np.array(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))))
                                plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name'] + '_gps-ins')
                            elif '偏差统计' in subtitle:
                                bin_edges, cdf_error = self.DataCDF(abs(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))
                                plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name']+'_gps-ins')
                            elif 'heading' in name_map[subtitle]:
                                plotObj.PlotData(ax
                                                 , plot_time, DataStatistics().angele_standardization(np.array(plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]))
                                                 , test_data['file_name']+'_gps-ins')
                            else:
                                plotObj.PlotData(ax
                                                 , plot_time, plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]]
                                                 , test_data['file_name']+'_gps-ins')
                elif title in imu_name_map.keys():
                    if 'df_imu' in test_data:
                        plot_time = test_data['df_imu'][imu_name_map[title][0]]
                        plotObj.PlotPoint(ax
                                          , plot_time, test_data['df_imu'][imu_name_map[subtitle][0]] - test_data['df_imu'][imu_name_map[subtitle][1]]
                                          , test_data['file_name'] + '_gps-imu', marker='.', markersize=5)
                elif title in gps_name_map.keys():
                    if 'df_gps' in test_data:
                        plot_time = test_data['df_gps'][gps_name_map[title][0]]
                        plotObj.PlotPoint(ax
                                          , plot_time, test_data['df_gps'][gps_name_map[subtitle][1]] - test_data['df_gps'][gps_name_map[subtitle][0]]
                                          , test_data['file_name'] + '_imu-gps', labels='self', marker='.', markersize=5)

            if '速' in title:  # 速度
                plotObj.ShowPlotFormat('', 'unit: m/s')
            elif '高度' in title:  # 高度
                plotObj.ShowPlotFormat('', 'unit: m')
            elif '度' in title:  # 经纬度
                plotObj.ShowPlotFormat('', 'unit: °')
            elif '时间间隔图' in title:  # 经纬度
                plotObj.ShowPlotFormat('', 'unit: s')
            else:
                plotObj.ShowPlotFormat('','unit: °')
            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + r"\n"

    def plot_error(self, plotObj, ax, title="位置水平误差"):
        try:
            name_map = {'位置水平误差':'horizontal_error', '位置纵向误差':'longitudinal_error', '位置横向误差':'lateral_error'
                                    , '位置水平偏差统计':'horizontal_error'}  # , '水平速度偏差统计':'velocity_diff'

            subtitle = title
            add_name = '\n'+self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
            ax.set_title(subtitle+add_name)
            ax_side = None

            for test_data in self.ins_test_data:
                if 'sync_df' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    if name_map[subtitle] in plot_data.keys():
                        if '偏差统计' in subtitle:
                            bin_edges, cdf_error = self.DataCDF(abs(plot_data[name_map[subtitle]]))
                            plotObj.PlotData(ax, bin_edges, cdf_error , test_data['file_name']+'_ref-ins')
                        else:
                            plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_ref-ins')
                if 'sync_df_gps' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_df_gps')
                    if name_map[subtitle] in plot_data.keys():
                        if '偏差统计' in subtitle:
                            bin_edges, cdf_error = self.DataCDF(abs(plot_data[name_map[subtitle]]))
                            plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name']+'_ref-gps')
                        else:
                            plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_ref-gps')
                            if 'flags_pos' in plot_data.keys():
                                ax_side = ax.twinx()
                                plotObj.PlotPoint(ax_side, plot_time, plot_data['flags_pos'], test_data['file_name'])
                elif 'sync_gps_ins' in test_data:                                                           # 没有ref 但是有ins帧和GPS帧
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_gps_ins')
                    if name_map[subtitle] in plot_data.keys():
                        if '偏差统计' in subtitle:
                            bin_edges, cdf_error = self.DataCDF(abs(plot_data[name_map[subtitle]]))
                            plotObj.PlotData(ax, bin_edges, cdf_error, test_data['file_name']+'_gps-ins')
                        else:
                            plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_gps-ins')
                            if 'flags_pos' in plot_data.keys():
                                ax_side = ax.twinx()
                                flags_pos = plot_data['flags_pos'].copy()
                                flags_pos[np.where(flags_pos > 6)[0]] = 0
                                plotObj.PlotPoint(ax_side, plot_time, flags_pos, test_data['file_name'])

            # ax.legend(bbox_to_anchor=(-0.1, -0.32), loc=3)
            ax.legend(bbox_to_anchor=(-0.1, 1), loc=3)
            if ax_side:
                plotObj.ShowPlotFormat('', '                      1: 单点 Single\n                  2: 差分 Diff\n'
                           '                4: 固定 Fix\n                    5: 浮点 Float\n                  0:其他 Else'
                         , legend_loc=1, rotation=0)
                if '速' in title:  # 速度
                    ax.set_ylabel('unit: m/s')
                elif '航向' in title:  # 经纬度
                    ax.set_ylabel('unit: °')
                elif '位置' in title:  # 位置误差
                    ax.set_ylabel('unit: m')
                else:
                    ax.set_ylabel('unit: m')
                ax_side.yaxis.set_major_locator(plt.MultipleLocator(1))
                ax_side.set_ylim(0, 6, emit=False)
            elif '速' in title:  # 速度
                plotObj.ShowPlotFormat('', 'unit: m/s')
            elif '航向' in title:  # 经纬度
                plotObj.ShowPlotFormat('', 'unit: °')
            else:
                plotObj.ShowPlotFormat('','unit: m')

            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + "\n"

    def plot_pdata_val(self, plotObj, ax, title="Pp"):
        try:
            name_map = {
                #  卡拉曼P
             'Pp': ['PpT', 'Pp', ['posLat', 'posLon', 'posH'], 'unit:m'], 'Pv': ['PvT', 'Pv', ['vlat', 'vlon', 'vh'], 'unit:m/s']
            , 'Patt': ['PattT', 'Patt', ['roll', 'pitch', 'yaw'], 'unit:degree'], 'Pba': ['PbaT', 'Pba', ['aex', 'aey', 'aez'], 'unit:g']
            , 'Pbg': ['PbgT', 'Pbg', ['gex', 'gey', 'gez'], 'unit:deg/s'], 'Palign': ['PalignT', 'Palign', ['PCtbRollError', 'PCtbPitchError', 'PCtbYawError'], 'unit:degree']
            , 'Plbbg': ['PlbbgT', 'Plbbg', ['LttgX', 'LttgY', 'LttgZ'], 'unit:m'], 'Plbbc': ['PlbbcT', 'Plbbc', ['LttgX', 'LttgY', 'LttgZ'], 'unit:m']
            , 'Pkws': ['PkwsT', 'Pkws', ['', 'left', 'right'], 'unit:m/puls'], 'PAtttg': ['PAtttgT', 'PAtttg', ['RollError', 'PitchError', 'YawError'], 'unit:degree']
                # 卡拉曼X
            , 'Xp': ['XpT', 'Xp', ['XLat', 'XLon', 'XH'], 'unit:m'], 'Xv': ['XvT', 'Xv', ['Xvn', 'Xve', 'Xvd'], 'unit:m/s']
            , 'Xatt': ['XattT', 'Xatt', ['Xroll', 'Xpitch', 'Xyaw'], 'unit:degree'], 'Xba': ['XbaT', 'Xba', ['Xbax', 'Xbay', 'Xbaz'], 'unit:g']
            , 'Xbg': ['XbgT', 'Xbg', ['Xbgx', 'Xbgy', 'Xbgz'], 'unit:deg/s'], 'Xalign': ['XalignT', 'Xalign', ['XalignX', 'XalignY', 'XalignZ'], 'unit:deg']
            , 'Xlbbg': ['XlbbgT', 'Xlbbg', ['XlbbgX', 'XlbbgY', 'XlbbgZ'], 'unit:m'], 'Xlbbc': ['XlbbcT', 'Xlbbc', ['XlttcX', 'XlttcY', 'XlttcZ'], 'unit:m']
            , 'Xkws': ['XkwsT', 'Xkws', ['left', 'right', ''], 'unit:m/puls'], 'XAtttg': ['XAtttgT', 'XAtttg', ['RollError', 'PitchError', 'YawError'], 'unit:m']
                # 零偏
            , 'Ba': ['BaT', 'Ba', ['ax', 'ay', 'az'], 'unit:g'], 'Bg': ['BgT', 'Bg', ['gx', 'gy', 'gz'], 'unit:deg/s']
            , 'Atttb': ['AlignT', 'Align', ['roll', 'pitch', 'yaw'], 'unit:degree'], 'Lttg': ['LbbgT', 'Lbbg', ['x', 'y', 'z'], 'unit:m']
            , 'Lttc': ['LbbcT', 'Lbbc', ['x', 'y', 'z'], 'unit:m'], 'Kws': ['KwsT', 'Kws', ['left', 'right', ''], 'unit:m/puls']
            , 'Atttg': ['AtttgT', 'Atttg', ['roll', 'pitch', 'yaw'], 'unit:degree'], 'Ltgc': ['LbgcT', 'Lbgc',  ['x', 'y', 'z'], 'unit:m']
                        }

            ax.set_title(title)

            for test_data in self.ins_test_data:
                if 'df_pdata' in test_data:
                    plot_data = test_data['df_pdata']
                    if name_map[title][0] in plot_data.keys() and name_map[title][1] in plot_data.keys():
                        plotObj.PlotData(ax, plot_data[name_map[title][0]], np.array(plot_data[name_map[title][1]])[:,0], test_data['file_name']+'_'+name_map[title][2][0])
                        plotObj.PlotData(ax, plot_data[name_map[title][0]], np.array(plot_data[name_map[title][1]])[:,1], test_data['file_name']+'_'+name_map[title][2][1])
                        plotObj.PlotData(ax, plot_data[name_map[title][0]], np.array(plot_data[name_map[title][1]])[:,2], test_data['file_name']+'_'+name_map[title][2][2])
                else:
                    return "无PData数据"

            plotObj.ShowPlotFormat('', name_map[title][-1])

            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + "\n"

    # INS 单图绘制
    def ins_start_plot(self, plot_title):
        error_msg = ""
        subplot_num = 1
        if plot_title == "GPS解状态占比饼状图":
            error_msg += self.plot_gps_quality(plot_title+'\n'+self.english_map[plot_title])
        else:
            # 先获取误差数据
            plotObj = PlotData()
            title = '\n'+self.english_map[plot_title] if plot_title in self.english_map.keys() else ''
            plotObj.fig.suptitle(plot_title+title)
            if plot_title == "速度误差(北东地)历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(3, 2, 1)
                    error_msg += self.plot_val(plotObj, plotObj.ax1, title='北向速度')
                    plotObj.ax2 = plt.subplot(322, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax2, title='北向速度对比')
                    plotObj.ax3 = plt.subplot(323, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax3, title='东向速度')
                    plotObj.ax4 = plt.subplot(324, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax4, title='东向速度对比')
                    plotObj.ax5 = plt.subplot(325, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax5, title='地向速度')
                    plotObj.ax6 = plt.subplot(326, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax6, title='地向速度对比')
            elif plot_title == "速度误差(前右下)历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(3, 2, 1)
                    error_msg += self.plot_val(plotObj, plotObj.ax1, title='前向速度')
                    plotObj.ax2 = plt.subplot(322, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax2, title='前向速度对比')
                    plotObj.ax3 = plt.subplot(323, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax3, title='右向速度')
                    plotObj.ax4 = plt.subplot(324, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax4, title='右向速度对比')
                    plotObj.ax5 = plt.subplot(325, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax5, title='下向速度')
                    plotObj.ax6 = plt.subplot(326, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax6, title='下向速度对比')
            elif plot_title == "轮速历元分布图":
                if 'df_vehicle' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无车辆轮速数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(2, 2, 1)
                    error_msg += self.plot_val(plotObj, plotObj.ax1, title='左前轮轮速')
                    plotObj.ax2 = plt.subplot(222, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax2, title='右前轮轮速')
                    plotObj.ax3 = plt.subplot(223, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax3, title='左后轮轮速')
                    plotObj.ax4 = plt.subplot(224, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax4, title='右后轮轮速')
            elif plot_title == "速度轮速对比历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                elif 'df_vehicle' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无车辆轮速数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(1, 1, 1)
                    error_msg += self.plot_val_compare(plotObj, plotObj.ax1, title=plot_title, title_on=False)
            elif plot_title == "里程(前右下)历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(3, 1, 1)
                    error_msg += self.plot_val(plotObj, plotObj.ax1, title='前向里程')
                    plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax2, title='右向里程')
                    plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax3, title='下向里程')
            elif plot_title == "历元间隔分布图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_time_gap(plotObj, plotObj.ax, title=plot_title, title_on=False)
            elif plot_title == "加表陀螺历元间隔分布图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_time_gap(plotObj, plotObj.ax, title=plot_title, title_on=False)
            elif plot_title == "坐标误差(经纬高)历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(3, 2, 1)
                    error_msg += self.plot_val(plotObj, plotObj.ax1, title='经度')
                    plotObj.ax2 = plt.subplot(322, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax2, title='经度对比')
                    plotObj.ax3 = plt.subplot(323, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax3, title='纬度')
                    plotObj.ax4 = plt.subplot(324, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax4, title='纬度对比')
                    plotObj.ax5 = plt.subplot(325, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax5, title='高度')
                    plotObj.ax6 = plt.subplot(326, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax6, title='高度对比')
            elif plot_title == "位置偏差(水平、横向、纵向)历元分布图":
                for test_data in self.ins_test_data:
                    if 'sync_df' in test_data.keys():
                        if 'longitudinal_error' in test_data["sync_df"]:
                            subplot_num = 3
                            break
                    elif 'sync_gps_ins' in test_data.keys():
                        if 'longitudinal_error' in test_data["sync_gps_ins"]:
                            subplot_num = 3
                            break
                plotObj.ax1 = plt.subplot(subplot_num, 1, 1)
                error_msg += self.plot_error(plotObj, plotObj.ax1, title='位置水平误差')
                if subplot_num == 3:
                    plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                    error_msg += self.plot_error(plotObj, plotObj.ax2, title='位置纵向误差')
                    plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                    error_msg += self.plot_error(plotObj, plotObj.ax3, title='位置横向误差')
                # error_msg += self.plot_pos_error(plotObj, plotObj.ax1, None, None, plot_title)
                # if subplot_num == 3:
                #     plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                #     error_msg = error_msg + self.plot_pos_error(plotObj, None, plotObj.ax2, None, plot_title)
                #     plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                #     error_msg = error_msg + self.plot_pos_error(plotObj, None, None, plotObj.ax3, plot_title)
            elif plot_title == "姿态误差(横滚、俯仰、航向)历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(3, 2, 1)
                    error_msg += self.plot_val(plotObj, plotObj.ax1, title='Roll')
                    plotObj.ax2 = plt.subplot(322, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax2, title='Roll对比')
                    plotObj.ax3 = plt.subplot(323, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax3, title='Pitch')
                    plotObj.ax4 = plt.subplot(324, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax4, title='Pitch对比')
                    plotObj.ax5 = plt.subplot(325, sharex=plotObj.ax1)
                    error_msg += self.plot_val(plotObj, plotObj.ax5, title='Yaw')
                    plotObj.ax6 = plt.subplot(326, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax6, title='Yaw对比')
            elif plot_title == "GPS偏差对比图":
                plotObj.ax1 = plt.subplot(3, 1, 1)
                error_msg += self.plot_val_diff(plotObj, plotObj.ax1, title='经度 实际与估计偏差对比')
                plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                error_msg += self.plot_val_diff(plotObj, plotObj.ax2, title='纬度 实际与估计偏差对比')
                plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                error_msg += self.plot_val_diff(plotObj, plotObj.ax3, title='高度 实际与估计偏差对比')
            elif plot_title == "统计误差分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(3, 1, 1)
                    error_msg += self.plot_error(plotObj, plotObj.ax1, title='位置水平偏差统计')
                    plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax2, title='航向偏差统计')
                    plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax3, title='水平速度偏差统计')
            elif plot_title == "同步数据轨迹图":
                plotObj.ax1 = plt.subplot(1, 1, 1)
                error_msg += self.plot_traj(plotObj, plotObj.ax)
            elif plot_title == "Kalman.P历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(5, 2, 1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax1, title='Pp')
                    plotObj.ax2 = plt.subplot(522, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax2, title='Pv')
                    plotObj.ax3 = plt.subplot(523, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax3, title='Patt')
                    plotObj.ax4 = plt.subplot(524, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax4, title='Pba')
                    plotObj.ax5 = plt.subplot(525, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax5, title='Pbg')
                    plotObj.ax6 = plt.subplot(526, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax6, title='Palign')
                    plotObj.ax7 = plt.subplot(527, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax7, title='Plbbg')
                    plotObj.ax8 = plt.subplot(528, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax8, title='Plbbc')
                    plotObj.ax9 = plt.subplot(529, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax9, title='Pkws')
                    plotObj.ax10 = plt.subplot(5,2,10, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax10, title='PAtttg')
            elif plot_title == "Kalman.X历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(5, 2, 1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax1, title='Xp')
                    plotObj.ax2 = plt.subplot(522, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax2, title='Xv')
                    plotObj.ax3 = plt.subplot(523, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax3, title='Xatt')
                    plotObj.ax4 = plt.subplot(524, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax4, title='Xba')
                    plotObj.ax5 = plt.subplot(525, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax5, title='Xbg')
                    plotObj.ax6 = plt.subplot(526, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax6, title='Xalign')
                    plotObj.ax7 = plt.subplot(527, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax7, title='Xlbbg')
                    plotObj.ax8 = plt.subplot(528, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax8, title='Xlbbc')
                    plotObj.ax9 = plt.subplot(529, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax9, title='Xkws')
                    plotObj.ax10 = plt.subplot(5,2,10, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax10, title='XAtttg')
            elif plot_title == "零偏历元分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(4, 2, 1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax1, title='Ba')
                    plotObj.ax2 = plt.subplot(422, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax2, title='Bg')
                    plotObj.ax3 = plt.subplot(423, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax3, title='Atttb')
                    plotObj.ax4 = plt.subplot(424, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax4, title='Lttg')
                    plotObj.ax5 = plt.subplot(425, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax5, title='Lttc')
                    plotObj.ax6 = plt.subplot(426, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax6, title='Kws')
                    plotObj.ax7 = plt.subplot(427, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax7, title='Atttg')
                    plotObj.ax8 = plt.subplot(428, sharex=plotObj.ax1)
                    error_msg += self.plot_pdata_val(plotObj, plotObj.ax8, title='Ltgc')
            elif plot_title == "加表(XYZ)历元分布图":
                plotObj.ax1 = plt.subplot(3, 1, 1)
                error_msg += self.plot_val(plotObj, plotObj.ax1, title='加表x轴')
                plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                error_msg += self.plot_val(plotObj, plotObj.ax2, title='加表y轴')
                plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                error_msg += self.plot_val(plotObj, plotObj.ax3, title='加表z轴')
            elif plot_title == "陀螺(XYZ)历元分布图":
                plotObj.ax1 = plt.subplot(3, 1, 1)
                error_msg += self.plot_val(plotObj, plotObj.ax1, title='陀螺x轴')
                plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                error_msg += self.plot_val(plotObj, plotObj.ax2, title='陀螺y轴')
                plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                error_msg += self.plot_val(plotObj, plotObj.ax3, title='陀螺z轴')
            elif plot_title == "轮速历元分布图":
                plotObj.ax1 = plt.subplot(2, 2, 1)
                error_msg += self.plot_val(plotObj, plotObj.ax1, title='左前轮轮速')
                plotObj.ax2 = plt.subplot(222, sharex=plotObj.ax1)
                error_msg += self.plot_val(plotObj, plotObj.ax2, title='右前轮轮速')
                plotObj.ax3 = plt.subplot(223, sharex=plotObj.ax1)
                error_msg += self.plot_val(plotObj, plotObj.ax3, title='左后轮轮速')
                plotObj.ax4 = plt.subplot(224, sharex=plotObj.ax1)
                error_msg += self.plot_val(plotObj, plotObj.ax4, title='右后轮轮速')
            elif plot_title == "相邻IMU-GPS时间间隔图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_val_diff(plotObj, plotObj.ax, title=plot_title, title_on=False)
            elif plot_title == "相邻GPS-IMU时间间隔图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_val_diff(plotObj, plotObj.ax, title=plot_title, title_on=False)
            else:
                error_msg += "不可画图：" + plot_title
        plt.show()  # 显示图像

        if error_msg != "":
            return error_msg

    # INS综合误差历元分布图
    def ins_multi_plot(self, plot_list):
        error_msg = ""
        subplot_num = len(plot_list)
        plotObj = PlotData()
        plotObj.fig.suptitle("INS综合误差历元分布图"+'\n'+self.english_map['INS综合误差历元分布图'])

        time_diff_name = ["历元间隔分布图", '加表陀螺历元间隔分布图']
        pos_error_name = ['位置水平误差', '位置横向误差', '位置纵向误差', '水平速度偏差统计']
        vel_diff_pic_name = ['北向速度对比', '东向速度对比', '地向速度对比', '前向速度对比', '右向速度对比', '下向速度对比', '水平速度对比'
                                , '经度对比', '纬度对比', '高度对比', 'Roll对比', 'Pitch对比', 'Yaw对比', '航向偏差统计'
                                , '经度 实际与估计偏差对比', '纬度 实际与估计偏差对比', '高度 实际与估计偏差对比'
                                , '相邻GPS-IMU时间间隔图']
        vel_pic_name = ['北向速度', '东向速度', '地向速度', '前向速度', '右向速度', '下向速度', '水平速度'
                                , '经度', '纬度', '高度', 'Roll', 'Pitch', 'Yaw', '前向里程', '右向里程', '下向里程'
                                , '左前轮轮速', '右前轮轮速', '左后轮轮速', '右后轮轮速'
                                , '陀螺x轴', '陀螺y轴', '陀螺z轴', '加表x轴', '加表y轴', '加表z轴']
        bias_name = ['Pp', 'Pv', 'Patt', 'Pba', 'Pbg', 'Palign', 'Plbbg', 'Plbbc', 'Pkws', 'PAtttg'
            , 'Xp', 'Xv', 'Xatt', 'Xba', 'Xbg', 'Xalign', 'Xlbbg', 'Xlbbc', 'Xkws', 'XAtttg'
            , 'Ba', 'Bg', 'Atttb', 'Lttg', 'Lttc', 'Kws', 'Atttg', 'Ltgc']

        for i in range(len(plot_list)):
            add_name = '\n' + self.english_map[plot_list[i]] if plot_list[i] in self.english_map.keys() else ''
            plotObj.ax = plt.subplot(subplot_num * 100 + 1 * 10 + (i + 1), sharex=plotObj.ax)
            plotObj.ax.set_title(plot_list[i]+add_name)

            if plot_list[i] in time_diff_name:
                error_msg += self.plot_time_gap(plotObj, plotObj.ax,  title=plot_list[i])
            elif plot_list[i] in pos_error_name:
                error_msg += self.plot_error(plotObj, plotObj.ax, title=plot_list[i])
            elif plot_list[i] in bias_name:
                error_msg += self.plot_pdata_val(plotObj, plotObj.ax, title=plot_list[i])
            elif plot_list[i] in vel_diff_pic_name:
                error_msg += self.plot_val_diff(plotObj, plotObj.ax, title=plot_list[i])
            elif plot_list[i] in vel_pic_name:
                error_msg += self.plot_val(plotObj, plotObj.ax, title=plot_list[i])
            else:
                error_msg += "不可画图：" + plot_list[i]
        plt.show()  # 显示图像

        if error_msg == "":
            return '【绘图成功】'
        else:
            return '【绘图失败】' + error_msg


if __name__ == "__main__":

    # data = {'ref_ins': pd.read_feather(r'D:\Files\test\dbFiles\test1\test1_LogINS_ref同步ins.feather'),
    #         'ref_gps': pd.read_feather(r'D:\Files\test\dbFiles\test1\test1_LogINS_ref同步gps.feather'),
    #         'gps_ins': pd.read_feather(r'D:\Files\test\dbFiles\test1\test1_LogINS_gps同步ins.feather')}
    # test1 = np.load(r'D:\Files\pyFiles\asensingdataanalysistoolv3.0\test1_file.npy', allow_pickle=True).item()
    data = [np.load(r'D:\Files\test\dbFiles\test1\test1_file.npy', allow_pickle=True).item()]
    # data[0]['sync_df'] = pd.read_feather(r'D:\Files\test\dbFiles\test1\test1_ref同步ins.feather')
    # data[0]['sync_df_gps'] = pd.read_feather(r'D:\Files\test\dbFiles\test1\test1_ref同步ins.feather')
    # data[0]['sync_gps_ins'] = pd.read_feather(r'D:\Files\test\dbFiles\test1\test1_ref同步ins.feather')

    obj = insDataPlot()
    obj.plot_config = {'time_type': 'gps', 'time': ['0', '0'],
                       'ins_plot_time_type': '计时时间（秒）', 'gnss_plot_time_type': 'utc时间',
                       'ins_plot_flags': {4: '固定解 Fix', 5: '浮点解 Float', 2: '差分解 Diff', 1: '单点解 Single', 0: '其他 Else'},
                        'ins_plot_freq': 10,}
    obj.ins_test_data = data
    error_msg = obj.ins_start_plot('相邻IMU-GPS时间间隔图')  # 画统计图流程
    if error_msg:
        print("【绘图失败】" + error_msg)
    else:
        print('done')
