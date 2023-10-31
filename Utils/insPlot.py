from Utils.dataMatPlot import PlotData
from Standardize.dataPreProcess import scenesDifferByTime
from Utils import  insStatistic
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.rc("font", family='MicroSoft YaHei', weight='bold')


class insDataPlot:
    def __init__(self):
        self.plot_config = {'time_type': 'gps', 'time': ['0', '0'],
                            'ins_plot_time_type': '计时时间（秒）', 'gnss_plot_time_type': 'utc时间',
                            'ins_plot_freq': 10,
                            'ins_plot_flags': {4: '固定解', 5: '浮点解', 2: '差分解', 1: '单点解', 0: '其他'}}
        self.ins_test_data = []

    def set_polt_data(self, test_data, df_name='sync_df'):
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

        # 画图时间轴类型
        if self.plot_config['ins_plot_time_type'] == 'utc时间':
            plot_time = pd.to_datetime(
                plot_data['utcDate'].astype('str') + ' ' + plot_data['utcTime'].astype('str'))
        elif self.plot_config['ins_plot_time_type'] == '计时时间（秒）':
            plot_time = plot_data['gpsItow'] - plot_data['gpsItow'][0]
        else:
            plot_time = plot_data['gpsItow']
        return plot_data, plot_time

    def plot_gps_quality(self, title="GPS解状态占比饼状图"):
        try:
            plotObj = PlotData()
            plotObj.fig.suptitle(title)
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

    def plot_time_gap(self, plotObj, ax, title="历元间隔分布图"):
        try:
            for test_data in self.ins_test_data:
                if 'df' in test_data.keys():
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='df')
                    time_differ = list(np.diff(plot_data['unixTime']))
                    plotObj.PlotData(ax, plot_time[1:], time_differ, test_data['file_name']+'_ins')
                elif 'df_gps' in test_data.keys():
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='df_gps')
                    time_differ = list(np.diff(plot_data['unixTime']))
                    plotObj.PlotData(ax, plot_time[1:], time_differ, test_data['file_name']+'_gps')
                else:
                    return '既INS帧，又无GPS帧数据。无法绘制历元间隔分布图。'
            plotObj.ShowPlotFormat('', 'unit: s')
            return ""
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
            name_map = {
                        '北向速度': 'north_vel', '东向速度': 'east_vel', '地向速度': 'ground_vel'
                        # '北向速度': 'NorthVelocity', '东向速度': 'EastVelocity', '地向速度': 'GroundVelocity'
                        , '前向速度': 'forward_vel', '右向速度': 'right_vel', '下向速度': 'downward_vel', '速度': 'velocity'
                        , '经度': 'longitude', '纬度':'latitude', '高度':'ellHeight'
                        , 'Roll': 'roll', 'Pitch':'pitch', 'Yaw':'heading'
                        , '前向里程':'forward_mile', '右向里程':'right_mile', '下向里程':'downward_mile'}
            # unit_map = {'速度':'unit: m/s', '位置':'unit: m', '经纬度':'unit: m', '其他':'unit: '}
            ref_flag = True

            subtitle = title
            ax.set_title(subtitle)

            for test_data in self.ins_test_data:
                if 'sync_df' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    if name_map[subtitle] in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_ins')
                    if ref_flag:
                        if name_map[subtitle]+'_x' in plot_data.keys():
                            plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle] + '_x'], 'ref')
                        ref_flag = False
                if 'sync_df_gps' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_df_gps')
                    if name_map[subtitle] in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_gps')
                    if ref_flag:
                        if name_map[subtitle]+'_x' in plot_data.keys():
                            plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle] + '_x'], 'ref')
                        ref_flag = False
                elif 'sync_gps_ins' in test_data:                                                           # 没有ref 但是有ins帧和GPS帧
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_gps_ins')
                    if name_map[subtitle] in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_ins')
                    if name_map[subtitle]+'_x' in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle] + '_x'], test_data['file_name']+'_gps')

            if '速' in title:
                plotObj.ShowPlotFormat('', 'unit: m/s')
            elif '度' in title or '里程' in title:
                plotObj.ShowPlotFormat('', 'unit: m')
            else:
                plotObj.ShowPlotFormat('','unit: °')

            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + "\n"

    def plot_val_diff(self, plotObj, ax, title="速度对比"):
        try:
            name_map = {'北向速度对比': 'north_vel', '东向速度对比': 'east_vel', '地向速度对比': 'ground_vel'
                        , '前向速度对比': 'forward_vel', '右向速度对比': 'right_vel', '下向速度对比': 'downward_vel', '速度对比': 'velocity'
                        , '经度对比': 'longitude', '纬度对比':'latitude', '高度对比':'ellHeight'
                        , 'Roll对比': 'roll', 'Pitch对比':'pitch', 'Yaw对比':'heading', '航向偏差统计':'heading'}

            subtitle = title
            ax.set_title(subtitle)

            for test_data in self.ins_test_data:
                if 'sync_df' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    if name_map[subtitle] in plot_data.keys() and name_map[subtitle]+'_x' in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]], test_data['file_name']+'_ref-ins')
                if 'sync_df_gps' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_df_gps')
                    if name_map[subtitle] in plot_data.keys() and name_map[subtitle]+'_x' in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]], test_data['file_name']+'_ref-gps')
                if 'sync_gps_ins' in test_data:                                                           # 没有ref 但是有ins帧和GPS帧
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_gps_ins')
                    if name_map[subtitle] in plot_data.keys() and name_map[subtitle]+'_x' in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle] + '_x']-plot_data[name_map[subtitle]], test_data['file_name']+'_gps-ins')

            if '速' in title:  # 速度
                plotObj.ShowPlotFormat('', 'unit: m/s')
            elif '度' in title:  # 经纬度
                plotObj.ShowPlotFormat('', 'unit: m')
            else:
                plotObj.ShowPlotFormat('','unit: °')
            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + r"\n"

    def plot_error(self, plotObj, ax, title="位置水平误差"):
        try:
            name_map = {'位置水平误差':'horizontal_error', '位置纵向误差':'longitudinal_error', '位置横向误差':'lateral_error'
                                    , '速度偏差统计':'velocity'}

            subtitle = title
            ax.set_title(subtitle)
            ax_side = None

            for test_data in self.ins_test_data:
                if 'sync_df' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    if name_map[subtitle] in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_ref-ins')
                if 'sync_df_gps' in test_data:
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_df_gps')
                    if name_map[subtitle] in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_ref-gps')
                    if 'flags_pos' in plot_data.keys():
                        ax_side = ax.twinx()
                        plotObj.PlotPoint(ax_side, plot_time, plot_data['flags_pos'], test_data['file_name'])
                elif 'sync_gps_ins' in test_data:                                                           # 没有ref 但是有ins帧和GPS帧
                    plot_data, plot_time = self.set_polt_data(test_data, df_name='sync_gps_ins')
                    if name_map[subtitle] in plot_data.keys():
                        plotObj.PlotData(ax, plot_time, plot_data[name_map[subtitle]], test_data['file_name']+'_gps-ins')
                    if 'flags_pos' in plot_data.keys():
                        ax_side = ax.twinx()
                        flags_pos = plot_data['flags_pos'].copy()
                        flags_pos[np.where(flags_pos > 6)[0]] = 0
                        plotObj.PlotPoint(ax_side, plot_time, flags_pos, test_data['file_name'])

            # ax.legend(bbox_to_anchor=(-0.1, -0.32), loc=3)
            ax.legend(bbox_to_anchor=(-0.1, 1), loc=3)
            if ax_side:
                if '速' in title:  # 速度
                    ax.set_ylabel('unit: m/s')
                elif '航向' in title:  # 经纬度
                    ax.set_ylabel('unit: °')
                elif '位置' in title:  # 位置误差
                    ax.set_ylabel('unit: m')
                else:
                    ax.set_ylabel('unit: m')
                ax_side.yaxis.set_major_locator(plt.MultipleLocator(1))
                ax_side.set_ylim(0, 6)
                plotObj.ShowPlotFormat('', '1: 单点\n2: 差分\n4: 固定\n5: 浮点\n0:异常', legend_loc=1)
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

            plotObj.ShowPlotFormat('', name_map[title][-1])

            return ""
        except Exception as e:
            return title + "生成失败原因:" + str(e) + "\n"

    # INS 单图绘制
    def ins_start_plot(self, plot_title):
        error_msg = ""
        subplot_num = 1
        if plot_title == "GPS解状态占比饼状图":
            error_msg += self.plot_gps_quality(plot_title)
        else:
            # 先获取误差数据
            plotObj = PlotData()
            plotObj.fig.suptitle(plot_title)
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
                error_msg = error_msg + self.plot_time_gap(plotObj, plotObj.ax, plot_title)
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
            elif plot_title == "统计误差分布图":
                if 'sync_' not in ','.join(list(self.ins_test_data[0].keys())):
                    error_msg += '因无同步数据，无法绘制' + plot_title
                    return error_msg
                else:
                    plotObj.ax1 = plt.subplot(3, 1, 1)
                    error_msg += self.plot_error(plotObj, plotObj.ax1, title='位置水平误差')
                    plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                    error_msg += self.plot_val_diff(plotObj, plotObj.ax2, title='航向偏差统计')
                    plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                    error_msg += self.plot_error(plotObj, plotObj.ax3, title='速度偏差统计')
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
        plotObj.fig.suptitle("INS综合误差历元分布图")

        pos_error_name = ['位置水平误差', '位置横向误差', '位置纵向误差', '速度偏差统计']
        vel_diff_pic_name = ['北向速度对比', '东向速度对比', '地向速度对比', '前向速度对比', '右向速度对比', '下向速度对比', '速度对比'
                                , '经度对比', '纬度对比', '高度对比', 'Roll对比', 'Pitch对比', 'Yaw对比', '航向偏差统计']
        vel_pic_name = ['北向速度', '东向速度', '地向速度', '前向速度', '右向速度', '下向速度', '速度'
                                , '经度', '纬度', '高度', 'Roll', 'Pitch', 'Yaw', '前向里程', '右向里程', '下向里程']
        bias_name = ['Pp', 'Pv', 'Patt', 'Pba', 'Pbg', 'Palign', 'Plbbg', 'Plbbc', 'Pkws', 'PAtttg'
            , 'Xp', 'Xv', 'Xatt', 'Xba', 'Xbg', 'Xalign', 'Xlbbg', 'Xlbbc', 'Xkws', 'XAtttg'
            , 'Ba', 'Bg', 'Atttb', 'Lttg', 'Lttc', 'Kws', 'Atttg', 'Ltgc']

        for i in range(len(plot_list)):
            plotObj.ax = plt.subplot(subplot_num * 100 + 1 * 10 + (i + 1), sharex=plotObj.ax)
            plotObj.ax.set_title(plot_list[i])

            if plot_list[i] == "历元间隔分布图":
                error_msg += self.plot_time_gap(plotObj, plotObj.ax, plot_list[i])
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
                       'ins_plot_flags': {4: '固定解', 5: '浮点解', 2: '差分解', 1: '单点解', 0: '其他'},
                        'ins_plot_freq': 10,}
    obj.ins_test_data = data
    error_msg = obj.ins_start_plot('同步数据轨迹图')  # 画统计图流程
    if error_msg:
        print("【绘图失败】" + error_msg)
    else:
        print('done')
