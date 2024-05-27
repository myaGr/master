from Utils.dataMatPlot import PlotData
from Standardize.dataPreProcess import scenesDifferByTime
from Utils.dataStatistics import DataStatistics
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.rc("font", family='MicroSoft YaHei', weight='bold')


class gnssDataPlot:
    def __init__(self):
        self.plot_config = {'time_type': 'gps', 'time': ['0', '0'],
                            'ins_plot_time_type': '计时时间（秒）', 'gnss_plot_time_type': 'utc时间',
                            'gnss_plot_flags': {4: '固定解 Fix', 5: '浮点解 Float', 2: '差分解 Diff', 1: '单点解 Single',
                                                0: '其他 Else'}}
        self.gnss_test_data = []
        self.english_map = {"GNSS综合误差历元分布图": 'GNSS Comprehensive Error Epoch Distribution'
                            , '历元间隔分布图': 'Epoch Interval Distribution'
                            , '差分龄期统计图': 'Age of Differential'
                            , '解状态占比饼状图': 'Solution Status Percentage'
                            , '解状态水平误差序列图': 'Solution Status Error Sequence'
                            , '位置水平误差历元分布图': 'Epoch Distribution of Horizontal Position Error'
                            , '位置纵向误差历元分布图': 'Epoch Distribution of Longitudinal Position Error'
                            , '位置横向误差历元分布图': 'Epoch Distribution of Lateral Position Error'
                            , '高程误差历元分布图': 'Epoch Distribution of Height Error'
                            , '速度误差(前向)历元分布图': 'Epoch Distribution of Front Velocity Error'
                            , '姿态误差(航向)历元分布图': 'Epoch Distribution of Attitude Error'
                            , '位置误差(横向、纵向、水平)历元分布图': 'Epoch Distribution of Positional Deviation (Lateral, Longitudinal, Horizontal)'
                            , '位置水平误差': 'Horizontal Position Error', '位置纵向误差': 'Longitudinal Position Error'
                            , '位置横向误差': 'Lateral Position Error'
                            , '双天线航向误差历元分布图': 'Epoch Distribution of Double Heading Error'
                            , "统计误差CDF分布图": 'Statistics Error CDF'
                            , '水平误差cdf': 'Horizontal Error CDF', '速度误差cdf': 'Velocity Error CDF'
                            , '航向误差cdf': 'Heading Error CDF'}

    def set_polt_data(self, test_data):
        time_cut = [
            {'time_type': self.plot_config['time_type'], 'time': self.plot_config['time'], 'id': "", 'name': ""}]
        # 限制画图范围
        if self.plot_config['time'] != ['0', '0']:
            cut_data = scenesDifferByTime(test_data["sync_df"], time_cut, test_data["file_name"])
            plot_data = cut_data[0]['data']
        else:
            plot_data = test_data['sync_df']

        # 画图时间轴类型
        if self.plot_config['gnss_plot_time_type'] == 'utc时间':
            plot_time = pd.to_datetime(
                plot_data['utcDate'].astype('str') + ' ' + plot_data['utcTime'].astype('str'))
        elif self.plot_config['gnss_plot_time_type'] == '计时时间（秒）':
            plot_time = plot_data['gpsItow'] - plot_data['gpsItow'][0]
        else:
            plot_time = plot_data['gpsItow']
        return plot_data, plot_time

    def plot_gps_quality(self, title="解状态占比饼状图"):
        try:
            plotObj = PlotData()
            add_name = '\n' + self.english_map[title] if title in self.english_map.keys() else ''
            plotObj.fig.suptitle(title + add_name)

            num = len(self.gnss_test_data)
            for i in range(num):
                plotObj.ax = plt.subplot(1, num, i + 1)
                plotObj.ax.set_title(self.gnss_test_data[i]['file_name'])
                plot_data, plot_time = self.set_polt_data(self.gnss_test_data[i])
                if 'gpsQuality' in plot_data:
                    gpsQuality = {}
                    length = len(plot_data['gpsQuality'])
                    count = plot_data['gpsQuality'].value_counts()
                    # 计算解状态的百分比
                    for k, v in self.plot_config['gnss_plot_flags'].items():
                        try:
                            gpsQuality[v] = count[k] / length
                        except KeyError:
                            gpsQuality[v] = 0
                    plt.pie(np.array(list(gpsQuality.values())),
                            labels=list(gpsQuality.keys()),  # 设置饼图标签
                            autopct='%.2f%%')  # 格式化输出百分比
            plotObj.ShowPlotFormat('', 'unit: s')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_time_gap(self, plotObj, ax, title="历元间隔分布图"):
        try:
            for test_data in self.gnss_test_data:
                plot_data, plot_time = self.set_polt_data(test_data)
                time_differ = list(np.diff(plot_data['unixTime']))
                plotObj.PlotData(ax, plot_time[1:], time_differ, test_data['file_name'])
            plotObj.ShowPlotFormat('', 'unit: s')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_age_differ(self, plotObj, ax, title='差分龄期统计图'):
        try:
            for test_data in self.gnss_test_data:
                plot_data, plot_time = self.set_polt_data(test_data)
                if 'gpsAge' in plot_data:
                    plotObj.PlotData(ax, plot_time, plot_data['gpsAge'], test_data['file_name'])
            plotObj.ShowPlotFormat('', 'unit: s')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_gps_quality_pos_error(self, plotObj, ax, title="解状态水平误差序列图"):
        try:
            for test_data in self.gnss_test_data:
                plot_data, plot_time = self.set_polt_data(test_data)
                if 'gpsQuality' in plot_data:
                    for k, v in self.plot_config['gnss_plot_flags'].items():  # 遍历字典给dataframe添加列（列为纵坐标）
                        plot_data[v] = plot_data.apply(lambda x: x.horizontal_error if x.gpsQuality == k else None,
                                                       axis=1)
                        plotObj.PlotData(ax, plot_time, plot_data[v], test_data['file_name'] + "【" + v + "】")
            plotObj.ShowPlotFormat('', 'unit: m')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_pos_error(self, plotObj, ax1, ax2, ax3, title="位置误差(水平、横向、纵向)历元分布图"):
        subtitle = ""
        try:
            if ax1:
                subtitle = '位置水平误差'
                add_name = '\n' + self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                ax1.set_title(subtitle + add_name)
                for test_data in self.gnss_test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    plotObj.PlotData(ax1, plot_time, plot_data['horizontal_error'], test_data['file_name'])
                plotObj.ShowPlotFormat('', 'unit: m')
                return ""
            if ax2 is not None:
                subtitle = '位置纵向误差'
                add_name = '\n' + self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                ax2.set_title(subtitle + add_name)
                for test_data in self.gnss_test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    plotObj.PlotData(ax2, plot_time, plot_data['longitudinal_error'], test_data['file_name'])
                plotObj.ShowPlotFormat('', 'unit: m')
                return ""
            if ax3 is not None:
                subtitle = '位置横向误差'
                add_name = '\n' + self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                ax3.set_title(subtitle + add_name)
                for test_data in self.gnss_test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    plotObj.PlotData(ax3, plot_time, plot_data['lateral_error'], test_data['file_name'])
                plotObj.ShowPlotFormat('', 'unit: m')
                return ""
        except Exception as e:
            error_msg = title + subtitle + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_elevation_error(self, plotObj, ax, title="高程误差历元分布图"):
        try:
            for test_data in self.gnss_test_data:
                plot_data, plot_time = self.set_polt_data(test_data)
                if 'elevation_error' in plot_data:
                    plotObj.PlotData(ax, plot_time, plot_data['elevation_error'], test_data['file_name'])
            plotObj.ShowPlotFormat('', 'unit: m')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_velocity_error(self, plotObj, ax, title="速度误差(前向)历元分布图"):
        try:
            for test_data in self.gnss_test_data:
                plot_data, plot_time = self.set_polt_data(test_data)
                if 'velocity_error' in plot_data:
                    plotObj.PlotData(ax, plot_time, plot_data['velocity_error'], test_data['file_name'])
            plotObj.ShowPlotFormat('', 'unit: m/s')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_heading_error(self, plotObj, ax, title="姿态误差(航向)历元分布图"):
        try:
            for test_data in self.gnss_test_data:
                plot_data, plot_time = self.set_polt_data(test_data)
                if 'heading_error' in plot_data:
                    plotObj.PlotData(ax, plot_time, plot_data['heading_error'], test_data['file_name'])
            plotObj.ShowPlotFormat('', 'unit:  ° ')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_double_heading_error(self, plotObj, ax, title="双天线航向误差历元分布图"):
        try:
            for test_data in self.gnss_test_data:
                plot_data, plot_time = self.set_polt_data(test_data)
                if 'double_heading_error' in plot_data:
                    plotObj.PlotData(ax, plot_time, plot_data['double_heading_error'], test_data['file_name'])
            plotObj.ShowPlotFormat('', 'unit: °')
            return ""
        except Exception as e:
            error_msg = title + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    def plot_cdf_error(self, plotObj, ax1, ax2, ax3, title='统计误差分布图'):
        subtitle = ""
        try:
            if ax1:
                subtitle = '水平误差cdf'
                add_name = '\n' + self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                # ax1.set_xlim(0.5, 1)    # 显示范围
                ax1.set_title(subtitle + add_name)
                for test_data in self.gnss_test_data:
                    plot_data, plot_time = self.set_polt_data(test_data)
                    bin_edges, cdf_error = DataStatistics.cdf_cal(plot_data['horizontal_error'].abs())
                    plotObj.PlotData(ax1, bin_edges, cdf_error, test_data['file_name'])
                plotObj.ShowPlotFormat('', 'pct: %')
                return ""
            if ax2:
                subtitle = '速度误差cdf'
                add_name = '\n' + self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                ax2.set_title(subtitle + add_name)
                for test_data in self.gnss_test_data:
                    if 'velocity_error' in test_data["sync_df"]:
                        plot_data, plot_time = self.set_polt_data(test_data)
                        bin_edges, cdf_error = DataStatistics.cdf_cal(plot_data['velocity_error'].abs())
                        plotObj.PlotData(ax2, bin_edges, cdf_error, test_data['file_name'])
                plotObj.ShowPlotFormat('', 'pct: %')
                return ""
            if ax3:
                subtitle = '航向误差cdf'
                add_name = '\n' + self.english_map[subtitle] if subtitle in self.english_map.keys() else ''
                ax3.set_title(subtitle + add_name)
                for test_data in self.gnss_test_data:
                    if 'heading_error' in test_data["sync_df"]:
                        plot_data, plot_time = self.set_polt_data(test_data)
                        bin_edges, cdf_error = DataStatistics.cdf_cal(plot_data['heading_error'].abs())
                        plotObj.PlotData(ax3, bin_edges, cdf_error, test_data['file_name'])
                plotObj.ShowPlotFormat('', 'pct: %')
                return ""
        except Exception as e:
            error_msg = title + subtitle + "生成失败原因:" + str(e) + r"\n"
            return error_msg

    # GNSS 单图绘制
    def gnss_start_plot(self, plot_title):
        error_msg = ""
        subplot_num = 1
        if plot_title == "解状态占比饼状图":
            error_msg = self.plot_gps_quality(plot_title)
        else:
            plotObj = PlotData()
            add_name = '\n' + self.english_map[plot_title] if plot_title in self.english_map.keys() else ''
            plotObj.fig.suptitle(plot_title + add_name)
            if plot_title == "历元间隔分布图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_time_gap(plotObj, plotObj.ax, plot_title)
            elif plot_title == "差分龄期统计图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_age_differ(plotObj, plotObj.ax, plot_title)
            elif plot_title == "解状态水平误差序列图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_gps_quality_pos_error(plotObj, plotObj.ax, plot_title)
            elif plot_title == "速度误差(前向)历元分布图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_velocity_error(plotObj, plotObj.ax, plot_title)
            elif plot_title == "姿态误差(航向)历元分布图":
                plotObj.ax = plt.subplot(1, 1, 1)
                error_msg = error_msg + self.plot_heading_error(plotObj, plotObj.ax, plot_title)
            elif plot_title == "位置误差(横向、纵向、水平)历元分布图":
                for test_data in self.gnss_test_data:
                    if 'longitudinal_error' in test_data["sync_df"]:
                        subplot_num = 3
                plotObj.ax1 = plt.subplot(subplot_num, 1, 1)
                error_msg = error_msg + self.plot_pos_error(plotObj, plotObj.ax1, None, None, plot_title)
                if subplot_num == 3:
                    plotObj.ax2 = plt.subplot(312, sharex=plotObj.ax1)
                    error_msg = error_msg + self.plot_pos_error(plotObj, None, plotObj.ax2, None, plot_title)
                    plotObj.ax3 = plt.subplot(313, sharex=plotObj.ax1)
                    error_msg = error_msg + self.plot_pos_error(plotObj, None, None, plotObj.ax3, plot_title)
            elif plot_title == "统计误差CDF分布图":
                for test_data in self.gnss_test_data:
                    if 'velocity_error' in test_data["sync_df"] and 'heading_error' in test_data["sync_df"]:
                        subplot_num = 3
                plotObj.ax1 = plt.subplot(subplot_num, 1, 1)
                error_msg = error_msg + self.plot_cdf_error(plotObj, plotObj.ax1, None, None, plot_title)
                if subplot_num == 3:
                    plotObj.ax2 = plt.subplot(312)
                    error_msg = error_msg + self.plot_cdf_error(plotObj, None, plotObj.ax2, None, plot_title)
                    plotObj.ax3 = plt.subplot(313)
                    error_msg = error_msg + self.plot_cdf_error(plotObj, None, None, plotObj.ax3, plot_title)
            else:
                error_msg = error_msg + "不可画图：" + plot_title
        plt.show()  # 显示图像

        if error_msg != "":
            return error_msg

    # GNSS综合误差历元分布图
    def gnss_multi_plot(self, plot_list):
        error_msg = ""
        subplot_num = len(plot_list)
        plotObj = PlotData()
        plotObj.fig.suptitle("GNSS综合误差历元分布图" + '\n' + self.english_map['GNSS综合误差历元分布图'])
        for i in range(len(plot_list)):
            plotObj.ax = plt.subplot(subplot_num * 100 + 1 * 10 + (i + 1), sharex=plotObj.ax)
            add_name = '\n' + self.english_map[plot_list[i]] if plot_list[i] in self.english_map.keys() else ''
            plotObj.ax.set_title(plot_list[i] + add_name)

            if plot_list[i] == "历元间隔分布图":
                error_msg = error_msg + self.plot_time_gap(plotObj, plotObj.ax, plot_list[i])
            elif plot_list[i] == "差分龄期统计图":
                error_msg = error_msg + self.plot_age_differ(plotObj, plotObj.ax, plot_list[i])
            elif plot_list[i] == "解状态水平误差序列图":
                error_msg = error_msg + self.plot_gps_quality_pos_error(plotObj, plotObj.ax, plot_list[i])
            elif plot_list[i] == "位置水平误差历元分布图":
                error_msg = error_msg + self.plot_pos_error(plotObj, plotObj.ax, None, None, plot_list[i])
            elif plot_list[i] == "位置纵向误差历元分布图":
                error_msg = error_msg + self.plot_pos_error(plotObj, None, plotObj.ax, None, plot_list[i])
            elif plot_list[i] == "位置横向误差历元分布图":
                error_msg = error_msg + self.plot_pos_error(plotObj, None, None, plotObj.ax, plot_list[i])
            elif plot_list[i] == "高程误差历元分布图":
                error_msg = error_msg + self.plot_elevation_error(plotObj, plotObj.ax, plot_list[i])
            elif plot_list[i] == "速度误差(前向)历元分布图":
                error_msg = error_msg + self.plot_velocity_error(plotObj, plotObj.ax, plot_list[i])
            elif plot_list[i] == "姿态误差(航向)历元分布图":
                error_msg = error_msg + self.plot_heading_error(plotObj, plotObj.ax, plot_list[i])
            elif plot_list[i] == "双天线航向误差历元分布图":
                error_msg = error_msg + self.plot_double_heading_error(plotObj, plotObj.ax, plot_list[i])
            else:
                error_msg = error_msg + "不可画图：" + plot_list
        plt.show()  # 显示图像

        if error_msg != "":
            return error_msg
