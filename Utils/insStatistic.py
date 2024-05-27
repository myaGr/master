# 计算精度统计指标
import os
import math
import pandas as pd
import numpy as np
from Utils.dataStatistics import DataStatistics
from openpyxl.reader.excel import load_workbook

# GPS解状态对应中英文
gps_flag = {"all": "所有解", "fixed": "固定解", "float": "浮点解", "pseduo": "伪距解", "single": "单点解",
            "none": "其他"}

R = 6378137.0  # 地球长半轴
f = 1 / 298.257223563  # 地球椭球扁率
e2 = f * (2 - f)  # 椭球偏心率的平方


class insStatistics(object):
    """
    功能： INS 精度统计
    @author: liqianwen
    @date: 2023-8-15
    """

    def __init__(self):
        self.cal_info_dict = None
        self.percent = 0
        self.scene_name = None
        self.horizontal_error = {}
        self.longitudinal_error = {}
        self.lateral_error = {}
        self.height_error = {}
        self.velocity_error = {}
        self.heading_error = {}
        self.hdop_sats = {}

    # 统计内容
    def data_statistics(self):
        # 1. 位置距离误差统计(水平\横向\纵向\高程)
        self.horizontal_error.update(self.precision_statistics("horizontal_error"))  # 水平偏差精度统计
        self.height_error.update(self.precision_statistics("elevation_error"))  # 高程偏差精度统计

        if "longitudinal_error" in self.cal_info_dict and "lateral_error" in self.cal_info_dict:
            self.longitudinal_error.update(self.precision_statistics("longitudinal_error"))  # 纵向偏差精度统计
            self.lateral_error.update(self.precision_statistics("lateral_error"))  # 横向偏差精度统计

        # 2. 速度误差统计
        if "north_vel_diff" in self.cal_info_dict:
            self.velocity_error.update(self.precision_statistics("velocity_error"))  # 对地速度精度统计
        if "heading_error" in self.cal_info_dict:
            self.heading_error.update(self.precision_statistics("heading_error"))  # 航向精度统计

        # 3. Hdop可用卫星数误差统计
        if "hDop" in self.cal_info_dict and "satsNum" in self.cal_info_dict:
            # self.hdop_sats.update(self.hdopSatesCalculation())
            pass

    def precision_statistics(self, error_key):
        # 初始化统计字段dic
        key_list = ["场景", "解状态", "测试设备", "历元数", "占比", "RMS(外)", "CEP68(1σ)", "CEP95(2σ)", "CEP997(3σ)",
                    "最大值", "最大值UTC", "STD", "均值"]
        precision = dict.fromkeys(key_list, "N/A")
        items = self.scene_name.split("_", 3)
        items[2] = gps_flag[items[2]]  # 转中文
        precision["场景"], precision["解状态"], precision["测试设备"] = items[1], items[2], items[3]
        precision["历元数"] = str(len(self.cal_info_dict[error_key]))
        # 开始统计指标
        if len(self.cal_info_dict[error_key]) > 0:
            precision["占比"] = str(round(self.percent * 100, 2)) + "%"
            precision["RMS(外)"] = round(DataStatistics.rms_cal(self.cal_info_dict[error_key]), 3)

            sigma_data = DataStatistics.sigma_err_cal(self.cal_info_dict[error_key])
            precision["CEP68(1σ)"] = sigma_data[0]
            precision["CEP95(2σ)"] = sigma_data[1]
            precision["CEP997(3σ)"] = sigma_data[2]
            precision["最大值"] = sigma_data[3]

            precision["最大值UTC"] = str(self.cal_info_dict["utcTime"][self.cal_info_dict[error_key].idxmax()])[:10]
            precision["STD"] = round(self.cal_info_dict[error_key].std(), 4)
            precision["均值"] = round(self.cal_info_dict[error_key].mean(), 4)

            if error_key in ["horizontal_error", "longitudinal_error", "lateral_error"]:
                precision.update(DataStatistics.percent_statistic(self.cal_info_dict[error_key]))

        return precision


class insReport(object):
    """
    功能： 输出 INS 统计报告 EXCEL
    @author: liqianwen
    @date: 2023-8-17
    """

    def __init__(self):
        self.reportPath = os.getcwd() + '\statistic.xlsx'  # 输出报告路径
        self.writer = None
        self.exist_flag = False

    # 保存统计结果为excel格式
    @staticmethod
    def info2feather(info, feather_path=None, newfile=False):
        """
        info里的数据在feather_path后面合并
        :param info:  数据大小要一致
        :param feather_path:
        :param newfile:
        :return:
        """
        feather_path = feather_path if feather_path else 'info.feather'

        output_info = ''
        try:
            if type(info) == dict:
                df = pd.DataFrame.from_dict(info)
            else:
                df = pd.DataFrame(info)

            if not os.path.exists(feather_path) or newfile:
                df.to_feather(feather_path)
            else:
                df_old = pd.read_feather(feather_path)
                if len(df) != len(df_old):
                    output_info += feather_path + '文件中数据大小长度和输入数据不一致，无法追加保存！'
                    return output_info
                pd.concat([df_old, df], axis=1).to_feather(feather_path)
            output_info += feather_path + " 已存储。"
        except Exception as e:
            output_info += feather_path + " 本地存储失败：" + "\n"
            output_info += str(e)

        return output_info

    # 保存统计结果为excel格式
    def info2excel(self, info, excel_path=None, sheet_name='原始信息'):
        excel_path = excel_path if excel_path else self.reportPath
        output_info = ''
        try:
            self.writer = pd.ExcelWriter(excel_path, engine="openpyxl", mode='w')
            if type(info) == dict:
                sheet = pd.DataFrame.from_dict(info, orient="index")
            else:
                sheet = pd.DataFrame(info)
            self.set_excel_style(sheet, sheet_name)
            self.writer.save()
            output_info += excel_path + '的sheet ' + sheet_name + " 已存储。"
        except Exception as e:
            output_info += excel_path + '的sheet ' + sheet_name + " 本地存储失败：" + "\n"
            output_info += e

        return output_info

    # 保存统计结果为excel格式
    def statistic2excel(self, scenes_df, data_type='基准与INS数据同步统计结果'):
        output_info = ''
        try:
            statistic_name = {'horizontal_error': '水平误差', 'longitudinal_error': '纵向误差'
                , 'lateral_error': '横向误差', 'elevation_error': '高程误差'
                , 'pitch_diff': '俯仰偏差', 'heading_diff': '航向偏差', 'roll_diff': '横滚偏差'
                , 'ground_vel_diff': 'Z（下）速度偏差', 'velocity_diff': '水平速度误差'
                , 'forward_vel_diff': 'X（前）速度误差', 'right_vel_diff': 'Y（右）速度偏差'}
            # 姿态：横滚、俯仰、速度、x速度、y速度、z速度
            # 位置：123theta、0.02-2占比水平误差，横纵高误差
            # gps:各种解状态占比
            # GPS的上述指标

            sheets = {}
            for scene_df in scenes_df:
                if len(scene_df['data']) == 0 or 'statistic_info' not in scene_df.keys():
                    continue
                for error_name, error_dict in scene_df['statistic_info'].items():
                    scene_base = scene_df['name'].split('_')
                    output_dict = {'名字': scene_df['name']
                        , '场景': scene_base[1]
                        , '解状态': scene_base[2]
                        , '文件名': '_'.join(scene_base[3:])
                        , '数据类型': data_type
                        , '时间段': str(scene_df['time']) if 'time' in scene_df.keys() else ''
                        , '帧数': len(scene_df['data'])
                        # , '场景占比': str(scene_df['percent'] * 100) + '%' if 'percent' in scene_df.keys() else ''
                       }
                    if error_name in statistic_name.keys():
                        output_dict.update(error_dict)
                        if statistic_name[error_name] in sheets.keys():
                            new_info = pd.DataFrame.from_dict(output_dict, orient="index").T
                            sheets[statistic_name[error_name]] = pd.concat(
                                [sheets[statistic_name[error_name]], new_info])
                        else:
                            sheets[statistic_name[error_name]] = pd.DataFrame.from_dict(output_dict, orient="index").T
                        # sheets.append(pd.DataFrame.from_dict(output_dict, orient="index").T)
                        # self.set_excel_style(sheets[-1], statistic_name[error_name])

            if os.path.exists(self.reportPath):
                self.exist_flag = True
                wb = load_workbook(self.reportPath)
                self.writer = pd.ExcelWriter(self.reportPath, engine='openpyxl', mode='w')
                self.writer.book = wb
                self.writer.sheets = dict((ws.title, ws) for ws in self.writer.book.worksheets)
            else:
                self.exist_flag = False
                self.writer = pd.ExcelWriter(self.reportPath, engine='openpyxl')

            for sheet_name in sheets:
                if sheet_name in self.writer.sheets.keys():
                    for i in range(len(sheets[sheet_name])):
                        # print(list(sheets[sheet_name].iloc[i]))
                        self.writer.sheets[sheet_name].append(list(sheets[sheet_name].iloc[i]))
                else:
                    sheets[sheet_name].to_excel(self.writer, sheet_name=sheet_name, index=False)

                self.writer.sheets[sheet_name].column_dimensions['A'].width = 0.1
                self.writer.sheets[sheet_name].column_dimensions['B'].width = 15
                self.writer.sheets[sheet_name].column_dimensions['C'].width = 10
                self.writer.sheets[sheet_name].column_dimensions['D'].width = 10
                self.writer.sheets[sheet_name].column_dimensions['E'].width = 25
                for key in [chr(i) for i in range(ord('F'), ord('R') + 1)]:
                    self.writer.sheets[sheet_name].column_dimensions[key].width = 11.5

            self.writer.save()
            self.writer.close()
            self.exist_flag = True
            output_info += data_type + '已保存'
        except Exception as e:
            print(str(self.reportPath) + " 统计结果输出失败，失败原因:")
            output_info += data_type + '输出失败，失败原因:' + '\n'
            output_info += str(e)
            self.writer.close()
        return output_info


def precision_statistics(df, val_name='', percent=1):
    # 初始化统计字段dic
    key_list = ["RMS(外)", "CEP68(1σ)", "CEP95(2σ)", "CEP997(3σ)", "最大值", "最大值UTC", "STD", "均值"]
    output = dict.fromkeys(key_list, "N/A")
    # 开始统计指标
    if len(df[val_name]) > 0:
        try:
            df[val_name] = pd.Series(df[val_name]) if type(df[val_name]) != pd.Series else df[val_name]
            output["占比"] = str(round(percent * 100, 2)) + "%"
            output["RMS(外)"] = round(DataStatistics.rms_cal(df[val_name]), 3)

            sigma_data = DataStatistics.sigma_err_cal(df[val_name])
            output["CEP68(1σ)"] = sigma_data[0]
            output["CEP95(2σ)"] = sigma_data[1]
            output["CEP997(3σ)"] = sigma_data[2]
            output["最大值"] = sigma_data[3]

            output["最大值UTC"] = str(df["utcTime"][df[val_name].idxmax()])[:10]
            output["STD"] = round(df[val_name].std(), 4)
            output["均值"] = round(df[val_name].mean(), 4)

            if val_name in ["horizontal_error", "longitudinal_error", "lateral_error"]:
                output.update(DataStatistics.percent_statistic(df[val_name]))
        except Exception as e:
            print(str(val_name) + '的统计指标有误：')
            print(e)

    return output


def pos_covert(df, pos0=None, yaw=None, bpos=None, val_name=None):
    """
    位置坐标转换:地理坐标系转北东地平面坐标坐标系
    :param df:
    :param pos0: 起始坐标点， 列表list
    :param yaw: 航向 数组array
    :param bpos: 杆臂值（固定偏差）， 列表list[前、右、下]
    :param val_name:
    :return： 平面坐标系下N、E、D数组array
    """
    val_name = ['latitude', 'longitude', 'ellHeight'] if val_name is None else val_name
    lat, lon, height = df[val_name[0]], df[val_name[1]], df[val_name[2]]
    if len(lat) == 0 or len(lon) == 0 or len(height) == 0:
        return dict.fromkeys(['n_axis', 'e_axis', 'd_axis'], np.array([]))
    # 如无起始坐标，则以坐标的第一个点为起始点。
    if pos0 is None:
        pos0 = [lat[0], lon[0], height[0]]

    # (1) 基础公式计算
    L = lat * math.pi / 180
    sinL = np.sin(L)
    cosL = np.cos(L)
    temp = np.sqrt(1 - e2 * sinL * sinL)
    temp3 = temp * temp * temp
    Rn = R * (1 - e2) / temp3  # 子午面曲率半径
    Re = R / temp  # 横向曲率半径
    RnhINS = Rn + height
    RehINS = (Re + height) * cosL

    # (2) 地理坐标转平面坐标
    north = (lat - pos0[0]) / 180 * math.pi * RnhINS
    east = (lon - pos0[1]) / 180 * math.pi * RehINS
    ground = height

    # (3)固定偏差补偿(如有)
    if yaw is not None:
        if bpos is None:
            bpos = np.array([0, 0, 0])

        # a. 杆臂值固定偏差（前右下转北东地坐标系）校正:
        sin3 = np.sin(yaw / 180 * math.pi)
        cos3 = np.cos(yaw / 180 * math.pi)
        pos_err = [cos3 * bpos[0] - sin3 * bpos[1], sin3 * bpos[0] + cos3 * bpos[1], - bpos[2]]
        # b. 北东地坐标系下坐标校正，补偿固定偏差:
        north = north - pos_err[0]
        east = east - pos_err[1]
        ground = ground - pos_err[2]

    return {'n_axis': north, 'e_axis': east, 'd_axis': ground}


def pos_error_cal(coordinate1, coordinate2=None, yaw=None, coor1_val_name=None, coor2_val_name=None):
    """
    位置误差计算
    :param coordinate1: 北东地平面坐标系下，坐标1(测试数据)的[N、E、D]数组array
    :param coordinate2: 北东地平面坐标系下，坐标2(参考数据)的[N、E、D]数组array
    :param yaw: 航向 数组array
    :param coor1_val_name: 航向 数组array
    :param coor2_val_name: 航向 数组array
    @author:liqianwen
    """
    coor1_val_name = ['n_axis', 'e_axis', 'd_axis'] if not coor1_val_name else coor1_val_name
    coor2_val_name = coor1_val_name if not coor2_val_name else coor2_val_name
    output = dict.fromkeys(['horizontal_error', 'longitudinal_error', 'lateral_error', 'elevation_error'], np.array([]))
    if coordinate2 is None:
        if len(coordinate1[coor1_val_name[0]]) == 0 or len(coordinate1[coor1_val_name[1]]) == 0:
            return output
        # 偏差角度（限定北纬东经）
        angle = np.arctan2(coordinate1[coor1_val_name[1]], coordinate1[coor1_val_name[0]]) * -1
        trans_data = 1 / 4 * 2 * math.pi * 5  # 回退90°
        output["theta"] = angle + trans_data
        # 水平偏差
        output["horizontal_error"] = np.sqrt(
            (coordinate1[coor1_val_name[0]]) ** 2 + (coordinate1[coor1_val_name[1]]) ** 2)
    else:
        if len(coordinate1[coor1_val_name[0]]) == 0 or len(coordinate2[coor1_val_name[0]]) == 0:
            return output
        if yaw is not None:
            # 计算航向三角函数
            sin3 = np.sin(yaw / 180 * math.pi)
            cos3 = np.cos(yaw / 180 * math.pi)

            # 纵向偏差
            output["longitudinal_error"] = cos3 * (coordinate1[coor1_val_name[0]] - coordinate2[coor2_val_name[0]]) \
                                           + sin3 * (coordinate1[coor1_val_name[1]] - coordinate2[coor2_val_name[1]])
            # 横向偏差
            output["lateral_error"] = -sin3 * (coordinate1[coor1_val_name[0]] - coordinate2[coor2_val_name[0]]) \
                                      + cos3 * (coordinate1[coor1_val_name[1]] - coordinate2[coor2_val_name[1]])
        # 水平偏差
        output["horizontal_error"] = np.sqrt((coordinate1[coor1_val_name[0]] - coordinate2[coor2_val_name[0]]) ** 2
                                             + (coordinate1[coor1_val_name[1]] - coordinate2[coor2_val_name[1]]) ** 2)
        # 高程偏差
        output["elevation_error"] = coordinate1[coor1_val_name[2]] - coordinate2[coor2_val_name[2]]

    return output


def four_wheel_speed_cal(velocity_df, kws_list, name_list=None):
    """

    :param velocity_df:
    :param kws_list:
    :param name_list: ["WheelSpeedFrontLeft", "WheelSpeedFrontRight", "WheelSpeedBackLeft", "WheelSpeedBackRight"] 对应 前左、前右、后左、后右 的字段名
    :return:
    """
    name_list = ["WheelSpeedFrontLeft", "WheelSpeedFrontRight", "WheelSpeedBackLeft",
                 "WheelSpeedBackRight"] if not name_list else name_list
    output_dict = dict.fromkeys(['DriR_L', 'DriR_R', 'NonDriR_L', 'NonDriR_R'], np.array([]))

    if len(kws_list) == 0 or len(velocity_df) == 0:
        return output_dict

    kws_mean = np.mean(kws_list, axis=0)
    output_dict["DriR_L"] = velocity_df[name_list[0]] * kws_mean[0]
    output_dict["DriR_R"] = velocity_df[name_list[1]] * kws_mean[1]
    output_dict["NonDriR_L"] = velocity_df[name_list[2]] * kws_mean[0]
    output_dict["NonDriR_R"] = velocity_df[name_list[3]] * kws_mean[1]

    return output_dict


def gps_flags_transfer(df, flags_pos_name='flagsPos', lat_name='latitude', lon_name='longitude'):
    output_dict = dict.fromkeys(['flags_pos', 'none', 'single', 'pseduo', 'float', 'fixed'], np.array([]))
    if len(df) == 0:
        return output_dict

    # # flags_pos = np.array([round(x) for x in np.array(df[flags_pos_name])])
    # flags_pos = np.array(df[flags_pos_name])
    # # flags_pos[np.where((flags_pos != 48) | (flags_pos != 49) | (flags_pos != 50) | (flags_pos != 16) | (flags_pos != 17) | (flags_pos != 34))[0]] = 0
    # lat_array = np.array(df[lat_name])
    # lon_array = np.array(df[lon_name])
    # output_dict['none'] = [lat_array[np.where(flags_pos == 0)[0]], lon_array[np.where(flags_pos == 0)[0]]]
    # output_dict['single'] = [lat_array[np.where(flags_pos == 16)[0]], lon_array[np.where(flags_pos == 16)[0]]]
    # output_dict['pseduo'] = [lat_array[np.where(flags_pos == 17)[0]], lon_array[np.where(flags_pos == 17)[0]]]
    # output_dict['float'] = [lat_array[np.where(flags_pos == 34)[0]], lon_array[np.where(flags_pos == 34)[0]]]
    # output_dict['fixed'] = [lat_array[np.where((flags_pos == 48) | (flags_pos == 49) | (flags_pos == 50))[0]]
    #                                 , lon_array[np.where((flags_pos == 48) | (flags_pos == 49) | (flags_pos == 50))[0]]]

    flags_pos = np.array(df[flags_pos_name])
    output_dict['flags_pos'] = np.array(flags_pos)
    output_dict['flags_pos'][np.where(flags_pos == 0)[0]] = 0
    output_dict['flags_pos'][np.where(flags_pos == 16)[0]] = 1
    output_dict['flags_pos'][np.where(flags_pos == 17)[0]] = 2
    output_dict['flags_pos'][np.where(flags_pos == 34)[0]] = 5
    output_dict['flags_pos'][np.where((flags_pos == 48) | (flags_pos == 49) | (flags_pos == 50))[0]] = 4

    lat_array = np.array(df[lat_name])
    lon_array = np.array(df[lon_name])
    output_dict['none'] = [lat_array[np.where(flags_pos == 0)[0]], lon_array[np.where(flags_pos == 0)[0]]]
    output_dict['single'] = [lat_array[np.where(flags_pos == 1)[0]], lon_array[np.where(flags_pos == 1)[0]]]
    output_dict['pseduo'] = [lat_array[np.where(flags_pos == 2)[0]], lon_array[np.where(flags_pos == 2)[0]]]
    output_dict['float'] = [lat_array[np.where(flags_pos == 5)[0]], lon_array[np.where(flags_pos == 5)[0]]]
    output_dict['fixed'] = [lat_array[np.where(flags_pos == 4)[0]], lon_array[np.where(flags_pos == 4)[0]]]

    output_dict['flags_pos'][np.where(output_dict['flags_pos'] > 6)[0]] = 0

    return output_dict


def speed_neg2frd(velocity_df, heading_list, speed_name=None):
    speed_name = ['north_vel', 'east_vel', 'ground_vel'] if not speed_name else speed_name
    output = {"forward_vel": np.array([]), "right_vel": np.array([]), "downward_vel": np.array([])}
    if len(velocity_df) == 0 or len(heading_list) == 0:
        print('速度 neg2frd 有误！')
        return output

    sin_yaw = np.sin(np.array(heading_list) / 180 * math.pi)
    cos_yaw = np.cos(np.array(heading_list) / 180 * math.pi)

    output['forward_vel'] = cos_yaw * velocity_df[speed_name[0]] + sin_yaw * velocity_df[speed_name[1]]
    output['right_vel'] = -sin_yaw * velocity_df[speed_name[0]] + cos_yaw * velocity_df[speed_name[1]]
    output['downward_vel'] = velocity_df[speed_name[2]]

    return output


def speed_hv2neg(velocity_df, track_angle_list, speed_name=None):
    """

    :param velocity_df:
    :param track_angle_list:  TrackAngle
    :param speed_name:
    :return:
    """
    speed_name = ['HSpd', 'velocity'] if not speed_name else speed_name
    output = dict.fromkeys(['north_vel', 'east_vel', 'ground_vel'], np.array([]))
    if len(velocity_df) == 0 or len(track_angle_list) == 0:
        print('速度 hv2neg 有误！')
        return output

    # 往往用于： GPS和INS同步数据 ['TrackAngle']
    sin_heading = np.sin(np.array(track_angle_list) / 180 * math.pi)
    cos_heading = np.cos(np.array(track_angle_list) / 180 * math.pi)

    if len(velocity_df[speed_name[0]]) != 0:
        output['north_vel'] = np.array(velocity_df[speed_name[0]]) * cos_heading
        output['east_vel'] = np.array(velocity_df[speed_name[0]]) * sin_heading

    if len(velocity_df[speed_name[1]]) != 0:
        output['ground_vel'] = velocity_df[speed_name[1]]

    return output


def diff_cal(df1, df2, df1_val_name=None, df2_val_name=None):
    df1_val_name = ["north_vel", "east_vel", "ground_vel"] if not df1_val_name else df1_val_name
    df2_val_name = ["north_vel", "east_vel", "ground_vel"] if not df2_val_name else df2_val_name
    output = dict.fromkeys([name + '_diff' for name in df2_val_name], np.array([0] * len(df1[df1_val_name[0]])))
    if len(df1_val_name) != len(df2_val_name):
        print('计算diff_cal时两 文件长度 或 字段名长度 不一致 ')
        return output

    for i in range(len(list(output.keys()))):
        output_key = list(output.keys())[i]
        if len(df1[df1_val_name[i]]) == len(df2[df2_val_name[i]]):
            if df2_val_name[i] == 'heading':
                output[output_key] = pd.Series(
                    DataStatistics().angele_standardization(np.array(df1[df1_val_name[i]] - df2[df2_val_name[i]])))
            else:
                output[output_key] = df1[df1_val_name[i]] - df2[df2_val_name[i]]

    return output


def sqrt_root_cal(df1, df1_val_name=None):
    df1_val_name = ["forward_vel", "right_vel"] if not df1_val_name else df1_val_name
    if len(df1) == 0:
        print('计算diff_cal时两 文件长度 或 字段名长度 不一致 ')
        return {}

    output_sum = pow(df1[df1_val_name[0]], 2)
    for i in range(len(df1_val_name)):
        output_sum = output_sum + pow(df1[df1_val_name[i]], 2)

    return np.sqrt(output_sum)


def frd_mileage_cal(velocity_df, time_list, speed_name=None):
    speed_name = ['forward_vel', 'right_vel', 'downward_vel'] if not speed_name else speed_name
    output = {"forward_mile": [], "right_mile": [], "downward_mile": []}
    try:
        if len(time_list) == 0 or len(velocity_df[speed_name[0]]) != len(time_list):
            print('计算frd里程有误！')
            return output

        time_diff = np.diff(time_list)
        output['forward_mile'] = np.cumsum(time_diff * np.array(velocity_df[speed_name[0]][1:]))
        output['right_mile'] = np.cumsum(time_diff * np.array(velocity_df[speed_name[1]][1:]))
        output['downward_mile'] = np.cumsum(time_diff * np.array(velocity_df[speed_name[2]][1:]))

        output['forward_mile'] = np.insert(output['forward_mile'], 0, 0)
        output['right_mile'] = np.insert(output['right_mile'], 0, 0)
        output['downward_mile'] = np.insert(output['downward_mile'], 0, 0)
    except Exception as e:
        print('sth wrong in frd_mileage_cal')
    return output


def change_val_name(df, orgin_name, new_name):
    require_df = dict.fromkeys(new_name, np.array([]))
    if len(df) == 0 or len(new_name) != len(orgin_name):
        print('名字转换有误！')
        print('涉及名字：' + str(orgin_name))
        return require_df

    for name in orgin_name:
        try:
            require_df[new_name[orgin_name.index(name)]] = df[name]
        except Exception as e:
            print('转换变量名字中有误，其中变量名为：', name)
            continue
        # df = df.drop(name, axis=1)

    return require_df
