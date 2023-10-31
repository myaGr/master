import numpy as np
import math


class DataStatistics:
    """
    数据统计
    @author: wenzixuan liqianwen
    """
    def __init__(self):
        self.R = 6378137.0  # 地球长半轴
        self.f = 1 / 298.257223563  # 地球椭球扁率
        self.e2 = self.f * (2 - self.f)  # 椭球偏心率的平方

    def pos_covert(self, lat, lon, height, pos0=None, yaw=None, bpos=None):
        """
        位置坐标转换:地理坐标系转北东地平面坐标坐标系
        :param  lat: 纬度, 数组array
        :param lon: 经度, 数组array
        :param height: 椭球高, 数组array
        :param pos0: 起始坐标点， 列表list
        :param yaw: 航向 数组array
        :param bpos: 杆臂值（固定偏差）， 列表list[N、E、D]
        :return： 平面坐标系下[N、E、D]数组array
        @author:wenzixuan
        """

        # 如无起始坐标，则以坐标的第一个点为起始点。
        if pos0 is None:
            pos0 = [lat[0], lon[0], height[0]]

        # (1) 基础公式计算
        L = lat * math.pi / 180
        sinL = np.sin(L)
        cosL = np.cos(L)
        temp = np.sqrt(1 - self.e2 * sinL * sinL)
        temp3 = temp * temp * temp
        Rn = self.R * (1 - self.e2) / temp3  # 子午面曲率半径
        Re = self.R / temp  # 横向曲率半径
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

        return np.array([north, east, ground])

    # 2组数据之间 位置误差计算
    @staticmethod
    def pos_error_cal(coordinate1, coordinate2=None, yaw=None):
        """
        位置误差计算
        :param coordinate1: 北东地平面坐标系下，坐标1(测试数据)的[N、E、D]数组array
        :param coordinate2: 北东地平面坐标系下，坐标2(参考数据)的[N、E、D]数组array
        :param yaw: 航向 数组array
        @author:wenzixuan
        """
        results = {}
        if coordinate2 is None:
            # 偏差角度（限定北纬东经）
            angle = np.arctan2(coordinate1[1], coordinate1[0]) * -1
            trans_data = 1 / 4 * 2 * math.pi * 5  # 回退90°
            results["theta"] = angle + trans_data
            # 水平偏差
            results["horizontal_error"] = np.sqrt((coordinate1[0]) ** 2 + (coordinate1[1]) ** 2)
        else:
            if yaw is not None:
                # 计算航向三角函数
                sin3 = np.sin(yaw / 180 * math.pi)
                cos3 = np.cos(yaw / 180 * math.pi)

                # 纵向偏差
                results["longitudinal_error"] = cos3 * (coordinate1[0] - coordinate2[0]) + sin3 * (
                        coordinate1[1] - coordinate2[1])
                # 横向偏差
                results["lateral_error"] = -sin3 * (coordinate1[0] - coordinate2[0]) + cos3 * (
                        coordinate1[1] - coordinate2[1])
            # 水平偏差
            results["horizontal_error"] = np.sqrt(
                (coordinate1[0] - coordinate2[0]) ** 2 + (coordinate1[1] - coordinate2[1]) ** 2)
            # 高程偏差
            results["elevation_error"] = coordinate1[2] - coordinate2[2]

        return results

    # 1σ偏差,2σ偏差,3σ偏差,最大偏差计算
    @staticmethod
    def sigma_err_cal(data_list):
        """
        概率密度函数1σ、2σ、3σ、最大值计算
        :param data_list: 待统计数据列表
        :return sigma_error: [1σ、2σ、3σ、最大值]对应值list
        """
        sigma_error = [0, 0, 0, 0]
        sigma = [0.6827, 0.9544, 0.9974]
        sorted_data = sorted(abs(data_list))
        sigma_error[0] = round(sorted_data[math.ceil(len(sorted_data) * sigma[0]) - 1], 3)  # 1倍sigame
        sigma_error[1] = round(sorted_data[math.ceil(len(sorted_data) * sigma[1]) - 1], 3)  # 2倍sigame
        sigma_error[2] = round(sorted_data[math.ceil(len(sorted_data) * sigma[2]) - 1], 3)  # 3倍sigame
        sigma_error[3] = round(sorted_data[-1], 3)  # 最大值
        return sigma_error

    # GNSS精度统计
    @staticmethod
    def percent_statistic(data_list):
        """
        小于指定数据范围占比统计
        :param data_list: 待统计数据列表
        :return: 小于指定数据范围占比统计结果（dictionary）
        @author: ZixuanWen
        @date: 2022-11-8
        """
        dist = [0.1, 0.2, 0.5, 1, 2]  # 指定统计范围
        data_list = abs(np.array(data_list))
        error_percent = {}
        for i in range(len(dist)):
            temp_error = len(data_list[np.where(data_list <= dist[i])]) / len(data_list)
            error_percent["<" + str(dist[i]) + "米占比"] = str(round(temp_error * 100, 2)) + "%"
        return error_percent

    # 均方根计算
    @staticmethod
    def rms_cal(data_list):
        """
        均方根计算
        :param data_list: 待统计数据列表
        :return: 均方根
        @author: ZixuanWen
        @date: 2022-11-8
        """
        rms = pow(sum([i ** 2 for i in data_list]) / len(data_list), 0.5)
        return rms

    # 所有角度统一到正负180°之间
    @staticmethod
    def angele_standardization(angle):
        """
        所有角度统一到正负180°之间, 输入angel为array
        :param angle: 输角度array
        :return: 标准化角度array
        @author: ZixuanWen
        @date: 2022-8-8
        """
        angle[np.where(angle > 180)] = angle[np.where(angle > 180)] - 360
        angle[np.where(angle < -180)] = angle[np.where(angle < -180)] + 360
        return angle

    # CDF值计算
    @staticmethod
    def cdf_cal(data_array):
        """
        数据CDF值计算
        :param data_array: list/array
        :return: cdf结果
        """
        hist, bin_edges = np.histogram(data_array, bins=len(data_array))
        cdf = np.cumsum(hist / sum(hist))
        return bin_edges[1:], cdf


# GNSS同步数据误差计算汇总
def gnssErrorCal(sync_data, bpox):
    """
    GNSS同步数据误差计算
    :param sync_data: 同步数据dataframe
    :param bpox: 测试数据杆臂值
    return：sync_data 包含误差值的dataframe
    """
    pos0 = [sync_data["latitude_x"][0], sync_data["longitude_x"][0], sync_data["ellHeight_x"][0]]
    if "heading_x" in sync_data:
        yaw = sync_data["heading_x"]
    else:
        yaw = None
    # 参考数据坐标转换
    ref_coords = DataStatistics().pos_covert(sync_data["latitude_x"], sync_data["longitude_x"],
                                             sync_data["ellHeight_x"], pos0, yaw)
    # 测试数据坐标转换（含杆臂值补偿）
    test_coords = DataStatistics().pos_covert(sync_data["latitude"], sync_data["longitude"], sync_data["ellHeight"],
                                              pos0, yaw, bpox)
    # 位置误差(横向纵向水平)计算
    results = DataStatistics.pos_error_cal(ref_coords, test_coords, yaw=yaw)

    # 输出结果保存
    sync_data["north_x"], sync_data["east_x"], sync_data["ground_x"] = ref_coords[0], ref_coords[1], ref_coords[2]
    sync_data["north"], sync_data["east"], sync_data["ground"] = test_coords[0], test_coords[1], test_coords[2]
    for key in results:
        sync_data[key] = results[key]
    if "velocity_x" in sync_data and "velocity" in sync_data:
        sync_data["velocity_error"] = sync_data['velocity_x'] - sync_data['velocity']  # 速度（对地速度）偏差计算
    if "heading_x" in sync_data and "heading" in sync_data:
        sync_data["heading_error"] = DataStatistics.angele_standardization(np.array(sync_data['heading_x'] - sync_data['heading']))  # 航向偏差计算
    if "heading_x" in sync_data and "doubleHeading" in sync_data:
        sync_data["double_heading_error"] = DataStatistics.angele_standardization(np.array(sync_data['heading_x'] - sync_data['doubleHeading']))  # 双天线航向偏差计算
    return sync_data


if __name__ == "__main__":
    # 导入数据
    load_dict = np.load(r'pos_cal_test_file.npy', allow_pickle=True).item()

    # 动态
    pos0 = [load_dict["lat"][0][0], load_dict["lon"][0][0], load_dict["h"][0][0]]  # 初始位置
    coordinate1 = DataStatistics().pos_covert(load_dict["lat"][0], load_dict["lon"][0], load_dict["h"][0], pos0)   # 坐标转换
    coordinate2 = DataStatistics().pos_covert(load_dict["lat"][1], load_dict["lon"][1], load_dict["h"][1], pos0)
    results = DataStatistics.pos_error_cal(coordinate1, coordinate2, yaw=load_dict["yaw"])  # 误差计算
    for key in results:
        print(key, results[key])

    # # 静态
    # lenArr = len(load_dict["lat"][0])
    # lon0 = np.sum(np.array(load_dict["lon"][1])) / lenArr
    # lat0 = np.sum(np.array(load_dict["lat"][1])) / lenArr
    # high0 = np.sum(np.array(np.array(load_dict["h"][1]))) / lenArr
    # pos0 = [lat0, lon0, high0]  # 初始位置
    # coordinate1 = DataStatistics().pos_covert(load_dict["lat"][0], load_dict["lon"][0], load_dict["h"][0], pos0)   # 坐标转换
    # results = DataStatistics.pos_error_cal(coordinate1)  # 误差计算
    #
    # print('end')

