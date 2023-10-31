import math
import pandas as pd
import numpy as np
from Utils.dataStatistics import DataStatistics

# GPS解状态对应中英文
gps_flag = {"all": "所有解", "fixed": "固定解", "float": "浮点解", "pseduo": "伪距解", "single": "单点解"}


class GnssStatistics(object):
    """
    功能： GNSS 精度统计
    @author: ZixuanWen
    @date: 2022-11-8
    """
    def __init__(self):
        self.SyncRefGpsData = None
        self.percent = 0
        self.scene_name = None
        self.horizontal_error = {}
        self.longitudinal_error = {}
        self.lateral_error = {}
        self.height_error = {}
        self.velocity_error = {}
        self.heading_error = {}
        self.hdop_sats = {}
        self.double_heading_error = {}

    # GNSS统计内容
    def dataStatistics(self):
        # 1. 位置距离误差统计(水平\横向\纵向\高程)
        self.horizontal_error.update(self.precisionStatistics("horizontal_error"))  # 水平偏差精度统计
        self.height_error.update(self.precisionStatistics("elevation_error"))  # 高程偏差精度统计

        if "longitudinal_error" in self.SyncRefGpsData and "lateral_error" in self.SyncRefGpsData:
            self.longitudinal_error.update(self.precisionStatistics("longitudinal_error"))  # 纵向偏差精度统计
            self.lateral_error.update(self.precisionStatistics("lateral_error"))  # 横向偏差精度统计

        # 2. 速度误差统计
        if "velocity_error" in self.SyncRefGpsData:
            self.velocity_error.update(self.precisionStatistics("velocity_error"))  # 对地速度精度统计
        if "heading_error" in self.SyncRefGpsData:
            self.heading_error.update(self.precisionStatistics("heading_error"))  # 航向精度统计
        if "double_heading_error" in self.SyncRefGpsData:
            self.double_heading_error.update(self.precisionStatistics("double_heading_error"))  # 航向精度统计

        # 3. Hdop可用卫星数误差统计
        if "hDop" in self.SyncRefGpsData and "satsNum" in self.SyncRefGpsData:
            self.hdop_sats.update(self.hdopSatesCalculation())

    # 计算精度统计指标
    def precisionStatistics(self, error_key):
        # 初始化统计字段dic
        key_list = ["场景", "解状态", "测试设备", "历元数", "占比", "RMS(外)", "CEP68(1σ)", "CEP95(2σ)", "CEP997(3σ)", "最大值", "最大值UTC", "STD"]
        precision = dict.fromkeys(key_list, "N/A")
        items = self.scene_name.split("_", 3)
        items[2] = gps_flag[items[2]]  # 转中文
        precision["场景"], precision["解状态"], precision["测试设备"] = items[1], items[2], items[3]
        precision["历元数"] = str(len(self.SyncRefGpsData[error_key]))
        # 开始统计指标
        if len(self.SyncRefGpsData[error_key]) > 0:
            precision["占比"] = str(round(self.percent * 100, 2)) + "%"
            precision["RMS(外)"] = round(DataStatistics.rms_cal(self.SyncRefGpsData[error_key]), 3)
            precision["CEP68(1σ)"] = (DataStatistics.sigma_err_cal(self.SyncRefGpsData[error_key]))[0]
            precision["CEP95(2σ)"] = (DataStatistics.sigma_err_cal(self.SyncRefGpsData[error_key]))[1]
            precision["CEP997(3σ)"] = (DataStatistics.sigma_err_cal(self.SyncRefGpsData[error_key]))[2]
            precision["最大值"] = (DataStatistics.sigma_err_cal(self.SyncRefGpsData[error_key]))[3]
            precision["最大值UTC"] = str(self.SyncRefGpsData["utcTime"][self.SyncRefGpsData[error_key].idxmax()])[:10]
            precision["STD"] = round(self.SyncRefGpsData[error_key].std(), 3)
            # precision["RMS_2D(内)] = round(rms_cal(self.SyncRefGpsData["error_key"]), 3)
            # precision["Max_unix时间"] = self.SyncRefGpsData["unixTime"][self.SyncRefGpsData[error_key].idxmax()
            if error_key in ["horizontal_error", "longitudinal_error", "lateral_error"]:
                precision.update(DataStatistics.percent_statistic(self.SyncRefGpsData[error_key]))
        return precision

    # Hdop可用卫星数统计
    def hdopSatesCalculation(self):
        # 初始化统计字段dic
        key_list = ["场景", "解状态", "测试设备", "历元数", "占比", "平均HDOP", "最小HDOP", "最大HDOP", "平均卫星数", "最少卫星数", "最多卫星数"]
        hdop_sats = dict.fromkeys(key_list, "N/A")
        items = self.scene_name.split("_", 3)
        items[2] = gps_flag[items[2]]  # 转中文
        hdop_sats["场景"], hdop_sats["解状态"], hdop_sats["测试设备"] = items[1], items[2], items[3]
        hdop_sats["历元数"] = str(len(self.SyncRefGpsData["hDop"]))
        if len(self.SyncRefGpsData["hDop"]) > 0:
            hdop_sats["占比"] = str(round(self.percent * 100, 2)) + "%"
            hdop_sats["平均HDOP"] = str(round(np.mean(self.SyncRefGpsData["hDop"]), 1))
            hdop_sats["最小HDOP"] = str(round(np.min(self.SyncRefGpsData["hDop"]), 1))
            hdop_sats["最大HDOP"] = str(round(np.max(self.SyncRefGpsData["hDop"]), 1))
            hdop_sats["平均卫星数"] = str(int(np.mean(self.SyncRefGpsData["satsNum"])))
            hdop_sats["最少卫星数"] = str(np.min(self.SyncRefGpsData["satsNum"]))
            hdop_sats["最多卫星数"] = str(np.max(self.SyncRefGpsData["satsNum"]))
        return hdop_sats


class GnssReport(object):
    """
    功能： 输出 GNSS 统计报告 EXCEL
    @author: ZixuanWen
    @date: 2022-11-8
    """

    def __init__(self):
        self.reportPath = None    # 输出报告路径
        self.horizontal_error = {}     # 水平偏差
        self.longitudinal_error = {}      # 纵向偏差
        self.lateral_error = {}      # 横向偏差
        self.height_error = {}    # 高度偏差
        self.velocity_error = {}  # 速度偏差
        self.heading_error = {}   # 航向偏差
        self.hdop_sats = {}       # HDOP和卫星数
        self.double_heading_error = {}  # 双天线航向偏差
        self.writer = None

    # 保存统计结果为excel格式
    def saveStatisticToExcel(self):
        self.writer = pd.ExcelWriter(self.reportPath, engine="openpyxl")
        sheet1 = pd.DataFrame.from_dict(self.horizontal_error, orient="index")
        self.SetExcelStyle(sheet1, '水平误差')
        sheet2 = pd.DataFrame.from_dict(self.longitudinal_error, orient="index")
        self.SetExcelStyle(sheet2, '纵向误差')
        sheet3 = pd.DataFrame.from_dict(self.lateral_error, orient="index")
        self.SetExcelStyle(sheet3, '横向误差')
        sheet4 = pd.DataFrame.from_dict(self.height_error, orient="index")
        self.SetExcelStyle(sheet4, '高程误差')
        sheet5 = pd.DataFrame.from_dict(self.velocity_error, orient="index")
        self.SetExcelStyle(sheet5, '速度误差')
        sheet6 = pd.DataFrame.from_dict(self.heading_error, orient="index")
        self.SetExcelStyle(sheet6, '航向误差')
        sheet7 = pd.DataFrame.from_dict(self.hdop_sats, orient="index")
        self.SetExcelStyle(sheet7, 'DOP和可用卫星数')
        sheet8 = pd.DataFrame.from_dict(self.double_heading_error, orient="index")
        self.SetExcelStyle(sheet8, '双天线航向偏差')
        self.writer.save()

    # 配置excel格式风格
    def SetExcelStyle(self, dataframe, sheet_name):
        dataframe = dataframe.sort_index()
        dataframe.style.set_properties(**{'text-align': 'center'}).to_excel(self.writer, sheet_name)
        sheet = self.writer.sheets[sheet_name]
        sheet.column_dimensions['A'].width = 0.1
        sheet.column_dimensions['B'].width = 12
        sheet.column_dimensions['C'].width = 10
        sheet.column_dimensions['D'].width = 20
        sheet.column_dimensions['E'].width = 10
        for key in [chr(i) for i in range(ord('F'), ord('R')+1)]:
            sheet.column_dimensions[key].width = 11.5
