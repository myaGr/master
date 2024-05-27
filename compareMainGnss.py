import os
import time
from Standardize import dataParse, dataPreProcess
from Utils import gnssStatistics, gnssPlot
from Utils.dataStatistics import gnssErrorCal
import uuid


class ParseRef:
    def __init__(self):
        self.ref_data = None  # 参考数据
        self.export_data = {}

    def run(self):
        # 1.参考数据解析
        self.output_msg("【导入参考数据】")
        try:
            self.output_msg("开始解析：" + self.ref_data["file_path"] + ", 数据类型：" + self.ref_data["file_type"] + ", 请等待...")
            self.output_msg("已设置测试日期： " + self.ref_data["date_time"] + ", 测试类型： " + self.ref_data["test_type"])
            if dataParse.allDataParse(self.ref_data, self.ref_data["date_time"], freq=self.ref_data["file_freq"]) == -1:
                self.output_msg("【导入失败】参考数据无解析结果, 请检查数据类型" + "\n")
                return
            self.sendDataframe()
            self.output_msg("【数据导入完毕】\n")
        except Exception as e:
            self.output_msg("【导入失败】参考数据解析失败,失败原因: 数据异常，" + str(e) + "\n")
        # 释放内存
        self.ref_data = {}

    # 更新消息框内容
    @staticmethod
    def output_msg(msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        print(msg)

    def sendDataframe(self):
        self.export_data = self.ref_data
        temp_path = r"./temp/"
        if not os.path.exists(temp_path):  # 创建仿真文件文件夹
            os.mkdir(temp_path)
        feather_path = temp_path + 'ref_' + str(uuid.uuid1()) + '.feather'
        self.ref_data["feather_path"] = feather_path
        self.ref_data["df"].to_feather(feather_path)


class CompareMainGnss:
    def __init__(self):
        self.ref_data = {}
        self.test_data = None  # 参考数据
        self.export_data = {}

    def run(self):
        self.output_msg("【导入测试数据】")
        # 1. 测试数据解析
        try:
            self.output_msg("开始解析：" + self.test_data["file_path"] + ", 数据类型：" + self.test_data["file_type"] + ", 请等待......")
            self.output_msg("已设置测试日期： " + self.test_data["date_time"])
            if dataParse.allDataParse(self.test_data, self.test_data["date_time"]) == -1:
                self.output_msg("【导入失败】无解析结果, 无法绘图统计, 请检查数据格式！" + "\n")
                return
            self.output_msg("测试数据解析完毕")
        except Exception as e:
            self.output_msg("【导入失败】测试数据解析失败,失败原因: 数据异常，" + str(e) + "\n")
            return

        # 2. 时间同步
        self.output_msg("开始时间同步, 同步参考数据为:" + str(self.test_data["ref_data"]))
        try:
            if self.test_data["test_type"] == '静态':
                sync_data = dataPreProcess.nullRefDataFun(self.ref_data, self.test_data["df"])
            else:
                sync_data = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df"])
            if len(sync_data[sync_data.keys()[0]]) == 0:
                self.output_msg("【导入失败】没有时间同步的结果,请检查参考数据对应的测试日期!")
                return
            self.test_data["sync_df"] = sync_data
        except Exception as e:
            self.output_msg("【导入失败】时间同步失败,失败原因: 数据异常，" + str(e) + "\n")
            return

        # 3. 误差计算
        try:
            bpox = self.test_data['bpox']   # 测试数据杆臂值
            if self.test_data["test_type"] == '动态':
                self.test_data["sync_df"] = gnssErrorCal(sync_data, bpox)
        except Exception as e:
            self.output_msg("同步数据统计误差计算失败，原因: 数据异常，" + str(e) + "\n")
            return

        # 传递解析数据
        self.sendDataframe()
        self.output_msg("【数据导入完毕】\n")
        # 释放内存
        self.test_data = {}
        self.ref_data = {}

    # 更新消息框内容
    @staticmethod
    def output_msg(msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        print(msg)

    def sendDataframe(self):
        self.export_data = self.test_data
        # print("df", self.test_data["df"])
        # print("sync_df", self.test_data["sync_df"])
        # print(self.test_data["sync_df"].keys())


class MainWindow:
    def __init__(self):
        self.ref_data = None
        self.gnss_test_data = []
        self.scene_list = []
        # 绘图配置初始化
        self.plot_config = {'time_type': 'gps', 'time': ['0', '0'],
                            'ins_plot_time_type': '计时时间（秒）', 'gnss_plot_time_type': 'utc时间',
                            'gnss_plot_flags': {4: '固定解', 5: '浮点解', 2: '差分解', 1: '单点解', 0: '其他'}}

    def export_gnss_statistics(self, export_file_path):
        self.output_msg("【统计开始】")
        # 场景分类
        statistics_all_scenes = []  # 汇总所有数据的场景信息
        for test_data in self.gnss_test_data:
            try:
                self.output_msg("开始场景分类:" + test_data["file_name"])
                # 每个场景的信息由{索引name、 数据data、 时间段time 和 占比percent} 组成
                scenes = dataPreProcess.sceneClassify(test_data, self.scene_list)
                statistics_all_scenes.extend(scenes)
            except Exception as e:
                self.output_msg("数据" + test_data["file_name"] + "场景分类失败,失败原因: 数据异常，" + str(e))

        # 精度统计
        reportObj = gnssStatistics.GnssReport()
        gps_flag = {"all": "所有解", "fixed": "固定解", "float": "浮点解", "pseduo": "伪距解", "single": "单点解"}
        self.output_msg("开始生成统计表")
        for scene in statistics_all_scenes:
            try:
                items = scene["name"].split("_", 3)
                items[2] = gps_flag[items[2]]
                if "all" in scene["name"]:
                    self.output_msg("统计场景：" + scene["name"] + "， 统计历元数：" + str(len(scene["data"])))
                if scene["data"].empty:  # 跳过场景分类后无数据的情况
                    self.output_msg("【提示】数据" + items[3] + "，场景：" + items[1] + "_" + items[2] + ", 无数据")
                statiObj = gnssStatistics.GnssStatistics()
                statiObj.percent = scene["percent"]
                statiObj.percent_ksxt = scene["percent_ksxt"]  # 双天线
                statiObj.scene_name = scene["name"]
                statiObj.SyncRefGpsData = scene["data"]
                statiObj.dataStatistics()  # 开始统计
                if self.ref_data['test_type'] == '动态':
                    reportObj.horizontal_error[scene["name"]] = statiObj.horizontal_error  # 水平误差
                    reportObj.longitudinal_error[scene["name"]] = statiObj.longitudinal_error  # 纵向误差
                    reportObj.lateral_error[scene["name"]] = statiObj.lateral_error  # 横向误差
                    reportObj.height_error[scene["name"]] = statiObj.height_error  # 高程误差
                    reportObj.velocity_error[scene["name"]] = statiObj.velocity_error  # 速度误差
                    reportObj.heading_error[scene["name"]] = statiObj.heading_error  # 航向误差
                    reportObj.hdop_sats[scene["name"]] = statiObj.hdop_sats  # Hdop和可用卫星数
                    reportObj.double_heading_error[scene["name"]] = statiObj.double_heading_error   # 双天线航向误差
                elif self.ref_data['test_type'] == '静态':
                    reportObj.horizontal_error[scene["name"]] = statiObj.horizontal_error  # 水平误差
                    reportObj.height_error[scene["name"]] = statiObj.height_error  # 高程误差
                    reportObj.hdop_sats[scene["name"]] = statiObj.hdop_sats  # Hdop和可用卫星数
            except Exception as e:
                self.output_msg("【提示】" + scene["name"] + "，精度统计失败，失败原因:" + str(e))

        # 生成统计表
        try:
            reportObj.reportPath = export_file_path
            reportObj.saveStatisticToExcel()
            self.output_msg("【统计完成】统计表输出路径：" + reportObj.reportPath + "\n")
        except Exception as e:
            self.output_msg("【统计失败】，失败原因:" + str(e))

    def gnss_data_plot(self, plot_title):
        if len(self.gnss_test_data) == 0:
            return "【绘图失败】，无数据可绘图"
        self.output_msg("【开始绘图】" + plot_title)
        plotObj = gnssPlot.gnssDataPlot()
        plotObj.plot_config = self.plot_config
        plotObj.gnss_test_data = self.gnss_test_data
        error_msg = plotObj.gnss_start_plot(plot_title)  # 画统计图流程
        if error_msg:
            self.output_msg("【绘图失败】" + error_msg)

    # 设置gnss综合误差图
    def gnss_multi_plot(self, plot_list):
        if len(self.gnss_test_data) == 0:
            self.output_msg("【绘图失败】 无GNSS数据可绘图")
            return
        self.output_msg("【开始绘图】绘制图有：" + str(plot_list))
        plotObj = gnssPlot.gnssDataPlot()
        plotObj.plot_config = self.plot_config
        plotObj.gnss_test_data = self.gnss_test_data
        error_msg = plotObj.gnss_multi_plot(plot_list)  # 画统计图流程
        if error_msg:
            self.output_msg("【绘图失败】" + error_msg)

    # 更新消息框内容
    @staticmethod
    def output_msg(msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        print(msg)


def add_scene(fdir):
    add_scenes = []
    try:
        with open(fdir, 'r', encoding='utf-8', errors='ignore') as fileObj:
            data = fileObj.readlines()
            for line in data:
                line = line.strip().split(",")
                scene = {"id": line[0],
                         "name": line[-1],
                         "time": [line[2], line[3]],
                         "time_type": line[1]}
                add_scenes.append(scene)
        return add_scenes
    except Exception as e:
        return print("场景解析失败：", e)


if __name__ == "__main__":
    test_date = '2023-10-18'
    sim_dir = r"C:\Users\wenzixuan\Downloads\kxst\2"
    objMain = MainWindow()
    # 1. 参考数据解析
    objParseRef = ParseRef()
    objParseRef.ref_data = {'test_type': '动态',
                            'static_ref_type': None,
                            'date_time': test_date,
                            'ref_cal_type': '均值',
                            # 'file_path': r'C:\Users\wenzixuan\Downloads\0914-tunnel-1.txt',
                            'file_path': sim_dir + r"\到POS320主天线自定义-1018PM.txt",
                            'file_type': 'POS320',  # 支持数据类型,  type: 【"NMEA", "100C", "POS320"】
                            'file_freq': None,
                            'csv_dict': {},
                            'file_name': 'ref_220919.txt'}  # os.path.basename(self.ref_data["file_path"])
    # objParseRef.ref_data = {'test_type': '静态', 'static_ref_type': '参考点坐标', 'latitude': '31.2078149109', 'longitude': '121.308974498', 'ellHeight': '14.494', 'bpox': [0, 0, 0], 'file_name': "['31.2078149109', '121.308974498', '14.494']"}
    objParseRef.run()
    objMain.ref_data = objParseRef.export_data

    # 2.  gnss测试数据解析
    objGnss = CompareMainGnss()
    objGnss.ref_data = objParseRef.export_data
    objGnss.test_data = {'algorith_type': 'GNSS',
                         'date_time': test_date,
                         # 'file_path': r'C:\Users\wenzixuan\Downloads\RTK20230807175605783\凯芯模组2-0725PM.log',
                         'file_path': sim_dir + r"\NAV3120-DIA-RTK-1018PM.ppp.nmea",
                         'file_type': 'NMEA',  # 支持数据类型,  type: 【"NMEA", "北云明文", "导远自定义-GPS"】
                         'csv_dict': {},
                         'test_type': '动态',
                         'file_name': 'test_220919.txt',
                         'ref_data': 'ref_220919.txt',
                         'bpox': [0, 0, 0]}
    # objGnss.test_data = {'algorithm_type': 'GNSS', 'date_time': '2023-08-15', 'file_path': 'C:/Users/wenzixuan/Downloads/01单边遮挡-误差统计/01-NAV3120-RTK-20230815.txt', 'file_type': 'NMEA', 'csv_dict': {}, 'bpox': [0, 0, 0]}
    objGnss.run()
    objMain.gnss_test_data.append(objGnss.export_data)

    # 3.GNSS精度统计流程
    file_path = os.getcwd() + r"\statistics_test123.xlsx"
    # # 方法1
    # objMain.scene_list = add_scene(r"C:\Users\wenzixuan\Downloads\csw1227\场景配置文件.txt")
    # # 方法2
    # objMain.scene_list = [{'id': '2', 'name': '内环高架上', 'time': ['0', '281800'], 'time_type': 'gps'},
    #                       {'id': '4', 'name': '林荫道', 'time': ['281810', '282000'], 'time_type': 'gps'},
    #                       # {'id': '5', 'name': '高楼多径', 'time': ['282010', '282200'], 'time_type': 'gps'},
    #                       # {'id': '71', 'name': '隧道1', 'time': ['282210', '0'], 'time_type': 'gps'}
    #                       ]

    objMain.export_gnss_statistics(file_path)  # 输出统计
    # objMain.gnss_multi_plot(["高程误差历元分布图"])
    objMain.gnss_data_plot("统计误差CDF分布图")
    # objMain.gnss_multi_plot(['历元间隔分布图', '差分龄期统计图', '位置水平误差历元分布图', '速度误差(前向)历元分布图', '姿态误差(航向)历元分布图'])
