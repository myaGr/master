import os
import time
from MainWindow import dataParse
from Standardize import dataPreProcess
from Utils import insStatistic
from Utils.dataStatistics import gnssErrorCal


class ParseRef:
    def __init__(self):
        self.ref_data = None  # 参考数据
        self.export_data = {}

    def run(self):
        # 1.参考数据解析
        self.output_msg("【导入参考数据】")
        try:
            self.output_msg(
                "开始解析：" + self.ref_data["file_path"] + ", 数据类型：" + self.ref_data["file_type"] + ", 请等待...")
            self.output_msg(
                "已设置测试日期： " + self.ref_data["date_time"] + ", 测试类型： " + self.ref_data["test_type"])
            if dataParse.allDataParse(self.ref_data, self.ref_data["date_time"]) == -1:
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
        # print(self.ref_data)


class CompareMainIns:
    def __init__(self):
        self.ref_data = {}
        self.test_data = None  # 参考数据
        self.export_data = {}

    def run(self):
        self.output_msg("【导入INS测试数据】")
        # 1. 测试数据解析
        try:
            self.output_msg("开始解析：" + self.test_data["file_path"] + ", 数据类型：" + self.test_data["file_type"] + ", 请等待......")
            self.output_msg("已设置测试日期： " + self.test_data["date_time"])
            if dataParse.allDataParse(self.test_data, self.test_data["date_time"]) == -1:
                self.output_msg("【导入失败】ins帧无解析结果, 无法绘图统计, 请检查数据格式！" + "\n")
                return
        except Exception as e:
            self.output_msg("【导入失败】测试数据解析失败,失败原因: 数据异常，" + str(e) + "\n")
            return
        self.output_msg("【导入成功】")

        reportObj = insStatistic.insReport()
        report_path = '.'.join(self.test_data["file_path"].split(".")[:-1])
        # 2.1 时间同步：  sync_df = ref同步df
        if "df" in self.test_data.keys():
            try:
                self.output_msg("ins数据开始时间同步, 同步参考数据为:" + str(self.test_data["ref_data"]))
                sync_data = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df"])
                if len(sync_data[sync_data.keys()[0]]) == 0:
                    self.output_msg("【导入失败】没有ins帧时间同步的结果,请检查参考数据对应的测试日期!" + "\n")
                    return
                else:
                    self.test_data["sync_df"] = sync_data
                    self.output_msg("【时间同步完成】：ins帧与参考数据 ")
                    self.output_msg(reportObj.info2feather(sync_data, feather_path=report_path+'_ref同步ins.feather', newfile=True))
            except Exception as e:
                self.output_msg("【导入失败】ins帧时间同步失败,失败原因: 数据异常，" + str(e) + "\n")
                return

        # 2.2 时间同步：  sync_df_gps = df_gps同步df
        if "df_gps" in self.test_data.keys():
            try:
                self.output_msg("gps数据开始时间同步, 同步参考数据为:" + str(self.test_data["ref_data"]))
                sync_data_gps = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df_gps"])
                if len(sync_data_gps[sync_data_gps.keys()[0]]) == 0:
                    self.output_msg("【注意】没有gps帧时间同步的结果,请检查参考数据对应的测试日期!")
                else:
                    self.test_data["sync_df_gps"] = sync_data_gps
                    self.output_msg("【时间同步完成】：gps帧与参考数据 ")
                    self.output_msg(reportObj.info2feather(sync_data_gps, feather_path=report_path+'_ref同步gps.feather', newfile=True))
            except Exception as e:
                self.output_msg("gps帧时间同步失败,失败原因: 数据异常，" + str(e))

        # 2.3 时间同步：  sync_gps_ins = df_gps同步df
        if "df_gps" in self.test_data.keys() and "df" in self.test_data.keys():
            try:
                self.output_msg("测试数据 gps帧与ins帧 开始时间同步：" + str(self.test_data["file_path"]))
                sync_data_gpsins = dataPreProcess.timeSynchronize(self.test_data["df_gps"], self.test_data["df"])
                if len(sync_data_gpsins[sync_data_gpsins.keys()[0]]) == 0:
                    self.output_msg("【注意】没有gps帧与ins帧时间同步的结果,请检查参考数据对应的测试日期!")
                else:
                    self.test_data["sync_gps_ins"] = sync_data_gpsins
                    self.output_msg("【时间同步完成】：gps与ins ")
                    self.output_msg(reportObj.info2feather(sync_data_gpsins, feather_path=report_path+'_gps同步ins.feather', newfile=True))
            except Exception as e:
                self.output_msg("gps帧时间同步失败,失败原因: 数据异常，" + str(e))

        self.sendDataframe()  # 传递解析数据
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
        print("df", self.test_data["df"])
        print("sync_df", self.test_data["sync_df"])
        print(self.test_data["sync_df"].keys())


class MainWindow:
    def __init__(self):
        self.ref_data = None
        self.gnss_test_data = []
        self.ins_test_data = []
        self.scene_list = []
        self.statistics_results = []

    # 中间层，用来控制不同统计函数得出结果
    def export_ins_statistics(self, export_file_path):
        self.output_msg("【ins统计开始】")

        statistics_all_scenes_ins = []  # 汇总所有数据的场景信息
        statistics_all_scenes_gps = []  # 汇总所有数据的场景信息
        statistics_all_scenes_insgps = []  # 汇总所有数据的场景信息
        for test_data in self.ins_test_data:
            reportObj = insStatistic.insReport()
            report_path = '.'.join(test_data["file_path"].split(".")[:-1])
            test_data_index = self.ins_test_data.index(test_data)
            # 1. 场景分类 #######################################################
            self.output_msg("开始场景分类:" + test_data["file_name"])
            # 每个场景的信息由{索引name、 数据data、 时间段time 和 占比percent} 组成
            if 'sync_df' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list)
                    statistics_all_scenes_ins.extend(scenes)
                except Exception as e:
                    self.output_msg("数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            if 'sync_df_gps' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list, classify_data_name='sync_df_gps')
                    statistics_all_scenes_gps.extend(scenes)
                except Exception as e:
                    self.output_msg("数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            if 'sync_gps_ins' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list, classify_data_name='sync_gps_ins')
                    statistics_all_scenes_insgps.extend(scenes)
                except Exception as e:
                    self.output_msg("数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            # 2. 计算相关参数, 精度统计 #######################################################
            info_keys = []
            statistics_info_ins = [{'ref_info':{}, 'ins_info':{}, 'cal_info':{}, 'statistic_info':{}} for i in range(len(statistics_all_scenes_ins))]
            statistics_info_gps = [{'ref_info':{}, 'gps_info':{}, 'cal_info':{}, 'statistic_info':{}} for i in range(len(statistics_all_scenes_gps))]
            statistics_info_insgps = [{'gps_info':{}, 'ins_info':{}, 'cal_info':{}, 'statistic_info':{}} for i in range(len(statistics_all_scenes_insgps))]
            # 速度计算：位置、姿态、neg xyz速度
            self.output_msg("ref和INS同步数据相关信息：")
            if len(statistics_all_scenes_ins) != 0:
                for scene in statistics_all_scenes_ins:
                    scene_index = statistics_all_scenes_ins.index(scene)
                    if len(scene['data']) == 0:
                        self.output_msg(scene['name']+' 无符合的数据，无法统计！')
                        statistics_all_scenes_ins[scene_index].update(statistics_info_ins[scene_index])
                        continue
                    else:
                        self.output_msg(scene['name']+' 的ref和INS同步数据统计中...')
                    ################################### 速度 ###################################
                    # Ref北东地速度转换成前右下坐标系下的速度
                    statistics_info_ins[scene_index]['ref_info'].update(insStatistic.change_val_name(scene['data']
                                                                    , ['NorthVelocity_x', 'EastVelocity_x', 'GroundVelocity_x']
                                                                    , ['north_vel', 'east_vel', 'ground_vel']))
                    statistics_info_ins[scene_index]['ref_info'].update(insStatistic.speed_neg2frd(statistics_info_ins[scene_index]['ref_info']
                                                                        , scene['data']['heading_x']))
                    # INS北东地速度转换成前右下坐标系下的速度
                    statistics_info_ins[scene_index]['ins_info'].update(insStatistic.change_val_name(scene['data']
                                                                    , ['NorthVelocity', 'EastVelocity', 'GroundVelocity']
                                                                    , ['north_vel', 'east_vel', 'ground_vel']))
                    statistics_info_ins[scene_index]['ins_info'].update(insStatistic.speed_neg2frd(statistics_info_ins[scene_index]['ins_info']
                                                                        , scene['data']['heading']))
                    # sqrt_root_cal 用于计算（前右向）速度误差
                    statistics_info_ins[scene_index]['ref_info']['vel'] = insStatistic.sqrt_root_cal(
                                                                          statistics_info_ins[scene_index]['ref_info'])
                    statistics_info_ins[scene_index]['ins_info']['vel'] = insStatistic.sqrt_root_cal(
                                                                          statistics_info_ins[scene_index]['ins_info'])
                    ################################### 姿态 ###################################
                    statistics_info_ins[scene_index]['ins_info'].update(insStatistic.pos_covert(scene['data']))
                    statistics_info_ins[scene_index]['ref_info'].update(insStatistic.pos_covert(scene['data']
                                                                    , val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))
                    statistics_info_ins[scene_index]['cal_info'].update(insStatistic.pos_error_cal(
                                                                    statistics_info_ins[scene_index]['ins_info']
                                                                    ,statistics_info_ins[scene_index]['ref_info']
                                                                    , yaw=scene['data']['heading_x']))

                    ################################### 计算差值 ###################################
                    statistics_info_ins[scene_index]['cal_info'].update(insStatistic.diff_cal(
                                                                          statistics_info_ins[scene_index]['ref_info']
                                                                        , statistics_info_ins[scene_index]['ins_info']
                                                                        , df1_val_name=["north_vel", "east_vel", "ground_vel",'forward_vel', 'right_vel', 'downward_vel', 'vel']
                                                                        , df2_val_name=["north_vel", "east_vel", "ground_vel",'forward_vel', 'right_vel', 'downward_vel', 'vel']))
                    statistics_info_ins[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data']
                                                                        , df1_val_name=["roll_x", "pitch_x", "heading_x"], df2_val_name=["roll", "pitch", "heading"]))

                    ################################### 精度统计 ###################################
                    statistics_info_ins[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                    for key in statistics_info_ins[scene_index]['cal_info']:
                        if key != 'utcTime':
                            statistics_info_ins[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info_ins[scene_index]['cal_info']
                                                                                                                        , key, percent=scene['percent'])

                    ################################### 存到对应场景信息中 ###################################
                    statistics_all_scenes_ins[scene_index].update(statistics_info_ins[scene_index])

                ################################### 存储为feather ###################################
                # self.output_msg(reportObj.info2feather(statistics_all_scenes_ins[0]['ref_info'], feather_path=report_path+'_ref同步ins.feather'))         # 有需要的话存储ref和ins的 frd速度和xyz坐标

                info_dict = statistics_all_scenes_ins[0]['cal_info'].copy()
                for i in ['utcTime']:
                    info_dict.pop(i)
                self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path+'_ref同步ins.feather'))
                self.ins_test_data[test_data_index]['ref_ins_statistic_info'] = info_dict
                info_dict = None
                statistics_info_ins = None
            else:
                self.output_msg('无数据，不进行统计。')

            self.output_msg("ref和GPS同步数据相关信息：")
            if len(statistics_all_scenes_gps) != 0:
                for scene in statistics_all_scenes_gps:
                    scene_index = statistics_all_scenes_gps.index(scene)
                    if len(scene['data']) == 0:
                        self.output_msg(scene['name']+' 无符合的数据，无法统计！')
                        statistics_all_scenes_gps[scene_index].update(statistics_info_gps[scene_index])
                        continue
                    else:
                        self.output_msg(scene['name']+' 的ref和GPS同步数据统计中...')
                    ################################### 速度 ###################################
                    statistics_info_gps[scene_index]['ref_info'].update(insStatistic.change_val_name(scene['data']
                                                                    , ['NorthVelocity_x', 'EastVelocity_x', 'GroundVelocity_x']
                                                                    , ['north_vel', 'east_vel', 'ground_vel']))
                    statistics_info_gps[scene_index]['gps_info'].update(insStatistic.speed_hv2neg(scene['data'], scene['data']['TrackAngle']))
                    ################################### 姿态 ###################################
                    statistics_info_gps[scene_index]['gps_info'].update(insStatistic.pos_covert(scene['data']))
                    statistics_info_gps[scene_index]['ref_info'].update(insStatistic.pos_covert(scene['data']
                                                                    , val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))
                    statistics_info_gps[scene_index]['cal_info'].update(insStatistic.pos_error_cal(
                                                                    statistics_info_gps[scene_index]['gps_info']
                                                                    ,statistics_info_gps[scene_index]['ref_info']
                                                                    , yaw=scene['data']['heading_x']))
                    ################################### 解状态转换 ###################################
                    statistics_info_gps[scene_index]['cal_info'].update(insStatistic.gps_flags_transfer(scene['data']))

                    ################################### 计算差值 ###################################
                    statistics_info_gps[scene_index]['cal_info'].update(insStatistic.diff_cal(
                                                                          statistics_info_gps[scene_index]['ref_info']
                                                                        , statistics_info_gps[scene_index]['gps_info']))
                    statistics_info_gps[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data']
                                                                        , df1_val_name=["pitch_x", "heading_x"], df2_val_name=["pitch", "heading"]))
                    ################################### 精度统计 ###################################
                    statistics_info_gps[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                    for key in statistics_info_gps[scene_index]['cal_info'].keys():
                        if key not in ['utcTime', 'flags_pos', 'none', 'single', 'pseduo', 'float', 'fixed']:
                            statistics_info_gps[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info_gps[scene_index]['cal_info'], key)

                    ################################### 存到对应场景信息中 ###################################
                    statistics_all_scenes_gps[scene_index].update(statistics_info_gps[scene_index])

                info_dict = statistics_info_gps[0]['cal_info'].copy()
                for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime']:
                    info_dict.pop(i)
                self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path+'_ref同步gps.feather'))
                self.ins_test_data[test_data_index]['ref_gps_statistic_info'] = info_dict
                info_dict = None
                statistics_info_gps = None
            else:
                self.output_msg('无数据，不进行统计。')

            self.output_msg("GPS和INS同步数据相关信息：")
            if len(statistics_all_scenes_insgps) != 0:
                for scene in statistics_all_scenes_insgps:
                    scene_index = statistics_all_scenes_insgps.index(scene)
                    if len(scene['data']) == 0:
                        self.output_msg(scene['name']+' 无符合的数据，无法统计！')
                        statistics_all_scenes_insgps[scene_index].update(statistics_all_scenes_insgps[scene_index])
                        continue
                    else:
                        self.output_msg(scene['name']+' 的GPS和INS同步数据统计中...')
                    ################################### 速度 ###################################
                    statistics_info_insgps[scene_index]['gps_info'].update(insStatistic.speed_hv2neg(scene['data']
                                                                                                     , scene['data']['TrackAngle_x']
                                                                                                     , speed_name=['HSpd_x', 'velocity_x']))
                    statistics_info_insgps[scene_index]['ins_info'].update(insStatistic.change_val_name(scene['data']
                                                                    , ['NorthVelocity', 'EastVelocity', 'GroundVelocity']
                                                                    , ['north_vel', 'east_vel', 'ground_vel']))
                    statistics_info_insgps[scene_index]['cal_info'].update(insStatistic.diff_cal(                           # GPS-INS 北、东、地速度差
                                                                          statistics_info_insgps[scene_index]['gps_info']
                                                                        , statistics_info_insgps[scene_index]['ins_info']))
                    statistics_info_insgps[scene_index]['ins_info'].update(insStatistic.speed_neg2frd(statistics_info_insgps[scene_index]['ins_info']
                                                                    , scene['data']['heading']))
                    statistics_info_insgps[scene_index]['cal_info'].update(insStatistic.frd_mileage_cal(                    # 前右下里程计算
                                                                    statistics_info_insgps[scene_index]['ins_info']
                                                                    , scene['data']['gpsItow']))
                    ################################### 姿态 ###################################
                    statistics_info_insgps[scene_index]['ins_info'].update(insStatistic.pos_covert(scene['data']))
                    statistics_info_insgps[scene_index]['gps_info'].update(insStatistic.pos_covert(scene['data']
                                                                    , val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))
                    statistics_info_insgps[scene_index]['cal_info'].update(insStatistic.pos_error_cal(
                                                                              statistics_info_insgps[scene_index]['ins_info']
                                                                            , statistics_info_insgps[scene_index]['gps_info']
                                                                            , yaw=scene['data']['heading']))
                    ################################### 解状态转换 ###################################
                    statistics_info_insgps[scene_index]['cal_info'].update(insStatistic.gps_flags_transfer(scene['data'], flags_pos_name='flagsPos_x'))

                    ################################### 计算差值 ###################################
                    statistics_info_insgps[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data']
                                                                        , df1_val_name=["pitch_x", "heading_x"], df2_val_name=["pitch", "heading"]))
                    ################################### 精度统计 ###################################
                    statistics_info_insgps[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                    for key in statistics_info_insgps[scene_index]['cal_info']:
                        if key not in ['utcTime', 'flags_pos', 'none', 'single', 'pseduo', 'float', 'fixed','forward_mile', 'right_mile', 'downward_mile']:
                            statistics_info_insgps[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info_insgps[scene_index]['cal_info'], key)

                    ################################### 存到对应场景信息中 ###################################
                    statistics_all_scenes_insgps[scene_index].update(statistics_info_insgps[scene_index])

                info_dict = statistics_info_insgps[0]['cal_info'].copy()
                for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime']:
                    info_dict.pop(i)
                self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path+'_gps同步ins.feather'))
                self.ins_test_data[test_data_index]['gps_ins_statistic_info'] = info_dict
                info_dict = None
                statistics_info_insgps = None
            else:
                self.output_msg('无数据，不进行统计。')

            # 3. 输出统计表 #######################################################
            reportObj.reportPath = export_file_path
            self.output_msg("【统计结果输出中】文件路径：" + reportObj.reportPath)
            if len(statistics_all_scenes_ins) != 0:
                self.output_msg(reportObj.statistic2excel(statistics_all_scenes_ins))
            # if len(statistics_all_scenes_gps) != 0:
            #     self.output_msg(reportObj.statistic2excel(statistics_all_scenes_gps, data_type='基准与GPS数据同步统计结果'))
            # if len(statistics_all_scenes_insgps) != 0:
            #     self.output_msg(reportObj.statistic2excel(statistics_all_scenes_insgps, data_type='GPS帧与INS帧同步统计结果'))

            self.output_msg("【统计完成】")

    # 更新消息框内容
    @staticmethod
    def output_msg(msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        print(msg)


if __name__ == "__main__":
    def ref_analysis():
        # 1. 参考数据解析
        objParseRef = ParseRef()
        objParseRef.ref_data = {'test_type': '动态',
                                'static_ref_type': None,
                                'date_time': '2022-05-14',
                                'ref_cal_type': '均值',
                                'file_path': r"E:\Downloads\PP_Truth_Ant12.csv",
                                'file_type': '大众',  # 支持数据类型,  type: 【"NMEA", "100C", "POS320", "大众"】
                                'csv_dict': {},
                                'file_name': '100C_test.txt'}
        objParseRef.run()
        return objParseRef.export_data

    def ins_analysis():
        # 2.  ins测试数据解析
        obj = CompareMainIns()
        obj.ref_data = objMain.ref_data
        obj.test_data = {'algorith_type': 'INS',
                             'date_time': '2022-05-14',
                             'file_path': r"D:\Files\test\dbFiles\test1\test1_LogINS.txt",
                             'file_type': '导远自定义格式',  # 支持数据类型,  type: 【"NMEA", "北云明文", "导远自定义"】
                             'csv_dict': {},
                             'test_type': '动态',
                             'file_name': 'test1_LogINS.txt',
                             'ref_data': '100C_test.txt',
                             'bpox': [0, 0, 0]}
        obj.run()
        return obj.export_data

    def ins_statistics():
        # 3.精度统计流程
        file_path = os.getcwd() + r"\statistics_test.xlsx"
        objMain.scene_list = [{'id': '1', 'name': '场景1', 'time': ['530881', '530945'], 'time_type': 'gps'},
                              {'id': '2', 'name': '场景2', 'time': ['531125', '531336'], 'time_type': 'gps'}]

        objMain.export_ins_statistics(file_path)  # 输出统计


    ################################# TESTING #################################

    objMain = MainWindow()

    objMain.ref_data = ref_analysis()
    objMain.ins_test_data.append(ins_analysis())

    ins_statistics()
