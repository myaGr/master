import os
import time
from MainWindow import dataParse
from Standardize import dataPreProcess
from Utils import gnssStatistics, gnssPlot, insStatistic, insPlot, timeExchangeMgr
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
        self.test_data_all = []  # 参考数据
        self.test_data = None  # 参考数据
        self.export_data = {}

    def run(self):
        self.output_msg("【导入INS测试数据】")
        for test_info in self.test_data_all:
            if 'ref_feather_path' in test_info:
                # 参考数据【已解析】
                # self.ref_data["df"] = pd.read_feather(test_info["ref_feather_path"])
                pass
            else:
                pass

            self.decodeInsData(test_info)
            if test_info['date_time'] != self.test_data['date_time']:
                test_info['date_time'] = self.test_data['date_time']
                print("change date time in test info:"+self.test_data['date_time'])
            self.sendDataframe(test_info=test_info)  # 传递解析数据
        # 释放内存
        self.ref_data = {}
        self.output_msg("【数据导入完毕】\n")

    def decodeInsData(self, test_info):
        self.test_data = test_info.copy()
        # 1. 测试数据解析
        if test_info['file_type'] == 'feather':
            pass
            # for file_name in os.listdir('./' + self.test_data["file_name"]):
            #     if 'df_pdata' in file_name:
            #         self.test_data["df_pdata_path"] = './' + self.test_data["file_name"] + '/' + file_name
            #     elif 'gps_' == file_name[:4]:
            #         self.test_data["df_gps"] = pd.read_feather('./' + self.test_data["file_name"] + '/' + file_name)
            #     elif 'ins_' == file_name[:4]:
            #         self.test_data["df"] = pd.read_feather('./' + self.test_data["file_name"] + '/' + file_name)
        else:
            try:
                self.output_msg("开始解析 ：" + self.test_data["file_path"] + "，数据类型：" + self.test_data["file_type"]
                                + "，输出位置：" + self.test_data["output_pos"] + "， 请等待......")
                self.output_msg("已设置测试日期： " + self.test_data["date_time"])
                insParseOutput = dataParse.allDataParse(self.test_data, self.test_data["date_time"])
                if insParseOutput == -1:
                    self.output_msg("【导入失败】ins帧无解析结果， 无法绘图统计， 请检查数据格式！" + "\n")
                    return
                elif type(insParseOutput) == str:
                    self.output_msg(insParseOutput)
                    if '【注意】gps帧时间为：' in insParseOutput and "df" in self.ref_data.keys():
                        try:
                            self.output_msg('PS. 数据测试时间设置错误，尝试修改基准的Unix时间：')
                            self.ref_data["df"].unixTime = self.ref_data["df"].gpsItow.apply(lambda x: timeExchangeMgr.gps2Unix(self.test_data["df_gps"].gpsWeek[0], x))
                            self.test_data['date_time'] = insParseOutput.split('【注意】gps帧时间为：')[-1].split('与设定时间不符')[0]
                        except Exception as e:
                            self.output_msg('基准的Unix时间修改失败:\n')
                            self.output_msg(e)
                self.output_msg("INS测试数据解析完毕。")
            except Exception as e:
                self.output_msg("【导入失败】测试数据解析失败，失败原因: 数据异常，" + str(e) + "\n")
                return

        # 2. 时间同步
        if "df" in self.test_data.keys() and "df" in self.ref_data.keys():
            try:
                self.output_msg("ins数据开始时间同步， 同步参考数据为:" + str(self.test_data["ref_data"]))
                sync_data = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df"])
                if len(sync_data[sync_data.keys()[0]]) == 0:
                    self.output_msg("【导入失败】没有ins帧时间同步的结果，请检查参考数据对应的测试日期!" + "\n")
                    return
                else:
                    self.test_data["sync_df"] = sync_data
                    self.output_msg("ins帧与参考数据时间同步完成 ")
            except Exception as e:
                self.output_msg("【导入失败】ins帧时间同步失败，失败原因: 数据异常，" + str(e) + "\n")
                return
        else:
            self.output_msg('ref与ins同步中，无ref参考数据同步')

        if "df_gps" in self.test_data.keys() and "df" in self.ref_data.keys():
            try:
                self.output_msg("gps数据开始时间同步， 同步参考数据为:" + str(self.test_data["ref_data"]))
                sync_data_gps = dataPreProcess.timeSynchronize(self.ref_data["df"], self.test_data["df_gps"])
                if len(sync_data_gps[sync_data_gps.keys()[0]]) == 0:
                    self.output_msg("【注意】没有gps帧时间同步的结果，请检查参考数据对应的测试日期!")
                else:
                    self.test_data["sync_df_gps"] = sync_data_gps
                    self.output_msg("gps帧与参考数据时间同步完成 ")
            except Exception as e:
                self.output_msg("gps帧时间同步失败，失败原因: 数据异常，" + str(e))
        else:
            self.output_msg('ref与gps同步中，无ref参考数据同步')

        if "df_gps" in self.test_data.keys() and "df" in self.test_data.keys():
            try:
                self.output_msg("测试数据 ins帧与自身gps帧 开始时间同步")
                sync_data_gpsins = dataPreProcess.timeSynchronize(self.test_data["df_gps"], self.test_data["df"])
                if len(sync_data_gpsins[sync_data_gpsins.keys()[0]]) == 0:
                    self.output_msg("【注意】没有gps帧与ins帧时间同步的结果,请检查参考数据对应的测试日期!")
                else:
                    self.test_data["sync_gps_ins"] = sync_data_gpsins
                    self.output_msg("ins与自身gps时间同步完成")
            except Exception as e:
                self.output_msg("gps帧时间同步失败，失败原因: 数据异常，" + str(e))

    # 更新消息框内容
    @staticmethod
    def output_msg(msg_str):
        msg = time.strftime('%H:%M:%S', time.localtime()) + ' ' + msg_str
        print(msg)

    def sendDataframe(self, test_info=None):
        self.export_data = self.test_data.copy()
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
    # 计算相关参数
    def ins_cal_error_diffs(self, all_scenes_data, data_type='sync_df', lbgc=[0, 0, 0]):
        """

        :param lbgc:  后轮轴中心补偿PData中的lbgc
        :param all_scenes_data: [{'name': '', 'data': {}, 'percent':1}]
        :param data_type:
        :return:
        """
        if len(all_scenes_data) == 0:
            self.output_msg('【导入失败】ins相关数据列表空，无法统计')
            return all_scenes_data
        else:
            statistics_info = [{'base_info': {}, 'test_info': {}, 'cal_info': {}, 'statistic_info': {}} for i in range(len(all_scenes_data))]
            precision_val_names = ['horizontal_error', 'longitudinal_error', 'lateral_error', 'elevation_error'
                                   , 'north_vel_diff', 'east_vel_diff', 'ground_vel_diff', 'forward_vel_diff'
                                   , 'right_vel_diff', 'downward_vel_diff', 'vel_diff'
                                   , 'roll_diff', 'pitch_diff', 'heading_diff']
        for scene in all_scenes_data:
            scene_index = all_scenes_data.index(scene)
            if data_type == 'sync_df':
                self.output_msg("ref和INS同步数据相关信息：")
                if len(scene['data']) == 0:
                    self.output_msg(scene['name'] + ' 无符合的数据，无法统计！')
                    all_scenes_data[scene_index].update(statistics_info[scene_index])
                    continue
                else:
                    self.output_msg(scene['name'] + ' 的ref和INS同步数据统计中...')

                ################################### 速度 ###################################
                # Ref北东地速度转换成前右下坐标系下的速度
                statistics_info[scene_index]['base_info'].update(insStatistic.change_val_name(scene['data']
                                                                                             , ['NorthVelocity_x', 'EastVelocity_x', 'GroundVelocity_x']
                                                                                             , ['north_vel', 'east_vel', 'ground_vel']))
                statistics_info[scene_index]['base_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['base_info'], scene['data']['heading_x']))
                # INS北东地速度转换成前右下坐标系下的速度
                statistics_info[scene_index]['test_info'].update(insStatistic.change_val_name(scene['data']
                                                                                             , ['NorthVelocity', 'EastVelocity', 'GroundVelocity']
                                                                                             , ['north_vel', 'east_vel', 'ground_vel']))
                statistics_info[scene_index]['test_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['test_info'], scene['data']['heading_x']))
                # 前右下里程计算
                statistics_info[scene_index]['test_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['test_info'], scene['data']['gpsItow']))
                statistics_info[scene_index]['base_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['base_info'], scene['data']['gpsItow']))
                # sqrt_root_cal 用于计算（前右向）速度误差
                statistics_info[scene_index]['base_info']['vel'] = insStatistic.sqrt_root_cal(statistics_info[scene_index]['base_info'])
                statistics_info[scene_index]['test_info']['vel'] = insStatistic.sqrt_root_cal(statistics_info[scene_index]['test_info'])
                ################################### 姿态 ###################################
                pos0 = [scene['data']['latitude_x'][0], scene['data']['longitude_x'][0], scene['data']['ellHeight_x'][0]]
                statistics_info[scene_index]['test_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x']))
                statistics_info[scene_index]['base_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x'], val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))
                statistics_info[scene_index]['cal_info'].update(insStatistic.pos_error_cal(statistics_info[scene_index]['test_info'], statistics_info[scene_index]['base_info'], yaw=scene['data']['heading_x']))

                ################################### 计算差值 ###################################
                statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(
                                                                                            statistics_info[scene_index]['base_info'], statistics_info[scene_index]['test_info']
                                                                                            , df1_val_name=["north_vel", "east_vel", "ground_vel", 'forward_vel', 'right_vel', 'downward_vel', 'vel']
                                                                                            , df2_val_name=["north_vel", "east_vel", "ground_vel", 'forward_vel', 'right_vel', 'downward_vel', 'vel']))
                statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data'], df1_val_name=["roll_x", "pitch_x", "heading_x"], df2_val_name=["roll", "pitch", "heading"]))

                ################################### 记录相关值 ###################################
                for i in ["forward_vel", "right_vel", "downward_vel", 'north_vel', 'east_vel', 'ground_vel', "n_axis", "e_axis", "d_axis", "forward_mile", "right_mile", "downward_mile"]:
                    statistics_info[scene_index]['cal_info'][i+'_x'] = statistics_info[scene_index]['base_info'][i]
                    statistics_info[scene_index]['cal_info'][i] = statistics_info[scene_index]['test_info'][i]
                ################################### 精度统计 ###################################
                statistics_info[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                for key in statistics_info[scene_index]['cal_info']:
                    if key in precision_val_names:
                        statistics_info[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info[scene_index]['cal_info'], key, percent=scene['percent'])

            elif data_type == 'sync_df_gps':
                if len(scene['data']) == 0:
                    # self.output_msg(scene['name'] + ' 无符合的数据，无法统计！')
                    all_scenes_data[scene_index].update(statistics_info[scene_index])
                    continue
                else:
                    # self.output_msg(scene['name'] + ' 的ref和GPS同步数据统计中...')
                    pass
                ################################### 速度 ###################################
                statistics_info[scene_index]['base_info'].update(insStatistic.change_val_name(scene['data']
                                                                                              , ['NorthVelocity_x', 'EastVelocity_x', 'GroundVelocity_x']
                                                                                              , ['north_vel', 'east_vel', 'ground_vel']))
                statistics_info[scene_index]['base_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['base_info'], scene['data']['heading_x']))
                statistics_info[scene_index]['test_info'].update(insStatistic.speed_hv2neg(scene['data'], scene['data']['TrackAngle']))
                statistics_info[scene_index]['test_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['test_info'], scene['data']['heading_x']))

                statistics_info[scene_index]['test_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['test_info'], scene['data']['gpsItow']))  # 前右下里程计算
                statistics_info[scene_index]['base_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['base_info'], scene['data']['gpsItow']))  # 前右下里程计算
                ################################### 姿态 ###################################
                pos0 = [scene['data']['latitude_x'][0], scene['data']['longitude_x'][0], scene['data']['ellHeight_x'][0]]
                statistics_info[scene_index]['test_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x']))
                statistics_info[scene_index]['base_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading_x'], val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))
                statistics_info[scene_index]['cal_info'].update(insStatistic.pos_error_cal(statistics_info[scene_index]['test_info'], statistics_info[scene_index]['base_info'], yaw=scene['data']['heading_x']))
                ################################### 解状态转换 ###################################
                statistics_info[scene_index]['cal_info'].update(insStatistic.gps_flags_transfer(scene['data']))

                ################################### 计算差值 ###################################
                statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(statistics_info[scene_index]['base_info'], statistics_info[scene_index]['test_info']))
                statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(scene['data'], scene['data'], df1_val_name=["pitch_x", "heading_x"], df2_val_name=["pitch", "heading"]))
                ################################### 记录相关值 ###################################
                for i in ["forward_vel", "right_vel", "downward_vel", 'north_vel', 'east_vel', 'ground_vel', "n_axis", "e_axis", "d_axis", "forward_mile", "right_mile", "downward_mile"]:
                    statistics_info[scene_index]['cal_info'][i+'_x'] = statistics_info[scene_index]['base_info'][i]
                    statistics_info[scene_index]['cal_info'][i] = statistics_info[scene_index]['test_info'][i]

                ################################### 精度统计 ###################################
                statistics_info[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                for key in statistics_info[scene_index]['cal_info'].keys():
                    if key in precision_val_names:
                        statistics_info[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info[scene_index]['cal_info'], key, percent=scene['percent'])
            elif data_type == 'sync_gps_ins':
                if len(scene['data']) == 0:
                    self.output_msg(scene['name'] + ' 无符合的数据，无法统计！')
                    all_scenes_data[scene_index].update(all_scenes_data[scene_index])
                    continue
                else:
                    self.output_msg(scene['name'] + ' 的GPS和INS同步数据统计中...')
                ################################### 速度 ###################################
                statistics_info[scene_index]['base_info'].update(insStatistic.speed_hv2neg(scene['data']
                                                                                                 , scene['data']['TrackAngle_x'], speed_name=['HSpd_x', 'velocity_x']))
                statistics_info[scene_index]['base_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['base_info'], scene['data']['heading_x']))
                statistics_info[scene_index]['test_info'].update(insStatistic.change_val_name(scene['data']
                                                                                                    , ['NorthVelocity', 'EastVelocity', 'GroundVelocity']
                                                                                                    , ['north_vel', 'east_vel', 'ground_vel']))
                statistics_info[scene_index]['cal_info'].update(insStatistic.diff_cal(statistics_info[scene_index]['base_info'], statistics_info[scene_index]['test_info']))  # GPS-INS 北、东、地速度差
                statistics_info[scene_index]['test_info'].update(insStatistic.speed_neg2frd(statistics_info[scene_index]['test_info'], scene['data']['heading_x']))
                statistics_info[scene_index]['test_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['test_info'], scene['data']['gpsItow']))  # 前右下里程计算
                statistics_info[scene_index]['base_info'].update(insStatistic.frd_mileage_cal(statistics_info[scene_index]['base_info'], scene['data']['gpsItow']))  # 前右下里程计算

                ################################### 姿态 ###################################
                pos0 = [scene['data']['latitude_x'][0], scene['data']['longitude_x'][0], scene['data']['ellHeight_x'][0]]
                statistics_info[scene_index]['test_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading']))
                statistics_info[scene_index]['base_info'].update(insStatistic.pos_covert(scene['data'], pos0=pos0, yaw=scene['data']['heading'], bpos=lbgc, val_name=['latitude_x', 'longitude_x', 'ellHeight_x']))

                statistics_info[scene_index]['cal_info'].update(insStatistic.pos_error_cal(
                    statistics_info[scene_index]['test_info'], statistics_info[scene_index]['base_info'], yaw=scene['data']['heading']))
                ################################### 解状态转换 ###################################
                statistics_info[scene_index]['cal_info'].update(insStatistic.gps_flags_transfer(scene['data'], flags_pos_name='flagsPos_x'))

                ################################### 计算差值 ###################################
                statistics_info[scene_index]['cal_info'].update(
                    insStatistic.diff_cal(scene['data'], scene['data'], df1_val_name=["pitch_x", "heading_x"], df2_val_name=["pitch", "heading"]))
                ################################### 记录对应数据 ###################################
                for i in ["forward_vel", "right_vel", "downward_vel", 'north_vel', 'east_vel', 'ground_vel', "n_axis", "e_axis", "d_axis", "forward_mile", "right_mile", "downward_mile"]:
                    if i in statistics_info[scene_index]['base_info'].keys() and i in statistics_info[scene_index]['test_info'].keys():
                        statistics_info[scene_index]['cal_info'][i+'_x'] = statistics_info[scene_index]['base_info'][i]
                        statistics_info[scene_index]['cal_info'][i] = statistics_info[scene_index]['test_info'][i]

                ################################### 精度统计 ###################################
                statistics_info[scene_index]['cal_info']['utcTime'] = scene['data']['utcTime']
                for key in statistics_info[scene_index]['cal_info']:
                    if key in precision_val_names:
                        statistics_info[scene_index]['statistic_info'][key] = insStatistic.precision_statistics(statistics_info[scene_index]['cal_info'], key, percent=scene['percent'])

            else:
                self.output_msg('INS同步数据中无该类型：' + str(data_type))

            ################################### 存到对应场景信息中 ###################################
            all_scenes_data[scene_index].update(statistics_info[scene_index])

        return all_scenes_data

    def export_ins_statistics(self, export_file_path):

        for test_data_o in self.ins_test_data:
            statistics_all_scenes_ins = []  # 汇总所有数据的场景信息
            statistics_all_scenes_gps = []  # 汇总所有数据的场景信息
            statistics_all_scenes_insgps = []  # 汇总所有数据的场景信息
            # reportObj = insStatistic.insReport()
            # report_path = '.'.join(test_data["file_path"].split(".")[:-1])
            # 处理杆臂值

            test_data = test_data_o.copy()
            if 'posture_bpox' in test_data.keys():
                for df_name in ['sync_df', 'sync_gps_ins']:
                    if df_name in test_data.keys():
                        for val_name in ['roll', 'pitch', 'heading']:
                            val_index = ['roll', 'pitch', 'heading'].index(val_name)
                            if val_name in test_data[df_name].keys():
                                test_data[df_name][val_name] -= test_data['posture_bpox'][val_index]
                            else:
                                continue
            if 'posture_bpox_gps' in test_data.keys():
                if 'sync_df_gps' in test_data.keys():
                    for val_name in ['roll', 'pitch', 'heading']:
                        val_index = ['roll', 'pitch', 'heading'].index(val_name)
                        if val_name in test_data['sync_df_gps'].keys():
                            test_data['sync_df_gps'][val_name] -= test_data['posture_bpox_gps'][val_index]
                        else:
                            continue
                if 'sync_gps_ins' in test_data.keys():
                    for val_name in ['roll', 'pitch', 'heading']:
                        val_index = ['roll', 'pitch', 'heading'].index(val_name)
                        if val_name+'_x' in test_data['sync_gps_ins'].keys():
                            test_data['sync_gps_ins'][val_name+'_x'] -= test_data['posture_bpox_gps'][val_index]
                        else:
                            continue

            # 1. 场景分类 #######################################################
            self.output_msg("开始场景分类:" + test_data["file_name"])
            # 每个场景的信息由{索引name、 数据data、 时间段time 和 占比percent} 组成
            if 'sync_df' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list)
                    statistics_all_scenes_ins.extend(scenes)
                except Exception as e:
                    self.output_msg(
                        "数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            if 'sync_df_gps' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list, classify_data_name='sync_df_gps')
                    statistics_all_scenes_gps.extend(scenes)
                except Exception as e:
                    self.output_msg(
                        "数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            if 'sync_gps_ins' in test_data.keys():
                try:
                    scenes = dataPreProcess.sceneClassify(test_data, self.scene_list, classify_data_name='sync_gps_ins')
                    statistics_all_scenes_insgps.extend(scenes)
                except Exception as e:
                    self.output_msg(
                        "数据" + test_data["file_name"] + "的ins帧数据场景分类失败,失败原因: 数据异常，" + str(e))

            # 2. 计算相关参数, 精度统计 #######################################################
            # 速度计算：位置、姿态、neg xyz速度
            if len(statistics_all_scenes_ins) > 0:
                statistics_all_scenes_ins = self.ins_cal_error_diffs(statistics_all_scenes_ins)
                ### 存储为feather
                # info_dict = statistics_all_scenes_ins[0]['cal_info'].copy()
                # for i in ['utcTime']:
                #     info_dict.pop(i)
                # self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path + '_ref同步ins.feather'))
                info_dict = None
            else:
                self.output_msg('无数据，不进行统计。')

            if len(statistics_all_scenes_gps) > 0:
                statistics_all_scenes_gps = self.ins_cal_error_diffs(statistics_all_scenes_gps, data_type='sync_df_gps')
                # info_dict = statistics_all_scenes_gps[0]['cal_info'].copy()
                # for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime']:
                #     info_dict.pop(i)
                # self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path + '_ref同步gps.feather'))
                info_dict = None
            else:
                self.output_msg('无数据，不进行统计。')

            if len(statistics_all_scenes_insgps) > 0:
                lbgc = [0, 0, 0]
                if test_data["output_pos"] == "后轮轴中心" and 'df_pdata' in test_data.keys():
                    if len(test_data['df_pdata']['Lbgc']) > 0:
                        lbgc = test_data['df_pdata']['Lbgc'][0]

                statistics_all_scenes_insgps = self.ins_cal_error_diffs(statistics_all_scenes_insgps
                                                                        , data_type='sync_gps_ins', lbgc=lbgc)
                # info_dict = statistics_all_scenes_gps[0]['cal_info'].copy()
                # for i in ['none', 'single', 'pseduo', 'float', 'fixed', 'utcTime']:
                #     info_dict.pop(i)
                # self.output_msg(reportObj.info2feather(info_dict, feather_path=report_path + '_gps同步ins.feather'))
                info_dict = None
            else:
                self.output_msg('无数据，不进行统计。')

            # 3. 输出统计表 #######################################################
            reportObj = insStatistic.insReport()
            reportObj.reportPath = export_file_path
            self.output_msg("【统计结果输出中】文件路径：" + reportObj.reportPath)
            if len(statistics_all_scenes_ins) != 0:
                self.output_msg(reportObj.statistic2excel(statistics_all_scenes_ins))
            # if len(statistics_all_scenes_gps) != 0:
            #     self.output_msg(
            #         reportObj.statistic2excel(statistics_all_scenes_gps, data_type='基准与GPS数据同步统计结果'))
            if len(statistics_all_scenes_insgps) != 0:
                self.output_msg(
                    reportObj.statistic2excel(statistics_all_scenes_insgps, data_type='GPS帧与INS帧同步统计结果'))

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
                                'date_time': '2022-08-01',
                                'ref_cal_type': '均值',
                                'file_path': r"D:\Files\test\dbFiles\test1\100C_test.txt",
                                'file_type': '100C',  # 支持数据类型,  type: 【"NMEA", "100C", "POS320", "大众"】
                                'csv_dict': {},
                                'file_name': '320.txt'}
        objParseRef.run()
        return objParseRef.export_data

    def ins_analysis():
        # 2.  ins测试数据解析
        obj = CompareMainIns()
        obj.ref_data = objMain.ref_data
        obj.test_data_all = [
            {'algorith_type': 'INS',
                             'date_time': '2022-08-01',
                             'file_path': r"D:\Files\test\dbFiles\test1\test1_LogINS.txt",  # D:\Files\test\dbFiles\test1\test1_LogINS.txt
                             'file_type': '导远自定义格式',  # 支持数据类型,  type: 【"NMEA", "北云明文", "导远自定义"】
                             'csv_dict': {}, 'test_type': '动态', 'output_pos': '后轮轴中心', 'file_name': 'bddb-test1_LogINS.txt', 'ref_data': '100C_test.txt', #100C_test.txt
                             'bpox': [0, 0, 0], 'posture_bpox': [1, 2, 3], 'posture_bpox_gps': [2, 3, 4]},
            # {'algorithm_type': 'INS', 'bpox': [0, 0, 0], 'csv_dict': {}, 'date_time': '2023-12-28',
            #   'file_path': 'E:\Downloads\Loc_BDDB_8482.log', 'file_type': '导远自定义格式', 'output_pos': '天线位置',
            #   'test_type': '无参考', 'file_name': 'Loc_BDDB_8482.log'}
                             ]
        obj.run()
        return obj.export_data

    def ins_statistics():
        # 3.精度统计流程
        file_path = os.getcwd() + r"\statistics_test.xlsx"
        objMain.scene_list = [{'id': '1', 'name': '场景1', 'time': ['0', '0'], 'time_type': 'gps'},
                              # {'id': '2', 'name': '场景2', 'time': ['531125', '531336'], 'time_type': 'gps'}
                              ]

        objMain.export_ins_statistics(file_path)  # 输出统计
        print('over')


    ################################# TESTING #################################

    objMain = MainWindow()

    objMain.ref_data = ref_analysis()
    objMain.ins_test_data.append(ins_analysis())

    ins_statistics()
