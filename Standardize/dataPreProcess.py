import numpy as np
import pandas as pd
from Utils import timeExchangeMgr
import copy
from itertools import groupby

'''
参考数据与测试数据【时间同步】, 涉及函数包含
1. 数据过滤
2. 批量数据线性插值
3. 航向值插值校准
'''


# 数据过滤：1.时间筛选 2.未初始化数据  3.无效数据
def dataFilter(df, ref=False):
    """
    数据过滤
    :param ref: 是否为参考数据bool
    :param df: 原始数据dataframe
    :return: 过滤完成数据
    @author: zixuanwen
    """
    # 参考数据不作处理
    if ref is False:
        # 测试数据过滤GPS解状态无效数据
        if 'gpsQuality' in df.keys():
            df = df[df['gpsQuality'] > 0]

    # 过滤定位坐标无效数据
    if 'latitude' in df.keys():
        df = df[df['latitude'] > 0]
        df = df[df['longitude'] > 0]

    # 过滤INS未初始化数据
    # 0x01 02 04 08 分别对应初始化值： P V A H，一般姿态A会先初始化
    if 'IMUstatus' in df.keys():
        df = df[df['IMUstatus'] % 16 == 15]

    # 过滤GPS无效数据
    if 'flagsPos' in df.keys():
        df = df[df['flagsPos'] > 0]

    # 重置数据索引
    df = df.reset_index(drop=True)
    return df


# 批量数据线性插值
def dataInterpolation(rawData, itime):
    """
    数据线性插值
    :param rawData: 原始数据
    :param itime: 插值时间
    :return: 插值数据
    @author: zixuanwen
    """
    # 限制插值时间范围不可超过同步数据
    itime = itime[(itime >= rawData['unixTime'][0]) & (itime <= rawData['unixTime'][len(rawData['unixTime']) - 1])]
    itime = itime.reset_index(drop=True)
    # 所有变量统一插值
    interpolation = {"unixTime": itime}
    interp_keys = ['latitude', 'longitude', 'altitude', 'ellHeight', 'HSpd', 'velocity', 'heading', 'TrackAngle',
                   'roll', 'pitch', 'AccX', 'AccY', 'AccZ', 'NorthVelocity', 'EastVelocity', 'GroundVelocity', 'flagsPos']  #
    for key in interp_keys:
        if key in rawData.keys():
            interpolation[key + "_x"] = np.interp(interpolation['unixTime'], rawData['unixTime'], rawData[key])

    # 航向插值异常值处理
    if 'heading_x' in interpolation.keys():
        interpolation = yawCorrect(rawData, itime, interpolation)
    # # 解状态插值异常值处理
    # if 'flagsPos_x' in interpolation.keys():
    #     interpolation = flagsPosFix(rawData, itime, interpolation)

    # 插值数据转Dataframe格式
    interpolationData = pd.DataFrame(interpolation)

    # 间隔时间大于【1秒】的异常数据处理
    time_diff_list = np.diff(rawData['unixTime'])
    times = np.array(rawData['unixTime'])
    time_list = [times[np.argwhere(time_diff_list > 1)], times[np.argwhere(time_diff_list > 1) + 1]]  # 间隔时长超过1秒的时间列表
    for i in range(len(time_list[0])):
        interpolationData = interpolationData[(interpolationData['unixTime'] < time_list[0][i][0]) |
                                              (interpolationData['unixTime'] > time_list[1][i][0])]
    interpolationData = interpolationData.reset_index(drop=True)  # 重置数据索引

    return interpolationData


# 航向值插值校准
def yawCorrect(rawData, itime, interpolation):
    """
    值插航向数据值校准
    :param rawData: 原始航向数据
    :param itime: 插值时间
    :param interpolation: 未修正航向的插值数据
    :return: 修正航向的插值数据
    @author: zixuanwen
    """
    # 航向插值异常值处理
    yaw_diff_list = np.diff(rawData['heading'])  # 计算相邻帧航向值变化量
    yaw = np.array(rawData['heading'])
    times = np.array(rawData['unixTime'])

    # 航向异常情况1：变化量>180
    time1 = [times[np.argwhere(yaw_diff_list > 180)], times[np.argwhere(yaw_diff_list > 180) + 1]]
    yaw1 = [yaw[np.argwhere(yaw_diff_list > 180)], yaw[np.argwhere(yaw_diff_list > 180) + 1]]
    for i in range(len(yaw1[0])):
        its = itime[(itime > time1[0][i][0]) & (itime < time1[1][i][0])]
        its = its.reset_index(drop=True)
        interYaw = (np.interp(its, [time1[0][i][0], time1[1][i][0]], [yaw1[0][i][0], yaw1[1][i][0] - 360])).tolist()  # 航向角对应插值
        for t in range(len(its)):
            interpolation['heading_x'][np.where(interpolation['unixTime'] == its[t])] = interYaw[t]

    # 航向异常情况2：变化量<-180
    time2 = [times[np.argwhere(yaw_diff_list < -180)], times[np.argwhere(yaw_diff_list < -180) + 1]]
    yaw2 = [yaw[np.argwhere(yaw_diff_list < -180)], yaw[np.argwhere(yaw_diff_list < -180) + 1]]
    for i in range(len(yaw2[0])):
        its = itime[(itime > time2[0][i][0]) & (itime < time2[1][i][0])]
        its = its.reset_index(drop=True)
        interYaw = (np.interp(its, [time2[0][i][0], time2[1][i][0]], [yaw2[0][i][0], yaw2[1][i][0] + 360])).tolist()  # 航向角对应插值
        for t in range(len(its)):
            interpolation['heading_x'][np.where(interpolation['unixTime'] == its[t])] = interYaw[t]
    return interpolation


# 【动态】数据时间同步
def timeSynchronize(syncdata1, syncdata2):
    """
    数据时间同步
    :param syncdata1: 待同步数据1
    :param syncdata2: 待同步数据2
    :return: 时间同步数据
    @author: zixuanwen
    """
    syncdata1 = dataFilter(syncdata1, ref=True)  # 过滤参考
    syncdata2 = dataFilter(syncdata2)  # 过滤测试
    interSyncdata1 = dataInterpolation(syncdata1, syncdata2["unixTime"])  # 插值
    SyncInsGpsData = pd.merge(interSyncdata1, syncdata2, left_on="unixTime", right_on="unixTime")  # 合并
    return SyncInsGpsData


# 【静态】数据时间同步
def nullRefDataFun(ref, testdata):
    """
    静态数据时间同步
    :param testdata: 测试数据dataframe
    :param ref: 参考数据dataframe
    :return: 合并后场景数据dataframe
    """
    testdata = dataFilter(testdata)  # 过滤
    lenArr = len(testdata['unixTime'])
    SyncInsGpsData = testdata
    SyncInsGpsData['latitude_x'] = [ref['latitude'] for x in range(lenArr)]
    SyncInsGpsData['longitude_x'] = [ref['longitude'] for x in range(lenArr)]
    SyncInsGpsData['ellHeight_x'] = [ref['ellHeight'] for x in range(lenArr)]
    return SyncInsGpsData


'''
同步数据【场景分类】 涉及函数包含：
1. 根据解状态切割数据
2. 根据时间段切割数据
3. 合并相同id场景数据
'''


# 根据解状态切割数据
def scenesDifferByGpsQuality(SyncRefGpsData, scene_name):
    """
    根据解状态切割数据
    :param SyncRefGpsData: 全场景数据dataframe
    :return: 根据解状态切割数据{场景名称，切割数据}
    @author: zixuanwen
    """
    # 解状态 = {"all": "所有解", "fixed": "固定解", "float": "浮点解", "pseduo": "伪距解", "single": "单点解"}
    SyncInsGpsScene = [{"name": scene_name.replace("all", "single"), "data": SyncRefGpsData[SyncRefGpsData['gpsQuality'] == 1].copy()},
                       {"name": scene_name.replace("all", "pseduo"), "data": SyncRefGpsData[SyncRefGpsData['gpsQuality'] == 2].copy()},
                       {"name": scene_name.replace("all", "float"), "data": SyncRefGpsData[SyncRefGpsData['gpsQuality'] == 5].copy()},
                       {"name": scene_name.replace("all", "fixed"), "data": SyncRefGpsData[SyncRefGpsData['gpsQuality'] == 4].copy()}]
    for i in range(len(SyncInsGpsScene)):
        SyncInsGpsScene[i]["data"] = SyncInsGpsScene[i]["data"].reset_index(drop=True)
        # 计算每种解状态在场景内的占比
        SyncInsGpsScene[i]["percent"] = len(SyncInsGpsScene[i]["data"][SyncInsGpsScene[i]["data"].keys()[0]]) / len(SyncRefGpsData[SyncRefGpsData.keys()[0]])
    return SyncInsGpsScene


# 根据时间段切割数据
def scenesDifferByTime(sync_df, add_scenes, file_name):
    """
    根据时间段切割数据
    :param sync_df: 全场景数据dataframe
    :param add_scenes: 切割场景信息dic{场景名称，时间段}
    :param file_name: 数据文件名
    :return: 切割场景信息dic{场景名称，时间段，切割数据}
    @author: zixuanwen
    """
    utcDate = sync_df["utcDate"][0]  # 获取UTC日期
    start = sync_df['unixTime'][0]   # 初始化开始时间
    end = sync_df['unixTime'][len(sync_df['unixTime']) - 1]   # 初始化结束时间
    scenes = copy.deepcopy(add_scenes)  # DF深拷贝

    for scene in scenes:
        scene["name"] = scene["id"] + "_" + scene["name"] + "_all_" + file_name
        # 当时间类型为GPS周内秒
        if scene['time_type'] == 'gps':
            gpsWeek = timeExchangeMgr.date2Gpsweek(str(utcDate))
            if float(scene['time'][0]) != 0:
                start = timeExchangeMgr.gps2Unix(gpsWeek, float(scene['time'][0]))
            if float(scene['time'][1]) != 0:
                end = timeExchangeMgr.gps2Unix(gpsWeek, float(scene['time'][1]))
        # 当时间类型为UTC时间
        if scene['time_type'] == 'utc':
            if float(scene['time'][0]) != 0:
                start_utc_str = str(utcDate) + " " + scene['time'][0][:2] + ":" + scene['time'][0][2:4] + ":" + scene['time'][0][4:]
                start = timeExchangeMgr.utc2Unix(start_utc_str)
            if float(scene['time'][1]) != 0:
                end_utc_str = str(utcDate) + " " + scene['time'][1][:2] + ":" + scene['time'][1][2:4] + ":" + scene['time'][1][4:]
                end = timeExchangeMgr.utc2Unix(end_utc_str)

        # 根据指定时间段筛选
        scene['data'] = sync_df[(sync_df['unixTime'] >= start) & (sync_df['unixTime'] <= end)].copy()
        scene['data'] = scene['data'].reset_index(drop=True)

        # 计算场景内对应解状态的占比，所有解是100%
        if len(sync_df[sync_df.keys()[0]]) > 0:
            scene['percent'] = 1  # len(scene['data'][scene['data'].keys()[0]])/len(sync_df[sync_df.keys()[0]])
    scenes = merge_scenes(scenes)  # 同类场景数据合并
    return scenes


# 合并场景索引中id相同的data
def merge_scenes(scenes):
    """
    合并name中id相同的data
    :param scenes: 全场景数据dict
    :return: 合并后场景数据dict
    """
    temp = []
    for scene in scenes:
        scene['id'] = scene['name'].split('_')[0]  # 提取name中，‘_’为分割的id
    user_sort = sorted(scenes, key=lambda x: (x['id']))  # 多字段分组
    scenes = groupby(user_sort, key=lambda x: (x['id']))  # 多字段分组
    for key, group in scenes:
        group = [doc for doc in group]
        scene = group[0]
        for i in range(1, len(list(group))):
            scene['data'] = pd.concat(objs=[scene['data'], group[i]['data']])  # dataframe合并
            scene['data'] = scene['data'].reset_index(drop=True)  # dataframe索引重置
            scene['percent'] = scene['percent']
            scene['time'] = scene['time'] + group[i]['time']  # 时间合并
        temp.append(scene)
    return temp


# 场景分类
def sceneClassify(test_data, scene_list, classify_data_name="sync_df"):
    """
    场景分类
    :param classify_data_name: 一般为 "sync_df"；'sync_df_gps'用于 ins的GPS数据
    :param test_data: 待分类数据dict，包含{"sync_df"：待分类数据dataframe，"file_name"：待分类数据名}}
    :param scene_list: 分类场景列表， 每个场景包含{场景id, 场景名称、场景时间段，场景时间类型}
    return：scenes_data 分类后的场景数据列表
    """
    # 索引name决定了统计表输出场景的顺序, GNSS数据需要以（1）场景id、（2）场景解状态、（3）数据名3个字段为序输出
    # 解状态 gps_flag = {"all": "所有解", "fixed": "固定解", "float": "浮点解", "pseduo": "伪距解", "single": "单点解"}
    scenes_data = [{"name": "0_全程_all_" + test_data["file_name"],
                    "data": test_data[classify_data_name], "time": [0, 0], "percent": 1}]
    # 1. 根据时间段场景分类
    scenes_data.extend(scenesDifferByTime(test_data[classify_data_name], scene_list, test_data["file_name"]))

    # 2. 根据解状态场景二次分类
    num_of_scene = range(len(scenes_data))
    for n in num_of_scene:
        if 'gpsQuality' in scenes_data[n]["data"]:
            if scenes_data[n]["data"].empty:  # 跳过场景分类后无数据的情况
                print("【提示】场景：" + scenes_data[n]["name"] + ", 无数据")
                continue
            scenes_data.extend(scenesDifferByGpsQuality(
                scenes_data[n]["data"], scenes_data[n]["name"]))
    return scenes_data
