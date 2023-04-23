import time
import numpy as np
import pandas as pd


# 数据同步预处理函数
class DataPreProcess(object):
    def __init__(self):
        self.t = [0, 0]

    # 数据过滤：1.时间筛选 2.未初始化数据  3.无效数据
    def Datafilter(self, df, times):
        """
            数据过滤
            :param df: 原始数据
            :param times: 过滤时间段
            :return: 过滤完成数据
            @author: zixuanwen
        """
        # 过滤指定时间数据
        if self.t[0] > 0:
            df = df[df[times] >= self.t[0]]
        if self.t[1] > 0:
            df = df[df[times] <= self.t[1]]

        # 过滤INS未初始化数据
        # 0x01 02 04 08 分别对应初始化值： P V A H，一般姿态A会先初始化
        if 'IMUstatus' in df.keys():
            df = df[df['IMUstatus'] % 16 == 15]

        # 过滤GPS无效数据
        if 'flagsPos' in df.keys():
            df = df[df['flagsPos'] > 0]

        # 时间跨周补偿
        time_diff_list = np.diff(df[times])
        index_list = df[times][:-1][time_diff_list < 0].index.tolist()
        if index_list:
            for i in range(len(index_list)):
                index = index_list[i]
                if df[times][index] > 604700 and df[times][index + 1] < 100:
                    # 确实是跨周情况
                    print('存在跨周情况！')
                    df[times][index:] = df[times][index:] + 604800
                # else:
                #     print('第%d帧时间有误！' % (index - 1), df[times][index - 1], 's(周内秒)')
                #     df = df[df[times] != df[times][index_list[i]]]
                #     index_list.pop(i)

        # 重置数据索引
        df = df.reset_index(drop=True)
        return df

    # 批量数据线性插值
    def dataInterpolation(self, rawData, itime):
        """
            数据线性插值
            :param rawData: 原始数据
            :param itime: 插值时间
            :return: 插值数据
            @author: zixuanwen
        """
        # 限制插值时间范围不可超过同步数据
        itime = itime[(itime >= rawData['time'][0]) & (itime <= rawData['time'][len(rawData['time']) - 1])]
        itime = itime.reset_index(drop=True)

        # 所有变量统一插值
        interpolation = {"time": itime}
        interp_keys = ['roll', 'pitch', 'yaw', 'AccX', 'AccY', 'AccZ', 'NorthVelocity', 'EastVelocity',
                       'GroundVelocity', 'lat', 'lon', 'height']
        for key in interp_keys:
            interpolation[key] = np.interp(interpolation['time'], rawData['time'], rawData[key])

        # 航向插值异常值处理
        interpolation = self.yawCorrect(rawData, itime, interpolation)

        # 插值数据转Dataframe格式
        interpolationData = pd.DataFrame(interpolation)
        return interpolationData

    # 航向值插值校准
    def yawCorrect(self, rawData, itime, interpolation):
        """
            值插航向数据值校准
            :param rawData: 原始航向数据
            :param itime: 插值时间
            :param interpolation: 未修正航向的插值数据
            :return: 修正航向的插值数据
            @author: zixuanwen
        """
        # 航向插值异常值处理
        yaw_diff_list = np.diff(rawData['yaw'])  # 计算相邻帧航向值变化量
        yaw = np.array(rawData['yaw'])
        times = np.array(rawData['time'])

        # 航向异常情况1：变化量>180
        time1 = [times[np.argwhere(yaw_diff_list > 180)], times[np.argwhere(yaw_diff_list > 180) + 1]]
        yaw1 = [yaw[np.argwhere(yaw_diff_list > 180)], yaw[np.argwhere(yaw_diff_list > 180) + 1]]
        for i in range(len(yaw1[0])):
            its = itime[(itime > time1[0][i][0]) & (itime < time1[1][i][0])]
            its = its.reset_index(drop=True)
            interYaw = (np.interp(its, [time1[0][i][0], time1[1][i][0]],
                                  [yaw1[0][i][0], yaw1[1][i][0] - 360])).tolist()  # 航向角对应插值
            for t in range(len(its)):
                interpolation['yaw'][np.where(interpolation['time'] == its[t])] = interYaw[t]

        # 航向异常情况2：变化量<-180
        time2 = [times[np.argwhere(yaw_diff_list < -180)], times[np.argwhere(yaw_diff_list < -180) + 1]]
        yaw2 = [yaw[np.argwhere(yaw_diff_list < -180)], yaw[np.argwhere(yaw_diff_list < -180) + 1]]
        for i in range(len(yaw2[0])):
            its = itime[(itime > time2[0][i][0]) & (itime < time2[1][i][0])]
            its = its.reset_index(drop=True)
            interYaw = (np.interp(its, [time2[0][i][0], time2[1][i][0]],
                                  [yaw2[0][i][0], yaw2[1][i][0] + 360])).tolist()  # 航向角对应插值
            for t in range(len(its)):
                interpolation['yaw'][np.where(interpolation['time'] == its[t])] = interYaw[t]
        return interpolation

    # 时间同步
    def timeSynchronize(self, syncdata1, syncdata2, time1, time2):
        """
            数据时间同步
            :param syncdata1: 待同步数据1
            :param syncdata2: 待同步数据2
            :param time1: 待同步数据1的时间字段
            :param time2: 待同步数据2的时间字段
            :return: 时间同步数据
            @author: zixuanwen
        """
        syncdata1 = self.Datafilter(syncdata1, time1)  # 过滤
        syncdata2 = self.Datafilter(syncdata2, time2)  # 过滤
        interSyncdata1 = self.dataInterpolation(syncdata1, syncdata2[time2])  # 插值
        # syncdata2['sync_' + time2] = round(syncdata2[time2] * 1000) / 1000 # 整毫秒化
        syncdata2['sync_' + time2] = syncdata2[time2]
        SyncInsGpsData = pd.merge(interSyncdata1, syncdata2, left_on=time1, right_on='sync_' + time2)
        return SyncInsGpsData

    def dm2d(self, dm):
        """
        经纬度 度分格式转度
        :param dm: 输入经纬度值，格式为 dddmm.mmmm
        :return : 输出经纬度值，格式为 ddd.dddddd
        """
        d = None
        if type(dm) == np.ndarray:
            int_d = np.floor(dm / 100)
            point_d = (dm - int_d * 100) / 60
            d = int_d + point_d
        return d

    ########################################## 旧版本 ##########################################
    # 数据线性差值为1hz（时间、位置、速度、姿态）
    def Datainterpolation1(self, RawData):
        interpolation = {"time": [], "yaw": []}
        yaw_diff_list = np.diff(RawData['yaw'])  # 计算相邻帧航向值变化量
        yaw_diff_list[np.where(yaw_diff_list > 180)] = yaw_diff_list[np.where(
            yaw_diff_list > 180)] - 360  # 变化量 >180度 的情况, eg.[-170, 170], - 360校准变化量
        yaw_diff_list[np.where(yaw_diff_list < -180)] = yaw_diff_list[np.where(
            yaw_diff_list < -180)] + 360  # 变化量 < -180度 的情况, eg.[170, -170],  + 360校准变化量
        print("> 180 ", np.argwhere(yaw_diff_list > 180))
        print("< -180", np.argwhere(yaw_diff_list < -180))
        for i in range(len(yaw_diff_list)):
            if RawData['time'][i + 1] - RawData['time'][i] >= 5:  # INS 中断超过5S的数据不予插值计算
                continue
            its = (np.arange(round(RawData['time'][i] * 1000), round(RawData['time'][i + 1] * 1000),
                             1) / 1000).tolist()  # 时间插值为1000hz
            yaw = (np.interp(its, [RawData['time'][i], RawData['time'][i + 1]],
                             [RawData['yaw'][i], RawData['yaw'][i] + yaw_diff_list[i]])).tolist()  # 航向角对应插值
            interpolation["time"].extend(its)
            interpolation["yaw"].extend(yaw)

        interp_keys = ['roll', 'pitch', 'AccX', 'AccY', 'AccZ', 'NorthVelocity', 'EastVelocity', 'GroundVelocity',
                       'lat', 'lon', 'height']
        for key in interp_keys:
            interpolation[key] = (np.interp(interpolation["time"], RawData['time'], RawData[key])).tolist()
        interpolationData = pd.DataFrame(interpolation)
        return interpolationData

    # 时间对齐，求交集
    # 输入参数：同步数据 syncdata1, syncdata2,  同步时间字段，time1, time2
    # @profile  # 内存分析修饰器，添加这句代码，表明对此函数进行内存分析，内存分析结果会打印输出
    def Timesynchronize1(self, syncdata1, syncdata2, time1, time2):
        # 数据过滤
        syncdata1 = self.Datafilter(syncdata1, time1)
        syncdata2 = self.Datafilter(syncdata2, time2)
        syncdata2['sync_' + time2] = round(syncdata2[time2] * 1000) / 1000
        SyncInsGpsData = pd.merge(syncdata1, syncdata2, left_on=time1, right_on='sync_' + time2)

        # print(time.strftime('%H:%M:%S', time.localtime()), "方法2")
        # xy, x_ind, y_ind = np.intersect1d(np.array(syncdata1[time1]), syncdata2['sync_' + time2], return_indices=True)
        # df1 = syncdata1.loc[x_ind.tolist()]
        # df2 = syncdata2.loc[y_ind.tolist()]
        # df1.set_index(time1, inplace=True)
        # df2.set_index('sync_' + time2, inplace=True)
        # t = df1.join(df2, lsuffix='_x', how='outer')
        # print(time.strftime('%H:%M:%S'), '结束')
        # print(t)
        return SyncInsGpsData
