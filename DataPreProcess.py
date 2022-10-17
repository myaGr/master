import time
import numpy as np
import pandas as pd


# 数据同步预处理函数
class DataPreProcess(object):
    def __init__(self):
        self.t = [0, 0]

    # 数据线性差值为1hz（时间、位置、速度、姿态）
    def Datainterpolation(self, RawData):
        interpolation = {"time": [], "yaw": []}
        yaw_diff_list = np.diff(RawData['yaw'])
        yaw_diff_list[np.where(yaw_diff_list > 180)] = yaw_diff_list[np.where(yaw_diff_list > 180)] - 360
        yaw_diff_list[np.where(yaw_diff_list < -180)] = yaw_diff_list[np.where(yaw_diff_list < -180)] + 360
        for i in range(len(yaw_diff_list)):
            if RawData['time'][i + 1] - RawData['time'][i] >= 5:  #INS 中断超过5S的数据不予插值计算
                continue
            its = (np.arange(round(RawData['time'][i] * 1000), round(RawData['time'][i + 1] * 1000), 1) / 1000).tolist()
            yaw = (np.interp(its, [RawData['time'][i], RawData['time'][i + 1]], [RawData['yaw'][i], RawData['yaw'][i] + yaw_diff_list[i]])).tolist()
            interpolation["time"].extend(its)
            interpolation["yaw"].extend(yaw)

        interp_keys = ['roll', 'pitch', 'AccX', 'AccY', 'AccZ', 'NorthVelocity', 'EastVelocity', 'GroundVelocity',
                       'lat', 'lon', 'height']
        for key in interp_keys:
            interpolation[key] = (np.interp(interpolation["time"], RawData['time'], RawData[key])).tolist()
        interpolationData = pd.DataFrame(interpolation)
        return interpolationData

    # 数据过滤：1.时间筛选 2.未初始化数据  3.无效数据
    def Datafilter(self, df, times):
        # 过滤指定时间数据
        if self.t[0] > 0:
            df = df[df[times] >= self.t[0]]
        if self.t[1] > 0:
            df = df[df[times] <= self.t[1]]

        # 过滤INS未初始化数据
        if 'IMUstatus' in df.keys():
            df = df[df['IMUstatus'] % 16 == 15]

        # 过滤GPS无效数据
        if 'flagsPos' in df.keys():
            df = df[df['flagsPos'] > 0]

        # 重置数据索引
        df = df.reset_index(drop=True)
        return df


    # 时间对齐，求交集
    # 输入参数：同步数据 syncdata1, syncdata2,  同步时间字段，time1, time2
    # @profile  # 内存分析修饰器，添加这句代码，表明对此函数进行内存分析，内存分析结果会打印输出
    def Timesynchronize(self, syncdata1, syncdata2, time1, time2):
        # 数据过滤
        # print(syncdata1.keys())
        syncdata1 = self.Datafilter(syncdata1, time1)
        syncdata2 = self.Datafilter(syncdata2, time2)
        syncdata2['sync_' + time2] = round(syncdata2[time2] * 1000) / 1000

        # print(time.strftime('%H:%M:%S', time.localtime()), "方法1")
        SyncInsGpsData = pd.merge(syncdata1, syncdata2, left_on=time1, right_on='sync_' + time2)
        # print(time.strftime('%H:%M:%S'), '结束')

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
