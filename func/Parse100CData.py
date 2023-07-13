import pynmea2
import pandas as pd
import os
import numpy as np
import sys


class Parse100CData(object):
    def __init__(self):
        self.filepath = ""
        self.ins100cdf = {}

    # 100C数据保存为Dataframe
    def save100Ctodf(self):
        msg_info = ''
        f = open(self.filepath, 'r')
        lines = f.readlines()
        startflag = 0
        keys_name = ['GPSTime', 'Latitude', 'Longitude', 'H-MSL', 'Heading', 'Pitch', 'Roll', 'Q', 'VNorth', 'VEast',
                     'VUp', 'H-Ell', 'AccBiasX', 'GyroDriftX', 'AccBiasY', 'GyroDriftY', 'AccBiasZ', 'GyroDriftZ']
        keys = ["time", "lat", "lon", "height", "yaw", "pitch", "roll", "Q", "NorthVelocity", "EastVelocity",
                "GroundVelocity", "H-Ell", "AccX", "GyroX", "AccY", "GyroY", "AccZ", "GyroZ"]
        values = {}
        for line in lines:
            li = line.replace('\n', '').split(" ")
            while '' in li:
                li.remove('')
            # print(len(li),li)
            if "GPSTime" in li and len(li) >= 18:  # 寻找100C数据字头
                diff_keys = set(keys_name) - set(li)
                if len(diff_keys) > 0:  # 判断是否缺少字段， 如果缺少直接返回
                    print('missing values:', diff_keys)
                    return diff_keys
                startflag = 1
                for i in li:
                    if i in keys_name:
                        values[keys[keys_name.index(i)]] = []

            if startflag == 1 and len(li) >= 22 and li[0] != '(sec)':
                values[keys[0]].append(float(li[0]))
                values[keys[1]].append(float(li[1]) + float(li[2]) / 60 + float(li[3]) / 3600)
                values[keys[2]].append(float(li[4]) + float(li[5]) / 60 + float(li[6]) / 3600)
                for k in range(3, len(keys)):
                    if keys[k] == 'height':
                        try:
                            values[keys[k]].append(float(li[k + 12]))
                            if msg_info:
                                msg_info += '基准数据高程误差计算采用正高值（即H-MSL)'
                        except Exception as e:
                            print(e)
                            values[keys[k]].append(float(li[k + 4]))
                            if msg_info:
                                msg_info += '基准数据H-MSL有误，高程误差计算采用椭球高（即H-ELL)'

                    else:
                        values[keys[k]].append(float(li[k + 4]))
        f.close()
        values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
        self.ins100cdf = pd.DataFrame(values)
        return msg_info

    # 100C数据保存为.csv数据
    def save100Ctocsv(self):
        if len(self.ins100cdf) == 0:
            return
        dir_path = os.path.split(self.filepath)
        new_filepath = dir_path[0] + '/' + dir_path[1][:-4] + '_CsvData.csv'
        self.ins100cdf.to_csv(new_filepath)
        print(self.ins100cdf)

    # 100C数据保存为Dataframe
    def save_320_to_df_1(self):
        """
        另外一种320格式
        @author: liqianwen
        @version: for V2.0.4 和后期重构
        @date: 2023-02-01
        """
        msg_info = ''
        f = open(self.filepath, 'r')
        lines = f.readlines()
        startflag = 0
        GroundVelocity = False
        # 320内应含有的字段
        keys_name = ['GPSTime', 'Latitude', 'Longitude', 'H-Ell', 'Heading', 'Pitch', 'Roll', 'posQuality',
                 'Vel-N', 'Vel-E', 'Vel-U', 'AcclX', 'AngVelX', 'AcclY', 'AngVelY', 'AcclZ', 'AngVelZ']
        required_keys = ["time", "lat", "lon", "H-Ell", "yaw", "pitch", "roll", "Q", "NorthVelocity", "EastVelocity",
                         "GroundVelocity", "AccX", "GyroX", "AccY", "GyroY", "AccZ", "GyroZ", "height"]
        values = {}
        for line in lines:
            li = line.replace('\n', '').split(" ")
            while '' in li:
                li.remove('')

            if "GPSTime" in li and len(li) >= 17:  # 寻找320数据字头，共10列
                diff_keys = set(keys_name) - set(li)

                # 兼容不同格式的垂直速度: 地向速度转天向速度
                if diff_keys == {'Vel-U'} and "Vel-D" in line:  # 地向速度转天向速度
                    GroundVelocity = True
                    line = line.replace("Vel-D", "Vel-U")
                    li = line.replace('\n', '').split(" ")
                    while '' in li:
                        li.remove('')
                    diff_keys = set(keys_name) - set(li)

                # 判断是否缺少字段， 如果缺少直接返回
                if len(diff_keys) > 0:
                    print('missing values:', diff_keys)
                    return diff_keys
                startflag = 1
                for i in li:
                    if i in keys_name:
                        values[required_keys[keys_name.index(i)]] = []
                continue

            if startflag == 1 and len(li) >= 17 and li[0] != '(sec)':
                for k in range(len(required_keys)-1):
                    values[required_keys[k]].append(float(li[k]))

        f.close()
        if "H-Ell" in values:
            values['height'] = values['H-Ell']
            msg_info += '采用字段为 H-Ell 的数据作为高程计算'
        if "GroundVelocity" in values and GroundVelocity is False:
            values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
        self.ins100cdf = pd.DataFrame(values)
        return msg_info

    def save_320_to_df(self):
        """320 文件数据向100C对齐，生成相同结构的数据结果
        @author: liqianwen
        @version: for V2.0.1
        """
        msg_info = ''
        f = open(self.filepath, 'r')
        lines = f.readlines()
        startflag = 0
        # 320内应含有的字段
        keys_name = ['GPSTime', 'Latitude', 'Longitude', 'H-MSL', 'Heading', 'Pitch', 'Roll', 'posQuality'
                    , 'Vel-N', 'Vel-E', 'Vel-U', 'H-Ell', 'Abx', 'Gbx', 'Aby', 'Gby', 'Abz', 'Gbz']
        required_keys = ["time", "lat", "lon", "height", "yaw", "pitch", "roll", "Q", "NorthVelocity", "EastVelocity",
                         "GroundVelocity", "H-Ell", "AccX", "GyroX", "AccY", "GyroY", "AccZ", "GyroZ"]
        values = {}
        for line in lines:
            li = line.replace('\n', '').split(" ")
            while '' in li:
                li.remove('')
            # print(len(li),li)
            if "GPSTime" in li and len(li) >= 18:  # 寻找100C数据字头
                diff_keys = set(keys_name) - set(li)

                # 兼容不同格式的正高
                if diff_keys == {'H-MSL'} and 'H-O' in line:
                    line = line.replace("H-O", "H-MSL")
                    li = line.replace('\n', '').split(" ")
                    while '' in li:
                        li.remove('')
                    diff_keys = set(keys_name) - set(li)
                elif diff_keys == {'H-MSL'} and 'H-N' in line:
                    line = line.replace("H-N", "H-MSL")
                    li = line.replace('\n', '').split(" ")
                    while '' in li:
                        li.remove('')
                    diff_keys = set(keys_name) - set(li)

                # 判断是否缺少字段， 如果缺少直接返回
                if len(diff_keys) > 0:
                    print('missing values:', diff_keys)
                    return diff_keys
                startflag = 1
                for i in li:
                    if i in keys_name:
                        values[required_keys[keys_name.index(i)]] = []
                continue

            if startflag == 1 and len(li) >= 22 and li[0] != '(sec)':
                values[required_keys[0]].append(float(li[0]))
                values[required_keys[1]].append(float(li[1]) + float(li[2]) / 60 + float(li[3]) / 3600)
                values[required_keys[2]].append(float(li[4]) + float(li[5]) / 60 + float(li[6]) / 3600)
                for k in range(3, len(required_keys)):
                    values[required_keys[k]].append(float(li[k + 4]))
        f.close()

        if "H-Ell" in values:
            values['height'] = values['H-Ell']
            msg_info += '采用字段为 H-Ell 的数据作为高程计算'
        if "GroundVelocity" in values:
            values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度

        self.ins100cdf = pd.DataFrame(values)
        return msg_info

    # 100C数据保存为.csv数据
    def save_320_to_csv(self):
        """for 320
        @author: liqianwen
        """
        if len(self.ins100cdf) == 0:
            return
        dir_path = os.path.split(self.filepath)
        new_filepath = dir_path[0] + '/' + dir_path[1][:-4] + '_CsvData.csv'
        self.ins100cdf.to_csv(new_filepath)
        print(self.ins100cdf)

    def save_lzq_to_df(self):
        """
        2023/06/19刘志强版本 文件数据向100C对齐，生成相同结构的数据结果
        @author: liqianwen
        @version: for V2.0.6.2
        """
        msg_info = ''
        begin = 0
        f = open(self.filepath, 'r')
        lines = f.readlines()
        # 320内应含有的字段
        keys_name = ['GPSWeekTime','GPSTime', 'Latitude', 'Longitude', 'H-Ell', 'posQuality'
                    , 'satnavsNum', 'deviation-E', 'deviation-N', 'deviation-H', 'CEN', 'CEH', 'CNH', 'PDOP', 'staticStatus'
                    , 'Vel-E', 'Vel-N', 'Vel-U', 'Roll', 'Pitch', 'Heading']
        required_keys = ['GPSWeekTime', "time", "lat", "lon", "height", "Q"
                        , 'satnavsNum', 'deviation-E', 'deviation-N', 'deviation-H', 'CEN', 'CEH', 'CNH', 'PDOP', 'staticStatus'
                        , "EastVelocity", "NorthVelocity","GroundVelocity", "roll", "pitch", "yaw"
                        # , "H-Ell", "AccX", "GyroX", "AccY", "GyroY", "AccZ", "GyroZ"
                         ]
        values = {}
        for line in lines:
            li = line.replace('\n', '').split(" ")
            while '' in li:
                li.remove('')

            # 该数据类型直接开始数据
            if begin == 0:
                begin = 1
                for key in keys_name:
                    values[required_keys[keys_name.index(key)]] = []

            if begin == 1 and len(li) == 21:
                for k in range(len(required_keys)):
                    try:
                        values[required_keys[k]].append(float(li[k]))
                    except:
                        values[required_keys[k]].append(li[k])

        f.close()
        values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度

        self.ins100cdf = pd.DataFrame(values)
        return msg_info

    def beiYunDataToDataframe(self, filepath):
        """
        beiYunDataToDataframe：数据格式转成pandas数据
        ：param filepath: 北云原始数据 INSPVAA
        ：return：北云数据的dataframe
        @auther：ZixuanWen
        @date：2023-07-12
       """
        keys_name = ['gpsWeek', 'time', 'lat', 'lon', 'height', 'NorthVelocity', 'EastVelocity', 'GroundVelocity',
                     'roll', 'pitch', 'yaw']
        values = {}
        for key in keys_name:
            values[key] = []
        # with判断文件是否存在
        with open(filepath, 'r', encoding='gb18030', errors='ignore') as f:
            lines = f.readlines()
            for line in lines:
                try:
                    if line == '' or 'BKGGA' in line or "\x00" in line:  # 过滤包含乱码情况
                        continue
                    if "#INSPVAA" in line and len(line.split(",")) == 21 and len(
                            line.split(";")[1].split(",")) == 12:  # 从$开启截取信息
                        tmp = line.rstrip('\n').split(";")[1].split(",")
                        for i in range(len(keys_name)):
                            values[keys_name[i]].append(float(tmp[i]))
                except Exception as e:
                    print(line + ", 数据解析失败" + str(e))
        f.close()
        values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
        return pd.DataFrame(values)

    def save_nmea_to_df(self):
        pass


if __name__ == "__main__":
    obj = Parse100CData()
    obj.filepath = r"D:\Files\test\dbFiles\test1\100C_test.txt"
    obj.save100Ctodf()
    obj.save100Ctocsv()
