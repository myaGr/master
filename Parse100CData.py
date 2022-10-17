import pandas as pd
import os
import numpy as np
import sys

class Parse100CData(object):
    def __init__(self):
        self.filepath = ""
        self.ins100cdf = {}

    #100C数据保存为Dataframe
    def save100Ctodf(self):
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
            if "GPSTime" in li and len(li) >= 18:  #寻找100C数据字头
                diff_keys = set(keys_name)-set(li)
                if len(diff_keys) > 0:  #判断是否缺少字段， 如果缺少直接返回
                    print('missing values:', diff_keys)
                    return diff_keys
                startflag = 1
                for i in li:
                    if i in keys_name:
                        values[keys[keys_name.index(i)]] = []

            if startflag == 1 and len(li) >= 22 and li[0] != '(sec)':
                values[keys[0]].append(float(li[0]))
                values[keys[1]].append(float(li[1]) + float(li[2])/60 + float(li[3])/3600)
                values[keys[2]].append(float(li[4]) + float(li[5])/60 + float(li[6])/3600)
                for k in range(3, len(keys)):
                    values[keys[k]].append(float(li[k+4]))
        f.close()
        values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
        self.ins100cdf = pd.DataFrame(values)

    # 100C数据保存为.csv数据
    def save100Ctocsv(self):
        if len(self.ins100cdf) == 0:
            return
        dir = os.path.split(self.filepath)
        new_filepath = dir[0] + '/' + dir[1][:-4] + '_CsvData.csv'
        self.ins100cdf.to_csv(new_filepath)
        print(self.ins100cdf)


if __name__ == "__main__":
    obj = Parse100CData()
    obj.filepath = "C:/Users/wenzixuan/Downloads/ls_data/hzb1.txt"
    obj.save100Ctodf()
    obj.save100Ctocsv()
