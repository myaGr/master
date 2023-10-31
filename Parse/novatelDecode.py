import pandas as pd
import numpy as np


# 100C数据格式转pandas数据
def novatelToDataframe(filepath, fre=None):
    """
    novatelToDataframe：数据格式转成pandas数据
    ：param filepath:100c文件路径
    ：return：100c数据的dataframe
    @auther：ZixuanWen
    @date：2022-11-11
    """
    frq_time = 0
    start_flag = 0
    # with判断文件是否存在
    with open(filepath, 'r', encoding='gb18030', errors='ignore') as f:
        lines = f.readlines()  # 读取list
    keys_name = ['GPSTime', 'Latitude', 'Longitude', 'H-MSL', 'Heading', 'Pitch', 'Roll', 'Q', 'VNorth', 'VEast',
                 'VUp', 'H-Ell', 'AccBiasX', 'GyroDriftX', 'AccBiasY', 'GyroDriftY', 'AccBiasZ', 'GyroDriftZ']
    keys = ["time", "lat", "lon", "height", "yaw", "pitch", "roll", "Q", "NorthVelocity", "EastVelocity",
            "GroundVelocity", "H-Ell", "AccX", "GyroX", "AccY", "GyroY", "AccZ", "GyroZ"]
    values = {}
    for line in lines:
        li = line.replace('\n', '').split(" ")
        while '' in li:
            li.remove('')
        if "GPSTime" in li and len(li) >= 18:  # 寻找100C数据字头
            diff_keys = set(keys_name) - set(li)
            if len(diff_keys) > 0:  # 判断是否缺少字段， 如果缺少直接返回
                print('missing values:', diff_keys)
                return diff_keys
            start_flag = 1
            for i in li:
                if i in keys_name:
                    values[keys[keys_name.index(i)]] = []
        if start_flag == 1 and len(li) >= 22 and li[0] != '(sec)':

            if fre:  # 降频处理
                if int(float(li[0]) * fre) == int(frq_time * fre):
                    continue
            frq_time = float(li[0])

            values[keys[0]].append(float(li[0]))
            values[keys[1]].append(float(li[1]) + float(li[2]) / 60 + float(li[3]) / 3600)
            values[keys[2]].append(float(li[4]) + float(li[5]) / 60 + float(li[6]) / 3600)
            for k in range(3, len(keys)):
                values[keys[k]].append(float(li[k + 4]))
    f.close()
    if "GroundVelocity" in values:
        values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
    return pd.DataFrame(values)


# POS320数据格式转pandas数据
def pos320ToDataframe(filepath, fre=None):
    """
    pos320ToDataframe：数据格式转成pandas数据
    ：param filepath:pos320文件路径
    ：return：pos320数据的dataframe
    @auther：ZixuanWen
    @date：2022-11-11

    """
    frq_time = 0
    startFlag = 0
    # with判断文件是否存在
    with open(filepath, 'r', encoding='gb18030', errors='ignore') as f:
        lines = f.readlines()  # 读取list
    """
    -      GPSTime      = GPS周内秒                         
    -      Latitude     = 纬度                             
    -      Longitude    = 经度                             
    -      H-MSL        = 正高                             
    -      Heading      = 航向                             
    -      Pitch        = 俯仰                             
    -      Roll         = 横滚                             
    -      posQuality   = 组合导航质量                         
    -      Vel-N        = 北向速度                           
    -      Vel-E        = 东向速度                           
    -      Vel-U        = 天向速度                           
    -      H-Ell        = 椭球高                            
    -      Abx          = 加表零偏 X                         
    -      Gbx          = 陀螺零偏 X                         
    -      Aby          = 加表零偏 Y                         
    -      Gby          = 陀螺零偏 Y                         
    -      Abz          = 加表零偏 Z                         
    -      Gbz          = 陀螺零偏 Z  
    """
    keys_name = ['GPSTime', 'Latitude', 'Longitude', 'H-MSL', 'Heading', 'Pitch', 'Roll', 'posQuality',
                 'Vel-N', 'Vel-E', 'Vel-U', 'H-Ell', 'Abx', 'Gbx', 'Aby', 'Gby', 'Abz', 'Gbz']
    keys = ["time", "lat", "lon", "height", "yaw", "pitch", "roll", "Q", "NorthVelocity", "EastVelocity",
            "GroundVelocity", "H-Ell", "AccX", "GyroX", "AccY", "GyroY", "AccZ", "GyroZ"]
    values = {}
    for line in lines:
        li = line.replace('\n', '').split(" ")
        while '' in li:
            li.remove('')
        if "GPSTime" in li and len(li) >= 18:  # 寻找pos320数据字头
            diff_keys = set(keys_name) - set(li)
            # 不同格式的正高
            if diff_keys == {'H-MSL'} and 'H-O' in line:
                line = line.replace("H-O", "H-MSL")
                li = line.replace('\n', '').split(" ")
                while '' in li:
                    li.remove('')
                diff_keys = set(keys_name) - set(li)
            if diff_keys == {'H-MSL'} and 'H-N' in line:
                line = line.replace("H-N", "H-MSL")
                li = line.replace('\n', '').split(" ")
                while '' in li:
                    li.remove('')
                diff_keys = set(keys_name) - set(li)
            # 判断是否缺少字段， 如果缺少直接返回
            if len(diff_keys) > 0:
                print('missing values:', diff_keys)
                return diff_keys
            startFlag = 1
            for i in li:
                if i in keys_name:
                    values[keys[keys_name.index(i)]] = []
        if startFlag == 1 and len(li) >= 22 and li[0] != '(sec)':

            if fre:  # 降频处理
                if int(float(li[0]) * fre) == int(frq_time * fre):
                    continue
            frq_time = float(li[0])

            values[keys[0]].append(float(li[0]))
            values[keys[1]].append(float(li[1]) + float(li[2]) / 60 + float(li[3]) / 3600)
            values[keys[2]].append(float(li[4]) + float(li[5]) / 60 + float(li[6]) / 3600)
            for k in range(3, len(keys)):
                values[keys[k]].append(float(li[k + 4]))
    f.close()
    if "GroundVelocity" in values:
        values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
    return pd.DataFrame(values)


def pos320ToDataframe_1(filepath, fre=None):
    """
    pos320ToDataframe：数据格式转成pandas数据
    ：param filepath:pos320_1文件路径
    ：return：pos320数据的dataframe
    @auther：wenzixuan liqianwen
    @date：2023-01-30

    -      GPSTime      = GPS周内秒
    -      Latitude     = 纬度
    -      Longitude    = 经度
    -      H-Ell        = 椭球高
    -      Heading      = 航向
    -      Pitch        = 俯仰
    -      Roll         = 横滚
    -      posQuality   = 组合导航质量
    -      Vel-N        = 北向速度
    -      Vel-E        = 东向速度
    -      Vel-U        = 天向速度
    -      AcclX        = X轴加速度
    -      AngVelX      = X轴角速度
    -      AcclY        = Y轴加速度
    -      AngVelY      = Y轴角速度
    -      AcclZ        = Z轴加速度
    -      AngVelZ      = Z轴角速度
   """
    frq_time = 0
    startFlag = 0
    GroundVelocity = False
    # with判断文件是否存在
    with open(filepath, 'r', encoding='gb18030', errors='ignore') as f:
        lines = f.readlines()  # 读取list
    keys_name = ['GPSTime', 'Latitude', 'Longitude', 'H-Ell', 'Heading', 'Pitch', 'Roll', 'posQuality',
                 'Vel-N', 'Vel-E', 'Vel-U', 'AcclX', 'AngVelX', 'AcclY', 'AngVelY', 'AcclZ', 'AngVelZ']
    keys = ["time", "lat", "lon", "H-Ell", "yaw", "pitch", "roll", "Q", "NorthVelocity", "EastVelocity",
            "GroundVelocity", "AccX", "GyroX", "AccY", "GyroY", "AccZ", "GyroZ"]
    values = {}
    for line in lines:
        li = line.replace('\n', '').split(" ")
        while '' in li:
            li.remove('')
        if "GPSTime" in li and len(li) >= 17:  # 寻找pos320数据字头
            diff_keys = set(keys_name) - set(li)
            # 地向速度转天向速度
            if diff_keys == {'Vel-U'} and "Vel-D" in line:
                GroundVelocity = True
                line = line.replace("Vel-D", "Vel-U")
                li = line.replace('\n', '').split(" ")
                while '' in li:
                    li.remove('')
                diff_keys = set(keys_name) - set(li)

            if len(diff_keys) > 0:  # 判断是否缺少字段， 如果缺少直接返回
                print('missing values:', diff_keys)
                return diff_keys
            startFlag = 1
            for i in li:
                if i in keys_name:
                    values[keys[keys_name.index(i)]] = []
            continue
        if startFlag == 1 and len(li) >= 17 and li[0] != '(sec)':

            if fre:  # 降频处理
                if int(float(li[0]) * fre) == int(frq_time * fre):
                    continue
            frq_time = float(li[0])

            for k in range(len(keys)):
                values[keys[k]].append(float(li[k]))
    f.close()
    if "GroundVelocity" in values and GroundVelocity is False:
        values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
    return pd.DataFrame(values)


def beiYunDataToDataframe(filepath, fre=None):
    """
    beiYunDataToDataframe：数据格式转成pandas数据
    ：param filepath: 北云原始数据 INSPVAA
    ：return：北云数据的dataframe
    @auther：wenzixuan liqianwen
    @date：2023-07-12
   """
    frq_time = 0
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

                if fre:  # 降频处理
                    # print("tmp", float(tmp[1]))
                    if int(float(tmp[1]) * fre) == int(frq_time * fre):
                        continue
                frq_time = float(tmp[1])

                for i in range(len(keys_name)):
                    values[keys_name[i]].append(float(tmp[i]))

        except Exception as e:
            print(line + ", 数据解析失败" + str(e))
    f.close()
    values["GroundVelocity"] = -np.array(values["GroundVelocity"])  # 天向速度转地向速度
    return pd.DataFrame(values)


def volkswagenDataToDataframe(filepath, fre=None):
    """
    VolkswagenDataToDataframe：数据格式转成pandas数据
    ：param filepath: Volkswagen CSV
    ：return：大众数据的dataframe
    @auther：liqianwen
    @date：2023-09-25

    -      GPS Week      = GPS周
    -      GPS Tow      = GPS周内秒（用这个
    -     Pos Mode
    -     iFlag
    -      GP
    -      GL
    -      UTCDate      =
    -      UTCTime      =
    -      Lat [deg]     = 纬度
    -      Lon [deg]    = 经度
    -      Alt Ellips [m]  = 椭球高
    -      SOG [m/s]
    -      COG [deg]
    -      Hdg [deg]
    -     Vert Vel [m/s] 速度
    -     PDOP   perpendicular_dil 垂直扩张
    -     HDOP  horizontal_dil 水平扩张(horizontal dilation)
    -     HRMS
    -     Roll [deg]    =
    -     Pitchl [deg]  =
    -     Yaw [deg]    =
   """
    # with判断文件是否存在
    frq_time = 0
    startFlag = 0
    # with判断文件是否存在
    with open(filepath, 'r', encoding='gb18030', errors='ignore') as f:
        lines = f.readlines()  # 读取list
    keys_name = ['GPS Week', 'GPS TOW [s]', 'Pos Mode', 'iFlag', 'GP', 'GL', 'UTCDate', 'UTCTime', 'Lat [deg]',
                 'Lon [deg]'
        , 'Alt Ellips [m]', 'SOG [m/s]', 'COG [deg]', 'Hdg [deg]', 'Vert Vel [m/s]', 'PDOP', 'HDOP', 'HRMS'
        , 'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]']
    keys = ['gpsWeek', 'gpsItow', 'posMode', 'iFlag', 'GP', 'GL', 'utcDate', 'utcTime', 'latitude', 'longitude',
            'ellHeight', 'SOG', 'COG', 'Hdg', 'Vel', 'PDOP', 'HDOP', 'HRMS', 'roll', 'pitch', 'heading']
    useless_val = ['posMode', 'iFlag', 'GP', 'GL', 'SOG', 'COG', 'Hdg', 'Vel', 'PDOP', 'HDOP', 'HRMS']
    values = {}
    for line in lines:
        li = line.replace('\n', '').split(",")
        while '' in li:
            li.remove('')
        if "GPS Week" in li and len(li) >= 21:  # 有足够多列
            diff_keys = set(keys_name) - set(li)
            if len(diff_keys) > 0:  # 判断是否缺少字段， 如果缺少直接返回
                print('missing values:', diff_keys)
                return diff_keys

            startFlag = 1
            for i in li:
                if i in keys_name:
                    values[keys[keys_name.index(i)]] = []
            continue

        if startFlag == 1 and len(li) >= 21:
            try:
                if fre:  # 降频处理
                    if int(float(li[1]) * fre) == int(frq_time * fre):
                        continue
                frq_time = float(li[1])

                for k in range(len(keys)):
                    if k in [3,6]:
                        values[keys[k]].append(li[k])
                    else:
                        values[keys[k]].append(float(li[k]))
            except Exception as e:
                print("Error in decoding msg：", line)
                print(e)
                continue
    f.close()
    for val in useless_val:
        values.pop(val)

    return pd.DataFrame(values)


def parse_file_to_dataframe(filepath, file_type, fre=None):
    result = None
    if file_type == "100c":
        result = novatelToDataframe(filepath, fre=fre)
    elif file_type == "pos320":
        result = pos320ToDataframe(filepath, fre=fre)
    elif file_type == "pos320_1":
        result = pos320ToDataframe_1(filepath, fre=fre)
    elif file_type == "北云明文格式":
        result = beiYunDataToDataframe(filepath, fre=fre)
    return result


if __name__ == "__main__":
    # filepath = r"C:\Users\wenzixuan\Downloads\INS20230628144941491\pos320-hlz.txt" #100c文件路径
    # dfPOS320 = pos320ToDataframe_1(filepath)
    # print(dfPOS320["GroundVelocity"])

    # filepath = r"C:\Users\wenzixuan\Downloads\jcy\320.txt"  # 文件路径
    # dfPOS320 = pos320ToDataframe_1(filepath)
    # print(dfPOS320)

    # filepath = r"D:\RTK\100C基准.txt" #100c文件路径
    # df100c = parse_file_to_dataframe(filepath, "100c")
    # print(df100c)
    # print(df100c.columns)

    # filepath = r"D:\data\320.txt"  # 文件路径 D:\RTK\POS320后轮轴中心.txt
    # dfPOS320 = parse_file_to_dataframe(filepath, "pos320")
    # print(dfPOS320)
    # print(dfPOS320.columns)

    # filepath = r"D:\data\320.txt"  # 文件路径
    # filepath = r"C:\Users\wenzixuan\Downloads\beiyun\到测试天线自定义-0626PM.txt"
    # dfPOS320_1 = parse_file_to_dataframe(filepath, "pos320")
    # print(dfPOS320_1)

    from Standardize import dataStandardization
    filepath = r"C:\Users\wenzixuan\Downloads\BYTest\北云-原数据-0711PM.DAT"
    dateTime = "2023-07-11"
    inputData = parse_file_to_dataframe(filepath, "北云明文格式", fre=20)
    print(inputData)
    dfBeiYun = dataStandardization.INSdataStandarsization(inputData, dateTime)
    print(dfBeiYun)
