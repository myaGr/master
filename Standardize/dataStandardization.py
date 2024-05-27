import datetime
from os import times

import pandas as pd
import numpy as np
from Utils import timeExchangeMgr
from Utils import posDMSExchangeMgr
from Parse.HexDataParse import HexDataParse


# Nmea数据标准化
def nmeaStandardization(inputData, date_time):
    """
    Nmea数据标准化
    :param inputData: 输入原始NMEA数据（格式Dataframe）
    :param date_time: 数据采集日期
    :return: outputData 标准化NMEA数据（格式Dataframe）
    """
    if "rmc" in inputData.keys():
        # 数据合并
        Nmea = inputData['gga'].merge(inputData['rmc'], on=['timestamp'])
        # 计算GPS时间、Unix时间
        gpsItow = []
        gpsWeek = []
        unixTime = []
        dateForm = Nmea['datestamp'].apply(str).str.cat(Nmea['timestamp'].apply(str), sep=" ")
        for i in range(len(Nmea['timestamp'])):
            unix_time = timeExchangeMgr.utc2Unix(dateForm[i])
            unixTime.append(unix_time)
            gps_week, gps_sec = timeExchangeMgr.utc2Gps(dateForm[i])
            gpsItow.append(gps_sec)
            gpsWeek.append(gps_week)

        # 经纬度格式转换
        latitude = posDMSExchangeMgr.dm2d(np.array(Nmea['lat_x'].apply(float)))
        longitude = posDMSExchangeMgr.dm2d(np.array(Nmea['lon_x'].apply(float)))

        # 计算椭球高
        ell_height = np.array(Nmea['altitude'].apply(float)) + np.array(Nmea['geo_sep'].apply(float))
        tempData = {"unixTime": np.array(unixTime),
                    "gpsItow": np.array(gpsItow), "gpsWeek": np.array(gpsWeek),
                    "utcTime": np.array(Nmea['timestamp']), "utcDate": Nmea['datestamp'].apply(str),  # UTC日期输出格式统一string
                    "latitude": np.array(latitude), "longitude": np.array(longitude),
                    "altitude": Nmea['altitude'].apply(float), "ellHeight": ell_height,
                    "gpsQuality": Nmea['gps_qual'].apply(int), "hDop": Nmea["horizontal_dil"].apply(float),
                    "satsNum": Nmea['num_sats'].apply(int), "gpsAge": Nmea['age_gps_data'].apply(float)}
        try:
            # RMC对地速度单位转换： 节 转 米/秒
            velocity = np.array(Nmea['spd_over_grnd'].apply(float)) * 0.5144
            tempData["velocity"] = velocity
        except Exception as e:
            print("速度解析失败" + str(e))
        try:
            tempData["heading"] = Nmea['true_course'].apply(float)
        except Exception as e:
            print("航向解析失败" + str(e))

        outputData = pd.DataFrame(tempData)

    else:
        Nmea = inputData['gga']
        # 日期计算
        if date_time is None:
            date_time = timeExchangeMgr.setCurrentTimer("str")
        # 计算UTC时间、GPS时间、Unix时间
        gpsItow = []
        gpsWeek = []
        unixTime = []
        utcDate = []
        for i in range(len(Nmea['timestamp'])):
            unix_time = timeExchangeMgr.utc2Unix(date_time + " " + str(Nmea['timestamp'][i]))
            if i >= 1:
                unix_time_last = timeExchangeMgr.utc2Unix(date_time + " " + str(Nmea['timestamp'][i - 1]))
                if unix_time - unix_time_last < -86000:  # 判断是否存在跨天数据
                    unix_time = unix_time + 86400
                    date_time = timeExchangeMgr.date_add_day(date_time, 1)
            unixTime.append(unix_time)
            gps_week, gps_sec = timeExchangeMgr.utc2Gps(date_time + " " + str(Nmea['timestamp'][i]))
            gpsItow.append(gps_sec)
            gpsWeek.append(gps_week)
            utcDate.append(date_time)

        # 经纬度格式转换
        latitude = posDMSExchangeMgr.dm2d(np.array(Nmea['lat'].apply(float)))
        longitude = posDMSExchangeMgr.dm2d(np.array(Nmea['lon'].apply(float)))

        # 计算椭球高
        ell_height = np.array(Nmea['altitude'].apply(float)) + np.array(Nmea['geo_sep'].apply(float))

        tempData = {"unixTime": np.array(unixTime),
                    "gpsItow": np.array(gpsItow), "gpsWeek": np.array(gpsWeek),
                    "utcTime": np.array(Nmea['timestamp']), "utcDate": np.array(utcDate),
                    "latitude": np.array(latitude), "longitude": np.array(longitude),
                    "altitude": Nmea['altitude'].apply(float), "ellHeight": ell_height,
                    "gpsQuality": Nmea['gps_qual'].apply(int), "hDop": Nmea["horizontal_dil"].apply(float),
                    "satsNum": Nmea['num_sats'].apply(int), "gpsAge": Nmea['age_gps_data'].apply(float)}
        outputData = pd.DataFrame(tempData)

    if "ksxt" in inputData.keys():
        unixTime = []
        timestamp = inputData["ksxt"]['timestamp']
        for i in range(len(timestamp)):
            utcDate = timestamp[i][6:8] + timestamp[i][4:6] + timestamp[i][2:4]
            utcTime = timestamp[i][8:]
            utc_date = timeExchangeMgr.utc2Date(utcDate, utcTime)
            unixTime.append(timeExchangeMgr.utc2Unix(utc_date))

        tempData = {"unixTime": np.array(unixTime),
                    "doubleHeading": np.array(inputData["ksxt"]['double_heading'])}
        ksxt_df = pd.DataFrame(tempData)
        outputData = outputData.merge(ksxt_df, on=['unixTime'])

    elif "dualan" in inputData.keys():
        gps_week = inputData["dualan"]['gpsWeek']
        gps_second = inputData["dualan"]['gpsItow']
        unixTime = timeExchangeMgr.gps2Unix(gps_week, gps_second)

        tempData = {"unixTime": np.array(unixTime),
                    "doubleHeading": np.array(inputData["dualan"]['double_heading'])}
        dua_df = pd.DataFrame(tempData)
        outputData = outputData.merge(dua_df, on=['unixTime'])

    return outputData


def novatelStandardization(inputData, date_time):
    """
    novatel100C数据标准化（也支持Pos320数据）
    :param inputData: 输入原始100C数据、或Pos320数据（格式Dataframe）
    :param date_time: 数据采集日期
    :return: outputData 标准化NMEA数据（格式Dataframe）
    """
    novatel = inputData

    # 计算时间
    unixTime = []
    utcTime = []
    utcDate = []
    gpsWeek = timeExchangeMgr.date2Gpsweek(date_time)  # 计算GPS周
    for i in range(len(novatel['time'])):
        unix = timeExchangeMgr.gps2Unix(gpsWeek, novatel['time'][i])  # 计算Unix时间
        utc = timeExchangeMgr.unix2Utc(unix)  # 计算UTC时间
        unixTime.append(unix)
        utcTime.append(utc.time())  # 提取utc时间部分
        utcDate.append(utc.date())  # 提取utc日期部分

    # 计算速度
    velocity = np.sqrt(np.array(novatel['NorthVelocity']) ** 2 + np.array(novatel['EastVelocity']) ** 2)

    tempData = {"unixTime": np.array(unixTime),
                "gpsItow": np.array(novatel['time']), "gpsWeek": np.array(gpsWeek),
                "utcTime": np.array(utcTime), "utcDate": np.array(utcDate),
                "latitude": np.array(novatel['lat']), "longitude": np.array(novatel['lon']),
                "ellHeight": np.array(novatel['H-Ell']),  # INS数据中高度由INSGPS数据决定，命名统一为ell_height
                "velocity": velocity, "NorthVelocity": np.array(novatel['NorthVelocity']),
                "EastVelocity": np.array(novatel['EastVelocity']), "GroundVelocity": np.array(novatel['GroundVelocity']),
                "pitch": np.array(novatel['pitch']), "roll": np.array(novatel['roll']), "heading": np.array(novatel['yaw'])
                }
    outputData = pd.DataFrame(tempData)
    return outputData


def InsGpsDataStandarsization(inputData, date_time):
    """
    ins数据中GPS数据标准化
    :param inputData：输入dataframe格式的数据
    :param date_time: 数据采集日期
    :return: outputData 标准化ins数据（格式Dataframe）
    """

    GPSdata = inputData

    # 计算时间
    unixTime = []
    utcTime = []
    utcDate = []

    # 计算GPS周
    if "gpsWeek" in GPSdata.keys():     # 采用内部数据计算GPS周
        gpsWeek = GPSdata["gpsWeek"][0]
    else:
        gpsWeek = timeExchangeMgr.date2Gpsweek(date_time)

    for i in range(len(GPSdata['itow_pos'])):
        unix = timeExchangeMgr.gps2Unix(gpsWeek, GPSdata['itow_pos'][i])  # 计算Unix时间
        utc = timeExchangeMgr.unix2Utc(unix)  # 计算UTC时间
        unixTime.append(unix)
        utcTime.append(utc.time())  # 提取utc时间部分
        utcDate.append(utc.date())  # 提取utc日期部分

    flagPos = np.array(GPSdata['flagsPos'])
    flagPos[np.where(flagPos == 0)] = 0   # 无效
    flagPos[np.where(flagPos == 16)] = 1  # 单点
    flagPos[np.where(flagPos == 17)] = 2  # 伪距差分
    flagPos[np.where(flagPos == 34)] = 5  # 浮点
    flagPos[np.where(flagPos == 48)] = 4  # 固定
    flagPos[np.where(flagPos == 49)] = 4  # 固定
    flagPos[np.where(flagPos == 50)] = 4  # 固定

    tempData = {"unixTime": np.array(unixTime),
                "gpsItow": np.array(GPSdata['itow_pos']), "gpsWeek": np.array(gpsWeek),
                "utcTime": np.array(utcTime), "utcDate": np.array(utcDate),
                "latitude": np.array(GPSdata['Lat']), "longitude": np.array(GPSdata['Lon']),
                "ellHeight": np.array(GPSdata['hMSL']),  # INSGPS数据如无特殊要求，高程一律认定为椭球高
                "gpsQuality": flagPos,
                "velocity": np.array(GPSdata['HSpd']), "satsNum": np.array(GPSdata['numSV']),
                "pitch": np.array(GPSdata['Pitch']), "heading": np.array(GPSdata['Heading']),  # GPS no roll value
                "TrackAngle": np.array(GPSdata['TrackAngle']),
                "aveSNR": np.array(GPSdata['aveSNR'])
                # "hDop": np.array(GPSdata['numSV'])  # INSGPS数据解析暂未解析 hdop
                }
    tempData.update(GPSdata)
    duplicateData = ['itow_pos', 'Lat', 'Lon', 'hMSL', 'VSpd', 'numSV', 'Heading', 'Pitch']
    for k in duplicateData:
        if k in tempData.keys():
            tempData.pop(k)

    outputData = pd.DataFrame(tempData)
    return outputData


def INSdataStandarsization(inputData, date_time):
    """
    ins数据标准化
    ：param inputData：输入dataframe格式的数据
    :param date_time: 数据采集日期
    :return: outputData 标准化ins数据（格式Dataframe）
    """

    INSdata = inputData
    unixTime = []
    utcTime = []
    utcDate = []

    # 计算GPS周
    if "gpsWeek" in inputData.keys() and inputData["gpsWeek"][0] != None:
        gpsWeek = inputData["gpsWeek"][0]
    else:
        gpsWeek = timeExchangeMgr.date2Gpsweek(date_time)

    # 计算Unix时间
    for i in range(len(INSdata['time'])):
        unix = timeExchangeMgr.gps2Unix(gpsWeek, INSdata['time'][i])
        utc = timeExchangeMgr.unix2Utc(unix)
        unixTime.append(unix)
        utcTime.append(utc.time())  # 提取utc时间部分
        utcDate.append(utc.date())  # 提取utc日期部分

    tempData = {"unixTime": np.array(unixTime),
                "gpsItow": np.array(INSdata['time']), "gpsWeek": np.array(gpsWeek),
                "utcTime": np.array(utcTime), "utcDate": np.array(utcDate),
                "latitude": np.array(INSdata['lat']), "longitude": np.array(INSdata['lon']),
                "ellHeight": np.array(INSdata['height']), "heading": np.array(INSdata['yaw']),
                "velocity": np.sqrt(np.array(INSdata['NorthVelocity']) ** 2 + np.array(INSdata['EastVelocity']) ** 2),
                }
    tempData.update(INSdata)
    duplicateData = ['time', 'lat', 'lon', 'height', 'yaw']
    for k in duplicateData:
        if k in tempData.keys():
            tempData.pop(k)

    outputData = pd.DataFrame(tempData)
    return outputData


def volkswagenStandardization(inputData, date_time):
    """
    Volkswagen大众PP格式数据标准化: 时间经纬高
    :param inputData: 输入原始100C数据、或Pos320数据（格式Dataframe）
    :param date_time: 数据采集日期
    :return: outputData 标准化NMEA数据（格式Dataframe）
    """
    # 计算时间
    unixTime = []
    for i in range(len(inputData['gpsItow'])):
        unix = timeExchangeMgr.gps2Unix(inputData['gpsWeek'][i], inputData['gpsItow'][i])  # 计算Unix时间
        unixTime.append(unix)
    inputData["unixTime"] = np.array(unixTime)

    add_keys = ["NorthVelocity", "EastVelocity", "GroundVelocity"]
    for key in add_keys:
        inputData[key] = np.nan
    return inputData


def insPlDataStandarsization(inputData, date_time):
    """
    insPlData数据标准化: 时间PL数据格式Dataframe）
    :param date_time: 数据采集日期
    :return 标准化后的dataframe
    """
    # inputData["itow"] = inputData["time"]  # / 10   # / 10 测试！！！
    gpsWeek = timeExchangeMgr.date2Gpsweek(date_time)
    inputData["unixTime"] = timeExchangeMgr.gps2Unix(gpsWeek, inputData["time"])  # 计算Unix时间
    inputData.pop("time")
    if "gps_flag_pos" in inputData:
        inputData["gpsQuality"] = inputData['gps_flag_pos']
        inputData.pop("gps_flag_pos")

    # 计算水平PL
    if "pos_horizontal_pl" not in inputData:
        inputData["pos_horizontal_pl"] = (inputData["pos_lat_pl"] ** 2 + inputData["pos_lon_pl"] ** 2) ** 0.5

    return inputData



if __name__ == "__main__":

    dateTime = "2022-09-19"
    # obj = HexDataParse()
    # obj.filePath = r"E:\PycharmProjects\工具(gnss-tool & gnssStarCustom)开发交接\GnssAnalysisTool\_testdata\gnss_test\nmea\ref.txt"
    # obj.filePath =r"D:\RTK\gnss_data\gnss_test\nmea\Statistics.xlsx"
    # pd.read_excel(r"D:\RTK\gnss_data\gnss_test\nmea\Statistics.xlsx")
    # types = []
    # obj.startParseFileHexData()  # 开始数据解析
    # obj.saveDataToDF()
    # inputData = obj.GpsDataDF
    # print(inputData.keys())
    # data = INSdataStandarsization(inputData, dateTime)
    # print(data)

    from Parse import nmeaDecode
    file_path = r"C:\Users\wenzixuan\Downloads\novatel\navdia2.log"
    inputData = nmeaDecode.nmeaToDataframe(file_path)
    print(inputData)
    if "rmc" not in inputData.keys():
        print("【提示】 数据缺少RMC，已设置测试日期为： " + dateTime)
    df = nmeaStandardization(inputData, dateTime)
    print(df, df.keys())

    # from Parse import novatelDecode
    # file_path = r"C:\Users\wenzixuan\Downloads\beiyun\北云-原数据-0626PM.DAT"
    # inputData = novatelDecode.beiYunDataToDataframe(file_path)
    # print(inputData)
    # df = INSdataStandarsization(inputData, dateTime)  # 标准化
    # print(df)
