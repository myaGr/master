import pandas as pd
from Utils import timeExchangeMgr
from Utils import posDMSExchangeMgr
import datetime


# 解析csv数据
def csvToDataframe(file, date, csv_dict, fre=None):
    """
    Nmea(gga、rmc)数据解析
    :param: file: 文件路径
    :param: date: 数据日期
    :param: csv_dict: csv 字段
    :param: fre: 输出频率,单位hz
    :return: gga数据列表， rmc数据列表
    """

    # 解析时间
    def decode_time(time_type, time_col, gps_week, date_time):
        unix = None
        gps_sec = None
        utc = None
        if time_type == "周内秒":
            gps_sec = float(time_col)
            unix = timeExchangeMgr.gps2Unix(gps_week, gps_sec)  # 计算Unix时间
            utc = timeExchangeMgr.unix2Utc(unix)  # 计算UTC时间

        elif time_type == "hhmmss.sss":
            time_col = time_col[0:2] + ":" + time_col[2:4] + ":" + time_col[4:]
            utc = date_time + " " + time_col
            gps_week, gps_sec = timeExchangeMgr.utc2Gps(utc)
            unix = timeExchangeMgr.gps2Unix(gps_week, gps_sec)
            utc = datetime.datetime.strptime(utc, '%Y-%m-%d %H:%M:%S.%f')

        elif time_type == "YYYY-MM-DD hh:mm:ss.sss":
            utc = time_col
            gps_week, gps_sec = timeExchangeMgr.utc2Gps(utc)
            unix = timeExchangeMgr.gps2Unix(gps_week, gps_sec)
            utc = datetime.datetime.strptime(utc, '%Y-%m-%d %H:%M:%S.%f')

        elif time_type == "YYYY/MM/DD hh:mm:ss":
            utc = time_col.replace("/", "-")
            gps_week, gps_sec = timeExchangeMgr.utc2Gps(utc)
            unix = timeExchangeMgr.gps2Unix(gps_week, gps_sec)
            utc = datetime.datetime.strptime(utc, '%Y-%m-%d %H:%M:%S.%f')

        elif time_type == "YYYY年MM月DD日hh时mm分ss秒":
            time_col = time_col.replace("年", "-")
            time_col = time_col.replace("月", "-")
            time_col = time_col.replace("日", "- ")
            time_col = time_col.replace("时", ":")
            time_col = time_col.replace("分", ":")
            utc = time_col.replace("秒", "")
            gps_week, gps_sec = timeExchangeMgr.utc2Gps(utc)
            unix = timeExchangeMgr.gps2Unix(gps_week, gps_sec)
            utc = datetime.datetime.strptime(utc, '%Y-%m-%d %H:%M:%S.%f')

        return unix, gps_sec, utc

    # 解析经纬度
    def decode_coord(coord_unit, coord_col):
        coord = 0
        if coord_unit == "DD.DDDD":
            coord = float(coord_col)
        elif coord_unit == "DD.DDDD°":
            coord_col = coord_col.replace("°", "")
            coord = float(coord_col)
        elif coord_unit == "DDMM.MMMM":
            coord = posDMSExchangeMgr.dm2d(float(coord_col))
        elif coord_unit == "DD°MM.MMMM′":
            coord_col = coord_col.replace("°", "")
            coord_col = coord_col.replace("′", "")
            coord = posDMSExchangeMgr.dm2d(float(coord_col))
        elif coord_unit == "DD.MMSSSS":
            coord = posDMSExchangeMgr.dd2ddmmss_single(float(coord_col))
        elif coord_unit == "DD°MM′SS.SSSS″":
            coord_col = coord_col.replace("°", "")
            coord_col = coord_col.replace("′", "")
            coord_col = coord_col.replace("″", "")
            coord = posDMSExchangeMgr.dd2ddmmss_single(float(coord_col) / 10000)
        return coord

    split_way = csv_dict["split_way"]
    protocol_type = csv_dict["protocol_type"]
    date_time = date.replace("/", "-")  # 统一日期格式
    gpsWeek = timeExchangeMgr.date2Gpsweek(date_time)  # 计算GPS周

    frq_time = 0
    data_dic = {}
    key = ["unixTime", "gpsItow", "gpsWeek", "utcTime", "utcDate", "latitude", "longitude",
           "ellHeight"]  # "velocity","heading"
    for i in key:
        data_dic[i] = []

    with open(file, 'r', encoding='gb18030', errors='ignore') as fileObj:
        data = fileObj.readlines()
        for line in data:
            # print(line)
            if line == '' or 'BKGGA' in line or "\x00" in line:  # 过滤包含乱码和无效数据情况
                continue
            try:
                line_list = line.replace('\n', '').split(split_way)
                while '' in line_list:
                    line_list.remove('')

                # 时间
                time_type = protocol_type['time'][1]
                time_col = line_list[protocol_type['time'][0]]
                unix, gps_sec, utc = decode_time(time_type, time_col, gpsWeek, date_time)  # 解析时间

                # 纬度
                lat_unit = protocol_type['latitude'][1]
                lat_col = line_list[protocol_type['latitude'][0]]
                lat = decode_coord(lat_unit, lat_col)
                if lat == 0:
                    continue

                # 经度
                lon_unit = protocol_type['longitude'][1]
                lon_col = line_list[protocol_type['longitude'][0]]
                lon = decode_coord(lon_unit, lon_col)
                if lon == 0:
                    continue

                # 高度
                height_unit = protocol_type['height'][1]
                if height_unit == "高度单位【米】":
                    height = float(line_list[protocol_type['height'][0]])

            except Exception as e:
                print(line + ", 数据解析失败: " + str(e))
                continue

            if fre:  # 降频处理
                if int(unix * fre) == int(frq_time * fre):
                    continue
            frq_time = unix

            data_dic["unixTime"].append(unix)
            data_dic["gpsItow"].append(gps_sec)
            data_dic["gpsWeek"].append(gpsWeek)
            data_dic["utcTime"].append(utc.time())
            data_dic["utcDate"].append(utc.date())
            data_dic["latitude"].append(lat)
            data_dic["longitude"].append(lon)
            data_dic["ellHeight"].append(height)

        outputData = pd.DataFrame(data_dic)
    fileObj.close()
    return outputData


if __name__ == '__main__':
    # # 案例1
    # # 1. 输入csv文件路径
    # filepath = r"E:\PycharmProjects\工具(gnss-tool & gnssStarCustom)开发交接\beiyun_to_nmea\北云0329PM数据\ins.log"
    # # 2. 输入测试日期，格式"日月年"
    # # date = "290323"
    # date = "2023/3/29"
    # # 3. 输入csv对应格式信息
    # csv_dict = {"protocol_type": {'time': [0, '周内秒'], 'latitude': [1, 'DD.DDDD'], 'longitude': [2, 'DD.DDDD'],
    #                               'height': [3, '高度单位【米】'], 'utcDate': [], 'altitude': [], 'velocity': [],
    #                               'heading': []},
    #             "split_way": ","}
    # df = csvToDataframe(filepath, date, csv_dict)
    # print(df)

    # 案例2
    # 1. 输入csv文件路径
    filepath = r"E:\PycharmProjects\工具(gnss-tool & gnssStarCustom)开发交接\GnssAnalysisTool\_testdata\gnss_test\nmea\ref_gga.txt"
    # 2. 输入测试日期，格式"日月年"
    # date = "290323"
    date = "2022/9/19"
    # 3. 输入csv对应格式信息
    csv_dict = {
        "protocol_type": {'time': [1, 'hhmmss.sss'], 'latitude': [2, 'DDMM.MMMM'], 'longitude': [4, 'DDMM.MMMM'],
                          'height': [11, '高度单位【米】']},
        "split_way": ","}
    df = csvToDataframe(filepath, date, csv_dict, fre=20)
    print(df)
