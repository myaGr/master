import pynmea2
import pandas as pd


# Nmea(gga、rmc)数据解析
def decodeNmeaGGA(file, fre=None):
    """
    Nmea(gga、rmc)数据解析
    :param: file: 文件路径
    :return: gga数据列表， rmc数据列表
    """
    if check_file_exist(file) != 1:
        return 0
    ggaList, rmcList,  ksxtList = [], [], []
    gga_frq_time, rmc_frq_time, ksxt_frq_time = 0, 0, 0   # 降频初始时间
    with open(file, 'r', encoding='gb18030', errors='ignore') as fileObj:
        data = fileObj.readlines()
        for line in data:
            if line == '' or 'BKGGA' in line or "\x00" in line:  # 过滤包含乱码情况
                continue
            if "$GNGGA" in line or "$GPGGA" in line and len(line.split(",")) == 15:  # 从$开启截取信息
                try:
                    line = line[line.rfind('$'):]    # str = str.rstrip('\n')
                    if line.split(",")[1] is None or line.split(",")[1] == '' or line.split(",")[2] == '' or line.split(",")[6] == '':
                        continue
                    elif float(line.split(",")[1]) == 0.0 or line.split(",")[6] == 0:
                        continue
                    tmp = pynmea2.parse(line)
                    if tmp.geo_sep == '':
                        tmp.geo_sep = 0
                    if tmp.age_gps_data == '':
                        tmp.age_gps_data = 0
                    # 把类强制转换成list，在变量前加tmp
                    gga = [tmp.timestamp, float(tmp.lat), tmp.lat_dir, float(tmp.lon), tmp.lon_dir, int(tmp.gps_qual),
                           int(tmp.num_sats), float(tmp.horizontal_dil), float(tmp.altitude), tmp.altitude_units,
                           float(tmp.geo_sep), tmp.geo_sep_units, float(tmp.age_gps_data), tmp.ref_station_id]

                    if fre:  # gga降频处理
                        if int(float(tmp.data[0]) * fre) == int(gga_frq_time * fre):
                            continue
                    gga_frq_time = float(tmp.data[0])

                    ggaList.append(gga)
                except Exception as e:
                    print(line + ", 数据解析失败" + str(e))
                    continue
            elif "$GNRMC" in line or "$GPRMC" in line and len(line.split(",")) == 13:  # 从$开启截取信息
                line = line[line.rfind('$'):]  # str = str.rstrip('\n')
                try:
                    tmprmc = pynmea2.parse(line)
                    if not tmprmc.timestamp or tmprmc.timestamp == ' ' or tmprmc.timestamp == 0.0:
                        continue
                    # 把类强制装换成list加了tmprmc
                    rmc = [tmprmc.timestamp, tmprmc.status, float(tmprmc.lat), tmprmc.lat_dir,
                           float(tmprmc.lon), tmprmc.lon_dir, float(tmprmc.spd_over_grnd), float(tmprmc.true_course),
                           tmprmc.datestamp, tmprmc.mag_variation, tmprmc.mag_var_dir]

                    if fre:  # rmc降频处理
                        if int(float(tmprmc.data[0]) * fre) == int(rmc_frq_time * fre):
                            continue
                    rmc_frq_time = float(tmprmc.data[0])

                    rmcList.append(rmc)
                except Exception as e:
                    print(line + ", 数据解析失败" + str(e))
                    continue
            elif "$KSXT" in line and len(line.split(",")) > 10:
                line = line[line.rfind('$'):]
                try:
                    line_list = line.replace('\n', '').split(",")
                    if fre:  # 降频处理
                        if int(float(line_list[1]) * fre) == int(ksxt_frq_time * fre):
                            continue
                    ksxt_frq_time = float(tmprmc.data[0])

                    date_time = line_list[1]
                    double_heading = float(line_list[5])

                    ksxt = [date_time, double_heading]
                    ksxtList.append(ksxt)
                except Exception as e:
                    print(line + ", 数据解析失败" + str(e))
                    continue

    return ggaList, rmcList, ksxtList


# 检查文件是否存在
def check_file_exist(path):
    """
        check_file_exist 检查文件是否存在
        :param file_path: 文件路径
        :return 1: 文件有效
        :return 0: 文件无效
    """
    try:
        f = open(path)
        f.close()
        return 1
    except FileNotFoundError:
        print(path + "  is not found.")
        return 0
    except IOError:
        print(path + " is not accessible.")
        return 0


# nmea数据格式转pandas数据
def nmeaToDataframe(file_path, fre=None):
    """
        nmeaToDataframe 数据格式转pandas数据
        :param fre: 输出频率
        :param file_path: nmea格式文件路径
        :return: gga的dataframe, rmc的dataframe
        @author: cuihe
        @date: 2022-10-25
    """
    ggalist, rmclist, ksxtList = decodeNmeaGGA(file_path, fre=fre)   # 路径是否存在验证
    if len(ggalist) == 0:
        return
    gga_columns = ['timestamp', 'lat', 'lat_dir', 'lon', 'lon_dir', 'gps_qual', 'num_sats', 'horizontal_dil',
                   'altitude', 'altitude_units', 'geo_sep', 'geo_sep_units', 'age_gps_data', 'ref_station_id']
    gga_index = range(len(ggalist))
    gga_df = pd.DataFrame(ggalist, columns=gga_columns, index=gga_index)

    rmc_columns = ['timestamp', 'status', 'lat', 'lat_dir', 'lon', 'lon_dir', 'spd_over_grnd', 'true_course',
                   'datestamp', 'mag_variation', 'mag_var_dir']
    rmc_index = range(len(rmclist))
    rmc_df = pd.DataFrame(rmclist, columns=rmc_columns, index=rmc_index)

    ksxt_columns = ['timestamp', 'double_heading']
    ksxt_index = range(len(ksxtList))
    ksxt_df = pd.DataFrame(ksxtList, columns=ksxt_columns, index=ksxt_index)

    # 有rmc的时候，输出rmc和gga的dataframe; 没有rmc的时候,只输出gga的dataframe，rmc为空={}，
    nmea_dict = {"gga": gga_df}
    if not rmc_df.empty:
        nmea_dict["rmc"] = rmc_df
    if not rmc_df.empty:
        nmea_dict["ksxt"] = ksxt_df
    return nmea_dict




if __name__ == '__main__':
    filepath = r"C:\Users\wenzixuan\Downloads\双天线测试\凯芯新硬件-RTK-1008PM.log"
    print(nmeaToDataframe(filepath))
    # gga_df, rmc_df = nmeaToDataframe(filepath)
    # print("gga", len(gga_df), gga_df.keys())
    # print(gga_df)
    # print("rmc", len(rmc_df), rmc_df.keys())
    # print(rmc_df)

'''
>>> line = '$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76'
>>> record = pynmea2.parse(line)
>>> record<GGA(timestamp=datetime.time(9, 27, 50), lat='5321.6802', lat_dir='N', lon='00630.3372', lon_dir='W', 
>>> gps_qual=1, num_sats='8', horizontal_dil='1.03', altitude=61.7, altitude_units='M', geo_sep='55.2', 
>>> geo_sep_units='M', age_gps_data='', ref_station_id='')>

>>> record<RMC(timestamp=datetime.time(1, 29, 38, 700000), status='A', lat='3117.070232', lat_dir='N',
>>> lon='12109.931233', lon_dir='E', spd_over_grnd=0.0, true_course=114.2, datestamp=datetime.date(2021, 11, 8), 
>>> mag_variation='', mag_var_dir='') data=['A']>
'''

# class GGA():
#     # GG的详细定义
#     timestamp = 0.0  # < 1 > UTC时间，格式为hhmmss.sss；
#     lat = 0.0  # < 2 > 纬度，格式为ddmm.mmmm(第一位是零也将传送)；
#     lat_dir = ''  # < 3 > 纬度半球，N或S(北纬或南纬)
#     lon = 0.0  # < 4 > 经度，格式为dddmm.mmmm(第一位零也将传送)；
#     lon_dir = ''  # < 5 > 经度半球，E或W(东经或西经)
#     gps_qual = 0  # < 6 > GPS状态， 0初始化， 1单点定位， 2 码差分， 3无效PPS， 4固定解， 5浮点解，
#     # 6正在估算  7，人工输入固定值， 8  模拟模式， 9    WAAS差分
#     num_sats = 0  # < 7 > 使用卫星数量，从00到12(第一个零也将传送)
#     horizontal_dil = 0.0  # < 8 > HDOP - 水平精度因子，0.5到99.9，一般认为HDOP越小，质量越好。
#     altitude = 0.0  # < 9 > 海拔高度，-9999.9到9999.9米
#     altitude_units = 'M'  # M指单位米
#     geo_sep = 0.0  # < 10 > 大地水准面高度异常差值，-9999.9到9999.9米
#     geo_sep_units = 'M'  # M  指单位米
#     age_gps_data = -1.0  # < 11 > 差分GPS数据期限(RTCM SC-104)，最后设立RTCM传送的秒数量，如不是差分定位则为空
#     ref_station_id = 0  # < 12 > 差分参考基站标号，从0000到1023(首位0也将传送)。



