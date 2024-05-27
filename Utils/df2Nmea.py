import pandas as pd
from Utils import posDMSExchangeMgr


def getBcc(buff):
    bcc = 0
    for tmpStr in buff:
        bcc = bcc ^ ord(tmpStr)
    return '{:02X}'.format(bcc)


def saveDfToNmea(df, output_path):
    """
    Nmea(gga、rmc)数据解析
    :param: file: 文件路径
    :return: gga数据列表， rmc数据列表
    """

    gps_qual = "6"
    num_sats = "30"
    horizontal_dil = "0.5"
    geo_sep = "0"
    age_gps_data = ""
    ref_station_id = ""
    spd_over_grnd = "0.000"
    true_course = "000.0"
    datestamp = str(df['utcDate'][0].day).zfill(2) + str(df['utcDate'][0].month).zfill(2) + str(df['utcDate'][0].year)[2:]
    mag_variation = ""
    mag_var_dir = ""
    mode = "A"

    nmea_file = open(output_path, 'w', encoding='gb18030')
    for row in df.itertuples():
        try:
            # 1. 数据转换
            timestamp = str(getattr(row, 'utcTime'))[:12].replace(":", "")
            lat = str(posDMSExchangeMgr.d2dm(float(str(getattr(row, 'latitude')))))
            lon = str(posDMSExchangeMgr.d2dm(float(str(getattr(row, 'longitude')))))
            altitude = str(round(getattr(row, 'ellHeight'), 2))
            gps_qual = str(getattr(row, 'gpsQuality')) if 'gpsQuality' in df else gps_qual
            num_sats = str(getattr(row, 'satsNum')) if 'satsNum' in df else num_sats
            horizontal_dil = str(getattr(row, 'hdop')) if 'hdop' in df else horizontal_dil
            age_gps_data = str(getattr(row, 'diffAge')) if 'diffAge' in df else age_gps_data
            spd_over_grnd = str(round(getattr(row, 'HSpd'), 3)) if 'HSpd' in df else spd_over_grnd
            true_course = str(round(getattr(row, 'TrackAngle'), 1)) if 'TrackAngle' in df else true_course

            # 2.数据组合
            gga_list = ["GPGGA", timestamp, lat, "N", lon, "E", gps_qual, num_sats, horizontal_dil, altitude, "M",
                        geo_sep, "M", age_gps_data, ref_station_id]
            rmc_list = ["GPRMC", timestamp, "A", lat, "N", lon, "E", spd_over_grnd, true_course, datestamp,
                        mag_variation, mag_var_dir, mode]
            gga_str = '$' + ','.join(gga_list) + '*' + getBcc(','.join(gga_list)) + '\n'
            rmc_str = '$' + ','.join(rmc_list) + '*' + getBcc(','.join(rmc_list)) + '\n'
            nmea_file.writelines(gga_str + rmc_str)
            # print(gga_str + rmc_str)
        except Exception as e:
            print("数据解析失败：" + str(e) + ", " + str(row))
    nmea_file.close()


if __name__ == "__main__":
    ins_file = r"E:\PycharmProjects\DataAnalysisTool\NAV3120SIA-INS10-INS-1207PM.log\ins_997326da-2dde-3849-9903-1517d1e44ac0.feather"
    gps_file = r"E:\PycharmProjects\DataAnalysisTool\NAV3120SIA-INS10-INS-1207PM.log\gps_997326da-2dde-3849-9903-1517d1e44ac0.feather"

    df_test = pd.read_feather(gps_file)
    saveDfToNmea(df_test, gps_file + ".nmea")
