from Parse.HexDataParse import HexDataParse
from Parse import novatelDecode, nmeaDecode, csvDecode
from Standardize import dataStandardization
from Utils import timeExchangeMgr


# 数据解析
def allDataParse(data_dic, date_time, data_analysis_flag=None, freq=None):
    """
    所有数据解析
    :param data_analysis_flag: INS解析帧dic
    :param data_dic: 数据解析dic, 包含{"file_path"：数据路径，"file_type"：数据类型}
    :param date_time: 测试日期str，格式“YYYY-MM-DD”
    :param freq: 解析数据输出频率（降频）,单位hz
    return：-1 解析失败， data_dic["df"]解析结果dataframe
    """
    data_analysis_flag = {'ins': True, 'gps': True, 'vehicle': True, 'imu': True, 'InsPl': False, 'GnssPl': False,
                          'ins2': False, 'imu2': False, 'sync': False,
                          'sat': False, 'sat2': False, 'ZeroBias': False, 'EKFhandle_type': False
                          } if not data_analysis_flag else data_analysis_flag

    if data_dic["file_type"] == "NMEA":
        input_data = nmeaDecode.nmeaToDataframe(data_dic["file_path"], fre=freq)
        if input_data is None:
            return -1
        elif "rmc" not in input_data.keys():
            print("【提示】 数据缺少RMC，已设置测试日期为： " + date_time)
        data_dic["df"] = dataStandardization.nmeaStandardization(input_data, date_time)
    elif data_dic["file_type"] == "100C":
        input_data = novatelDecode.novatelToDataframe(data_dic["file_path"], fre=freq)
        if input_data.empty:
            return -1
        data_dic["df"] = dataStandardization.novatelStandardization(input_data, date_time)
    elif data_dic["file_type"] == "POS320":
        input_data = novatelDecode.pos320ToDataframe(data_dic["file_path"], fre=freq)  # Pos320版本(2)
        if input_data.empty:
            input_data = novatelDecode.pos320ToDataframe_1(data_dic["file_path"], fre=freq)  # Pos320版本(1)
        if input_data.empty:
            return -1
        data_dic["df"] = dataStandardization.novatelStandardization(input_data, date_time)
    elif data_dic["file_type"] == "导远自定义格式":
        obj = HexDataParse()
        obj.data_analysis_flag = data_analysis_flag
        obj.filePath = data_dic["file_path"]
        obj.startParseFileHexData()
        obj.saveDataToDF()
        # check frequency
        obj.checkFreq()
        # obj.outputDataFrameStatsResult()

        input_ins_data = obj.InsDataDF
        input_gps_data = obj.GpsDataDF
        input_vehicle_data = obj.VehicleDataDF
        input_pdata = obj.PDataDict
        input_imu = obj.ImuDataDF
        if input_gps_data.empty:
            obj.full_info += '\n无gps帧，将无法鉴定该测试数据的具体时间。'
        else:
            utc_time_gps = timeExchangeMgr.gps2UTC(input_gps_data.gpsWeek[0], input_gps_data.itow_pos[0]).strftime('%Y-%m-%d')
            if utc_time_gps != date_time:
                obj.full_info += '\n【注意】gps帧时间为：' + utc_time_gps + '与设定时间不符，采用GPS帧内时间。'
                date_time = utc_time_gps
            data_dic["df_gps"] = dataStandardization.InsGpsDataStandarsization(input_gps_data, date_time)
            obj.full_info += '\n该数据含GPS帧'
        if input_ins_data.empty:
            obj.full_info += '\n无ins帧，影响：界面无轨迹及数据列表展示、无ins帧相关画图、无统计功能'
        else:
            data_dic["df"] = dataStandardization.INSdataStandarsization(input_ins_data, date_time)
            data_dic["df_pdata"] = input_pdata
        if input_imu.empty:
            obj.full_info += '\n无imu帧'
        else:
            data_dic["df_imu"] = input_imu
            obj.full_info += '\n该数据含imu帧'
        if input_vehicle_data.empty:
            obj.full_info += '\n无轮速帧'
        else:
            data_dic["df_vehicle"] = input_vehicle_data
            obj.full_info += '\n该数据含轮速帧'
        return obj.full_info
    elif data_dic["file_type"] == "导远自定义-GPS":
        obj = HexDataParse()
        obj.data_analysis_flag = dict.fromkeys(obj.data_analysis_flag, False)
        obj.data_analysis_flag['gps'] = True
        obj.filePath = data_dic["file_path"]
        obj.startParseFileHexData()
        obj.saveDataToDF()
        input_gps_data = obj.GpsDataDF
        if input_gps_data.empty:
            return -1
        data_dic["df"] = dataStandardization.InsGpsDataStandarsization(input_gps_data, date_time)
    elif data_dic["file_type"] == "北云明文格式":
        input_data = novatelDecode.beiYunDataToDataframe(data_dic["file_path"], fre=freq)
        if input_data.empty:
            return -1
        data_dic["df"] = dataStandardization.INSdataStandarsization(input_data, date_time)
    elif data_dic["file_type"] == "大众PP格式":
        input_data = novatelDecode.volkswagenDataToDataframe(data_dic["file_path"], fre=freq)
        if input_data.empty:
            return -1
        data_dic["df"] = dataStandardization.volkswagenStandardization(input_data, date_time)
    elif data_dic["file_type"] == "CSV自定义":
        input_data = csvDecode.csvToDataframe(data_dic["file_path"], date_time, data_dic["csv_dict"], fre=freq)
        if input_data.empty:
            return -1
        data_dic["df"] = input_data
