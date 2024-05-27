import datetime
import time
import pytz
from pytz import timezone
import gnsscal
import pandas as pd

UNIX_GPS_DIF = 315964800  # unix与GPS时间固定时间差（秒）
GPS_LEAP_SEC = 18  # GPS 润秒
SECONDS_PER_WEEK = 604800  # 一周时间的秒数:3600 * 24 *7 = 604800

# 跳秒列表
leapSecond_list = [
    [18, 1483228800, 1930.0, datetime.datetime(2017, 1, 1, 0, 0)],
    [17, 1435708800, 1851.0, datetime.datetime(2015, 7, 1, 0, 0)],
    [16, 1341100800, 1695.0, datetime.datetime(2012, 7, 1, 0, 0)],
    [15, 1230768000, 1512.0, datetime.datetime(2009, 1, 1, 0, 0)],
    [14, 1136073600, 1356.0, datetime.datetime(2006, 1, 1, 0, 0)],
    [13, 915148800, 990.0, datetime.datetime(1999, 1, 1, 0, 0)],
    [12, 867715200, 912.0, datetime.datetime(1997, 7, 1, 0, 0)],
    [11, 820454400, 834.0, datetime.datetime(1996, 1, 1, 0, 0)],
    [10, 773020800, 755.0, datetime.datetime(1994, 7, 1, 0, 0)],
    [9, 741484800, 703.0, datetime.datetime(1993, 7, 1, 0, 0)],
    [8, 709948800, 651.0, datetime.datetime(1992, 7, 1, 0, 0)],
    [7, 662688000, 573.0, datetime.datetime(1991, 1, 1, 0, 0)],
    [6, 631152000, 521.0, datetime.datetime(1990, 1, 1, 0, 0)],
    [5, 567993600, 416.0, datetime.datetime(1988, 1, 1, 0, 0)],
    [4, 489024000, 286.0, datetime.datetime(1985, 7, 1, 0, 0)],
    [3, 425865600, 181.0, datetime.datetime(1983, 7, 1, 0, 0)],
    [2, 394329600, 129.0, datetime.datetime(1982, 7, 1, 0, 0)],
    [1, 362793600, 77.0, datetime.datetime(1981, 7, 1, 0, 0)]
]


def cal_leapSecond(time_type, time_value):
    leap_second = 0
    for leap in leapSecond_list:
        if time_type == "unixTime":
            if time_value >= leap[1]:
                leap_second = leap[0]
                return leap_second
        elif time_type == "gpsWeek":
            if time_value >= leap[2]:
                leap_second = leap[0]
                return leap_second
        elif time_type == "dateTime":
            if time_value >= leap[3]:
                leap_second = leap[0]
                return leap_second
    return leap_second


# 返回当前时间
def setCurrentTimer(datetype="date", format='%Y-%m-%d %H:%M:%S.%f'):
    """
    返回当前时间
    :param format: 返回时间格式
    :param datetype: 返回时间据类型，默认返回日期
    :return: 当前时间 ("Date"返回日期; "Unix"返回Unix时间)
    """
    if datetype == "date":
        timeDate = datetime.datetime.now().strftime(format)
        return timeDate
    if datetype == "unix":
        timeUnix2 = time.time()
        return timeUnix2
    if datetype == "str":
        dateStr = datetime.datetime.now().strftime('%Y-%m-%d')
        return str(dateStr)


# UTC时间转日期格式
def utc2Date(utc_date, utc_time):
    """
    UTC时间转日期
    :param utc_date: utc日期, 字符串（格式：“DDMMYY”）
    :param utc_time: utc时间，字符串（格式：“HHMMSS.f”）
    :return: 返回日期时间（格式： %Y-%m-%d %H:%M:%S.%f）
    """
    dateTimeStr = "20" + utc_date[-2:] + "-" + utc_date[2:4] + "-" + utc_date[:2] + " " \
                  + utc_time[:2] + ":" + utc_time[2:4] + ":" + utc_time[4:]
    return dateTimeStr


# UTC时间（日期）转北京时间（日期）
def utc2BjTime(utc_time):
    """
    UTC时间（日期）转北京时间（日期）
    :param utc_time: utc时间，（格式： %Y-%m-%d %H:%M:%S.%f）
    :return: 北京时间日期（格式： %Y-%m-%d %H:%M:%S.%f）
    """
    if "." not in utc_time:
        utc_time = utc_time + ".0"
    retDateTime = datetime.datetime.strptime(utc_time, '%Y-%m-%d %H:%M:%S.%f').replace(tzinfo=pytz.timezone('UTC'))
    pk_time = retDateTime.astimezone(timezone('Asia/Shanghai'))
    return pk_time


# UTC时间（日期）转北京时间（日期）
def bjTime2Utc(pk_time):
    if "." not in pk_time:
        pk_time = pk_time + ".0"
    retDateTime = datetime.datetime.strptime(pk_time, '%Y-%m-%d %H:%M:%S.%f')
    utc_time = retDateTime.astimezone(timezone('UTC'))
    return utc_time


# Unix时间转北京时间（日期）
def unix2date(unix_time, unit="s"):
    """
    Unix时间转北京时间（日期）
    :param unix_time: unix时间
    :param unit: unix时间单位，默认秒（秒：s、毫秒：ms）
    :return: 返回完整日期格式（"%Y-%m-%d %H:%M:%S"）
    """
    if unit == "ms":  # 毫秒
        unix_time = unix_time / 1000
    retTime = datetime.datetime.fromtimestamp(unix_time)
    return retTime


# 北京时间（日期）转Unix时间
def date2Unix(date_time):
    """
    北京时间（日期）转Unix时间
    :param date_time: 日期格式时间（"%Y-%m-%d %H:%M:%S.%f"）
    :return: 返回unix时间格式
    """
    if "." not in date_time:
        date_time = date_time + ".0"
    date_time = datetime.datetime.strptime(date_time, '%Y-%m-%d %H:%M:%S.%f')
    unix_time = time.mktime(date_time.timetuple()) + date_time.microsecond / 1000000
    return unix_time


# Unix时间转UTC时间（日期）
def unix2Utc(unix_time, unit="s"):
    """
    Unix时间转UTC时间（日期）
    :param unix_time: unix时间
    :param unit: unix时间单位，默认秒（秒：s、毫秒：ms）
    :return: 返回完整日期格式（"%Y-%m-%d %H:%M:%S"）
    """
    if unit == "ms":  # 毫秒
        unix_time = unix_time / 1000
    utc_time = datetime.datetime.utcfromtimestamp(unix_time)
    return utc_time


# UTC时间（日期）转unix时间
def utc2Unix(utc_time):
    """
    UTC时间转unix时间
    :param utc_time: 日期时间（格式： %Y-%m-%d %H:%M:%S.%f）
    :return: 返回unix时间格式
    """
    pk_time = utc2BjTime(utc_time)
    unix_time = time.mktime(pk_time.timetuple()) + pk_time.microsecond / 1000000
    return unix_time


# Unix时间转GPS时间（日期）
def unix2Gps(unix_time, unit="s"):
    """
    Unix时间转GPS时间（日期）
    :param unix_time: unix时间
    :param unit: unix时间单位，默认秒（秒：s、毫秒：ms）
    :return: 返回完整日期格式（"%Y-%m-%d %H:%M:%S"）
    """
    if unit == "ms":  # 毫秒
        unix_time = unix_time / 1000

    leapSecond = cal_leapSecond("unixTime", unix_time)
    gps_time = datetime.datetime.utcfromtimestamp(unix_time + leapSecond)
    return gps_time


# GPS时间（日期）转unix时间
def gpsTime2Unix(gps_time):
    """
    GPS时间转unix时间
    :param gps_time: 日期时间（格式： %Y-%m-%d %H:%M:%S.%f）
    :return: 返回unix时间格式
    """
    pk_time = utc2BjTime(gps_time)
    unix_time = time.mktime(pk_time.timetuple()) + pk_time.microsecond / 1000000
    leapSecond = cal_leapSecond("unixTime", unix_time)
    return unix_time - leapSecond


# gps时间转Unix时间
def gps2Unix(gps_week, gps_sec):
    """
    gps时间转Unix时间
    :param gps_week: gps周
    :param gps_sec: gps周内秒
    :return: 返回unix时间格式
    """
    gps_week = gps_week[0] if isinstance(gps_week, pd.Series) else gps_week
    leapSecond = cal_leapSecond("gpsWeek", gps_week)
    tempSec = gps_sec + UNIX_GPS_DIF - leapSecond
    unix_time = gps_week * SECONDS_PER_WEEK + tempSec
    return unix_time


# gps时间转UTC时间
def gps2UTC(gps_week, gps_sec):
    """
    gps时间转UTC时间
    :param gps_week: gps周
    :param gps_sec: gps周内秒
    :return: 返回UTC时间格式
    """
    leapSecond = cal_leapSecond("gpsWeek", gps_week)
    tempSec = gps_sec + UNIX_GPS_DIF - leapSecond
    unix_time = gps_week * SECONDS_PER_WEEK + tempSec
    utc_time = unix2Utc(unix_time)
    return utc_time


# UTC时间转gps时间
def utc2Gps(date_time):
    """
    UTC时间转gps时间
    :param date_time: 日期时间（格式： %Y-%m-%d %H:%M:%S.%f）
    :return: gps周, gps周内秒
    """
    if "." not in date_time:
        date_time = date_time + ".0"
    unix_time = utc2Unix(date_time)
    leapSecond = cal_leapSecond("unixTime", unix_time)
    tempSec = unix_time - UNIX_GPS_DIF + leapSecond
    gps_week = (tempSec - tempSec % SECONDS_PER_WEEK) / SECONDS_PER_WEEK
    gps_sec = tempSec % SECONDS_PER_WEEK
    return gps_week, gps_sec


# utc转GPS周
def date2Gpsweek(date_time_str):
    """
    UTC时间转gps时间
    :param date_time_str: 日期时间字符串（格式： %Y-%m-%d）
    :return: gps周
    """
    date_time = date_time_str + " 00:00:00.00"  # %H:%M:%S.%f
    unix_time = utc2Unix(date_time)
    leapSecond = cal_leapSecond("unixTime", unix_time)
    tempSec = unix_time - UNIX_GPS_DIF + leapSecond
    gps_week = (tempSec - tempSec % SECONDS_PER_WEEK) / SECONDS_PER_WEEK
    return gps_week


def date_add_day(date_time, days):
    """
    UTC日期跨天
    :param date_time: 日期时间字符串（格式： %Y-%m-%d）
    :param days: 变化天数（类型： int）
    :return: date_time_str
    """
    date_time_str = datetime.datetime.strptime(date_time + " 00:00:00.00", '%Y-%m-%d %H:%M:%S.%f')
    add_date_time = date_time_str + datetime.timedelta(days=days)  # 日期跨天
    add_datetime_str = add_date_time.strftime("%Y-%m-%d")
    return add_datetime_str


# 日期转年积日
def timedate2doy(date_time):
    year, day = gnsscal.date2yrdoy(date_time.date())
    return year, day


# 年积日转日期
def doy2timedate(year, day):
    date_time = gnsscal.yrdoy2date(year, day)
    return date_time.strftime('%Y-%m-%d %H:%M:%S')


if __name__ == '__main__':
    print("current date", setCurrentTimer("date"))
    print("current unix", setCurrentTimer("unix"))

    unixTime = 1669659413.22
    print("unix2date", [unixTime], unix2date(unixTime, unit="s"))

    dateTime = "2022-11-28 04:07:18.2"
    print("date2Unix", [dateTime], date2Unix(dateTime))  # "%Y-%m-%d %H:%M:%S.%f"

    unixTime = 1661934000.22
    print("unix2Utc", [unixTime], unix2Utc(unixTime))

    utcDate = "281122"  # DDMMYY
    utcTime = "040718.2"  # HHMMSS
    utc_date = utc2Date(utcDate, utcTime)
    print("utc2Unix", [utcDate, utcTime], utc_date, utc2Unix(utc_date))

    gpsWeek = 2234
    gpsSec = 145218.22  # 2022-10-31 16:20:22
    print("gps2UTC", [gpsWeek, gpsSec], gps2UTC(gpsWeek, gpsSec))
    print("gps2Unix", [gpsWeek, gpsSec], gps2Unix(gpsWeek, gpsSec))

    utcTime = "2022-11-28 04:07:18.2"
    print("utc2Gps", [utcTime], utc2Gps(utcTime))

    date_time = "2022-09-19"
    print("date2Gpsweek", date_time, date2Gpsweek(date_time))

    date = "2022-09-19"
    date_time = datetime.datetime.strptime(date_time, '%Y-%m-%d')
    print("timedate2doy", date_time, timedate2doy(date_time))

    year, day = 2022, 262
    print("doy2timedate", year, day, doy2timedate(year, day))

    unixTime = 1661934000.22
    print("unix2Gps", [unixTime], unix2Gps(unixTime))
