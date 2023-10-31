from datetime import datetime
from urllib.request import urlopen
import time
import ssl
import sys
import tkinter
import tkinter.messagebox as msgbox

ssl._create_default_https_context = ssl._create_unverified_context
root = tkinter.Tk()
root.withdraw()


def lock():
    now = get_network_time()
    end_date = datetime(2022, 8, 26)
    if now > end_date:
        print("试用已到期")
        sys.exit()


def get_network_time():
    url = "https://www.baidu.com"
    try:
        with urlopen(url) as res:
            utctime = gmtstr_to_localtime(res.getheader("Date"))
            return datetime_from_utc_to_local(utctime)
    except Exception as e:
        print("请检查网络设置")
        msgbox.showinfo('提示', '无法获取当前网络时间，请将设备连接网络后使用。')
        sys.exit()


def gmtstr_to_localtime(gmtstr) -> datetime:
    return datetime.strptime(gmtstr, "%a, %d %b %Y %H:%M:%S GMT")


def datetime_from_utc_to_local(utc_datetime):
    now_timestamp = time.time()
    offset = datetime.fromtimestamp(now_timestamp) - datetime.utcfromtimestamp(
        now_timestamp
    )
    return utc_datetime + offset


if __name__ == "__main__":

    today = get_network_time()
    print(today)
    currentTime = time.mktime(time.strptime(str(today), "%Y-%m-%d %H:%M:%S"))

    lastDay = '2022-08-10 0:0:0'  # 设置截止使用时间
    print(lastDay)
    lastTime = time.mktime(time.strptime(lastDay, "%Y-%m-%d %H:%M:%S"))
    print("lastTime", lastTime)

    root = tkinter.Tk()
    root.withdraw()
    if currentTime > lastTime:
        print('试用时间已到，请联系工具所有者更新。')
        msgbox.showinfo('提示', '试用时间已到，请联系工具所有者更新。')
