from PyQt5 import QtWidgets
from MainWindow import MainWindow  # 主界面的逻辑文件(解析数据)
from MainWindow_DataAnalysis import MainWindow  # 主界面的逻辑文件(数据统计画图)
import sys
import func.SetTime
import time
import tkinter
import tkinter.messagebox as msgbox
import requests
import json


# 检查试用期时间是否过期
def checkLicenseDate():
    # today = func.SetTime.get_network_time()  # 当前时间
    # currentTime = time.mktime(time.strptime(str(today), "%Y-%m-%d %H:%M:%S"))

    thisDay = '2023-02-03 0:0:0'  # 调用截止使用时间
    currentTime = time.mktime(time.strptime(thisDay, "%Y-%m-%d %H:%M:%S"))

    lastDay = '2024-07-19 0:0:0'  # 调用截止使用时间
    lastTime = time.mktime(time.strptime(lastDay, "%Y-%m-%d %H:%M:%S"))

    root = tkinter.Tk()
    root.withdraw()
    if currentTime > lastTime:
        msgbox.showinfo('提示', '试用时间已到，请联系工具所有者更新。')
        sys.exit()


def checkVersion(now_exe_name):
    """
    @author: liqianwen
    :param now_exe_name:
    :return:
    """
    url = 'http://10.1.135.6/solutions/tools/version_test/' + now_exe_name
    # param = {'tool':now_exe_name}
    param = {}
    res_info = request_get(url, param)
    if 'fileName' in res_info.keys():
        yes_download = msgbox.askyesno(title='版本更新',
                                       message='最新版本为 %s ，是否更新？\n（若使用此版本，请选否）' % res_info['fileName'])
        if yes_download:
            download_url = 'http://10.1.135.6/solutions/download/tools/' + res_info['fileName']
            res = requests.get(download_url, param)
            with open(res_info['fileName'], 'wb') as code:
                code.write(res.content)

            msgbox.showinfo('提示', '最新工具已下载。')
            sys.exit()


def request_get(url_get, param_get):
    """
    @author: liqianwen
    :param url_get:
    :param param_get:
    :return:
    """
    fails = 0
    text = {}
    while True:
        try:
            if fails >= 5:
                print('网络连接出现问题, 无法检查当前版本')
                text['message'] = "网络连接失败,无法检查当前版本。"
                return text

            ret = requests.get(url=url_get, params=param_get, timeout=10)
            if ret.status_code == 200:
                text = json.loads(ret.text)
                return text
            else:
                fails += 1
                continue
        except Exception as e:
            fails += 1
            print('网络连接出现问题', e)
            text['message'] = '网络连接失败,无法检查当前版本。'
    return text


def request_post(url_post, param_post):
    """
    @author: liqianwen
    :param url_post:
    :param param_post:
    :return:
    """
    fails = 0
    while True:
        try:
            if fails >= 5:
                print('网络连接出现问题, 无法检查当前版本')
                text['message'] = "网络连接失败,无法检查当前版本。"
                return text

            headers = {'content-type': 'application/json'}
            ret = requests.post(url_post, json=param_post, headers=headers, timeout=10)

            if ret.status_code == 200:
                text = json.loads(ret.text)
            else:
                fails += 1
                continue
        except Exception as e:
            fails += 1
            print(e)
            print('网络连接出现问题, 正在尝试再次请求: ', fails)
            text['message'] = '网络连接失败,无法检查当前版本。'

    return text


if __name__ == "__main__":
    # Check whether the newest version
    try:
        checkVersion('数据解析与分析工具V2.0.9.exe')
    except Exception as e:
        print(e)
        msgbox.showinfo('提示', '网络有问题，无法检测最新版本！')


    # # check License date
    # checkLicenseDate()

    app = QtWidgets.QApplication(sys.argv)
    mainWindow = MainWindow()  # 实例化主界面
    mainWindow.show()
    sys.exit(app.exec_())
