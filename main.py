import global_var
import sys
import time
from Utils import setTime
from PyQt5.QtWidgets import QApplication
from MainWindow import MainWindow  # 主界面的逻辑文件(解析数据)
import tkinter
import tkinter.messagebox as msgbox
import requests
import json


# 检查试用期时间是否过期
def checkLicenseDate():
    today = setTime.get_network_time()  # 当前时间
    currentTime = time.mktime(time.strptime(str(today), "%Y-%m-%d %H:%M:%S"))

    lastDay = '2024-09-10 0:0:0'  # 调用截止使用时间
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
    url = 'http://10.1.135.105/solutions/tools/version_test/' + now_exe_name
    # param = {'tool':now_exe_name}
    param = {}
    res_info = request_get(url, param)
    if 'fileName' in res_info.keys():
        yes_download = msgbox.askyesno(title='版本更新',
                                       message='最新版本为 %s ，是否更新？\n（若使用此版本，请选否）' % res_info['fileName'])
        if yes_download:
            download_url = 'http://10.1.135.105/solutions/download/tools/' + res_info['fileName']
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


if __name__ == "__main__":
    # 检查试用期时间是否过期
    # checkLicenseDate()

    # 工具版本设置
    tool_version = '数据解析与分析工具V3.4'
    global_var.set_value("TOOL_VERSION", tool_version)

    # 版本检测
    try:
        checkVersion(tool_version + '.exe')
    except Exception as e:
        print(e)
        msgbox.showinfo('提示', '网络有问题，无法检测最新版本！')

    # 实例化主界面
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())
