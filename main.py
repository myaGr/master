from PyQt5 import QtWidgets
from MainWindow import MainWindow  # 主界面的逻辑文件(解析数据)
from MainWindow_DataAnalysis import MainWindow  # 主界面的逻辑文件(数据统计画图)
# from MainWindow_DataAnalysis_Test1 import MainWindow  # 主界面的逻辑文件(数据统计画图)
# from MainWindow_DataAnalysis_Test2 import MainWindow  # 主界面的逻辑文件(数据统计画图)
import sys
import SetTime
import time
import tkinter
import tkinter.messagebox as msgbox


# 检查试用期时间是否过期
def checkLicenseDate():
    today = SetTime.get_network_time()  # 当前时间
    currentTime = time.mktime(time.strptime(str(today), "%Y-%m-%d %H:%M:%S"))

    lastDay = '2023-09-10 0:0:0'  # 调用截止使用时间
    lastTime = time.mktime(time.strptime(lastDay, "%Y-%m-%d %H:%M:%S"))

    root = tkinter.Tk()
    root.withdraw()
    if currentTime > lastTime:
        msgbox.showinfo('提示', '试用时间已到，请联系工具所有者更新。')
        sys.exit()


if __name__ == "__main__":
    checkLicenseDate()
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = MainWindow()  # 实例化主界面
    mainWindow.show()
    sys.exit(app.exec_())
