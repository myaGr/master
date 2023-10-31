from PyQt5.QtCore import QDate
# 设置全局变量

_global_dict = {"TEST_DATE": QDate.currentDate()}

def set_value(key, value):
    # 定义一个全局变量
    global _global_dict
    _global_dict[key] = value


def get_value(key):
    # 获得一个全局变量，不存在则提示读取对应变量失败
    try:
        global _global_dict
        return _global_dict[key]
    except Exception as e:
        print('读取' + key + '失败\r\n' + str(e))
