import numpy as np


# 经纬度度分格式转度
def dm2d(dm):
    """
    度分格式转度
    :param dm: 输入经纬度值，格式为dddmm.mmmm, 数据类型：Array
    :return : 输出经纬度值，格式为ddd.dddddd
    """
    if type(dm) == np.ndarray:
        int_d = np.floor(dm / 100)
        point_d = (dm - int_d * 100) / 60
        dd = int_d + point_d
    else:
        int_d = int(dm / 100)
        point_d = (dm - int_d * 100) / 60
        dd = int_d + point_d
        dd = round(dd, 9)
    return dd


def d2dm(d):
    """
    度格式转度分【未测试验证】
    :param d: 输入经纬度值，格式为ddd.dddddd，数据类型：Array
    :return dm: 输出经纬度值，格式为dddmm.mmmm
    """
    if type(d) == np.ndarray:
        ddd = np.floor(d) * 100
        mm = (d - np.floor(d)) * 60
        dm = ddd + mm
    else:
        ddd = int(d) * 100
        mm = (d - int(d)) * 60
        dm = ddd + mm
        dm = round(dm, 7)
    return dm


def ddmmss2dd(dd, mm, ss):
    """
    :param ddmmss: dd, mm, ss 12, 14, 36.96
    :return:                  12.2436
    """
    m = mm / 60  # 分 m = 0.35
    s = ss / 3600  # 秒 s = 0.005
    s = round(s, 9)

    return dd + m + s


def dd2ddmmss(dd):
    """
    :param dd: 12.2436
    :return:   12, 14, 36.96
    """
    d = int(dd)
    m = int((dd - d) * 60)
    s = (dd - d - (int((dd - d) * 60) / 60)) * 3600
    s = round(s, 5)
    return d, m, s


# 不同单位转换
def ddmmss2dd_single(ddmmss):
    """
    :param ddmmss: dd.mmss 12.143696
    :return:               12.2436
    """
    d = int(ddmmss)  # 度 ddmmss = 3.2118
    a = ddmmss - d - (ddmmss - d) % 0.01  # a = 0.21
    m = int(a * 100)  # 分 m = 0.35
    b = (ddmmss - d) % 0.01  # b = 0.0018
    s = b * 10000  # 秒 s = 0.005

    return d + m / 60 + s / 3600


def dd2ddmmss_single(dd):
    """
    :param dd: 12.2436
    :return:   12.143696
    """
    d = int(dd)
    m = int((dd - d) * 60)
    s = (dd - d - (int((dd - d) * 60) / 60)) * 3600

    return d + m / 100 + s / 10000


if __name__ == '__main__':
    dm = 12118.751924
    d = dm2d(dm)
    print(dm, "dm2d", d)
    d = 121.31253206666666
    dm = d2dm(d)
    print(d, "d2dm", dm)
