from numpy import *
import numpy as np
import pandas as pd
from enum import Enum
import time

_STANDF_INTERVEL = 0.05
_STANDF_MAX_PE_PL = 10
_STANDF_AX_PE_NUM = int(_STANDF_MAX_PE_PL / _STANDF_INTERVEL)
_AL = 3     # Alert  Limit (AL)：告警门限
DR_DIST = 411

class FilterStatus(Enum):
    FILTER_POS_SPP = 0
    FILTER_POS_PREDICT = 1
    FILTER_POS_INS = 2
    FILTER_POS_RTD = 3
    FILTER_POS_FLOAT = 4
    FILTER_POS_PREFIX = 5
    FILTER_POS_FIX = 6
    FILTER_POS_FIX_WL = 7
    FILTER_POS_FIX_EWL = 8
    FILTER_POS_MAX = 9


def StanfordIdexDecode(v):
    return v * _STANDF_INTERVEL


def StanfordIdexIncode(v):
    return int(v / _STANDF_INTERVEL)


# 处理dataframe的情况
def StanfordIdexIncode_df(v):
    return np.trunc(v / _STANDF_INTERVEL).astype(int)


def index(x, y):
    if x >= _STANDF_MAX_PE_PL or y >= _STANDF_MAX_PE_PL:
        return (
            StanfordIdexIncode(_STANDF_MAX_PE_PL - 0.1),
            StanfordIdexIncode(_STANDF_MAX_PE_PL - 0.1))
    else:
        return StanfordIdexIncode(x), StanfordIdexIncode(y)


# 处理dataframe的情况
def index_df(df):
    df_1 = df[(df['pl'] <_STANDF_MAX_PE_PL) & (df['pe'] <_STANDF_MAX_PE_PL)].copy()   # 筛选不超过阈值的数据
    df_1['pl_index'] = StanfordIdexIncode_df(df_1['pl'])
    df_1['pe_index'] = StanfordIdexIncode_df(df_1['pe'])

    df_2 = df[(df['pl'] >= _STANDF_MAX_PE_PL) | (df['pe'] >= _STANDF_MAX_PE_PL)].copy()  # 筛选超过阈值的数据
    df_2['pl_index'] = StanfordIdexIncode_df(df_2['pl']*0 + _STANDF_MAX_PE_PL - 0.1)
    df_2['pe_index'] = StanfordIdexIncode_df(df_2['pl']*0 + _STANDF_MAX_PE_PL - 0.1)
    df = pd.concat([df_1, df_2])
    df['pl'] = np.round(df['pl'] * 2, 1) / 2
    df['pe'] = np.round(df['pe'] * 2, 1) / 2
    return df


def StanfordData(pe, pl, df):
    ne = len(pe)
    nl = len(pl)
    if ne != nl:
        print(f'pe:{ne} 个数不等于 pl:{nl}')
        return None

    # 新方法
    df_idx = index_df(df)
    data2 = df_idx.value_counts(["pe_index", "pl_index"])
    data2 = np.log(data2 + 1e-5)  # log_e
    data2 = data2.to_frame(name="color_col")
    merged_df = pd.merge(data2.reset_index(), df_idx, left_on=['pe_index', 'pl_index'], right_on=['pe_index', 'pl_index'])
    stanford_list = merged_df[['pe', 'pl', "color_col"]].values.tolist()
    max_col = max(merged_df["color_col"])
    min_col = min(merged_df["color_col"])

    # # 原方法
    # color_col = []
    # stanford_list = []
    # data1 = np.mat(zeros((_STANDF_AX_PE_NUM, _STANDF_AX_PE_NUM)))
    # print(time.strftime('%H:%M:%S', time.localtime()) + '方法2')
    # for i in range(ne):
    #     peidx, plidx = index(pe[i], pl[i])
    #     # print(peidx, plidx)
    #     data1[peidx, plidx] = data1[peidx, plidx] + 1
    # data1 = np.log(data1 + 1e-5)  # log_e
    # for i in range(ne):
    #     peidx, plidx = index(pe[i], pl[i])
    #     color_col.append(data1[peidx, plidx])
    #     # stanford_list.append([pe[i], pl[i], data1[peidx, plidx]])
    #     stanford_list.append([round(pe[i]*2, 1)/2, round(pl[i]*2, 1)/2, round(data1[peidx, plidx], 3)])  # 降低分辨率到0.05
    #     stanford_list.append([round(pe[i], 1), round(pl[i], 1), round(data1[peidx, plidx], 3)])  # 降低分辨率到0.1
    # max_col = max(color_col)
    # min_col = min(color_col)
    return stanford_list, max_col, min_col


def visual(df, fix_rate, plot_type, comment=""):
    """
    :param plot_type: 是否画图
    :param df: comumns: ['pl', 'pe', ...]
    :param fix_rate: algorithm fix rate
    :param comment: figure extend name
    :return:
    """

    # 计算MD和FA. Missed detection(MD) 漏警; False alert(FA) 误警, epoch and percentage
    print(time.strftime('%H:%M:%S', time.localtime()) + ' epoch计算')
    epoch = df.shape[0]
    fa_epoch = df[(df['pl'] > _AL) & (df['pe'] < _AL)].shape[0]
    md_epoch = df[(df['pl'] < _AL) & (df['pe'] > _AL)].shape[0]
    availability_epoch = df[(df['pl'] < _AL)].shape[0]
    # mi_epoch = (df[(df['pl'] < _AL) & (df['pe'] < _AL) & (df['pl'] < df['pe'])].shape[0] +
    #             df[(df['pl'] > _AL) & (df['pe'] > _AL) & (df['pl'] < df['pe'])].shape[0])
    mi_epoch = df[(df['pl'] < df['pe'])].shape[0]

    print(time.strftime('%H:%M:%S', time.localtime()) + ' fa,md, availability, mi计算')
    fa = fa_epoch / df.shape[0]
    md = md_epoch / df.shape[0]
    availability = availability_epoch / df.shape[0]
    mi = mi_epoch / df.shape[0]
    df['pl'], df['pe'] = df['pl'].to_numpy(), df['pe'].to_numpy()

    # 斯坦福图
    title = (f'{comment}\n '
             f'EPOCH={epoch:d}; RTK_FIX={fix_rate * 100:.2f}%;MD={md:.2e}; '
             f'FA={fa*100:.2f}%; MI={mi:.2e}; Availability={availability*100:.2f}%')

    print(time.strftime('%H:%M:%S', time.localtime()) + ' 开始斯坦福图热力度计算')
    if plot_type:
        stanford_list, max_col, min_col = StanfordData(df['pe'], df['pl'], df)
    else:
        stanford_list, max_col, min_col = [], 0, 0
    print(time.strftime('%H:%M:%S', time.localtime()) + ' 斯坦福图计算完毕')

    stanford_list = [list(t) for t in set(tuple(_) for _ in stanford_list)]  # 删除斯坦福画图列表的重复项

    stanford_plot = {"title": title, "min": min_col, "max": max_col, "limit": _AL, "values": stanford_list}
    # print("stanford_plot:", stanford_plot)

    # 统计结果
    stanford_statistics = {
        "status": comment,
        "epoch": f"{epoch:d}",
        "fix_rate": f"{fix_rate * 100:.2f}%",
        "availability": f"{availability*100:.2f}%",
        "MD_HMI": f"{md:.2e}",
        "MI": f"{mi:.2e}",
        "FA": f"{fa*100:.2f}%",
    }
    # print("stanford_stati：", result)

    return stanford_plot, stanford_statistics


def integrity_test(df, pl_name, pe_name, plot_type, fix_rate_out=nan):
    """
    斯坦福测试统计入口
    :param pe_name: PE 对应字段名
    :param pl_name: PL对应字段名
    :param fix_rate_out: 外部输入的固定率
    :param plot_type: 画图类型
    :param df: Dataframe: ['pl', 'pe', ...]
    :return plot_list: 画图值列表
    :return statistics_list：统计指列表
    """

    # Stanford 统计（ALL）
    if "state" in df:
        fix_rate = df[df["state"] == 4].shape[0] / df.shape[0]
    elif "gpsQuality" in df:
        fix_rate = df[df["gpsQuality"] == 4].shape[0] / df.shape[0]
    elif fix_rate_out:
        fix_rate = fix_rate_out
    else:
        fix_rate = nan
    df_hor = pd.DataFrame({'pl': df[pl_name], 'pe': df[pe_name]})
    plot, statistics = visual(df_hor, fix_rate, plot_type, comment="ALL")
    plot_list, statistics_list = [plot], [statistics]

    # Stanford 统计（算法内部解状态）
    for i in FilterStatus:
        if i == FilterStatus.FILTER_POS_MAX:
            break
        dfc = df[df['state_algo'] == i.value] if 'state_algo' in df else pd.DataFrame({})
        if dfc.empty:
            continue
        # Stanford 统计
        comment = i.name.split("_")[-1]
        fix_rate = dfc[dfc["state"] == 4].shape[0] / dfc.shape[0] if "state" in dfc else df[df["gpsQuality"] == 4].shape[0] / df.shape[0]
        df_hor = pd.DataFrame({'pl': dfc[pl_name], 'pe': dfc[pe_name]})
        plot, statistics = visual(df_hor, fix_rate, plot_type, comment=comment)
        plot_list.append(plot)
        statistics_list.append(statistics)

    return plot_list, statistics_list


def integrity_series(df, pl_name, pe_name):
    """
    全程时间序列图统计入口
    :param df: comumns: ['pl', 'pe', ...]
    :return plot_list: 画图值列表
    :return statistics_list：统计指列表
    """
    title = "PE/PL时间序列图"
    plot_dict = {"PE": "line", "PL": "line", "HMI": "scatter", "Not available": "scatter", "FA": "scatter", "MI": "scatter"}
    result = []
    time_name = 'itow' if 'itow' in df else "gpsItow"
    for plot in plot_dict:
        value = {"name": plot, "type": plot_dict[plot]}
        if plot == "PE":
            print(df.shape)
            value["data"] = df[[time_name, pe_name]].values.tolist()
        elif plot == "PL":
            value["data"] = df[[time_name, pl_name]].values.tolist()
        elif plot == "HMI":
            plot_df = df[(df[pl_name] <= _AL) & (df[pe_name] > _AL)].copy()
            plot_df["value"] = plot_df[pl_name] * 0 - 1
            value["data"] = plot_df[[time_name, "value"]].values.tolist()
        elif plot == "Not available":
            plot_df = df[df[pl_name] > _AL].copy()
            plot_df["value"] = plot_df[pl_name] * 0 - 2
            value["data"] = plot_df[[time_name, "value"]].values.tolist()
        elif plot == "FA":
            plot_df = df[(df[pl_name] > _AL) & (df[pe_name] <= _AL)].copy()
            plot_df["value"] = plot_df[pl_name] * 0 - 3
            value["data"] = plot_df[[time_name, "value"]].values.tolist()
        elif plot == "MI":
            # plot_df_available = df[(df[pl_name] < _AL) & (df[pe_name] < _AL) & (df[pl_name] < df[pe_name])].copy()
            # plot_df_unavailable = df[(df[pl_name] > _AL) & (df[pe_name] > _AL) & (df[pl_name] < df[pe_name])].copy()
            # plot_df = pd.concat([plot_df_available, plot_df_unavailable])
            plot_df = df[(df[pl_name] < df[pe_name])].copy()
            plot_df["value"] = plot_df[pl_name] * 0 - 4
            value["data"] = plot_df[[time_name, "value"]].values.tolist()
        result.append(value)
    return {"title": title, "value": result}


def integrity_dr_test(df, pl_name, pe_name):
    """
    dr场景完好性统计
    :param df: comumns: ['pl', 'pe', ...]
    :return statistics_list：统计指列表
    """
    # 1. 初始时刻判断PL<AL是否满足，不满足则不进行后续统计
    first_pl = df[pl_name][0]
    if first_pl >= _AL:
        return {"speeding": "NA", "firstPl": str(first_pl), "maxPl": "NA", "MI": "NA", "eligibility": "NA"}

    # 2.计算筛选出从进入隧道至在隧道内行驶411m内的所有历元
    mileage = np.cumsum(np.diff(df["unixTime"]) * np.array(df["velocity"][1:]))  # 里程计算
    df["mileage"] = np.insert(mileage, 0, 0)  # 补充0位， 统一长度
    df = df[df['mileage'] <= DR_DIST]  # 筛选出从进入隧道至在隧道内行驶411m内的所有历元

    # 3. 判断411m内，速度是否都在100km/h以下，不满足则不进行后续统计
    df_speeding = df[df['velocity'] >= 27.778]
    if not df_speeding.empty:
        return {"speeding": "否", "firstPl": str(first_pl), "maxPl": "NA", "MI": "NA", "eligibility": "NA"}

    # 4.对进入隧道后411m内的所有历元，逐个判断是否满足Lateral PL < 3m
    df_exce_pl = df[df[pl_name] >= _AL]

    # 5. 统计411m内的 MI（PL<AL）占比，判断是否满足MI<10^-4/
    mi_epoch = df[(df[pl_name] < df[pe_name])].copy()
    mi = len(mi_epoch) / df.shape[0]

    # 统计411m内最大PL
    max_pl = df[pl_name].max()

    if (not df_exce_pl.empty) or mi > 0:
        return {"speeding": "是", "firstPl": str(first_pl), "maxPl": str(round(max_pl, 2)), "MI": str(round(mi*100, 4)) + "%", "eligibility": "不达标"}
    else:
        return {"speeding": "是", "firstPl": str(first_pl), "maxPl": str(round(max_pl, 2)), "MI": str(round(mi*100, 4)) + "%", "eligibility": "达标"}


if __name__ == "__main__":
    print(time.strftime('%H:%M:%S', time.localtime()) + ' 开始完好性统计')

    df_paths = [
        # r"E:\PycharmProjects\TestProject\Datasets\PPP2022121712345678\main-pb_config_ppp_wenzixuan_20231017175003616_log-full-20221230-070944\log-full-20221230-070944.dcp.pl.feather",
        # r"E:\PycharmProjects\TestProject\Datasets\PPP2022121712345678\main-pb_config_ppp_wenzixuan_20231017175003616_log-full-20221230-070944\log-full-20221230-070944.ppp.pl.feather",
        r"C:\Users\wenzixuan\Downloads\PPP20231012153822236\PPP20231012153822236_f7bac4b_multi_freq-pb_config_ins2_ipc_ins_lc_ipclog_lishanshan_20231121102933556_Loc_Core_Debug_3469\bddb-ins.pl.feather"
    ]
    print(time.strftime('%H:%M:%S', time.localtime()) + ' ' + str(len(df_paths)) + "份测试数据")

    # 组合完好性数据
    print(time.strftime('%H:%M:%S', time.localtime()) + ' 开始读取数据')
    df_list = [pd.read_feather(path) for path in df_paths]
    pl_df = pd.concat(df_list)
    pl_df = pl_df.reset_index(drop=True)
    print(time.strftime('%H:%M:%S', time.localtime()) + ' 读取数据完毕')

    # 统计字段名称
    pl, pe = "pos_horizontal_pl", "pos_horizontal_pe"
    plot_type = None  # "stanford"

    # # 开始完好性统计
    # plot_list, statistics_list = integrity_test(pl_df, pl, pe)
    # # print("plot_list:",  plot_list)
    # print("statistics_list:", statistics_list)
    # print(time.strftime('%H:%M:%S', time.localtime()) + ' 完好性统计结束')

    # 时间序列图统计
    pepl_list = integrity_series(pl_df, pl, pe) if 'itow' in pl_df else None
    print("peplPlot:", pepl_list)
    print(time.strftime('%H:%M:%S', time.localtime()) + ' 时间序列图统计统计结束')


