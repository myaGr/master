import matplotlib.pyplot as plt
import mplcursors
import pandas as pd
import numpy as np
from matplotlib.dates import DateFormatter, drange
import matplotlib.dates as mdates


# 统计画图功能
class PlotData:
    def __init__(self):
        super().__init__()
        # 实例化数据解析对象，默认1个figure只有1个图像
        self.fig, self.ax = plt.subplots()
        # self.fig.canvas.manager.window.showMaximized() # 自动最大化

    def call_back(self, event):
        # 使用鼠标滚轮放大/缩小图像的X轴
        axtemp = event.inaxes
        x_min, x_max = axtemp.get_xlim()
        scale = (x_max - x_min) / 10
        if event.button == 'up':
            axtemp.set(xlim=(x_min + scale, x_max - scale))
            print('up')
        elif event.button == 'down':
            axtemp.set(xlim=(x_min - scale, x_max + scale))
            print('down')
        self.fig.canvas.draw_idle()  # 绘图动作实时反映在图像上

    def PlotData(self, ax, x, y, label, labels=None, labels_format='{}'):
        # 线性图绘图
        # ax:绘图轴, x：自变量, y：因变量, label：因变量标签
        lines = ax.plot(x, y, label=label, linewidth=1, alpha=0.55)
        # self.fig.canvas.mpl_connect('scroll_event', self.call_back)
        cursor = mplcursors.cursor(lines, multiple=True, bindings={'left': "shift+left", 'right': "shift+right"})  # 数据游标
        if labels:
            cursor.connect("add", lambda sel: sel.annotation.set_text(labels_format.format(labels[int(sel.target.index)])))

    def ShowPlotFormat(self, xlabel, ylabel, hspace=0.5, legend=True, legend_loc=None):
        # 统一绘图样式设置
        # 设置（xlabel：X坐标轴名称，ylabel：Y坐标轴名称）
        plt.rc("font", family='MicroSoft YaHei', weight='bold')
        plt.subplots_adjust(hspace=hspace)
        plt.autoscale(enable=True, axis='y')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid()
        if legend:
            if not legend_loc:
                plt.legend(loc='best')
            else:
                plt.legend(bbox_to_anchor=(1.05, 1.1), loc=legend_loc)

    def PlotPoint(self, ax, x, y, label, marker='*', color='r', labels=None, labels_format='{}'):
        # 线性图绘图
        # ax:绘图轴, x：自变量, y：因变量, label：因变量标签
        lines = ax.plot(x, y, label=label, linestyle="", marker=marker, alpha=0.3, color=color)
        if labels:
            self.fig.canvas.mpl_connect('scroll_event', self.call_back)
            cursor = mplcursors.cursor(lines, multiple=True, bindings={'left': "shift+left", 'right': "shift+right"})  # 数据游标
            cursor.connect("add", lambda sel: sel.annotation.set_text(labels_format.format(labels[int(sel.target.index)])))


if __name__ == "__main__":
    data = pd.DataFrame(np.arange(18).reshape(3, 6), index=['a', 'b', 'c'], columns=['A', 'B', 'C', 'D', 'E', 'F'])
    obj = PlotData()
    obj.fig.suptitle('This is the main title')
    obj.ax1 = plt.subplot(3, 2, 1)
    obj.ax1.set_title('This is the ax1 title')
    obj.PlotData(obj.ax1, data['A'], data['B'], 'test1')
    obj.PlotData(obj.ax1, data['C'], data['B'], 'test2')
    obj.ShowPlotFormat('', 'C')
    plt.show()
