import matplotlib.pyplot as plt
import mplcursors
import pandas as pd
import numpy as np


class PlotGpsInsData:
    def __init__(self, parent=None):
        super().__init__()
        # 实例化数据解析对象，默认1个figure只有1个图像
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.window.showMaximized()

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

    def call_back_xy(self, event):
        # 使用鼠标滚轮等比例放大/缩小图像的XY轴
        axtemp = event.inaxes
        x_min, x_max = axtemp.get_xlim()
        y_min, y_max = axtemp.get_ylim()
        xscale = (x_max - x_min) / 10
        yscale = (y_max - y_min) / (x_max - x_min) * (x_max - x_min) / 10
        if event.button == 'up':
            axtemp.set(xlim=(x_min + xscale, x_max - xscale))
            axtemp.set(ylim=(y_min + yscale, y_max - yscale))
            print('up')
        elif event.button == 'down':
            axtemp.set(xlim=(x_min - xscale, x_max + xscale))
            axtemp.set(ylim=(y_min - yscale, y_max + yscale))
            print('down')
        self.fig.canvas.draw_idle()  # 绘图动作实时反映在图像上

    def PlotData(self, ax, x, y, label):
        # 线性图绘图
        # ax:绘图, x：自变量, y：因变量, label：因变量标签
        lines = ax.plot(x, y, label=label, linewidth=1, alpha=0.7)
        self.fig.canvas.mpl_connect('scroll_event', self.call_back)
        mplcursors.cursor(lines, multiple=True)  # 数据游标

    def PlotDataXY(self, ax, x, y, label, labels, color, linestyle, marker):
        # 绘制等比例图 + 游标显示标签信息
        # ax:绘图, x：自变量, y：因变量, label：因变量标签, labels:其他标签, linestyle:线条格式, marker：
        ax.axis('equal')
        ax = plt.gca()
        if color is not None:
            lines = ax.plot(x, y, label=label, linewidth=1, linestyle=linestyle, marker=marker, color=color, alpha=0.7)
        else:
            lines = ax.plot(x, y, label=label, linewidth=1, linestyle=linestyle, marker=marker, alpha=0.7)
        self.fig.canvas.mpl_connect('scroll_event', self.call_back_xy)
        if labels:
            cursor = mplcursors.cursor(lines, hover=True)  # 数据游标
            cursor.connect("add", lambda sel: sel.annotation.set_text(labels[sel.target.index]))

    def ShowPlotFormat(self, xlabel, ylabel, hspace=0.5):
        # 统一绘图样式设置
        # 设置（xlabel：X坐标轴名称，ylabel：Y坐标轴名称）
        plt.rc("font", family='MicroSoft YaHei', weight='bold')
        plt.subplots_adjust(hspace=hspace)
        plt.autoscale(enable=True, axis='y')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid()
        plt.legend(loc='best')


if __name__ == "__main__":
    data = pd.DataFrame(np.arange(18).reshape(3, 6), index=['a', 'b', 'c'], columns=['A', 'B', 'C', 'D', 'E', 'F'])
    obj = PlotGpsInsData()
    obj.fig.suptitle('This is the main title')
    obj.ax1 = plt.subplot(3, 1, 1)
    obj.ax1.set_title('This is the ax1 title')
    obj.PlotData(obj.ax1, data['A'], data['B'], 'test1')
    obj.PlotData(obj.ax1, data['C'], data['B'], 'test2')
    obj.ShowPlotFormat('', 'C')
    plt.show()
