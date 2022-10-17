from matplotlib.widgets import Cursor
import mplcursors
import matplotlib.pyplot as plt
import numpy as np
from pylab import *
import math

def test(name: object = 'aa') -> object:
    X = np.linspace(-np.pi, np.pi, 256, endpoint=True)
    C, S = np.cos(X), np.sin(X)

    def fig1():
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, facecolor='#FFFFCC')
        ax.plot(X, C, color="blue", linewidth=2.5, linestyle="-")
        ax.plot(X, S, color="red", linewidth=2.5, linestyle="-")
        cursor0 = Cursor(ax, horizOn=False, vertOn=True, useblit=False, color='grey', linewidth=1, linestyle='--')
        return cursor0

    def fig2(name='test'):
        fig = plt.figure(figsize=(5, 5))
        ax = fig.add_subplot(111)
        line1 = ax.plot(C, X, color="blue", linewidth=2.5, linestyle="-")
        line2 = ax.plot(S, X, color="red", linewidth=2.5, linestyle="-")
        ax.set_xlabel(' ')
        ax.set_ylabel('xxx')
        ax.autoscale(enable=True, axis='y')
        ax.grid()
        ax.legend(loc='best')
        ax.set_title(name)

        labels = [i for i in range(len(X), 0)]
        cursor_1 = mplcursors.cursor(line1, hover=True)  # 数据游标
        # cursor_1.connect("add", lambda sel: sel.annotation.set_text(labels[sel.index]))
        cursor_1.connect(event="add"
                         , func=lambda sel: sel.annotation.set_text('label:{}'.format(labels[math.floor(sel.target.index)])))

        labels = [i for i in range(len(X))]
        cursor_2 = mplcursors.cursor(line2, hover=True)  # 数据游标
        # cursor_1.connect("add", lambda sel: sel.annotation.set_text(labels[sel.index]))
        cursor_2.connect(event="add"
                         , func=lambda sel: sel.annotation.set_text(
                'label:{}'.format(labels[math.floor(sel.target.index)])))

        cursor1 = Cursor(ax, horizOn=False, vertOn=True, useblit=False, color='black', linewidth=1, linestyle='--')
        return cursor1

    cursor0 = fig1()
    cursor1 = fig2()

    plt.show()
    print('hh')


if __name__ == "__main__":
    test()

    # from matplotlib import pyplot as plt
    #
    # from matplotlib.patheffects import withSimplePatchShadow
    # import mplcursors
    # from pandas import DataFrame
    #
    # df = DataFrame(
    #     dict(
    #         Suburb=["Ames", "Somerset", "Sawyer"],
    #         Area=[1023, 2093, 723],
    #         SalePrice=[507500, 647000, 546999],
    #     )
    # )
    #
    # df.plot.scatter(x="Area", y="SalePrice", s=100)
    #
    # def show_hover_panel(get_text_func=None):
    #     cursor = mplcursors.cursor(
    #         hover=2,  # Transient
    #         annotation_kwargs=dict(
    #             bbox=dict(
    #                 boxstyle="square,pad=0.5",
    #                 facecolor="white",
    #                 edgecolor="#ddd",
    #                 linewidth=0.5,
    #                 path_effects=[withSimplePatchShadow(offset=(1.5, -1.5))],
    #             ),
    #             linespacing=1.5,
    #             arrowprops=None,
    #         ),
    #         highlight=True,
    #         highlight_kwargs=dict(linewidth=2),
    #     )
    #
    #     if get_text_func:
    #         cursor.connect(
    #             event="add",
    #             func=lambda sel: sel.annotation.set_text(get_text_func(sel.index)),
    #         )
    #         print('gg')
    #
    #     return cursor
    #
    #
    # def on_add(index):
    #     item = df.iloc[index]
    #     parts = [
    #         f"Suburb: {item.Suburb}",
    #         f"Area: {item.Area:,.0f}m²",
    #         f"Sale price: ${item.SalePrice:,.0f}",
    #     ]
    #
    #     return "\n".join(parts)
    #
    #
    # show_hover_panel(on_add)
    # plt.show()
    print('over')
