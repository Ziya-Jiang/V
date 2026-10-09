import numpy
import matplotlib
from matplotlib import pyplot as plt
import pandas as pd

from .cursor import Cursor

def plot_data(data, labels=[]):
    plt.figure()
    if isinstance(data, numpy.ndarray):
        data = data.tolist()
    if not labels or len(labels) != len(data):
        labels = [""]*len(data)
    T = numpy.arange(len(data[0]))
    for i in range(len(data)):
        plt.plot(T, data[i], label = labels[i])
    plt.legend()

def plot_pose_by_axis(pts, plot_title):
    n = len(pts)
    titles = ['x', 'y', 'z', 'Rx', 'Ry', 'Rz']
    titles = [x +' values' for x in titles]
    plt.figure()
    for i in range(6):
        ax = plt.subplot(2,3,i+1)
        data = [cycle[i] for cycle in pts]
        plt.scatter(numpy.arange(n), data)
        ax.set_title(titles[i])
        plt.suptitle(plot_title)
        
def plot_bar_chart(data, titles, data_labels, x_axis_labels, sub_row, sub_col):
    """
    Plot bar chart
    Args:
        data: (n x m x k) shape, where n = no. of subplot, m = no. of 
            data type, and k = no. of x-axis type
        titles: titles of n subplot
        data_labels: labels of m data type
        x_axis_labels: labels of k x-axis type
        sub_row: no. of subplot rows
        sub_col: no. of subplot columns
    """
    fig, axs = plt.subplots(sub_row, sub_col, layout='constrained')
    for i in range(len(titles)):
        plot_data = data[i]
        ax = axs.ravel()[i]
        x = numpy.arange(len(x_axis_labels))  # the label locations
        width = 0.25  # the width of the bars
        multiplier = 0
        for j, one_type_data in enumerate(plot_data):
            offset = width * multiplier
            rects = ax.bar(x + offset, one_type_data, width, label=data_labels[j])
            ax.bar_label(rects, padding=0, fmt='%.3f', rotation=90, fontsize=9)
            multiplier += 1# Add some text for labels, title and custom x-axis tick labels, etc.
        ax.set_xticks(x + width, x_axis_labels)
        ax.set_title(titles[i])
        ax.legend(loc='upper left', ncols=3)
        ax.set_ylim([min(0, numpy.min(plot_data)*1.3), max(numpy.max(plot_data)*1.3, 0)])
        
def plot_with_scatters(df:pd.DataFrame, scatter_indexs):
    df.plot()
    for i in range(df.shape[1]):
        plt.scatter(scatter_indexs, df.iloc[scatter_indexs, i], s=30, alpha=0.5)
        
def set_figure_location(fig, x, y):
    """
    Display figure on given location
    Args:
        fig: figure for move
        x: x coordinate of the figure upper left corner
        y: y coordinate of the figure upper left corner
    """ 
    backend = matplotlib.get_backend()
    if backend == 'TkAgg':
        fig.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
    elif backend == 'WXAgg':
        fig.canvas.manager.window.SetPosition((x, y))
    else:
        fig.canvas.manager.window.move(x, y)
        
def print_cursor_position_on_click(blitted_cursor:Cursor):
    """ Print cursor position when right click """
    fig = plt.gcf()
    fig.canvas.mpl_connect('motion_notify_event',
                            blitted_cursor.on_mouse_move)
    fig.canvas.mpl_connect('button_press_event',
                            blitted_cursor.on_right_click)
    