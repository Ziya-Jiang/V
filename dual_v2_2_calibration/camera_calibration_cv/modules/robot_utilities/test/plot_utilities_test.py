import numpy
import pandas as pd
from matplotlib import pyplot as plt

from scripts.cursor import Cursor
from scripts.plot_utilities import set_figure_location, plot_with_scatters, plot_bar_chart, \
    print_cursor_position_on_click, plot_data, plot_pose_by_axis

if __name__ == "__main__":
    data = numpy.random.rand(3,10)
    df = pd.DataFrame(data.transpose(), columns=['A','B','C'])
    
    #------------------------Figure. 1-------------------------
    plot_data(data, labels=['X','Y','Z'])
    #Display figure on given location
    set_figure_location(plt.gcf(), 1000, 500)
    #Right click to print the cursor position
    cursor = Cursor(plt.gca())
    print_cursor_position_on_click(cursor)
    
    #------------------------Figure. 2-------------------------
    #Scatter given indexs
    plot_with_scatters(df, [1,3,5,7])
    
    #------------------------Figure. 2-------------------------
    #Plot bar chart
    titles = ['plot1', 'plot2']
    data_labels = ['A', 'B', 'C']
    x_axis_labels = ['grp1', 'grp2']
    data1 = numpy.random.rand(len(titles),len(data_labels),len(x_axis_labels))
    print("data1: \n", data1[0])
    plot_bar_chart(data1, titles, data_labels, x_axis_labels, 1, 2)
    
    #------------------------Figure. 4-------------------------
    data2 = numpy.random.rand(6,10)
    plot_pose_by_axis(data2, "Pose with (x, y, z, Rx, RY, Rz) format")
    
    plt.show()
