import numpy
import pandas as pd
from matplotlib import pyplot as plt

from scripts.robot import Robot
from scripts.ndi_utilities import read_raw_ndi_file
from scripts.robot_utilities import read_json_file
from scripts.process_data_utilities import *
numpy.set_printoptions(suppress=True)

if __name__ == "__main__":
    data_path = 'data/process_data_utilities_test/1.csv'
    param_path = 'data/process_data_utilities_test/param.json'
    params = read_json_file(param_path)
    
    state = params["data_state"]
    cols = params["data_columns"]
    data = read_raw_ndi_file(data_path, state=state, columns=cols)
    
    #Extract data point
    data_param = params["data"]["1.csv"]
    pts1 = extract_measurement_points(data, data_param, plot=True)
    
    #Extract data point2
    ind2, pts2 = get_interval_points(data[:,0], 65, 1000, 5)
    ind3, pts3 = take_average_around_point(data[:,0], ind2, L=40)
    pd.DataFrame(data[:,0]).plot(legend=False)
    plt.scatter(ind2, pts2, label="get_interval_points")
    plt.scatter(ind3, pts3, label="take_average_around_point", alpha=0.7)
    plt.legend()
    
    #Point cloud matching
    A = numpy.random.rand(30,3)
    R = Robot.Rx(1.0)@Robot.Rz(0.8)
    B = A@R
    M1 = get_point_cloud_transformation_matrix(A, B)
    C = transform_point_cloud(A, B, M1, plot=True)
    R1 = M1[:3,:3]
    print("Equal: ", Robot.equal(R, R1.transpose()))
    
    plt.show()
    
    