import numpy
import pandas as pd
from enum import Enum
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as ScipyR

from .plot_utilities import plot_data, plot_with_scatters

class AddPointMethod(Enum):
    MIDDLE = 0
    AVERAGE = 1

def extract_measurement_points(data, params, plot=False):
    """
    Extract measuremnet points from data
    Args:
        data: data array with each row in (x,y,z,qx,qy,qz,qw) format
        params: paramter dictionary
        tool: used ndi tool
        robot: used robot
        plot: plot extracted points for debug
    Returns:
        measured_pts: extracted measurement points
    """
    n = params["total_cycles"] #No of cycles
    c1, c2, c3 = params["get_constant_points"]
    start, end = params["extract_data_range"]
    point_per_cycle = params["point_per_cycle"]
    reference =  params["get_constant_points_reference"]
    
    #Extract measured points
    indexs = get_constant_points(data[:,reference], c1, c2, c3, AddPointMethod.MIDDLE)[0]
    indexs = indexs[start:end]
    measured_pts = data[indexs, :]
    print("Total number of extracted points: ", len(indexs))
    
    if plot:
        ############## Plot extracted points position and quaternion ############### 
        df1 = pd.DataFrame(data[:,:3], columns=['x','y','z'])
        df2 = pd.DataFrame(data[:,3:], columns=['qx','qy','qz','qw'])
        plot_with_scatters(df1, indexs)
        plt.title("extract_measurement_points: position")
        plot_with_scatters(df2, indexs)
        plt.title("extract_measurement_points: quaternion")

    if len(indexs) != n*point_per_cycle:
        print("Error: Number of extracted points is wrong!")
        raise RuntimeError
    return measured_pts

def get_constant_points(data, t, L, delta, method: AddPointMethod):
    """
    Extract points from data within a constant period
    Args:
        data: data for extracting point
        t: time for caculating average 
        L: minimum point size required for a valid constant period
        dealta: threshold for considered as staying constant 
        method: method for adding the point
    Returns:
        (index, points):
        index = index of points in data \n
        points = vector of points
    """
    
    def add_point(data, start, end, L, method: AddPointMethod, res):
        """
        Add the middle index point of a period to res
        Args:
            data: data for extracting point
            start: start index of data
            end: end index of data
            L: minimum point size required for a valid period
            method: method for calculating the point value
            res: array to add
        """
        if (start != None):
            if (end - start > L):
                if method == AddPointMethod.MIDDLE:
                    i = (start+end)//2
                    res.append((i, data[i]))
                elif method == AddPointMethod.AVERAGE:
                    i = (start+end)//2
                    res.append((i, sum(data[start:end])/(end-start)))
        
    res = []
    start = end = None
    for i, x in enumerate(data):
        ave = sum(data[i-t:i]/t)
        if abs(x - ave) <= delta:
            if start == None:
                start = end = i
            else:
                end = i
        else:
            add_point(data, start, end, L, method, res)
            start = end = None
    add_point(data, start, end, L, method, res)
    index, points = list(zip(*res))
    return list(index), list(points)

def get_interval_points(data, start, L, n):
    """
    Get n points from data with a fixed time interval.
    Args:
        t: start time
        L: time interval between two points
        n: total no. of points
    Returns:
        (index, points):
        index = indexes of point \n
        points = values of point
    """
    res = []
    for i in range(n):
        ind = start+int(i*L)
        res.append((ind, data[ind]))
    index, points = list(zip(*res))
    return list(index), list(points)

def take_average_around_point(data, indexes, L):
    """
    Take average of points 
    Args:
        indexes: point indexes
        L: range for taking average
    Returns:
        (index, points):
        index = indexes of point \n
        points = values of point
    """
    res = []
    l, r = L//2, (L+1)//2
    for i in indexes:
        ave = numpy.average(data[i-l:i+r])
        # outliers = set([i for i in range(i-l, i+r) if abs(data[i]-ave) > 5])
        # if outliers:
        #     ave = sum([data[i] for i in range(i-l, i+r) if i not in outliers])/len(outliers)
        res.append((i, ave))
    index, points = list(zip(*res))
    return list(index), list(points)

def get_point_cloud_transformation_matrix(A, B):
    """
    Get the transformation matrix from point cloud A to point cloud B by calculating SVD of their positions.
    The algorithm fails if points are collinear, i.e. det(R) == -1 and none of singular value of H is 0.
    Args:
        A: position array of point cloud A in [nx3] shape. n is the total number of points.
        B: position array of point cloud B in [nx3] shape. n is the total number of points.
    Returns:
        result: [4x4] transformation matrix
    Reference:
        [K. S. Arun, T. S. Huang, and S. D. Blostein, “Least-Squares Fitting of Two 3-D Point Sets,”
        IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. PAMI-9, no. 5, pp. 698–700,
        May 1987]
    """
    result = []
    A, B = pd.DataFrame(A, columns=['x','y','z']), pd.DataFrame(B, columns=['x','y','z'])
    COG_A = A.mean(axis=0).to_numpy()
    COG_B = B.mean(axis=0).to_numpy()
    a, b = A.to_numpy(), B.to_numpy()
    H = numpy.matmul(numpy.transpose((a - COG_A)), (b - COG_B))/len(A) #covariance matrix
    u, s, vh = numpy.linalg.svd(H, full_matrices=True)
    R = numpy.matmul(numpy.transpose(vh), numpy.transpose(u))

    #Check if the SVD returned a ‘reflection’ matrix
    if numpy.linalg.det(R) < 0:
        print("Reflection matrix")
        vh[2,:] *= -1
        R = numpy.matmul(numpy.transpose(vh), numpy.transpose(u))
    P = COG_B - numpy.matmul(R, COG_A)
    result = numpy.eye(4)
    result[0:3, 0:3] = R
    result[0:3, 3] = P
    return result

def get_point_cloud_transformation_matrix1(A,B):
    """
    Get the transformation matrix from point cloud A to point cloud B
    Args:
        A: position array of point cloud A in [nx3] shape. n is the total number of points.
        B: position array of point cloud B in [nx3] shape. n is the total number of points.
    Returns:
        result: [4x4] transformation matrix
    Referenece:
        [J. Cashbaugh and C. Kitts, "Automatic Calculation of a Transformation Matrix Between 
        Two Frames," in IEEE Access, vol. 6, pp. 9614-9622, 2018, doi: 10.1109/ACCESS.2018.2799173.]
    """  
    a1 = numpy.column_stack((A,numpy.ones(A.shape[0])))
    b1 = numpy.column_stack((B,numpy.ones(B.shape[0])))

    matrixA = numpy.tensordot(a1,a1.T,axes=[0,1])
    matrixR = numpy.tensordot(a1,b1.T,axes=[0,1])
    matrixAinv = numpy.linalg.inv(matrixA)

    tm = numpy.tensordot(matrixAinv,matrixR,axes=1).transpose()
    return tm

def get_point_cloud_transformation_matrix2(A,B):
    """
    Get the transformation matrix from point cloud A to point cloud B by Least-Squares Fitting using
    Unit Quaternion. It avoids reflections as occured in SVD method. However, the advantage of SVD 
    over unit quaternions is that the former generalizes well to n dimensions and would be an ideal
    approach in case n > 3.
    Args:
        A: position array of point cloud A in [nx3] shape. n is the total number of points.
        B: position array of point cloud B in [nx3] shape. n is the total number of points.
    Returns:
        result: [4x4] transformation matrix
    Reference:
        [P. J. Besl and N. D. McKay, “A method for registration of 3-D shapes,” IEEE Transactions
        on Pattern Analysis and Machine Intelligence, vol. 14, no. 2, pp. 239–256, Feb. 1992]
    """
    result = []
    A, B = pd.DataFrame(A, columns=['x','y','z']), pd.DataFrame(B, columns=['x','y','z'])
    COG_A = A.mean(axis=0).to_numpy()
    COG_B = B.mean(axis=0).to_numpy()
    a, b = A.to_numpy(), B.to_numpy()
    H = numpy.matmul(numpy.transpose((a - COG_A)), (b - COG_B))/len(A) #covariance matrix
    anti_sym = H - H.transpose()
    delta = numpy.array([anti_sym[1,2], anti_sym[2,0], anti_sym[0,1]]).transpose()
    Q = numpy.zeros((4,4))
    Q[0,0] = numpy.trace(H)
    Q[0,1:] = delta.transpose()
    Q[1:,0] = delta
    Q[1:, 1:] = H + H.transpose() - numpy.trace(H)*numpy.eye(3)
    eig_vals, eig_vecs =  numpy.linalg.eig(Q)
    largest_eigenvector = eig_vecs[:, numpy.argmax(eig_vals)]
    qw, qx, qy, qz = largest_eigenvector
    R = ScipyR.from_quat([qx,qy,qz,qw]).as_matrix()
    P = COG_B - numpy.matmul(R, COG_A)
    result = numpy.eye(4)
    result[0:3, 0:3] = R
    result[0:3, 3] = P
    return result

def transform_point_cloud(A, B, M, print_err=False, plot=False, print_outliders=False):    
    """
    Transform point cloud A with transformation matrix M. And compare the differences.
    Args:
        A: point cloud A in [nx3] shape, where n is the total number of points.
        B: point cloud B in [nx3] shape, where n is the total number of points.
        M: transformation matrix from A to B.
    Returns:
        C: transformed point cloud of A.
    """
    R = M[0:3, 0:3]
    P = M[0:3, 3]
    C = numpy.transpose(numpy.matmul(R, numpy.transpose(A[:,:3]))) + P
    if A.shape[-1] > 3:
        C_rot = numpy.zeros((len(A), 3))
        for i in range(len(A)):
            axis_angle = A[i, 3:]
            R0 = ScipyR.from_rotvec(axis_angle).as_matrix()
            R1 = R0@R
            C_rot[i] = ScipyR.from_matrix(R1).as_rotvec()
        C = numpy.concatenate((C, C_rot), axis=1)
        
    if print_err:
        err = numpy.linalg.norm(C-B, axis=1)
        idd = numpy.argmax(err)
        mean,sd = sum(err)/len(err), numpy.std(err)
        print(f"\nPoint set SVD matching:\nErr mean: {mean}, Err sd: {sd}")
        print(f"Max Err: {max(err)}, Max Err id: {idd},\nMin Err: {min(err)}, Average Err: {sum(err)/len(err)}")
        
    if print_outliders:
        # Check for outlier
        outliers_ind = [i for i in range(len(err)) if err[i] > mean+0.5*sd]
        print(f"Outlier is defined as error > mean+0.5sd")
        print(f"No. of outliers: {len(outliers_ind)}\nOutliers err: {[err[i] for i in outliers_ind]}")
        
    ###################### Plot Points ########################
    if plot:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(A[:5, 0], A[:5, 1], A[:5, 2], c=numpy.arange(5), cmap='viridis' )
        ax.scatter3D(B[:5, 0], B[:5, 1], B[:5, 2], c=numpy.arange(5), cmap='viridis', marker='X', s=60)
        ax.scatter3D(C[:5, 0], C[:5, 1], C[:5, 2], c=numpy.arange(5), cmap='viridis', marker='^')
        ax.plot3D(A[:, 0], A[:, 1], A[:, 2], label="measurement", alpha=0.3)
        ax.plot3D(B[:, 0], B[:, 1], B[:, 2], label="norm", linewidth=2.0)
        ax.plot3D(C[:, 0], C[:, 1], C[:, 2], label="SVD fit", alpha=0.8)
        fig.tight_layout()
        fig.subplots_adjust(right=0.8)
        ax.legend(loc='center left', bbox_to_anchor=(1.07, 0.5), fontsize=7)
    return C
