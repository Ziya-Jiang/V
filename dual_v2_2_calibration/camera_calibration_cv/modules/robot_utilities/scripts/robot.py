import numpy
from abc import ABC, abstractmethod

class Robot(ABC):
    JOINT_LIMIT_L = None
    JOINT_LIMIT_R = None
    
    @staticmethod
    def sk(w):
        """ skew-symmetric matrix representation of vector w """
        return numpy.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

    @staticmethod
    def dRx(alpha):
        ans = numpy.array([[0,                 0,                 0],
                           [0, -numpy.sin(alpha), -numpy.cos(alpha)],
                           [0, +numpy.cos(alpha), -numpy.sin(alpha)]])
        return ans
    
    @staticmethod
    def dRy(beta):
        ans = numpy.array([[-numpy.sin(beta), 0, +numpy.cos(beta)],
                           [0               , 0,                0],
                           [-numpy.cos(beta), 0, -numpy.sin(beta)]])
        return ans

    @staticmethod
    def dRz(theta):
        ans = numpy.array([[-numpy.sin(theta), -numpy.cos(theta), 0],
                           [+numpy.cos(theta), -numpy.sin(theta), 0],
                           [0,                 0                , 0]])
        return ans

    @staticmethod
    def Rx(alpha):
        ans = numpy.array([[1,                 0,                 0],
                           [0, +numpy.cos(alpha), -numpy.sin(alpha)],
                           [0, +numpy.sin(alpha), +numpy.cos(alpha)]])
        return ans

    @staticmethod
    def Ry(beta):
        ans = numpy.array([[+numpy.cos(beta), 0, +numpy.sin(beta)],
                           [0               , 1,                0],
                           [-numpy.sin(beta), 0, +numpy.cos(beta)]])
        return ans

    @staticmethod
    def Rz(theta):
        ans = numpy.array([[+numpy.cos(theta), -numpy.sin(theta), 0],
                           [+numpy.sin(theta), +numpy.cos(theta), 0],
                           [0,                 0                , 1]])
        return ans

    @staticmethod
    def dDx():
        return numpy.array([1, 0, 0])

    @staticmethod
    def dDy():
        return numpy.array([0, 1, 0])

    @staticmethod
    def dDz():
        return numpy.array([0, 0, 1])

    @staticmethod
    def equal(M1, M2, eps=0.001):
        """Check if two matrixs(1D or 2D) are equal."""
        arr1 = numpy.array(M1)
        arr2 = numpy.array(M2)
        if (not arr1.size == arr2.size):
            return False
        if arr1.ndim == 1 :
            for i in range(arr1.size):
                if abs(arr1[i] - arr2[i]) > eps:
                    return False
        else:
            for i in range(arr1.shape[0]):
                    for j in range(arr1.shape[1]):
                        if abs(arr1[i][j] - arr2[i][j]) > eps:
                            return False
        return True

    @staticmethod
    def RPY_to_R(roll, pitch, yaw):
        """
        Convert from roll,pitch,yaw to rotation matrix
        Args:
            roll: rotation about x axis
            pitch: rotation about y axis
            yaw: rotation about z axis
        Returns:
            R: rotation matrix
        """
        sA, cA = numpy.sin(roll), numpy.cos(roll)
        sB, cB = numpy.sin(pitch), numpy.cos(pitch)
        sC, cC = numpy.sin(yaw), numpy.cos(yaw)
        Rx = numpy.array([[1,0,0], [0,cA,-sA], [0,sA,cA]])
        Ry = numpy.array([[cB,0,sB], [0,1,0], [-sB, 0, cB]])
        Rz = numpy.array([[cC,-sC,0], [sC,cC,0], [0,0,1]])
        R = numpy.dot(Rx,Ry)
        R = numpy.dot(R,Rz)
        return R

    @staticmethod
    def R_to_RPY(R):
        r00,r01,r02,r10,r11,r12,r20,r21,r22 = R.flatten()
        r = numpy.arctan2(-r12,r22)
        p = numpy.arctan2(r02, numpy.sqrt(r01*r01+r00*r00))
        y = numpy.arctan2(-r01, r00)
        return r,p,y

    @staticmethod
    def axis_angle_to_R(axis_angle):
        angle = numpy.linalg.norm(axis_angle)
        
        if angle < 1e-5:
            return numpy.eye(3)
        
        axis  = axis_angle/angle
        cross_product_mat_axis = numpy.array([[0, -axis[2], axis[1]],
                                        [axis[2], 0, -axis[0]],
                                        [-axis[1], axis[0], 0]])
        return numpy.cos(angle)*numpy.eye(3) + numpy.sin(angle) * cross_product_mat_axis \
                + (1.0 - numpy.cos(angle))*numpy.outer(axis,axis)

    @staticmethod
    def R_to_axis_angle(R, eps=0.0001):
        r00,r01,r02,r10,r11,r12,r20,r21,r22 = R.flatten()
        angle = numpy.arccos((numpy.trace(R)-1)/2)
        Rt = numpy.transpose(R)
        if numpy.allclose(R, Rt, eps, eps):
            w,v = numpy.linalg.eig(R)
            i = numpy.where(w == 1)[0]
            uV = v[:,i]
        else:
            uV = numpy.array([r21-r12, r02-r20, r10-r01])*(1/(2*numpy.sin(angle)))
        return list(angle*uV)
        
    @staticmethod
    def R_to_quat(R):
        """ Given column-major rotation matrix, return quaternion in (x,y,z, w) order """
        R = numpy.transpose(R) #To row major R
        m00, m01, m02, m10, m11, m12, m20, m21, m22 = R.flatten()
        if m22 < 0:
            if m00 > m11:
                t = 1 + m00 - m11 - m22
                q = numpy.array([t, m01+m10, m20+m02, m12-m21])
            else:
                t = 1 - m00 + m11 - m22
                q = numpy.array([m01+m10, t, m12+m21, m20-m02])
        else:
            if m00 < -m11:
                t = 1 - m00 - m11 + m22
                q = numpy.array([m20+m02, m12+m21, t, m01-m10])
            else:
                t = 1 + m00 + m11 + m22
                q = numpy.array([m12-m21, m20-m02, m01-m10, t])
        q *= 0.5 / numpy.sqrt(t)
        return q

    @staticmethod
    def quat_to_R(quat):
        """ Given quaternion in (x,y,z, w) order, return column-major rotation matrix """
        x, y, z, w = quat[0], quat[1], quat[2], quat[3]
        s = numpy.linalg.norm(quat)**(-2)
        r00 = 1-2*s(y*y-z*z)
        r01 = 2*s(x*y-w*z)
        r02 = 2*s(x*z+w*y)
        r10 = 2*s(x*y+w*z)
        r11 = 1-2*s(x*x-z*z)
        r12 = 2*s(y*z-w*x)
        r20 = 2*s(x*z-w*y)
        r21 = 2*s(y*z+w*x)
        r22 = 1-2*s(x*x-y*y)
        R = numpy.array([[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]])
        return R
    
    @abstractmethod
    def FK():
        pass
    
    @abstractmethod
    def IK():
        pass