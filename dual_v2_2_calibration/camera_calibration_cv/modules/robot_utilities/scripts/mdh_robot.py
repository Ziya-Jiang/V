import numpy
from matplotlib import pyplot as plt

from .robot import Robot

class MDHRobot(Robot):
    """Modified Denavit–Hartenberg defined robot"""
    def __init__(self, joint_size, joint_type, a, alpha, d, theta):
        self.JOINT_SIZE = joint_size
        self.A = a
        self.ALPHA = alpha
        self.D = d
        self.THETA = theta
        if len(joint_type) == 0:
            self.JOINT_TYPE = numpy.zeros((joint_size))
        else:
            self.JOINT_TYPE = numpy.array(joint_type)
        return

    def A1(self, theta, d):
        ans = numpy.array([[+numpy.cos(theta), -numpy.sin(theta), 0, 0],
                           [+numpy.sin(theta), +numpy.cos(theta), 0, 0],
                           [                0,                 0, 1, d],
                           [                0,                 0, 0, 1]])
        return ans

    def A2(self, alpha, a):
        ans = numpy.array([[1,                 0,                 0, a],
                           [0, +numpy.cos(alpha), -numpy.sin(alpha), 0],
                           [0, +numpy.sin(alpha), +numpy.cos(alpha), 0],
                           [0,                 0,                 0, 1]])
        return ans

    def MDH(self, a, alpha, d, theta):
        """ modified DH method (Creig's book) of a single joint
        #
        # i-1         i
        #  +----------+  Oi
        #             |         i+1
        #             +----------+  Qi+1
        """
        return numpy.dot(self.A2(alpha, a), self.A1(theta, d))

    def FK(self, theta):
        """
        Forward kinematics
        Args:
            theta: all joint angles in rad.
        Returns:
            T = end-effector transformation matrix
        """
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(0, self.JOINT_SIZE):
            if self.JOINT_TYPE[k] == 0:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], self.THETA[k]+theta[k]))
            else:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k]+theta[k], self.THETA[k]))
        return T
    
    def MDK(self, theta):
        Te = self.FK(theta)
        T = numpy.zeros((4, 4))
        J = numpy.zeros((6, self.JOINT_SIZE))
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(0, self.JOINT_SIZE):
            if self.JOINT_TYPE[k] == 0:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], self.THETA[k]+theta[k]))
                J[0:3, k] = numpy.cross(T[0:3, 2], Te[0:3, 3] - T[0:3, 3])
                J[3:6, k] = T[0:3, 2]
            else:
                T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k]+theta[k], self.THETA[k]))
                J[0:3, k] = T[0:3, 2]
        return J

    def IK(self, Rt, Pt, q, iterate_times = 20):
        """
        Inverse kinematics
        Args:
            Rt: [3x3] end-effector rotation target matrix
            Pt: [3x1] end-effector position target vector
            q: current joint angles in rad.
            iterate_times: max. iterate times
        Returns:
            q = all joint angles in rad.
        """
        q_ans = q.copy()
        Tc = self.FK(q)
        TempRt = numpy.append(Rt,[[0,0,0]],axis = 0)
        TempPt = numpy.array([numpy.append(Pt,1)])
        Tt = numpy.concatenate((TempRt, numpy.transpose(TempPt)), axis = 1)
        errorP = Pt - Tc[0:3, 3]
        errorR = Rt - Tc[0:3,0:3]
        count = 0
        # print("ERROR {0}, {1}".format(numpy.linalg.norm(errorR), numpy.linalg.norm(errorP)))
        #limit errorPt more strictly
        while (numpy.linalg.norm(errorR) > 1e-3 or numpy.linalg.norm(errorP) > 1e-4) and count < iterate_times:
            J = self.MDK(q)
            if abs(numpy.linalg.norm(J)) < 1e-5:
                print('singularity')
                return q_ans
            # dv = error
            dv = errorP
            dw = 0.5*(numpy.cross(Tc[0:3, 0], Rt[0:3, 0]) + numpy.cross(Tc[0:3, 1], Rt[0:3, 1]) + numpy.cross(Tc[0:3, 2], Rt[0:3, 2]))
            dx = numpy.array([dv[0], dv[1], dv[2], dw[0], dw[1], dw[2]])*0.5
            dq = numpy.dot(numpy.linalg.pinv(J), dx)
            q = q + dq.flatten()
            Tc = self.FK(q)
            errorP = Pt - Tc[0:3, 3]
            errorR = Rt - Tc[0:3, 0:3]
            count = count + 1
        # print('iterates ', count, 'times')
        if numpy.linalg.norm(errorR) > 1e-3 or numpy.linalg.norm(errorP) > 1e-4:
            # print('iterates more than ' + str(iterate_times) + ' times')
            return q_ans

        return q

    def draw(self, q, fig, show_frame=False):
        """
        Draw robot
        Args:
            q: all joint angles in rad.
            fig: figure to draw on
            show_frame: show axes for each joint
        Returns:
            ax = figure axes
        """
        scale = 0.01
        ax = fig.add_subplot(projection='3d')
        ax.bar3d(-5*scale, -5*scale, -3*scale, 10*scale, 10*scale, 3*scale, color='gray')
        T = numpy.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        P = T[0:3, 3]
        R = T[0:3, 0:3]
        ax.quiver(-0.3, 0.3, 0, R[0, 0]*scale*5, R[1, 0]*scale*5, R[2, 0]*scale*5, length=0.1, normalize=True, color='r')
        ax.quiver(-0.3, 0.3, 0, R[0, 1]*scale*5, R[1, 1]*scale*5, R[2, 1]*scale*5, length=0.1, normalize=True, color='g')
        ax.quiver(-0.3, 0.3, 0, R[0, 2]*scale*5, R[1, 2]*scale*5, R[2, 2]*scale*5, length=0.1, normalize=True, color='b')
        ax.text(-0.3 + R[0, 0]*scale*15, 0.3 + R[1, 0]*scale*15, R[2, 0]*scale*15, 'x')
        ax.text(-0.3 + R[0, 1]*scale*15, 0.3 + R[1, 1]*scale*15, R[2, 1]*scale*15, 'y')
        ax.text(-0.3 + R[0, 2]*scale*10, 0.3 + R[1, 2]*scale*10, R[2, 2]*scale*10, 'z')
        
        for k in range(0, self.JOINT_SIZE):
            T = numpy.dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], q[k]))
            Q = P
            P = T[0:3, 3]
            R = T[0:3, 0:3]
            # draw line
            ax.plot([Q[0], P[0]], [Q[1], P[1]], [Q[2], P[2]], linewidth=5, color='orange')

            # draw cylinder
            n = 20
            u = numpy.linspace(0, 2*numpy.pi, n)
            x = numpy.array([numpy.cos(u)*scale*2, numpy.cos(u)*scale*2])
            y = numpy.array([numpy.sin(u)*scale*2, numpy.sin(u)*scale*2])
            z = numpy.array([[-scale*4]*n, [+scale*4]*n])

            for j in range(0, n):
                xx0 = R[0, 0]*x[0][j] + R[0, 1]*y[0][j] + R[0, 2]*z[0][j] + P[0]
                yy0 = R[1, 0]*x[0][j] + R[1, 1]*y[0][j] + R[1, 2]*z[0][j] + P[1]
                zz0 = R[2, 0]*x[0][j] + R[2, 1]*y[0][j] + R[2, 2]*z[0][j] + P[2]

                xx1 = R[0, 0]*x[1][j] + R[0, 1]*y[1][j] + R[0, 2]*z[1][j] + P[0]
                yy1 = R[1, 0]*x[1][j] + R[1, 1]*y[1][j] + R[1, 2]*z[1][j] + P[1]
                zz1 = R[2, 0]*x[1][j] + R[2, 1]*y[1][j] + R[2, 2]*z[1][j] + P[2]

                x[0][j] = xx0
                y[0][j] = yy0
                z[0][j] = zz0

                x[1][j] = xx1
                y[1][j] = yy1
                z[1][j] = zz1
            ax.plot_surface(x, y, z, color='lightblue')

            # draw coordinate
            if show_frame == True or k == self.JOINT_SIZE-1:
                ax.quiver(P[0], P[1], P[2], R[0, 0]*scale*5, R[1, 0]*scale*5, R[2, 0]*scale*5, length=0.1, normalize=True, color='r')
                ax.quiver(P[0], P[1], P[2], R[0, 1]*scale*5, R[1, 1]*scale*5, R[2, 1]*scale*5, length=0.1, normalize=True, color='g')
                ax.quiver(P[0], P[1], P[2], R[0, 2]*scale*5, R[1, 2]*scale*5, R[2, 2]*scale*5, length=0.1, normalize=True, color='b')

        #ax.set_xlabel('X (m)')
        ax.set_xlim(-0.5, 0.5)
        #ax.set_ylabel('Y (m)')
        ax.set_ylim(-0.5, 0.5)
        #ax.set_zlabel('Z (m)')
        ax.set_zlim(-0.3, 0.7)
        #ax.view_init(azim=135)
        plt.axis('off')
        return ax

    def modify_dh(self, alpha, a, theta, d):
        for k in range(len(alpha)):
            self.ALPHA[k] += alpha[k]
            self.A[k] += a[k]
            self.THETA[k] += theta[k]
            self.D[k] += d[k]
        
    def add_to_dh(self, alpha, a, theta, d):
        self.modify_dh(alpha, a, theta, d)
    
    def sub_from_dh(self, alpha, a, theta, d):
        params = -1.0*numpy.array([alpha, a, theta, d])
        self.modify_dh(*params)

    def mul_Ti(self, T, i, theta):
        """
        Multiple T by ith transformation matrix
        Args:
            T: Transformation matrix from T0 to Ti-1
            i: index
            theta: ith joint angle in rad.
        Returns:
            T = Transformation matrix from T0 to Ti
        """
        if (self.JOINT_TYPE[i] == 0):
            T = numpy.dot(T, super().MDH(self.A[i], self.ALPHA[i], self.D[i], theta + self.THETA[i]))
        else:
            T = numpy.dot(T, super().MDH(self.A[i], self.ALPHA[i], self.D[i] + theta, self.THETA[i]))
        return T
