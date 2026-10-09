
#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <cmath>
#include <iostream>
#include "robot_struct.h"
#include "robotics_math.h"
#include "Matrix.hpp"
#include "SquareMatrix.hpp"
#include "Vector.hpp"
// #include "Quaternion.h"



namespace kinematics
{

    int forwardKinematics(const int &robot_index, const double q_in[AXISNUM], double cartesian_position[6]);
    int forwardKinematics1(const int &robot_index, const double q_in[AXISNUM], double cartesian_position[12]);

    int inverseKinematicsSingularityRobust(const int &robot_index, const double destination_cartesian[6],
                                           const double tolerance, const int max_iterative_num, const double last_q_in[AXISNUM], double q_out[AXISNUM]);

    void calculateErrorJacobian(const int &robot_index, const double *q, const double *desEular, matrix::Matrix<double, 6, 1> &dxe, matrix::Matrix<double, 6, AXISNUM> &jacobian_matrix);

    void worldToBase(const int &robot_index, double base_cartesian[6], const double world_cartesian[6]);

    void baseToWorld(const int &robot_index, double world_cartesian[6],const double base_cartesian[6]);
    void baseToWorld1(const int &robot_index, double world_cartesian[12],const matrix::Matrix<double, 4, 4>& base_cartesian);
    void addTool(double EEF_world_cartesian[6], const double link7_world_cartesian[6]);
    void removeTool(double link7_world_cartesian[6], const double EEF_world_cartesian[6]);
    void show(double cartesian_position[6]);

    void showjoint(double joint[AXISNUM]);

    void show(matrix::Matrix<double, 4, 4> T);

    void computeRegression(const int &robot_index,double q_in[AXISNUM],matrix::Matrix<double, 7, 28>& Y_G);
}
#endif