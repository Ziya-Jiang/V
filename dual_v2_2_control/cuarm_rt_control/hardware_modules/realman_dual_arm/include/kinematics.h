#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "robot_struct.h"
#include "robotics_math.h"
#include "Matrix.hpp"
#include "SquareMatrix.hpp"
#include "Vector.hpp"

namespace kinematics {
    double mdh_to_joints(const Robot& robot, int joint_index, double mdh_degrees);
    double joints_to_mdh(const Robot& robot, int joint_index, double joints_degrees);

    int forwardKinematics(const Robot& robot, const double q_in[NUM_JOINTS_PER_ARM], double cartesian_position[6]);

    int inverseKinematicsSingularityRobust(const Robot& robot, const double destination_cartesian[6],
                                           double tolerance, int max_iterative_num, const double last_q_in[NUM_JOINTS_PER_ARM], double q_out[NUM_JOINTS_PER_ARM]);

    void calculateErrorJacobian(const Robot& robot, const double *q, const double *desEular, matrix::Matrix<double, 6, 1> &dxe, matrix::Matrix<double, 6, NUM_JOINTS_PER_ARM> &jacobian_matrix);

    void worldToBase(const Robot& robot, double base_cartesian[6], const double world_cartesian[6]);

    void baseToWorld(const Robot& robot, double world_cartesian[6], const double base_cartesian[6]);

    void addTool(const Robot& robot, double eef_pos[6], const double link7_pos[6]);

    void removeTool(const Robot& robot, double link7_pos[6], const double eef_pos[6]);

    void show(const double cartesian_position[6]);

    void showjoint(const double joint[NUM_JOINTS_PER_ARM]);

    void show(const matrix::Matrix<double, 4, 4>& T);
}

#endif