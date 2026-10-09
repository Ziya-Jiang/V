#ifndef ROS_ROBOT_STATE_H
#define ROS_ROBOT_STATE_H
#include <QList>
struct RosRobotState
{
    QList<float> motor_pos;
    QList<float> motor_vel;
    QList<float> motor_tor;
    QList<float> joint_pos;
    QList<float> joint_vel;
    QList<float> pose;
};
Q_DECLARE_METATYPE(RosRobotState);

#endif