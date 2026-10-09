#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include "robot_platform_utils/cpp/include/cuarm_state.h"

typedef struct save_data {
    int motion_type;

    long   communication_cost;
    long   hardware_communication_cost;
    long   running_cost;
    long   robotics_total_cost; 
    long   robotics_setup_cost;
    long   robotics_Null_cost;
    long   robotics_FK_cost;
    long   robotics_IK_cost;
    long   robotics_ID_cost;
    long   robotics_simulation_addition_cost;

    float  gripper_command[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float  gripper_act_set_command[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float  gripper_position[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    int command_mode;
    float joint_gravity[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float gravity[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float joint_impedance[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float impedance[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float sat_motor_cmd[MAX_ARM_SIZE][MAX_MOTOR_SIZE];

    //Gravity 
    float filtered_joint_vel[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float friction[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float gravity_kd_effect[MAX_ARM_SIZE][MAX_MOTOR_SIZE];

    double motor_raw_position[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double motor_target[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double joint_position[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double joint_velocity[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double joint_acceleration[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double joint_torque[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double torque_sensor_raw[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double torque_sensor[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double encoder_sensor_raw[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double motor_velocity[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double motor_torque[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double motor_position[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double filtered_motor_torque[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double filtered_motor_torque_kalman[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double filtered_motor_torque_euro[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double sliced_tar[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    double tool_pose[MAX_ARM_SIZE][7];
    double target_before_interpolation[MAX_ARM_SIZE][MAX_MOTOR_SIZE];

}save_data;

bool queue_insert(const save_data& new_node);
bool queue_poptop(save_data* new_node);
int queue_get_size();

#endif