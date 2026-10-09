#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <map>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <functional>
#include "motor.h"
#include "torque_sensor.h"
#include "polynomial_trajectory.h"
#include "pos_controller.h"
#include "kinematics.h"

class RobotArms {
public:
    static const int num_joints_ = 7;
    using StopRequested = std::function<bool()>;

    RobotArms(int arm_index=0, const Robot& robot= Robot(),
              StopRequested stop_requested = {});
    ~RobotArms();
    int initialize();
    int shut_down();
    int enable_motor(int index, bool enable);
    int recover_startup_fault(int index);
    int fix_motor_initial_position();
    int set_control_mode(CONTROL_MODE mode);
    int set_control_mode(int index, CONTROL_MODE mode);
    int set_mdh_position(int index, double mdh_pos);
    int set_velocity(int index, double vel_rad_s);
    int set_current(int index, double cur_amp);
    int read_status(int index);
    double get_mdh_position(int index);
    double get_velocity(int index);
    double get_current(int index);
    int move_joint(const double* mdh_degrees, double dt, double T);
    int read_torque_sensor(int index);
    double get_torque_sensor_data(int index);
    int poll_responses(int timeout_ms = 10, int expected_motor_index = -1,
                       bool expect_write_ack = false);
    int poll_torque_sensor_response(int timeout_ms = 10);
    double mdh_to_joints(int index, const double mdh_degrees);
    double joints_to_mdh(int index, const double joints_degrees);
    int motor_enable_list_[num_joints_];
    int sensor_enable_list_[num_joints_];
    int move_line_online(const double* target_cart, double dt, double T);
    int get_tcp_data(double tcp[6]);
    int move_line_offline(const double* target_cart, double dt, double T);
    void update_encoder_zero_offset(int index, float offset);
    bool is_brake_opened(int index);
    bool stopRequested() const;
private:
    int _plan_line_offline(const double* target_cart, double dt, double T,
                                   std::vector<std::vector<double>>& trajectory);
    int _execute_offline_trajectory(const std::vector<std::vector<double>>& trajectory,
                                           double dt, FILE* fwrite_data);
    

    const int arm_index_;
    const int can_channel_;
    const int sensor_channel_;
    WhjMotor arm_motors_[num_joints_];
    TorqueSensor arm_torque_sensor_[num_joints_];
    std::map<uint32_t, WhjMotor*> motor_map_;
    std::map<uint32_t, TorqueSensor*> sensor_map_;
    const Robot& robot_config_;
    StopRequested stop_requested_;
};

#endif