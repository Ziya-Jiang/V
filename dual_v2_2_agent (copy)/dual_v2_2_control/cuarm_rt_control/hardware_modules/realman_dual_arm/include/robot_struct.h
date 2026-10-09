#pragma once
#ifndef ROBOT_STRUCT_H_
#define ROBOT_STRUCT_H_

#include <array>
#include <cstdint>
#include <cstddef>
constexpr size_t NUM_ARM = 2;
constexpr size_t NUM_JOINTS_PER_ARM = 7;
constexpr size_t NUM_JOINTS_HEAD = 2;
constexpr double PI = 3.141592653589793;
constexpr double DEGREE2RAD = PI / 180.0;
constexpr double RAD2DEGREE = 180.0 / PI;

struct MDH {
    std::array<double, NUM_JOINTS_PER_ARM> a{};
    std::array<double, NUM_JOINTS_PER_ARM> alpha{};
    std::array<double, NUM_JOINTS_PER_ARM> d{};
    std::array<double, NUM_JOINTS_PER_ARM> offset{};
    std::array<double, NUM_JOINTS_PER_ARM> sign{};
    std::array<double, 6> base_to_world{};
    std::array<double, NUM_JOINTS_PER_ARM> m_init_mdhpos{};
    std::array<double, 6> tool{};
};

struct JointLimit {
    std::array<double, NUM_JOINTS_PER_ARM> joint_positive_limit{};
    std::array<double, NUM_JOINTS_PER_ARM> joint_negative_limit{};
};

struct CartesianSpaceLimit {
    std::array<double, 6> cartesian_positive_limit{};
    std::array<double, 6> cartesian_negative_limit{};
};


struct DynamicParms {
    std::array<double, NUM_JOINTS_PER_ARM * 4> dynamic_parameters{};
};

struct MotorParams {
    std::array<uint32_t, NUM_JOINTS_PER_ARM> ids{};
    std::array<int, NUM_JOINTS_PER_ARM> enabled{};
    std::array<double, NUM_JOINTS_PER_ARM> gear_ratios{};
    std::array<double, NUM_JOINTS_PER_ARM> rated_torque{};
    std::array<double, NUM_JOINTS_PER_ARM> max_velocity_rpm{};
    std::array<double, NUM_JOINTS_PER_ARM> max_current_mA{};
    std::array<double, NUM_JOINTS_PER_ARM> max_position_step{};
    int can_channel = 0;
    int pdo_auto_report_ms = 0;
    int use_pdo = 0;
};

struct TorqueSensorParams {
    std::array<unsigned int, NUM_JOINTS_PER_ARM> ids{};
    std::array<int, NUM_JOINTS_PER_ARM> enabled{};
    std::array<int, NUM_JOINTS_PER_ARM> sign{};
    int device_type = 0;  // 0: CANABLE, 1: USBCANII
    int can_channel = 0;
};

struct HandParams {
    int id = 0;
    std::array<int16_t, 6> hand_cmd{};
};

struct ControlParams {
    struct PositionControl {
        double dt = 0.0;
        double T = 0.0;
        bool use_hand = false;
        bool use_torque_sensor = false;
        std::array<double, NUM_JOINTS_PER_ARM> command_degrees{};
        std::array<double, 6> cartesian_command{};
    };

    struct VelocityControl {
        double interpolation_dt = 0.0;
        int interpolation_T = 0;
        std::array<double, NUM_JOINTS_PER_ARM> kp{};
        std::array<double, NUM_JOINTS_PER_ARM> max_velocity{};
        double error_tolerance = 0.0;
        bool use_torque_sensor = false;
        std::array<double, NUM_JOINTS_PER_ARM> command_degrees{};
    };

    std::array<PositionControl, NUM_ARM> position{};
    std::array<VelocityControl, NUM_ARM> velocity{};
    PositionControl head_position{};
    VelocityControl head_velocity{};
};

class Robot {
public:
    Robot() = default;

    MDH mdh;
    JointLimit joint_limit;
    CartesianSpaceLimit cartesian_space_limit;
    DynamicParms dynamic;
    MotorParams motors;  
    TorqueSensorParams torque_sensors;  
    HandParams hands;  
    ControlParams control;  
};

#endif