#ifndef CUARM_STATE_H
#define CUARM_STATE_H

#include <inttypes.h>
#include <string>
#include <vector>
#include "3rd_party/magic_enum.hpp"
#include "time_utils.h"

// #ifdef __cplusplus
// extern "C" 
// {
// #endif
enum class MotionControl{kJoint, kNull, kTask, kMotor, kTaskLine, kControlAlgorithm, kJointJog, kTaskJog, kTaskLineJog, kPlayback};
enum OrientControl{kEnd, kBase};
enum InterpolationMethod{kLinear, kCos, kCubic, kQuintic, kNone, kQuinticPath};
enum class InterpolationMotionPhase{kInitialized, kAcceleration, kConstantVelocity, kDeceleration, kFinished, kInterrupted};
enum class ControlType{kInvalid=-1, kPosition, kVelocity, kTorque};
enum class CameraMode{kCapturing, kStreaming};
enum class ControlAlgorithm{
    kNone=0,
    kGravity,
    kGravityWithBoundary,
    kTaskAdmittance,
    kJointAdmittance,
    kTaskImpedance,
    kJointImpedance,
    kNullAdmittance,
};
enum class PanelRemoteState{kStop, kStart};
enum class PanelTargetMode{kSinglePoint, kWaypoint, kRawSinglePoint};
enum ComponentType{kArm, kGripper};
enum ConnectionState{kWaiting, kRemote, kShutDown};
enum class LineMoveStrategy {
    kRejectEntirely = 0, // Abort the move immediately; do not start moving.
    kStopAtBoundary = 1, // Move and stop smoothly at the last reachable point before the failure.
    kDeviateAndBypassing = 2, // Deviate from the line (e.g., switch to joint space) to bypass the unreachable zone.
    kSegmentedExecution = 3 // Execute valid parts, skip the bad segment, and resume line tracking later.
};
enum class PlaybackState{kStop, kStart, kReset};

//operational state
enum class SystemState{
    kStartup    = 0,
    kIdle       = 1, // Completely stationary; safe to accept new paths
    kMoving     = 2, // Actively running an interpolator, velocity command, or jog stream
    kSettling   = 3, // After planning finished / Changing Control Mode / Stopping
    kError      = 4, // Safeguard stop triggered; hardware limits breached or E-stop
    kRecovery   = 5, // Resetting safety loops and clearing faults
    kShutdown   = 6, // Disabling amplifiers and powering down safely
};

enum class PlanResult {
    kSuccess                    = 0,  // IK passed, time allocation valid, safe to execute
    kPoseNotReachable           = 1,  // Target 6D pose is physically outside the workspace
    kLinearPathFailed           = 2,  // Reachable target, but continuous linear path is blocked (Singularity / Joint Limit)
};

namespace DiagnosticFlags {    
    constexpr uint32_t kNone                         = 0; // No warning
    
    // --- Target Profile Saturation (Pre-Interpolation) ---
    constexpr uint32_t kTargetPosSaturation          = 1 << 0; // User target clamped by soft position limits
    constexpr uint32_t kTargetVelSaturation          = 1 << 1; // User target clamped by soft velocity limits
    constexpr uint32_t kTargetTorSaturation          = 1 << 2; // User target clamped by soft torque limits
    
    // --- Real-Time Command Saturation (Post-PID Loop) ---
    constexpr uint32_t kActuatorPosSaturation        = 1 << 3; // Actuator command clamped by soft position limits
    constexpr uint32_t kActuatorVelSaturation        = 1 << 4; // Actuator command clamped by soft velocity limits
    constexpr uint32_t kActuatorTorSaturation        = 1 << 5; // Actuator command clamped by soft torque limits
    constexpr uint32_t kActuatorPosJumpSaturation    = 1 << 6; // Actuator command clamped by position jump limit
    constexpr uint32_t kActuatorVelJumpSaturation    = 1 << 7; // Actuator command clamped by velocity jump limit
    constexpr uint32_t kActuatorTorJumpSaturation    = 1 << 8; // Actuator command clamped by torque jump limit
    
    // --- Soft Workspace Boundary Safety Interceptions ---
    constexpr uint32_t kBoundaryVelClamp             = 1 << 9;  // Velocity zeroed in limit direction due to position boundary reached
    constexpr uint32_t kBoundaryJointImpedance       = 1 << 10; // Joint impedance applied due to position boundary reached
    
    // --- Algorithmic & Kinematic Planner Modifications ---
    constexpr uint32_t kPlanTimelineExtended         = 1 << 11; // Trajectory segment duration (dt) stretched for velocity limits
    constexpr uint32_t kPlanVelLimitInvalid          = 1 << 12; // Specified max velocity profile is smaller than the minimum allowed velocity
    constexpr uint32_t kPlanDeltaTooLarge            = 1 << 13; // Distance between points is too large for a dynamic timeline
    constexpr uint32_t kPlanVelocitySnap             = 1 << 14; // Large velocity shift over zero distance
    constexpr uint32_t kPlanPointSkipped             = 1 << 15; // Duplicated points skipped

    // --- Fault Flags ---
    constexpr uint32_t kFaultPosHardLimitReached     = 1 << 16;
    constexpr uint32_t kFaultVelHardLimitReached     = 1 << 17;
    constexpr uint32_t kFaultTorHardLimitReached     = 1 << 18;
    constexpr uint32_t kFaultPosTrackingFailed       = 1 << 19;
    constexpr uint32_t kFaultVelTrackingFailed       = 1 << 20;
    constexpr uint32_t kFaultTorTrackingFailed       = 1 << 21;
    
    constexpr uint32_t kFaultArmNotFound             = 1 << 22;
    constexpr uint32_t kFaultGripperNotFound         = 1 << 23;
    constexpr uint32_t kFaultHardwareInitFailed      = 1 << 24;
    constexpr uint32_t kFaultCommandTimeout          = 1 << 25;
    constexpr uint32_t kFaultInvalidCommand          = 1 << 26;
    constexpr uint32_t kFaultHardwareCommunication   = 1 << 27;
    constexpr uint32_t kFaultControlDeadlineMissed   = 1 << 28;
    constexpr uint32_t kFaultSafeHoldFailed          = 1 << 29;

    // Warning-only recovery state. Bit 30 is intentionally excluded from
    // Upper Software's fault mask even though it is above the fault range.
    constexpr uint32_t kWarningStartupPositionMismatch = 1u << 30;

    constexpr uint32_t kFaultUnknown                 = 1 << 31;
};

/*
 * Micro for data transfermation
 */
#define MAX_ARM_SIZE                  2
#define MAX_MOTOR_SIZE                7
#define MAX_JOINT_SIZE                14
#define MAX_ORIENTATION_SIZE          8
#define MAX_POSITION_SIZE             6
#define MAX_MESSAGE_SIZE              80
#define MAX_ROBOTNAME_SIZE            30
#define MAX_CAMERA_SIZE               3
#define MAX_IMAGE_SIZE                921600 //480x640x3
#define MAX_WAYPOINTS                 20   
#define MAX_SHPERES_SIZE              65536
#define MAX_COMPONENT_SIZE            3 //kArm, kGripper

template<typename T>
std::string enumToString(T x){
    static_assert(std::is_enum_v<T>, "enumToString: T must be an enum type");
    auto name = magic_enum::enum_name(x);
    return name.empty() ? "Unknown" : std::string(name.substr(1));
}

template<typename T>
T stringToEnum(const std::string& s){
    static_assert(std::is_enum_v<T>, "stringToEnum: T must be an enum type");
    std::string s1 = "k" + s;
    return magic_enum::enum_cast<T>(s1).value_or(static_cast<T>(0));
}

template<typename T>
std::vector<std::string> getAllEnumNames(){
    static_assert(std::is_enum_v<T>, "getAllEnumNames: T must be an enum type");

    constexpr auto names_view = magic_enum::enum_names<T>();
    std::vector<std::string> names;
    names.reserve(names_view.size());
    for (const auto& sv : names_view) {
        names.emplace_back(std::string(sv.substr(1)));
    }
    return names;
}

struct PlannerState{
    long        send_timestamp;
    long        received_panel_command_timestamp;
    bool        setting_update_finished;
    uint32_t    received_sequence_id;

    uint8_t ArmSize;
    uint8_t JointSize[MAX_ARM_SIZE];
    bool    enabled_joint[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float JointPos[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float JointVel[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float JointTor[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float MotorPos[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float MotorVel[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float MotorCur[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float MotorTor[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float JointPose[MAX_ARM_SIZE][MAX_JOINT_SIZE][7];
    float EEPose[MAX_ARM_SIZE][7];
    float ToolPose[MAX_ARM_SIZE][7];

    float interpolated_target[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Gripper related
    uint8_t GripperSize;
    uint8_t GripperJointSize[MAX_ARM_SIZE];
    float   GripperJointPos[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Config setting
    float tool_offset[MAX_ARM_SIZE][3];
    float arm_soft_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_soft_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_soft_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_hard_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_hard_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_hard_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_follow_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float arm_follow_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float arm_follow_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float arm_jump_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float arm_jump_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float arm_jump_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float gripper_soft_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float gripper_hard_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];

    //Status
    SystemState     system_state;
    PlanResult      plan_result;
    uint32_t        system_diagnostic_flags; 
    uint32_t        arm_joint_diagnostic_flags[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    ////////////for debug & internal use/////////////////////
    float FilteredMotorTor[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float FilteredMotorTorKalman[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float FilteredMotorTorEuro[MAX_ARM_SIZE][MAX_MOTOR_SIZE];

    float target_before_interpolation[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_acceleration[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_gravity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float gravity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_impedance[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float impedance[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float body_forces[6];

    float sat_motor_cmd_[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //for gravity comp
    float filtered_joint_vel[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float friction[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float gravity_kd_effect[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //s curve
    float s_curve_pos[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float s_curve_vel[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //cost 
    long setup_cost;
    long Null_cost;
    long FK_cost;
    long IK_cost;
    long ID_cost;
    long simulation_addition_cost;

    char ErrorTitle[MAX_MESSAGE_SIZE];
    char ErrorMsg[MAX_MESSAGE_SIZE];
};

struct RobotState{
    long    SendTimeStamp;
    long    RecieveTimeStamp;

    uint8_t ArmSize;
    uint8_t JointSize[MAX_ARM_SIZE];
    float MotorPos[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float MotorVel[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float MotorCur[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
    float MotorTor[MAX_ARM_SIZE][MAX_MOTOR_SIZE];

    uint8_t GripperSize;
    uint8_t GripperJointSize[MAX_ARM_SIZE];
    float   GripperJointPos[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    bool enable_joint_torque_sensor;
    float JointTorqueSensor[MAX_ARM_SIZE][MAX_MOTOR_SIZE];
};

struct PanelCommand{
    long    send_timestamp;
    uint32_t sequence_id;
    ConnectionState connection_state;
    
    //Settings
    bool need_setting_update;
    bool simulation;
    MotionControl motion_type;
    OrientControl task_orien_type;
    bool reset_control_mem;
    ControlType target_type;
    ControlType actuator_mode;
    InterpolationMethod interpolation_type;
    float interpolation_speed_ratio;
    float InterpolationAccTime;
    float InterpolationConstVelTime;
    float NoneInterpolationSaturationRatio;
    bool reset_interpolation;
    ControlAlgorithm control_algorithm;

    //Recording & Playback
    bool enable_recording;
    PlaybackState playback_cmd;

    //Arm related
    PanelTargetMode arm_target_mode;
    uint8_t ArmSize;
    uint8_t JointSize[MAX_ARM_SIZE];
    char    RobotName[MAX_ARM_SIZE][MAX_ROBOTNAME_SIZE];
    bool    enable_joint[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Arm single point mode
    float MotorCmd[MAX_ARM_SIZE][MAX_JOINT_SIZE];   //in Radian for pos/vel
    float JointCmd[MAX_ARM_SIZE][MAX_JOINT_SIZE];   //in Radian for pos/vel
    float TaskCmd[MAX_ARM_SIZE][6];                 //in Radian for pos

    //Arm waypoint mode(only for PanelRemoteCommand now)
    uint8_t ArmWaypointSize;
    float ArmWaypointDt;
    float ArmWaypointCmd[MAX_WAYPOINTS][MAX_ARM_SIZE][MAX_JOINT_SIZE]; //in Radian for pos/vel
    float ArmWaypointInterpolationTime[MAX_WAYPOINTS]; //in Radian for pos/vel
    float ArmWaypointInterpolationSpeedRatio[MAX_WAYPOINTS]; //in Radian for pos/vel

    //Gripper related
    uint8_t GripperSize;
    uint8_t GripperJointSize[MAX_ARM_SIZE];
    float   GripperJointCmd[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Config setting
    float tool_offset[MAX_ARM_SIZE][3];
    float arm_soft_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_soft_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float gripper_soft_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    
    ////////////for debug & internal use/////////////////////
    float MotorCmdDeg[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float JointCmdDeg[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float TaskCmdDeg[MAX_ARM_SIZE][6];
    float ArmWaypointCmdDeg[MAX_WAYPOINTS][MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float arm_soft_limit_position_deg[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float arm_soft_limit_velocity_deg[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];
    float gripper_soft_limit_position_deg[MAX_ARM_SIZE][MAX_JOINT_SIZE][2];

    //Time cost
    long LastReceiveTime;
    long LastSendTime;
    long ReceiveUDPCost;
    long ReceiveTimerTimeoutCall;
    long SendUDPCost;
    long TimerTimeoutCall;
    long RunningCost;
};

struct PlannerCommand{
    ConnectionState connection_state;
    ControlType command_mode;
    bool enable_set = false;
    uint8_t ArmSize;
    uint8_t JointSize[MAX_ARM_SIZE];
    float Target[MAX_ARM_SIZE][MAX_JOINT_SIZE];
};

struct PanelState{
    PanelRemoteState state;

    //Arm related
    uint8_t ArmSize;
    uint8_t JointSize[MAX_ARM_SIZE];
    float JointPos[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float JointVel[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float JointTor[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float EEPose[MAX_ARM_SIZE][7];
    float ToolPose[MAX_ARM_SIZE][7];

    float interpolated_target[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Gripper related
    uint8_t GripperSize;
    uint8_t GripperJointSize[MAX_ARM_SIZE];
    float   GripperJointPos[MAX_ARM_SIZE][MAX_JOINT_SIZE];
};

struct RemotePanelCommand{
    float interpolation_acceleration_time = 5.0f;
    float no_interpolation_max_velocity_ratio = 0.1f;
    ControlType target_type;
    ControlType actuator_mode;

    //Arm related
    PanelTargetMode arm_target_mode;
    InterpolationMethod interpolation_type;
    uint8_t ArmSize;
    uint8_t ArmJointSize[MAX_ARM_SIZE];
    
    //Arm single point mode
    float ArmJointCmd[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Arm waypoint mode
    uint8_t ArmWaypointSize;
    float ArmWaypointDt;
    float ArmWaypointCmd[MAX_WAYPOINTS][MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Gripper related
    uint8_t GripperSize;
    uint8_t GripperJointSize[MAX_ARM_SIZE];
    float   GripperJointCmd[MAX_ARM_SIZE][MAX_JOINT_SIZE];
};

struct RtConfigState{
    char server_udp_ip[15];
    char client_udp_ip[15];
    uint16_t server_udp_port;
    uint16_t client_udp_port;
    uint8_t arm_size;
    uint8_t arm_joint_size[MAX_ARM_SIZE];
    
    float joint_soft_limit_position[2][MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_soft_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_soft_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    float joint_hard_limit_position[2][MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_hard_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_hard_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    float joint_follow_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_follow_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_follow_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    float joint_jump_limit_position[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_jump_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Gripper
    uint8_t gripper_size;
    uint8_t gripper_joint_size[MAX_ARM_SIZE];
    float gripper_joint_soft_limit_position[2][MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float gripper_joint_hard_limit_position[2][MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float tool_offset[MAX_ARM_SIZE][3];
};

struct PanelConfigCommand{
    bool read_only;
    uint16_t remote_udp_port;
    uint8_t arm_size;
    uint8_t arm_joint_size[MAX_ARM_SIZE];

    float joint_soft_limit_position[2][MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_soft_limit_velocity[MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float joint_soft_limit_torque[MAX_ARM_SIZE][MAX_JOINT_SIZE];

    //Gripper
    uint8_t gripper_size;
    uint8_t gripper_joint_size[MAX_ARM_SIZE];
    float gripper_soft_limit_position[2][MAX_ARM_SIZE][MAX_JOINT_SIZE];
    float tool_offset[MAX_ARM_SIZE][3];
};

void print_panel_command(PanelCommand* panel);
void print_planner_state(PlannerState* state);
void print_planner_command(PlannerCommand* command);
void print_rt_config_state(RtConfigState* state);
void print_diagnostic_flags(uint32_t flags);
///////////////////////////////////////////Curobo//////////////////////////////////////////////////////
struct CuroboCommand{
    //Arm related
    uint8_t JointSize;
    float JointPosCmd[MAX_JOINT_SIZE];

    //Gripper related
    uint8_t GripperJointSize;
    float   GripperJointCmd[MAX_JOINT_SIZE];

    uint8_t CmdPlanSize;

    uint8_t WaypointsSize;
    float WaypointsDT;
    float Waypoints[MAX_WAYPOINTS][MAX_JOINT_SIZE];
};

// #ifdef __cplusplus
// }
// #endif
#endif