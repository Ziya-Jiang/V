/********** ********** ********** ********** ********** ********** **********
 *                                 ( 0-0 )                                  *
 *                            (" \/       \/ ")                             *
 ********** ********** ********** ********** ********** ********** **********
 * Copyright (C) 2018 - 2022 CURI & HKCLR                                   *
 * File name   : motor_control.c                                            *
 * Author      : CHEN Wei                                                   *
 * Version     : 1.0.0                                                      *
 * Date        : 2022-01-20                                                 *
 * Description : generial motor control code, can run on different kernel.  *
 * Others      : None                                                       *
 * History     : 2022-01-06 the first version with different kernels.       *
 *             : 2022-01-20 add different communication protocols.          *
 ********** ********** ********** ********** ********** ********** **********
 *                              (            )                              *
 *                               \ __ /\ __ /                               *
 ********** ********** ********** ********** ********** ********** **********/
#include <thread>
// #define SIMULATION
#if KERNEL_XENOMAI
    #include <sys/time.h>
    #include <alchemy/task.h>
    #include <alchemy/timer.h>
    #include <sys/mman.h>
    #include <alchemy/sem.h>
    RT_TASK task_handle_control;
    RT_TASK task_handle_data;
    RT_TASK task_handle_udp;
    RT_SEM mysync;
#elif KERNEL_PREEMPT_RT
    #ifndef _GNU_SOURCE
        #define _GNU_SOURCE
    #endif
    #include <sched.h>  
    #include <pthread.h>  
    #include <limits.h>
    #include <sys/mman.h>	    // Needed for mlockall()
    #include <unistd.h>		    // Needed for sysconf(int name);
    #include <malloc.h>
    #include <sys/time.h>	    // Needed for getrusage
    #include <sys/resource.h>	// Needed for getrusage
    pthread_t task_handle_control;
    pthread_t task_handle_data;
    pthread_t task_handle_udp;
    pthread_attr_t task_attr_control;
    pthread_attr_t task_attr_data;
    pthread_attr_t task_attr_udp;
#else
    // #include <osal.h>
    // OSAL_THREAD_HANDLE task_handle_control;
    // OSAL_THREAD_HANDLE task_handle_data;
    // OSAL_THREAD_HANDLE task_handle_udp;
    #include <thread>
#endif

#include "math.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <array>
#include <csignal>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <algorithm> //std::max
#include <sys/syscall.h>
#include <unordered_map>
#include <memory>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <chrono>

//3rd party
#include <yaml-cpp/yaml.h>

//this repository
#include "src/circular_queue.h"
#include "src/gripper/uart_servo_two_finger_gripper.h"
#ifdef DUAL_V1
    #include "src/arm/dual_v1_arm.h"
    #include "src/gripper/inspire_hand_wrapper.h"
#elif defined(DUAL_V2_1)
    #include "src/arm/dual_v2_1_arm.h"
#elif defined(DUAL_V2_2)
    #include "src/arm/dual_v2_2_arm.h"
    #include "inspire_hand.h"
    #include "hand_can_transport.h"
#elif defined(SPARK2_V1)
    #include "src/arm/spark2_v1.h"
#elif defined(SPARK2_V2)
    #include "hardware_modules/spark2/spark2.h"
#endif
#include "cuarm_robotics/src/robot_planner.h"

//outside this repository
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_message_handler.h"

#ifndef PI
    #define PI                            3.1415926
#endif

int udp_receive_delay_us; // sender need to be slower than receiver
int control_loop_delay_us; // read from config file
int write_data_delay_us; // read from config file
int write_freq = 1;
FILE *fwrite_data = nullptr;
std::string encoder_last_position_path;
save_data new_data;

std::unique_ptr<CuarmUdp<PanelCommand, PlannerState>> udp_ptr;
std::unique_ptr<RobotPlanner> planner_ptr;
RobotState robot_state;
std::unique_ptr<PanelCommand> panel_command_ptr;
std::unique_ptr<PanelCommand> transmit_panel_command_ptr;
std::mutex planel_command_mutex;
PlannerState* planner_state_ptr;
PlannerCommand* planner_command_ptr;

struct RTControlConfig{
    std::vector<std::string> arm_name;
    bool hardware_simulation = true;

    //Safety Check Related
    bool pause_joint_follow_check = false; //Pause for one more circle after re-enabled motor_set
    std::vector<std::vector<float>> last_planner_target; //Prev planner target for joint follow comparsion
    std::vector<int> can_consecutive_exchange_failures;

    //0 = lower limit, 1 = upper limit
    std::array<std::vector<std::vector<float>>, 2> joint_position_limit;
    std::array<std::vector<std::vector<float>>, 2> joint_velocity_limit;
    std::array<std::vector<std::vector<float>>, 2> joint_torque_limit;

    std::vector<std::vector<float>> joint_position_follow_threshold;
    std::vector<std::vector<float>> joint_velocity_follow_threshold;
    std::vector<std::vector<float>> joint_torque_follow_threshold;

    std::vector<std::vector<float>> joint_jump_position_limit;
    std::vector<std::vector<float>> encoder_last_position;
    std::vector<std::vector<bool>> startup_position_mismatch_joint;

    long command_watchdog_timeout_us = 250000;
    bool allow_startup_position_mismatch = false;
};
RTControlConfig rt_config;

std::atomic<bool> shutdown_all_threads = false;
std::atomic<long> last_panel_command_time_us{0};
std::atomic<bool> panel_command_seen{false};
std::atomic<bool> startup_position_mismatch_active{false};
std::mutex encoder_snapshot_mutex;
bool startup_zero_command_seen = false;
int startup_zero_stable_cycles = 0;
bool skip_arm_test_mode = false;
bool left_hand_only_test_mode = false;
enum class MotorAction{kInit, kNormal, kShutDown};
constexpr float kStartupZeroPositionToleranceRad = 0.05f;
constexpr float kStartupZeroVelocityToleranceRadS = 0.05f;
constexpr int kStartupZeroStableDurationUs = 500000;
constexpr int kCanStationaryHoldMaxTransientFailures = 2;
constexpr float kCanStationaryHoldVelocityThresholdRadS = 0.01f;
constexpr float kCanStationaryHoldTargetDeltaRad = 0.0001f;
constexpr float kCanStationaryHoldPositionErrorRad = 0.01f;

//Robot
std::vector<std::unique_ptr<ArmBase>> arm_ptr_vec;

//Gripper
std::vector<std::unique_ptr<AbstractGripper>> gripper_ptr_vec;

void request_shutdown_from_signal(int) {
    shutdown_all_threads.store(true, std::memory_order_relaxed);
}

void mark_system_fault(uint32_t fault, const std::string& message) {
    std::cerr << "[FAULT] " << message << std::endl;
    if (planner_state_ptr) {
        planner_state_ptr->system_diagnostic_flags |= fault;
        planner_state_ptr->system_state = SystemState::kError;
    }
    shutdown_all_threads.store(true, std::memory_order_relaxed);
}

bool disable_all_arms() {
    if (rt_config.hardware_simulation || skip_arm_test_mode) return true;
    bool success = true;
    for (auto& arm : arm_ptr_vec) {
        if (!arm) continue;
        const std::vector<bool> disable(static_cast<std::size_t>(arm->getJointSize()), false);
        if (arm->enableMotors(disable) != RetCode::kSuccess) success = false;
    }
    if (!success && planner_state_ptr) {
        planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultHardwareCommunication;
    }
    return success;
}

bool hold_current_pose_or_disable() {
    if (rt_config.hardware_simulation || skip_arm_test_mode || arm_ptr_vec.empty()) return true;

    bool all_held = true;
    for (std::size_t arm_i = 0; arm_i < arm_ptr_vec.size(); ++arm_i) {
        ArmBase* arm = arm_ptr_vec[arm_i].get();
        if (!arm) continue;

        const int joint_size = arm->getJointSize();
        std::vector<float> hold_position(static_cast<std::size_t>(joint_size), 0.0f);
        std::vector<float> measured_velocity(static_cast<std::size_t>(joint_size), 0.0f);
        std::vector<float> measured_torque(static_cast<std::size_t>(joint_size), 0.0f);
        bool held = arm->getState(&hold_position, &measured_velocity, &measured_torque) == RetCode::kSuccess;
        held &= arm->setMode(ControlType::kPosition) == RetCode::kSuccess;

        for (int attempt = 0; held && attempt < 3; ++attempt) {
            std::vector<float> measured_position = hold_position;
            held &= arm->setCommandAndGetState(
                        hold_position, &measured_position, &measured_velocity, &measured_torque) ==
                    RetCode::kSuccess;
            sleep_period(10000);
        }

        if (!held) {
            all_held = false;
            const std::vector<bool> disable(static_cast<std::size_t>(joint_size), false);
            arm->enableMotors(disable);
            if (planner_state_ptr) {
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultSafeHoldFailed;
                for (int joint_i = 0; joint_i < joint_size; ++joint_i) {
                    planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |=
                        DiagnosticFlags::kFaultSafeHoldFailed;
                }
            }
            std::cerr << "[FAULT] Arm " << arm_i
                      << " failed to hold its current pose and was disabled." << std::endl;
        }
    }
    return all_held;
}


#ifdef DUAL_V2_2
namespace {
constexpr int kVlmPanelHandIndex = 0;
constexpr int kVlmLeftHandIndex = 0;
constexpr int kVlmRightGripperIndex = 1;
constexpr float kVlmLeftHandCloseThreshold = 0.2f;
constexpr float kVlmRightGripperCloseThreshold = 0.5f;
constexpr int16_t kVlmLeftHandOpenVerifyMin = 900;
constexpr int kVlmLeftHandOpenMaxCommands = 4;
constexpr auto kVlmLeftHandOpenVerifyInterval = std::chrono::milliseconds(500);
constexpr auto kVlmLeftHandOpenCommandInterval = std::chrono::seconds(2);
constexpr auto kVlmLeftHandOpenTimeout = std::chrono::seconds(8);
int g_vlm_hand_ids[2] = {2, 1};
std::shared_ptr<IHandCanTransport> g_vlm_hand_transport;
std::unique_ptr<InspireHand> g_vlm_left_hand;
bool g_vlm_left_hand_available = false;
std::vector<float> g_vlm_left_hand_last_panel_command(6, 0.0f);
bool g_vlm_last_hand_close_valid = false;
bool g_vlm_last_hand_close = false;
bool g_vlm_last_direct_angles_valid = false;
int16_t g_vlm_last_direct_angles[6] = {0, 0, 0, 0, 0, 0};
bool g_vlm_left_hand_open_pending = false;
bool g_vlm_left_hand_open_failed = false;
int g_vlm_left_hand_open_attempts = 0;
std::chrono::steady_clock::time_point g_vlm_left_hand_next_open_check;
std::chrono::steady_clock::time_point g_vlm_left_hand_next_open_command;
std::chrono::steady_clock::time_point g_vlm_left_hand_open_deadline;

const int16_t* vlmLeftHandAngles(bool close_hand) {
    static const int16_t open_angles[6] = {1000, 1000, 1000, 1000, 1000,0};
    static const int16_t close_angles[6] = {200, 300, 300, 500, 300, 0};
    return close_hand ? close_angles : open_angles;
}

bool vlmLeftHandUsesDirectAngles(const std::vector<float>& panel_cmd) {
    if (panel_cmd.size() < 6) return false;
    for (size_t i = 0; i < 6; ++i) {
        if (std::abs(panel_cmd[i]) > 1.0f) return true;
    }
    return false;
}

int16_t vlmClampHandAngle(float value) {
    if (value < -32768.0f) return -32768;
    if (value > 32767.0f) return 32767;
    return static_cast<int16_t>(value);
}

bool vlmDirectAnglesUnchanged(const int16_t angles[6]) {
    if (!g_vlm_last_direct_angles_valid) return false;
    for (int i = 0; i < 6; ++i) {
        if (g_vlm_last_direct_angles[i] != angles[i]) return false;
    }
    return true;
}

bool vlmLeftHandShouldClose(const std::vector<float>& panel_cmd) {
    if (panel_cmd.size() <= 1) return false;
    float max_flex = 0.0f;
    for (size_t i = 1; i < panel_cmd.size(); ++i) {
        max_flex = std::max(max_flex, panel_cmd[i]);
    }
    return max_flex > kVlmLeftHandCloseThreshold;
}

bool vlmLeftHandOpenPositionReached() {
    if (!g_vlm_left_hand || g_vlm_left_hand->getAngles(kVlmLeftHandIndex) != 0) {
        std::cerr << "left Inspire Hand OPEN verification failed to read angles."
                  << std::endl;
        return false;
    }

    bool reached = true;
    std::cout << "left Inspire Hand OPEN actual angles=[";
    for (int i = 0; i < 6; ++i) {
        const int16_t actual = g_vlm_left_hand->joint_angle_[kVlmLeftHandIndex][i];
        std::cout << actual << (i == 5 ? "" : ",");
        if (i < 5 && actual < kVlmLeftHandOpenVerifyMin) {
            reached = false;
        }
    }
    std::cout << "] " << (reached ? "verified" : "not fully open") << std::endl;
    return reached;
}

bool vlmLeftHandPrepareForOpen() {
    if (!g_vlm_left_hand) return false;

    const int error_ret = g_vlm_left_hand->getError(kVlmLeftHandIndex);
    const int status_ret = g_vlm_left_hand->getStatus(kVlmLeftHandIndex);
    if (error_ret != 0 || status_ret != 0) {
        std::cerr << "left Inspire Hand failed to read J0 fault/status before OPEN."
                  << std::endl;
        return false;
    }

    const int error =
        static_cast<unsigned short>(
            g_vlm_left_hand->joint_error_[kVlmLeftHandIndex][0]
        ) & 0xff;
    const int status =
        static_cast<unsigned short>(
            g_vlm_left_hand->joint_status_[kVlmLeftHandIndex][0]
        ) & 0xff;
    if (error == 0 && status == 0xff) {
        std::cerr
            << "left Inspire Hand J0 status unavailable (0xff); "
            << "fault register is clear, continuing with angle verification."
            << std::endl;
    } else if (error != 0 || status >= 4) {
        std::cerr << "left Inspire Hand J0 before OPEN: error=0x"
                  << std::hex << error << std::dec
                  << ", status=" << status << std::endl;
    }

    constexpr int kRecoverableErrorMask = 0x01 | 0x04;  // stalled/overcurrent
    constexpr int kNonRecoverableErrorMask =
        0x02 | 0x08 | 0x10;  // over-temperature/motor/communication
    if ((error & kNonRecoverableErrorMask) != 0 || status == 6) {
        std::cerr
            << "left Inspire Hand J0 has a non-recoverable active fault; "
            << "OPEN is blocked to avoid repeated motor stress."
            << std::endl;
        return false;
    }

    if ((error & kRecoverableErrorMask) != 0 || status == 4 || status == 5) {
        std::cerr
            << "left Inspire Hand J0 has stalled/current protection; "
            << "clearing the recoverable fault before OPEN."
            << std::endl;
        if (g_vlm_left_hand->clearError(kVlmLeftHandIndex) != 0) {
            std::cerr << "left Inspire Hand J0 fault clear failed." << std::endl;
            return false;
        }
    }
    return true;
}

RetCode initializeVlmLeftHand() {
    try {
        g_vlm_hand_transport = std::make_shared<CanableHandTransport>(g_vlm_hand_ids);
        auto canable = std::dynamic_pointer_cast<CanableHandTransport>(g_vlm_hand_transport);
        constexpr int kProbeIds[] = {2, 1, 3, 4, 5, 6};
        constexpr int kProbeTimeoutMs = 400;
        int detected_hand_id = -1;
        uint32_t unexpected_id = 0;
        bool saw_unexpected = false;

        for (int candidate : kProbeIds) {
            HandCanFrame request;
            HandCanFrame response;
            // Same extended-ID layout as InspireHand::build_ext_id for a read of ANGLE_ACT (0x060A).
            request.id = (static_cast<uint32_t>(0x060A & 0xFFF) << 14) |
                         (static_cast<uint32_t>(candidate) & 0x3FFF);
            request.is_extended = true;
            request.dlc = 1;
            request.data[0] = 8;

            std::cout << "Probing left Inspire Hand id=" << candidate << std::endl;
            if (!canable ||
                canable->exchange(request, response, kProbeTimeoutMs, false, candidate) != HAND_CAN_SUCCESS) {
                continue;
            }

            const uint32_t reply_hand_id = response.id & 0x3FFF;
            std::cout << "CAN reply id=0x" << std::hex << response.id << std::dec
                      << " hand_id_field=" << reply_hand_id << std::endl;
            if (reply_hand_id == static_cast<uint32_t>(candidate)) {
                detected_hand_id = candidate;
                break;
            }
            saw_unexpected = true;
            unexpected_id = response.id;
        }

        if (detected_hand_id < 0) {
            std::cerr << "Initial left Inspire Hand getAngles failed. Hand disabled. "
                      << "No reply for ids 2,1,3,4,5,6.";
            if (saw_unexpected) {
                std::cerr << " Saw unrelated frame id=0x" << std::hex << unexpected_id << std::dec
                          << " hand_id_field=" << (unexpected_id & 0x3FFF) << ".";
            } else {
                std::cerr << " Bus stayed silent.";
            }
            std::cerr << std::endl;
            g_vlm_left_hand.reset();
            g_vlm_hand_transport.reset();
            g_vlm_left_hand_available = false;
            return RetCode::kFailed;
        }

        if (detected_hand_id != g_vlm_hand_ids[kVlmLeftHandIndex]) {
            std::cout << "Left Inspire Hand responded as id=" << detected_hand_id
                      << " instead of configured id=" << g_vlm_hand_ids[kVlmLeftHandIndex]
                      << std::endl;
        }
        g_vlm_hand_ids[kVlmLeftHandIndex] = detected_hand_id;
        g_vlm_left_hand = std::make_unique<InspireHand>(g_vlm_hand_ids, g_vlm_hand_transport);
        if (g_vlm_left_hand->getAngles(kVlmLeftHandIndex) != 0) {
            std::cerr << "Initial left Inspire Hand getAngles failed. Hand disabled. "
                      << "transport=canable hand_index=" << kVlmLeftHandIndex
                      << " hand_id=" << g_vlm_hand_ids[kVlmLeftHandIndex] << std::endl;
            g_vlm_left_hand.reset();
            g_vlm_hand_transport.reset();
            g_vlm_left_hand_available = false;
            return RetCode::kFailed;
        }

        int16_t speed[6] = {100, 100, 100, 100, 100, 100};
        // Channel 3 previously latched overcurrent while trying to close past
        // its reachable position. Limit that channel's force.
        int16_t force[6] = {500, 1000, 1000, 400, 1000, 1000};
        g_vlm_left_hand->setSpeed(kVlmLeftHandIndex, speed);
        usleep(2000);
        g_vlm_left_hand->setForce(kVlmLeftHandIndex, force);
        usleep(2000);
        g_vlm_left_hand_available = true;
        std::cout << "left Inspire Hand initialized: mode=canable hand_index="
                  << kVlmLeftHandIndex << " hand_id=" << g_vlm_hand_ids[kVlmLeftHandIndex]
                  << std::endl;
        return RetCode::kSuccess;
    } catch (const std::exception& e) {
        std::cerr << "left Inspire Hand initialization failed. Hand disabled: " << e.what() << std::endl;
        g_vlm_left_hand.reset();
        g_vlm_hand_transport.reset();
        g_vlm_left_hand_available = false;
        return RetCode::kFailed;
    }
}

RetCode setVlmLeftHandFromPanelCommand(const std::vector<float>& panel_cmd) {
    g_vlm_left_hand_last_panel_command = panel_cmd;
    int16_t direct_angles[6] = {0, 0, 0, 0, 0, 0};
    bool direct_mode = vlmLeftHandUsesDirectAngles(panel_cmd);
    bool close_hand = false;
    const int16_t* angles = nullptr;
    const auto now = std::chrono::steady_clock::now();

    if (!g_vlm_left_hand_available || !g_vlm_left_hand) {
        std::cout << "left Inspire Hand command skipped: hand unavailable"
                  << std::endl;
        return RetCode::kFailed;
    }

    if (direct_mode) {
        g_vlm_left_hand_open_pending = false;
        g_vlm_left_hand_open_failed = false;
        g_vlm_left_hand_open_attempts = 0;
        for (int i = 0; i < 6; ++i) direct_angles[i] = vlmClampHandAngle(panel_cmd[i]);
        if (vlmDirectAnglesUnchanged(direct_angles)) {
            return RetCode::kSuccess;
        }
        angles = direct_angles;
    } else {
        close_hand = vlmLeftHandShouldClose(panel_cmd);
        angles = vlmLeftHandAngles(close_hand);
        if (close_hand) {
            g_vlm_left_hand_open_pending = false;
            g_vlm_left_hand_open_failed = false;
            g_vlm_left_hand_open_attempts = 0;
            if (g_vlm_last_hand_close_valid && g_vlm_last_hand_close) {
                return RetCode::kSuccess;
            }
        } else {
            if (
                g_vlm_last_hand_close_valid
                && !g_vlm_last_hand_close
                && !g_vlm_left_hand_open_pending
                && !g_vlm_left_hand_open_failed
            ) {
                return RetCode::kSuccess;
            }
            if (g_vlm_left_hand_open_failed) {
                return RetCode::kFailed;
            }
            if (!g_vlm_left_hand_open_pending) {
                g_vlm_left_hand_open_pending = true;
                g_vlm_left_hand_open_attempts = 0;
                g_vlm_left_hand_next_open_check = now;
                g_vlm_left_hand_next_open_command = now;
                g_vlm_left_hand_open_deadline =
                    now + kVlmLeftHandOpenTimeout;
                g_vlm_last_hand_close_valid = false;
            }
            if (now < g_vlm_left_hand_next_open_check) {
                return RetCode::kSuccess;
            }

            if (
                g_vlm_left_hand_open_attempts > 0
                && vlmLeftHandOpenPositionReached()
            ) {
                g_vlm_left_hand_open_pending = false;
                g_vlm_left_hand_open_attempts = 0;
                g_vlm_last_hand_close_valid = true;
                g_vlm_last_hand_close = false;
                g_vlm_last_direct_angles_valid = false;
                return RetCode::kSuccess;
            }
            if (now >= g_vlm_left_hand_open_deadline) {
                g_vlm_left_hand_open_pending = false;
                g_vlm_left_hand_open_failed = true;
                std::cerr
                    << "left Inspire Hand OPEN timed out after "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                           kVlmLeftHandOpenTimeout
                       ).count()
                    << " ms (commands_sent="
                    << g_vlm_left_hand_open_attempts
                    << "); stop retrying until a new CLOSE command "
                    << "to avoid motor stress."
                    << std::endl;
                return RetCode::kFailed;
            }
            if (
                g_vlm_left_hand_open_attempts
                    >= kVlmLeftHandOpenMaxCommands
                || now < g_vlm_left_hand_next_open_command
            ) {
                g_vlm_left_hand_next_open_check =
                    now + kVlmLeftHandOpenVerifyInterval;
                return RetCode::kSuccess;
            }
            if (!vlmLeftHandPrepareForOpen()) {
                g_vlm_left_hand_open_pending = false;
                g_vlm_left_hand_open_failed = true;
                return RetCode::kFailed;
            }
        }
    }

    std::cout << "left Inspire Hand <- " << (direct_mode ? "ANGLES" : (close_hand ? "CLOSE" : "OPEN")) << " angles=[";
    for (int i = 0; i < 6; ++i) {
        std::cout << angles[i] << (i == 5 ? "" : ",");
    }
    std::cout << "]";

    g_vlm_left_hand->setAngles(kVlmLeftHandIndex, angles);
    if (direct_mode) {
        for (int i = 0; i < 6; ++i) g_vlm_last_direct_angles[i] = angles[i];
        g_vlm_last_direct_angles_valid = true;
        g_vlm_last_hand_close_valid = false;
    } else if (close_hand) {
        g_vlm_last_hand_close_valid = true;
        g_vlm_last_hand_close = true;
        g_vlm_last_direct_angles_valid = false;
    } else {
        ++g_vlm_left_hand_open_attempts;
        g_vlm_left_hand_next_open_check =
            now + kVlmLeftHandOpenVerifyInterval;
        g_vlm_left_hand_next_open_command =
            now + kVlmLeftHandOpenCommandInterval;
        g_vlm_last_hand_close_valid = false;
        g_vlm_last_direct_angles_valid = false;
        std::cout << " command=" << g_vlm_left_hand_open_attempts
                  << "/" << kVlmLeftHandOpenMaxCommands;
    }
    std::cout << std::endl;
    return RetCode::kSuccess;
}
}
#endif

bool envFlagEnabled(const char* name) {
    const char* value = std::getenv(name);
    if (!value) return false;
    std::string text(value);
    return text == "1" || text == "true" || text == "TRUE" || text == "on" || text == "ON" || text == "yes" || text == "YES";
}

int envInteger(const char* name, int default_value) {
    const char* value = std::getenv(name);
    if (!value) return default_value;
    try {
        std::size_t parsed = 0;
        const int result = std::stoi(value, &parsed);
        return parsed == std::strlen(value) ? result : default_value;
    } catch (const std::exception&) {
        return default_value;
    }
}

void save_encoder_last_position(int run_cnt, const save_data* snapshot);

void* write_data_task(void* arg){
    // std::cout << "write_data_task Thread ID (TID): " << syscall(SYS_gettid) << std::endl;
    save_data save_item;
    int j;
    int persisted_items = 0;
    // Do not drain queued log records during shutdown. File I/O must never
    // delay Ctrl+C or prevent the real-time process from joining this thread.
    while (!shutdown_all_threads){
        if (queue_poptop(&save_item)){
            if (shutdown_all_threads) break;
            fprintf(fwrite_data, "%ld %ld %ld %ld ", save_item.running_cost, save_item.robotics_total_cost, save_item.communication_cost, save_item.hardware_communication_cost);
            fprintf(fwrite_data, "%ld %ld %ld %ld %ld %ld ", save_item.robotics_setup_cost, save_item.robotics_Null_cost, save_item.robotics_FK_cost, save_item.robotics_IK_cost, save_item.robotics_ID_cost, save_item.robotics_simulation_addition_cost);
            fprintf(fwrite_data, "%d %d ", save_item.command_mode, save_item.motion_type);

            for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%lf %lf %lf %lf %lf %lf %lf %lf ", save_item.sliced_tar[arm_i][j], save_item.motor_target[arm_i][j], 
                        save_item.joint_position[arm_i][j], save_item.joint_velocity[arm_i][j], save_item.joint_acceleration[arm_i][j],
                        save_item.motor_position[arm_i][j], save_item.motor_velocity[arm_i][j], save_item.motor_torque[arm_i][j]);
                }

                for (j = 0; j < 7; ++ j){
                    fprintf(fwrite_data,"%lf ", save_item.tool_pose[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%lf ", save_item.target_before_interpolation[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%lf ", save_item.motor_raw_position[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.joint_gravity[arm_i][j]);
                }
                
                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.gravity[arm_i][j]);
                }
                
                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.sat_motor_cmd[arm_i][j]);
                }
                
                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.joint_impedance[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.impedance[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%lf ", save_item.joint_torque[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%lf ", save_item.torque_sensor_raw[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%lf ", save_item.torque_sensor[arm_i][j]);
                }

                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%lf ", save_item.encoder_sensor_raw[arm_i][j]);
                }
                
                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.friction[arm_i][j]);
                }
                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.gravity_kd_effect[arm_i][j]);
                }
                for (j = 0; j < robot_state.JointSize[arm_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.filtered_joint_vel[arm_i][j]);
                }
            }
            
            for (int gri_i = 0; gri_i < robot_state.GripperSize; ++ gri_i){
                for (j = 0; j < robot_state.GripperJointSize[gri_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.gripper_command[gri_i][j]);
                }
            }
            for (int gri_i = 0; gri_i < robot_state.GripperSize; ++ gri_i){
                for (j = 0; j < robot_state.GripperJointSize[gri_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.gripper_act_set_command[gri_i][j]);
                }
            }
            for (int gri_i = 0; gri_i < robot_state.GripperSize; ++ gri_i){
                for (j = 0; j < robot_state.GripperJointSize[gri_i]; ++ j){
                    fprintf(fwrite_data,"%f ", save_item.gripper_position[gri_i][j]);
                }
            }

            fprintf(fwrite_data, "\n");
            if (!shutdown_all_threads) {
                save_encoder_last_position(persisted_items++, &save_item);
            }
        }
        sleep_period(write_data_delay_us);
    }
    std::cout << "write_data_task exited.\n";
    return nullptr;
}

void* receive_udp_task(void* arg){
    // std::cout << "receive_udp_task Thread ID (TID): " << syscall(SYS_gettid) << std::endl;
    long t0, t1;
    std::unique_ptr<PanelCommand> temp_ptr;
    while (!shutdown_all_threads){
        t0 = get_time_now();
        temp_ptr = std::make_unique<PanelCommand>();
        if (udp_ptr->receive(temp_ptr.get(), 50)){
            last_panel_command_time_us.store(get_time_now(), std::memory_order_relaxed);
            panel_command_seen.store(true, std::memory_order_release);
            std::lock_guard<std::mutex> lock(planel_command_mutex);
            transmit_panel_command_ptr = std::move(temp_ptr);
        }

        t1 = get_time_now();
        sleep_period(udp_receive_delay_us);
    }
    std::cout << "receive_udp_task exited.\n";
    return nullptr;
}

void safety_check(){
    if (shutdown_all_threads) return;

    //Pause for one more circle after re-enabled motor_set to ensure motor data updated
    bool check_joint_follow = !rt_config.pause_joint_follow_check && planner_command_ptr->enable_set;
    check_joint_follow &= !panel_command_ptr->simulation && !rt_config.hardware_simulation;
    if (!planner_command_ptr->enable_set)
        rt_config.pause_joint_follow_check = true;
    else if (rt_config.pause_joint_follow_check && planner_command_ptr->enable_set)
        rt_config.pause_joint_follow_check = false;

    for (int arm_i = 0; arm_i < planner_state_ptr->ArmSize; ++ arm_i){
        int joint_size = planner_state_ptr->JointSize[arm_i];
        for (int joint_i = 0; joint_i < joint_size; ++ joint_i){
            float read_pos = planner_state_ptr->JointPos[arm_i][joint_i] * 180.0 / PI;
            float read_vel = planner_state_ptr->JointVel[arm_i][joint_i] * 180.0 / PI;
            float read_acc = planner_state_ptr->joint_acceleration[arm_i][joint_i] * 180.0 / PI;
            float read_tor = planner_state_ptr->JointTor[arm_i][joint_i];

            if (!std::isfinite(read_pos) || !std::isfinite(read_vel) ||
                !std::isfinite(read_acc) || !std::isfinite(read_tor)) {
                printf("Arm %d Joint %d returned a non-finite hardware state.\n", arm_i, joint_i);
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultHardwareCommunication;
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |=
                    DiagnosticFlags::kFaultHardwareCommunication;
                shutdown_all_threads = true;
                break;
            }

            //Ensure Position within limit 
            if (read_pos < rt_config.joint_position_limit[0][arm_i][joint_i] || read_pos > rt_config.joint_position_limit[1][arm_i][joint_i]){
                printf("Arm %d Joint %d position exceed limit. Val: %f (Min: %f, Max: %f).\n", arm_i, joint_i, read_pos,
                        rt_config.joint_position_limit[0][arm_i][joint_i], rt_config.joint_position_limit[1][arm_i][joint_i]);
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultPosHardLimitReached;
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |= DiagnosticFlags::kFaultPosHardLimitReached;    
                shutdown_all_threads = true;
                break;
            }

            //Ensure Velocity within limit 
            if (read_vel < rt_config.joint_velocity_limit[0][arm_i][joint_i] || read_vel > rt_config.joint_velocity_limit[1][arm_i][joint_i]){
                printf("Arm %d Joint %d velocity exceed limit. Val: %f (Min: %f, Max: %f).\n", arm_i, joint_i, read_vel,
                        rt_config.joint_velocity_limit[0][arm_i][joint_i], rt_config.joint_velocity_limit[1][arm_i][joint_i]);
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultVelHardLimitReached;
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |= DiagnosticFlags::kFaultVelHardLimitReached;
                shutdown_all_threads = true;
                break;
            }

            //Ensure Torque within limit 
            if (read_tor < rt_config.joint_torque_limit[0][arm_i][joint_i] || read_tor > rt_config.joint_torque_limit[1][arm_i][joint_i]){
                printf("Arm %d Joint %d torque exceed limit. Val: %f (Min: %f, Max: %f).\n", arm_i, joint_i, read_tor,
                        rt_config.joint_torque_limit[0][arm_i][joint_i], rt_config.joint_torque_limit[1][arm_i][joint_i]);
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultTorHardLimitReached;
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |= DiagnosticFlags::kFaultTorHardLimitReached;
                shutdown_all_threads = true;
                break;
            }

            float set_cmd = rt_config.last_planner_target[arm_i][joint_i];
            if(check_joint_follow && panel_command_ptr->enable_joint[arm_i][joint_i]){
                set_cmd = (planner_command_ptr->command_mode == ControlType::kTorque)? set_cmd : set_cmd * 180.0 / PI;
                if (planner_command_ptr->command_mode == ControlType::kPosition) {
                    //Ensure Position follow
                    float error = std::abs(read_pos - set_cmd);
                    if (error > rt_config.joint_position_follow_threshold[arm_i][joint_i]){
                        printf("Arm %d Joint %d position failed to follow. Target: %f, Actual: %f, Error: %f\n", 
                                arm_i, joint_i, set_cmd, read_pos, error);
                        planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultPosTrackingFailed;
                        planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |= DiagnosticFlags::kFaultPosTrackingFailed;
                        shutdown_all_threads = true;
                        break;
                    }
                } else if (planner_command_ptr->command_mode == ControlType::kVelocity) {
                    //Ensure Velocity follow
                    float error = std::abs(read_vel - set_cmd);
                    if (error > rt_config.joint_velocity_follow_threshold[arm_i][joint_i]){
                        printf("Arm %d Joint %d velocity failed to follow. Target: %f, Actual: %f, Error: %f\n", 
                                arm_i, joint_i, set_cmd, read_vel, error);
                        planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultVelTrackingFailed;
                        planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |= DiagnosticFlags::kFaultVelTrackingFailed;
                        shutdown_all_threads = true;
                        break;
                    }
                }else if (planner_command_ptr->command_mode == ControlType::kTorque) { 
                    //Ensure Torque follow
                    float error = std::abs(read_tor - set_cmd);
                    if (error > rt_config.joint_torque_follow_threshold[arm_i][joint_i]){
                        printf("Arm %d Joint %d torque failed to follow. Target: %f, Actual: %f, Error: %f\n", 
                                arm_i, joint_i, set_cmd, read_tor, error);
                        planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultTorTrackingFailed;
                        planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |= DiagnosticFlags::kFaultTorTrackingFailed;
                        shutdown_all_threads = true;
                        break;
                    }
                }
            }
            rt_config.last_planner_target[arm_i][joint_i] = planner_command_ptr->Target[arm_i][joint_i]; 
        }
    }
    if (shutdown_all_threads){
        planner_state_ptr->system_state = SystemState::kError;
    }
}

void apply_startup_position_mismatch_warning(){
    if (!startup_position_mismatch_active.load(std::memory_order_acquire) ||
        !planner_state_ptr) {
        return;
    }

    planner_state_ptr->system_diagnostic_flags |=
        DiagnosticFlags::kWarningStartupPositionMismatch;
    for (int arm_i = 0;
         arm_i < std::min<int>(
                     rt_config.startup_position_mismatch_joint.size(),
                     MAX_ARM_SIZE);
         ++arm_i) {
        const int joint_count = std::min<int>(
            rt_config.startup_position_mismatch_joint[arm_i].size(),
            MAX_JOINT_SIZE);
        for (int joint_i = 0; joint_i < joint_count; ++joint_i) {
            if (rt_config.startup_position_mismatch_joint[arm_i][joint_i]) {
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |=
                    DiagnosticFlags::kWarningStartupPositionMismatch;
            }
        }
    }
}

bool is_upper_zero_recovery_command(){
    if (!panel_command_ptr ||
        panel_command_ptr->connection_state != ConnectionState::kRemote ||
        panel_command_ptr->motion_type != MotionControl::kJoint ||
        panel_command_ptr->target_type != ControlType::kPosition ||
        panel_command_ptr->actuator_mode != ControlType::kPosition ||
        !panel_command_ptr->need_setting_update) {
        return false;
    }

    bool has_enabled_joint = false;
    for (int arm_i = 0; arm_i < panel_command_ptr->ArmSize; ++arm_i) {
        for (int joint_i = 0;
             joint_i < panel_command_ptr->JointSize[arm_i];
             ++joint_i) {
            if (!panel_command_ptr->enable_joint[arm_i][joint_i]) {
                continue;
            }
            has_enabled_joint = true;
            if (std::abs(panel_command_ptr->JointCmd[arm_i][joint_i]) >
                1e-4f) {
                return false;
            }
        }
    }
    return has_enabled_joint;
}

void update_startup_zero_recovery(){
    if (!startup_position_mismatch_active.load(std::memory_order_acquire) ||
        !planner_state_ptr) {
        return;
    }

    apply_startup_position_mismatch_warning();
    if (!startup_zero_command_seen && is_upper_zero_recovery_command()) {
        startup_zero_command_seen = true;
        std::cout
            << "[Recovery] Upper Software zero command accepted. Waiting for "
               "all enabled joints to settle at zero."
            << std::endl;
    }
    if (!startup_zero_command_seen) {
        startup_zero_stable_cycles = 0;
        return;
    }

    bool all_enabled_joints_at_zero = true;
    for (int arm_i = 0; arm_i < planner_state_ptr->ArmSize; ++arm_i) {
        for (int joint_i = 0;
             joint_i < planner_state_ptr->JointSize[arm_i];
             ++joint_i) {
            if (!planner_state_ptr->enabled_joint[arm_i][joint_i]) {
                continue;
            }
            if (std::abs(planner_state_ptr->JointPos[arm_i][joint_i]) >
                    kStartupZeroPositionToleranceRad ||
                std::abs(planner_state_ptr->JointVel[arm_i][joint_i]) >
                    kStartupZeroVelocityToleranceRadS) {
                all_enabled_joints_at_zero = false;
                break;
            }
        }
        if (!all_enabled_joints_at_zero) {
            break;
        }
    }

    if (!all_enabled_joints_at_zero) {
        startup_zero_stable_cycles = 0;
        return;
    }

    ++startup_zero_stable_cycles;
    const int required_stable_cycles = std::max(
        1, kStartupZeroStableDurationUs / std::max(control_loop_delay_us, 1));
    if (startup_zero_stable_cycles < required_stable_cycles) {
        return;
    }

    startup_position_mismatch_active.store(false, std::memory_order_release);
    startup_zero_command_seen = false;
    startup_zero_stable_cycles = 0;
    planner_state_ptr->system_diagnostic_flags &=
        ~DiagnosticFlags::kWarningStartupPositionMismatch;
    for (int arm_i = 0;
         arm_i < std::min<int>(
                     rt_config.startup_position_mismatch_joint.size(),
                     MAX_ARM_SIZE);
         ++arm_i) {
        for (int joint_i = 0;
             joint_i < std::min<int>(
                           rt_config.startup_position_mismatch_joint[arm_i].size(),
                           MAX_JOINT_SIZE);
             ++joint_i) {
            planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] &=
                ~DiagnosticFlags::kWarningStartupPositionMismatch;
            rt_config.startup_position_mismatch_joint[arm_i][joint_i] = false;
        }
    }

    std::cout
        << "[Recovery] All enabled joints reached zero. Startup position "
           "warning cleared and encoder snapshot refreshed."
        << std::endl;
    save_encoder_last_position(0, nullptr);
}

bool validate_panel_command(){
    if (!panel_command_ptr) {
        mark_system_fault(DiagnosticFlags::kFaultInvalidCommand, "PanelCommand pointer is null.");
        return false;
    }

    bool valid = true;
    auto reject = [&](const std::string& reason) {
        std::cerr << "[COMMAND REJECTED] " << reason << std::endl;
        valid = false;
    };
    auto enum_valid = [](int value, int minimum, int maximum) {
        return value >= minimum && value <= maximum;
    };

    if (panel_command_ptr->ArmSize < 0 || panel_command_ptr->ArmSize > MAX_ARM_SIZE ||
        panel_command_ptr->ArmSize != robot_state.ArmSize) {
        reject("Arm size mismatch or out of range. Panel=" +
               std::to_string(panel_command_ptr->ArmSize) + ", Planner=" +
               std::to_string(robot_state.ArmSize));
    }
    if (panel_command_ptr->ArmWaypointSize < 0 ||
        panel_command_ptr->ArmWaypointSize > MAX_WAYPOINTS) {
        reject("Waypoint count is out of range.");
    }
    if (panel_command_ptr->GripperSize < 0 ||
        panel_command_ptr->GripperSize > MAX_ARM_SIZE) {
        reject("Gripper count is out of range.");
    }

    valid &= enum_valid(static_cast<int>(panel_command_ptr->connection_state),
                        static_cast<int>(ConnectionState::kWaiting),
                        static_cast<int>(ConnectionState::kShutDown));
    valid &= enum_valid(static_cast<int>(panel_command_ptr->target_type),
                        static_cast<int>(ControlType::kPosition),
                        static_cast<int>(ControlType::kTorque));
    valid &= enum_valid(static_cast<int>(panel_command_ptr->actuator_mode),
                        static_cast<int>(ControlType::kPosition),
                        static_cast<int>(ControlType::kTorque));
    valid &= enum_valid(static_cast<int>(panel_command_ptr->motion_type),
                        static_cast<int>(MotionControl::kJoint),
                        static_cast<int>(MotionControl::kPlayback));
    valid &= enum_valid(static_cast<int>(panel_command_ptr->arm_target_mode),
                        static_cast<int>(PanelTargetMode::kSinglePoint),
                        static_cast<int>(PanelTargetMode::kRawSinglePoint));

    const int safe_arm_count =
        std::min(static_cast<int>(robot_state.ArmSize), MAX_ARM_SIZE);
    for (int arm_i = 0; arm_i < safe_arm_count; ++arm_i) {
        const std::size_t panel_name_length =
            strnlen(panel_command_ptr->RobotName[arm_i], MAX_ROBOTNAME_SIZE);
        if (panel_name_length >= MAX_ROBOTNAME_SIZE ||
            rt_config.arm_name[arm_i] !=
                std::string(panel_command_ptr->RobotName[arm_i], panel_name_length)) {
            reject("Robot name mismatch for arm " + std::to_string(arm_i) + ".");
        }

        if (panel_command_ptr->JointSize[arm_i] < 0 ||
            panel_command_ptr->JointSize[arm_i] > MAX_JOINT_SIZE ||
            panel_command_ptr->JointSize[arm_i] != robot_state.JointSize[arm_i]) {
            reject("Joint size mismatch for arm " + std::to_string(arm_i) + ".");
            continue;
        }

        for (int joint_i = 0; joint_i < robot_state.JointSize[arm_i]; ++joint_i) {
            if (!std::isfinite(panel_command_ptr->MotorCmd[arm_i][joint_i]) ||
                !std::isfinite(panel_command_ptr->JointCmd[arm_i][joint_i]) ||
                !std::isfinite(panel_command_ptr->arm_soft_limit_position[arm_i][joint_i][0]) ||
                !std::isfinite(panel_command_ptr->arm_soft_limit_position[arm_i][joint_i][1]) ||
                !std::isfinite(panel_command_ptr->arm_soft_limit_velocity[arm_i][joint_i][0]) ||
                !std::isfinite(panel_command_ptr->arm_soft_limit_velocity[arm_i][joint_i][1])) {
                reject("Non-finite value for arm " + std::to_string(arm_i) +
                       " joint " + std::to_string(joint_i) + ".");
                break;
            }
        }
        for (int task_i = 0; task_i < 6; ++task_i) {
            if (!std::isfinite(panel_command_ptr->TaskCmd[arm_i][task_i])) {
                reject("Non-finite task command.");
                break;
            }
        }
    }

    if (!std::isfinite(panel_command_ptr->interpolation_speed_ratio) ||
        !std::isfinite(panel_command_ptr->InterpolationAccTime) ||
        !std::isfinite(panel_command_ptr->InterpolationConstVelTime) ||
        !std::isfinite(panel_command_ptr->NoneInterpolationSaturationRatio) ||
        !std::isfinite(panel_command_ptr->ArmWaypointDt)) {
        reject("Non-finite interpolation parameter.");
    }

    if (!panel_command_ptr->simulation) {
        if (panel_command_ptr->GripperSize != robot_state.GripperSize) {
            reject("Gripper size mismatch.");
        } else {
            for (int gripper_i = 0; gripper_i < robot_state.GripperSize; ++gripper_i) {
                if (panel_command_ptr->GripperJointSize[gripper_i] < 0 ||
                    panel_command_ptr->GripperJointSize[gripper_i] > MAX_JOINT_SIZE ||
                    panel_command_ptr->GripperJointSize[gripper_i] !=
                        robot_state.GripperJointSize[gripper_i]) {
                    reject("Gripper joint size mismatch for gripper " +
                           std::to_string(gripper_i) + ".");
                }
            }
        }
    }

    if (!valid) {
        mark_system_fault(DiagnosticFlags::kFaultInvalidCommand,
                          "Invalid PanelCommand caused a fail-safe stop.");
    }
    return valid;
}

void save_encoder_last_position(int run_cnt, const save_data* snapshot){
    if (shutdown_all_threads || encoder_last_position_path.empty() ||
        rt_config.hardware_simulation) {
        return;
    }
    std::lock_guard<std::mutex> snapshot_lock(encoder_snapshot_mutex);
    if (startup_position_mismatch_active.load(std::memory_order_acquire)) {
        return;
    }

    const int period_us = std::max(write_data_delay_us, 1);
    const int cnt_per_second = std::max(1, 1000000 / period_us);
    if (run_cnt != 0 && run_cnt % cnt_per_second != 0) return;

    const std::string temporary_path = encoder_last_position_path + ".tmp";
    std::ofstream output(temporary_path, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        mark_system_fault(DiagnosticFlags::kFaultHardwareInitFailed,
                          "Cannot open temporary encoder snapshot file.");
        return;
    }

    for (int arm_i = 0; arm_i < robot_state.ArmSize; ++arm_i) {
        for (int joint_i = 0; joint_i < robot_state.JointSize[arm_i]; ++joint_i) {
            const float position_rad = snapshot
                ? static_cast<float>(snapshot->motor_position[arm_i][joint_i] * PI / 180.0)
                : robot_state.MotorPos[arm_i][joint_i];
            output.write(reinterpret_cast<const char*>(&position_rad), sizeof(position_rad));
        }
    }
    output.flush();
    if (!output.good()) {
        output.close();
        std::remove(temporary_path.c_str());
        mark_system_fault(DiagnosticFlags::kFaultHardwareInitFailed,
                          "Failed to persist the encoder snapshot.");
        return;
    }
    output.close();

    if (std::rename(temporary_path.c_str(), encoder_last_position_path.c_str()) != 0) {
        std::remove(temporary_path.c_str());
        mark_system_fault(DiagnosticFlags::kFaultHardwareInitFailed,
                          "Failed to atomically replace the encoder snapshot.");
    }
}

bool set_and_read_arm(int arm_i, ArmBase* robot_p, MotorAction action = MotorAction::kNormal){
    if (rt_config.hardware_simulation || panel_command_ptr == nullptr || planner_command_ptr == nullptr) return true;
    if (!robot_p) {
        mark_system_fault(DiagnosticFlags::kFaultHardwareCommunication,
                          "Arm " + std::to_string(arm_i) + " is unavailable.");
        return false;
    }

    int can_delay_us = 10;
    int joint_size = robot_p->getJointSize();

    // Enable Motor
    std::vector<bool> enable_cmd(panel_command_ptr->enable_joint[arm_i], panel_command_ptr->enable_joint[arm_i] + joint_size);
    if (arm_ptr_vec[arm_i]->enableMotors(enable_cmd) != RetCode::kSuccess) {
        mark_system_fault(DiagnosticFlags::kFaultHardwareCommunication,
                          "Failed to update motor enable state for arm " + std::to_string(arm_i) + ".");
        return false;
    }
    for (int joint_i=0; joint_i<joint_size; joint_i++){
        planner_state_ptr->enabled_joint[arm_i][joint_i] = enable_cmd[joint_i];
    }

    // Set Operation mode
    RetCode set_mode_ret = RetCode::kFailed;
    if (action == MotorAction::kInit || action == MotorAction::kShutDown){
        set_mode_ret = robot_p->setMode(ControlType::kPosition);
    }else if (action == MotorAction::kNormal){  
        set_mode_ret = robot_p->setMode(planner_command_ptr->command_mode);
    }
    if (set_mode_ret != RetCode::kSuccess) {
        mark_system_fault(DiagnosticFlags::kFaultHardwareCommunication,
                          "Failed to set control mode for arm " + std::to_string(arm_i) + ".");
        return false;
    }

    // Set & read robot
    std::vector<float> command(joint_size, 0.0f);
    std::vector<float> position(&robot_state.MotorPos[arm_i][0],  &robot_state.MotorPos[arm_i][0] + joint_size);
    std::vector<float> velocity(&robot_state.MotorVel[arm_i][0],  &robot_state.MotorVel[arm_i][0] + joint_size);
    std::vector<float> torque(&robot_state.MotorTor[arm_i][0],  &robot_state.MotorTor[arm_i][0] + joint_size);
    RetCode io_ret = RetCode::kFailed;
    if (action == MotorAction::kInit){
        io_ret = robot_p->getState(&position, &velocity, &torque);
    } else if (action == MotorAction::kShutDown) {
        command = position;
        io_ret = robot_p->setCommandAndGetState(command, &position, &velocity, &torque);
    }else{
        if (planner_command_ptr->enable_set){
            for (int joint_i =0; joint_i < joint_size; ++ joint_i){
                command[joint_i] = planner_command_ptr->Target[arm_i][joint_i];
            }
            io_ret = robot_p->setCommandAndGetState(command, &position, &velocity, &torque);
        }else{ //Read only
            io_ret = robot_p->getState(&position, &velocity, &torque);
        }
    }
    if (io_ret != RetCode::kSuccess) {
        // Judge each arm independently. A gripper-only action may make the
        // global system non-idle while both arms are safely holding position.
        bool confirmed_stationary_hold =
            action == MotorAction::kNormal &&
            planner_state_ptr != nullptr &&
            planner_command_ptr->connection_state == ConnectionState::kRemote &&
            planner_command_ptr->enable_set &&
            planner_command_ptr->command_mode == ControlType::kPosition &&
            arm_i >= 0 &&
            arm_i < static_cast<int>(rt_config.last_planner_target.size()) &&
            arm_i < static_cast<int>(rt_config.can_consecutive_exchange_failures.size());

        if (confirmed_stationary_hold) {
            for (int joint_i = 0; joint_i < joint_size; ++joint_i) {
                if (!panel_command_ptr->enable_joint[arm_i][joint_i]) continue;
                const float target = planner_command_ptr->Target[arm_i][joint_i];
                const float previous_target =
                    rt_config.last_planner_target[arm_i][joint_i];
                const float position = robot_state.MotorPos[arm_i][joint_i];
                const float velocity = robot_state.MotorVel[arm_i][joint_i];
                if (!std::isfinite(target) ||
                    !std::isfinite(previous_target) ||
                    !std::isfinite(position) ||
                    !std::isfinite(velocity) ||
                    std::abs(target - previous_target) >
                        kCanStationaryHoldTargetDeltaRad ||
                    std::abs(target - position) >
                        kCanStationaryHoldPositionErrorRad ||
                    std::abs(velocity) >
                        kCanStationaryHoldVelocityThresholdRadS) {
                    confirmed_stationary_hold = false;
                    break;
                }
            }
        }

        int consecutive_failures = 1;
        if (arm_i >= 0 &&
            arm_i < static_cast<int>(rt_config.can_consecutive_exchange_failures.size())) {
            consecutive_failures =
                ++rt_config.can_consecutive_exchange_failures[arm_i];
        }
        if (confirmed_stationary_hold &&
            consecutive_failures <= kCanStationaryHoldMaxTransientFailures) {
            std::cerr << "[WARN] Transient CAN command/state exchange failure for arm "
                      << arm_i << " while confirmed per-arm stationary hold; attempt "
                      << consecutive_failures << "/"
                      << kCanStationaryHoldMaxTransientFailures
                      << ", ret=" << static_cast<int>(io_ret)
                      << ". Keeping the last confirmed state." << std::endl;
            return true;
        }

        if (planner_state_ptr) {
            planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultHardwareCommunication;
            for (int joint_i = 0; joint_i < joint_size; ++joint_i) {
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |=
                    DiagnosticFlags::kFaultHardwareCommunication;
            }
        }
        mark_system_fault(DiagnosticFlags::kFaultHardwareCommunication,
                          "CAN command/state exchange failed for arm " +
                              std::to_string(arm_i) +
                              " (ret=" + std::to_string(static_cast<int>(io_ret)) +
                              ", consecutive=" +
                              std::to_string(consecutive_failures) +
                              ", mode=" +
                              (confirmed_stationary_hold
                                   ? "stationary-hold"
                                   : "strict") +
                              ").");
        return false;
    }

    if (arm_i >= 0 &&
        arm_i < static_cast<int>(rt_config.can_consecutive_exchange_failures.size()) &&
        rt_config.can_consecutive_exchange_failures[arm_i] > 0) {
        std::cerr << "[Recovery] CAN command/state exchange recovered for arm "
                  << arm_i << " after "
                  << rt_config.can_consecutive_exchange_failures[arm_i]
                  << " transient failure(s)." << std::endl;
        rt_config.can_consecutive_exchange_failures[arm_i] = 0;
    }

    for (int joint_i =0; joint_i < joint_size; ++ joint_i){
        robot_state.MotorPos[arm_i][joint_i]   = position[joint_i];
        robot_state.MotorVel[arm_i][joint_i]   = velocity[joint_i];
        robot_state.MotorTor[arm_i][joint_i]   = torque[joint_i];
    }
    sleep_period(can_delay_us);
    return true;
}

void set_and_read_gripper(int gripper_i, AbstractGripper* gripper_ptr){
    if (rt_config.hardware_simulation) return;

    //Set gripper command
    std::vector<float> normalized_cmd(panel_command_ptr->GripperJointSize[gripper_i]);
    normalized_cmd.assign(panel_command_ptr->GripperJointCmd[gripper_i], panel_command_ptr->GripperJointCmd[gripper_i] + panel_command_ptr->GripperJointSize[gripper_i]);

#ifdef DUAL_V2_2
    if (gripper_i == kVlmPanelHandIndex) {
#ifdef CUARM_RT_CONTROL_IGNORE_LEFT_HAND
        return;
#elif defined(CUARM_RT_CONTROL_NO_HAND)
        g_vlm_left_hand_last_panel_command = normalized_cmd;
#else
        setVlmLeftHandFromPanelCommand(normalized_cmd);
#endif
        std::copy(normalized_cmd.begin(), normalized_cmd.end(), robot_state.GripperJointPos[gripper_i]);
        return;
    }

    if (gripper_i == kVlmRightGripperIndex && normalized_cmd.size() == 1) {
        float panel_value = (normalized_cmd[0] > kVlmRightGripperCloseThreshold) ? 1.0f : 0.0f;
        gripper_ptr->setPanelCommand({panel_value});
    } else {
        gripper_ptr->setPanelCommand(normalized_cmd);
    }
#else
    gripper_ptr->setPanelCommand(normalized_cmd);
#endif

    //Read back gripper position
    std::vector<float> read_normalized_cmd(panel_command_ptr->GripperJointSize[gripper_i]);
    if (gripper_ptr->getPanelPos(read_normalized_cmd) == RetCode::kSuccess){
        std::copy(read_normalized_cmd.begin(), read_normalized_cmd.end(), robot_state.GripperJointPos[gripper_i]);
    }
}

void* control_task(void* arg)
{ 
    // std::cout << "control_task Thread ID (TID): " << syscall(SYS_gettid) << std::endl;
    printf("Task Start!\n");
    int  run_cnt = 0;
    int consecutive_deadline_misses = 0;
    bool session_synchronized = false;
    long t1, t2, t3, t4, t5, t6;
    long t_max = 0, t_sum = 0, t_min = 1000000, tt_max = 0, tt_sum = 0, tt_min = 1000000;
    long t0 = get_time_now();
    planner_command_ptr->connection_state = ConnectionState::kWaiting;
    planner_state_ptr->system_state = SystemState::kIdle;
    while (planner_command_ptr->connection_state != ConnectionState::kShutDown && !shutdown_all_threads){
        //Update panel command
        if (transmit_panel_command_ptr){
            std::lock_guard<std::mutex> lock(planel_command_mutex);
            panel_command_ptr = std::move(transmit_panel_command_ptr);
        }

        if (panel_command_ptr && !session_synchronized &&
            panel_command_ptr->connection_state != ConnectionState::kWaiting) {
            if (run_cnt % 100 == 0) {
                std::cerr << "[SESSION] Ignoring command until a Waiting handshake is received."
                          << std::endl;
            }
            panel_command_ptr.reset();
        }
        
        if (panel_command_ptr){
            if (!validate_panel_command()) break;
            if (!session_synchronized) {
                session_synchronized = true;
                std::cout << "[SESSION] Panel handshake accepted; waiting for synchronized targets."
                          << std::endl;
            }

            if (session_synchronized &&
                planner_command_ptr->connection_state == ConnectionState::kRemote &&
                panel_command_seen.load(std::memory_order_acquire)) {
                const long command_age =
                    get_time_now() -
                    last_panel_command_time_us.load(std::memory_order_relaxed);
                if (command_age > rt_config.command_watchdog_timeout_us) {
                    mark_system_fault(
                        DiagnosticFlags::kFaultCommandTimeout,
                        "Panel command watchdog expired after " +
                            std::to_string(command_age / 1000) + " ms.");
                    break;
                }
            }
            
            //Update planner command
            planner_ptr->update(*panel_command_ptr, robot_state);
            new_data.robotics_total_cost = get_time_now()-t0;

            if (planner_command_ptr->connection_state == ConnectionState::kRemote){
                //Set & read arms
                if (!skip_arm_test_mode) {
                    for (int arm_i=0; arm_i<arm_ptr_vec.size(); arm_i++){
                        if (panel_command_ptr->simulation){
                            if (!set_and_read_arm(arm_i, arm_ptr_vec[arm_i].get(), MotorAction::kInit)) break;
                        }else{
                            if (!set_and_read_arm(arm_i, arm_ptr_vec[arm_i].get())) break;
                        }
                    }
                }

                //Set & read gripper
                for (int gri_i=0; gri_i<gripper_ptr_vec.size(); gri_i++){
                    if (!panel_command_ptr->simulation && gripper_ptr_vec[gri_i]) {
                        set_and_read_gripper(gri_i, gripper_ptr_vec[gri_i].get());
                    }
#ifdef DUAL_V2_2
                    else if (!panel_command_ptr->simulation && gri_i == kVlmPanelHandIndex) {
                        set_and_read_gripper(gri_i, nullptr);
                    }
#endif
                }
                new_data.hardware_communication_cost = get_time_now()-t0;
            }
            
            //Update robot state to planner state
            planner_ptr->updatePlannerState(robot_state);
            update_startup_zero_recovery();

            if (planner_command_ptr->connection_state == ConnectionState::kWaiting) {
                for (int arm_i = 0; arm_i < robot_state.ArmSize; ++arm_i) {
                    for (int joint_i = 0; joint_i < robot_state.JointSize[arm_i]; ++joint_i) {
                        rt_config.last_planner_target[arm_i][joint_i] =
                            planner_state_ptr->JointPos[arm_i][joint_i];
                    }
                }
            }
            
            safety_check();
            
            //Update log data
            float k = (planner_command_ptr->command_mode != ControlType::kTorque)? 180/PI : 1;
            for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
                for (int j = 0; j < robot_state.JointSize[arm_i]; ++ j) {
                    new_data.command_mode = (int)planner_command_ptr->command_mode;
                    new_data.motion_type = (int)panel_command_ptr->motion_type;

                    new_data.motor_target[arm_i][j]         = planner_command_ptr->Target[arm_i][j] * k;
                    new_data.motor_position[arm_i][j]       = robot_state.MotorPos[arm_i][j] * 180/PI;
                    new_data.motor_velocity[arm_i][j]       = robot_state.MotorVel[arm_i][j] * 180/PI;
                    new_data.motor_torque[arm_i][j]         = robot_state.MotorTor[arm_i][j];
                    new_data.joint_position[arm_i][j]       = planner_state_ptr->JointPos[arm_i][j]*180/PI;
                    new_data.joint_velocity[arm_i][j]       = planner_state_ptr->JointVel[arm_i][j]*180/PI;
                    new_data.joint_acceleration[arm_i][j]   = planner_state_ptr->joint_acceleration[arm_i][j]*180/PI;
                    new_data.joint_torque[arm_i][j]         = planner_state_ptr->JointTor[arm_i][j];
                    new_data.sat_motor_cmd[arm_i][j]   = planner_state_ptr->sat_motor_cmd_[arm_i][j];

                    new_data.gravity[arm_i][j]         = planner_state_ptr->gravity[arm_i][j];
                    new_data.joint_gravity[arm_i][j]   = planner_state_ptr->joint_gravity[arm_i][j];
                    new_data.impedance[arm_i][j]       = planner_state_ptr->impedance[arm_i][j];
                    new_data.joint_impedance[arm_i][j] = planner_state_ptr->joint_impedance[arm_i][j];
                    new_data.filtered_motor_torque[arm_i][j]        = planner_state_ptr->FilteredMotorTor[arm_i][j];
                    new_data.filtered_motor_torque_euro[arm_i][j]   = planner_state_ptr->FilteredMotorTorEuro[arm_i][j];
                    new_data.filtered_motor_torque_kalman[arm_i][j] = planner_state_ptr->FilteredMotorTorKalman[arm_i][j];
                    new_data.friction[arm_i][j]            = planner_state_ptr->friction[arm_i][j];
                    new_data.filtered_joint_vel[arm_i][j] = planner_state_ptr->filtered_joint_vel[arm_i][j];
                    new_data.gravity_kd_effect[arm_i][j]   = planner_state_ptr->gravity_kd_effect[arm_i][j];
                }

                for (int j=0; j<7; j++){
                    new_data.tool_pose[arm_i][j] = planner_state_ptr->ToolPose[arm_i][j];
                }

                if (panel_command_ptr->motion_type == MotionControl::kTaskLine || panel_command_ptr->motion_type == MotionControl::kTask){
                    for (int j=0; j<6; j++){
                        new_data.target_before_interpolation[arm_i][j] = planner_state_ptr->target_before_interpolation[arm_i][j];
                        if (j > 2) new_data.target_before_interpolation[arm_i][j] *= k;
                    }
                }else{
                    for (int j=0; j<planner_command_ptr->JointSize[arm_i]; j++){
                        new_data.target_before_interpolation[arm_i][j] = planner_state_ptr->target_before_interpolation[arm_i][j]*k;
                    }
                }


                if (panel_command_ptr->motion_type == MotionControl::kTaskLine){
                    for (int j=0; j<6; j++){
                        new_data.sliced_tar[arm_i][j] = planner_state_ptr->interpolated_target[arm_i][j];
                        if (j > 2) new_data.sliced_tar[arm_i][j] *= k;
                    }
                }else{
                    for (int j=0; j<planner_command_ptr->JointSize[arm_i]; j++){
                        new_data.sliced_tar[arm_i][j] = planner_state_ptr->interpolated_target[arm_i][j]*k;
                    }
                }
            }

            //Update gripper log
            for (int gri_i=0; gri_i<gripper_ptr_vec.size(); gri_i++){
                std::vector<float> last_set_command;
                if (gripper_ptr_vec[gri_i]) {
                    last_set_command = gripper_ptr_vec[gri_i]->getLastSetCommand();
                }
#ifdef DUAL_V2_2
                else if (gri_i == kVlmPanelHandIndex) {
                    last_set_command = g_vlm_left_hand_last_panel_command;
                }
#endif
                for (int gripper_joint_i=0; gripper_joint_i< panel_command_ptr->GripperJointSize[gri_i]; gripper_joint_i++){
                    new_data.gripper_command[gri_i][gripper_joint_i] = panel_command_ptr->GripperJointCmd[gri_i][gripper_joint_i];
                    new_data.gripper_act_set_command[gri_i][gripper_joint_i] = gripper_joint_i < last_set_command.size() ? last_set_command[gripper_joint_i] : 0.0f;
                    new_data.gripper_position[gri_i][gripper_joint_i] = robot_state.GripperJointPos[gri_i][gripper_joint_i];
                }
            }
            
            //Update planner cost
            new_data.robotics_setup_cost = planner_state_ptr->setup_cost;
            new_data.robotics_Null_cost = planner_state_ptr->Null_cost;
            new_data.robotics_FK_cost = planner_state_ptr->FK_cost;
            new_data.robotics_IK_cost = planner_state_ptr->IK_cost;
            new_data.robotics_ID_cost = planner_state_ptr->ID_cost;
            new_data.robotics_simulation_addition_cost = planner_state_ptr->simulation_addition_cost;
        }

        // send data to up machine by udp
        planner_state_ptr->send_timestamp = get_time_now();
        try {
            udp_ptr->send(planner_state_ptr);
        } catch (const std::exception& e) {
            mark_system_fault(DiagnosticFlags::kFaultInvalidCommand,
                              std::string("Failed to pack/send PlannerState: ") + e.what());
        }

        for (int arm_i = 0; arm_i < robot_state.ArmSize; ++arm_i) {
            for (int joint_i = 0; joint_i < robot_state.JointSize[arm_i]; ++joint_i) {
                new_data.motor_position[arm_i][joint_i] =
                    robot_state.MotorPos[arm_i][joint_i] * 180.0 / PI;
            }
        }

        if (!shutdown_all_threads && run_cnt%write_freq == 0){
            int queue_sleep_cnts = 0;
            int max_queue_cnt = 50;
            while (!queue_insert(new_data) && queue_sleep_cnts < max_queue_cnt) {
                sleep_period(10);
                queue_sleep_cnts ++;
            }
        }

        t1 = get_time_now();
        new_data.communication_cost = t1 - t0;
        t_sum += new_data.communication_cost;
        if (t_min > new_data.communication_cost)
            t_min = new_data.communication_cost;
        if (t_max < new_data.communication_cost)
            t_max = new_data.communication_cost;

        if (control_loop_delay_us > t1 - t0)
            sleep_period(control_loop_delay_us - t1 + t0);

        t5 = get_time_now();
        new_data.running_cost = t5 - t0;
        if (new_data.running_cost > control_loop_delay_us * 5L) {
            ++consecutive_deadline_misses;
            if (consecutive_deadline_misses >= 5) {
                mark_system_fault(
                    DiagnosticFlags::kFaultControlDeadlineMissed,
                    "Control loop exceeded five times its deadline for five consecutive cycles.");
            }
        } else {
            consecutive_deadline_misses = 0;
        }
        tt_sum += new_data.running_cost;
        if (tt_min > new_data.running_cost)
            tt_min = new_data.running_cost;
        if (tt_max < new_data.running_cost)
            tt_max = new_data.running_cost;
        t0 = t5;
        run_cnt ++;
	}

    if (!hold_current_pose_or_disable() && planner_state_ptr) {
        planner_state_ptr->system_state = SystemState::kError;
    }
    if (planner_state_ptr && udp_ptr) {
        planner_state_ptr->send_timestamp = get_time_now();
        try {
            udp_ptr->send(planner_state_ptr);
        } catch (const std::exception& e) {
            std::cerr << "Failed to send final PlannerState: " << e.what() << std::endl;
        }
    }

    const int divisor = std::max(run_cnt, 1);
    printf("PDO max time %ld, min time %ld, ave time %ld us. [%d]\n", t_max, t_min, t_sum / divisor, run_cnt);
    printf("Task max time %ld, min time %ld, ave time %ld us. [%d]\n", tt_max, tt_min, tt_sum / divisor, run_cnt);

    shutdown_all_threads = true;
    std::cout << "Control task exited.\n";
    return nullptr;
}

#if KERNEL_PREEMPT_RT
int creat_rt_thread(pthread_t *task_handle, pthread_attr_t *task_attr, int priority, void *(*fun)(void *))
{
    // Initialize pthread attributes (default values)
    int ret = pthread_attr_init(task_attr);
    if (ret) {
        printf("init pthread attributes failed\n");
        return -1;
    }
    // Set a specific stack size  
    ret = pthread_attr_setstacksize(task_attr, 1024000);
    if (ret) {
        printf("pthread setstacksize failed\n");
        pthread_attr_destroy(task_attr);
        return -2;
    }

    // Set scheduler policy and priority of pthread
    // ret = pthread_attr_setschedpolicy(task_attr, SCHED_RR);
    ret = pthread_attr_setschedpolicy(task_attr, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        pthread_attr_destroy(task_attr);
        return -3;
    }
    struct sched_param param;
    param.sched_priority = priority;
    ret = pthread_attr_setschedparam(task_attr, &param);
    if (ret) {
        printf("pthread setschedparam failed\n");
        pthread_attr_destroy(task_attr);
        return -4;
    }

    // Use scheduling parameters of attr
    ret = pthread_attr_setinheritsched(task_attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        printf("pthread setinheritsched failed\n");
        pthread_attr_destroy(task_attr);
        return -5;
    }

    // Create a pthread with specified attributes
    ret = pthread_create(task_handle, task_attr, fun, NULL);
    if (ret) {
        printf("Create pthread failed! ");
        printf("Make sure to run with sudo, since only root can create SCHED_FIFO threads.\n");
        pthread_attr_destroy(task_attr);
        return -6;
    }
    pthread_attr_destroy(task_attr);
    return 0;
}
#endif

void read_encoder_last_position(const std::string& filename){
    size_t total_bytes_read = 0;
    size_t expected_total_bytes = 0;
    size_t type_size = sizeof(float);

    //Initialize offset array
    rt_config.encoder_last_position.resize(robot_state.ArmSize);
    for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
        rt_config.encoder_last_position[arm_i].resize(robot_state.JointSize[arm_i], 0.0f);
        expected_total_bytes += robot_state.JointSize[arm_i] * type_size;
    }

    //Read offset from file
    std::ifstream in_file(filename, std::ios::binary | std::ios::in);
    if (in_file){
        for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i) {
            in_file.read(reinterpret_cast<char*>(rt_config.encoder_last_position[arm_i].data()), robot_state.JointSize[arm_i] * type_size);
            total_bytes_read += in_file.gcount();
        }

        //Set offset to 0 if the file content is not complete
        if (total_bytes_read != expected_total_bytes){
            std::cerr << "Warning: Encoder offset file is incomplete. Expected " << expected_total_bytes << " bytes, but read " << total_bytes_read << " bytes. All offsets will be set to 0." << std::endl;
            for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
                std::fill(rt_config.encoder_last_position[arm_i].begin(), rt_config.encoder_last_position[arm_i].end(), 0.0f);
            }
        }
        in_file.close();
    }
}

void open_encoder_file_for_writing(const std::string& filename){
    std::filesystem::path p(filename);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }
    encoder_last_position_path = filename;
}

void arm_update_encode_zero_offset(int arm_i){
    for (int joint_i=0; joint_i < arm_ptr_vec[arm_i]->getJointSize(); joint_i++){
        if (arm_ptr_vec[arm_i]->getPosition(joint_i, &robot_state.MotorPos[arm_i][joint_i]) != RetCode::kSuccess){
            std::cout <<"Failed to read initial position for arm " << arm_i << " joint " << joint_i << std::endl;
            if (planner_state_ptr) {
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultHardwareInitFailed;
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |=
                    DiagnosticFlags::kFaultHardwareInitFailed;
            }
            shutdown_all_threads = true;
            return;
        }
    }
    
    //Update encoder zero offset
    float position_tolerance_rad = 0.3f; //If the initial position is larger than this threshold, we consider the encoder zero offset is not valid and will not update it
    float lap_count_boundary = 2*PI - position_tolerance_rad; // If more than this value, consider pass 1 circle
    for (int joint_i=0; joint_i < arm_ptr_vec[arm_i]->getJointSize(); joint_i++){
        float pos_err = robot_state.MotorPos[arm_i][joint_i] - rt_config.encoder_last_position[arm_i][joint_i];
        int laps = int(pos_err / lap_count_boundary);
        float remainder = abs(pos_err - laps*2*PI);
        if (remainder > position_tolerance_rad){
            std::cerr << "[Safety] Arm: "<< arm_i <<" Joint: "<<joint_i << " Startup Position Mismatch ignored: "
            << "Saved position: " << rt_config.encoder_last_position[arm_i][joint_i] <<", Current pos:" << robot_state.MotorPos[arm_i][joint_i]
            << ", Laps: "<< laps <<", Remainder: " <<remainder
            << "\nMismatch is kept at 0 and does not stop startup." << std::endl;
            continue;
        }else if (laps == 0){ //Normal
            continue;
        }else{
            arm_ptr_vec[arm_i]->updateEncoderZeroOffset(joint_i, 2*PI*laps);
        }
    }
}

void read_config(const YAML::Node& config)
{   
    // Initalize robot_state's motor
    robot_state.ArmSize = config["robot"]["arm"].size();
    if (robot_state.ArmSize <= 0 || robot_state.ArmSize > MAX_ARM_SIZE) {
        throw std::runtime_error("Configured arm count is outside protocol limits.");
    }
    rt_config.arm_name.resize(robot_state.ArmSize);
    rt_config.startup_position_mismatch_joint.resize(robot_state.ArmSize);
    for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
        std::string arm_name = config["robot"]["arm"][arm_i]["name"].as<std::string>();
        std::string arm_type = config["robot"]["arm"][arm_i]["type"].as<std::string>();
        std::string robot_name = arm_name + "-" + arm_type;
        if (robot_name.size() >= MAX_ROBOTNAME_SIZE) {
            throw std::runtime_error("Robot name exceeds protocol limit: " + robot_name);
        }
        rt_config.arm_name[arm_i] = robot_name;
        robot_state.JointSize[arm_i] = config["robot"]["arm"][arm_i]["joint_size"].as<int>();
        if (robot_state.JointSize[arm_i] <= 0 ||
            robot_state.JointSize[arm_i] > MAX_JOINT_SIZE) {
            throw std::runtime_error("Configured joint count is outside protocol limits.");
        }
        rt_config.startup_position_mismatch_joint[arm_i].assign(
            robot_state.JointSize[arm_i], false);
        for (int j = 0; j < robot_state.JointSize[arm_i]; ++ j) {
            robot_state.MotorPos[arm_i][j]   = 0.0f;
            robot_state.MotorVel[arm_i][j]   = 0.0f;
            robot_state.MotorTor[arm_i][j]   = 0.0f;
        }
    }

    // Initalize robot_state's gripper
    if (config["robot"]["gripper"]){
        robot_state.GripperSize = config["robot"]["gripper"].size();
        if (robot_state.GripperSize < 0 || robot_state.GripperSize > MAX_ARM_SIZE) {
            throw std::runtime_error("Configured gripper count is outside protocol limits.");
        }
        for (int gri_i=0; gri_i < robot_state.GripperSize; gri_i++){
            robot_state.GripperJointSize[gri_i] = config["robot"]["gripper"][gri_i]["joint_size"].as<int>();
            if (robot_state.GripperJointSize[gri_i] < 0 ||
                robot_state.GripperJointSize[gri_i] > MAX_JOINT_SIZE) {
                throw std::runtime_error("Configured gripper joint count is outside protocol limits.");
            }
            for (int gripper_joint_i=0; gripper_joint_i< robot_state.GripperJointSize[gri_i]; gripper_joint_i++){
                robot_state.GripperJointPos[gri_i][gripper_joint_i] = 0.0f;
            }
        }
    }
        
    rt_config.hardware_simulation = config["rt_control"]["hardware_simulation"].as<bool>();
    if (config["rt_control"]["command_watchdog_timeout_ms"]) {
        const long timeout_ms =
            config["rt_control"]["command_watchdog_timeout_ms"].as<long>();
        if (timeout_ms < 50 || timeout_ms > 5000) {
            throw std::runtime_error("command_watchdog_timeout_ms must be between 50 and 5000.");
        }
        rt_config.command_watchdog_timeout_us = timeout_ms * 1000;
    }

    //Load joint_hard_limit
    for (int i=0; i < 2; i++){
        int sign = i == 0 ? -1 : 1;
        rt_config.joint_position_limit[i].resize(robot_state.ArmSize);
        rt_config.joint_velocity_limit[i].resize(robot_state.ArmSize);
        rt_config.joint_torque_limit[i].resize(robot_state.ArmSize);
        for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
            rt_config.joint_position_limit[i][arm_i].resize(robot_state.JointSize[arm_i]);
            rt_config.joint_velocity_limit[i][arm_i].resize(robot_state.JointSize[arm_i]);
            rt_config.joint_torque_limit[i][arm_i].resize(robot_state.JointSize[arm_i]);

            for (int joint_i=0; joint_i < robot_state.JointSize[arm_i]; joint_i++){
                const YAML::Node& safety_config = config["robot"]["arm"][arm_i]["safety"];
                rt_config.joint_position_limit[i][arm_i][joint_i]     = safety_config["joint_hard_limit"]["position"][i][joint_i].as<float>();
                rt_config.joint_velocity_limit[i][arm_i][joint_i]     = safety_config["joint_hard_limit"]["velocity"][joint_i].as<float>() * sign;
                rt_config.joint_torque_limit[i][arm_i][joint_i]       = safety_config["joint_hard_limit"]["torque"][joint_i].as<float>() * sign;
            }
        }
    }
    
    //Load joint_follow
    rt_config.joint_position_follow_threshold.resize(robot_state.ArmSize);
    rt_config.joint_velocity_follow_threshold.resize(robot_state.ArmSize);
    rt_config.joint_torque_follow_threshold.resize(robot_state.ArmSize);
    for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
        const YAML::Node& safety_config = config["robot"]["arm"][arm_i]["safety"];
        rt_config.joint_position_follow_threshold[arm_i] = safety_config["joint_follow"]["position"].as<std::vector<float>>();
        rt_config.joint_velocity_follow_threshold[arm_i] = safety_config["joint_follow"]["velocity"].as<std::vector<float>>();
        rt_config.joint_torque_follow_threshold[arm_i]   = safety_config["joint_follow"]["torque"].as<std::vector<float>>();
        const std::size_t expected = static_cast<std::size_t>(robot_state.JointSize[arm_i]);
        if (rt_config.joint_position_follow_threshold[arm_i].size() != expected ||
            rt_config.joint_velocity_follow_threshold[arm_i].size() != expected ||
            rt_config.joint_torque_follow_threshold[arm_i].size() != expected) {
            throw std::runtime_error("Joint-follow threshold dimensions do not match joint_size.");
        }
    }

    rt_config.last_planner_target.resize(robot_state.ArmSize);
    rt_config.can_consecutive_exchange_failures.assign(robot_state.ArmSize, 0);
    for (int arm_i =0; arm_i < robot_state.ArmSize; ++ arm_i){
        rt_config.last_planner_target[arm_i].assign(robot_state.JointSize[arm_i], 0.0);
    }
}

void initialize_arm(const YAML::Node& config, std::string project_base_path){
    if (skip_arm_test_mode) {
        std::cout << "CUARM_RT_CONTROL_SKIP_ARM enabled: skipping arm hardware initialization." << std::endl;
        return;
    }
    if (config["robot"]["arm"] && !rt_config.hardware_simulation){
        int arm_size = config["robot"]["arm"].size();
        for (int arm_i=0; arm_i<arm_size; arm_i++){
            std::string arm_type = config["robot"]["arm"][arm_i]["type"].as<std::string>();
            std::string arm_name = config["robot"]["arm"][arm_i]["name"].as<std::string>();
            
            //Generate Arm object
            if (arm_type == "DualV1Arm"){
            #ifdef DUAL_V1
                arm_ptr_vec.push_back(std::make_unique<DualV1Arm>(arm_name, arm_i));
            #else
                throw std::invalid_argument("DualV1Arm is not compiled");
            #endif
            }else if (arm_type == "DualV2_1Arm"){
            #ifdef DUAL_V2_1
                arm_ptr_vec.push_back(std::make_unique<DualV2_1Arm>(arm_name, arm_i));
            #else
                throw std::invalid_argument("DualV2_1Arm is not compiled");
            #endif
            }else if (arm_type == "DualV2_2Arm"){
            #ifdef DUAL_V2_2
                arm_ptr_vec.push_back(std::make_unique<DualV2_2Arm>(
                    arm_name, arm_i, [] {
                        return shutdown_all_threads.load(std::memory_order_relaxed);
                    }));
            #else
                throw std::invalid_argument("DualV2_2Arm is not compiled");
            #endif
            }else if (arm_type == "Spark2_V1"){
            #ifdef SPARK2_V1
                arm_ptr_vec.push_back(std::make_unique<ArmV1>(arm_name));
            #else
                throw std::invalid_argument("ArmV1 is not compiled");
            #endif
            }else if (arm_type == "Spark2_V2"){
            #ifdef SPARK2_V2
                arm_ptr_vec.push_back(std::make_unique<Spark2>(arm_name));
            #else
                throw std::invalid_argument("Spark2 is not compiled");
            #endif
            }else{
                throw std::invalid_argument("Unknown arm type " + arm_type);
            }

            //Open arm
            RetCode success = arm_ptr_vec[arm_i]->open();
            if (shutdown_all_threads) {
                std::cout << "[Shutdown] Arm initialization cancelled by signal."
                          << std::endl;
                return;
            }
            if (success != RetCode::kSuccess){
                std::cout << "Failed to open arm "<< arm_i <<", SHUTTING DOWN!\n";
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultArmNotFound;
                shutdown_all_threads = true;
                return;
            }
            arm_update_encode_zero_offset(arm_i);
            if (shutdown_all_threads) return;
            
            //Read initial position for all joints
            for (int joint_i=0; joint_i < arm_ptr_vec[arm_i]->getJointSize(); joint_i++){
                if (arm_ptr_vec[arm_i]->getPosition(joint_i, &robot_state.MotorPos[arm_i][joint_i]) != RetCode::kSuccess){
                    std::cout <<"Failed to read initial position for arm " << arm_i << " joint " << joint_i  <<", SHUTTING DOWN!\n";
                    planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] |= DiagnosticFlags::kFaultHardwareInitFailed;
                    shutdown_all_threads = true;
                }
                rt_config.last_planner_target[arm_i][joint_i] = robot_state.MotorPos[arm_i][joint_i];
            }

            //Update motor enable state
            std::vector<bool> enable = config["robot"]["arm"][arm_i]["enable_joint"].as<std::vector<bool>>();
            if (enable.size() != static_cast<std::size_t>(arm_ptr_vec[arm_i]->getJointSize()) ||
                arm_ptr_vec[arm_i]->enableMotors(enable) != RetCode::kSuccess) {
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultHardwareInitFailed;
                shutdown_all_threads = true;
                return;
            }
            for (int joint_i=0; joint_i<arm_ptr_vec[arm_i]->getJointSize(); joint_i++){
                planner_state_ptr->enabled_joint[arm_i][joint_i] = enable[joint_i];
            }
        }
    }
}

void initialize_gripper(const YAML::Node& config, std::string project_base_path){
    if (config["robot"]["gripper"] && !rt_config.hardware_simulation){
        printf("Setup gripper.\n");
        int gripper_size = config["robot"]["gripper"].size();
        std::unordered_map<std::string, int> opened_serial_com;
        for (int gri_i=0; gri_i<gripper_size; gri_i++){
            std::string gripper_name = config["robot"]["gripper"][gri_i]["name"].as<std::string>();
            std::string gripper_type = config["robot"]["gripper"][gri_i]["type"].as<std::string>();

            if (left_hand_only_test_mode && gri_i != kVlmPanelHandIndex) {
                std::cout << "CUARM_RT_CONTROL_LEFT_HAND_ONLY enabled: skipping gripper " << gri_i << "." << std::endl;
                gripper_ptr_vec.push_back(nullptr);
                continue;
            }

            // Generate gripper object
            if (gripper_type == "UartServoTwoFingerGripper"){
                int id = config["robot"]["gripper"][gri_i]["hardware"]["servo_id"].as<int>();
                std::string com_id = config["robot"]["gripper"][gri_i]["hardware"]["com_id"].as<std::string>();
                if (!opened_serial_com.count(com_id)){ //First time open this com port
                    gripper_ptr_vec.push_back(std::make_unique<UartServoTwoFingerGripper>(gripper_name, com_id, id));
                    opened_serial_com[com_id] = gripper_ptr_vec.size() - 1;
                }else{ //Re-use opened com port
                    int i = opened_serial_com[com_id];
                    UartServoTwoFingerGripper* existing_gripper = dynamic_cast<UartServoTwoFingerGripper*>(gripper_ptr_vec[i].get());
                    gripper_ptr_vec.push_back(std::make_unique<UartServoTwoFingerGripper>(gripper_name, *existing_gripper, id));
                }

                //Update hardware limit
                std::vector<float> lower_limit = config["robot"]["gripper"][gri_i]["hardware"]["servo_limit"][0].as<std::vector<float>>();
                std::vector<float> upper_limit = config["robot"]["gripper"][gri_i]["hardware"]["servo_limit"][1].as<std::vector<float>>();
                gripper_ptr_vec[gri_i]->setJointHardwareLimit(std::move(lower_limit), std::move(upper_limit));
            }else if (gripper_type == "InspireDexterousHand"){
            #ifdef DUAL_V1
                gripper_ptr_vec.push_back(std::make_unique<InspireHandWrapper>(gripper_name, gri_i));
            #elif defined(DUAL_V2_2)
                if (gri_i != kVlmPanelHandIndex){
                    throw std::invalid_argument("InspireDexterousHand must be PanelCommand gripper index 0 for dual_v2_2");
                }
#ifdef CUARM_RT_CONTROL_IGNORE_LEFT_HAND
                std::cout << "dual_v2_2_control_2: left Inspire Hand is ignored."
                          << std::endl;
#elif defined(CUARM_RT_CONTROL_NO_HAND)
                std::cout << "CUARM_RT_CONTROL_NO_HAND: left Inspire Hand ignored; "
                             "panel command is written through." << std::endl;
#else
                if (initializeVlmLeftHand() != RetCode::kSuccess){
                    planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultGripperNotFound;
                    shutdown_all_threads = true;
                }
#endif
                gripper_ptr_vec.push_back(nullptr);
            #else
                throw std::invalid_argument("InspireDexterousHand is not compiled");
            #endif
            }else{
                throw std::invalid_argument("Unknown gripper type " + gripper_type);
            }

            // //Set joint limit
            // if (config["robot"]["gripper"][gri_i]["safety"] && config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]){
            //     std::vector<float> lower_limit;
            //     std::vector<float> upper_limit;
            //     for (int j=0; j< config["robot"]["gripper"][gri_i]["joint_size"].as<int>(); j++){
            //         lower_limit.push_back(config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][0][j].as<float>());
            //         upper_limit.push_back(config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][1][j].as<float>());
            //     }
            //     gripper_ptr_vec[gri_i]->setJointLimit(lower_limit, upper_limit);
            // }

            if (!gripper_ptr_vec[gri_i]){
                continue;
            }

            // Open gripper
            if (gripper_ptr_vec[gri_i]->open() != RetCode::kSuccess){
                std::cout << "Failed to open gripper" + std::to_string(gri_i) <<", SHUTTING DOWN!\n";
                planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultGripperNotFound;
                shutdown_all_threads = true;
            }
            
            //Set gripper to open position
            std::vector<float> cmd(gripper_ptr_vec[gri_i]->getJointSize(), 1.0f);
            gripper_ptr_vec[gri_i]->setNormalizedPosition(cmd);
        }
    }
}

void initialize_log_file(const std::string& directory_path) {
    namespace fs = std::filesystem;
    if (!fs::exists(directory_path)){
        throw std::runtime_error("Directory " + directory_path + " does not exist.");
    };

    std::string write_path = directory_path + "/33.txt";
    fwrite_data = fopen(write_path.c_str(), "wb");
    if (!fwrite_data) {
        throw std::runtime_error("fwrite_data create failed!");
    }
}

int main(int argc, char *argv[])
{   
    std::signal(SIGINT, request_shutdown_from_signal);
    std::signal(SIGTERM, request_shutdown_from_signal);
    int exit_code = 0;

    #if KERNEL_XENOMAI | KERNEL_PREEMPT_RT
        // use isolate cpu to do the rt thread
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(3, &mask); // add CPU0 to cpu set
        CPU_SET(4, &mask); // add CPU0 to cpu set
        // Set the CPU affinity for a pid 
        if (sched_setaffinity(getpid(), sizeof(cpu_set_t), &mask) == -1) {
            perror("sched_setaffinity");
        }

        // if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &mask) == -1) {
        //     perror("pthread_setaffinity_np");
        // }
        // std::cout << "control_task Thread ID (TID): " << syscall(SYS_gettid) << std::endl;
        // std::cout << "pid: " << getpid() << std::endl;
    #if KERNEL_XENOMAI
        rt_sem_p(&mysync,TM_INFINITE);
        rt_task_set_periodic(NULL, TM_INFINITE, control_loop_frequency * 1000);
    #endif
    #endif
    
    // std::cout << "main Thread ID (TID): " << syscall(SYS_gettid) << std::endl;
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::string config_path = config_prefix_path+"/config.yaml";
    std::string encoder_last_position_filename = config_prefix_path + "/rt_control/encoder_last_position.bin";

    try{
        // Read config
        YAML::Node config = loadYamlConfig(config_path);
        read_config(config);
        if (envFlagEnabled("CUARM_RT_CONTROL_SIMULATION")) {
            rt_config.hardware_simulation = true;
            std::cout << "CUARM_RT_CONTROL_SIMULATION enabled: all hardware I/O is disabled."
                      << std::endl;
        }
        rt_config.allow_startup_position_mismatch =
            envFlagEnabled("CUARM_ALLOW_STARTUP_POSITION_MISMATCH");
        if (rt_config.allow_startup_position_mismatch) {
            std::cerr
                << "[Safety Override] CUARM_ALLOW_STARTUP_POSITION_MISMATCH is "
                   "enabled. Position mismatches will be warnings only."
                << std::endl;
        }
        read_encoder_last_position(encoder_last_position_filename);

        //Connect to upper software
        std::string local_ip = config["rt_control"]["ip"].as<std::string>();
        std::string remote_ip = config["panel"]["ip"].as<std::string>();
        int local_port = config["rt_control"]["port"].as<int>();
        int remote_port = config["panel"]["port"].as<int>();
        const int port_offset = envInteger("CUARM_UDP_PORT_OFFSET", 0);
        local_port += port_offset;
        remote_port += port_offset;
        if (local_port <= 0 || local_port > 65535 ||
            remote_port <= 0 || remote_port > 65535) {
            throw std::runtime_error("CUARM_UDP_PORT_OFFSET produces an invalid UDP port.");
        }
        int buffer_size = 60000;
        udp_ptr = std::make_unique<CuarmUdp<PanelCommand, PlannerState>>(
            local_ip, local_port , remote_ip, remote_port, &CuarmMessageHandler::unpack_panel_command,
            &CuarmMessageHandler::pack_planner_state, buffer_size);

        // Initialize planner
        planner_ptr = std::make_unique<RobotPlanner>(config, config_prefix_path);
        planner_ptr->getPlannerCommandAndPlannerState(planner_command_ptr, planner_state_ptr);
        planner_state_ptr->system_state = SystemState::kStartup;
        planner_state_ptr->plan_result = PlanResult::kSuccess;
        planner_state_ptr->system_diagnostic_flags = DiagnosticFlags::kNone;
        for (int arm_i=0; arm_i<planner_state_ptr->ArmSize; arm_i++){
            for (int joint_i=0; joint_i<planner_state_ptr->JointSize[arm_i]; joint_i++){
                planner_state_ptr->arm_joint_diagnostic_flags[arm_i][joint_i] = DiagnosticFlags::kNone;
            }
        }

        //Init logger
        std::string project_base_path = PROJECT_SOURCE_DIR;
        initialize_log_file(project_base_path+"/build/");
        
        //Update delay time
        float dt_send = config["rt_control"]["dt_send"].as<float>();
        float dt_receive = config["rt_control"]["dt_receive"].as<float>();
        if (!std::isfinite(dt_send) || !std::isfinite(dt_receive) ||
            dt_send <= 0.0f || dt_receive <= 0.0f) {
            throw std::runtime_error("RT send/receive periods must be finite and positive.");
        }
        control_loop_delay_us = int(dt_send*1000000);
        udp_receive_delay_us = int(dt_receive*1000000);
        write_data_delay_us = control_loop_delay_us*write_freq;

        skip_arm_test_mode = envFlagEnabled("CUARM_RT_CONTROL_SKIP_ARM") || envFlagEnabled("CUARM_RT_CONTROL_LEFT_HAND_ONLY");
        left_hand_only_test_mode = envFlagEnabled("CUARM_RT_CONTROL_LEFT_HAND_ONLY");
        if (skip_arm_test_mode) {
            std::cout << "RT control hand/gripper test mode: arm hardware is skipped." << std::endl;
        }
        if (left_hand_only_test_mode) {
            std::cout << "RT control left-hand-only test mode: non-left grippers are skipped." << std::endl;
        }

        //Init robot
        initialize_arm(config, project_base_path);

        //Init gripper
        if (!shutdown_all_threads) {
            initialize_gripper(config, project_base_path);
        }

    }catch (const std::exception& e){
        std::cerr << "Catch exception: " << e.what() << std::endl;
        if (planner_state_ptr) {
            planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultUnknown;
        }
        shutdown_all_threads = true;
        exit_code = 1;
    }
    
    // Need to ensure hardware is initialized successfully before open encoder file for writing, 
    // otherwise, the file would be replaced to default values.
    if (!rt_config.hardware_simulation && !shutdown_all_threads && !skip_arm_test_mode){
        open_encoder_file_for_writing(encoder_last_position_filename);
        save_encoder_last_position(0, nullptr);
    }

    if (!shutdown_all_threads){
        //Start threading
    #if (KERNEL_XENOMAI | KERNEL_PREEMPT_RT)
        if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            printf("mlockall failed: %m\n");
            mark_system_fault(DiagnosticFlags::kFaultHardwareInitFailed,
                              "Unable to lock process memory for real-time control.");
            exit_code = 2;
        }
        // Turn off malloc trimming.
        mallopt(M_TRIM_THRESHOLD, -1);
        // Turn off mmap usage. 
        mallopt(M_MMAP_MAX, 0);
    #endif

    #if KERNEL_XENOMAI
        printf("KERNEL_XENOMAI\n");
        // create thread for rt control
        if (rt_task_create(&task_handle_control, "Control", 1024000, 90, 0)) {
            rt_task_delete(&task_handle_control);
            printf("Error: Cannot create control thread.\n");
            return -3;
        }
        rt_task_start(&task_handle_control, &control_task, NULL);

        // create thread for data writing
        if (rt_task_create(&task_handle_data, "Data", 1024000, 90, 0)) {
            rt_task_delete(&task_handle_data);
            printf("Error: Cannot create data thread.\n");
            return -3;
        }
        rt_task_start(&task_handle_data, &write_data_task, NULL);

        // create thread for udp
        if (rt_task_create(&task_handle_udp, "UDP", 1024000, 90, 0)) {
            rt_task_delete(&task_handle_udp);
            printf("Error: Cannot create udp thread.\n");
            return -3;
        }
        rt_task_start(&task_handle_udp, &receive_udp_task, NULL);

        // Run until a signal, safety fault, or peer shutdown requests exit.
        while (!shutdown_all_threads)
            sched_yield();
        rt_task_delete(&task_handle_control);
        rt_task_delete(&task_handle_data);
        rt_task_delete(&task_handle_udp);
    #elif KERNEL_PREEMPT_RT
        printf("KERNEL_PREEMPT_RT\n");
        bool control_started = false;
        bool data_started = false;
        bool udp_started = false;

        if (!shutdown_all_threads &&
            creat_rt_thread(&task_handle_control, &task_attr_control, 80, control_task) == 0) {
            control_started = true;
        } else if (!shutdown_all_threads) {
            mark_system_fault(DiagnosticFlags::kFaultHardwareInitFailed,
                              "Cannot create real-time control thread.");
            exit_code = 4;
        }

        if (!shutdown_all_threads &&
            creat_rt_thread(&task_handle_data, &task_attr_data, 70, write_data_task) == 0) {
            data_started = true;
        } else if (!shutdown_all_threads) {
            mark_system_fault(DiagnosticFlags::kFaultHardwareInitFailed,
                              "Cannot create data writer thread.");
            exit_code = 4;
        }

        if (!shutdown_all_threads &&
            creat_rt_thread(&task_handle_udp, &task_attr_udp, 80, receive_udp_task) == 0) {
            udp_started = true;
        } else if (!shutdown_all_threads) {
            mark_system_fault(DiagnosticFlags::kFaultHardwareInitFailed,
                              "Cannot create UDP receiver thread.");
            exit_code = 4;
        }

        while (!shutdown_all_threads) sched_yield();
        if (control_started && pthread_join(task_handle_control, NULL))
            printf("join control_task pthread failed: %m\n");
        if (data_started && pthread_join(task_handle_data, NULL))
            printf("join task_handle_data pthread failed: %m\n");
        if (udp_started && pthread_join(task_handle_udp, NULL))
            printf("join task_handle_udp pthread failed: %m\n");
    #else
        printf("LINUX\n");
        std::thread control_task_thread(control_task, nullptr);
        std::thread receive_udp_task_thread(receive_udp_task, nullptr);
        std::thread write_data_task_thread(write_data_task, nullptr);
        control_task_thread.join();
        receive_udp_task_thread.join();
        write_data_task_thread.join();

        // osal_thread_create(&task_handle_control, 128000, (void*) &control_task);
        // osal_thread_join(task_handle_control);
        // osal_thread_create(&task_handle_data, 128000, (void*) &write_data_task);
        // osal_thread_join(task_handle_data);
        // osal_thread_create(&task_handle_udp, 128000, (void*) &receive_udp_task);
        // osal_thread_join(task_handle_udp);
    #endif   
    }

    if (planner_state_ptr && planner_state_ptr->system_state == SystemState::kError) {
        exit_code = std::max(exit_code, 1);
    }

    disable_all_arms();
    if (udp_ptr) udp_ptr->close();
    if (!rt_config.hardware_simulation){
        for (int arm_i=0; arm_i < arm_ptr_vec.size(); arm_i++){
            arm_ptr_vec[arm_i]->close();
        }
        for (int i = 0; i < gripper_ptr_vec.size(); i++){
            if (gripper_ptr_vec[i]) {
                gripper_ptr_vec[i]->close();
            }
        }
    }

    // Destroy polymorphic runtime objects explicitly while all dependencies
    // are still alive. Leaving them to process-wide static destruction made
    // shutdown order undefined and could end in "__cxa_pure_virtual".
    planner_state_ptr = nullptr;
    planner_command_ptr = nullptr;
    planner_ptr.reset();
    gripper_ptr_vec.clear();
    arm_ptr_vec.clear();
#ifdef DUAL_V2_2
    g_vlm_left_hand_available = false;
    g_vlm_left_hand.reset();
    g_vlm_hand_transport.reset();
#endif
    panel_command_ptr.reset();
    transmit_panel_command_ptr.reset();
    udp_ptr.reset();

    if (fwrite_data) {
        fclose(fwrite_data);
        fwrite_data = nullptr;
    }
    printf("Terminated all threads, exit program.\n");
    return exit_code;
}
