#include "robot_arm.h"
#include "can_normal.h"
#include "can_fd.h"
#include <cstdlib>
#include <utility>

RobotArms::RobotArms(int arm_index, const Robot& robot,
                     StopRequested stop_requested)
    : arm_index_(arm_index),
      can_channel_(robot.motors.can_channel),
      sensor_channel_(robot.torque_sensors.can_channel),
      robot_config_(robot),
      stop_requested_(std::move(stop_requested)) {
    for (int i = 0; i < num_joints_; ++i) {
        arm_motors_[i] = WhjMotor(robot.motors.ids[i], can_channel_,
                                  robot.motors.gear_ratios[i], robot.mdh.sign[i],
                                  robot.mdh.m_init_mdhpos[i]);
        arm_torque_sensor_[i] = TorqueSensor(robot.torque_sensors.ids[i], sensor_channel_,
                                             EU_CANABLE, robot.torque_sensors.sign[i]);
        motor_enable_list_[i] = robot.motors.enabled[i];
        sensor_enable_list_[i] = robot.torque_sensors.enabled[i];
        motor_map_[arm_motors_[i].motor_id_] = &arm_motors_[i];
        sensor_map_[arm_torque_sensor_[i].sensor_id_] = &arm_torque_sensor_[i];
    }
}

RobotArms::~RobotArms() {
    // Cleanup resources
}

bool RobotArms::stopRequested() const {
    return stop_requested_ && stop_requested_();
}

int RobotArms::poll_responses(int timeout_ms, int expected_motor_index, bool expect_write_ack) {
    auto start_time = std::chrono::steady_clock::now();
    bool expected_response_received = false;

    while (!expected_response_received) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) {
            break;
        }

        uint32_t respId = 0;
        uint8_t respMaxData[64];
        uint8_t respDlc = 0;
        uint8_t receiveStatus = canfd_receive(can_channel_, 0, &respId, respMaxData, &respDlc, 1);
        if (receiveStatus < 0) {
            continue;
        }

        auto it = motor_map_.find(respId & 0xFF);
        if (it != motor_map_.end()) {
            WhjMotor* motor = it->second;
            if (respId == motor->motor_id_ + 0x500) {
                motor->control_success_ = 1;
                motor->decode_motor_data(respMaxData);
            } else if (respId == motor->motor_id_ + 0x100) {
                motor->write_success_ = 1;
            } else if (respId == motor->motor_id_ + 0x700) {
                motor->control_success_ = 1;
                motor->decode_motor_data2(respMaxData);
                // std::cout << "Received status response from Motor ID: " << motor->motor_id_ 
                //           << ", Pos: " << motor->pos_ << " rad, Vel: " << motor->vel_ << " rad/s, Cur: " << motor->cur_ << " A" 
                //           << ", Enable Status: " << motor->enable_status_ << ", Error Code: " << motor->error_code_ 
                //           << ", Brake Status: " << motor->brake_status_ << std::endl;
            }

            if (expected_motor_index < 0) {
                expected_response_received = true;
            } else if (expected_motor_index < num_joints_ &&
                       motor == &arm_motors_[expected_motor_index]) {
                expected_response_received = expect_write_ack
                    ? motor->write_success_ == 1
                    : motor->control_success_ == 1;
            }
        }
    }

    if (!expected_response_received) {
        // std::cerr << "Motor: No responses received in " << timeout_ms << " ms (timeout)" << std::endl;
        return -1;
    }
    return 0;
}

double RobotArms::mdh_to_joints(int index, const double mdh_rad) {
    double joints_rad = arm_motors_[index].sign_ * (mdh_rad + arm_motors_[index].init_mdh_rad_);
    return joints_rad;
}

double RobotArms::joints_to_mdh(int index, const double joints_rad) {
    double mdh_rad = arm_motors_[index].sign_ * joints_rad - arm_motors_[index].init_mdh_rad_;
    return mdh_rad;
}

int RobotArms::initialize() {
    int max_retry_times = 500;
    const char* recover_env = std::getenv("CUARM_RECOVER_STARTUP_FAULT");
    const bool recover_startup_faults =
        recover_env != nullptr && std::string(recover_env) == "1";

    if (recover_startup_faults) {
        std::cout
            << "[Recovery] Startup fault recovery enabled. Only fault 0x20 "
               "will be cleared, once per joint."
            << std::endl;
    }

    for (int joint = 0; joint < num_joints_; ++joint) {
        if (stopRequested()) {
            std::cerr << "[Shutdown] Arm " << arm_index_
                      << " initialization cancelled before motor "
                      << arm_motors_[joint].motor_id_ << "." << std::endl;
            return -3;
        }
        if (motor_enable_list_[joint] == 0)
            continue;

        int retry_count = 0;
        bool success = false;
        std::cout << "Initializing motor ID: "
                  << arm_motors_[joint].motor_id_ << " ......"
                  << std::endl;
        while (!success && retry_count < max_retry_times) {
            if (stopRequested()) {
                std::cerr << "[Shutdown] Arm " << arm_index_
                          << " initialization cancelled while waiting for motor "
                          << arm_motors_[joint].motor_id_ << "." << std::endl;
                return -3;
            }
            arm_motors_[joint].write_success_ = 0;
            if (arm_motors_[joint].set_IAP() == 0) {
                poll_responses(10, joint, true);
                success = arm_motors_[joint].write_success_ == 1;
            } else {
                std::ostringstream oss;
                oss << "Arm " << arm_index_ << " Motor ID: "
                    << arm_motors_[joint].motor_id_
                    << " sending IAP cmd failed, check wire connection"
                    << retry_count;
                std::cerr << oss.str() << std::endl;
                LOG_ERROR(oss.str());
                return -1;
            }

            retry_count++;
            if (!success && retry_count >= max_retry_times) {
                std::ostringstream oss;
                oss << "Arm " << arm_index_ << " Failed to set IAP "
                    << max_retry_times << " times, motor ID: "
                    << arm_motors_[joint].motor_id_;
                std::cerr << oss.str() << std::endl;
                LOG_ERROR(oss.str());
                return -1;
            }
            if (!success) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        if (stopRequested()) {
            return -3;
        }
        if (recover_startup_faults && recover_startup_fault(joint) != 0) {
            return -1;
        }
        if (enable_motor(joint, true) != 0) {
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (stopRequested()) {
        return -3;
    }
    if (fix_motor_initial_position() != 0) {
        return -1;
    }
    return 0;
}

int RobotArms::recover_startup_fault(int index) {
    if (index < 0 || index >= num_joints_) {
        std::cerr << "[Recovery] Invalid joint index: " << index << std::endl;
        return -1;
    }
    if (stopRequested()) {
        return -3;
    }

    // Keep the joint disabled throughout fault inspection and clearing.
    if (enable_motor(index, false) != 0) {
        std::cerr << "[Recovery] Arm " << arm_index_ << " motor "
                  << arm_motors_[index].motor_id_
                  << " could not be disabled before fault recovery."
                  << std::endl;
        return -1;
    }
    if (stopRequested()) {
        return -3;
    }

    arm_motors_[index].control_success_ = 0;
    if (arm_motors_[index].read_status() != 0 ||
        poll_responses(20, index, false) != 0) {
        std::cerr << "[Recovery] Arm " << arm_index_ << " motor "
                  << arm_motors_[index].motor_id_
                  << " status read failed before fault recovery."
                  << std::endl;
        return -1;
    }

    const int error_code = arm_motors_[index].error_code_;
    if (error_code == 0) {
        return 0;
    }
    if (error_code != 0x20) {
        std::cerr << "[Recovery] Arm " << arm_index_ << " motor "
                  << arm_motors_[index].motor_id_ << " has fault code 0x"
                  << std::hex << error_code << std::dec
                  << "; automatic recovery is restricted to 0x20."
                  << std::endl;
        return -1;
    }

    arm_motors_[index].write_success_ = 0;
    if (arm_motors_[index].clear_error() != 0 ||
        poll_responses(20, index, true) != 0) {
        std::cerr << "[Recovery] Arm " << arm_index_ << " motor "
                  << arm_motors_[index].motor_id_
                  << " did not acknowledge the clear-error register write."
                  << std::endl;
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (stopRequested()) {
        return -3;
    }
    arm_motors_[index].control_success_ = 0;
    if (arm_motors_[index].read_status() != 0 ||
        poll_responses(20, index, false) != 0) {
        std::cerr << "[Recovery] Arm " << arm_index_ << " motor "
                  << arm_motors_[index].motor_id_
                  << " status read failed after fault recovery."
                  << std::endl;
        return -1;
    }

    if (arm_motors_[index].error_code_ != 0) {
        std::cerr << "[Recovery] Arm " << arm_index_ << " motor "
                  << arm_motors_[index].motor_id_
                  << " still reports fault code 0x" << std::hex
                  << arm_motors_[index].error_code_ << std::dec
                  << " after clear; joint remains disabled."
                  << std::endl;
        return -1;
    }

    std::cout << "[Recovery] Arm " << arm_index_ << " motor "
              << arm_motors_[index].motor_id_
              << " fault 0x20 cleared and verified while disabled."
              << std::endl;
    return 0;
}

int RobotArms::shut_down() {
    bool all_disabled = true;
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        arm_motors_[joint].write_success_ = 0;
        if (arm_motors_[joint].disable() != 0 ||
            poll_responses(10, joint, true) != 0) {
            all_disabled = false;
            std::cerr << "[Shutdown] Arm " << arm_index_ << " motor "
                      << arm_motors_[joint].motor_id_
                      << " did not acknowledge disable." << std::endl;
            continue;
        }
        motor_enable_list_[joint] = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return all_disabled ? 0 : -1;
}

int RobotArms::enable_motor(int index, bool enable){
    if (index < 0 || index >= num_joints_) {
        return -1;
    }
    int max_retry_times = 500;
    int retry_count = 0;
    bool success = false;
    std::string func_type_str= (enable)? " enable " : " disable ";
    int (WhjMotor::*funcPtr)(void) = (enable)? &WhjMotor::enable : &WhjMotor::disable;
    while (!success && retry_count < max_retry_times) {
        // On shutdown, never start/retry an enable. A disable still gets one
        // best-effort attempt so Ctrl+C does not leave the motor energized.
        if (stopRequested() && (enable || retry_count > 0)) {
            std::cerr << "[Shutdown] Arm " << arm_index_ << " motor "
                      << arm_motors_[index].motor_id_ << func_type_str
                      << "cancelled." << std::endl;
            return -3;
        }
        arm_motors_[index].write_success_ = 0;
        if ((arm_motors_[index].*funcPtr)() == 0) {
            poll_responses(10, index, true);
            if (arm_motors_[index].write_success_ == 1) {
                // std::ostringstream oss;
                // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[index].motor_id_ << func_type_str << "successfully.";
                // std::cout << oss.str() << std::endl;
                // LOG_INFO(oss.str());
                success = true;
                motor_enable_list_[index] = enable ? 1 : 0;
            } else {
                // std::ostringstream oss;
                // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[index].motor_id_ 
                //     << func_type_str << "failed, retrying .... " << retry_count;
                // LOG_DEBUG(oss.str());
            }
        } else {
            std::ostringstream oss;
            oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[index].motor_id_ 
                << func_type_str << "failed, check wire connection" << retry_count;
            std::cerr << oss.str() << std::endl;
            LOG_ERROR(oss.str());
            return -1;
        }
        
        retry_count++;
        if (!success && retry_count >= max_retry_times) {
            std::ostringstream oss;
            oss << "Arm " << arm_index_ << " Failed to" << func_type_str << max_retry_times 
                << " times, motor ID: " << arm_motors_[index].motor_id_;
            LOG_ERROR(oss.str());
            std::cerr << oss.str() << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

void RobotArms::update_encoder_zero_offset(int index, float offset){
    arm_motors_[index].init_mdh_rad_ += offset;
}

bool RobotArms::is_brake_opened(int index){
    return arm_motors_[index].brake_status_ == 0? true : false;
}

int RobotArms::fix_motor_initial_position(){
    int max_retry_times = 100;
    double zero_mdh_pos[num_joints_] = {0.0};
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (stopRequested()) {
            return -3;
        }
        if (motor_enable_list_[joint] == 0) 
            continue;
        int retry_count = 0;
        bool success = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        while (!success && retry_count < max_retry_times) {
            if (stopRequested()) {
                std::cerr << "[Shutdown] Arm " << arm_index_
                          << " initial-position read cancelled at motor "
                          << arm_motors_[joint].motor_id_ << "." << std::endl;
                return -3;
            }
            arm_motors_[joint].control_success_ = 0;
            if (arm_motors_[joint].read_status() == 0) {
                poll_responses(10, joint, false);
                if (arm_motors_[joint].control_success_ == 1) {
                    success = true;
                    zero_mdh_pos[joint] = get_mdh_position(joint);
                    // std::ostringstream oss;
                    // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[joint].motor_id_ << "Current mdh pos: "
                    //     << zero_mdh_pos[joint];
                    // LOG_INFO(oss.str());
                    // std::cout << oss.str() << std::endl;
                    if ((zero_mdh_pos[joint] - joints_to_mdh(joint, arm_motors_[joint].init_mdh_rad_)) >= PI) {
                        // std::ostringstream oss;
                        // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[joint].motor_id_ << " Lack One Circle";
                        // LOG_ERROR(oss.str());
                        // std::cerr << oss.str() << std::endl;
                        // arm_motors_[joint].init_mdh_rad_ = arm_motors_[joint].init_mdh_rad_ + PI*2 * arm_motors_[joint].sign_;
                        // throw std::runtime_error("RobotArms::fix_motor_initial_position: Lack One Circle. \n!!!!TODO: Enable init_mdh_rad_ modification");
                    } else if ((zero_mdh_pos[joint] - joints_to_mdh(joint, arm_motors_[joint].init_mdh_rad_)) <= -PI) {
                        // std::ostringstream oss;
                        // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[joint].motor_id_ << " More One Circle";
                        // LOG_ERROR(oss.str());
                        // std::cerr << oss.str() << std::endl;
                        // arm_motors_[joint].init_mdh_rad_ = arm_motors_[joint].init_mdh_rad_ - PI*2 * arm_motors_[joint].sign_;
                        // throw std::runtime_error("RobotArms::fix_motor_initial_position: More One Circle \n!!!!TODO: Enable init_mdh_rad_ modification");
                    }
                } else {
                    // std::ostringstream oss;
                    // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[joint].motor_id_ << " read status failed, retrying .... " << retry_count;
                    // LOG_DEBUG(oss.str());
                }
            } 
            retry_count++;
            if (!success && retry_count >= max_retry_times) {
                std::ostringstream oss;
                oss << "Arm " << arm_index_ << " Failed to read status " << max_retry_times 
                    << " times, motor ID: " << arm_motors_[joint].motor_id_;
                LOG_ERROR(oss.str());
                std::cerr << oss.str() << std::endl;
                return -2;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return 0;
}

int RobotArms::set_control_mode(CONTROL_MODE mode) {
    bool success = true;
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (stopRequested()) {
            return -3;
        }
        if (set_control_mode(joint, mode) != 0) success = false;
    }
    return success ? 0 : -1;
}

int RobotArms::set_control_mode(int index, CONTROL_MODE mode) {
    if (index < 0 || index >= num_joints_) {
        return -1;
    }
    if (stopRequested()) {
        return -3;
    }
    if (motor_enable_list_[index] == 0) 
        return 0;
    // std::ostringstream oss;
    // oss << "Arm " << arm_index_ << " motor id "<< arm_motors_[index].motor_id_ 
    //     << " set control mode to " << (int)mode << " ......";
    // LOG_INFO(oss.str());
    // std::cout << oss.str() << std::endl;
    int max_retry_times = 100;
    int retry_count = 0;
    bool success = false;
    while (!success && retry_count < max_retry_times) {
        if (stopRequested()) {
            std::cerr << "[Shutdown] Arm " << arm_index_
                      << " mode selection cancelled at motor "
                      << arm_motors_[index].motor_id_ << "." << std::endl;
            return -3;
        }
        arm_motors_[index].write_success_ = 0;
        if (arm_motors_[index].select_mode(mode) == 0) {
            poll_responses(10, index, true);
            if (arm_motors_[index].write_success_ == 1) {
                // std::ostringstream oss;
                // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[index].motor_id_ 
                //     << " select mode " << (int)mode << " successfully.";
                // // LOG_INFO(oss.str());
                // std::cout << oss.str() << std::endl;
                success = true;         
            } else {
                // std::ostringstream oss;
                // oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[index].motor_id_ 
                //     << " select mode failed, retrying .... " << retry_count;
                // LOG_DEBUG(oss.str());
            }
        } else {
            std::ostringstream oss;
            oss << "Arm " << arm_index_ << " Motor ID: " << arm_motors_[index].motor_id_ 
                << " sending select mode cmd failed, check wire connection" << retry_count;
            std::cerr << oss.str() << std::endl;
            LOG_ERROR(oss.str());
            return -1;
        }
        retry_count++;
        if (!success && retry_count >= max_retry_times) {
            std::ostringstream oss;
            oss << "Arm " << arm_index_ << " Failed to select mode " << max_retry_times 
                << " times, motor ID: " << arm_motors_[index].motor_id_;
            LOG_ERROR(oss.str());
            std::cerr << oss.str() << std::endl;
            return -2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

int RobotArms::set_mdh_position(int index, double mdh_pos) {
    double joint_pos = mdh_to_joints(index, mdh_pos);
    arm_motors_[index].control_success_ = 0;
    if (arm_motors_[index].set_absposition(joint_pos) !=0)
        return -1;
    return poll_responses(10, index, false) == 0 ? 0 : -2;
}

int RobotArms::set_velocity(int index, double vel_rad_s) {
    // if (std::abs(vel_rpm) > robot_config_.motors.max_velocity_rpm[index]) {
    //     std::cerr << "[Error] Arm " << arm_index_ << " Joint " << index 
    //               << " Velocity command too large: " << vel_rpm << " rpm" << std::endl;
    //     return -2;
    // }
    arm_motors_[index].control_success_ = 0;
    if (arm_motors_[index].set_velocity(vel_rad_s * arm_motors_[index].sign_) != 0)
        return -1;
    return poll_responses(10, index, false) == 0 ? 0 : -2;
}

int RobotArms::set_current(int index, double cur_amp) {
    // if (std::abs(cur_ma) > robot_config_.motors.max_current_mA[index]) {
    //     std::cerr << "[Error] Arm " << arm_index_ << " Joint " << index 
    //               << " Current command too large: " << cur_ma << " mA" << std::endl;
    //     return -2;
    // }
    arm_motors_[index].control_success_ = 0;
    if (arm_motors_[index].set_current(cur_amp * arm_motors_[index].sign_) != 0)
        return -1;
    return poll_responses(10, index, false) == 0 ? 0 : -2;
}

int RobotArms::read_status(int index) {   
    arm_motors_[index].control_success_ = 0;
    if (arm_motors_[index].read_status())
        return -1;
    if (poll_responses(10, index, false) != 0)
        return -2;
    if (arm_motors_[index].error_code_ != 0) {
        std::cerr << "Arm " << arm_index_ << " motor "
                  << arm_motors_[index].motor_id_ << " reported fault code 0x"
                  << std::hex << arm_motors_[index].error_code_ << std::dec
                  << std::endl;
        return -3;
    }
    return 0;
}

double RobotArms::get_mdh_position(int index) {
    double mdh_pos = joints_to_mdh(index, arm_motors_[index].pos_);
    return joints_to_mdh(index, arm_motors_[index].pos_);
}

double RobotArms::get_velocity(int index) {
    // if (std::abs(arm_motors_[index].vel_) > robot_config_.motors.max_velocity_rpm[index]) {
    //     std::cerr << "[Warning] Arm " << arm_index_ << " Joint " << index 
    //               << " Velocity out of limit: " << arm_motors_[index].vel_ << " deg/s" << std::endl;
    //     std::cerr << "Disable motor for safety" << std::endl;
    //     shut_down();
    // }
    return arm_motors_[index].vel_ * arm_motors_[index].sign_;
}

double RobotArms::get_current(int index) {
    // if (std::abs(arm_motors_[index].cur_) > robot_config_.motors.max_current_mA[index]) {
    //     std::cerr << "[Warning] Arm " << arm_index_ << " Joint " << index 
    //               << " Current out of limit: " << arm_motors_[index].cur_ << " mA" << std::endl;
    //     std::cerr << "Disable motor for safety" << std::endl;
    //     shut_down();
    // }
    return arm_motors_[index].cur_ * arm_motors_[index].sign_;
}

int RobotArms::move_joint(const double* mdh_rad, double dt, double T) {
    char filename[64];
    snprintf(filename, sizeof(filename), "joint_data_arm_%d.txt", arm_index_);
    FILE* fwrite_data = fopen(filename, "w");
    if (fwrite_data == nullptr) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return -1;
    }
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        if (mdh_rad[joint] < robot_config_.joint_limit.joint_negative_limit[joint] || 
            mdh_rad[joint] > robot_config_.joint_limit.joint_positive_limit[joint]) {
            std::cerr << "[Error] Arm " << arm_index_ << " Joint " << joint 
                      << " Target position out of limit: " << mdh_rad[joint] << " rad" << std::endl;
            fclose(fwrite_data);
            return -2;
        }
    }
    
    double start_pos[num_joints_];
    std::vector<double> traj_coef[num_joints_];
    double pid_output[num_joints_];
    double torque_sensor_data[num_joints_];
    int stop_flag = 0;
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        start_pos[joint] = get_mdh_position(joint);
        traj_coef[joint] = polynomial_trajectory::calculateCoefficients(start_pos[joint], 0.0, 0.0, 
                                                                        mdh_rad[joint], 0.0, 0.0, T);                                                 
    }
    double t = dt;
    while (!stop_flag) {
        stop_flag = 1;
        if (t <= T) {
            auto clock_start = std::chrono::steady_clock::now();
            for (int joint = 0; joint < num_joints_; ++joint) {
                if (motor_enable_list_[joint] == 0) 
                    continue;
                double ref_pos, vel, acc;
                std::tie(ref_pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[joint], t);
                if (set_mdh_position(joint, ref_pos)!= 0)
                {
                    fclose(fwrite_data);
                    return -2;
                }
                if (sensor_enable_list_[joint] == 1) {
                    read_torque_sensor(joint);
                    torque_sensor_data[joint] = get_torque_sensor_data(joint);
                }
                fprintf(fwrite_data, " %.4f %.4f %.4f %.4f %.4f ", 
                        ref_pos, get_mdh_position(joint), get_velocity(joint), get_current(joint), torque_sensor_data[joint]);
                if ((int)(t / dt) % 10 == 0) {
                    std::cout << "arm " << arm_index_ << " | joint " << joint << " | target pos: " << ref_pos
                                << " | act pos: " << get_mdh_position(joint) << " | act vel: " << get_velocity(joint) 
                                << " | act cur: " << get_current(joint) << "| act tor: "<< torque_sensor_data[joint] << std::endl;
                }    
            }
            t += dt;
            stop_flag = 0;
            auto clock_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = clock_end - clock_start;
            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_seconds).count();
            if (elapsed_us < dt * 1000000)
                std::this_thread::sleep_for(std::chrono::microseconds((long)(dt * 1000000 - elapsed_us)));
            fprintf(fwrite_data, "%ld \n", elapsed_us);
    } else {
            stop_flag = 1;
        }
    }
    fclose(fwrite_data);
    return 0;
}

int RobotArms::read_torque_sensor(int index) {
    if (arm_torque_sensor_[index].read_sensor() != 0)
        return -1;
    poll_torque_sensor_response();
    return 0;
}

double RobotArms::get_torque_sensor_data(int index) {
    return arm_torque_sensor_[index].torque_data_ * arm_torque_sensor_[index].sign_;
}

int RobotArms::poll_torque_sensor_response(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    int received_flag = 0;

    while (!received_flag) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) {
            break;
        }

        uint32_t respId = 0;
        uint8_t respMaxData[8];
        uint8_t respDlc = 0;
        uint8_t receiveStatus = can_normal_receive(EU_CANABLE, respMaxData, &respId, sensor_channel_, sensor_channel_);
        if (receiveStatus < 0) {
            continue;
        }

        auto it = sensor_map_.find(respId & 0xFF);
        if (it != sensor_map_.end()) {
            TorqueSensor* sensor = it->second;
            if (respId == sensor->sensor_id_) {
                sensor->control_success_ = 1;
                sensor->decode_sensor_data(respMaxData);
            }
            received_flag = 1;
        }
    }

    if (received_flag == 0) {
        std::cerr << "Sensor :No responses received in " << timeout_ms << " ms (timeout)" << std::endl;
        return -1;
    }
    return 0;
}

int RobotArms::move_line_online(const double* target_cart, double dt, double T) {
    char filename[64];
    snprintf(filename, sizeof(filename), "cart_data_arm_%d.txt", arm_index_);
    FILE* fwrite_data = fopen(filename, "w");
    if (fwrite_data == nullptr) {
        std::cerr << "open files wrong: " << filename << std::endl;
        return -1;
    }
    if (motor_enable_list_[0] == 0 || motor_enable_list_[1] == 0 || motor_enable_list_[2] == 0 ||
        motor_enable_list_[3] == 0 || motor_enable_list_[4] == 0 || motor_enable_list_[5] == 0) {
        std::cerr << "arm " << arm_index_ << " 's some joints disable, can not move_line_online" << std::endl;
        fclose(fwrite_data);
        return -2; 
    }
    // 获取当前关节角 (MDH)
    double q_current[num_joints_];
    for (int joint = 0; joint < num_joints_; ++joint) {
        q_current[joint] = get_mdh_position(joint);
    }

    // 计算当前位姿
    double curr_cart[6];
    int fk_status = kinematics::forwardKinematics(robot_config_, q_current, curr_cart);
    if (fk_status != 0) {
        std::cerr << "arm " << arm_index_ << " fk failed: " << fk_status << std::endl;
        fclose(fwrite_data);
        return -4;  // 关节超限或其他 FK 错误
    }

    // 转四元数用于 SLERP 姿态插值
    double q_curr_quat[4], q_target_quat[4];
    robotics_math::Eul2Quat(curr_cart[5], curr_cart[4], curr_cart[3], q_curr_quat);
    robotics_math::Eul2Quat(target_cart[5], target_cart[4], target_cart[3], q_target_quat);
    std::vector<double> traj_coeffs = polynomial_trajectory::calculateCoefficients(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, T);
    if (traj_coeffs.empty()) {
        std::cerr << "interpolation failed" << std::endl;
        fclose(fwrite_data);
        return -3;  // 视为规划失败
    }

    // 上一关节角初始化为当前
    double last_q[num_joints_];
    memcpy(last_q, q_current, sizeof(q_current));

    // 循环执行每步
    double t = dt;
    int stop_flag = 0;
    while (!stop_flag) {
        if (t > T) {
            stop_flag = 1;
            break;
        }

        auto clock_start = std::chrono::steady_clock::now();

        // 使用五次多项式计算平滑插值因子 ratio
        double ratio, v_ignore, a_ignore;
        std::tie(ratio, v_ignore, a_ignore) = polynomial_trajectory::evaluatePolynomial(traj_coeffs, t);

        // 位置线性插值
        double interp_cart[6];
        for (int i = 0; i < 3; ++i) {
            interp_cart[i] = curr_cart[i] + ratio * (target_cart[i] - curr_cart[i]);
        }
      
        double interp_quat[4];
        robotics_math::slerp(q_curr_quat, q_target_quat, ratio, interp_quat);
        double interp_rx, interp_ry, interp_rz;
        robotics_math::Quat2Eul(interp_quat, interp_rz, interp_ry, interp_rx);
        interp_cart[3] = interp_rx;
        interp_cart[4] = interp_ry;
        interp_cart[5] = interp_rz;
        // 逆运动学求关节角
        double interp_q[num_joints_];
        int ik_status = kinematics::inverseKinematicsSingularityRobust(robot_config_, interp_cart, 1e-6, 20, last_q, interp_q);
        if (ik_status != 0) {
            std::cerr << "arm " << arm_index_ << " ik failed (t=" << t << "): " << ik_status << std::endl;
            fclose(fwrite_data);
            return -3;  // IK 无解
        }

        // 更新 last_q
        memcpy(last_q, interp_q, sizeof(interp_q));

        // 发送位置指令并记录 (pos_control)
        double tor_data[num_joints_] = {0.0};
        for (int joint = 0; joint < num_joints_; ++joint) {
            if (motor_enable_list_[joint] == 0) continue;

            if (set_mdh_position(joint, interp_q[joint]) != 0) {
                fclose(fwrite_data);
                return -2;
            }

            // 扭矩传感器
            if (sensor_enable_list_[joint] == 1) {
                if (read_torque_sensor(joint) != 0) {
                    std::cerr << "read sensor failed: joint " << joint << std::endl;
                }
                tor_data[joint] = get_torque_sensor_data(joint);
            }

            // 记录数据
            fprintf(fwrite_data, "%.4f %.4f %.4f %.4f %.4f ",
                    interp_q[joint], get_mdh_position(joint), get_velocity(joint), get_current(joint), tor_data[joint]);

            // 打印采样
            if (static_cast<int>(t / dt) % 10 == 0) {
                std::cout << "arm " << arm_index_ << " | joint " << joint << " | target q: " << interp_q[joint] 
                          << " | act pos: " << get_mdh_position(joint) << " | vel: " << get_velocity(joint)
                          << " | cur: " << get_current(joint) << " | tor: " << tor_data[joint] << std::endl;
            }
        }
        // 轮询响应
        t += dt;
        // 精确计时睡眠
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        if (elapsed_us < dt * 1000000) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(dt * 1000000 - elapsed_us)));
        }
        fprintf(fwrite_data, "%ld \n", elapsed_us);
    }

    fclose(fwrite_data);
    return 0;
}

int RobotArms::_plan_line_offline(const double* target_cart, double dt, double T,
                                   std::vector<std::vector<double>>& trajectory)
{
    trajectory.clear();

    // 获取当前关节角 (rad)
    double q_current[num_joints_];
    for (int joint = 0; joint < num_joints_; ++joint) {
        q_current[joint] = get_mdh_position(joint);
    }

    // 计算当前位姿
    double curr_cart[6];
    int fk_status = kinematics::forwardKinematics(robot_config_, q_current, curr_cart);
    if (fk_status != 0) {
        std::cerr << "arm " << arm_index_ << " fk failed: " << fk_status << std::endl;
        return -4;
    }

    // 转四元数用于 SLERP
    double q_curr_quat[4], q_target_quat[4];
    robotics_math::Eul2Quat(curr_cart[5], curr_cart[4], curr_cart[3], q_curr_quat);
    robotics_math::Eul2Quat(target_cart[5], target_cart[4], target_cart[3], q_target_quat);

    // 五次多项式系数
    std::vector<double> traj_coeffs = polynomial_trajectory::calculateCoefficients(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, T);
    if (traj_coeffs.empty()) {
        std::cerr << "interpolation failed" << std::endl;
        return -3;
    }

    // 初始化上一关节角
    double last_q[num_joints_];
    memcpy(last_q, q_current, sizeof(last_q));

    double t = dt;
    while (t <= T) {  // 
        // 计算平滑插值因子
        double ratio, v_ignore, a_ignore;
        std::tie(ratio, v_ignore, a_ignore) = polynomial_trajectory::evaluatePolynomial(traj_coeffs, t);

        // 位置线性插值
        double interp_cart[6];
        for (int i = 0; i < 3; ++i) {
            interp_cart[i] = curr_cart[i] + ratio * (target_cart[i] - curr_cart[i]);
        }

        // 姿态 SLERP 插值
        double interp_quat[4];
        robotics_math::slerp(q_curr_quat, q_target_quat, ratio, interp_quat);
        double interp_rx, interp_ry, interp_rz;
        robotics_math::Quat2Eul(interp_quat, interp_rz, interp_ry, interp_rx);
        interp_cart[3] = interp_rx;
        interp_cart[4] = interp_ry;
        interp_cart[5] = interp_rz;

        // 逆运动学
        double interp_q[num_joints_];
        int ik_status = kinematics::inverseKinematicsSingularityRobust(robot_config_, interp_cart, 1e-6, 20, last_q, interp_q);
        if (ik_status != 0) {
            std::cerr << "arm " << arm_index_ << " ik failed (t=" << t << "): " << ik_status << std::endl;
            return -3;
        }

        // 保存该步关节角（rad）
        trajectory.emplace_back(interp_q, interp_q + num_joints_);

        // 更新 last_q 用于下一次 IK 初始猜值
        memcpy(last_q, interp_q, sizeof(last_q));

        t += dt;
    }

    return 0;  // 规划成功
}

int RobotArms::_execute_offline_trajectory(const std::vector<std::vector<double>>& trajectory,
                                           double dt, FILE* fwrite_data)
{
    if (trajectory.empty()) {
        return 0;  // 无轨迹点，直接返回成功（与原代码 T < dt 时行为一致）
    }

    double t = dt;
    for (size_t step = 0; step < trajectory.size(); ++step) {
        auto clock_start = std::chrono::steady_clock::now();

        double tor_data[num_joints_] = {0.0};

        for (int joint = 0; joint < num_joints_; ++joint) {
            if (motor_enable_list_[joint] == 0) continue;

            // 发送目标位置（deg）
            double target_deg = trajectory[step][joint];
            if (set_mdh_position(joint, target_deg) != 0) {
                fclose(fwrite_data);
                return -2;
            }

            // 读取扭矩传感器
            if (sensor_enable_list_[joint] == 1) {
                if (read_torque_sensor(joint) != 0) {
                    std::cerr << "read sensor failed: joint " << joint << std::endl;
                }
                tor_data[joint] = get_torque_sensor_data(joint);
            }

            // 记录数据（保持原格式：规划值 rad，实际位置 deg）
            fprintf(fwrite_data, "%.4f %.4f %.4f %.4f %.4f ",
                    trajectory[step][joint],                     // planned q (rad)
                    get_mdh_position(joint),                     // act pos (deg)
                    get_velocity(joint),
                    get_current(joint),
                    tor_data[joint]);

            // 打印采样（与原代码一致）
            if (static_cast<int>(t / dt) % 10 == 0) {
                std::cout << "arm " << arm_index_ << " | joint " << joint
                          << " | target q: " << target_deg
                          << " | act pos: " << get_mdh_position(joint)
                          << " | vel: " << get_velocity(joint)
                          << " | cur: " << get_current(joint)
                          << " | tor: " << tor_data[joint] << std::endl;
            }
        }

        // 精确计时睡眠
        auto clock_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = clock_end - clock_start;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        if (elapsed_us < dt * 1000000) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(dt * 1000000 - elapsed_us)));
        }

        // 记录本周期耗时
        fprintf(fwrite_data, "%ld \n", elapsed_us);

        t += dt;
    }

    return 0;
}

int RobotArms::move_line_offline(const double* target_cart, double dt, double T)
{
    char filename[64];
    snprintf(filename, sizeof(filename), "cart_data_arm_%d.txt", arm_index_);
    FILE* fwrite_data = fopen(filename, "w");
    if (fwrite_data == nullptr) {
        std::cerr << "open files wrong: " << filename << std::endl;
        return -1;
    }

    // 检查电机使能
    if (motor_enable_list_[0] == 0 || motor_enable_list_[1] == 0 || motor_enable_list_[2] == 0 ||
        motor_enable_list_[3] == 0 || motor_enable_list_[4] == 0 || motor_enable_list_[5] == 0) {
        std::cerr << "arm " << arm_index_ << " 's some joints disable, can not moveL" << std::endl;
        fclose(fwrite_data);
        return -2;
    }

    // 离线规划
    std::vector<std::vector<double>> trajectory;
    int plan_status = _plan_line_offline(target_cart, dt, T, trajectory);
    if (plan_status != 0) {
        std::cerr << "arm " << arm_index_ << " offline planning failed: " << plan_status << std::endl;
        std::cerr << "Robot may occurs singularity or out of work space, cannot execute trajectory." << std::endl;
        fclose(fwrite_data);
        return plan_status;
    }

    // 执行规划好的轨迹
    int exec_status = _execute_offline_trajectory(trajectory, dt, fwrite_data);
    fclose(fwrite_data);

    if (exec_status != 0) {
        return exec_status;
    }

    return 0;
}
int RobotArms::get_tcp_data(double tcp[6])
{
    double q[num_joints_];
    for (int joint = 0;joint<num_joints_;++joint)
    {
        if (motor_enable_list_[joint] == 0) 
        {
            std::cout << "arm " << arm_index_ << " joint " << joint << " is disabled!" << "can not calculate fk"<< std::endl;
            return -1;
        }else{
            q[joint] = get_mdh_position(joint);

        }
    }
    kinematics::forwardKinematics(robot_config_, q, tcp);
    return 0;
}