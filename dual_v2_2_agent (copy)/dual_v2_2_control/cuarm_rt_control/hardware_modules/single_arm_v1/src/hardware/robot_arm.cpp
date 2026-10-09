#include "robot_arm.h"
#include "can_normal.h"
#include "can_fd.h"

RobotArm::RobotArm(const Robot& robot)
    : robot_config_(robot), can_channel_(robot.motors.can_channel) {
    motors_.resize(num_joints_);

    for (int i = 0; i < num_joints_; ++i) {
        uint32_t id = robot.motors.ids[i];
        MotorType type = robot.motors.types[i];  // 从配置读取类型

        motor_enable_list_[i] = robot.motors.enabled[i];
        joint_offsets_[i] = robot.mdh.init_mdhpos[i];  // 假设 Robot 有这个字段
        joint_signs_[i] = robot.mdh.sign[i];     // 1 或 -1
        if (type == MotorType::WHJ) {
            auto motor = std::make_unique<WhjMotor>(id, can_channel_);
            whj_motor_map_[id] = motor.get();
            motors_[i] = std::move(motor);
        } else if (type == MotorType::RMD) {
            auto motor = std::make_unique<RmdMotor>(id, can_channel_);
            rmd_motor_map_[id] = motor.get();
            motors_[i] = std::move(motor);
        } else {
            std::cerr << "Unknown motor type for joint " << i << std::endl;
        }
    }
    for (int i = 0; i < num_sensors_;++i){
        tqe_sensor_enable_list_[i] = robot.sensors.enabled[i];
        uint32_t id = robot.sensors.ids[i]; 
        new (&tqe_sensor_[i]) TorqueSensor(id, can_channel_);
        tqe_sensor_map_[id] = &tqe_sensor_[i];   
        torque_signs[i] = robot.sensors.sign[i];
    }
}

RobotArm::~RobotArm() {
    // Cleanup resources
}

int RobotArm::poll_canfd_responses(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    int received_flag = 0;

    while (!received_flag) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) break;

        uint32_t respId = 0;
        uint8_t respMaxData[64] = {0};
        uint8_t respDlc = 0;
        uint8_t receiveStatus = canfd_receive(can_channel_, 0, &respId, respMaxData, &respDlc, 1);
        if (receiveStatus < 0) continue;

        // 通过 motor_id 查找（Whj ID 通常低字节区分）
        auto it = whj_motor_map_.find(respId & 0xFF);
        if (it != whj_motor_map_.end()) {
            MotorInterface* motor = it->second;
            // 统一调用虚函数 decode_feedback（推荐方式）
            motor->decode_feedback(respId, respMaxData, respDlc);
            received_flag = 1;
        }
    }

    if (!received_flag) {
        std::cerr << "WhjMotor: No CAN-FD responses in " << timeout_ms << " ms" << std::endl;
        return -1;
    }
    return 0;
}

int RobotArm::poll_can_responses(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    int received_flag = 0;

    while (!received_flag) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) break;

        uint32_t respId = 0;
        uint8_t respMaxData[8] = {0};
        uint8_t respDlc = 0;
        uint8_t receiveStatus = can_normal_receive(can_channel_, respMaxData, &respId);
        //std::cout << "RobotArm poll_can_responses received ID: " << std::hex << respId << std::dec << std::endl;

        auto it = rmd_motor_map_.find(respId - 0x240);
        if (it != rmd_motor_map_.end()) {
            MotorInterface* motor = it->second;
            // 统一调用虚函数 decode_feedback
            motor->decode_feedback(respId, respMaxData, respDlc);
            received_flag = 1;
        }
        auto sit = tqe_sensor_map_.find(respId - 0x100);
        if (sit != tqe_sensor_map_.end() && sit->second != nullptr) {
            sit->second->decode_sensor_data(respMaxData);
            received_flag = 1;
        }
    }

    if (!received_flag) {
        std::cerr << " No CAN responses in " << timeout_ms << " ms" << std::endl;
        return -1;
    }
    return 0;
}
double RobotArm::mdh_to_joints(int index, const double mdh_rad) {
    double joints_rad = joint_signs_[index] * (mdh_rad + joint_offsets_[index]);
    return joints_rad;
}

double RobotArm::joints_to_mdh(int index, const double joints_rad) {
    double mdh_rad = (joint_signs_[index] * joints_rad - joint_offsets_[index]);
    return mdh_rad;
}

int RobotArm::initialize(int max_retry_times) { 
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        int retry_count = 0;    
        MotorInterface* motor = motors_[joint].get();
        if (motor->get_type() != MotorType::WHJ) {
            if (enable_motor(joint, true) != 0) return -1;
        }else{
            bool success = false;
            while (!success && retry_count < max_retry_times){
                if (motor->set_IAP() == 0) {
                    poll_canfd_responses(10);
                    if (motor->is_control_success()) {
                        success = true;
                        // std::cout << "Joint " << joint << "  set_IAP success" << std::endl;
                    } else {
                        std::cout << "Joint " << joint << "  set_IAP failed, retrying .... " << retry_count << std::endl;
                    }
                } else {
                    std::ostringstream oss;
                    std::cout << "JOINT " << joint << "  sending IAP cmd failed, check wire connection" << retry_count << std::endl;
                    return -1;
                }
                retry_count++;
            }
            if (enable_motor(joint, true, max_retry_times) != 0) return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return 0;
}

int RobotArm::shut_down() {
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        enable_motor(joint, false, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

int RobotArm::set_control_mode(CONTROL_MODE mode) {
    for (int joint = 0; joint < num_joints_; ++joint) {
        set_control_mode(joint, mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

int RobotArm::set_control_mode(int index, CONTROL_MODE mode){
    if (motor_enable_list_[index] == 0) return 0;
    int max_retry_times = 100;
    int retry_count = 0;
    bool success = false;
    while (!success && retry_count < max_retry_times) {
        if (motors_[index]->select_mode(mode) == 0) {
            if (motors_[index]->get_type() != MotorType::WHJ) {
                poll_can_responses(10);
            }
            else
            {
                poll_canfd_responses(10);
            }
            if (motors_[index]->is_control_success()) {
                success = true;
                // std::cout << "Joint " << index << "  select_mode success" << std::endl;
            } else {
                // std::cout << "Joint " << index << "  select_mode failed, retrying .... " << retry_count << std::endl;
            }
        } else {
            std::ostringstream oss;
            std::cout << "JOINT " << index << "sending select_mode cmd failed, check wire connection" << retry_count << std::endl;
            return -1;
        }
    }
    return 0;
}

void RobotArm::update_encoder_zero_offset(int index, float offset){
    joint_offsets_[index] += offset;
}

bool RobotArm::is_brake_opened(int index) const{
    if (motors_[index]->get_type() == MotorType::WHJ){
        WhjMotor* whj_motor = dynamic_cast<WhjMotor*>(motors_[index].get());
        return whj_motor->brake_status_ == 0? true : false;
    }
    return true;
}

MotorType RobotArm::get_motor_type(int index) const{
    return motors_[index]->get_type();
}

int RobotArm::enable_motor(int index, bool enable, int max_retry_times){
    int retry_count = 0;
    bool success = false;
    motor_enable_list_[index] == (enable)? 1 : 0;
    std::string func_type_str= (enable)? " enable " : " disable ";
    int (MotorInterface::*funcPtr)(void) = (enable)? &MotorInterface::enable : &MotorInterface::disable;
    while (!success && retry_count < max_retry_times) {
        if (motors_[index]->get_type() != MotorType::WHJ) {
            if (enable){
                motors_[index]->set_velocity(0.0);
                poll_can_responses(10);
                if (motors_[index]->is_control_success()) {
                    success = true;
                }
            }else{
                motors_[index]->disable();
                // poll_can_responses(10);
                success = true;
            }
        }else{
            if (((*motors_[index]).*funcPtr)() == 0) {
                poll_canfd_responses(10);
                if (motors_[index]->is_control_success()) {
                    success = true;
                }
            } else {
                std::ostringstream oss;
                oss << "WHJ Motor ID: " << motors_[index]->get_id() 
                    << func_type_str << "failed, check wire connection" << retry_count;
                std::cerr << oss.str() << std::endl;
                return -1;
            }
        }
        retry_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!success) {
        std::ostringstream oss;
        oss << " Failed to" << func_type_str << max_retry_times 
            << " times, motor ID: " << motors_[index]->get_id();
        std::cerr << oss.str() << std::endl;
        return -1;
    }

    //Check if brake opened 
    if (enable){
        set_control_mode(index, CONTROL_MODE::pos_control); //WHJ must set to position for brake to be release
        retry_count = 0;
        bool brake_opened = false;
        while (!brake_opened && retry_count < max_retry_times){
            read_status(index);
            brake_opened = is_brake_opened(index);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            retry_count++;
        }
        if (!brake_opened){
            std::cout << "joint: " << index <<" brake haven't opened!!!!\n";
            return -2;
        }
    }
    return 0;
}

int RobotArm::set_mdh_position(int index, double mdh_pos) {
    double joint_pos = mdh_to_joints(index, mdh_pos);
    if (motors_[index]->set_absposition(joint_pos) !=0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    return 0;
}

int RobotArm::set_velocity(int index, double vel_rad_s) {
    if (motors_[index]->set_velocity(vel_rad_s * joint_signs_[index]) != 0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    return 0;
}

int RobotArm::set_current(int index, double cur_A) {
    if (motors_[index]->set_current(cur_A * joint_signs_[index]) != 0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    return 0;
}
int RobotArm::read_torque_sensor(int index) {
    if (index < 0 || index >= num_sensors_ ) {
        std::cout << "[Sensor] index out of range" << std::endl;
        return -1;
    }
    int ret = tqe_sensor_[index].read_sensor();
    poll_can_responses();
    return ret;
}
int RobotArm::read_status(int index) {   
    if (motors_[index]->read_status() != 0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    // 更新本地缓存
    return 0;
}

double RobotArm::get_mdh_position(int index) {
    double mdh_pos = joints_to_mdh(index, motors_[index]->get_position());
    return mdh_pos;
}

double RobotArm::get_velocity(int index) {
    return motors_[index]->get_velocity() * joint_signs_[index];
}

double RobotArm::get_current(int index) {
    return motors_[index]->get_current() * joint_signs_[index];
}

double RobotArm::get_torque(int index) {
    if (index < 0 || index >= num_sensors_ ) {
        std::cout << "[Sensor] index out of range" << std::endl;
        return 0.0;
    }
    return tqe_sensor_[index].get_torque() * torque_signs[index];
}
void RobotArm::get_all_positions(double positions[6]) const
{
}
int RobotArm::move_joint(double *target_joints, double dt, double T)
{
    FILE* fwrite_data = fopen("trajectory_data.txt", "w");
    if (fwrite_data == nullptr) {
        std::cerr << "Failed to open file: trajectory_data.txt" << std::endl;
        return -1;
    }
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (target_joints[joint] < robot_config_.mdh.limit_min[joint] ||
            target_joints[joint] > robot_config_.mdh.limit_max[joint]) {
            std::cerr << "Target joint " << joint << " position " << target_joints[joint]
                      << " out of limits [" << robot_config_.mdh.limit_min[joint]
                      << ", " << robot_config_.mdh.limit_max[joint] << "]" << std::endl;
            return -1;
        }
    }
    double start_pos[num_joints_];
    std::vector<double> traj_coef[num_joints_];
    int stop_flag = 0;
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        read_status(joint);
        start_pos[joint] = get_mdh_position(joint);
        std::cout << "Joint " << joint << " start pos: " << start_pos[joint]
                  << ", target pos: " << target_joints[joint] << std::endl;
        traj_coef[joint] = polynomial_trajectory::calculateCoefficients(start_pos[joint], 0.0, 0.0, 
                                                                        target_joints[joint], 0.0, 0.0, T);                                                 
    }
    
    
    double t = dt;
    while (t <= T)
    {
        auto cycle_start = std::chrono::steady_clock::now();
        for (int joint = 0; joint < num_joints_; ++joint) {
            if (motor_enable_list_[joint] == 0) 
                continue;
            double ref_pos, vel, acc;
            std::tie(ref_pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[joint], t);
            if (set_mdh_position(joint, ref_pos) != 0) {
                std::cerr << "Failed to set position for joint " << joint << std::endl;
                return -1;
            }
            if (motors_[joint]->get_type() != MotorType::WHJ) 
                read_status(joint);
            
            // if ((int)(t / dt) % 10 == 0) {
            //     std::cout <<"joint " << joint << " | target pos: " << ref_pos
            //                 << " | act pos: " << get_mdh_position(joint) << " | act vel: " << get_velocity(joint) 
            //                 << " | act cur: " << get_current(joint)  << std::endl;
            // }    
            fprintf(fwrite_data, "%.4f %.4f %.4f ", 
                    ref_pos, get_mdh_position(joint), get_velocity(joint)*RAD_TO_DEG);
        }

        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();
        long target_cycle_us = static_cast<long>(dt * 1000000);  // 10ms = 10000us
        long remaining_us = target_cycle_us - elapsed_us;
    
        // 打印详细的耗时信息
        std::cout << "Cycle time: " << elapsed_us << "us, "
              << "Target: " << target_cycle_us << "us, "
              << "Remaining: " << remaining_us << "us" << std::endl;

        
        if (remaining_us > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(remaining_us));
        } else {
            std::cerr << "Warning: Cycle " << (t/dt) << " took too long! " 
                    << elapsed_us << "us > " << target_cycle_us << "us" << std::endl;
        }
        auto actual_cycle_end = std::chrono::steady_clock::now();
        double actual_dt = std::chrono::duration<double>(actual_cycle_end - cycle_start).count();
        t += actual_dt;
        fprintf(fwrite_data, "%ld %ld %ld %.6f \n", 
            elapsed_us, remaining_us, target_cycle_us, actual_dt);
    }
    
    return 0;
}


int RobotArm::move_line_online(const double* target_cart, double dt, double T) {
    char filename[64];
    snprintf(filename, sizeof(filename), "cart_data_arm.txt");
    FILE* fwrite_data = fopen(filename, "w");
    if (fwrite_data == nullptr) {
        std::cerr << "open files wrong: " << filename << std::endl;
        return -1;
    }
    if (motor_enable_list_[0] == 0 || motor_enable_list_[1] == 0 || motor_enable_list_[2] == 0 ||
        motor_enable_list_[3] == 0 || motor_enable_list_[4] == 0 || motor_enable_list_[5] == 0) {
        std::cerr << " 's some joints disable, can not move_line_online" << std::endl;
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
        std::cerr <<  " fk failed: " << fk_status << std::endl;
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
            std::cerr << " ik failed (t=" << t << "): " << ik_status << std::endl;
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


            // 记录数据
            fprintf(fwrite_data, "%.4f",
                    interp_q[joint]);

            // 打印采样
            if (static_cast<int>(t / dt) % 10 == 0) {
                std::cout << " | joint " << joint << " | target q: " << interp_q[joint] 
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
