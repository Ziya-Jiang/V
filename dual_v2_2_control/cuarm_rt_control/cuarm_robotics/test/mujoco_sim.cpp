
#include <unistd.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <stdexcept>

// #include <pinocchio/fwd.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_planner.h"

bool close_thread = false;
udp_node udp, udp_mujoco;
PanelCommand   pan_cmd_received;
MujocoState    mujoco_sta_received;
std::mutex mtx;
long send_running_cost;
long send_cal_cost;
long receive_running_cost;
long receive_cal_cost;

void writeToFile(FILE* fwrite_data, const MujocoState& mujoco_sta, const PlannerCommand& pla_cmd, const RobotState& robot_sta, const PanelCommand& pan_cmd, const PlannerState& pla_sta){
    
    fprintf(fwrite_data, "%ld %ld %ld %ld ", send_running_cost, send_cal_cost, receive_running_cost, receive_cal_cost);
    for (int arm_i=0; arm_i<pla_cmd.ArmSize; arm_i++){
        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", pla_cmd.Target[arm_i][i]);
        }

        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", mujoco_sta.JointPos[arm_i][i]);
        }
        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", mujoco_sta.JointVel[arm_i][i]);
        }
        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", mujoco_sta.JointTor[arm_i][i]);
        }
        
        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", robot_sta.MotorPos[arm_i][i]);
        }
        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", robot_sta.MotorVel[arm_i][i]);
        }
        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", robot_sta.MotorTor[arm_i][i]);
        }

        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", pan_cmd.JointCmd[arm_i][i]);
        }

        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", pla_sta.interpolated_target[arm_i][i]);
        }

        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            fprintf(fwrite_data, "%f ", pla_sta.joint_gravity[arm_i][i]);
        }
    }
    fprintf(fwrite_data, "\n");
}

void convertPlannerTargetToMujoco(RobotPlanner& planner, PlannerCommand& pla_cmd){
    //Convert degree to radian
    bool isTau = (pla_cmd.command_mode == 10)? true : false;
    float k = (isTau)? 1 : M_PI/180;
    for (int arm_i=0; arm_i<pla_cmd.ArmSize; arm_i++){
        Vector7d target;
        for (int i =0; i<pla_cmd.JointSize[arm_i]; i++){
            target[i] = pla_cmd.Target[arm_i][i]*k;
        }

        //Convert motor target to joint target
        target = planner.convertMotorJoint(target, ConvertType::kJoint, isTau);
        for (int i =0; i < pla_cmd.JointSize[arm_i]; i++){
            pla_cmd.Target[arm_i][i] = target[i];
        }
    }
}

void convertMujocoStateToPlanner(RobotPlanner& planner, MujocoState& mujoco_sta, RobotState& robot_sta){
    //Convert from mujoco joint state to planner motor state
    robot_sta.ArmSize = mujoco_sta.ArmSize;
    for (int arm_i=0; arm_i<mujoco_sta.ArmSize; arm_i++){
        robot_sta.JointSize[arm_i] = mujoco_sta.JointSize[arm_i];
        Vector7d pos, vel, tor;
        for (int i =0; i < mujoco_sta.JointSize[arm_i]; i++){
            pos[i] = mujoco_sta.JointPos[arm_i][i];
            vel[i] = mujoco_sta.JointVel[arm_i][i];
            tor[i] = mujoco_sta.JointTor[arm_i][i];
        }
        pos = planner.convertMotorJoint(pos, ConvertType::kMotor, false);
        vel = planner.convertMotorJoint(vel, ConvertType::kMotor, false);
        tor = planner.convertMotorJoint(tor, ConvertType::kMotor, true);
        for (int i =0; i < mujoco_sta.JointSize[arm_i]; i++){
            robot_sta.MotorPos[arm_i][i] = pos[i];
            robot_sta.MotorVel[arm_i][i] = vel[i];
            robot_sta.MotorTor[arm_i][i] = tor[i];
        }
    }
}

void receive_udp_task(){
    int delay_us = 2222; //2222us = 450Hz
    long t0, t1, t2;
    t0 = get_time_now();
    PanelCommand pan_cmd_temp;
    MujocoState mujoco_sta_temp;
    while (!close_thread){
        if (udp_select(&udp, 50)){
            udp_unpack_panel_command(&udp, &pan_cmd_temp);
        }

        if (udp_select(&udp_mujoco, 50)){
            udp_unpack_mujoco_state(&udp_mujoco, &mujoco_sta_temp);
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            copy_panel_command(&pan_cmd_temp, &pan_cmd_received);
            copy_mujoco_state(&mujoco_sta_temp, &mujoco_sta_received);
        }
        t1 = get_time_now();
        int sleep_time = delay_us - (t1-t0);
        if (sleep_time > 0){
            usleep(sleep_time);
        }
        t2 = get_time_now();
        receive_cal_cost = t1 - t0;
        receive_running_cost = t2 - t0;
        t0 = t2;
    }
}

int main(int argc, char **argv){
    //Init udp
    udp_init(&udp, (char*)"127.0.0.1", (char*)"127.0.0.1", 10201, 10101);
    udp_init(&udp_mujoco, (char*)"127.0.0.1", (char*)"127.0.0.1", 10301, 10401);

    //Open write file
    std::string project_base_path = PROJECT_SOURCE_DIR;
    std::string write_path = project_base_path+"/build/33.txt";
    FILE* fwrite_data = fopen(write_path.c_str(), "wb");

    //Open config file
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::string config_path = config_prefix_path+"/planner_config_mujoco.yaml";
    YAML::Node config = YAML::LoadFile(config_path);

    //Init planner
    PlannerCommand* pla_cmd_ptr;
    PlannerState*   pla_sta_ptr;
    RobotPlanner planner(config, config_prefix_path);
    planner.getPlannerCommandAndPlannerState(pla_cmd_ptr, pla_sta_ptr);
    
    //Setup 
    int cnt = 0;
    int consistent_checked = false;
    PanelCommand   pan_cmd;
    MujocoState    mujoco_sta;
    RobotState     robot_sta;
    std::vector<std::string> arm_names;
    float dt = config["dt"].as<float>();
    int arm_size = config["arm"].size();
    for (int arm_i=0; arm_i<arm_size; arm_i++){
        arm_names.push_back(config["arm"][arm_i]["robot_name"].as<std::string>());
    }
    pan_cmd.RunningState = RunningState::kWaiting;

    //Start receiver thread
    std::thread udp_thread(receive_udp_task);

    int delay_us = dt*1e6;
    long t0, t1, t2;
    t0 = get_time_now();
    while (pan_cmd.RunningState != RunningState::kShutDown){
        {
            std::lock_guard<std::mutex> lock(mtx);
            copy_panel_command(&pan_cmd_received, &pan_cmd);
            copy_mujoco_state(&mujoco_sta_received, &mujoco_sta);
        }

        if (pan_cmd.RunningState == RunningState::kRemote){
            if (!consistent_checked){
                if (arm_size != pan_cmd.ArmSize) throw std::runtime_error("Inconsistent arm size with panel command.");
                for (int arm_i=0; arm_i<arm_size; arm_i++){
                    if (arm_names[arm_i].compare(pan_cmd.RobotName[arm_i]) != 0){
                        throw std::runtime_error("Inconsistent arm name with panel command.");
                    }
                }
                consistent_checked = true;
            }

            convertMujocoStateToPlanner(planner, mujoco_sta, robot_sta);
            planner.update(pan_cmd, robot_sta);
            writeToFile(fwrite_data, mujoco_sta, *pla_cmd_ptr, robot_sta, pan_cmd, *pla_sta_ptr);
            convertPlannerTargetToMujoco(planner, *pla_cmd_ptr);
        }

        udp_pack_planner_command(&udp_mujoco, pla_cmd_ptr);
        udp_send(&udp_mujoco);

        udp_pack_planner_state(&udp, pla_sta_ptr);
        udp_send(&udp);

        t1 = get_time_now();
        int sleep_time = delay_us - (t1-t0);
        if (sleep_time > 0){
            usleep(sleep_time);
        }
        t2 = get_time_now();
        send_cal_cost = t1 - t0;
        send_running_cost = t2 - t0;
        t0 = t2;
        cnt ++;
    }
    //Send shutdown command to mujoco
    pla_cmd_ptr->RunningState = RunningState::kShutDown;
    udp_pack_planner_command(&udp_mujoco, pla_cmd_ptr);
    udp_send(&udp_mujoco);

    close_thread = true;
    udp_thread.join();

    fclose(fwrite_data);
    udp_close(&udp_mujoco);
    udp_close(&udp);
}
