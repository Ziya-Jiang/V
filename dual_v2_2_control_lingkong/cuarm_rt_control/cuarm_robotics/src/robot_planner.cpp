#include "robot_planner.h"
#include "sole_arm_planner.h"
#include "dual_arm_planner.h"
#include <filesystem>

RobotPlanner::RobotPlanner(const YAML::Node& config, const std::string& config_prefix_path){
    float dt = config["rt_control"]["dt_send"].as<float>();
    std::string library_name = config["rt_control"]["robotics_library"].as<std::string>();
    planner_state_.ArmSize = static_cast<uint8_t>(config["robot"]["arm"].size());
    planner_command_.ArmSize =  static_cast<uint8_t>(config["robot"]["arm"].size());
    
    std::string urdf_filename = config["robot"]["urdf"].as<std::string>();
    std::string urdf_path = std::filesystem::canonical(std::filesystem::path(config_prefix_path + '/' + urdf_filename)).string();
    if (planner_state_.ArmSize > 1){
        arm_planner_ = new DualArmPlanner(planner_command_, planner_state_, dt, config["robot"], config_prefix_path, urdf_path,library_name);
    }else{
        int id = 0;
        arm_planner_ = new SoleArmPlanner(planner_command_, planner_state_, dt, config["robot"], config_prefix_path, urdf_path, library_name, id);
    }

    //Config for gripper
    if (config["robot"]["gripper"]){
        YAML::Node gripper_node = config["robot"]["gripper"];
        planner_state_.GripperSize =  static_cast<uint8_t>(gripper_node.size());
        for (int gri_i=0; gri_i < planner_state_.GripperSize; gri_i++){
            planner_state_.GripperJointSize[gri_i] = gripper_node[gri_i]["joint_size"].as<int>();

            //Update gripper limit
            for (int i=0; i<planner_state_.GripperJointSize[gri_i]; i++){
                if (gripper_node[gri_i]["safety"].IsDefined() && gripper_node[gri_i]["safety"]["joint_soft_limit"].IsDefined()){
                    planner_state_.gripper_soft_limit_position[gri_i][i][0] = gripper_node[gri_i]["safety"]["joint_soft_limit"]["position"][0][i].as<float>();
                    planner_state_.gripper_soft_limit_position[gri_i][i][1] = gripper_node[gri_i]["safety"]["joint_soft_limit"]["position"][1][i].as<float>();
                }
                    
                if (gripper_node[gri_i]["safety"].IsDefined() && gripper_node[gri_i]["safety"]["joint_hard_limit"].IsDefined()){
                    planner_state_.gripper_hard_limit_position[gri_i][i][0] = gripper_node[gri_i]["safety"]["joint_hard_limit"]["position"][0][i].as<float>();
                    planner_state_.gripper_hard_limit_position[gri_i][i][1] = gripper_node[gri_i]["safety"]["joint_hard_limit"]["position"][1][i].as<float>();
                }
            }
        }
    }else{
        planner_state_.GripperSize = 0;
    }
}

RobotPlanner::~RobotPlanner(){
    delete arm_planner_;
    arm_planner_ = nullptr;
}

void RobotPlanner::updatePlannerState(const RobotState& ro_state){
    arm_planner_->updatePlannerState(ro_state);
}

void RobotPlanner::update(PanelCommand& pa_cmd, const RobotState& ro_state){
    arm_planner_->update(pa_cmd, ro_state);
    if (planner_command_.enable_set){
        planner_state_.setting_update_finished = true;
    }else{
        planner_state_.setting_update_finished = false;
    }
    planner_state_.received_panel_command_timestamp = pa_cmd.send_timestamp;

    //Gripper related
    for (int gri_i=0; gri_i<ro_state.GripperSize; gri_i++){
        for (int i=0; i<ro_state.GripperJointSize[gri_i]; i++){
            if (pa_cmd.simulation){
                planner_state_.GripperJointPos[gri_i][i] = pa_cmd.GripperJointCmd[gri_i][i];
            }else{
                planner_state_.GripperJointPos[gri_i][i] = ro_state.GripperJointPos[gri_i][i];
            }
        }
    }
}

void RobotPlanner::getPlannerCommandAndPlannerState(PlannerCommand*& pl_cmd_ptr, PlannerState*& pl_state_ptr){
    pl_cmd_ptr = &planner_command_;
    pl_state_ptr = &planner_state_;
}

VectorXd RobotPlanner::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau, int arm_i){
    return arm_planner_->convertMotorJoint(X, convert_to, is_tau, arm_i);
}