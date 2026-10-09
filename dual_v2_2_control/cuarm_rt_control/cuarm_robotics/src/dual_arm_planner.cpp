#include "dual_arm_planner.h"

DualArmPlanner::DualArmPlanner(PlannerCommand& pl_cmd, PlannerState& pl_state, float dt, const YAML::Node& config,
    const std::string& config_prefix_path, const std::string& urdf_path, const std::string& library_name)
    : planner_command_(pl_cmd),  planner_state_(pl_state){
    arm_size_ = static_cast<int>(config["arm"].size());
    for (int arm_i=0; arm_i < arm_size_; arm_i++){
        SoleArmPlanner* sole_arm = new SoleArmPlanner(pl_cmd, pl_state, dt, config, config_prefix_path, urdf_path, library_name, arm_i);
        sole_arm_planners_.push_back(sole_arm);
    }
}

DualArmPlanner::~DualArmPlanner(){
    for (int arm_i=0; arm_i < arm_size_; arm_i++){
        delete sole_arm_planners_[arm_i];
        sole_arm_planners_[arm_i] = nullptr;
    }
}

void DualArmPlanner::updatePlannerState(const RobotState& ro_state){
    for (int arm_i=0; arm_i < arm_size_; arm_i++){
        sole_arm_planners_[arm_i]->updatePlannerState(ro_state);
    }
}

void DualArmPlanner::update(const PanelCommand& pa_cmd, const RobotState& ro_state){
    for (int arm_i=0; arm_i < arm_size_; arm_i++){
        sole_arm_planners_[arm_i]->update(pa_cmd, ro_state);
    }
}

VectorXd DualArmPlanner::convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau, int arm_i) const {
    return sole_arm_planners_[arm_i]->convertMotorJoint(X, convert_to, is_tau, arm_i);
}