#ifndef ROBOT_PLANNER_H
#define ROBOT_PLANNER_H

#include "arm_planner.h"

class RobotPlanner{
    public:
        RobotPlanner(const YAML::Node& config, const std::string& config_prefix_path);
        ~RobotPlanner();
        void updatePlannerState(const RobotState& ro_state);
        void update(PanelCommand& pa_cmd, const RobotState& ro_state);
        void getPlannerCommandAndPlannerState(PlannerCommand*& pl_cmd_ptr, PlannerState*& pl_state_ptr);
        VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau, int arm_i=0);
    private:
        ArmPlanner* arm_planner_;
        PlannerState planner_state_;
        PlannerCommand planner_command_;
};

#endif