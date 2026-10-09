#ifndef DUAL_ARM_PLANNER_H
#define DUAL_ARM_PLANNER_H

#include <vector>
#include <yaml-cpp/yaml.h>

#include "arm_planner.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "sole_arm_planner.h"

class DualArmPlanner: public ArmPlanner{
    public:
        DualArmPlanner(PlannerCommand& pl_cmd, PlannerState& pl_state, float dt, const YAML::Node& config, const std::string& config_prefix_path, const std::string& urdf_path, const std::string& library_name);
        ~DualArmPlanner();
        void updatePlannerState(const RobotState& ro_state);
        void update(const PanelCommand& pa_cmd, const RobotState& ro_state) override;
        VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau, int arm_i) const override;
    private:
        int arm_size_;
        PlannerState& planner_state_;
        PlannerCommand& planner_command_;
        std::vector<SoleArmPlanner*> sole_arm_planners_;
};

#endif