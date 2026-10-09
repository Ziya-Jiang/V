#ifndef ARM_PLANNER_H
#define ARM_PLANNER_H

#include <yaml-cpp/yaml.h>

#include "robotics/robot/arm.h" //For ConvertType
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"

class ArmPlanner{
    public:
        virtual ~ArmPlanner() = default;
        virtual void updatePlannerState(const RobotState& ro_state) = 0;
        virtual void update(const PanelCommand& pa_cmd, const RobotState& ro_state) = 0;
        virtual VectorXd convertMotorJoint(const VectorXd& X, ConvertType convert_to, bool is_tau, int arm_i) const = 0;
};

#endif