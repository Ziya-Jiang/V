#ifndef GRIPPER_WIDGET_H
#define GRIPPER_WIDGET_H

#include <QGroupBox>

#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"
#include "common_widget.h"
#include "zero_set_block.h"
#include "control_interface.h"
#include "speed_interface.h"
#include "slider_block.h"

class GripperWidget: public QGroupBox, public ControlInterface, public SpeedInterface, public ZeroSetInterface {
    Q_OBJECT
    public:
        GripperWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<std::string>& gripper_names, const std::vector<std::vector<std::string>>& gripper_joint_names, QWidget* parent=0);
        void zero();
        void set();
        void setStep(float step);
        void setSliderResol(float resol);
        void setInFront(bool in_front);
        bool sendVal();
    protected:
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        int gripper_size_ = 0;
        std::vector<int> joint_size_;
        std::vector<std::vector<JointSliderBlock*>> gripper_joint_blocks_;
};

#endif