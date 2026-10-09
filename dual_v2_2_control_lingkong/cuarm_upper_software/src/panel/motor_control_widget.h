#ifndef MOTOR_CONTROL_WIDGET_H
#define MOTOR_CONTROL_WIDGET_H
#include <vector>

#include <QLabel>
#include <QComboBox>
#include <QVBoxLayout>
#include <QWidget>

#include "slider_block.h"
#include "control_interface.h"
#include "zero_set_block.h"
#include "common_widget.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"
#include "speed_interface.h"

class MotorControlWidget: public QWidget, public ControlInterface, public SpeedInterface, public ZeroSetInterface{
    Q_OBJECT
    public:
        MotorControlWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& motor_size, QWidget* parent=0);
        void zero();
        void set();
        void setStep(float step);
        void setSliderResol(float resol);
        void setInFront(bool in_front);
        bool sendVal();
    protected:
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        QVBoxLayout* layout_;
        ControlType top_, base_;
        float slider_limit_ = 1000;
        bool in_front_ = false;
        
        int arm_size_;
        std::vector<int> motor_size_;
        std::vector<VectorXf> motor_val_;
        std::vector<std::vector<JointSliderBlock*>> motor_blocks_;
        void updateMotorTxt();
    protected slots:
        void slotControlTypeChanged(ControlType top, ControlType base);
        void slotForceControlEnabled(bool enable, ControlType top, ControlType base);
};
#endif