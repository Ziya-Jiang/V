#ifndef JOINT_CONTROL_WIDGET_H
#define JOINT_CONTROL_WIDGET_H
#include <functional>
#include <string>
#include <vector>

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QStackedLayout>
#include <QStringList>
#include <QVBoxLayout>
#include <QWidget>

#include <yaml-cpp/yaml.h>

#include "common_widget.h"
#include "slider_block.h"
#include "speed_interface.h"
#include "zero_set_block.h"
#include "control_interface.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

class JointControlWidget: public QWidget, public ControlInterface, public SpeedInterface, public ZeroSetInterface {
    Q_OBJECT
    public:
        JointControlWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<Matrix2Xf>& joint_limits, const YAML::Node& config, QWidget* parent=0);
        void zero();
        void set();
        void setStep(float step);
        void setSliderResol(float resol);
        void setInFront(bool in_front);
        bool sendVal();
        void setMotionControlType(const std::string& motion, const std::string& orientation);
        void setJointMotionTarget(const std::vector<VectorXf>& vals);
        void setTaskMotionTarget(const std::vector<Vector6f>& vals);
        MotionControl getCurrentMotionControl() const;
        void setControl(const QString& label);
        void updateJointTxt();
    protected:
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        ComboxButton* control_combox_;
        ComboxButton* orien_combox;
        MotionControl motion_control_ = MotionControl::kJoint;
        OrientControl orient_control_ = OrientControl::kEnd;
        ControlType top_ = ControlType::kVelocity;
        SpeedSliderBlock* trans_speed_block_;
        SpeedSliderBlock* ori_speed_block_;
        QStackedLayout* stack_layout_;
        
        int arm_size_;
        std::vector<int> joint_size_;
        std::vector<VectorXf> joint_val_;
        std::vector<float> null_val_;
        std::vector<VectorXf> task_val_;
        std::vector<std::vector<JointSliderBlock*>> joint_blocks_;
        std::vector<std::vector<JointSliderBlock*>> null_blocks_;
        std::vector<std::vector<JointSliderBlock*>> task_blocks_;
        std::vector<std::vector<JointSliderBlock*>>* current_blocks_=nullptr;
    signals:
        void setControlType(ControlType top, ControlType base);
        void armMotionTypeChanged(MotionControl type);
    protected slots:
        void slotControlComboxButtonClicked(QAbstractButton* bn);
        void slotOrienComboxButtonClicked(QAbstractButton* bn);
        void slotControlTypeChanged(ControlType top, ControlType base);
        void slotForceControlEnabled(bool enable, ControlType top, ControlType base);
};

#endif