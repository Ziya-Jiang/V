#ifndef ARM_WIDGET_H
#define ARM_WIDGET_H

#include <QTabWidget>

#include "control_interface.h"
#include "arm_setting_widget.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"  
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

enum class ArmWidgetTab{
    kMotor,
    kJoint,
    kSetting
};

class ArmWidget: public QTabWidget, public ControlInterface{
    Q_OBJECT
    public:
        ArmWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<int>& motor_size, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent = nullptr);
        void setInFront(bool in_front);
        bool sendVal();
        ArmSettingWidget* getArmSettingWidget();
        void enableTab(const ArmWidgetTab& tab, bool enable);
        void resetWidget();
    private:
        QString current_tab_name_;
        std::map<const QString, QWidget*> widgets_;
        std::map<const QString, ControlInterface*> control_widgets_;

        bool control_algorithm_enabled_ = false;
        bool motor_page_disabled_ = false;

        QWidget* initialControlArmMotorPage(const QString& name, ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<int>& motor_size, QWidget* parent);
        QWidget* initialControlArmJointPage(const QString& name, ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent);
        QWidget* initialControlArmSettingPage(const QString& name, ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const YAML::Node& config, QWidget* parent);
    private slots:
        void slotTabCurrentChanged(int i);
        void slotForceControlEnabled(bool enable, ControlType top, ControlType base);
};
#endif