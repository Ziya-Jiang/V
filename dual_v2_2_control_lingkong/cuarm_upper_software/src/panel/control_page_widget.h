#ifndef CONTROL_PAGE_WIDGET_H
#define CONTROL_PAGE_WIDGET_H
#include <iostream>

#include <QString>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "arm_setting_widget.h"
#include "control_interface.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

class ControlPageWidget: public QWidget, public ControlInterface{
    Q_OBJECT
    public:
        ControlPageWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<int>& motor_size, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent = nullptr);
        void setInFront(bool in_front);
        bool sendVal();
        ArmSettingWidget* getArmSettingWidget();
        void addGripperWidget(const std::vector<std::string>& gripper_names, const std::vector<std::vector<std::string>>& gripper_joint_names);
    private:
        QHBoxLayout* layout_;
        QVBoxLayout* layout_R_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::map<const QString, ControlInterface*> control_widgets_;
        QWidget* initialControlArmPage(const QString& name, const std::vector<int>& motor_size, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent);
        QWidget* initialControlGripperPage(const QString& name, const std::vector<std::string>& gripper_names, const std::vector<std::vector<std::string>>& gripper_joint_names, QWidget* parent);
    public slots:
        void slotRecordingStateChanged(bool enable);
        void slotResetPage();
};

#endif