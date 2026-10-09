#include "control_page_widget.h"
#include <QLabel>

#include <QLabel>
#include <QDebug>

#include "arm_widget.h"
#include "gripper_widget.h"

const QString kControlArmPage     = "Arm";
const QString kControlGripperPage = "Gripper";

ControlPageWidget::ControlPageWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<int>& motor_size, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent): QWidget(parent), safe_panel_command_(safe_panel_command), planner_state_shared_ptr_(pla_state){
    QWidget* arm_widget = initialControlArmPage(kControlArmPage, motor_size, joint_limits, config, this);

    QVBoxLayout* arm_layout = new QVBoxLayout();
    arm_layout->setContentsMargins(0,5,0,0);
    arm_layout->addWidget(arm_widget);

    layout_R_ = new QVBoxLayout();
    layout_R_->setContentsMargins(0,5,0,0);
    // layout_R_->setContentsMargins(0,35,0,0);
    layout_R_->setSpacing(15);
    // layout_R_->addStretch();

    layout_ = new QHBoxLayout(this);
    layout_->setContentsMargins(0,0,0,0);
    layout_->setSpacing(10);
    layout_->addLayout(arm_layout);
    layout_->addLayout(layout_R_);
}

void ControlPageWidget::setInFront(bool in_front){
    for (const std::pair<const QString, ControlInterface*>& x : control_widgets_){
        if (x.second) x.second->setInFront(in_front);
    }
}

bool ControlPageWidget::sendVal(){
    bool changed = false;
    for (const std::pair<const QString, ControlInterface*>& x : control_widgets_){
        if (x.second) changed |= x.second->sendVal();
    }
    return true;
}

ArmSettingWidget* ControlPageWidget::getArmSettingWidget(){
    ArmWidget* w = (ArmWidget*)control_widgets_[kControlArmPage];
    return (ArmSettingWidget*)w->getArmSettingWidget();
}

void ControlPageWidget::addGripperWidget(const std::vector<std::string>& gripper_names, const std::vector<std::vector<std::string>>& gripper_joint_names){
    if (gripper_names.size() < 0) return;
    QWidget* gripper_widget = initialControlGripperPage(kControlGripperPage, gripper_names, gripper_joint_names, this);
    layout_R_->insertWidget(layout_R_->count()-1, gripper_widget);
}

QWidget* ControlPageWidget::initialControlArmPage(const QString& name, const std::vector<int>& motor_size, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent){
    ArmWidget* w = new ArmWidget(safe_panel_command_, planner_state_shared_ptr_, motor_size, joint_limits, config);
    int arm_size = joint_limits.size();
    w->setMinimumWidth(300*arm_size);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name, w));
    return w;
}

QWidget* ControlPageWidget::initialControlGripperPage(const QString& name, const std::vector<std::string>& gripper_names, const std::vector<std::vector<std::string>>& gripper_joint_names, QWidget* parent){
    GripperWidget* w =  new GripperWidget(safe_panel_command_, planner_state_shared_ptr_, gripper_names, gripper_joint_names);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name, w));
    return w;
}

void ControlPageWidget::slotRecordingStateChanged(bool enable){
    ArmWidget* w = (ArmWidget*)control_widgets_[kControlArmPage];
    w->enableTab(ArmWidgetTab::kMotor, !enable);
}

void ControlPageWidget::slotResetPage(){
    ArmWidget* w = (ArmWidget*)control_widgets_[kControlArmPage];
    w->resetWidget();
}