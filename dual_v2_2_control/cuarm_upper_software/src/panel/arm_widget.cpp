#include "arm_widget.h"

#include <algorithm>    // std::find

#include <QVBoxLayout>
#include <QScrollArea>
#include <QString>
#include <QStringList>

#include "slider_block.h"
#include "zero_set_block.h"
#include "motor_control_widget.h"
#include "joint_control_widget.h"
#include "arm_setting_widget.h"

const QString kMotorPage   = "Motor";
const QString kJointPage   = "Joint";
const QString kSettingPage = "Setting";
// const  QStringList kTabOrder {kMotorPage, kJointPage, kSettingPage};
const  QStringList kTabOrder {kJointPage, kSettingPage};
const int total_page = kTabOrder.size();

ArmWidget::ArmWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<int>& motor_size, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent) : QTabWidget(parent){
    connect(this, &QTabWidget::currentChanged, this, &ArmWidget::slotTabCurrentChanged);
    // this->addTab(initialControlArmMotorPage(kMotorPage, pan_cmd, pla_state, motor_size, this), kMotorPage);
    this->addTab(initialControlArmJointPage(kJointPage, safe_panel_command, pla_state, joint_limits, config, this), kJointPage); 
    this->addTab(initialControlArmSettingPage(kSettingPage, safe_panel_command, pla_state, config, this), kSettingPage); 
    // current_tab_name_ = kMotorPage;
    current_tab_name_ = kJointPage;
    safe_panel_command.updateCommand([&](PanelCommand& command){
        command.motion_type = MotionControl::kJoint;
    });
    
    // if (config["panel"]["disable_display"] && config["panel"]["disable_display"]["control_tab-motor_tab"]){
    //     motor_page_disabled_ = config["panel"]["disable_display"]["control_tab-motor_tab"].as<bool>();
    //     this->enableTab(ArmWidgetTab::kMotor, !motor_page_disabled_);
    // }

    JointControlWidget* joint_w = (JointControlWidget*)widgets_[kJointPage];
    // MotorControlWidget* motor_w = (MotorControlWidget*)widgets_[kMotorPage];
    ArmSettingWidget* arm_setting = (ArmSettingWidget*)widgets_[kSettingPage];
    // connect(arm_setting, SIGNAL(controlTypeChanged(ControlType, ControlType)), motor_w, SLOT(slotControlTypeChanged(ControlType, ControlType)));
    connect(arm_setting, SIGNAL(controlTypeChanged(ControlType, ControlType)), joint_w, SLOT(slotControlTypeChanged(ControlType, ControlType)));
    // connect(arm_setting, SIGNAL(forceControlEnabled(bool, ControlType, ControlType)), motor_w, SLOT(slotForceControlEnabled(bool, ControlType, ControlType)));
    connect(arm_setting, SIGNAL(forceControlEnabled(bool, ControlType, ControlType)), joint_w, SLOT(slotForceControlEnabled(bool, ControlType, ControlType)));
    connect(arm_setting, SIGNAL(forceControlEnabled(bool, ControlType, ControlType)), this, SLOT(slotForceControlEnabled(bool, ControlType, ControlType)));
    connect(joint_w, SIGNAL(setControlType(ControlType, ControlType)), arm_setting, SLOT(slotSetControlType(ControlType, ControlType)));
}

void ArmWidget::setInFront(bool in_front){
    if (!in_front){
        if (control_widgets_[current_tab_name_])
            control_widgets_[current_tab_name_]->setInFront(in_front);
    }else{ //Set to joint page except it is under force control
        current_tab_name_ = (control_algorithm_enabled_)? kSettingPage : kJointPage;
        this->setCurrentIndex(kTabOrder.indexOf(current_tab_name_));
        if (control_widgets_[current_tab_name_]) //the slotTabCurrentChanged will not be called when the same tab is clicked, so we need to setInFront manually
            control_widgets_[current_tab_name_]->setInFront(in_front);
    }
}

bool ArmWidget::sendVal(){
    if (control_widgets_[current_tab_name_])
        return control_widgets_[current_tab_name_]->sendVal();
    return false;
}

ArmSettingWidget* ArmWidget::getArmSettingWidget(){
    return (ArmSettingWidget*)widgets_[kSettingPage];
}

void ArmWidget::enableTab(const ArmWidgetTab& tab, bool enable){
    QString tab_name;
    switch(tab){
        case ArmWidgetTab::kMotor:
            tab_name = kMotorPage;
            break;
        case ArmWidgetTab::kJoint:  
            tab_name = kJointPage;
            break;
        case ArmWidgetTab::kSetting:
            tab_name = kSettingPage;
            break;
    }
    this->setTabEnabled(kTabOrder.indexOf(tab_name), enable);
}

QWidget* ArmWidget::initialControlArmMotorPage(const QString& name, ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<int>& motor_size, QWidget* parent){
    QWidget* page = new QWidget(parent);
    QWidget* content = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(content);
    layout->setContentsMargins(10,20,10,10);

    SpeedSliderBlock* speed = new SpeedSliderBlock("Speed");
    ZeroSetBlock* zero_set = new ZeroSetBlock();
    MotorControlWidget* motorController = new MotorControlWidget(safe_panel_command, pla_state, motor_size, this);
    speed->addWidget(motorController);
    zero_set->addWidget(motorController);
    layout->addWidget(motorController);
    layout->addStretch();
    layout->addWidget(speed);
    layout->addWidget(zero_set);
    
    //Make it scrollable
    QVBoxLayout* mainLayout = new QVBoxLayout(page);
    QScrollArea* scrollArea = new QScrollArea();
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidget(content);
    mainLayout->addWidget(scrollArea);
    mainLayout->setContentsMargins(0,0,0,0);
    widgets_.insert(std::pair<QString, QWidget*>(name, motorController));
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name, motorController));
    return page;
}

QWidget* ArmWidget::initialControlArmJointPage(const QString& name, ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const std::vector<Matrix2Xf> joint_limits, const YAML::Node& config, QWidget* parent){
    QWidget* page = new QWidget(parent);
    QWidget* content = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(content);
    ZeroSetBlock* zero_set = new ZeroSetBlock();
    JointControlWidget* w = new JointControlWidget(safe_panel_command, pla_state, joint_limits, config);
    zero_set->addWidget(w);
    layout->addWidget(w);
    layout->addWidget(zero_set);
    layout->setContentsMargins(10,20,10,10);
    
    //Make it scrollable
    QVBoxLayout* mainLayout = new QVBoxLayout(page);
    QScrollArea* scrollArea = new QScrollArea();
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidget(content);
    mainLayout->addWidget(scrollArea);
    mainLayout->setContentsMargins(0,0,0,0);
    widgets_.insert(std::pair<QString, QWidget*>(name, w));
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name, w));
    return page;
}

QWidget* ArmWidget::initialControlArmSettingPage(const QString& name, ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& pla_state, const YAML::Node& config, QWidget* parent){
    QWidget* page = new QWidget(parent);
    QVBoxLayout* layout = new QVBoxLayout(page);
    ArmSettingWidget* arm_setter = new ArmSettingWidget(safe_panel_command, pla_state, config);
    layout->addWidget(arm_setter);
    layout->addStretch();
    layout->setAlignment(Qt::AlignTop);
    layout->setContentsMargins(10,10,10,10); 
    widgets_.insert(std::pair<QString, QWidget*>(name, arm_setter));
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name, arm_setter));
    return page;
}

void ArmWidget::resetWidget(){
    // uncheck force control
    ArmSettingWidget* arm_setting = (ArmSettingWidget*)widgets_[kSettingPage];
    arm_setting->resetForceControl();

    //Switch to Joint Page, which would update joint txt
    if (current_tab_name_ != kJointPage){
        int i = kTabOrder.indexOf(kJointPage);
        this->setCurrentIndex(i);
    }else{
        //Already in Joint Page, update joint txt explicitly
        JointControlWidget* joint_w = (JointControlWidget*)widgets_[kJointPage];
        joint_w->updateJointTxt();
    }
}

//Called whenever current tab is changed.
void ArmWidget::slotTabCurrentChanged(int i){
    if (control_widgets_.size() < total_page) return;

    //Disable previous tab
    if (control_widgets_[current_tab_name_])
        control_widgets_[current_tab_name_]->setInFront(false);

    //Enable new tab
    current_tab_name_ = kTabOrder[i];
    if (control_widgets_[kTabOrder[i]])
        control_widgets_[current_tab_name_]->setInFront(true);
}

void ArmWidget::slotForceControlEnabled(bool enable, ControlType top, ControlType base){
    control_algorithm_enabled_ = enable;
    enableTab(ArmWidgetTab::kMotor, !control_algorithm_enabled_ && !motor_page_disabled_);
    enableTab(ArmWidgetTab::kJoint, !control_algorithm_enabled_);
}


