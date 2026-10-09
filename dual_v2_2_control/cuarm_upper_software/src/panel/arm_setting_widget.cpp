#include "arm_setting_widget.h"

#include <math.h>
#include "boost/variant.hpp"
#include <QDebug>
#include <QLabel>
#include <QScrollArea>
#include <QMap>
#include <map>

const QString kInterpolationLabel =         "Interpolation: ";
const QString kMotorTopLayerLabel =         "Target Type: ";
const QString kMotorBaseLayerLabel =        "Actuator Mode: ";
const QString kAccTLabel =                  "Acc. time(s): ";
const QString kVelTLabel =                  "Constant Vel. time(s): ";
const QString kSaturationRatioLabel =       "Saturation ratio: ";
const QString kGravityLabel =               "Gravity Compensation: ";
const QString kGravityWithBoundaryLabel =   "Gravity With Boundary: ";
const QString kTaskAdmittanceLabel =        "Task Admittance Control: ";
const QString kJointAdmittanceLabel =       "Joint Admittance Control: ";
const QString kTaskImpedanceLable =         "Task Impedance Control: ";
const QString kJointImpedanceLabel =        "Joint Impedance Control: ";
const QString kNullAdmittanceLabel =        "Null Admittance Control: ";


//////////////////////////////////ArmSettingWidget////////////////////////////////////////
ArmSettingWidget::ArmSettingWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const YAML::Node& config, QWidget* parent): safe_panel_command_(safe_panel_command), planner_state_shared_ptr_(ps), FormWidget(parent){    
    QStringList interpolation_labels = toQStringList(getAllEnumNames<InterpolationMethod>());
    // interpolation_labels.removeOne("QuinticPath");
    QStringList motor_labels = toQStringList(getAllEnumNames<ControlType>());
    // motor_labels.removeOne("Invalid");

    this->enableScroll(true, Qt::ScrollBarAlwaysOff, Qt::ScrollBarAsNeeded);
    addComboRow(kInterpolationLabel, interpolation_labels);
    setComboxWidgetItemVisible(kInterpolationLabel, (int)InterpolationMethod::kQuinticPath, false);
    addEditorRow(kAccTLabel);
    addEditorRow(kVelTLabel);
    this->setWidgetWidth(kAccTLabel, 80);
    this->setWidgetWidth(kVelTLabel, 80);
    addEditorRow(kSaturationRatioLabel);
    this->setWidgetWidth(kSaturationRatioLabel, 80);
    this->setRowVisible(kSaturationRatioLabel, false); // Hide max vel ratio for now
    addComboRow(kMotorTopLayerLabel, motor_labels);
    addComboRow(kMotorBaseLayerLabel, motor_labels);
    int invalid_index = motor_labels.indexOf(enumToString<ControlType>(ControlType::kInvalid));
    setComboxWidgetItemVisible(kMotorTopLayerLabel, invalid_index, false);
    setComboxWidgetItemVisible(kMotorBaseLayerLabel, invalid_index, false);

    //Force control checkbox
    control_algo_map_[kGravityLabel]             = ControlAlgoStruct{ControlAlgorithm::kGravity,             ControlType::kTorque,   ControlType::kTorque,   false};
    control_algo_map_[kGravityWithBoundaryLabel] = ControlAlgoStruct{ControlAlgorithm::kGravityWithBoundary, ControlType::kTorque,   ControlType::kTorque,   false};
    control_algo_map_[kTaskAdmittanceLabel]      = ControlAlgoStruct{ControlAlgorithm::kTaskAdmittance,      ControlType::kVelocity, ControlType::kVelocity, false};
    control_algo_map_[kJointAdmittanceLabel]     = ControlAlgoStruct{ControlAlgorithm::kJointAdmittance,     ControlType::kVelocity, ControlType::kVelocity, false};
    control_algo_map_[kTaskImpedanceLable]       = ControlAlgoStruct{ControlAlgorithm::kTaskImpedance,       ControlType::kTorque,   ControlType::kTorque,   false};
    control_algo_map_[kJointImpedanceLabel]      = ControlAlgoStruct{ControlAlgorithm::kJointImpedance,      ControlType::kTorque,   ControlType::kTorque,   false};
    control_algo_map_[kNullAdmittanceLabel]      = ControlAlgoStruct{ControlAlgorithm::kNullAdmittance,      ControlType::kPosition, ControlType::kVelocity, false};
    // control_algo_insert_order_ = {kGravityLabel, kGravityWithBoundaryLabel, kTaskAdmittanceLabel, kJointAdmittanceLabel, kTaskImpedanceLable, kJointImpedanceLabel,
    //                                 kNullAdmittanceLabel};
    control_algo_insert_order_ = {kGravityLabel};
    for (QString label: control_algo_insert_order_){
        addCheckBoxRow(label);
    }
    enableExclusiveButtons(true); // Make checkboxes exclusive
    
    if (config["panel"]["disable_display"] && config["panel"]["disable_display"]["control_tab-setting_tab-force_control-all"]){
        bool disable_all_control_algo = config["panel"]["disable_display"]["control_tab-setting_tab-force_control-all"].as<bool>();
        enableButtonGroup(!disable_all_control_algo);
    }

    //Connect singals and slots
    connect(this, SIGNAL(editorTextChanged(const QString&, const QString&)), this, SLOT(slotEditorTextChanged(const QString&, const QString&)), Qt::DirectConnection);
    connect(this, SIGNAL(comboCurrentTextChanged(const QString&, const QString&)), this, SLOT(slotComboCurrentTextChanged(const QString&, const QString&)), Qt::DirectConnection);
    connect(this, SIGNAL(checkBoxStateChanged(const QString&, int)), this, SLOT(slotCheckBoxStateChanged(const QString&, int)), Qt::DirectConnection);

    //Initial widget values
    YAML::Node general_config = config["panel"]["general"];
    interpolation_method_ = stringToEnum<InterpolationMethod>(general_config["interpolation"].as<std::string>());
    target_type_ = stringToEnum<ControlType>(general_config["target_type"].as<std::string>());
    actuator_mode_ = stringToEnum<ControlType>(general_config["actuator_mode"].as<std::string>());
    acc_t_ =  general_config["acc_time"].as<float>();
    vel_t_ =  general_config["vel_time"].as<float>();
    sat_ratio_ =  general_config["none_interpolation_saturation_adjust"].as<float>();
    
    setAllValue();
}

void ArmSettingWidget::setInFront(bool in_front){
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.motion_type = MotionControl::kJoint;
        command.need_setting_update = true;
    });
}

bool ArmSettingWidget::sendVal(){
    bool changed = false;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        changed = command.need_setting_update;
        if (command.target_type == ControlType::kPosition){
            for (int arm_i=0; arm_i < command.ArmSize; arm_i++){
                for (int j=0; j<command.JointSize[arm_i]; j++){
                    command.JointCmdDeg[arm_i][j] = planner_state_shared_ptr_->JointPos[arm_i][j];
                    command.MotorCmdDeg[arm_i][j] = planner_state_shared_ptr_->MotorPos[arm_i][j];
                }
            }
        }else{
            for (int arm_i=0; arm_i < command.ArmSize; arm_i++){
                for (int j=0; j<command.JointSize[arm_i]; j++){
                    command.JointCmdDeg[arm_i][j] = 0;
                    command.MotorCmdDeg[arm_i][j] = 0;
                }
            }
        }
    });
    return changed;
}

void ArmSettingWidget::setControlCheckBox(bool gravity, bool task_admittance, bool joint_admittance, bool task_impedance, bool joint_impedance, \
                                            bool null_admittance, bool task_impe_PI, bool task_impe_two_phase_pid, bool task_impe_two_phase_vel){
    // gravity_ = gravity;
    // task_admittance_ = task_admittance;
    // joint_admittance_ = joint_admittance;
    // task_impedance_ = task_impedance;
    // joint_impedance_ = joint_impedance;
    // null_admittance_ = null_admittance;
    // task_impe_PI_ = task_impe_PI;
    // task_impe_two_phase_pid_ = task_impe_two_phase_pid;
    // task_impe_two_phase_vel_ = task_impe_two_phase_vel;
    // setAllValue();
}

void ArmSettingWidget::setValues( const std::string& interpolation, const std::string& top, const std::string& base,\
                                    float acc_time, float vel_time, float sat_ratio){
    interpolation_method_ = stringToEnum<InterpolationMethod>(interpolation);
    acc_t_ = acc_time;
    vel_t_ = vel_time;
    sat_ratio_ = sat_ratio;
    if (!control_layer_disabled_){
        target_type_ = stringToEnum<ControlType>(top);
        actuator_mode_ = stringToEnum<ControlType>(base);
    }
    setAllValue();
}

void ArmSettingWidget::setAllValue(){
    int interpolation = (int)interpolation_method_;
    int top = (int)target_type_ + 1; //First is invalid
    int base = (int)actuator_mode_ + 1; //First is invalid
    QString acc =  QString::number(acc_t_);
    QString vel =  QString::number(vel_t_);
    QString sat_ratio =  QString::number(sat_ratio_);

    FormVariantMap v;
    v.insert({kInterpolationLabel, interpolation});
    v.insert({kAccTLabel, acc});
    v.insert({kVelTLabel, vel});
    v.insert({kSaturationRatioLabel, sat_ratio});
    v.insert({kMotorTopLayerLabel, top});
    v.insert({kMotorBaseLayerLabel, base});
    for (QString label: control_algo_insert_order_){
        v.insert({label, control_algo_map_[label].enable});
    }
    FormWidget::setAllValue(v);
}

void ArmSettingWidget::getControlType(ControlType& top, ControlType& base) const{
    top = target_type_;
    base = actuator_mode_;
}

void ArmSettingWidget::setControlType(const ControlType& top, const ControlType& base){
    target_type_  = top;
    actuator_mode_ = base;
    setAllValue();
}

void ArmSettingWidget::emitAllSignals(){
    ControlAlgorithm enabled_control_algorithm = ControlAlgorithm::kNone;
    for (const std::pair<QString, ControlAlgoStruct>& p: control_algo_map_){
        if (p.second.enable){
            enabled_control_algorithm = p.second.control_algorithm_type;
            break;
        }
    }
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        if (enabled_control_algorithm != ControlAlgorithm::kNone)
            command.motion_type = MotionControl::kControlAlgorithm;
        command.control_algorithm = enabled_control_algorithm;
        command.need_setting_update = true;
    });


    emit interpolationMethodChanged(interpolation_method_);
    emit interpolationTimeChanged(acc_t_, vel_t_);
    emit controlTypeChanged(target_type_, actuator_mode_);
    emit forceControlEnabled(false, target_type_, actuator_mode_);
}

void ArmSettingWidget::updateBaseLayerSelection(){
    bool pos_enable = (target_type_ == ControlType::kPosition)? true : false;
    bool vel_enable = (target_type_ != ControlType::kTorque)? true : false;
    enableComboxWidgetItem(kMotorBaseLayerLabel, 0, pos_enable);
    enableComboxWidgetItem(kMotorBaseLayerLabel, 1, vel_enable);
    if (target_type_ == ControlType::kVelocity && actuator_mode_ == ControlType::kPosition){ // position base layer is not allowed at Velocity top layer control
        setComboxWidgetSelectedItem(kMotorBaseLayerLabel, 1); //Set to Velocity 
        actuator_mode_ = ControlType::kVelocity;
        setAllValue();
    }else if (target_type_ == ControlType::kTorque && actuator_mode_ != ControlType::kTorque){ // only torque base layer is allowed at torque top layer control
        setComboxWidgetSelectedItem(kMotorBaseLayerLabel, 2); //Set to Torque 
        actuator_mode_ = ControlType::kTorque;
        setAllValue();
    }
}

void ArmSettingWidget::resetForceControl(){
    for (QString label : control_algo_insert_order_){
        this->setWidgetChecked(label, false);
    }
}

void ArmSettingWidget::slotEditorTextChanged(const QString& label, const QString& text){
    float val = text.toFloat();
    if (label == kAccTLabel){
        if (val == 0){
            std::cout << "Acceleration time must be larger than 0." << std::endl;
            return;
        }
        acc_t_ = val;
    }else if (label == kVelTLabel){
        vel_t_ = val;
    }else if (label == kSaturationRatioLabel){
        if (val > 1.0f || val < 0.0f){
            std::cout << "Max ratio must be <= 1.0 and >= 0" << std::endl;
        }
        sat_ratio_ = std::clamp(val, 0.0f, 1.0f);
    }
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.InterpolationAccTime = acc_t_;
        command.InterpolationConstVelTime = vel_t_;
        command.NoneInterpolationSaturationRatio = sat_ratio_;
        command.need_setting_update = true;
    });
    
    emit interpolationTimeChanged(acc_t_, vel_t_);
}

void ArmSettingWidget::slotComboCurrentTextChanged(const QString& label, const QString& text){
    if (label == kInterpolationLabel){
        interpolation_method_ = stringToEnum<InterpolationMethod>(text.toStdString());
        safe_panel_command_.updateCommand([&](PanelCommand& command){
            command.interpolation_type = interpolation_method_;
            command.need_setting_update = true;
        });
        emit interpolationMethodChanged(interpolation_method_);
        if (interpolation_method_ == InterpolationMethod::kNone){
            this->setRowVisible(kAccTLabel, false);
            this->setRowVisible(kVelTLabel, false);
            this->setRowVisible(kSaturationRatioLabel, true);
        }else{
            this->setRowVisible(kAccTLabel, true);
            this->setRowVisible(kVelTLabel, true);
            this->setRowVisible(kSaturationRatioLabel, false);
        }
    }else{
        if (label == kMotorTopLayerLabel){
            target_type_ = stringToEnum<ControlType>(text.toStdString());
        }else{
            actuator_mode_ = stringToEnum<ControlType>(text.toStdString());
        }
        updateBaseLayerSelection();
        safe_panel_command_.updateCommand([&](PanelCommand& command){
            command.target_type = target_type_;
            command.actuator_mode = actuator_mode_;
            command.need_setting_update = true;
        });
        emit controlTypeChanged(target_type_, actuator_mode_);
    }
}

void ArmSettingWidget::slotCheckBoxStateChanged(const QString& label, int state){
    bool enable = (state == Qt::CheckState::Checked)? true:false;
    ControlAlgoStruct& fc = control_algo_map_[label];
    if (enable){        //Change top/base control layer.
        last_target_type_ = target_type_;
        last_actuator_mode_ = actuator_mode_;
        target_type_ = fc.target_type;
        actuator_mode_ = fc.actuator_mode;
    }else{              //Restore original control
        target_type_ = last_target_type_;
        actuator_mode_ = last_actuator_mode_;
    }

    fc.enable = enable; 
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.control_algorithm = (!enable)? ControlAlgorithm::kNone : fc.control_algorithm_type;
        command.motion_type = (!enable)? MotionControl::kJoint : MotionControl::kControlAlgorithm;
        command.need_setting_update = true;
    });
    emit forceControlEnabled(enable, target_type_, actuator_mode_);
    setAllValue();
    this->setRowDisable(kMotorTopLayerLabel, enable);
    this->setRowDisable(kMotorBaseLayerLabel, enable);
    control_layer_disabled_ = enable;
}

void ArmSettingWidget::slotSetControlType(ControlType top, ControlType base){
    if (!control_layer_disabled_){
        target_type_ = top;
        actuator_mode_ = base;
        setAllValue();
    }
}

QStringList ArmSettingWidget::toQStringList(const std::vector<std::string>& vec) {
    QStringList list;
    list.reserve(static_cast<qsizetype>(vec.size()));
    for (const auto& s : vec) {
        list << QString::fromStdString(s);
    }
    return list;
}