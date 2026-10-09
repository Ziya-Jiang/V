#include "joint_control_widget.h"

#include <stdexcept>
#include <QDebug>

JointControlWidget::JointControlWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<Matrix2Xf>& joint_limits, const YAML::Node& config, QWidget* parent): QWidget(parent), safe_panel_command_(safe_panel_command), planner_state_shared_ptr_(ps){
    arm_size_ = joint_limits.size();
    for (int arm_i=0; arm_i<arm_size_; arm_i++)
        joint_size_.push_back(joint_limits[arm_i].cols());

    control_combox_ = new ComboxButton("Motion control:", kControlTypes.mid(0,3));
    orien_combox = new ComboxButton("Orientation frame:", kOrienTypes);
    connect(control_combox_, SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(slotControlComboxButtonClicked(QAbstractButton*)), Qt::DirectConnection);
    connect(orien_combox, SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(slotOrienComboxButtonClicked(QAbstractButton*))), Qt::DirectConnection;
    
    bool disable_null_control = joint_size_[0] < 7;
    if (config["panel"]["disable_display"] && config["panel"]["disable_display"]["control_tab-joint_tab-null_control"]){
        disable_null_control |= config["panel"]["disable_display"]["control_tab-joint_tab-null_control"].as<bool>();
    }
    control_combox_->setDisabled("Null", disable_null_control);


    QWidget* joint_w = new QWidget();
    QVBoxLayout* joint_layout = new QVBoxLayout(joint_w);
    QHBoxLayout* inner_joint_layout = new QHBoxLayout();
    joint_layout->setContentsMargins(0,0,0,0);
    inner_joint_layout->setContentsMargins(0,0,0,0);
    inner_joint_layout->setSpacing(15);
    joint_layout->addLayout(inner_joint_layout);

    QWidget* null_w = new QWidget();
    QVBoxLayout* null_layout = new QVBoxLayout(null_w);
    QHBoxLayout* inner_null_layout = new QHBoxLayout();
    null_layout->setContentsMargins(0,0,0,0);
    inner_null_layout->setContentsMargins(0,0,0,0);
    inner_null_layout->setSpacing(15);
    null_layout->addLayout(inner_null_layout);

    QWidget* task_w = new QWidget();
    QVBoxLayout* task_layout = new QVBoxLayout(task_w);
    QHBoxLayout* inner_task_layout = new QHBoxLayout();
    task_layout->setContentsMargins(0,0,0,0);
    inner_task_layout->setContentsMargins(0,0,0,0);
    inner_task_layout->setSpacing(15);
    task_layout->addWidget(orien_combox);
    task_layout->addLayout(inner_task_layout);

    joint_val_.reserve(arm_size_);
    null_val_.reserve(arm_size_);
    task_val_.reserve(arm_size_);
    joint_blocks_.reserve(arm_size_);
    null_blocks_.reserve(arm_size_);
    task_blocks_.reserve(arm_size_);
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        //Joint Control Widget
        QVBoxLayout* sub_joint_layout = new QVBoxLayout();
        sub_joint_layout->setContentsMargins(0,0,0,0);
        sub_joint_layout->setSpacing(10);
        std::vector<JointSliderBlock*> sub_joint_block;
        for (int i=0; i<joint_size_[arm_i]; i++){
            QString name = "J" + QString::number(i+1);
            JointSliderBlock* J = new JointSliderBlock(name);
            J->setLimits({joint_limits[arm_i](0,i), joint_limits[arm_i](1,i)});
            J->setStep(1);
            J->setSliderResol(1/0.0001);
            sub_joint_block.push_back(J);
            sub_joint_layout->addWidget(J);
        }
        joint_blocks_.push_back(sub_joint_block);
        inner_joint_layout->addLayout(sub_joint_layout);

        //Null Control Widget
        QVBoxLayout* sub_null_layout = new QVBoxLayout();
        sub_null_layout->setContentsMargins(0,0,0,0);
        sub_null_layout->setSpacing(10);
        QString name = "Angle";
        std::vector<JointSliderBlock*> sub_null_block;
        JointSliderBlock* J = new JointSliderBlock(name);
        J->setLimits({-180,180});
        J->setStep(1);
        J->setSliderResol(1/0.0001);
        sub_null_layout->addWidget(J);
        inner_null_layout->addLayout(sub_null_layout);
        sub_null_block.push_back(J);
        null_blocks_.push_back(sub_null_block);

        //Task Control Widget
        QVBoxLayout* sub_task_layout = new QVBoxLayout();
        sub_task_layout->setContentsMargins(0,0,0,0);
        sub_task_layout->setSpacing(10);
        float limit_L1[6] = {-1, -1, -1, -360, -360, -360};
        float limit_R1[6] = {1, 1, 1, 360, 360, 360};
        const QStringList kTaskControlTypes = {"X", "Y", "Z", "RX", "RY", "RZ"};
        std::vector<JointSliderBlock*> sub_task_block;
        for (int i=0; i<6; i++){
            QString name = kTaskControlTypes[i];
            JointSliderBlock* J = new JointSliderBlock(name);
            J->setLimits({limit_L1[i], limit_R1[i]});
            J->setSliderResol(1/0.0001);
            if (i < 3){
                J->setStep(0.01f);
            }else{
                J->setStep(1);
            }
            sub_task_layout->addWidget(J);
            sub_task_block.push_back(J);
        }
        inner_task_layout->addLayout(sub_task_layout);
        task_blocks_.push_back(sub_task_block);
        joint_val_.push_back(VectorXf::Zero(joint_size_[arm_i]));
        task_val_.push_back(VectorXf::Zero(joint_size_[arm_i]));

        if (arm_size_>1 && arm_i < arm_size_-1){
            std::vector<QHBoxLayout*> inner_layouts{inner_joint_layout, inner_null_layout, inner_task_layout};
            for (int i=0; i<inner_layouts.size(); i++){
                QFrame* vline = new QFrame();
                vline->setFrameShape(QFrame::VLine);
                vline->setLineWidth(1);
                inner_layouts[i]->addWidget(vline);
            }
        }
    }
    current_blocks_ = &joint_blocks_;

    //Add Speed Sliders
    SpeedSliderBlock* joint_speed = new SpeedSliderBlock("Speed");
    joint_speed->setInitialVal(1);
    joint_speed->addWidget(this);
    joint_layout->addWidget(joint_speed);

    SpeedSliderBlock* null_speed = new SpeedSliderBlock("Speed");
    null_speed->setInitialVal(1);
    null_speed->addWidget(this);
    null_layout->addStretch();
    null_layout->addWidget(null_speed);

    trans_speed_block_ = new SpeedSliderBlock("Trans\nSpeed");
    trans_speed_block_->setLimits({0,0.01f});
    trans_speed_block_->setSliderResol(1/0.0001);
    trans_speed_block_->setInitialVal(0.01f);
    trans_speed_block_->addWidget(this);
    ori_speed_block_ = new SpeedSliderBlock("Orien\nSpeed");
    ori_speed_block_->setInitialVal(1);
    ori_speed_block_->addWidget(this);
    task_layout->addWidget(trans_speed_block_);
    task_layout->addWidget(ori_speed_block_);
    
    //Add layouts to a stacklayout
    stack_layout_ = new QStackedLayout();
    stack_layout_->setContentsMargins(0,0,0,0);
    stack_layout_->addWidget(joint_w);
    stack_layout_->addWidget(null_w);
    stack_layout_->addWidget(task_w);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    layout->addWidget(control_combox_);
    layout->addLayout(stack_layout_);
}

void JointControlWidget::zero(){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (JointSliderBlock* j: (*current_blocks_)[arm_i]){
            j->zero();
        }
    }
}

void JointControlWidget::set(){
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.need_setting_update = true;
        command.reset_interpolation = true;
    });
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (JointSliderBlock* j: (*current_blocks_)[arm_i]){
            j->set();
        }
    }
}

void JointControlWidget::setStep(float step){
    if (motion_control_ == MotionControl::kTaskLine || motion_control_ == MotionControl::kTask){
        float trans_step = trans_speed_block_->getSliderVal();
        float orien_step = ori_speed_block_->getSliderVal();
        for (int arm_i=0; arm_i<arm_size_; arm_i++){
            for (size_t i=0; i<6; i++){
                JointSliderBlock* j = task_blocks_[arm_i][i];
                float step = (i < 3)? trans_step: orien_step;
                j->setStep(step);
            }
        }
    }else{
        for (int arm_i=0; arm_i<arm_size_; arm_i++){
            for (JointSliderBlock* j: (*current_blocks_)[arm_i]){
                j->setStep(step);
            }
        }
    }
}

void JointControlWidget::setSliderResol(float resol){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (JointSliderBlock* j: (*current_blocks_)[arm_i]){
            j->setSliderResol(resol);
        }
    }
}

void JointControlWidget::setInFront(bool in_front){ 
    //Always start with joint control to prevent accident move. For example route [task control-> replay -> gravity -> arm(result in task)] 
    motion_control_ = MotionControl::kJoint;
    control_combox_->setClick(kControlTypes[(int)motion_control_]);
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.motion_type = motion_control_;
        command.task_orien_type = orient_control_;
        command.reset_control_mem = true;
        command.need_setting_update = true;
    });
    updateJointTxt(); //Update Slider to current joint cmd
}

void JointControlWidget::updateJointTxt(){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (int i=0; i<joint_size_[arm_i]; i++){
            float val = (top_ == ControlType::kPosition)? planner_state_shared_ptr_->JointPos[arm_i][i] : 0;
            joint_blocks_[arm_i][i]->setInitialVal(val);
        }
    }
}

bool JointControlWidget::sendVal(){
    bool changed = false;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        changed = command.need_setting_update;
        if (motion_control_ == MotionControl::kJoint){
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                for (int i=0; i < joint_blocks_[arm_i].size(); i++){
                    command.JointCmdDeg[arm_i][i] = joint_blocks_[arm_i][i]->getTargetVal();
                    changed |= (command.JointCmdDeg[arm_i][i] != joint_val_[arm_i][i]);
                    joint_val_[arm_i][i] = command.JointCmdDeg[arm_i][i];
                    joint_blocks_[arm_i][i]->setSliderVal(planner_state_shared_ptr_->interpolated_target[arm_i][i]);
                }
            }
        }else if (motion_control_ == MotionControl::kNull){
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                command.JointCmdDeg[arm_i][0] = null_blocks_[arm_i][0]->getTargetVal();
                changed |= (command.JointCmdDeg[arm_i][0] != null_val_[arm_i]);
                null_val_[arm_i] = command.JointCmdDeg[arm_i][0];
                null_blocks_[arm_i][0]->setSliderVal(planner_state_shared_ptr_->interpolated_target[arm_i][0]);
            }
        }else{
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                for (int i=0; i < task_blocks_[arm_i].size(); i++){
                    command.TaskCmdDeg[arm_i][i] = task_blocks_[arm_i][i]->getTargetVal();
                    changed |= (command.TaskCmdDeg[arm_i][i] != task_val_[arm_i][i]);
                    task_val_[arm_i][i] = command.TaskCmdDeg[arm_i][i];
                    task_blocks_[arm_i][i]->setSliderVal(planner_state_shared_ptr_->interpolated_target[arm_i][i]);
                }
            }
        }
    });
    return changed;
}

MotionControl JointControlWidget::getCurrentMotionControl() const{
    return motion_control_;
}

void JointControlWidget::setMotionControlType(const std::string& motion, const std::string& orientation){
    QString m_str = QString::fromStdString(motion);
    QString o_str = QString::fromStdString(orientation);
    MotionControl m_c = MotionControl(kControlTypes.indexOf(m_str));
    OrientControl o_c = OrientControl(kOrienTypes.indexOf(o_str));
    if ((int)m_c == -1){
        qDebug()<< "Valid control types are" << kControlTypes<< "(case sensitive)";
        return;
    }
    if ((int)o_c == -1){
        qDebug()<< "Valid orentation types are" << kOrienTypes << "(case sensitive)";
        return;
    }
    if (motion_control_ != m_c)
        control_combox_->setClick(m_str);
    if (orient_control_ != o_c)
        orien_combox->setClick(o_str);
}

void JointControlWidget::setJointMotionTarget(const std::vector<VectorXf>& vals){
    if (vals.size() != arm_size_) throw std::runtime_error("setJointMotionTarget input with wrong arm size");
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (int i=0; i<joint_blocks_[arm_i].size(); i++){
            joint_blocks_[arm_i][i]->setTxtVal(vals[arm_i][i]);
            joint_blocks_[arm_i][i]->set();
        }
    }
}

void JointControlWidget::setTaskMotionTarget(const std::vector<Vector6f>& vals){
    if (vals.size() != arm_size_) throw std::runtime_error("setTaskMotionTarget input with wrong arm size");
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (int i=0; i<task_blocks_[arm_i].size(); i++){
            task_blocks_[arm_i][i]->setTxtVal(vals[arm_i][i]);
            task_blocks_[arm_i][i]->set();
        }
    }
}

void JointControlWidget::setControl(const QString& label){
    control_combox_->setClick(label);
}

// Motion Control type changed.
void JointControlWidget::slotControlComboxButtonClicked(QAbstractButton* bn){
    QString txt = bn->text();
    int i = kControlTypes.indexOf(txt);
    stack_layout_->setCurrentIndex(i);
    if (i == 0){
        motion_control_ = MotionControl::kJoint;
    }else if (i == 1){
        motion_control_ = MotionControl::kNull;
    }else if (i == 2){
        motion_control_ = MotionControl::kTaskLine;
    }else{
        throw std::runtime_error("Invalid motion control type");
    }

    switch (motion_control_){
        case MotionControl::kNull:
            current_blocks_ = &null_blocks_;
            zero();
            break;
        case MotionControl::kTaskLine:
            current_blocks_ = &task_blocks_;
            zero();
            break;
        case MotionControl::kTask:
            current_blocks_ = &task_blocks_;
            zero();
            break;
        default:
            current_blocks_ = &joint_blocks_;
            updateJointTxt();
            break;
    }
    
    // //Force Null & Task in pos-vel control
    // if (motion_control_ != MotionControl::kJoint){
    //     emit setControlType(ControlType::kPosition, ControlType::kVelocity);
    // }

    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.motion_type = motion_control_;
        command.need_setting_update = true;
    });
    emit armMotionTypeChanged(motion_control_);
}

void JointControlWidget::slotOrienComboxButtonClicked(QAbstractButton* bn){
    QString txt = bn->text();
    orient_control_ = (OrientControl)kOrienTypes.indexOf(txt);
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.task_orien_type = orient_control_;
        command.need_setting_update = true;
    });
}


void JointControlWidget::slotControlTypeChanged(ControlType top, ControlType base){
    top_ = top;
    updateJointTxt();
}

/* Without this function, slotControlTypeChanged will not be called if top&base keeps
the same after switching off the force control. e.g. We first move the robot using pos-vel
arm control, then we perform null admittnace(which is pos-vel). After we swtich off the
null admittance, the robot will go back to its original joint pos since the top&base layers did't
changed. */
void JointControlWidget::slotForceControlEnabled(bool enable, ControlType top, ControlType base){
    if (!enable){
        slotControlTypeChanged(top, base);
    }
}