#include "motor_control_widget.h"
#include <QString>
#include <QDebug>
#include <QFrame>
MotorControlWidget::MotorControlWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& motor_size, QWidget* parent): QWidget(parent), safe_panel_command_(safe_panel_command), planner_state_shared_ptr_(ps), motor_size_(motor_size){
    layout_ = new QVBoxLayout(this);
    layout_->setContentsMargins(0,0,0,0);
    
    QHBoxLayout* motor_layout = new QHBoxLayout();
    motor_layout->setContentsMargins(0,0,0,0);
    motor_layout->setSpacing(15);
    arm_size_ = motor_size.size();
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        QVBoxLayout* sub_layout = new QVBoxLayout();
        sub_layout->setAlignment(Qt::AlignTop);
        sub_layout->setContentsMargins(0,0,0,0);
        sub_layout->setSpacing(10);
        motor_val_.push_back(VectorXf::Zero(motor_size_[arm_i]));
        std::vector<JointSliderBlock*> sub_motor_block;
        for (int i=0; i<motor_size_[arm_i]; i++){
            QString name = QString::fromStdString("M" + std::to_string(i+1));
            JointSliderBlock* mb = new JointSliderBlock(name);
            mb->setLimits({-slider_limit_, slider_limit_});
            sub_motor_block.push_back(mb);
            sub_layout->addWidget(mb);
        }
        motor_layout->addLayout(sub_layout);
        motor_blocks_.push_back(sub_motor_block);

        if (arm_size_>1 && arm_i < arm_size_-1){
            QFrame* vline = new QFrame();
            vline->setFrameShape(QFrame::VLine);
            vline->setLineWidth(1);
            motor_layout->addWidget(vline);
        }
    }
    layout_->addLayout(motor_layout);
}

void MotorControlWidget::zero(){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (JointSliderBlock* mb: motor_blocks_[arm_i]){
            mb->zero();
        }
    }
}

void MotorControlWidget::set(){
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.need_setting_update = true;
        command.reset_interpolation = true;
    });
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (JointSliderBlock* mb: motor_blocks_[arm_i]){
            mb->set();
        }
    }
}

void MotorControlWidget::setStep(float step){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (JointSliderBlock* mb: motor_blocks_[arm_i]){
            mb->setStep(step);
        }
    }
}

void MotorControlWidget::setSliderResol(float resol){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (JointSliderBlock* mb: motor_blocks_[arm_i]){
            mb->setSliderResol(resol);
        }
    }
}

void MotorControlWidget::setInFront(bool in_front){
    in_front_ = in_front;
    if (in_front){
        safe_panel_command_.updateCommand([&](PanelCommand& command){
            command.motion_type = MotionControl::kMotor;
            command.need_setting_update = true;
        });
    }
    updateMotorTxt();
}

void MotorControlWidget::updateMotorTxt(){
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (int i=0; i<motor_size_[arm_i]; i++){
            float val = (top_ == ControlType::kPosition)? planner_state_shared_ptr_->MotorPos[arm_i][i] : 0;
            motor_blocks_[arm_i][i]->setInitialVal(val);
        }
    }
}

bool MotorControlWidget::sendVal(){
    bool changed = false;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        changed = command.need_setting_update;
        for (int arm_i=0; arm_i<arm_size_; arm_i++){
            for (int i=0; i<motor_size_[arm_i]; i++){
                motor_blocks_[arm_i][i]->setSliderVal(planner_state_shared_ptr_->interpolated_target[arm_i][i]);
                command.MotorCmdDeg[arm_i][i] = motor_blocks_[arm_i][i]->getTargetVal();
                changed |= (command.MotorCmdDeg[arm_i][i] != motor_val_[arm_i][i]);
                motor_val_[arm_i][i] = command.MotorCmdDeg[arm_i][i];
            }
        }
    });
    return changed;
}

void MotorControlWidget::slotControlTypeChanged(ControlType top, ControlType base){
    top_ = top;
    base_ = base;
    for (int arm_i=0; arm_i<arm_size_; arm_i++){
        for (int i=0; i<motor_size_[arm_i]; i++){
            JointSliderBlock* block = motor_blocks_[arm_i][i];
            float val;
            if (top == ControlType::kPosition){
                val = planner_state_shared_ptr_->MotorPos[arm_i][i];
                if (in_front_ && abs(val) > slider_limit_){
                    QString title = "Excess Slider Limit";
                    QString msg =   "<b>Arm " + QString::number(arm_i) + " " + block->getName() + "</b>: current position excess slider limits!" +
                                    "<br>cur_pos: " + QString::number(val) + 
                                    "<br>slider_limit: " + QString::number(slider_limit_);
                }
            }else{
                val = 0;
            }
            block->setInitialVal(val);
        }
    }
}

void MotorControlWidget::slotForceControlEnabled(bool enable, ControlType top, ControlType base){
    if (!enable){
        slotControlTypeChanged(top, base);
    }
}