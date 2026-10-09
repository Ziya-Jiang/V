#include "gripper_widget.h"
#include <QString>
#include <QDebug>

#include <QVBoxLayout>
#include <QFrame>

GripperWidget::GripperWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<std::string>& gripper_names, const std::vector<std::vector<std::string>>& gripper_joint_names, QWidget* parent):
                                            safe_panel_command_(safe_panel_command), planner_state_shared_ptr_(ps), QGroupBox(parent){
    this->setTitle("Gripper Control");
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(5,15,5,5);
    layout->setAlignment(Qt::AlignTop);
    gripper_size_ = gripper_names.size();
    float initial_step = 0.1f;
    for (int gri_i=0; gri_i < gripper_size_; gri_i++){
        QVBoxLayout* single_gripper_layout = new QVBoxLayout();
        single_gripper_layout->setContentsMargins(0,0,0,0);
        joint_size_.push_back(gripper_joint_names[gri_i].size());
        std::vector<JointSliderBlock*> single_gripper_joint_blocks;
        for (int joint_i=0; joint_i < joint_size_[gri_i]; joint_i++){
            QString name = QString::fromStdString(gripper_joint_names[gri_i][joint_i]);
            JointSliderBlock* bk = new JointSliderBlock(name, true);
            bk->setLimits({0, 1});
            bk->setInitialVal(0);
            bk->setStep(initial_step);
            bk->setSliderResol(1/0.01);
            bk->setLabelWidthRatio(0.26f);
            bk->setTxtWidthRatio(0.1f);
            single_gripper_layout->addWidget(bk);
            single_gripper_joint_blocks.push_back(bk);
        }
        gripper_joint_blocks_.push_back(single_gripper_joint_blocks);
        layout->addLayout(single_gripper_layout);
    }
    layout->addStretch();

    //Add Speed Sliders
    SpeedSliderBlock* slider_speed = new SpeedSliderBlock("Speed");
    slider_speed->setLimits({0,0.2f});
    slider_speed->setSliderResol(1/0.01);
    slider_speed->setInitialVal(initial_step);
    slider_speed->addWidget(this);
    layout->addWidget(slider_speed);

    //Add Zero Set Block
    ZeroSetBlock* zero_set = new ZeroSetBlock();
    zero_set->addWidget(this);
    layout->addWidget(zero_set);
}

void GripperWidget::zero(){
    for (int gri_i=0; gri_i < gripper_size_; gri_i++){
        for (int joint_i=0; joint_i < joint_size_[gri_i]; joint_i++){
            gripper_joint_blocks_[gri_i][joint_i]->zero();
        }
    }
}

void GripperWidget::set(){
    for (int gri_i=0; gri_i < gripper_size_; gri_i++){
        for (int joint_i=0; joint_i < joint_size_[gri_i]; joint_i++){
            gripper_joint_blocks_[gri_i][joint_i]->set();
        }
    }
}

void GripperWidget::setStep(float step){
    for (int gri_i=0; gri_i < gripper_size_; gri_i++){
        for (int joint_i=0; joint_i < joint_size_[gri_i]; joint_i++){
            gripper_joint_blocks_[gri_i][joint_i]->setStep(step);
        }
    }
}

void GripperWidget::setSliderResol(float resol){
    for (int gri_i=0; gri_i < gripper_size_; gri_i++){
        for (int joint_i=0; joint_i < joint_size_[gri_i]; joint_i++){
            gripper_joint_blocks_[gri_i][joint_i]->setSliderResol(resol);
        }
    }
}

void GripperWidget::setInFront(bool in_front){
    if (in_front){
        for (int gri_i=0; gri_i < gripper_size_; gri_i++){
            for (int joint_i=0; joint_i < joint_size_[gri_i]; joint_i++){
                gripper_joint_blocks_[gri_i][joint_i]->setInitialVal(planner_state_shared_ptr_->GripperJointPos[gri_i][joint_i]);
            }
        }
    }
}

bool GripperWidget::sendVal(){
    bool changed = false;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        changed = command.need_setting_update;
        for (int gri_i=0; gri_i < gripper_size_; gri_i++){
            for (int joint_i=0; joint_i < joint_size_[gri_i]; joint_i++){
                float val = gripper_joint_blocks_[gri_i][joint_i]->getTargetVal();
                gripper_joint_blocks_[gri_i][joint_i]->setSliderVal(val);
                changed |= (command.GripperJointCmd[gri_i][joint_i] != val);
                command.GripperJointCmd[gri_i][joint_i] = val;
            }
        }
    });
    return changed;
}