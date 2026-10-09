#include "record_trajectory_widget.h"

#include <iostream>
#include <fstream>
#include <string>

#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>

const QString kSaveCartesian = "Save End-effector Cartesian Pose";
const QStringList kOptionList = {kSaveCartesian};

RecordTrajectoryWidget::RecordTrajectoryWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& joint_size, const std::vector<int>& gripper_joint_size, 
    const std::string& config_prefix_path, ControlType actuator_mode, QWidget* parent): RecordAndReplayBase(safe_panel_command, ps, joint_size, 
    gripper_joint_size, config_prefix_path){
    actuator_mode_ = (actuator_mode != ControlType::kInvalid)? actuator_mode : ControlType::kPosition;

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setAlignment(Qt::AlignTop);

    QLabel* label = new QLabel("Record New Trajectory");

    QHBoxLayout* file_layout = new QHBoxLayout();
    QLabel* save_to_label = new QLabel("Save To: ");
    file_line_edit_ = new QLineEdit("");
    file_line_edit_ -> setAlignment(Qt::AlignRight);
    connect(file_line_edit_, SIGNAL(textChanged(const QString&)), this, SLOT(slotFileLineTextChanged(const QString&)));

    QPushButton* file_button = new QPushButton("Select");
    QFileDialog* dialog = new QFileDialog(this);
    connect(file_button, &QPushButton::clicked, this, [this, dialog](){slotSaveFileClicked(dialog);});
    dialog->setFileMode(QFileDialog::AnyFile);
    file_layout->addWidget(save_to_label);
    file_layout->addWidget(file_line_edit_);
    file_layout->addWidget(file_button);

    QHBoxLayout* buttons_layout = new QHBoxLayout();
    start_button_  = new QPushButton("Start");
    savePt_button_ = new QPushButton("Take Point");
    finish_button_ = new QPushButton("Finish");
    cancel_button_ = new QPushButton("Cancel");
    connect(start_button_, SIGNAL(clicked(bool)), this, SLOT(slotStartButtonClicked()));
    connect(savePt_button_, SIGNAL(clicked(bool)), this, SLOT(slotSavePointButtonClicked()));
    connect(finish_button_, SIGNAL(clicked(bool)), this, SLOT(slotFinishButtonClicked()));
    connect(cancel_button_, SIGNAL(clicked(bool)), this, SLOT(slotCancelButtonClicked()));
    buttons_layout->addWidget(start_button_);
    buttons_layout->addWidget(savePt_button_);
    buttons_layout->addWidget(finish_button_);
    buttons_layout->addWidget(cancel_button_);

    QHBoxLayout* data_layout = new QHBoxLayout();
    QLabel* last_pt_label = new QLabel("Last saved point: ");
    last_pt_ = new QLabel("None");
    QLabel* total_pt_label = new QLabel("Total points: ");
    total_pt_ = new QLabel("0");
    const QString secondary_text_style =
        "color: #8A8178; background-color: transparent; font-size: 9pt;";
    last_pt_label->setStyleSheet(secondary_text_style);
    last_pt_->setStyleSheet(secondary_text_style);
    total_pt_label->setStyleSheet(secondary_text_style);
    total_pt_->setStyleSheet(secondary_text_style);

    total_pt_->setFixedWidth(50);
    data_layout->addWidget(last_pt_label);
    data_layout->addWidget(last_pt_);
    data_layout->addStretch();
    data_layout->addWidget(total_pt_label);
    data_layout->addWidget(total_pt_);

    //Options
    option_form_w_ = new FormWidget();
    option_form_w_->setContentsMargins(0, 0, 0, 0);
    for (const QString& option : kOptionList){
        option_form_w_->addCheckBoxRow(option, false);
        option_map_[option] = false;
        connect(option_form_w_, SIGNAL(checkBoxStateChanged(const QString&, int)), this, SLOT(slotOptionFormCheckBoxChanged(const QString&, int)));
    }

    layout->addWidget(label);
    layout->addLayout(file_layout);
    layout->addSpacing(20);
    layout->addWidget(option_form_w_);
    layout->addStretch();
    layout->addLayout(buttons_layout);
    layout->addLayout(data_layout);
    setRecording(false);
    start_button_->setEnabled(false);

    cur_cartesian_pose_.assign(joint_size.size(), Vector7f::Zero());
}

void RecordTrajectoryWidget::setInFront(bool in_front){
    if (in_front){
        safe_panel_command_.updateCommand([&](PanelCommand& command){
            command.motion_type = MotionControl::kJoint;
            command.need_setting_update = true;
        });
    
        //Assign joint_cmd from planner_state_
        for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
            ComponentData& comp= pair.second;
            for (int comp_i=0; comp_i<comp.component_size; comp_i++){
                comp.joint_cmd[comp_i] = Eigen::Map<VectorXf>((planner_state_shared_ptr_.get()->*comp.state_ref)[comp_i], comp.joint_size[comp_i]);
            }
        }
        emit setControlType(ControlType::kPosition, actuator_mode_);
    }
}

bool RecordTrajectoryWidget::sendVal(){
    bool changed = false;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        changed = command.need_setting_update;
        //Update cur_joint_pos
        for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
            ComponentData& comp= pair.second;
            for (int comp_i=0; comp_i<comp.component_size; comp_i++){
                comp.cur_joint_pos[comp_i] = Eigen::Map<VectorXf>((planner_state_shared_ptr_.get()->*comp.state_ref)[comp_i], comp.joint_size[comp_i]);
                for (int joint_i=0; joint_i<comp.joint_size[comp_i]; joint_i++){
                    (command.*comp.cmd_ref)[comp_i][joint_i] = comp.joint_cmd[comp_i][joint_i];
                }
            }
        }

        //Update cur_cartesian_pose_ when save cartesian
        if (option_map_[kSaveCartesian]){
            for (int arm_i=0; arm_i<command.ArmSize; arm_i++){
                int joint_size = command.JointSize[arm_i];
                cur_cartesian_pose_[arm_i] = Eigen::Map<Vector7f>(planner_state_shared_ptr_->JointPose[arm_i][joint_size-1], 7);
            }
        }
    });
    return changed;
}

void RecordTrajectoryWidget::resizeEvent(QResizeEvent *event){
    int w = this->width();
    int w0 = file_line_edit_->pos().x();
    int w1 = w - (w0 + file_line_edit_->width());
    option_form_w_->setContentsMargins(w0, 0, w1, 0);
    QWidget::resizeEvent(event);
}

void RecordTrajectoryWidget::setRecording(bool record, const QString& file_path){ 
    start_button_->setEnabled(!record);
    savePt_button_->setEnabled(record);
    finish_button_->setEnabled(record);
    cancel_button_->setEnabled(record);
    if  (!record){
        data_.clear();
        last_pt_->setText("None");
        total_pt_->setText("0");
        file_line_edit_->setText(file_path);
    }

    option_form_w_->setEnabled(!record);
    cartesian_pose_.clear();
    emit recordingStateChanged(record);
}

void RecordTrajectoryWidget::slotStartButtonClicked(){
    setRecording(true);
}

void RecordTrajectoryWidget::slotSavePointButtonClicked(){
    std::map<ComponentType, std::vector<VectorXf>> temp;
    for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
        ComponentData& comp= pair.second;
        for (int comp_i=0; comp_i<comp.component_size; comp_i++){
            temp[pair.first].push_back(comp.cur_joint_pos[comp_i]);
        }
    }
    data_.push_back(temp);

    int n = data_.size();
    std::stringstream ss;
    Eigen::IOFormat CleanFmt(4, Eigen::DontAlignCols, ", ", "\n");
    for (const ComponentType& comp_type : component_order_){
        if (!component_map_.count(comp_type) || !component_map_[comp_type].component_size) continue;
        ss << enumToString<ComponentType>(comp_type) << ": ";
        for (int comp_i=0; comp_i<component_map_[comp_type].component_size; comp_i++){
            ss << '[' << component_map_[comp_type].cur_joint_pos[comp_i].transpose().format(CleanFmt) << "], ";
        }
    }
    std::string s = ss.str();
    if (s.length() > 2) s.erase(s.end()-2, s.end());
    last_pt_->setText(QString::fromStdString(s));
    total_pt_->setText(QString::number(n));

    //Save cartesian pose
    if (option_map_[kSaveCartesian]){
        cartesian_pose_.push_back(cur_cartesian_pose_);
    }
}

void RecordTrajectoryWidget::slotFinishButtonClicked(){
    //Save Data
    QString path = file_line_edit_->text();
    writeToText(data_, path.toStdString());

    //Save cartesian pose
    if (option_map_[kSaveCartesian]){
        int lastSlashIndex = path.lastIndexOf('/');
        std::string cart_folder = config_prefix_path_ + "/panel/replay/cartesian_pose";
        std::string cart_path = cart_folder + path.mid(lastSlashIndex).toStdString();
        createFolderIfNotExist(cart_folder);
        writeToText(cartesian_pose_, cart_path, "x, y, z, qw, qx, qy, qz");
    } 
    
    //Reset
    setRecording(false, path);
}

void RecordTrajectoryWidget::slotCancelButtonClicked(){
    QString path = file_line_edit_->text();
    setRecording(false, path); //Reset
}

void RecordTrajectoryWidget::slotFileLineTextChanged(const QString& text){
    start_button_->setDisabled(text.isEmpty());
}

void RecordTrajectoryWidget::slotSaveFileClicked(QFileDialog* dialog){
    QString selfilter = tr("Text files (*.txt)");
    QString default_path = QString::fromStdString(save_path_ + "/untitled.txt");
    QString file_name = QFileDialog::getSaveFileName(this, tr("Save File Path"),
                                                    default_path,
                                                    tr("Text files (*.txt)"),&selfilter, QFileDialog::DontUseNativeDialog);
    file_line_edit_->setText(file_name);
}

void RecordTrajectoryWidget::slotOptionFormCheckBoxChanged(const QString& label, int clicked){
    option_map_[label] = (clicked == Qt::Checked);
}