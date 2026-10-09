#include "replay_trajectory_widget.h"

#include <iostream>
#include <fstream>
#include <string>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>

ReplayTrajectoryWidget::ReplayTrajectoryWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& joint_size, const std::vector<int>& gripper_joint_size, 
    const std::string& config_prefix_path, ControlType actuator_mode, QWidget* parent): RecordAndReplayBase(safe_panel_command, ps, joint_size,
    gripper_joint_size, config_prefix_path){
    actuator_mode_ = (actuator_mode != ControlType::kInvalid)? actuator_mode : ControlType::kPosition;
    
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    layout->setAlignment(Qt::AlignTop);

    QLabel* label = new QLabel("Replay Trajectory From File");
    QHBoxLayout* file_layout = new QHBoxLayout();
    QLabel* open_label = new QLabel("Open: ");
    file_line_edit_ = new QLineEdit("");
    file_line_edit_ -> setAlignment(Qt::AlignRight);
    QPushButton* file_button = new QPushButton("Select");
    QFileDialog* dialog = new QFileDialog(this);
    connect(file_button, &QPushButton::clicked, this, [this, dialog](){slotOpenFileClicked(dialog);});
    dialog->setFileMode(QFileDialog::AnyFile);
    file_layout->addWidget(open_label);
    file_layout->addWidget(file_line_edit_);
    file_layout->addWidget(file_button);

    QHBoxLayout* main_layout = new QHBoxLayout();
    main_layout->setContentsMargins(0,0,0,0);
    pts_group_ = new ReplayPointGroupWidget("Replay Points", component_order_);

    button_container_ = new QWidget();
    QVBoxLayout* button_container_layout = new QVBoxLayout(button_container_);
    button_container_layout->setContentsMargins(0,0,0,0);
    rest_form_ = new FormWidget();
    const QString rest_time_label = "Rest time(s): ";
    rest_form_->addEditorRow(rest_time_label, QString::number(rest_time_));
    rest_form_->setWidgetWidth(rest_time_label,50);
    connect(rest_form_, SIGNAL(editorTextChanged(const QString&, const QString&)), this, SLOT(slotRestFormTextChanged(const QString&, const QString&))); 

    QPushButton* set_start_button = new QPushButton("Set Start");
    QPushButton* set_end_button = new QPushButton("Set End");
    QPushButton* insert_above_button = new QPushButton("Insert Above");
    QPushButton* insert_below_button = new QPushButton("Insert Below");
    QPushButton* delete_pt_button = new QPushButton("Delete Point");
    QPushButton* run_button = new QPushButton("RUN");
    stop_button_ = new QPushButton("STOP");
    stop_button_->setDisabled(true);
    connect(set_start_button, SIGNAL(clicked(bool)), this, SLOT(slotSetStartButtonClicked()));
    connect(set_end_button, SIGNAL(clicked(bool)), this, SLOT(slotSetEndButtonClicked()));
    connect(insert_above_button, &QPushButton::clicked, this, [this](){slotInsertButtonClicked(true);});
    connect(insert_below_button, &QPushButton::clicked, this, [this](){slotInsertButtonClicked(false);});
    connect(delete_pt_button, SIGNAL(clicked(bool)), this, SLOT(slotDeleteButtonClicked()));
    connect(run_button, SIGNAL(clicked(bool)), this, SLOT(slotRunButtonClicked()));
    connect(stop_button_, SIGNAL(clicked(bool)), this, SLOT(slotStopButtonClicked()));

    button_container_layout->addWidget(rest_form_);
    button_container_layout->addWidget(set_start_button);
    button_container_layout->addWidget(set_end_button);
    button_container_layout->addWidget(insert_above_button);
    button_container_layout->addWidget(insert_below_button);
    button_container_layout->addWidget(delete_pt_button);
    button_container_layout->addWidget(run_button);

    QVBoxLayout* buttons_layout = new QVBoxLayout();
    buttons_layout->setAlignment(Qt::AlignHCenter);
    buttons_layout->setAlignment(Qt::AlignVCenter);
    buttons_layout->setContentsMargins(20,20,20,20);
    buttons_layout->addWidget(button_container_);
    buttons_layout->addWidget(stop_button_);

    QWidget* main_right_widget = new QWidget();
    main_right_widget->setMaximumWidth(300);
    // main_right_widget->setMinimumHeight(500);

    QVBoxLayout* main_right_layout = new QVBoxLayout(main_right_widget);
    ResetSaveBlock* reset_save = new ResetSaveBlock();
    reset_save->addWidget(this);

    QHBoxLayout* counter_layout = new QHBoxLayout();
    counter_layout->setContentsMargins(0,0,0,0);
    counter_layout->setAlignment(Qt::AlignHCenter);
    QLabel* counter_label = new QLabel("Played points: ");
    counter_ = new QLabel("0/0");
    counter_layout->addWidget(counter_label);
    counter_layout->addWidget(counter_);

    main_right_layout->addLayout(buttons_layout);
    main_right_layout->addStretch();
    main_right_layout->addLayout(counter_layout);
    main_right_layout->addWidget(reset_save);

    main_layout->addWidget(pts_group_);
    main_layout->addWidget(main_right_widget);
    layout->addWidget(label);
    layout->addLayout(file_layout);
    layout->addLayout(main_layout);
}


void ReplayTrajectoryWidget::setInFront(bool in_front){
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
    }else{
        stop_button_->click();
    }
}

void ReplayTrajectoryWidget::getNextTrajPoint(){
    std::map<ComponentType, std::vector<VectorXf>> next_data = trajectory_.front();
    for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
        if (!next_data.count(pair.first)) continue;
        ComponentData& comp= pair.second;
        for (int comp_i=0; comp_i<comp.component_size; comp_i++){
            comp.joint_cmd[comp_i] = next_data[pair.first][comp_i];
        }
    }
    trajectory_.pop_front();
    points_count_ += 1;
    start_time_ = get_time_now();
}

bool ReplayTrajectoryWidget::sendVal(){
    bool changed = false;
    long t_diff = 0, t = 0;
    safe_panel_command_.viewCommand([&](const PanelCommand& command){
        changed = command.need_setting_update;
        t = (command.InterpolationAccTime + command.InterpolationConstVelTime + rest_time_)*1e6;
        t_diff = get_time_now() - start_time_;
    });

    if (state_ == ReplayState::kStart){
        getNextTrajPoint();
        pts_group_->setStartAsButtonChecked();
        state_ = ReplayState::kRunning;
        changed = true;
    }else if (state_ == ReplayState::kRunning && t_diff > t){
        if (trajectory_.empty()){
            state_ = ReplayState::kFinish;
            button_container_->setDisabled(false);
        }else{
            getNextTrajPoint();
            pts_group_->setButtonCheckedToNext();
            changed = true;
        }
    }

    counter_->setText(QString("%1/%2").arg(QString::number(points_count_), QString::number(points_total_)));
    //Update cur_joint_pos and panel_command_
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
            ComponentData& comp= pair.second;
            for (int comp_i=0; comp_i<comp.component_size; comp_i++){
                comp.cur_joint_pos[comp_i] = Eigen::Map<VectorXf>((planner_state_shared_ptr_.get()->*comp.state_ref)[comp_i], comp.joint_size[comp_i]);
                for (int joint_i=0; joint_i<comp.joint_size[comp_i]; joint_i++){
                    (command.*comp.cmd_ref)[comp_i][joint_i] = comp.joint_cmd[comp_i][joint_i];
                }
            }
        }
    });
    return changed;
}

void ReplayTrajectoryWidget::reset(){
    QString file_name = file_line_edit_->text();
    displayPoints(file_name.toStdString());
}

void ReplayTrajectoryWidget::save(){
    std::string path = file_line_edit_->text().toStdString();
    std::vector<std::map<ComponentType, std::vector<VectorXf>>> trajectory = pts_group_->getFullTrajectory();
    writeToText(trajectory, path);
}

void ReplayTrajectoryWidget::displayPoints(std::string file_name){
    data_ = extractFromText(file_name);
    pts_group_->clear();
    pts_group_->addAllPoints(data_);
}

void ReplayTrajectoryWidget::slotRestFormTextChanged(const QString& label, const QString& text){
    rest_time_ = text.toFloat();
}

void ReplayTrajectoryWidget::slotSetStartButtonClicked(){
    pts_group_->setSelectedPointAsStart();
}

void ReplayTrajectoryWidget::slotSetEndButtonClicked(){
    pts_group_->setSelectedPointAsEnd();
}

void ReplayTrajectoryWidget::slotInsertButtonClicked(bool above){
    std::map<ComponentType, std::vector<VectorXf>> cus_pos;
    for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
        ComponentData& comp= pair.second;
        for (int comp_i=0; comp_i<comp.component_size; comp_i++){
            cus_pos[pair.first].push_back(comp.cur_joint_pos[comp_i]);
        }
    }
    pts_group_->insertPoint(cus_pos, above);
}

void ReplayTrajectoryWidget::slotDeleteButtonClicked(){
    pts_group_->deleteSelectedPoint();
}

void ReplayTrajectoryWidget::slotRunButtonClicked(){
    if (state_ == ReplayState::kFinish){
        trajectory_ = pts_group_->getTrajectory();
        points_total_ = trajectory_.size();
        points_count_ = 0;
        if (!trajectory_.empty()){
            state_ = ReplayState::kStart;
            button_container_->setDisabled(true);
            stop_button_->setDisabled(false);
        }
    }
}

void ReplayTrajectoryWidget::slotStopButtonClicked(){
    state_ = ReplayState::kFinish;
    button_container_->setDisabled(false);
    trajectory_.clear();
    stop_button_->setDisabled(true);
    for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
        ComponentData& comp= pair.second;
        for (int comp_i=0; comp_i<comp.component_size; comp_i++){
            comp.joint_cmd[comp_i] = comp.cur_joint_pos[comp_i];
        }
    }
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.need_setting_update = true;
    });
}

void ReplayTrajectoryWidget::slotOpenFileClicked(QFileDialog* dialog){
    QString selfilter = tr("Text files (*.txt)");
    QString default_path = QString::fromStdString(save_path_);
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    default_path,
                                                    tr("Text files (*.txt)"),&selfilter, QFileDialog::DontUseNativeDialog);
    file_line_edit_->setText(file_name);
    displayPoints(file_name.toStdString());
}
