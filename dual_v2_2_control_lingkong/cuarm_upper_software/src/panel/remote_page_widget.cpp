#include "remote_page_widget.h"

#include <iostream>
#include <chrono>

#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QStringList>

#include "robot_platform_utils/cpp/include/cuarm_message_handler.h"

const QString kConnectText = "Connect";
const QString kDisconnectText = "Disconnect";
const QString kStartText = "Start";
const QString kStopText = "Stop";

RemotePageWidget::RemotePageWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const YAML::Node& config, QWidget* parent): safe_panel_command_(safe_panel_command), planner_state_shared_ptr_(ps), QWidget(parent){
    local_ip_    = config["panel"]["remote_page"]["default_local_ip"].as<std::string>();
    local_port_  = config["panel"]["remote_page"]["default_local_port"].as<int>();
    remote_ip_   = config["panel"]["remote_page"]["default_remote_ip"].as<std::string>();
    remote_port_ = config["panel"]["remote_page"]["default_remote_port"].as<int>();
    float dt_ms  = config["panel"]["remote_page"]["dt_receive"].as<float>() * 1000.0f;
    
    ip_port_widget_ = new QWidget();
    QGridLayout* ip_port_layout = new QGridLayout(ip_port_widget_);
    ip_port_layout->setHorizontalSpacing(24);
    addConfigLayout(ip_port_layout, 0, "Local", QString::fromStdString(local_ip_), QString::number(local_port_));
    addConfigLayout(ip_port_layout, 1, "Remote", QString::fromStdString(remote_ip_), QString::number(remote_port_));

    connect_bnt_ = new QPushButton(kConnectText);
    start_bnt_ = new QPushButton(kStartText);
    start_bnt_->setEnabled(false);
    connect_bnt_->setFixedWidth(300);
    start_bnt_->setFixedWidth(300);
    connect(connect_bnt_, &QPushButton::clicked, this, &RemotePageWidget::slotConnectButtonClicked);
    connect(start_bnt_, &QPushButton::clicked, this, &RemotePageWidget::slotStartButtonClicked);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(ip_port_widget_);
    layout->addStretch();
    layout->addWidget(connect_bnt_, 0, Qt::AlignHCenter);
    layout->addWidget(start_bnt_, 0, Qt::AlignHCenter);
    this->setMaximumWidth(620);

    //Start UDP receive thread
    udp_receive_thread_ = std::thread(&RemotePageWidget::udpReceiveTask, this, dt_ms);

    //Init PanelState
    safe_panel_command_.viewCommand([&](const PanelCommand& command){
        panel_state_.state = PanelRemoteState::kStop;
        panel_state_.ArmSize = command.ArmSize;
        for (int arm_i=0; arm_i<command.ArmSize; arm_i++){
            panel_state_.JointSize[arm_i] = command.JointSize[arm_i];
        }
        panel_state_.GripperSize = command.GripperSize;
        for (int gripper_i=0; gripper_i<command.GripperSize; gripper_i++){
            panel_state_.GripperJointSize[gripper_i] = command.GripperJointSize[gripper_i];
        }
    });
}

RemotePageWidget::~RemotePageWidget(){
    thread_terminate_ = true;
    if (udp_receive_thread_.joinable()) {
        udp_receive_thread_.join();
    }
}

bool RemotePageWidget::connectAndStartAutomatically(){
    if (!is_connected_) {
        slotConnectButtonClicked(false);
    }
    if (!is_running_) {
        slotStartButtonClicked(false);
    }
    const bool ready = is_connected_ && is_running_;
    if (ready) {
        std::cout
            << "[AUTO] Remote Panel connected and started on "
            << local_ip_ << ":" << local_port_
            << " -> " << remote_ip_ << ":" << remote_port_
            << std::endl;
    }
    return ready;
}

void RemotePageWidget::addConfigLayout(QGridLayout* grid_layout, int row_index, QString label_text, QString default_ip, QString default_port){
    //Create IP layout
    QLabel* ip_label = new QLabel(label_text + " IP: ");
    QStringList ip_list = default_ip.split(".");
    QLabel* ip_dot1 = new QLabel(".");
    QLabel* ip_dot2 = new QLabel(".");
    QLabel* ip_dot3 = new QLabel(".");
    QLineEdit* ip_edit1 = new QLineEdit(ip_list[0]);
    QLineEdit* ip_edit2 = new QLineEdit(ip_list[1]);
    QLineEdit* ip_edit3 = new QLineEdit(ip_list[2]);
    QLineEdit* ip_edit4 = new QLineEdit(ip_list[3]);
    ip_dot1->setAlignment(Qt::AlignCenter);
    ip_dot2->setAlignment(Qt::AlignCenter);
    ip_dot3->setAlignment(Qt::AlignCenter);
    ip_edit1->setFixedWidth(52);
    ip_edit2->setFixedWidth(52);
    ip_edit3->setFixedWidth(52);
    ip_edit4->setFixedWidth(52);

    QWidget* ip_input_widget = new QWidget();
    ip_input_widget->setMinimumWidth(225);
    ip_input_widget->setMaximumWidth(240);
    QHBoxLayout* ip_input_layout = new QHBoxLayout(ip_input_widget);
    ip_input_layout->setSpacing(0);
    ip_input_layout->setContentsMargins(0,0,0,0);
    ip_input_layout->addWidget(ip_edit1);
    ip_input_layout->addWidget(ip_dot1);
    ip_input_layout->addWidget(ip_edit2);
    ip_input_layout->addWidget(ip_dot2);
    ip_input_layout->addWidget(ip_edit3);
    ip_input_layout->addWidget(ip_dot3);
    ip_input_layout->addWidget(ip_edit4);

    QHBoxLayout* ip_layout = new QHBoxLayout();
    ip_layout->addWidget(ip_label);
    ip_layout->addStretch();
    ip_layout->addWidget(ip_input_widget);

    //Create Port layout
    QLabel* port_label = new QLabel(label_text + " Port: ");
    QLineEdit* port_edit = new QLineEdit(default_port);
    port_edit->setFixedWidth(78);
    QHBoxLayout* port_layout = new QHBoxLayout();
    port_layout->addWidget(port_label);
    port_layout->addStretch();
    port_layout->addWidget(port_edit);
    
    //Add to grid layout
    grid_layout->addLayout(ip_layout, row_index, 0);
    grid_layout->addLayout(port_layout, row_index, 1);
}

void RemotePageWidget::updateIpPort(int row_index, std::string* ip_ptr, int* port_ptr){
    QGridLayout* grid_layout = qobject_cast<QGridLayout*>(ip_port_widget_->layout());
    
    // Get IP layout from column 0
    QLayoutItem* ip_item = grid_layout->itemAtPosition(row_index, 0);
    if (ip_item) {
        QHBoxLayout* ip_layout = qobject_cast<QHBoxLayout*>(ip_item->layout());
        if (ip_layout) {
            QWidget* ip_input_widget = ip_layout->itemAt(2)->widget(); // index 2 is ip_input_widget
            QHBoxLayout* ip_input_layout = qobject_cast<QHBoxLayout*>(ip_input_widget->layout());
            if (ip_input_layout && ip_input_layout->count() >= 7) {
                QLineEdit* ip_edit1 = qobject_cast<QLineEdit*>(ip_input_layout->itemAt(0)->widget());
                QLineEdit* ip_edit2 = qobject_cast<QLineEdit*>(ip_input_layout->itemAt(2)->widget());
                QLineEdit* ip_edit3 = qobject_cast<QLineEdit*>(ip_input_layout->itemAt(4)->widget());
                QLineEdit* ip_edit4 = qobject_cast<QLineEdit*>(ip_input_layout->itemAt(6)->widget());

                QString ip = QString("%1.%2.%3.%4")
                                .arg(ip_edit1->text())
                                .arg(ip_edit2->text())
                                .arg(ip_edit3->text())
                                .arg(ip_edit4->text());
                *ip_ptr = ip.toStdString();
            }
        }
    }

    // Get Port layout from column 1
    QLayoutItem* port_item = grid_layout->itemAtPosition(row_index, 1);
    if (port_item) {
        QHBoxLayout* port_layout = qobject_cast<QHBoxLayout*>(port_item->layout());
        if (port_layout && port_layout->count() >= 3) {
            QLineEdit* port_edit = qobject_cast<QLineEdit*>(port_layout->itemAt(2)->widget());
            QString port = port_edit->text();
            *port_ptr = port.toInt();
        }
    }
}

void RemotePageWidget::setInFront(bool in_front){
    if (in_front){
        safe_panel_command_.updateCommand([&](PanelCommand& command){
            if (command.motion_type != MotionControl::kJoint){
                command.motion_type = MotionControl::kJoint;
                command.need_setting_update = true;
            }
            config_target_type_ = last_set_target_type_ = command.target_type;
            config_actuator_mode_ = last_set_actuator_mode_ = command.actuator_mode;
            config_acceleration_time_ = command.InterpolationAccTime;
            config_saturation_ratio_ = command.NoneInterpolationSaturationRatio;
        });
    }else{
        last_set_target_type_ = ControlType::kInvalid;
        last_set_actuator_mode_ = ControlType::kInvalid;
        if (is_running_) start_bnt_->click(); //Stop motion before leave
        resetConfigSetting();
    }
}

void RemotePageWidget::clearJointAndGripperCommand(){
    //Update joint command
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.arm_target_mode = PanelTargetMode::kSinglePoint;
        command.need_setting_update = true; // Update interpolator in planner. TODO: Need update for kSinglePoint but not kWaypoint is ugly!
        for (int arm_i=0; arm_i<command.ArmSize; arm_i++){
            for (int joint_i=0; joint_i<command.JointSize[arm_i]; joint_i++){
                switch (command.target_type){
                    case ControlType::kPosition:
                        command.JointCmdDeg[arm_i][joint_i] = planner_state_shared_ptr_->JointPos[arm_i][joint_i];
                        break;
                    default:
                        command.JointCmdDeg[arm_i][joint_i] = 0.0f;
                        break;
                }
            }
        }

        //Update gripper command
        for (int gripper_i=0; gripper_i<command.GripperSize; gripper_i++){
            for (int joint_i=0; joint_i<command.GripperJointSize[gripper_i]; joint_i++){
                command.GripperJointCmd[gripper_i][joint_i] = planner_state_shared_ptr_->GripperJointPos[gripper_i][joint_i];
            }
        }
});
}

void RemotePageWidget::resetConfigSetting(){
    emit setControlType(config_target_type_, config_actuator_mode_);
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.InterpolationAccTime = config_acceleration_time_;
        command.NoneInterpolationSaturationRatio = config_saturation_ratio_;
    });
}

bool RemotePageWidget::isValidRemoteCommand(const RemotePanelCommand& remote_cmd){
    //Check JointCmdDeg size
    try {
        if (remote_cmd.ArmSize != panel_state_.ArmSize){
            throw std::runtime_error("RemotePanelCommand ArmSize invalid, expected " + std::to_string(panel_state_.ArmSize) + ", got " + std::to_string(remote_cmd.ArmSize));
        }
        for (int arm_i=0; arm_i<panel_state_.ArmSize; arm_i++){
            if (remote_cmd.ArmJointSize[arm_i] != panel_state_.JointSize[arm_i]){
                throw std::runtime_error("RemotePanelCommand ArmJointSize[" + std::to_string(arm_i) + "] invalid, expected " + std::to_string(panel_state_.JointSize[arm_i]) + ", got " + std::to_string(remote_cmd.ArmJointSize[arm_i]));
            }
        }

        //Check GripperJointCmd size
        if (remote_cmd.GripperSize != panel_state_.GripperSize){
            throw std::runtime_error("RemotePanelCommand GripperSize invalid, expected " + std::to_string(panel_state_.GripperSize) + ", got " + std::to_string(remote_cmd.GripperSize));
        }
        for (int gripper_i=0; gripper_i<panel_state_.GripperSize; gripper_i++){
            if (remote_cmd.GripperJointSize[gripper_i] != panel_state_.GripperJointSize[gripper_i]){
                throw std::runtime_error("RemotePanelCommand GripperJointSize[" + std::to_string(gripper_i) + "] invalid, expected " + std::to_string(panel_state_.GripperJointSize[gripper_i]) + ", got " + std::to_string(remote_cmd.GripperJointSize[gripper_i]));
            }
        }
    }catch (const std::runtime_error& e){
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
    return true;
}

void RemotePageWidget::updateJointAndGripperCommand(const RemotePanelCommand& remote_cmd){
    if (!isValidRemoteCommand(remote_cmd)) return;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.InterpolationAccTime = remote_cmd.interpolation_acceleration_time;
        command.interpolation_speed_ratio = remote_cmd.no_interpolation_max_velocity_ratio;
        command.arm_target_mode = remote_cmd.arm_target_mode;
        if (command.arm_target_mode != PanelTargetMode::kWaypoint){
            //Update joint command
            for (int arm_i=0; arm_i<command.ArmSize; arm_i++){
                for (int joint_i=0; joint_i<command.JointSize[arm_i]; joint_i++){
                    command.JointCmdDeg[arm_i][joint_i] = remote_cmd.ArmJointCmd[arm_i][joint_i] * 180.0f / M_PI;
                }
            }
        }else{
            //Update path command
            command.ArmWaypointSize = remote_cmd.ArmWaypointSize;
            command.ArmWaypointDt = remote_cmd.ArmWaypointDt;
            for (int point_i =0; point_i < remote_cmd.ArmWaypointSize; point_i++){
                for (int arm_i = 0; arm_i < command.ArmSize; arm_i++){
                    for (int joint_i =0; joint_i < command.JointSize[arm_i]; joint_i++){
                        command.ArmWaypointCmdDeg[point_i][arm_i][joint_i] = remote_cmd.ArmWaypointCmd[point_i][arm_i][joint_i] * 180.0f / M_PI;
                    }
                }
                command.ArmWaypointInterpolationTime[point_i] = remote_cmd.ArmWaypointDt;
                command.ArmWaypointInterpolationSpeedRatio[point_i] = 0.0f; //Default is no speed ratio
            }
        }

        //Update gripper command
        for (int gripper_i=0; gripper_i<command.GripperSize; gripper_i++){
            for (int joint_i=0; joint_i<command.GripperJointSize[gripper_i]; joint_i++){
                command.GripperJointCmd[gripper_i][joint_i] = remote_cmd.GripperJointCmd[gripper_i][joint_i];
            }
        }
    });
}

void RemotePageWidget::updatePanelState(){
    //Update joint state
    for (int arm_i=0; arm_i<panel_state_.ArmSize; arm_i++){
        for (int joint_i=0; joint_i<panel_state_.JointSize[arm_i]; joint_i++){
            panel_state_.JointPos[arm_i][joint_i] = planner_state_shared_ptr_->JointPos[arm_i][joint_i] * M_PI/180.0f;
            panel_state_.JointVel[arm_i][joint_i] = planner_state_shared_ptr_->JointVel[arm_i][joint_i] * M_PI/180.0f;
            panel_state_.JointTor[arm_i][joint_i] = planner_state_shared_ptr_->JointTor[arm_i][joint_i];
        }

        for (int i=0; i<7; i++){
            int ee_joint_i = panel_state_.JointSize[arm_i] - 1;
            panel_state_.EEPose[arm_i][i] = planner_state_shared_ptr_->JointPose[arm_i][ee_joint_i][i];
            panel_state_.ToolPose[arm_i][i] = -1;
        }

        for (int joint_i=0; joint_i<panel_state_.JointSize[arm_i]; joint_i++){
            panel_state_.interpolated_target[arm_i][joint_i] = planner_state_shared_ptr_->interpolated_target[arm_i][joint_i] * M_PI/180.0f;
        }
    }

    //Update gripper state
    for (int gripper_i=0; gripper_i<panel_state_.GripperSize; gripper_i++){
        for (int joint_i=0; joint_i<panel_state_.GripperJointSize[gripper_i]; joint_i++){
            panel_state_.GripperJointPos[gripper_i][joint_i] = planner_state_shared_ptr_->GripperJointPos[gripper_i][joint_i];
        }
    }
}

bool RemotePageWidget::sendVal(){
    bool changed = false;
    if (!is_connected_ || !udp_unique_ptr_) return changed;
    
    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (is_running_ && command_lease_.expired(now)) {
        std::cerr << "[REMOTE WATCHDOG] Planner command stream lost; holding measured joints. Press Start to resume." << std::endl;
        slotStartButtonClicked(false);
        changed = true;
    }

    //Send PanelState
    updatePanelState();
    udp_unique_ptr_->send(&panel_state_);

    //Update PanelCommand
    if (is_running_){
        double received_at = 0.;
        //Get lastest RemotePanelCommand
        {
            std::lock_guard<std::mutex> lock(received_mutex_);
            remote_panel_command_unique_ptr_ = std::move(received_remote_panel_command_unique_ptr_);
            received_at = received_remote_at_;
        }
        if (!remote_panel_command_unique_ptr_) return changed;
        RemotePanelCommand remote_cmd = *remote_panel_command_unique_ptr_.get();
        if (!isValidRemoteCommand(remote_cmd)) return changed;
        const double consumed_at = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!command_lease_.accept(received_at, consumed_at)) {
            std::cerr << "[REMOTE WATCHDOG] Expired queued command rejected; manual Start required." << std::endl;
            slotStartButtonClicked(false);
            return true;
        }

        //Set control type if current control type is different from remote cmd
        ControlType target_type = ControlType::kInvalid;
        ControlType actuator_mode = ControlType::kInvalid;
        safe_panel_command_.viewCommand([&](const PanelCommand& command){
            target_type = command.target_type;
            actuator_mode = command.actuator_mode;
        });
        if (target_type != remote_cmd.target_type || actuator_mode != remote_cmd.actuator_mode){
            if (last_set_target_type_ != remote_cmd.target_type || last_set_actuator_mode_ != remote_cmd.actuator_mode){
                last_set_target_type_ = remote_cmd.target_type;
                last_set_actuator_mode_ = remote_cmd.actuator_mode;
                emit setControlType(last_set_target_type_, last_set_actuator_mode_);
            }else{
                return changed; //setControlType signal sent, but panel_command_ haven't updated. Wait for panel_command_ update.
            }
        }else{
            updateJointAndGripperCommand(remote_cmd);
            changed = true;
        }
    }else if (set_stop_joint_command_){
        set_stop_joint_command_ = false;
        changed = true;
    }

    return changed;
}

void RemotePageWidget::connectRemote(){
    updateIpPort(0, &local_ip_, &local_port_);
    updateIpPort(1, &remote_ip_, &remote_port_);
    is_connected_ = true;
}

void RemotePageWidget::disconnectRemote(){
    is_connected_ = false;
}

void RemotePageWidget::start(){
    is_running_ = true;
    command_lease_.start();
    clearJointAndGripperCommand();
    panel_state_.state = PanelRemoteState::kStart;
    { std::lock_guard<std::mutex> lock(received_mutex_);
      received_remote_panel_command_unique_ptr_.reset(nullptr); received_remote_at_ = 0.; } 
    remote_panel_command_unique_ptr_.reset(nullptr); 
    emit startButtonClicked(true);   // Notify MainWindow to disable other pages
}

void RemotePageWidget::stop(){
    is_running_ = false;
    command_lease_.stop();
    panel_state_.state = PanelRemoteState::kStop;
    { std::lock_guard<std::mutex> lock(received_mutex_);
      received_remote_panel_command_unique_ptr_.reset(nullptr); received_remote_at_ = 0.; } 
    remote_panel_command_unique_ptr_.reset(nullptr); 
    emit startButtonClicked(false);  // Notify MainWindow to enable other pages
    clearJointAndGripperCommand();
    set_stop_joint_command_ = true; // Indicate to send stop joint command once
}

void RemotePageWidget::slotConnectButtonClicked(bool clicked){
    if (!is_connected_){
        connectRemote();
        ip_port_widget_->setEnabled(false);
        start_bnt_->setEnabled(true);
        connect_bnt_->setText(kDisconnectText);
    }else{
        disconnectRemote();
        ip_port_widget_->setEnabled(true);
        start_bnt_->setEnabled(false);
        connect_bnt_->setText(kConnectText);
    }
}

void RemotePageWidget::slotStartButtonClicked(bool clicked){
    if (is_running_){
        stop();
        start_bnt_->setText(kStartText);
        connect_bnt_->setEnabled(true);
    }else{
        start();
        start_bnt_->setText(kStopText);
        connect_bnt_->setEnabled(false);
    }
}

void RemotePageWidget::udpReceiveTask(int dt_ms){
    while (!thread_terminate_){
        if (is_connected_){
            // Initialize UDP if not already done
            if (!udp_unique_ptr_){
                int buffer_size_ = 4096; // Define buffer size
                udp_unique_ptr_ = std::make_unique<CuarmUdp<RemotePanelCommand, PanelState>>(
                    local_ip_, local_port_, remote_ip_, remote_port_,
                    &CuarmMessageHandler::unpack_remote_panel_command, 
                    &CuarmMessageHandler::pack_panel_state, buffer_size_);
            }

            // Receive UDP data
            std::unique_ptr<RemotePanelCommand> temp_ptr = std::make_unique<RemotePanelCommand>();
            if (udp_unique_ptr_->receive(temp_ptr.get(), 50)){
                std::lock_guard<std::mutex> lock(received_mutex_);
                received_remote_panel_command_unique_ptr_ = std::move(temp_ptr);
                received_remote_at_ = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            }
        }else{
            // Clean up UDP 
            if (udp_unique_ptr_){
                udp_unique_ptr_->close();
                udp_unique_ptr_.reset(nullptr); 
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
    }
    if (udp_unique_ptr_) udp_unique_ptr_->close();
}