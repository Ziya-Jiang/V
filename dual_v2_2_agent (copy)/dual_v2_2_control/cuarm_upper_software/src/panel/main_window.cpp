#include "main_window.h"

#include <cmath> 
#include <array>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <Eigen/Geometry>

#include <QLabel>
#include <QMessageBox>
#include <QScrollArea>
#include <QStatusBar>
#include <QDebug>
#include <QDockWidget>

#include "common_widget.h"
#include "slider_block.h"
#include "zero_set_block.h"
#include "reset_save_block.h"
#include "collapsible_widget.h"
#include "plotWidget.h"
#include "arm_setting_widget.h"
#include "control_page_widget.h"
#include "record_trajectory_widget.h"
#include "replay_trajectory_widget.h"
#include "remote_page_widget.h"
#include "joint_enable_widget.h"
#include "connection_button.h"
#include "qml_simulation_widget.h"
#include "arm_diagnostic_dock.h"

#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_message_handler.h"

// const QString kPlotPage = "Plot";
const QString kControlPage = "Control";
const QString kRecordPage  = "Teach";
const QString kReplayPage  = "Replay";
const QString kRemotePage  = "Remote";
const QString kStatePage   = "State";
const QString kMotorEnable = "EnableMotor";
const QString kQmlSimulation = "QmlSimulation";
std::vector<QString> kMainTabOrder {kControlPage, kRecordPage, kReplayPage, kRemotePage};
std::vector<QString> kAllWidget {kControlPage, kRecordPage, kReplayPage, kRemotePage, kStatePage, kMotorEnable, kQmlSimulation};
const int totalWidget = kAllWidget.size();

namespace {
int environmentPortOffset() {
    const char* value = std::getenv("CUARM_UDP_PORT_OFFSET");
    if (!value) return 0;
    try {
        std::size_t parsed = 0;
        const int offset = std::stoi(value, &parsed);
        return parsed == std::strlen(value) ? offset : 0;
    } catch (const std::exception&) {
        return 0;
    }
}
}

using namespace cuarm_panel;
MainWindow::MainWindow(const std::string& config_prefix_path, const std::string& project_base_path, QWidget *parent): config_prefix_path_(config_prefix_path), QMainWindow(parent){
    qRegisterMetaType<RosRobotState>();

    this->config_file_ = config_prefix_path_ + "/config.yaml";
    config_ = loadYamlConfig(config_file_);
    YAML::Node panel_config = config_["panel"];
    dt_receive_ = panel_config["dt_receive"].as<float>();
    dt_send_ = panel_config["dt_send"].as<float>();
    int w = panel_config["mainwindow_size"][0].as<int>();
    int h = panel_config["mainwindow_size"][1].as<int>();
    setWindowTitle(tr("CUARM · 双臂控制中心"));
    resize(w, h);
    setMinimumSize(std::min(w, 960), std::min(h, 720));
    setContentsMargins(12, 8, 12, 0);

    YAML::Node arm_node = config_["robot"]["arm"];
    arm_size_ = arm_node.size();
    if (arm_size_ > 2) throw std::runtime_error("Arm size > 2 is not supported!\n");

    motor_size_.reserve(arm_size_);
    joint_size_.reserve(arm_size_);
    joint_limits_.reserve(arm_size_);
    // pose_.reserve(arm_size_);
    for (int arm_i=0; arm_i < arm_size_; arm_i++){
        motor_size_.push_back(arm_node[arm_i]["joint_size"].as<int>());
        joint_size_.push_back(arm_node[arm_i]["joint_size"].as<int>());
        joint_limits_.push_back(Matrix2Xf::Zero(2, joint_size_[arm_i]));
        for (int i =0; i <joint_size_[arm_i]; i++){
            joint_limits_[arm_i](0,i) = arm_node[arm_i]["safety"]["joint_soft_limit"]["position"][0][i].as<float>();
            joint_limits_[arm_i](1,i) = arm_node[arm_i]["safety"]["joint_soft_limit"]["position"][1][i].as<float>();
        }
    } 

    if (config_["robot"]["gripper"]){
        YAML::Node gripper_node = config_["robot"]["gripper"];
        gripper_size_ = gripper_node.size();
        if (gripper_size_ != 0 && gripper_size_ != arm_size_) throw std::runtime_error("Gripper size should be either 0 or equal to arm size!\n");
        for (int gri_i=0; gri_i < gripper_size_; gri_i++){
            gripper_joint_size_.push_back(gripper_node[gri_i]["joint_size"].as<int>());
            gripper_names_.push_back(gripper_node[gri_i]["name"].as<std::string>());
            std::vector<std::string> single_gripper_joint_names;
            for (int ji=0; ji<gripper_joint_size_[gri_i]; ji++){
                std::string joint_name = (gripper_node[gri_i]["joint_names"])? gripper_node[gri_i]["joint_names"][ji].as<std::string>() :
                                        gripper_names_[gri_i] + "_J" + std::to_string(ji);
                single_gripper_joint_names.push_back(joint_name);
            }
            gripper_joint_names_.push_back(single_gripper_joint_names);
        }
    }

    //Must initCommandAndState before arm_setting_w->emitAllSignals() since some slotControlTypeChanged may use PlannerState's values
    initCommandAndState(config_);

    // QMenuBar* bar = this->menuBar();
    // QMenu* file= bar->addMenu("File");
    // file->addAction("show");
    // file->addAction("add");
    // file->addAction("remove");
    // file.triggered[QAction].connect(self.processtrigger)

    ConnectionButton* planner_state_bn = new ConnectionButton(safe_panel_command_, this);
    planner_state_bn->setObjectName(QStringLiteral("ConnectionToggle"));
    connect(planner_state_bn, SIGNAL(connectUDP(bool)), this, SLOT(slotConnectUDP(bool)));
    this->statusBar()->addPermanentWidget(planner_state_bn);
    control_type_display_ = new QLabel("Top: , Base: ", this);
    this->statusBar()->addWidget(control_type_display_);

    QWidget* main_widget = new QWidget(this);
    QGridLayout* layout = new QGridLayout(main_widget);
    layout->setContentsMargins(0,0,0,0);
    layout->setHorizontalSpacing(10);
    layout->setVerticalSpacing(10);
    // plot_manager_ = new PlotManager(dt_, main_widget);

    main_page_tab_w_ = new QTabWidget(main_widget);
    connect(main_page_tab_w_, &QTabWidget::currentChanged, this, &MainWindow::slotTabWCurrentChanged);
    for (QString page_name : kMainTabOrder){
        main_page_tab_w_->addTab(genNewPage(page_name, main_page_tab_w_), page_name);
    }
    RecordTrajectoryWidget* record_w = (RecordTrajectoryWidget*)all_widgets_[kRecordPage];
    ReplayTrajectoryWidget* replay_w = (ReplayTrajectoryWidget*)all_widgets_[kReplayPage];
    ControlPageWidget* control_w = (ControlPageWidget*)all_widgets_[kControlPage];
    RemotePageWidget* remote_w = (RemotePageWidget*)all_widgets_[kRemotePage];
    ArmSettingWidget* arm_setting_w = control_w->getArmSettingWidget();
    connect(record_w, SIGNAL(setControlType(ControlType, ControlType)), arm_setting_w, SLOT(slotSetControlType(ControlType, ControlType)));
    connect(record_w, &RecordTrajectoryWidget::recordingStateChanged, control_w, &ControlPageWidget::slotRecordingStateChanged);
    connect(replay_w, SIGNAL(setControlType(ControlType, ControlType)), arm_setting_w, SLOT(slotSetControlType(ControlType, ControlType)));
    connect(this, SIGNAL(resetControlPage()), control_w, SLOT(slotResetPage()));
    connect(this, SIGNAL(setControlType(ControlType, ControlType)), arm_setting_w, SLOT(slotSetControlType(ControlType, ControlType)));
    connect(arm_setting_w, SIGNAL(controlTypeChanged(ControlType, ControlType)), record_w, SLOT(slotControlTypeChanged(ControlType, ControlType)));
    connect(arm_setting_w, SIGNAL(controlTypeChanged(ControlType, ControlType)), replay_w, SLOT(slotControlTypeChanged(ControlType, ControlType)));
    connect(arm_setting_w, SIGNAL(controlTypeChanged(ControlType, ControlType)), this, SLOT(slotControlTypeChanged(ControlType, ControlType)));
    connect(remote_w, &RemotePageWidget::startButtonClicked, this, &MainWindow::slotRemoteStartButtonClicked);
    connect(remote_w, SIGNAL(setControlType(ControlType, ControlType)), arm_setting_w, SLOT(slotSetControlType(ControlType, ControlType)));
    arm_setting_w->emitAllSignals();

    QWidget* state_widget = genNewPage(kStatePage, main_widget);
    state_widget->setMinimumWidth(180 * arm_size_);
    state_widget->setMaximumWidth(230 * arm_size_);
    state_widget->setMaximumHeight(static_cast<int>(h * 0.60));

    QWidget* joint_enable_page = genNewPage(kMotorEnable, main_widget);
    joint_enable_page->setMinimumWidth(130 * arm_size_);
    joint_enable_page->setMaximumWidth(160 * arm_size_);
    joint_enable_page->setMaximumHeight(static_cast<int>(h * 0.60));
    JointEnableWidget* joint_enable_widget = (JointEnableWidget*)all_widgets_[kMotorEnable];
    connect(joint_enable_widget, SIGNAL(jointEnableChanged()), this, SLOT(slotJointEnableChanged()));

    QWidget* qml_page = genNewPage(kQmlSimulation, main_widget);
    QmlSimulationWidget* qml_widget = (QmlSimulationWidget*)all_widgets_[kQmlSimulation];
    connect(this, SIGNAL(poseChanged()), qml_widget, SLOT(slotUpdatePose()));

    layout->addWidget(qml_widget,1,1);
    layout->addWidget(joint_enable_page,1,2);
    layout->addWidget(state_widget,1,3);
    layout->addWidget(main_page_tab_w_,2,1,1,3); // int row, int column, int rowSpan, int columnSpan
    layout->setColumnStretch(1, 4);
    layout->setColumnStretch(2, 0);
    layout->setColumnStretch(3, 3);
    layout->setRowStretch(1, 5);
    layout->setRowStretch(2, 3);
    this->setCentralWidget(main_widget);

    QStringList arm_names;
    for (int arm_i = 0; arm_i < arm_size_; ++arm_i) {
        arm_names.append(
            QString::fromStdString(arm_node[arm_i]["name"].as<std::string>()));
    }
    diagnostic_dock_ =
        new ArmDiagnosticDock(arm_names, joint_size_, this);
    addDockWidget(Qt::RightDockWidgetArea, diagnostic_dock_);
    diagnostic_dock_->setFloating(true);
    diagnostic_dock_->resize(860, 440);
    diagnostic_dock_->move(this->x() + this->width() + 20, this->y());
    QAction* diagnostic_action = diagnostic_dock_->toggleViewAction();
    diagnostic_action->setText(tr("机械臂诊断"));
    menuBar()->addMenu(tr("视图"))->addAction(diagnostic_action);
    diagnostic_dock_->hide();

    //Connect to Robot
    int buffer_size = 60000;
    const int port_offset = environmentPortOffset();
    const int panel_port = config_["panel"]["port"].as<int>() + port_offset;
    const int rt_control_port = config_["rt_control"]["port"].as<int>() + port_offset;
    if (panel_port <= 0 || panel_port > 65535 ||
        rt_control_port <= 0 || rt_control_port > 65535) {
        throw std::runtime_error("CUARM_UDP_PORT_OFFSET produces an invalid UDP port.");
    }
    udp_ = std::make_unique<CuarmUdp<PlannerState, PanelCommand>>(
        config_["panel"]["ip"].as<std::string>(),
        panel_port,
        config_["rt_control"]["ip"].as<std::string>(),
        rt_control_port,
        &CuarmMessageHandler::unpack_planner_state,
        &CuarmMessageHandler::pack_panel_command, buffer_size);

    uint32_t expected_waiting_ack = 0;
    for (int i = 0; i < 3; ++i) {
        sendPanelCmdOnce();
    }
    safe_panel_command_.viewCommand([&](const PanelCommand& command) {
        expected_waiting_ack = command.sequence_id - 1;
    });

    std::cout << "Connecting to Robot with safe target synchronization..." << std::endl;
    const long handshake_deadline = get_time_now() + 10000000;
    bool handshake_acknowledged = false;
    while (get_time_now() < handshake_deadline && !handshake_acknowledged){
        if (receivePlannerState() && consumeReceivedPlannerState()) {
            handshake_acknowledged =
                planner_state_shared_ptr_->received_sequence_id == expected_waiting_ack;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!handshake_acknowledged) {
        throw std::runtime_error(
            "Robot did not acknowledge the safe startup handshake within 10 seconds.");
    }
    required_ack_sequence_.store(
        expected_waiting_ack, std::memory_order_release);
    required_ack_valid_.store(true, std::memory_order_release);
    handshake_frames_remaining_ = 0;
    session_synchronized_ = false;
    std::cout << "Safe waiting handshake acknowledged; synchronized control will arm "
                 "after the UI event loop starts."
              << std::endl;

    TriangleButton::setImagePath(QString::fromStdString(project_base_path+"/images/button.png"));

    //Connect to NDI
    ndi_enabled_ = config_["panel"]["enable_ndi"].as<bool>();
    if (ndi_enabled_){
        int ndi_buffer_size = 2048;
        udp_ndi_ = std::make_unique<CuarmUdp<std::nullptr_t, PanelState>>(
            config_["panel"]["ip"].as<std::string>(), 10001,
            config_["ndi"]["ip"].as<std::string>(), 10002,
            nullptr, &CuarmMessageHandler::pack_panel_state, ndi_buffer_size);
        ndi_thread_ = new QThread(this);
        ndi_timer_ = new QTimer(0); 
        ndi_timer_->setInterval(4); 
        connect(ndi_timer_, SIGNAL(timeout()), this, SLOT(slotNdiTimerTimeout()), Qt::DirectConnection);
        ndi_timer_->moveToThread(ndi_thread_);
        ndi_timer_->connect(ndi_thread_, SIGNAL(started()), SLOT(start()));
        ndi_thread_->start();
        std::cout << "Connect to NDI." <<std::endl;
    }

    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.LastSendTime = get_time_now();
        command.LastReceiveTime = get_time_now();
    });
    current_tab_name_ = kControlPage;

    //Timer
    timer_ = new QTimer(0);
    timer_->setInterval(dt_send_*1000);
    connect(timer_, SIGNAL(timeout()), this, SLOT(slotTimerTimeout()), Qt::DirectConnection);

    receive_timer_ = new QTimer(0); 
    receive_timer_->setInterval(dt_receive_*1000); 
    connect(receive_timer_, SIGNAL(timeout()), this, SLOT(slotReceiveTimerTimeout()), Qt::DirectConnection);

    // Keep the safety heartbeat independent from GUI painting, user input and
    // state processing. A stalled GUI event loop must not trip RT Control's
    // command watchdog.
    heartbeat_thread_ = new QThread(this);
    heartbeat_timer_ = new QTimer(0);
    heartbeat_timer_->setTimerType(Qt::PreciseTimer);
    heartbeat_timer_->setInterval(std::max(1, static_cast<int>(dt_send_ * 1000)));
    connect(heartbeat_timer_, SIGNAL(timeout()), this,
            SLOT(slotHeartbeatTimerTimeout()), Qt::DirectConnection);
    heartbeat_timer_->moveToThread(heartbeat_thread_);
    connect(heartbeat_thread_, &QThread::started,
            heartbeat_timer_, qOverload<>(&QTimer::start));
    connect(heartbeat_thread_, &QThread::finished,
            heartbeat_timer_, &QObject::deleteLater);

    //Create separate thread for high frequency receive events.
    receive_thread_ = new QThread(this);
    receive_timer_->moveToThread(receive_thread_);

    connect(receive_thread_, &QThread::started,
            receive_timer_, qOverload<>(&QTimer::start));
    receive_thread_->start();
    heartbeat_thread_->start();
    timer_->start();
    QTimer::singleShot(0, this, &MainWindow::slotTimerTimeout);
}

MainWindow::~MainWindow(){
    timer_->stop();

    // Stop command production before sending the final shutdown frames and
    // before destroying UDP or command state.
    if (heartbeat_timer_ && heartbeat_thread_ && heartbeat_thread_->isRunning()) {
        QMetaObject::invokeMethod(
            heartbeat_timer_, "stop", Qt::BlockingQueuedConnection);
    }
    heartbeat_thread_->quit();
    heartbeat_thread_->wait();
    delete heartbeat_thread_;
    heartbeat_thread_ = nullptr;
    heartbeat_timer_ = nullptr;

    if (receive_timer_ && receive_thread_ && receive_thread_->isRunning()) {
        QMetaObject::invokeMethod(
            receive_timer_, "stop", Qt::BlockingQueuedConnection);
    }
    receive_thread_->quit();
    receive_thread_->wait();
    delete receive_timer_;
    delete timer_;
    delete receive_thread_;
    receive_thread_ = nullptr;
    timer_ = nullptr;
    receive_timer_ = nullptr;

    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.connection_state = ConnectionState::kShutDown;
    });
    for (int i =0; i < 3; i++){
        sendPanelCmdOnce();
        sleep_period(dt_send_*1e6);
    }

    if (ndi_enabled_){
        if (ndi_timer_ && ndi_thread_ && ndi_thread_->isRunning()) {
            QMetaObject::invokeMethod(
                ndi_timer_, "stop", Qt::BlockingQueuedConnection);
        }
        ndi_thread_->quit();
        ndi_thread_->wait();
        delete ndi_timer_;
        delete ndi_thread_;
        ndi_thread_ = nullptr;
        ndi_timer_ = nullptr;
        ndi_planner_state_shared_ptr_.reset();
        if (udp_ndi_) udp_ndi_->close();
    }

    if (udp_) udp_->close();
}

void MainWindow::initCommandAndState(const YAML::Node& config){
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.sequence_id = 0;
        command.need_setting_update = true;
        command.simulation = true;
        command.motion_type = MotionControl::kMotor;
        command.connection_state = ConnectionState::kWaiting;
        command.reset_control_mem = false;
        command.reset_interpolation = false;
        command.control_algorithm = ControlAlgorithm::kNone;
        command.interpolation_speed_ratio = 0.5f; //Default is no speed ratio

        std::string target_type_str = config["panel"]["general"]["target_type"].as<std::string>();
        std::string actuator_mode_str = config["panel"]["general"]["actuator_mode"].as<std::string>();
        command.target_type = stringToEnum<ControlType>(target_type_str);
        command.actuator_mode = stringToEnum<ControlType>(actuator_mode_str);
        
        command.arm_target_mode = PanelTargetMode::kSinglePoint;
        command.ArmWaypointSize = 0;
        command.ArmSize = arm_size_;
        planner_state_shared_ptr_ = std::make_shared<PlannerState>();
        for (int arm_i=0; arm_i<arm_size_; arm_i++){
            YAML::Node arm_node = config["robot"]["arm"][arm_i];
            std::string arm_name = arm_node["name"].as<std::string>();
            std::string arm_type = arm_node["type"].as<std::string>();
            std::string robot_name = arm_name + "-" + arm_type;
            strcpy(command.RobotName[arm_i], robot_name.c_str());
            command.JointSize[arm_i] = joint_size_[arm_i];

            for (int i =0; i < joint_size_[arm_i]; i++){
                command.MotorCmdDeg[arm_i][i] = 0;
                command.JointCmdDeg[arm_i][i] = 0;
                if (i < 6)
                    command.TaskCmdDeg[arm_i][i] = 0;

                planner_state_shared_ptr_->JointPos[arm_i][i] = 0;
                planner_state_shared_ptr_->JointVel[arm_i][i] = 0;
                planner_state_shared_ptr_->JointTor[arm_i][i] = 0;
                planner_state_shared_ptr_->MotorPos[arm_i][i] = 0;
                planner_state_shared_ptr_->MotorVel[arm_i][i] = 0;
                planner_state_shared_ptr_->MotorCur[arm_i][i] = 0;
                planner_state_shared_ptr_->MotorTor[arm_i][i] = 0;
            }

            //Init joint enable state
            for (int joint_i=0; joint_i<joint_size_[arm_i]; joint_i++){
                bool enable = config["robot"]["arm"][arm_i]["enable_joint"][joint_i].as<bool>();
                command.enable_joint[arm_i][joint_i] = enable;
            }
        }

        command.GripperSize = gripper_size_;
        for (int gri_i=0; gri_i<gripper_size_; gri_i++){
            command.GripperJointSize[gri_i] = gripper_joint_size_[gri_i];
            for (int i=0; i < gripper_joint_size_[gri_i]; i++){
                command.GripperJointCmd[gri_i][i] = 0;
                planner_state_shared_ptr_->GripperJointPos[gri_i][i] = 0;
            }
        }

        //Config setting
        const float kDegToRad = M_PI/180.0f;
        for (int arm_i=0; arm_i<arm_size_; arm_i++){
            command.tool_offset[arm_i][0] = 0.0f; //Default tool offset is 0
            command.tool_offset[arm_i][1] = 0.0f; //Default tool offset is 0
            command.tool_offset[arm_i][2] = 0.0f; //Default tool offset is 0
            for (int joint_i=0; joint_i<joint_size_[arm_i]; joint_i++){
                command.arm_soft_limit_position[arm_i][joint_i][0] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["position"][0][joint_i].as<float>() * kDegToRad;
                command.arm_soft_limit_position[arm_i][joint_i][1] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["position"][1][joint_i].as<float>() * kDegToRad;
                command.arm_soft_limit_velocity[arm_i][joint_i][0] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["velocity"][joint_i].as<float>() * kDegToRad * -1.0f;
                command.arm_soft_limit_velocity[arm_i][joint_i][1] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["velocity"][joint_i].as<float>() * kDegToRad;
            }
        }
        
        for (int gri_i=0; gri_i<gripper_size_; gri_i++){
            command.tool_offset[gri_i][0] = config["robot"]["gripper"][gri_i]["offset_from_ee"][0].as<float>();
            command.tool_offset[gri_i][1] = config["robot"]["gripper"][gri_i]["offset_from_ee"][1].as<float>();
            command.tool_offset[gri_i][2] = config["robot"]["gripper"][gri_i]["offset_from_ee"][2].as<float>();
            for (int joint_i=0; joint_i<gripper_joint_size_[gri_i]; joint_i++){
                command.gripper_soft_limit_position[gri_i][joint_i][0] = config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][0][joint_i].as<float>() * kDegToRad;
                command.gripper_soft_limit_position[gri_i][joint_i][1] = config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][1][joint_i].as<float>() * kDegToRad;
            }
        }
    });
}

void MainWindow::slotReceiveTimerTimeout(){
    long t0 = get_time_now();
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.ReceiveTimerTimeoutCall = t0 - command.LastReceiveTime;
        command.LastReceiveTime = t0;
    });
    bool received = receivePlannerState();
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.ReceiveUDPCost = get_time_now()-t0;
    });
}

void MainWindow::slotNdiTimerTimeout(){
    {
        std::lock_guard<std::mutex> lock(planner_state_mutex_);
        ndi_planner_state_shared_ptr_ = planner_state_shared_ptr_;
    }
    // ndi_planner_state_shared_ptr_->connection_state = ConnectionState::kRemote;
    sendPlannerStateToNdi(ndi_planner_state_shared_ptr_.get()); 
}

void MainWindow::slotHeartbeatTimerTimeout(){
    const long t0 = get_time_now();
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.TimerTimeoutCall = t0 - command.LastSendTime;
        command.LastSendTime = t0;
    });

    sendPanelCmdOnce();

    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.SendUDPCost = get_time_now() - t0;
    });
}

void MainWindow::slotTimerTimeout(){
    long t0 = get_time_now();

    //Process UI
    consumeReceivedPlannerState();

    if (!session_synchronized_) {
        control_widgets_[kStatePage]->sendVal();
        emit poseChanged();

        if (handshake_frames_remaining_ > 0) {
            --handshake_frames_remaining_;
        } else if (planner_state_shared_ptr_ && isLatestTargetReceived()) {
            safe_panel_command_.updateCommand([&](PanelCommand& command) {
                command.connection_state = ConnectionState::kRemote;
                command.need_setting_update = true;
                command.reset_interpolation = true;
                required_ack_sequence_.store(
                    command.sequence_id, std::memory_order_release);
                required_ack_valid_.store(true, std::memory_order_release);
            });
            session_synchronized_ = true;
            statusBar()->showMessage(tr("安全握手完成，控制目标已同步"), 5000);
            std::cout << "[SESSION] Upper control heartbeat armed with synchronized targets."
                      << std::endl;
        }
        return;
    }

    bool update = false;
    if (control_widgets_[current_tab_name_]){
        update = control_widgets_[current_tab_name_]->sendVal();
    }
    control_widgets_[kStatePage]->sendVal();
    emit poseChanged();

    // Update command state. The dedicated heartbeat thread sends it.
    sendPanelCmdAndAck(update);
    
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.RunningCost = get_time_now()-t0;
    });
    // sendRosRobotState();
}

bool MainWindow::receivePlannerState(){
    std::unique_ptr<PlannerState> temp = std::make_unique<PlannerState>();
    if (udp_->receive(temp.get(), 50)){
        if (!validatePlannerState(*temp)) {
            if (diagnostic_dock_) {
                diagnostic_dock_->appendExternalError(
                    tr("通信数据"),
                    tr("收到的 PlannerState 尺寸或数值无效，报文已丢弃"));
            }
            return false;
        }

        //Convert from rad to deg for pos and vel.
        for (int arm_i=0; arm_i<arm_size_; arm_i++){
            for (int i=0; i < joint_size_[arm_i]; i++){
                temp->JointPos[arm_i][i] *= 180/M_PI;
                temp->JointVel[arm_i][i] *= 180/M_PI;
                temp->MotorPos[arm_i][i] *= 180/M_PI;
                temp->MotorVel[arm_i][i] *= 180/M_PI;
            }
            
            if (last_target_type_ != ControlType::kTorque){
                if (last_motion_control_ != MotionControl::kTask && last_motion_control_ != MotionControl::kTaskLine){
                    for (int i=0; i < joint_size_[arm_i]; i++){
                        temp->interpolated_target[arm_i][i] *= 180/M_PI;
                    }
                }else{
                    for (int i=3; i < 6; i++){
                        temp->interpolated_target[arm_i][i] *= 180/M_PI;
                    }
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock(planner_state_mutex_);
            transmit_planner_state_shared_ptr_ = std::move(temp);
        }
        return true;
    }
    return false;
}

bool MainWindow::validatePlannerState(const PlannerState& state) const {
    if (state.ArmSize != arm_size_ || state.ArmSize > MAX_ARM_SIZE ||
        state.GripperSize != gripper_size_ || state.GripperSize > MAX_ARM_SIZE) {
        return false;
    }

    for (int arm_i = 0; arm_i < arm_size_; ++arm_i) {
        if (state.JointSize[arm_i] != joint_size_[arm_i] ||
            state.JointSize[arm_i] > MAX_JOINT_SIZE) {
            return false;
        }
        for (int joint_i = 0; joint_i < joint_size_[arm_i]; ++joint_i) {
            if (!std::isfinite(state.JointPos[arm_i][joint_i]) ||
                !std::isfinite(state.JointVel[arm_i][joint_i]) ||
                !std::isfinite(state.JointTor[arm_i][joint_i]) ||
                !std::isfinite(state.MotorPos[arm_i][joint_i]) ||
                !std::isfinite(state.MotorVel[arm_i][joint_i]) ||
                !std::isfinite(state.MotorTor[arm_i][joint_i])) {
                return false;
            }
        }
    }
    for (int gripper_i = 0; gripper_i < gripper_size_; ++gripper_i) {
        if (state.GripperJointSize[gripper_i] != gripper_joint_size_[gripper_i] ||
            state.GripperJointSize[gripper_i] > MAX_JOINT_SIZE) {
            return false;
        }
    }
    return true;
}

void MainWindow::synchronizeTargetsFromState(const PlannerState& state) {
    safe_panel_command_.updateCommand([&](PanelCommand& command) {
        command.motion_type = MotionControl::kJoint;
        command.target_type = ControlType::kPosition;
        command.actuator_mode = ControlType::kPosition;
        command.control_algorithm = ControlAlgorithm::kNone;
        command.arm_target_mode = PanelTargetMode::kSinglePoint;
        command.ArmWaypointSize = 0;
        command.reset_control_mem = true;
        command.reset_interpolation = true;
        command.need_setting_update = true;
        for (int arm_i = 0; arm_i < arm_size_; ++arm_i) {
            for (int joint_i = 0; joint_i < joint_size_[arm_i]; ++joint_i) {
                command.MotorCmdDeg[arm_i][joint_i] = state.MotorPos[arm_i][joint_i];
                command.JointCmdDeg[arm_i][joint_i] = state.JointPos[arm_i][joint_i];
            }
        }
    });
    emit setControlType(ControlType::kPosition, ControlType::kPosition);
    emit resetControlPage();
}

void MainWindow::beginHandshake(const PlannerState& state, const QString& reason) {
    session_synchronized_ = false;
    handshake_frames_remaining_ = 3;
    synchronizeTargetsFromState(state);
    safe_panel_command_.updateCommand([&](PanelCommand& command) {
        command.sequence_id = 0;
        command.connection_state = ConnectionState::kWaiting;
        required_ack_sequence_.store(0, std::memory_order_release);
        required_ack_valid_.store(true, std::memory_order_release);
    });
    statusBar()->showMessage(reason);
    if (diagnostic_dock_) {
        diagnostic_dock_->appendExternalError(tr("通信会话"), reason);
    }
}

bool MainWindow::consumeReceivedPlannerState() {
    std::shared_ptr<PlannerState> received;
    {
        std::lock_guard<std::mutex> lock(planner_state_mutex_);
        if (!transmit_planner_state_shared_ptr_) return false;
        received = std::move(transmit_planner_state_shared_ptr_);
    }

    const bool timestamp_reset =
        session_synchronized_ && last_rt_received_command_timestamp_ > 0 &&
        received->received_panel_command_timestamp == 0;
    const bool ack_regressed =
        session_synchronized_ && planner_ack_seen_ &&
        received->received_sequence_id < last_planner_ack_ &&
        last_planner_ack_ - received->received_sequence_id > 10;

    {
        std::lock_guard<std::mutex> lock(planner_state_mutex_);
        planner_state_shared_ptr_ = std::move(received);
    }
    if (diagnostic_dock_) {
        diagnostic_dock_->updatePlannerState(*planner_state_shared_ptr_);
    }

    if (timestamp_reset || ack_regressed) {
        beginHandshake(
            *planner_state_shared_ptr_,
            tr("检测到 RT Control 会话重启，已暂停控制并重新同步当前位姿"));
    } else if (!session_synchronized_) {
        synchronizeTargetsFromState(*planner_state_shared_ptr_);
    }

    planner_ack_seen_ = true;
    last_planner_ack_ = planner_state_shared_ptr_->received_sequence_id;
    last_rt_received_command_timestamp_ =
        planner_state_shared_ptr_->received_panel_command_timestamp;
    return true;
}

void MainWindow::sendPanelCmdAndAck(bool need_send){
    bool latest_target_received =
        planner_state_shared_ptr_ && isLatestTargetReceived();

    safe_panel_command_.updateCommand([&](PanelCommand& command){
        bool command_changed = need_send;
        if (last_actuator_mode_ != command.actuator_mode){
            command.need_setting_update = true;
            command_changed = true;
        }
        if (latest_target_received) {
            command_changed =
                command_changed || command.need_setting_update ||
                command.reset_interpolation || command.reset_control_mem;
            command.need_setting_update = false;
            command.reset_interpolation = false;
            command.reset_control_mem = false;
        }
        if (command_changed) {
            // The heartbeat thread will send this sequence after releasing
            // the command mutex. Track that command version instead of
            // requiring an exact match with the continuously advancing
            // heartbeat sequence.
            required_ack_sequence_.store(
                command.sequence_id, std::memory_order_release);
            required_ack_valid_.store(true, std::memory_order_release);
        }
    });

    if (latest_target_received && wait_robot_switch_mode_) {
        robotSetupFinished();
        resetControlPageToFront();
    }

}

void MainWindow::sendPanelCmdOnce(){
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        command.send_timestamp = get_time_now();
        
        //Convert from deg to rad for pos and vel.
        float a = (command.target_type != ControlType::kTorque)? float(M_PI/180) : 1;
        for (int arm_i=0; arm_i<arm_size_; arm_i++){
            for (int i=0; i < motor_size_[arm_i]; i++)
                command.MotorCmd[arm_i][i] = command.MotorCmdDeg[arm_i][i] * a;
            for (int i=0; i < joint_size_[arm_i]; i++)
                command.JointCmd[arm_i][i] = command.JointCmdDeg[arm_i][i] * a;
            for (int i=0; i < 6; i++){
                command.TaskCmd[arm_i][i]  = command.TaskCmdDeg[arm_i][i];
                if (i > 2)
                    command.TaskCmd[arm_i][i] *= (float)(M_PI/180);  //Last 3 variables are rotation
            }
        }

        for (int i = 0; i < command.ArmWaypointSize; i++){
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                for (int joint_i=0; joint_i < joint_size_[arm_i]; joint_i++){
                    command.ArmWaypointCmd[i][arm_i][joint_i] = command.ArmWaypointCmdDeg[i][arm_i][joint_i] * a;
                }
            }
        }
        udp_->send(&command);
        last_motion_control_ = command.motion_type;
        last_target_type_ = command.target_type;
        last_actuator_mode_ = command.actuator_mode;
        command.sequence_id++;
    });
}

bool MainWindow::isLatestTargetReceived(){
    if (!planner_state_shared_ptr_) return false;
    if (!required_ack_valid_.load(std::memory_order_acquire)) return false;

    const uint32_t required =
        required_ack_sequence_.load(std::memory_order_acquire);
    const uint32_t received = planner_state_shared_ptr_->received_sequence_id;
    // Signed subtraction handles uint32 sequence wrap as long as fewer than
    // 2^31 commands are outstanding.
    return static_cast<int32_t>(received - required) >= 0;
}

void MainWindow::sendPlannerStateToNdi(PlannerState* ps){
    // udp_pack_planner_state_to_ndi(p, ps);
    // udp_send(p);
    throw std::runtime_error("PlannerState connection_state was removed. "
        "Need to re-implement NDI connection. Should be using PanelState!\n");
}

QTabWidget* MainWindow::genNewTab(const std::vector<QString>& names){
    QTabWidget* tab = new QTabWidget();
    for (QString name: names){
        QWidget* page = genNewPage(name, tab);
        tab->addTab(page, name);
    }
    return tab;
}

QWidget* MainWindow::genNewPage(const QString& name, QWidget* parent){
    QWidget* page = new QWidget(parent);
    initialPage(page, name);
    return page;
}

void MainWindow::initialPage(QWidget* page, const QString& name){
    InitFnMap mapping;
    mapping[kControlPage] = &MainWindow::initialControlPage;
    mapping[kRecordPage] = &MainWindow::initialRecordPage;
    mapping[kReplayPage] = &MainWindow::initialReplayPage;
    mapping[kRemotePage] = &MainWindow::initialRemotePage;
    mapping[kStatePage] = &MainWindow::initialStatePage;
    mapping[kMotorEnable] = &MainWindow::initialMotorEnable;
    mapping[kQmlSimulation] = &MainWindow::initialQmlSimulation;
    // mapping[kPlotPage] = &MainWindow::initialPlotPage;
    InitFnMap::iterator x = mapping.find(name);
    if (x != mapping.end()){
        (this->*(x->second))(page, name);
    }
}

void MainWindow::initialControlPage(QWidget* page, const QString& name){
    ControlPageWidget* w = new ControlPageWidget(safe_panel_command_, planner_state_shared_ptr_, motor_size_, joint_limits_, config_);
    if (gripper_size_ > 0) w->addGripperWidget(gripper_names_, gripper_joint_names_);
    QVBoxLayout* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0,0,0,0);
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name,w));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));
}

void MainWindow::initialStatePage(QWidget* page, const QString& name){
    QVBoxLayout* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0,0,0,0);
    StateDisplayWidget* w = new StateDisplayWidget(planner_state_shared_ptr_, joint_size_);
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name,w));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));
}

void MainWindow::initialPlotPage(QWidget* page, const QString& name){
    QVBoxLayout* layout = new QVBoxLayout(page);
    plotWidget* w = new plotWidget();
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name,w));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));
}

void MainWindow::initialRecordPage(QWidget* page, const QString& name){
    ControlType actuator_mode;
    safe_panel_command_.updateCommand([&actuator_mode](PanelCommand& command){
        actuator_mode = command.actuator_mode;
    });
    QVBoxLayout* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0,0,0,0);
    RecordTrajectoryWidget* w = new RecordTrajectoryWidget(safe_panel_command_, planner_state_shared_ptr_,
        joint_size_, gripper_joint_size_, config_prefix_path_, actuator_mode);
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name,w));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));
}

void MainWindow::initialReplayPage(QWidget* page, const QString& name){
    ControlType actuator_mode;
    safe_panel_command_.updateCommand([&actuator_mode](PanelCommand& command){
        actuator_mode = command.actuator_mode;
    });
    QWidget* content = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(content);
    ReplayTrajectoryWidget* w = new ReplayTrajectoryWidget(safe_panel_command_, planner_state_shared_ptr_, 
        joint_size_, gripper_joint_size_, config_prefix_path_, actuator_mode);
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name,w));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));

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
}

void MainWindow::initialRemotePage(QWidget* page, const QString& name){
    QVBoxLayout* layout = new QVBoxLayout(page);
    layout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
    RemotePageWidget* w = new RemotePageWidget(safe_panel_command_, planner_state_shared_ptr_, config_);
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name,w));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));
}

void MainWindow::initialMotorEnable(QWidget* page, const QString& name){
    QVBoxLayout* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0,0,0,0);
    layout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
    JointEnableWidget* w = new JointEnableWidget(safe_panel_command_, config_);
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name, nullptr));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));
}

void MainWindow::initialQmlSimulation(QWidget* page, const QString& name){
    QVBoxLayout* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0,0,0,0);
    QmlSimulationWidget* w = new QmlSimulationWidget(planner_state_shared_ptr_, config_, config_prefix_path_);
    layout->addWidget(w);
    control_widgets_.insert(std::pair<QString, ControlInterface*>(name, nullptr));
    all_widgets_.insert(std::pair<QString, QWidget*>(name, w));
}

//Called whenever the main tab is changed.
void MainWindow::slotTabWCurrentChanged(int i){
    if (control_widgets_.size() < totalWidget){
        return;
    }
    if (control_widgets_[current_tab_name_])
        control_widgets_[current_tab_name_]->setInFront(false);
    if (control_widgets_[kMainTabOrder[i]])
        current_tab_name_ = kMainTabOrder[i];
        control_widgets_[current_tab_name_]->setInFront(true);
}

void MainWindow::slotStatusBarMessageChanged(bool show, const QString& msg){
    if (!show){
        this->statusBar()-> showMessage("");
    }else{
        this->statusBar()-> showMessage(msg);
    }
}

void MainWindow::slotErrorOccured(QString title, QString msg){
    if (diagnostic_dock_) {
        diagnostic_dock_->appendExternalError(title, msg);
    }
    QMessageBox messageBox;
    messageBox.critical(this, title, msg);
}

void MainWindow::slotConnectUDP(bool connect){
    waitForRobotSetup();
}

void MainWindow::slotJointEnableChanged(){
    waitForRobotSetup();
}

void MainWindow::waitForRobotSetup(){
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        saved_target_type_for_setup_ = command.target_type;
        saved_actuator_mode_for_setup_ = command.actuator_mode;
        command.motion_type = MotionControl::kJoint;
    });
    // DualV2_2 deliberately rejects velocity mode. Enter hardware control by
    // holding the latest measured position, then restore the user's mode only
    // after RT Control acknowledges the setup command.
    if (planner_state_shared_ptr_) {
        synchronizeTargetsFromState(*planner_state_shared_ptr_);
    } else {
        emit setControlType(ControlType::kPosition, ControlType::kPosition);
    }
    wait_robot_switch_mode_ = true;
}

void MainWindow::robotSetupFinished(){
    emit setControlType(saved_target_type_for_setup_, saved_actuator_mode_for_setup_);
    wait_robot_switch_mode_ = false;
}

void MainWindow::slotControlTypeChanged(ControlType top, ControlType base){
    QString s1 = QString::fromStdString(enumToString<ControlType>(top));
    QString s2 = QString::fromStdString(enumToString<ControlType>(base));
    QString txt = QString("Tar: %1, Act: %2").arg(s1).arg(s2);
    control_type_display_->setText(txt);
}

// Set arm control be the main tab
void MainWindow::resetControlPageToFront(){
    if (control_widgets_.size() < totalWidget){
        return;
    }
    emit resetControlPage();
    if (current_tab_name_ != kControlPage){
        int i = std::find(kMainTabOrder.begin(), kMainTabOrder.end(), kControlPage) - kMainTabOrder.begin();
        main_page_tab_w_->setCurrentIndex(i);
    }
}

void MainWindow::slotRemoteStartButtonClicked(bool remote_start){
    bool disable_pages = remote_start;
    for (int i = 0; i < main_page_tab_w_->count(); ++i) {
        if (main_page_tab_w_->tabText(i) != kRemotePage) {
            main_page_tab_w_->setTabEnabled(i, !disable_pages);
        }
    }
}