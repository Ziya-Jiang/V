#ifndef REMOTE_PAGE_WIDGET_H
#define REMOTE_PAGE_WIDGET_H

#include <thread>
#include <mutex>
#include <memory>
#include "remote_command_lease.h"

#include <QLayout>
#include <QPushButton>
#include <QWidget>
#include <QString>
#include <yaml-cpp/yaml.h>

#include "control_interface.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

class RemotePageWidget: public QWidget, public ControlInterface{
    Q_OBJECT
    public:
        RemotePageWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const YAML::Node& config, QWidget* parent = 0);
        ~RemotePageWidget();
        void setInFront(bool in_front);
        bool sendVal();
        bool connectAndStartAutomatically();
    private:
        void addConfigLayout(QGridLayout* grid_layout, int row_index, QString label_text, QString default_ip, QString default_port);
        void connectRemote();
        void disconnectRemote();
        void start();
        void stop();
        void clearJointAndGripperCommand();
        void updateJointAndGripperCommand(const RemotePanelCommand& remote_cmd);
        void updatePanelState();
        void updateIpPort(int row_index, std::string* ip_ptr, int* port_ptr);
        void udpReceiveTask(int dt_ms);
        bool isValidRemoteCommand(const RemotePanelCommand& remote_cmd);
        void resetConfigSetting();

        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        PanelState panel_state_;
        ControlType last_set_target_type_ = ControlType::kInvalid;
        ControlType last_set_actuator_mode_ = ControlType::kInvalid;
        ControlType config_target_type_ = ControlType::kInvalid;
        ControlType config_actuator_mode_ = ControlType::kInvalid;
        float config_acceleration_time_ = 0.0;
        float config_saturation_ratio_ = 0.0;

        QWidget* ip_port_widget_;
        QPushButton* connect_bnt_;
        QPushButton* start_bnt_;
        bool is_connected_ = false;
        bool is_running_ = false;
        bool set_stop_joint_command_ = false;
        std::string local_ip_;
        std::string remote_ip_;
        int local_port_;
        int remote_port_;

        bool thread_terminate_ = false;
        std::unique_ptr<CuarmUdp<RemotePanelCommand, PanelState>> udp_unique_ptr_ = nullptr;
        std::thread udp_receive_thread_;
        std::mutex received_mutex_;
        double received_remote_at_ = 0.;
        RemoteCommandLease command_lease_;
        std::unique_ptr<RemotePanelCommand> received_remote_panel_command_unique_ptr_;
        std::unique_ptr<RemotePanelCommand> remote_panel_command_unique_ptr_;
    private slots:
        void slotConnectButtonClicked(bool clicked);
        void slotStartButtonClicked(bool clicked);
    signals:
        void startButtonClicked(bool set_start);
        void setControlType(ControlType top, ControlType base);
};

#endif