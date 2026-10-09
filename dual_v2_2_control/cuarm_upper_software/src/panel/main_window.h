#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H
#include <iostream>
#include <type_traits>
#include <cstdio>
#include <unordered_map>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <memory>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <cstddef> // for std::nullptr_t

#include <yaml-cpp/yaml.h>
#include <QFrame>
#include <QGridLayout>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QString>
#include <QStringList>
#include <QTabWidget>
#include <QThread>
#include <QTimer>
#include <QWidget>
#include <QList>
#include <QLabel>

// #include "plot_manager.h"
#include "ros_robot_state.h"
#include "state_display_widget.h"
#include "control_interface.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

class RemotePageWidget;

namespace cuarm_panel{
    class ArmDiagnosticDock;

    class MainWindow : public QMainWindow{
        Q_OBJECT
        public:
            MainWindow(const std::string& config_prefix_path, const std::string& project_base_path, bool headless_auto = false, QWidget* parent = 0);
            ~MainWindow();
            void setArmControlTab();
            void startHeadlessAuto();
        private:
            bool expand_;
            QThread* heartbeat_thread_;
            QThread* receive_thread_;
            QThread* ndi_thread_;
            QTimer* timer_;
            QTimer* heartbeat_timer_;
            QTimer* receive_timer_;
            QTimer* ndi_timer_;
            float dt_receive_;
            float dt_send_;
            std::string config_prefix_path_;
            std::string config_file_;
            bool headless_auto_ = false;
            bool headless_auto_remote_pending_ = false;
            YAML::Node config_;
            std::unique_ptr<CuarmUdp<PlannerState, PanelCommand>> udp_;
            std::unique_ptr<CuarmUdp<std::nullptr_t, PanelState>> udp_ndi_;
            bool ndi_enabled_;
            std::shared_ptr<PlannerState> ndi_planner_state_shared_ptr_;
            std::shared_ptr<PlannerState> planner_state_shared_ptr_;
            std::shared_ptr<PlannerState> transmit_planner_state_shared_ptr_;
            std::mutex planner_state_mutex_;
            ThreadSafeCommand<PanelCommand> safe_panel_command_;
            bool session_synchronized_ = false;
            int handshake_frames_remaining_ = 0;
            bool planner_ack_seen_ = false;
            uint32_t last_planner_ack_ = 0;
            long last_rt_received_command_timestamp_ = 0;
            std::atomic<uint32_t> required_ack_sequence_{0};
            std::atomic<bool> required_ack_valid_{false};

            QString current_tab_name_;
            QTabWidget* main_page_tab_w_;
            RemotePageWidget* remote_page_widget_ = nullptr;
            // PlotManager* plot_manager_;

            std::atomic<MotionControl> last_motion_control_{MotionControl::kNull};
            std::atomic<ControlType> last_target_type_{ControlType::kInvalid};
            std::atomic<ControlType> last_actuator_mode_{ControlType::kInvalid};
            bool wait_robot_switch_mode_ = false;
            ControlType   saved_target_type_for_setup_;
            ControlType   saved_actuator_mode_for_setup_;
            QLabel* control_type_display_;
            ArmDiagnosticDock* diagnostic_dock_ = nullptr;
            
            //Arm related 
            int arm_size_;
            std::vector<int> motor_size_;
            std::vector<int> joint_size_;
            std::vector<Matrix2Xf> joint_limits_;

            //Gripper related
            int gripper_size_ = 0;
            std::vector<std::string> gripper_names_;
            std::vector<std::vector<std::string>> gripper_joint_names_;
            std::vector<int> gripper_joint_size_;

            std::map<const QString, ControlInterface*> control_widgets_;
            std::map<const QString, QWidget*> all_widgets_;
            void initCommandAndState(const YAML::Node& config);
            QTabWidget* genNewTab(const std::vector<QString>& names);
            QWidget* genNewPage(const QString& name, QWidget* parent=0);
            void initialPage(QWidget* page, const QString& name);
            void initialControlPage(QWidget* page, const QString& name);
            void initialStatePage(QWidget* page, const QString& name);
            void initialPlotPage(QWidget* page, const QString& name);
            void initialRecordPage(QWidget* page, const QString& name);
            void initialReplayPage(QWidget* page, const QString& name);
            void initialRemotePage(QWidget* page, const QString& name);
            void initialMotorEnable(QWidget* page, const QString& name);
            void initialQmlSimulation(QWidget* page, const QString& name);
            bool receivePlannerState();
            bool consumeReceivedPlannerState();
            bool validatePlannerState(const PlannerState& state) const;
            void synchronizeTargetsFromState(const PlannerState& state);
            void beginHandshake(const PlannerState& state, const QString& reason);
            void sendPanelCmdAndAck(bool need_send);
            void sendPanelCmdOnce();
            void sendPlannerStateToNdi(PlannerState* ps);
            void waitForRobotSetup();
            void robotSetupFinished();
            void resetControlPageToFront();
            bool isLatestTargetReceived();
                
        signals:
            void robotStateChanged(RosRobotState state);
            void poseChanged();
            void setControlType(ControlType top, ControlType base);
            void resetControlPage();

        private slots:
            void slotTimerTimeout();
            void slotHeartbeatTimerTimeout();
            void slotNdiTimerTimeout();
            void slotReceiveTimerTimeout();
            void slotTabWCurrentChanged(int i);
            void slotStatusBarMessageChanged(bool show, const QString& msg);
            void slotErrorOccured(QString title, QString msg);
            void slotConnectUDP(bool connect);
            void slotControlTypeChanged(ControlType top, ControlType base);
            void slotRemoteStartButtonClicked(bool remote_start);
            void slotJointEnableChanged();
    };
    typedef void (MainWindow::*InitialFn)(QWidget*, const QString&);
    typedef std::map<const QString, InitialFn> InitFnMap;
}

#endif // MAINWINDOW_H
