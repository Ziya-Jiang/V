#ifndef REPLAY_TRAJECTORY_WIDGET_H
#define REPLAY_TRAJECTORY_WIDGET_H

#include <vector>
#include <map>
#include <string>
#include <deque>

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QFileDialog>
#include <QString>

#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"
#include "common_widget.h"
#include "reset_save_block.h"
#include "control_interface.h"
#include "record_and_replay_base.h"
#include "replay_point_group_widget.h"

enum ReplayState{kStart, kRunning, kRest, kFinish};
class ReplayTrajectoryWidget: public RecordAndReplayBase, public ControlInterface, public AbstractResetSave{
    Q_OBJECT
    public:
        ReplayTrajectoryWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& joint_size, const std::vector<int>& gripper_joint_size, 
                               const std::string& config_prefix_path, ControlType actuator_mode, QWidget* parent = 0);
        void setInFront(bool in_front);
        bool sendVal();
        void reset();
        void save();
    private:
        ControlType actuator_mode_;
        int points_total_ = 0;
        int points_count_ = 0;
        QLabel* counter_;
        ReplayPointGroupWidget* pts_group_;
        QLineEdit* file_line_edit_;
        FormWidget* rest_form_;
        QWidget* button_container_;
        QPushButton* stop_button_;
        ReplayState state_ = kFinish;
        float rest_time_ = 0.5;
        long start_time_ = 0;
        std::vector<std::map<ComponentType, std::vector<VectorXf>>> data_;
        std::deque<std::map<ComponentType, std::vector<VectorXf>>> trajectory_;
        void displayPoints(std::string file_name);
        void getNextTrajPoint();
    signals:
        void setControlType(ControlType top, ControlType base);
    protected slots:
        void slotRestFormTextChanged(const QString& label, const QString& text);
        void slotSetStartButtonClicked();
        void slotSetEndButtonClicked();
        void slotInsertButtonClicked(bool above);
        void slotDeleteButtonClicked();
        void slotRunButtonClicked();
        void slotStopButtonClicked();
        void slotOpenFileClicked(QFileDialog* dialog);
};

#endif