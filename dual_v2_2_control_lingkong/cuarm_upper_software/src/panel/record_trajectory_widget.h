#ifndef RECORD_TRAJECTORY_WIDGET_H
#define RECORD_TRAJECTORY_WIDGET_H

#include <vector>
#include <map>
#include <string>

#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QFileDialog>
#include <QString>

#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"
#include "control_interface.h"
#include "record_and_replay_base.h"
#include <common_widget.h>

class RecordTrajectoryWidget: public RecordAndReplayBase, public ControlInterface{
    Q_OBJECT
    public:
        RecordTrajectoryWidget(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& joint_size, const std::vector<int>& gripper_joint_size, 
                               const std::string& config_prefix_path, ControlType actuator_mode, QWidget* parent=0);
        void setInFront(bool in_front);
        bool sendVal();
    private:
        ControlType actuator_mode_;
        QPushButton* start_button_;
        QPushButton* savePt_button_;
        QPushButton* finish_button_;
        QPushButton* cancel_button_;
        QLineEdit* file_line_edit_;
        QLabel* last_pt_;
        QLabel* total_pt_;
        FormWidget* option_form_w_;
        std::vector<std::map<ComponentType, std::vector<VectorXf>>> data_;
        std::map<QString, bool> option_map_; 

        std::vector<Vector7f> cur_cartesian_pose_; //Arm size x joint size
        std::vector<std::vector<Vector7f>> cartesian_pose_;

        void resizeEvent(QResizeEvent *event) override;
        void setRecording(bool record, const QString& file_path="");
    signals:
        void recordingStateChanged(bool enable);
    protected slots:
        void slotStartButtonClicked();
        void slotSavePointButtonClicked();
        void slotFinishButtonClicked();
        void slotCancelButtonClicked();
        void slotFileLineTextChanged(const QString& text);
        void slotSaveFileClicked(QFileDialog* dialog);
        void slotOptionFormCheckBoxChanged(const QString& label, int clicked);
};

#endif