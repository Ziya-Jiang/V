#ifndef RECORD_AND_REPLAY_BASE_H
#define RECORD_AND_REPLAY_BASE_H  

#include <vector>
#include <map>
#include <string>
#include <variant>

#include <QWidget>

#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/thread_safe_command.h"

class RecordAndReplayBase: public QWidget{
    Q_OBJECT
    public:
        RecordAndReplayBase(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& joint_size, const std::vector<int>& gripper_joint_size, 
                            const std::string& config_prefix_path, QWidget* parent = 0);
    protected:
        ThreadSafeCommand<PanelCommand>& safe_panel_command_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        const std::string config_prefix_path_;
        std::string save_path_;

        struct ComponentData{
            int component_size;
            std::vector<int> joint_size;
            std::vector<VectorXf> cur_joint_pos;
            std::vector<VectorXf> joint_cmd;
            // float (*state_ref)[MAX_JOINT_SIZE];
            // float (*cmd_ref)[MAX_JOINT_SIZE];
            float (PlannerState::*state_ref)[MAX_ARM_SIZE][MAX_JOINT_SIZE];
            float (PanelCommand::*cmd_ref)[MAX_ARM_SIZE][MAX_JOINT_SIZE];
        };
        std::map<ComponentType, ComponentData> component_map_;
        const std::vector<ComponentType> component_order_ = {ComponentType::kArm, ComponentType::kGripper};
        void createFolderIfNotExist(std::string directory);
        void writeToText(const std::vector<std::map<ComponentType, std::vector<VectorXf>>>& data, const std::string& file_path);
        void writeToText(const std::vector<std::vector<Vector7f>>& data, const std::string& file_path, const std::string& header="");
        std::vector<ComponentType> extractComponentOrder(const std::string& header);
        std::vector<std::map<ComponentType, std::vector<VectorXf>>> extractFromText(const std::string& file_path);
    signals:
        void setControlType(ControlType top, ControlType base);
    protected slots:
        void slotControlTypeChanged(ControlType top, ControlType base);
};

#endif