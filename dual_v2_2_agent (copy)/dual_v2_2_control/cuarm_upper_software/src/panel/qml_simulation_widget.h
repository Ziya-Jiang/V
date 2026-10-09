#ifndef QMLSIMULATIONWIDGET_H
#define QMLSIMULATIONWIDGET_H

#include <filesystem>
#include <yaml-cpp/yaml.h>

#include <QQuickWidget>
#include <QWidget>

#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include <urdf_parser/urdf_parser.h>

enum class JointType{
    kFixed = 0,
    kRevolute = 1,
    // kPrismatic = 1,
};

struct LinkMesh{
    std::string name;
    JointType type = JointType::kFixed;
    Vector3f position_offset;
    Eigen::Quaternionf orientation_offset;
    std::string mesh_source = "";
    std::string relative_to_link = ""; // Add relative_to link reference
    Vector3f mesh_scale = Vector3f::Ones(); // mesh scale from URDF (default 1,1,1)
};

struct RobotMesh{
    std::vector<std::vector<LinkMesh>> arm_meshes;
};

class QmlSimulationWidget : public QQuickWidget{
    Q_OBJECT
    public:
        QmlSimulationWidget(std::shared_ptr<PlannerState>& ps, const YAML::Node& config, const std::string& config_prefix_path, QWidget *parent = 0);
        Q_PROPERTY(std::vector<MatrixX7f> rpose MEMBER pose_ NOTIFY poseChanged);
        Q_INVOKABLE int qmlGetArmSize();
        Q_INVOKABLE int qmlGetMeshCnt(int arm_i);
        Q_INVOKABLE QList<float> qmlGetSimuJointPose(int arm_i, int i);
        Q_INVOKABLE QString qmlGetMeshSource(int arm_i, int i);
        Q_INVOKABLE QList<float> qmlGetMeshScale(int arm_i, int i);
        Q_INVOKABLE float qmlGetCameraFieldOfView();
        Q_INVOKABLE QList<float> qmlGetCameraPosition();
        Q_INVOKABLE QList<float>  qmlGetCameraViewCenter();

    private:
        std::string getUrdfMeshPath(const std::filesystem::path& p, const std::string& keyword);
        std::string readFileAsString(const std::string& filePath);
        
        const YAML::Node& config_;
        std::shared_ptr<PlannerState>& planner_state_shared_ptr_;
        urdf::ModelInterfaceSharedPtr urdf_model_;

        struct GripperJointInfo {
            int gri_i;
            int ji;
        };
        std::map<std::string, GripperJointInfo> gripper_joint_map_;
        std::map<std::string, float> cached_joint_positions_;
        float getUrdfJointPosition(const std::string& joint_name);

        int arm_size_ = 0;
        std::vector<int> arm_joint_size_;
        std::vector<MatrixX7f> pose_;
        RobotMesh robot_mesh_;
        
    signals:
        void poseChanged();

    private slots:
        void slotUpdatePose();
};

#endif