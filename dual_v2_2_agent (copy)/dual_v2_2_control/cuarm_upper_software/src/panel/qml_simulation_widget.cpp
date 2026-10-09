#include "qml_simulation_widget.h"

#include <fstream>

#include <urdf_parser/urdf_parser.h>

#include <Qt3DQuickExtras/qt3dquickwindow.h>
#include <Qt3DQuick/QQmlAspectEngine>
#include <QQmlEngine>
#include <QQmlContext>

#include "config_loader.h"

QmlSimulationWidget::QmlSimulationWidget(std::shared_ptr<PlannerState>& ps, const YAML::Node& config, const std::string& config_prefix_path, QWidget *parent) : 
    planner_state_shared_ptr_(ps), QQuickWidget(parent), config_(config){
    // printYamlNode(config);
    arm_size_ = config["robot"]["arm"].size();
    pose_.reserve(arm_size_);
    for (int arm_i=0; arm_i < arm_size_; arm_i++){
        arm_joint_size_.push_back(config["robot"]["arm"][arm_i]["joint_size"].as<int>());
        pose_.push_back(MatrixX7f::Zero(arm_joint_size_[arm_i], 7));
    }

    std::string urdf_path;
    urdf_path = config_prefix_path + "/" + config_["robot"]["urdf"].as<std::string>();
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_path);
    if (!model) {
        throw std::runtime_error("Failed to parse URDF: " + urdf_path);
    }
    urdf_model_ = model;
    
    if (config_["robot"]["gripper"]){
        YAML::Node gripper_node = config_["robot"]["gripper"];
        for (int gri_i=0; gri_i < gripper_node.size(); gri_i++){
            if (gripper_node[gri_i]["urdf_joints"]){
                for (int ji=0; ji<gripper_node[gri_i]["urdf_joints"].size(); ji++){
                    std::string j_name = gripper_node[gri_i]["urdf_joints"][ji].as<std::string>();
                    gripper_joint_map_[j_name] = {gri_i, ji};
                }
            }
        }
    }
    
    std::string mesh_dir = "share/meshes/";
    std::filesystem::path prefix(config_prefix_path);
    std::filesystem::path sub_dir(mesh_dir);
    auto addLinkMesh = [&](std::string link_name, bool is_kFixed, std::string relative_to, std::vector<LinkMesh>& meshes) {
        urdf::LinkConstSharedPtr link = model->getLink(link_name);
        if (!link || link->visual_array.empty()) return;
        
        LinkMesh mesh;
        mesh.name = link->name;
        mesh.type = is_kFixed ? JointType::kFixed : JointType::kRevolute;
        mesh.relative_to_link = relative_to;
        
        for (size_t vis_i = 0; vis_i < link->visual_array.size(); vis_i++) {
            urdf::VisualConstSharedPtr visual = link->visual_array[vis_i];
            if (!visual || !visual->geometry) continue;
            
            if (visual->geometry->type == urdf::Geometry::MESH) {
                urdf::Pose transform = visual->origin;
                Eigen::Vector3f v_pos(transform.position.x, transform.position.y, transform.position.z);
                Eigen::Quaternionf v_rot(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
                
                LinkMesh sub_mesh = mesh;
                sub_mesh.position_offset = v_pos;
                sub_mesh.orientation_offset = v_rot;
                
                urdf::MeshConstSharedPtr urdf_mesh = urdf::dynamic_pointer_cast<const urdf::Mesh>(visual->geometry);
                if (urdf_mesh) {
                    std::filesystem::path filePath = urdf_mesh->filename;
                    std::filesystem::path absolute_mesh_path = prefix / sub_dir / getUrdfMeshPath(filePath, "meshes");
                    sub_mesh.mesh_source = absolute_mesh_path.make_preferred().string();
                    sub_mesh.mesh_scale = Vector3f(urdf_mesh->scale.x, urdf_mesh->scale.y, urdf_mesh->scale.z);
                    if (sub_mesh.mesh_source != "") meshes.push_back(sub_mesh);
                }
            }
        }
    };

    for (int arm_i=0; arm_i < arm_size_; arm_i++){
        std::vector<LinkMesh> single_arm_meshes;

        // If arm 0, add base links first
        if (arm_i == 0 && config_["robot"]["base"] && config_["robot"]["base"]["urdf_links"]) {
            std::string root_base = config_["robot"]["base"]["urdf_links"][0].as<std::string>();
            for (int i=0; i<config_["robot"]["base"]["urdf_links"].size(); i++) {
                std::string l_name = config_["robot"]["base"]["urdf_links"][i].as<std::string>();
                addLinkMesh(l_name, true, (i==0) ? "" : root_base, single_arm_meshes);
            }
        }

        // Add arm links
        std::string last_revolute_link = "";
        if (config_["robot"]["arm"][arm_i]["urdf_links"]) {
            int link_size = config_["robot"]["arm"][arm_i]["urdf_links"].size();
            for (int i=0; i<link_size; i++) {
                bool is_kFixed = (i >= arm_joint_size_[arm_i]);
                std::string l_name = config_["robot"]["arm"][arm_i]["urdf_links"][i].as<std::string>();
                
                std::string rel = (is_kFixed) ? last_revolute_link : "";
                addLinkMesh(l_name, is_kFixed, rel, single_arm_meshes);
                
                if (!is_kFixed) {
                    last_revolute_link = l_name;
                }
            }
        }
        
        // Recursively find and add all descendants of last_revolute_link (covers grippers/hands)
        if (last_revolute_link != "") {
            urdf::LinkConstSharedPtr root_link = model->getLink(last_revolute_link);
            if (root_link) {
                std::vector<urdf::LinkConstSharedPtr> open_list;
                for (const auto& child_link : root_link->child_links) {
                    open_list.push_back(child_link);
                }
                
                while(!open_list.empty()) {
                    urdf::LinkConstSharedPtr curr = open_list.front();
                    open_list.erase(open_list.begin());
                    
                    if (curr) {
                        bool already_added = false;
                        for (const auto& m : single_arm_meshes) {
                            if (m.name == curr->name) {
                                already_added = true; break;
                            }
                        }
                        if (!already_added) {
                            addLinkMesh(curr->name, true, last_revolute_link, single_arm_meshes);
                        }
                        for (const auto& child_link : curr->child_links) {
                            open_list.push_back(child_link);
                        }
                    }
                }
            }
        }
        
        robot_mesh_.arm_meshes.push_back(single_arm_meshes);
    }

    this->setContentsMargins(20,20,20,0);
    this->setResizeMode(QQuickWidget::SizeRootObjectToView);
    this->rootContext()->setContextProperty("cpp_program", this);
    this->setSource(QUrl("qrc:/main.qml"));
}

int QmlSimulationWidget::qmlGetArmSize(){
    return arm_size_;
}

float QmlSimulationWidget::getUrdfJointPosition(const std::string& joint_name){
    auto it = cached_joint_positions_.find(joint_name);
    if (it != cached_joint_positions_.end()) {
        return it->second;
    }

    float value = 0.0f;
    if (urdf_model_) {
        urdf::JointConstSharedPtr joint = urdf_model_->getJoint(joint_name);
        if (joint && joint->mimic && joint->mimic->joint_name != joint_name) {
            float base_val = getUrdfJointPosition(joint->mimic->joint_name);
            value = base_val * joint->mimic->multiplier + joint->mimic->offset;
            cached_joint_positions_[joint_name] = value;
            return value;
        }
    }
    if (gripper_joint_map_.find(joint_name) != gripper_joint_map_.end()){
        auto info = gripper_joint_map_[joint_name];
        float normalized_val = planner_state_shared_ptr_->GripperJointPos[info.gri_i][info.ji];
        
        if (urdf_model_) {
            urdf::JointConstSharedPtr joint = urdf_model_->getJoint(joint_name);
            if (joint && joint->limits) {
                float lower = joint->limits->lower;
                float upper = joint->limits->upper;
                normalized_val = lower + normalized_val * (upper - lower);
            }
        }
        value = normalized_val;
    }
    cached_joint_positions_[joint_name] = value;
    return value;
}

int QmlSimulationWidget::qmlGetMeshCnt(int arm_i){
    return robot_mesh_.arm_meshes[arm_i].size();
}

QList<float> QmlSimulationWidget::qmlGetSimuJointPose(int arm_i, int i){
    if (i >= robot_mesh_.arm_meshes[arm_i].size()) return {0,0,0,1,0,0,0};
    
    // Find the closest active revolute joint index for this mesh.
    int joint_idx = -1;
    for(int idx = 0; idx <= i; ++idx) {
        if(robot_mesh_.arm_meshes[arm_i][idx].type == JointType::kRevolute) {
            joint_idx++;
        }
    }
    
    // Safety check - cap to the last valid joint if needed
    if (joint_idx >= arm_joint_size_[arm_i]) {
        joint_idx = arm_joint_size_[arm_i] - 1;
    }
    
    Eigen::Vector3f P;
    Eigen::Matrix3f R;
    
    if (joint_idx >= 0) {
        pose_[arm_i].row(joint_idx) = Eigen::Map<Eigen::VectorXf>(planner_state_shared_ptr_->JointPose[arm_i][joint_idx], 7);
        VectorXf vec = pose_[arm_i].row(joint_idx);
        P = vec.head(3);
        Eigen::Quaternionf quat(vec[3], vec[4], vec[5], vec[6]);
        R = quat.toRotationMatrix();
    } else {
        P = Eigen::Vector3f::Zero();
        R = Eigen::Matrix3f::Identity();
    }
    
    const LinkMesh& mesh_link = robot_mesh_.arm_meshes[arm_i][i];
    
    // Start with the geometric visual offset
    Eigen::Vector3f k_pos = mesh_link.position_offset;
    Eigen::Quaternionf k_rot = mesh_link.orientation_offset;
    
    if (urdf_model_ && mesh_link.relative_to_link != "") {
        urdf::LinkConstSharedPtr curr = urdf_model_->getLink(mesh_link.name);
        while (curr && curr->name != mesh_link.relative_to_link) {
            if (curr->parent_joint) {
                urdf::Pose t = curr->parent_joint->parent_to_joint_origin_transform;
                Eigen::Vector3f p(t.position.x, t.position.y, t.position.z);
                Eigen::Quaternionf r(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
                
                if (curr->parent_joint->type == urdf::Joint::REVOLUTE || curr->parent_joint->type == urdf::Joint::CONTINUOUS) {
                    float j_val = getUrdfJointPosition(curr->parent_joint->name);
                    Eigen::Vector3f axis(curr->parent_joint->axis.x, curr->parent_joint->axis.y, curr->parent_joint->axis.z);
                    r = r * Eigen::Quaternionf(Eigen::AngleAxisf(j_val, axis));
                } else if (curr->parent_joint->type == urdf::Joint::PRISMATIC) {
                    float j_val = getUrdfJointPosition(curr->parent_joint->name);
                    Eigen::Vector3f axis(curr->parent_joint->axis.x, curr->parent_joint->axis.y, curr->parent_joint->axis.z);
                    p = p + r * (axis * j_val);
                }
                
                k_pos = p + r * k_pos;
                k_rot = r * k_rot;
                
                curr = urdf_model_->getLink(curr->parent_joint->parent_link_name);
            } else {
                break;
            }
        }
    }
    
    Eigen::Vector3f P1 = P + R*k_pos;
    Eigen::Quaternionf quat1 = Eigen::Quaternionf(R*k_rot.toRotationMatrix());
    QList<float> jp = {P1[0], P1[1], P1[2], quat1.w(), quat1.x(), quat1.y(), quat1.z()};
    return jp;
}

QString QmlSimulationWidget::qmlGetMeshSource(int arm_i, int i){
    std::string raw_path = robot_mesh_.arm_meshes[arm_i][i].mesh_source;
    return QUrl::fromLocalFile(QString::fromStdString(raw_path)).toString();
}

QList<float> QmlSimulationWidget::qmlGetMeshScale(int arm_i, int i){
    if (i >= robot_mesh_.arm_meshes[arm_i].size()) return {1, 1, 1};
    const LinkMesh& mesh = robot_mesh_.arm_meshes[arm_i][i];
    return {mesh.mesh_scale[0], mesh.mesh_scale[1], mesh.mesh_scale[2]};
}

float QmlSimulationWidget::qmlGetCameraFieldOfView(){
    return config_["panel"]["qml"]["camera"]["field_of_view"].as<float>();
}

QList<float> QmlSimulationWidget::qmlGetCameraPosition(){
    QList<float> res(3);
    for (int i=0; i<3; i++){
        res[i] = config_["panel"]["qml"]["camera"]["position"][i].as<float>();
    }
    return res;
}

QList<float> QmlSimulationWidget::qmlGetCameraViewCenter(){
    QList<float> res(3);
    for (int i=0; i<3; i++){
        res[i] = config_["panel"]["qml"]["camera"]["view_center"][i].as<float>();
    }
    return res;
}

std::string QmlSimulationWidget::getUrdfMeshPath(const std::filesystem::path& p, const std::string& keyword){
    bool found = false;
    std::filesystem::path result;

    for (const auto& part : p) {
        if (found) {
            result /= part;  // accumulate remaining parts
        } else if (part == keyword) {
            found = true;    // start collecting after keyword
        }
    }
    return result.generic_string(); // normalized with '/'
}

std::string QmlSimulationWidget::readFileAsString(const std::string& filePath){
    std::ifstream file(filePath);
    if (!file){
        throw std::runtime_error("Could not open file: " + filePath);
    }
    std::ostringstream buffer;
    buffer << file.rdbuf(); // Read entire file into buffer
    return buffer.str();
}

void QmlSimulationWidget::slotUpdatePose(){
    cached_joint_positions_.clear();
    emit poseChanged();
}
