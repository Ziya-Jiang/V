#include "record_and_replay_base.h"

#include <iostream>
#include <fstream>
#include <string>

#include <boost/filesystem.hpp>

RecordAndReplayBase::RecordAndReplayBase(ThreadSafeCommand<PanelCommand>& safe_panel_command, std::shared_ptr<PlannerState>& ps, const std::vector<int>& joint_size, const std::vector<int>& gripper_joint_size, 
                                         const std::string& config_prefix_path, QWidget* parent): safe_panel_command_(safe_panel_command), planner_state_shared_ptr_(ps), QWidget(parent), config_prefix_path_(config_prefix_path){
    save_path_ = config_prefix_path + "/panel/replay";
    createFolderIfNotExist(save_path_);
    
    component_map_[ComponentType::kArm].component_size = joint_size.size();
    component_map_[ComponentType::kArm].state_ref = &PlannerState::JointPos;
    component_map_[ComponentType::kArm].cmd_ref = &PanelCommand::JointCmdDeg;
    for (int arm_i=0; arm_i<joint_size.size(); arm_i++){
        component_map_[ComponentType::kArm].joint_size.push_back(joint_size[arm_i]);
        component_map_[ComponentType::kArm].cur_joint_pos.push_back(VectorXf::Zero(joint_size[arm_i]));
        component_map_[ComponentType::kArm].joint_cmd.push_back(VectorXf::Zero(joint_size[arm_i]));
    }

    component_map_[ComponentType::kGripper].component_size = gripper_joint_size.size();
    component_map_[ComponentType::kGripper].state_ref = &PlannerState::GripperJointPos;
    component_map_[ComponentType::kGripper].cmd_ref = &PanelCommand::GripperJointCmd;
    for (int gri_i=0; gri_i<gripper_joint_size.size(); gri_i++){
        component_map_[ComponentType::kGripper].joint_size.push_back(gripper_joint_size[gri_i]);
        component_map_[ComponentType::kGripper].cur_joint_pos.push_back(VectorXf::Zero(gripper_joint_size[gri_i]));
        component_map_[ComponentType::kGripper].joint_cmd.push_back(VectorXf::Zero(gripper_joint_size[gri_i]));
    }
}

void RecordAndReplayBase::createFolderIfNotExist(std::string directory){
    boost::filesystem::path dir(directory);
    
    // Check if the exact target directory already exists
    if (boost::filesystem::is_directory(dir)) {
        return;
    }

    // Automatically checks and creates ALL missing upper/parent directories
    if (boost::filesystem::create_directories(dir)) {
        std::cerr << "Directory chain created successfully: " << directory << std::endl;
    } else {
        std::cerr << "Failed to create directory chain: " << directory << std::endl;
    }
}

void RecordAndReplayBase::writeToText(const std::vector<std::map<ComponentType, std::vector<VectorXf>>>& data, const std::string& file_path){
    std::ofstream out(file_path);
    Eigen::IOFormat CleanFmt(4, Eigen::DontAlignCols, ", ", "\n");
    std::stringstream ss;

    //Add Header(first line)
    std::string header_str = "#";
    for (const ComponentType& comp_type: component_order_){
        if (!component_map_.count(comp_type) || !component_map_[comp_type].component_size) continue;
        for (int comp_i=0; comp_i<component_map_[comp_type].component_size; comp_i++){
            header_str += enumToString<ComponentType>(comp_type) + "_" + std::to_string(comp_i+1) + ":" + std::to_string(component_map_[comp_type].joint_size[comp_i]) + ", ";
        }
    }
    header_str.erase(header_str.end()-2, header_str.end());
    ss << header_str << "\n";

    //Add Data
    for (std::map<ComponentType, std::vector<VectorXf>> row: data){
        std::stringstream row_ss;
        for (const ComponentType& comp_type: component_order_){
            if (!component_map_.count(comp_type) || !component_map_[comp_type].component_size) continue;
            for (int comp_i=0; comp_i<component_map_[comp_type].component_size; comp_i++){
                row_ss << '[' << row[comp_type][comp_i].transpose().format(CleanFmt) << "], ";
            }
        }
        std::string row_str = row_ss.str();
        if (row_str.length() > 2) row_str.erase(row_str.end()-2, row_str.end()); //Remove the last two characters
        ss << row_str << "\n";
    }

    if (out.good()){
        out << ss.str();
    }
    out.close();
}

void RecordAndReplayBase::writeToText(const std::vector<std::vector<Vector7f>>& data, const std::string& file_path, const std::string& header){
    std::ofstream out(file_path);
    Eigen::IOFormat CleanFmt(4, Eigen::DontAlignCols, ", ", "\n");
    std::stringstream ss;

    if (!header.empty()) ss << "#" << header << "\n";
    for (std::vector<Vector7f> row: data){
        for (int i=0; i<row.size(); i++){
            ss << '[' << row[i].transpose().format(CleanFmt) << ']';
            if (i < row.size() - 1) ss << ", ";
        }
        ss << "\n";
    }

    if (out.good()){
        out << ss.str();
    }
    out.close();
}

std::vector<ComponentType> RecordAndReplayBase::extractComponentOrder(const std::string& header){
    std::vector<ComponentType> res;
    std::string input = header;
    input.erase(0, 1); //Remove '#'
    std::istringstream iss(input);
    std::string token;
    std::string prev_component_str = "";
    while (std::getline(iss, token, ',')){
        // Use find_if to find the first non-space character
        auto start = std::find_if(token.begin(), token.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        });
        // Erase leading spaces
        token.erase(token.begin(), start);
    
        std::size_t underscore_pos = token.find('_');
        std::size_t colon_pos = token.find(':');
        std::string component_str = token.substr(0, underscore_pos);
        int component_id = std::stoi(token.substr(underscore_pos+1, colon_pos - underscore_pos - 1)) - 1;
        int joint_size = std::stoi(token.substr(colon_pos+1));
        for (std::pair<const ComponentType, ComponentData>& pair: component_map_){
            if (component_str == enumToString<ComponentType>(pair.first)){
                if (component_id > pair.second.component_size - 1 || joint_size != pair.second.joint_size[component_id]){
                    throw std::invalid_argument(token + " does not match with the configuration file!");
                }else if (component_id < pair.second.component_size - 1 && (!prev_component_str.empty() && component_str != prev_component_str)){
                    throw std::invalid_argument("Not enough component size for " + prev_component_str + "!");
                }else if (component_id == pair.second.component_size - 1 && (prev_component_str.empty() || component_str == prev_component_str)){
                    res.push_back(pair.first);
                    prev_component_str = "";
                }else{
                    prev_component_str = component_str;
                }
            }
        }
    }
    return res;
}

std::vector<std::map<ComponentType, std::vector<VectorXf>>> RecordAndReplayBase::extractFromText(const std::string& file_path){
    std::vector<std::map<ComponentType, std::vector<VectorXf>>> res;
    std::ifstream in(file_path);
    std::string line;
    char ch;
    std::vector<ComponentType> order={ComponentType::kArm};
    try{
        if (in.good()){
            while (std::getline(in, line)){   
                if (line[0] == '#'){
                    order = extractComponentOrder(line);
                    continue;
                };
                
                bool hasBracket = line[0] == '[';
                if (!hasBracket && component_map_[ComponentType::kArm].component_size > 1){
                    throw std::runtime_error("Multiple arms should be enclosed in brackets!\n");
                }
                
                std::istringstream iss(line);
                std::map<ComponentType, std::vector<VectorXf>> row;
                for (const ComponentType& comp_type: order){
                    std::vector<VectorXf> comp_row;
                    for (int comp_i=0; comp_i<component_map_[comp_type].component_size; comp_i++){
                        VectorXf joint = VectorXf::Zero(component_map_[comp_type].joint_size[comp_i]);
                        if (hasBracket) iss >> ch;            //Remove '['
                        for (int i=0; i<component_map_[comp_type].joint_size[comp_i]; i++){
                            iss>>joint[i];
                            if (iss.peek() == ',') {
                                iss.ignore();
                            }
                        }
                        comp_row.push_back(joint);
                        if (hasBracket) iss >> ch;            //Remove ']'
                        iss >> ch;  //Remote ','
                    }
                    row[comp_type] = comp_row;
                }
                res.push_back(row);
            }
        }
    }catch (std::exception& e){
        std::cerr << "Error: " << e.what() << std::endl;
        in.close();
        return std::vector<std::map<ComponentType, std::vector<VectorXf>>>();
    }
    in.close();
    return res;
}

void RecordAndReplayBase::slotControlTypeChanged(ControlType top, ControlType base){
    bool isPos = top == ControlType::kPosition;
    int arm_size = component_map_[ComponentType::kArm].component_size;
    const std::vector<int>& joint_size = component_map_[ComponentType::kArm].joint_size;
    safe_panel_command_.updateCommand([&](PanelCommand& command){
        for (int arm_i=0; arm_i<arm_size; arm_i++){
            for (int i=0; i<joint_size[arm_i]; i++){
                (command.*component_map_[ComponentType::kArm].cmd_ref)[arm_i][i] = (isPos)? (planner_state_shared_ptr_.get()->*component_map_[ComponentType::kArm].state_ref)[arm_i][i] : 0;    
            }
        }
    });
}