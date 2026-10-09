#include "cuarm_message_handler.h"

#include "time_utils.h"

#include <charconv>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <vector>

// #include <typeinfo>
// #include <stdio.h>
// #include <inttypes.h>
// #include <unistd.h>		   // Needed for sysconf(int name);

/* gettimeofday() does not appear on linux without this. */
// #define _BSD_SOURCE
namespace {
bool parseIntegerToken(std::string_view token, int& value) {
    if (token.empty()) return false;
    const char* begin = token.data();
    const char* end = begin + token.size();
    const auto result = std::from_chars(begin, end, value);
    return result.ec == std::errc() && result.ptr == end;
}

bool splitTokens(const char* buffer, std::vector<std::string_view>& tokens) {
    if (!buffer) return false;
    const char* token_begin = buffer;
    for (const char* p = buffer; *p != '\0'; ++p) {
        const unsigned char ch = static_cast<unsigned char>(*p);
        if (ch == '#') {
            tokens.emplace_back(token_begin, static_cast<std::size_t>(p - token_begin));
            token_begin = p + 1;
        } else if (ch < 0x20 || ch > 0x7e) {
            return false;
        }
    }
    return true;
}

bool validatePanelCommandLayout(const char* buffer) {
    std::vector<std::string_view> tokens;
    if (!splitTokens(buffer, tokens)) return false;

    // Fixed fields through ArmSize.
    constexpr std::size_t kArmSizeToken = 20;
    if (tokens.size() <= kArmSizeToken) return false;

    std::size_t index = kArmSizeToken;
    int arm_size = 0;
    if (!parseIntegerToken(tokens[index++], arm_size) ||
        arm_size < 0 || arm_size > MAX_ARM_SIZE) {
        return false;
    }

    std::vector<int> joint_sizes(static_cast<std::size_t>(arm_size), 0);
    for (int arm_i = 0; arm_i < arm_size; ++arm_i) {
        if (index >= tokens.size() ||
            !parseIntegerToken(tokens[index++], joint_sizes[arm_i]) ||
            joint_sizes[arm_i] < 0 || joint_sizes[arm_i] > MAX_JOINT_SIZE) {
            return false;
        }

        const std::size_t arm_fields =
            1u + static_cast<std::size_t>(joint_sizes[arm_i]) * 3u + 6u;
        if (index + arm_fields > tokens.size()) return false;
        if (tokens[index].size() >= MAX_ROBOTNAME_SIZE) return false;
        index += arm_fields;
    }

    int waypoint_size = 0;
    if (index + 2 > tokens.size() ||
        !parseIntegerToken(tokens[index++], waypoint_size) ||
        waypoint_size < 0 || waypoint_size > MAX_WAYPOINTS) {
        return false;
    }
    ++index;  // ArmWaypointDt

    std::size_t joints_per_waypoint = 0;
    for (int size : joint_sizes) joints_per_waypoint += static_cast<std::size_t>(size);
    const std::size_t waypoint_fields =
        static_cast<std::size_t>(waypoint_size) * (joints_per_waypoint + 2u);
    if (index + waypoint_fields > tokens.size()) return false;
    index += waypoint_fields;

    int gripper_size = 0;
    if (index >= tokens.size() ||
        !parseIntegerToken(tokens[index++], gripper_size) ||
        gripper_size < 0 || gripper_size > MAX_ARM_SIZE) {
        return false;
    }

    std::vector<int> gripper_joint_sizes(static_cast<std::size_t>(gripper_size), 0);
    for (int gripper_i = 0; gripper_i < gripper_size; ++gripper_i) {
        if (index >= tokens.size() ||
            !parseIntegerToken(tokens[index++], gripper_joint_sizes[gripper_i]) ||
            gripper_joint_sizes[gripper_i] < 0 ||
            gripper_joint_sizes[gripper_i] > MAX_JOINT_SIZE) {
            return false;
        }
        const std::size_t count = static_cast<std::size_t>(gripper_joint_sizes[gripper_i]);
        if (index + count > tokens.size()) return false;
        index += count;
    }

    std::size_t trailing_fields = static_cast<std::size_t>(arm_size) * 3u;
    for (int size : joint_sizes) trailing_fields += static_cast<std::size_t>(size) * 4u;
    for (int size : gripper_joint_sizes) trailing_fields += static_cast<std::size_t>(size) * 2u;
    return index + trailing_fields <= tokens.size();
}

bool validatePlannerStateLayout(const char* buffer) {
    std::vector<std::string_view> tokens;
    if (!splitTokens(buffer, tokens) || tokens.size() < 5) return false;

    std::size_t index = 4;
    int arm_size = 0;
    if (!parseIntegerToken(tokens[index++], arm_size) ||
        arm_size <= 0 || arm_size > MAX_ARM_SIZE) {
        return false;
    }

    std::vector<int> joint_sizes(static_cast<std::size_t>(arm_size), 0);
    for (int arm_i = 0; arm_i < arm_size; ++arm_i) {
        if (index >= tokens.size() ||
            !parseIntegerToken(tokens[index++], joint_sizes[arm_i]) ||
            joint_sizes[arm_i] <= 0 || joint_sizes[arm_i] > MAX_JOINT_SIZE) {
            return false;
        }
        const std::size_t arm_state_fields =
            static_cast<std::size_t>(joint_sizes[arm_i]) * 16u + 14u;
        if (index + arm_state_fields > tokens.size()) return false;
        index += arm_state_fields;
    }

    int gripper_size = 0;
    if (index >= tokens.size() ||
        !parseIntegerToken(tokens[index++], gripper_size) ||
        gripper_size < 0 || gripper_size > MAX_ARM_SIZE) {
        return false;
    }

    std::vector<int> gripper_joint_sizes(static_cast<std::size_t>(gripper_size), 0);
    for (int gripper_i = 0; gripper_i < gripper_size; ++gripper_i) {
        if (index >= tokens.size() ||
            !parseIntegerToken(tokens[index++], gripper_joint_sizes[gripper_i]) ||
            gripper_joint_sizes[gripper_i] < 0 ||
            gripper_joint_sizes[gripper_i] > MAX_JOINT_SIZE) {
            return false;
        }
        const std::size_t count = static_cast<std::size_t>(gripper_joint_sizes[gripper_i]);
        if (index + count > tokens.size()) return false;
        index += count;
    }

    std::size_t trailing_fields = static_cast<std::size_t>(arm_size) * 3u + 3u;
    for (int size : joint_sizes) trailing_fields += static_cast<std::size_t>(size) * 19u;
    for (int size : gripper_joint_sizes) trailing_fields += static_cast<std::size_t>(size) * 4u;
    return index + trailing_fields == tokens.size();
}

template <typename Enum>
bool enumInRange(Enum value, int minimum, int maximum) {
    const int raw = static_cast<int>(value);
    return raw >= minimum && raw <= maximum;
}
}

int CuarmMessageHandler::get_next_data_index(char* receive_buffer, int index){
    while (receive_buffer[index] && receive_buffer[index] != '#') 
        index ++; 
    return receive_buffer[index] == '#' ? index + 1 : index;
}

void CuarmMessageHandler::pack_panel_command(PanelCommand* panel, char* send_buffer, int buffer_size){
    if (!panel || !send_buffer || buffer_size < 60000 ||
        panel->ArmSize > MAX_ARM_SIZE || panel->GripperSize > MAX_ARM_SIZE ||
        panel->ArmWaypointSize > MAX_WAYPOINTS) {
        throw std::runtime_error("PanelCommand cannot be packed safely.");
    }
    for (int arm_i = 0; arm_i < panel->ArmSize; ++arm_i) {
        if (panel->JointSize[arm_i] > MAX_JOINT_SIZE) {
            throw std::runtime_error("PanelCommand joint count exceeds protocol capacity.");
        }
    }
    for (int gripper_i = 0; gripper_i < panel->GripperSize; ++gripper_i) {
        if (panel->GripperJointSize[gripper_i] > MAX_JOINT_SIZE) {
            throw std::runtime_error("PanelCommand gripper count exceeds protocol capacity.");
        }
    }
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%ld#", panel->send_timestamp);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", panel->sequence_id);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->connection_state);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->need_setting_update);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->simulation);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->motion_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->task_orien_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->reset_control_mem);
    
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->target_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->actuator_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->interpolation_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->interpolation_speed_ratio);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->InterpolationAccTime);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->InterpolationConstVelTime);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->NoneInterpolationSaturationRatio);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->reset_interpolation);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->control_algorithm);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->enable_recording);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->playback_cmd);

    //Arm related
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->arm_target_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->ArmSize);
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->JointSize[arm_i]);
        snprintf(&(send_buffer[strlen(send_buffer)]), MAX_ROBOTNAME_SIZE+1, "%s#", panel->RobotName[arm_i]);
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)panel->enable_joint[arm_i][i]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->MotorCmd[arm_i][i]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->JointCmd[arm_i][i]);
        }
        for (int i = 0; i <6; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->TaskCmd[arm_i][i]);
        }
    }

    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->ArmWaypointSize);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->ArmWaypointDt);
    for (int point_i =0; point_i < panel->ArmWaypointSize; point_i++){
        for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
            for (int i =0; i < panel->JointSize[arm_i]; i++){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", panel->ArmWaypointCmd[point_i][arm_i][i]);
            }
        }
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->ArmWaypointInterpolationTime[point_i]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->ArmWaypointInterpolationSpeedRatio[point_i]);
    }

    //Gripper related
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->GripperSize);
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", panel->GripperJointSize[gri_i]);
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->GripperJointCmd[gri_i][i]);
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->tool_offset[arm_i][0]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->tool_offset[arm_i][1]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->tool_offset[arm_i][2]);
    }
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_position[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_position[arm_i][i][1]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_velocity[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->arm_soft_limit_velocity[arm_i][i][1]);
        }
    }
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->gripper_soft_limit_position[gri_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%e#", panel->gripper_soft_limit_position[gri_i][i][1]);
        }
    }
}

void CuarmMessageHandler::unpack_panel_command(PanelCommand* panel, char* receive_buffer){
    if (!panel || !validatePanelCommandLayout(receive_buffer)) {
        if (panel) *panel = PanelCommand{};
        std::cerr << "[UDP] Rejected malformed PanelCommand payload." << std::endl;
        return;
    }

    int index = 0;
    char robot_name_format[10];
    snprintf(robot_name_format, sizeof(robot_name_format), "%%%d[^#]", MAX_ROBOTNAME_SIZE-1);
    sscanf(receive_buffer, "%ld#", &(panel->send_timestamp));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(panel->sequence_id));
    scan_type<ConnectionState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->connection_state);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->need_setting_update);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->simulation);
    scan_type<MotionControl, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->motion_type);
    scan_type<OrientControl, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->task_orien_type);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->reset_control_mem);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->target_type);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->actuator_mode);
    scan_type<InterpolationMethod, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->interpolation_type);

    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->interpolation_speed_ratio));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->InterpolationAccTime));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->InterpolationConstVelTime));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->NoneInterpolationSaturationRatio));
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->reset_interpolation);
    scan_type<ControlAlgorithm, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->control_algorithm);
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->enable_recording);
    scan_type<PlaybackState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->playback_cmd);
    
    //Arm related
    scan_type<PanelTargetMode, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->arm_target_mode);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->ArmSize));
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->JointSize[arm_i]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), robot_name_format, &(panel->RobotName[arm_i]));
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
            scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), panel->enable_joint[arm_i][i]);
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->MotorCmd[arm_i][i]));
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->JointCmd[arm_i][i]));
        }
        for (int i = 0; i < 6; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->TaskCmd[arm_i][i]));
        }
    }

    //Path related
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->ArmWaypointSize));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointDt));
    for (int point_i = 0; point_i < panel->ArmWaypointSize; point_i++){
        for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
            for (int i = 0; i < panel->JointSize[arm_i]; ++ i) {
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointCmd[point_i][arm_i][i]));
            }
        }
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointInterpolationTime[point_i]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->ArmWaypointInterpolationSpeedRatio[point_i]));
    }

    //Gripper related
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->GripperSize));
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(panel->GripperJointSize[gri_i]));
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->GripperJointCmd[gri_i][i]));
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->tool_offset[arm_i][0]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->tool_offset[arm_i][1]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->tool_offset[arm_i][2]));
    }
    for (int arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_position[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_position[arm_i][i][1]));
        }
        for (int i = 0; i < panel->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_velocity[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->arm_soft_limit_velocity[arm_i][i][1]));
        }
    }
    for (int gri_i = 0; gri_i < panel->GripperSize; gri_i++){
        for (int i = 0; i < panel->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->gripper_soft_limit_position[gri_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(panel->gripper_soft_limit_position[gri_i][i][1]));
        }
    }

    bool valid = enumInRange(panel->connection_state, static_cast<int>(ConnectionState::kWaiting),
                             static_cast<int>(ConnectionState::kShutDown));
    valid &= enumInRange(panel->target_type, static_cast<int>(ControlType::kPosition),
                         static_cast<int>(ControlType::kTorque));
    valid &= enumInRange(panel->actuator_mode, static_cast<int>(ControlType::kPosition),
                         static_cast<int>(ControlType::kTorque));
    valid &= enumInRange(panel->motion_type, static_cast<int>(MotionControl::kJoint),
                         static_cast<int>(MotionControl::kPlayback));
    valid &= enumInRange(panel->arm_target_mode, static_cast<int>(PanelTargetMode::kSinglePoint),
                         static_cast<int>(PanelTargetMode::kRawSinglePoint));
    valid &= std::isfinite(panel->interpolation_speed_ratio) &&
             std::isfinite(panel->InterpolationAccTime) &&
             std::isfinite(panel->InterpolationConstVelTime) &&
             std::isfinite(panel->NoneInterpolationSaturationRatio);

    for (int arm_i = 0; valid && arm_i < panel->ArmSize; ++arm_i) {
        for (int joint_i = 0; valid && joint_i < panel->JointSize[arm_i]; ++joint_i) {
            valid &= std::isfinite(panel->MotorCmd[arm_i][joint_i]) &&
                     std::isfinite(panel->JointCmd[arm_i][joint_i]) &&
                     std::isfinite(panel->arm_soft_limit_position[arm_i][joint_i][0]) &&
                     std::isfinite(panel->arm_soft_limit_position[arm_i][joint_i][1]) &&
                     std::isfinite(panel->arm_soft_limit_velocity[arm_i][joint_i][0]) &&
                     std::isfinite(panel->arm_soft_limit_velocity[arm_i][joint_i][1]);
        }
        for (int task_i = 0; valid && task_i < 6; ++task_i) {
            valid &= std::isfinite(panel->TaskCmd[arm_i][task_i]);
        }
    }
    if (!valid) {
        *panel = PanelCommand{};
        std::cerr << "[UDP] Rejected PanelCommand with invalid enum or non-finite value." << std::endl;
    }
}

void CuarmMessageHandler::pack_panel_state(PanelState* state, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%d#", (int)state->state);
    
    //Arm
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->ArmSize);
    for (int arm_i = 0; arm_i < (int)state->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->JointSize[arm_i]);
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointPos[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointVel[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointTor[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->EEPose[arm_i][i]);
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->ToolPose[arm_i][i]);
        }
    }

    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)state->JointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->interpolated_target[arm_i][joint_i]);
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperSize);
    if (state->GripperSize > 0){
        for (int gripper_i = 0; gripper_i < (int)state->GripperSize; gripper_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperJointSize[gripper_i]);
        }
        for (int gripper_i = 0; gripper_i < state->GripperSize; gripper_i++){
            for (int joint_i = 0; joint_i < (int)state->GripperJointSize[gripper_i]; joint_i++){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->GripperJointPos[gripper_i][joint_i]);
            }
        }
    }else{
        sprintf(&(send_buffer[strlen(send_buffer)]), "#");
        sprintf(&(send_buffer[strlen(send_buffer)]), "#");
    }
}

void CuarmMessageHandler::unpack_panel_state(PanelState* state, char* receive_buffer){
    int index = 0;
    scan_type<PanelRemoteState, int>(receive_buffer, index, state->state);

    //Arm 
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->ArmSize));
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->JointSize[arm_i]));
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->JointPos[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->JointVel[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->JointTor[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->EEPose[arm_i][i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < 7; i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->ToolPose[arm_i][i]));
        }
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < state->JointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->interpolated_target[arm_i][joint_i]));
        }
    }

    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperSize));
    if (state->GripperSize > 0){
        for (int gripper_i = 0; gripper_i < state->GripperSize; gripper_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperJointSize[gripper_i]));
        }
        for (int gripper_i = 0; gripper_i < state->GripperSize; gripper_i++){
            for (int joint_i = 0; joint_i < state->GripperJointSize[gripper_i]; joint_i++){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->GripperJointPos[gripper_i][joint_i]));
            }
        }
    }else{
        //If no gripper, skip the two fields for gripper joint size and position
        index = get_next_data_index(receive_buffer, index);
        index = get_next_data_index(receive_buffer, index);
    }
}

void CuarmMessageHandler::pack_planner_command(PlannerCommand* command, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%hhd#", (uint8_t)command->connection_state);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->command_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmSize);
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->JointSize[arm_i]);
        for (int i = 0; i < command->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->Target[arm_i][i]);
        }
    }
}

void CuarmMessageHandler::unpack_planner_command(PlannerCommand* command, char* receive_buffer){
    int index = 0;
    scan_type<ConnectionState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->connection_state);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->command_mode);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmSize));
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->JointSize[arm_i]));
        for (int i = 0; i < command->JointSize[arm_i]; ++ i) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(command->Target[arm_i][i]));
        }
    }
}

void CuarmMessageHandler::unpack_planner_state(PlannerState* state, char* receive_buffer){
    if (!state || !validatePlannerStateLayout(receive_buffer)) {
        if (state) *state = PlannerState{};
        std::cerr << "[UDP] Rejected malformed PlannerState payload." << std::endl;
        return;
    }
    int index = 0;
    sscanf(receive_buffer, "%ld#", &(state->received_panel_command_timestamp));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%ld#", &(state->send_timestamp));
    scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->setting_update_finished);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(state->received_sequence_id));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->ArmSize));
    if (state->ArmSize == 0 || state->ArmSize > MAX_ARM_SIZE) {
        *state = PlannerState{};
        std::cerr << "[UDP] Rejected PlannerState with invalid arm count." << std::endl;
        return;
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->JointSize[arm_i]));
        if (state->JointSize[arm_i] == 0 || state->JointSize[arm_i] > MAX_JOINT_SIZE) {
            *state = PlannerState{};
            std::cerr << "[UDP] Rejected PlannerState with invalid joint count." << std::endl;
            return;
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            scan_type<bool, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->enabled_joint[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointPos[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointVel[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointTor[arm_i][i]));
        }

        
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorPos[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorVel[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorCur[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->MotorTor[arm_i][i]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            for (int j = 0; j < 7; j++) {
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->JointPose[arm_i][i][j]));
            }
        }
        for (int j = 0; j < 7; j++) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->EEPose[arm_i][j]));
        }
        for (int j = 0; j < 7; j++) {
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->ToolPose[arm_i][j]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->interpolated_target[arm_i][i]));
        }
    }

    //Gripper related
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperSize));
    if (state->GripperSize > MAX_ARM_SIZE) {
        *state = PlannerState{};
        std::cerr << "[UDP] Rejected PlannerState with invalid gripper count." << std::endl;
        return;
    }
    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->GripperJointSize[gri_i]));
        if (state->GripperJointSize[gri_i] > MAX_JOINT_SIZE) {
            *state = PlannerState{};
            std::cerr << "[UDP] Rejected PlannerState with invalid gripper joint count." << std::endl;
            return;
        }
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->GripperJointPos[gri_i][i]));
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->tool_offset[arm_i][0]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->tool_offset[arm_i][1]));
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->tool_offset[arm_i][2]));
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_position[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_position[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_velocity[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_velocity[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_torque[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_soft_limit_torque[arm_i][i][1]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_position[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_position[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_velocity[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_velocity[arm_i][i][1]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_torque[arm_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_hard_limit_torque[arm_i][i][1]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_follow_limit_position[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_follow_limit_velocity[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_follow_limit_torque[arm_i][i]));
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_jump_limit_position[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_jump_limit_velocity[arm_i][i]));
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->arm_jump_limit_torque[arm_i][i]));
        }
    }

    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_soft_limit_position[gri_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_soft_limit_position[gri_i][i][1]));
        }
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_hard_limit_position[gri_i][i][0]));
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%e#", &(state->gripper_hard_limit_position[gri_i][i][1]));
        }
    }

    scan_type<SystemState, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->system_state);
    scan_type<PlanResult, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), state->plan_result);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(state->system_diagnostic_flags));
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%u#", &(state->arm_joint_diagnostic_flags[arm_i][i]));
        }
    }
}

void CuarmMessageHandler::pack_planner_state(PlannerState* state, char* send_buffer, int buffer_size){
    if (!state || !send_buffer || buffer_size < 60000 ||
        state->ArmSize > MAX_ARM_SIZE || state->GripperSize > MAX_ARM_SIZE) {
        throw std::runtime_error("PlannerState cannot be packed safely.");
    }
    for (int arm_i = 0; arm_i < state->ArmSize; ++arm_i) {
        if (state->JointSize[arm_i] > MAX_JOINT_SIZE) {
            throw std::runtime_error("PlannerState joint count exceeds protocol capacity.");
        }
    }
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%ld#", state->received_panel_command_timestamp);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%ld#", state->send_timestamp);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->setting_update_finished);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", state->received_sequence_id);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->ArmSize);
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->JointSize[arm_i]);
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->enabled_joint[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointPos[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointVel[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointTor[arm_i][i]);
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorPos[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorVel[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorCur[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->MotorTor[arm_i][i]);
        }

        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            for (int j = 0; j < 7; ++ j){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->JointPose[arm_i][i][j]);
            }
        }
        for (int j = 0; j < 7; ++ j){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->EEPose[arm_i][j]);
        }
        for (int j = 0; j < 7; ++ j){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->ToolPose[arm_i][j]);
        }
        
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->interpolated_target[arm_i][i]);
        }
    }

    //Gripper related
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperSize);
    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->GripperJointSize[gri_i]);
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->GripperJointPos[gri_i][i]);
        }
    }

    //Config setting
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[arm_i][0]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[arm_i][1]);
        sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[arm_i][2]);
    }
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_position[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_position[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_velocity[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_velocity[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_torque[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_soft_limit_torque[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_position[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_position[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_velocity[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_velocity[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_torque[arm_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_hard_limit_torque[arm_i][i][1]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_follow_limit_position[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_follow_limit_velocity[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_follow_limit_torque[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_jump_limit_position[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_jump_limit_velocity[arm_i][i]);
        }
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->arm_jump_limit_torque[arm_i][i]);
        }
    }
    for (int gri_i = 0; gri_i < state->GripperSize; gri_i++){
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_soft_limit_position[gri_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_soft_limit_position[gri_i][i][1]);
        }
        for (int i = 0; i < state->GripperJointSize[gri_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_hard_limit_position[gri_i][i][0]);
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_hard_limit_position[gri_i][i][1]);
        }
    }

    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->system_state);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)state->plan_result);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", state->system_diagnostic_flags);
    for (int arm_i = 0; arm_i < state->ArmSize; arm_i++){
        for (int i = 0; i < state->JointSize[arm_i]; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%u#", state->arm_joint_diagnostic_flags[arm_i][i]);
        }
    }
}

void CuarmMessageHandler::pack_remote_panel_command(RemotePanelCommand* command, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);
    sprintf(send_buffer, "%f#", command->interpolation_acceleration_time);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->no_interpolation_max_velocity_ratio);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->target_type);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->actuator_mode);
    
    //Arm
    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->arm_target_mode);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmSize);
    for (int arm_i = 0; arm_i < (int)command->ArmSize; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmJointSize[arm_i]);
    }
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < (int)command->ArmJointSize[arm_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->ArmJointCmd[arm_i][joint_i]);
        }
    }

    //Path
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->ArmWaypointSize);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->ArmWaypointDt);
    for (int point_i =0; point_i < command->ArmWaypointSize; point_i++){
        for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
            for (int joint_i =0; joint_i < (int)command->ArmJointSize[arm_i]; joint_i++){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->ArmWaypointCmd[point_i][arm_i][joint_i]);
            }
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->GripperSize);
    for (int gripper_i = 0; gripper_i < (int)command->GripperSize; gripper_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->GripperJointSize[gripper_i]);
    }
    for (int gripper_i = 0; gripper_i < command->GripperSize; gripper_i++){
        for (int joint_i = 0; joint_i < (int)command->GripperJointSize[gripper_i]; joint_i++){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->GripperJointCmd[gripper_i][joint_i]);
        }
    }
}

void CuarmMessageHandler::unpack_remote_panel_command(RemotePanelCommand* command, char* receive_buffer){
    int index = 0;
    sscanf(receive_buffer, "%f#", &(command->interpolation_acceleration_time));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->no_interpolation_max_velocity_ratio));
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->target_type);
    scan_type<ControlType, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->actuator_mode);

    //Arm 
    scan_type<PanelTargetMode, int>(receive_buffer, index=get_next_data_index(receive_buffer, index), command->arm_target_mode);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmSize));
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmJointSize[arm_i]));
    }
    for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
        for (int joint_i = 0; joint_i < command->ArmJointSize[arm_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->ArmJointCmd[arm_i][joint_i]));
        }
    }

    //Path
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->ArmWaypointSize));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->ArmWaypointDt));
    if (command->ArmWaypointSize > 0){
        for (int point_i =0; point_i < command->ArmWaypointSize; point_i++){
            for (int arm_i = 0; arm_i < command->ArmSize; arm_i++){
                for (int joint_i =0; joint_i < command->ArmJointSize[arm_i]; joint_i++){
                    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->ArmWaypointCmd[point_i][arm_i][joint_i]));
                }
            }
        }
    }else{
        index = get_next_data_index(receive_buffer, index); //Remove a redundant #
    }
    
    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->GripperSize));
    for (int gripper_i = 0; gripper_i < command->GripperSize; gripper_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->GripperJointSize[gripper_i]));
    }
    for (int gripper_i = 0; gripper_i < command->GripperSize; gripper_i++){
        for (int joint_i = 0; joint_i < command->GripperJointSize[gripper_i]; joint_i++){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->GripperJointCmd[gripper_i][joint_i]));
        }
    }
}

void CuarmMessageHandler::pack_panel_config_command(PanelConfigCommand* command, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);

    sprintf(&(send_buffer[strlen(send_buffer)]), "%d#", (int)command->read_only);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hu#", command->remote_udp_port);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->arm_size);
    for (int arm_i = 0; arm_i < command->arm_size; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->arm_joint_size[arm_i]);
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->joint_soft_limit_position[limit_i][arm_i][joint_i]);
            }
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->joint_soft_limit_velocity[arm_i][joint_i]);
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->joint_soft_limit_torque[arm_i][joint_i]);
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->gripper_size);
    for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", command->gripper_joint_size[gripper_i]);
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < command->gripper_joint_size[gripper_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->gripper_soft_limit_position[limit_i][gripper_i][joint_i]);
            }
        }
    }
    
    for (int gripper_i = 0; gripper_i < command->gripper_size; gripper_i++){
        for (int i = 0; i < 3; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", command->tool_offset[gripper_i][i]);
        }
    }

}

void CuarmMessageHandler::unpack_panel_config_command(PanelConfigCommand* command, char* receive_buffer){
    int index = 0;

    scan_type<bool, int>(receive_buffer, index, command->read_only);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hu#", &(command->remote_udp_port));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->arm_size));
    for (int arm_i = 0; arm_i < command->arm_size; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->arm_joint_size[arm_i]));
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->joint_soft_limit_position[limit_i][arm_i][joint_i]));
            }
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->joint_soft_limit_velocity[arm_i][joint_i]));
        }
    }

    for (int arm_i = 0; arm_i < command->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < command->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->joint_soft_limit_torque[arm_i][joint_i]));
        }
    }

    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->gripper_size));
    for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(command->gripper_joint_size[gripper_i]));
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < command->gripper_joint_size[gripper_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->gripper_soft_limit_position[limit_i][gripper_i][joint_i]));
            }
        }
    }
    
    for (int gripper_i = 0; gripper_i < command->gripper_size; ++ gripper_i){
        for (int i = 0; i < 3; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(command->tool_offset[gripper_i][i]));
        }
    }
}

void CuarmMessageHandler::pack_rt_config_state(RtConfigState* state, char* send_buffer, int buffer_size){
    memset(send_buffer, 0, buffer_size);

    sprintf(&(send_buffer[strlen(send_buffer)]), "%s#", state->server_udp_ip);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%s#", state->client_udp_ip);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hu#", state->server_udp_port);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hu#", state->client_udp_port);
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->arm_size);
    for (int arm_i = 0; arm_i < state->arm_size; arm_i++){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->arm_joint_size[arm_i]);
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_soft_limit_position[limit_i][arm_i][joint_i]);
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_soft_limit_velocity[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_soft_limit_torque[arm_i][joint_i]);
        }
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_hard_limit_position[limit_i][arm_i][joint_i]);
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_hard_limit_velocity[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_hard_limit_torque[arm_i][joint_i]);
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_follow_limit_position[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_follow_limit_velocity[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_follow_limit_torque[arm_i][joint_i]);
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_jump_limit_position[arm_i][joint_i]);
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->joint_jump_limit_torque[arm_i][joint_i]);
        }
    }

    //Gripper
    sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->gripper_size);
    for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
        sprintf(&(send_buffer[strlen(send_buffer)]), "%hhd#", state->gripper_joint_size[gripper_i]);
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_joint_soft_limit_position[limit_i][gripper_i][joint_i]);
            }
        }
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->gripper_joint_hard_limit_position[limit_i][gripper_i][joint_i]);
            }
        }
    }

    for (int gripper_i = 0; gripper_i < state->gripper_size; gripper_i++){
        for (int i = 0; i < 3; ++ i){
            sprintf(&(send_buffer[strlen(send_buffer)]), "%f#", state->tool_offset[gripper_i][i]);
        }
    }
}

void CuarmMessageHandler::unpack_rt_config_state(RtConfigState* state, char* receive_buffer){
    int index = 0;
    char ip_format[10];
    snprintf(ip_format, sizeof(ip_format), "%%%d[^#]", 14);

    sscanf(receive_buffer, ip_format, state->server_udp_ip);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), ip_format, state->client_udp_ip);
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hu#", &(state->server_udp_port));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hu#", &(state->client_udp_port));
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->arm_size));
    for (int arm_i = 0; arm_i < state->arm_size; arm_i++){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->arm_joint_size[arm_i]));
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_soft_limit_position[limit_i][arm_i][joint_i]));
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_soft_limit_velocity[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_soft_limit_torque[arm_i][joint_i]));
        }
    }

    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
            for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_hard_limit_position[limit_i][arm_i][joint_i]));
            }
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_hard_limit_velocity[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_hard_limit_torque[arm_i][joint_i]));
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_follow_limit_position[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_follow_limit_velocity[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_follow_limit_torque[arm_i][joint_i]));
        }
    }

    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_jump_limit_position[arm_i][joint_i]));
        }
    }
    for (int arm_i = 0; arm_i < state->arm_size; ++ arm_i){
        for (int joint_i = 0; joint_i < state->arm_joint_size[arm_i]; ++ joint_i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->joint_jump_limit_torque[arm_i][joint_i]));
        }
    }

    //Gripper
    sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->gripper_size));
    for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
        sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%hhd#", &(state->gripper_joint_size[gripper_i]));
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->gripper_joint_soft_limit_position[limit_i][gripper_i][joint_i]));
            }
        }
    }
    for (int limit_i = 0; limit_i < 2; ++ limit_i){
        for (int gripper_i = 0; gripper_i < state->gripper_size; ++ gripper_i){
            for (int joint_i = 0; joint_i < state->gripper_joint_size[gripper_i]; ++ joint_i){
                sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->gripper_joint_hard_limit_position[limit_i][gripper_i][joint_i]));
            }
        }
    }
    
    for (int gripper_i = 0; gripper_i < state->gripper_size; gripper_i++){
        for (int i = 0; i < 3; ++ i){
            sscanf(&(receive_buffer[index = get_next_data_index(receive_buffer, index)]), "%f#", &(state->tool_offset[gripper_i][i]));
        }
    }
}

