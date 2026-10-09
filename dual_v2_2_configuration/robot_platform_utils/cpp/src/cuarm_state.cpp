#include "cuarm_state.h"

void print_panel_command(PanelCommand* panel){
	int i, arm_i;
	printf("\nRobot: (send_timestamp: %ld, ArmSize: %d)\n", panel->send_timestamp, panel->ArmSize);
	printf("simulation: %d, motion_type: %d, task_orien_type: %d, reset_control_mem: %d \n", panel->simulation, (int)panel->motion_type, (int)panel->task_orien_type, panel->reset_control_mem);
	printf("target_type: %d, actuator_mode: %d, interpolation_type: %d\n", (int)panel->target_type, (int)panel->actuator_mode, (int)panel->interpolation_type);
    printf("InterpolationAccTime: %f, InterpolationConstVelTime: %f, reset_interpolation: %d \n", panel->InterpolationAccTime, panel->InterpolationConstVelTime, panel->reset_interpolation);
	printf("control_algorithm: %d \n", (int)panel->control_algorithm);
    
    for (arm_i = 0; arm_i < panel->ArmSize; arm_i++){
        printf("Arm_i: %d\n", arm_i);
        printf("JointSize: %d\n", panel->JointSize[arm_i]);
        printf("\nMotorCmd      :");
        for (i = 0; i < panel->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)panel->MotorCmd[arm_i][i]);
        }
        printf("\nJointCmd      :");
        for (i = 0; i < panel->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)panel->JointCmd[arm_i][i]);
        }
        printf("\nTaskCmd       :");
        for (i = 0; i < 6; ++ i) {
            printf("%9.4lf ", (double)panel->TaskCmd[arm_i][i]);
        }
        printf("\n");
    }
}

void print_planner_state(PlannerState* robot){
	int i, j, arm_i;
	printf("\nRobot: (received_panel_command_timestamp: %ld ArmSize: %d)\n", robot->received_panel_command_timestamp, robot->ArmSize);
    printf("setting_update_finished: %d\n", (int)robot->setting_update_finished);
    
    for (arm_i = 0; arm_i < robot->ArmSize; arm_i++){
        printf("Arm_i: %hhd\n", arm_i);
        printf("JointSize: %hhd", robot->JointSize[arm_i]);
        printf("\ninterpolated_target       :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->interpolated_target[arm_i][i]);
        }

        printf("\nJointPos        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->JointPos[arm_i][i]);
        }
        printf("\nJointVel        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->JointVel[arm_i][i]);
        }
        printf("\nJointTor        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->JointTor[arm_i][i]);
        }

        printf("\nMotorPos        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->MotorPos[arm_i][i]);
        }
        printf("\nMotorVel        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->MotorVel[arm_i][i]);
        }
        printf("\nMotorTor        :");
        for (i = 0; i < robot->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)robot->MotorTor[arm_i][i]);
        }
        printf("\nEEPose            :\n");
        for (j=0; j<7; j++){
            printf("%9.4lf ", (double)robot->EEPose[arm_i][j]);
        }
        printf("\nToolPose            :\n");
        for (j=0; j<7; j++){
            printf("%9.4lf ", (double)robot->ToolPose[arm_i][j]);
        }

        printf("\n");
    }
}

void print_planner_command(PlannerCommand* command){
	int i, arm_i;
	printf("connection_state: %hhd, command_mode: %hhd, ArmSize: %hhd\n", command->connection_state, (int)command->command_mode, command->ArmSize);
    for (arm_i = 0; arm_i < command->ArmSize; arm_i++){
        printf("Arm %d: \n", arm_i);
        printf("JointSize %d:\n", command->JointSize[arm_i]);
        printf("    Target        :");
        for (i = 0; i < command->JointSize[arm_i]; ++ i){
            printf("%9.4lf ", (double)command->Target[arm_i][i]);
        }
        printf("\n");
    }
    // for (i=0; i<MAX_MESSAGE_SIZE; i++){
    //     r2->ErrorTitle[i] = r1->ErrorTitle[i];
    //     r2->ErrorMsg[i] = r1->ErrorMsg[i];
    // }
}

void print_rt_config_state(RtConfigState* state){
    printf("arm_size: %d\n", state->arm_size);
    printf("arm_joint_size: ");
    for (int i = 0; i < state->arm_size; i++){
        printf("%d ", state->arm_joint_size[i]);
    }
    printf("\n");

    printf("gripper_size: %d\n", state->gripper_size);
    printf("gripper_joint_size: ");
    for (int i = 0; i < state->gripper_size; i++){
        printf("%d ", state->gripper_joint_size[i]);
    }
    printf("\n");

    printf("\njoint_soft_limit_position:");
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < state->arm_size; j++){
            for (int k = 0; k < state->arm_joint_size[j]; k++){
                printf("%f ", state->joint_soft_limit_position[i][j][k]);
            }
        }
        printf("| ");
    }
    printf("\njoint_soft_limit_velocity:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_soft_limit_velocity[i][j]);
        }
    }
    printf("\njoint_soft_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_soft_limit_torque[i][j]);
        }
    }

    printf("\njoint_hard_limit_position:");
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < state->arm_size; j++){
            for (int k = 0; k < state->arm_joint_size[i]; k++){
                printf("%f ", state->joint_hard_limit_position[i][j][k]);
            }
        }
        printf("| ");
    }
    printf("\njoint_hard_limit_velocity:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_hard_limit_velocity[i][j]);
        }
    }
    printf("\njoint_hard_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_hard_limit_torque[i][j]);
        }
    }
    
    printf("\njoint_follow_limit_position:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_follow_limit_position[i][j]);
        }
    }
    printf("\njoint_follow_limit_velocity:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_follow_limit_velocity[i][j]);
        }
    }
    printf("\njoint_follow_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_follow_limit_torque[i][j]);
        }
    }

    printf("\njoint_jump_limit_position:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_jump_limit_position[i][j]);
        }
    }
    printf("\njoint_jump_limit_torque:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < state->arm_joint_size[i]; j++){
            printf("%f ", state->joint_jump_limit_torque[i][j]);
        }
    }

    printf("\ntool_offset:");
    for (int i = 0; i < state->arm_size; i++){
        for (int j = 0; j < 3; j++){
            printf("%f ", state->tool_offset[i][j]);
        }
    }
    printf("\n");
}

void print_diagnostic_flags(uint32_t flags){
    if (flags == DiagnosticFlags::kNone){
        printf("None\n");
        return;
    }
    
    if (flags & DiagnosticFlags::kTargetPosSaturation){
        printf("TargetPosSaturation: User target clamped by soft position limits\n");
    }
    if (flags & DiagnosticFlags::kTargetVelSaturation){
        printf("TargetVelSaturation: User target clamped by soft velocity limits\n");
    }
    if (flags & DiagnosticFlags::kTargetTorSaturation){
        printf("TargetTorSaturation: User target clamped by soft torque limits\n");
    }
    if (flags & DiagnosticFlags::kActuatorPosSaturation){
        printf("ActuatorPosSaturation: Actuator command clamped by soft position limits\n");
    }
    if (flags & DiagnosticFlags::kActuatorVelSaturation){
        printf("ActuatorVelSaturation: Actuator command clamped by soft velocity limits\n");
    }
    if (flags & DiagnosticFlags::kActuatorTorSaturation){
        printf("ActuatorTorSaturation: Actuator command clamped by soft torque limits\n");
    }
    if (flags & DiagnosticFlags::kActuatorPosJumpSaturation){
        printf("ActuatorPosJumpSaturation: Actuator command clamped by position jump limit\n");
    }
    if (flags & DiagnosticFlags::kActuatorVelJumpSaturation){
        printf("ActuatorVelJumpSaturation: Actuator command clamped by velocity jump limit\n");
    }
    if (flags & DiagnosticFlags::kActuatorTorJumpSaturation){
        printf("ActuatorTorJumpSaturation: Actuator command clamped by torque jump limit\n");
    }
    if (flags & DiagnosticFlags::kBoundaryVelClamp){
        printf("BoundaryVelClamp: Velocity zeroed in limit direction due to position boundary reached\n");
    }
    if (flags & DiagnosticFlags::kBoundaryJointImpedance){
        printf("BoundaryJointImpedance: Joint impedance applied due to position boundary reached\n");
    }

    // --- Algorithmic & Kinematic Planner Modifications ---
    if (flags & DiagnosticFlags::kPlanTimelineExtended){
        printf("PlanTimelineExtended: Trajectory segment duration (dt) stretched for velocity limits\n");
    }
    if (flags & DiagnosticFlags::kPlanVelLimitInvalid){
        printf("PlanVelLimitInvalid: Specified max velocity profile is below tolerance(1e-4)\n");
    }
    if (flags & DiagnosticFlags::kPlanDeltaTooLarge){
        printf("PlanDeltaTooLarge: Distance between points requires unachievable time scaling\n");
    }
    if (flags & DiagnosticFlags::kPlanVelocitySnap){
        printf("PlanVelocitySnap: Large velocity shift over zero distance\n");
    }
    if (flags & DiagnosticFlags::kPlanPointSkipped){
        printf("PlanPointSkipped: Duplicated points skipped\n");
    }
    if (flags & DiagnosticFlags::kWarningStartupPositionMismatch){
        printf("WarningStartupPositionMismatch: Restore all enabled arm joints to zero\n");
    }

    // --- Fault Flags ---
    if (flags & DiagnosticFlags::kFaultPosHardLimitReached){
        printf("FaultPosHardLimitReached: Position hard limit reached\n");
    }
    if (flags & DiagnosticFlags::kFaultVelHardLimitReached){
        printf("FaultVelHardLimitReached: Velocity hard limit reached\n");
    }
    if (flags & DiagnosticFlags::kFaultTorHardLimitReached){
        printf("FaultTorHardLimitReached: Torque hard limit reached\n");
    }
    if (flags & DiagnosticFlags::kFaultPosTrackingFailed){
        printf("FaultPosTrackingFailed: Position tracking failed\n");
    }
    if (flags & DiagnosticFlags::kFaultVelTrackingFailed){
        printf("FaultVelTrackingFailed: Velocity tracking failed\n");
    }
    if (flags & DiagnosticFlags::kFaultTorTrackingFailed){
        printf("FaultTorTrackingFailed: Torque tracking failed\n");
    }
    if (flags & DiagnosticFlags::kFaultArmNotFound){
        printf("FaultArmNotFound: Arm not found\n");
    }
    if (flags & DiagnosticFlags::kFaultGripperNotFound){
        printf("FaultGripperNotFound: Gripper not found\n");
    }
    if (flags & DiagnosticFlags::kFaultHardwareInitFailed){
        printf("FaultHardwareInitFailed: Hardware initialization failed\n");
    }
    if (flags & DiagnosticFlags::kFaultUnknown){
        printf("FaultUnknown: Unknown fault\n");
    }
}