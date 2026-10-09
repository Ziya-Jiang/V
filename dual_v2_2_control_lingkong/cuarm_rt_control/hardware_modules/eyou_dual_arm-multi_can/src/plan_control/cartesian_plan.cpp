#include "cartesian_plan.h"
#include "polynomial_trajectory.h"
#include "kinematics.h"
#include "robotics_math.h"
#include <iostream>

CartesianTrajectoryPlanner::CartesianTrajectoryPlanner(int arm_index, double dt)
    : arm_index_(arm_index), dt_(dt), arm_moving_(false), traj_progress_(0.0) {}

void CartesianTrajectoryPlanner::addTrajectory(const CartesianTrajectory& traj) {
    traj_queue_.push(traj);
}

void CartesianTrajectoryPlanner::initializeNewTrajectory() {
    current_traj_ = traj_queue_.front();
    traj_queue_.pop();
    arm_moving_ = true;
    traj_progress_ = 0.0;
    
    traj_params_ = polynomial_trajectory::calculateCoefficients(
        0, 0, 0, 1, 0, 0, current_traj_.duration);
}

std::tuple<double, double, double> CartesianTrajectoryPlanner::evaluateTrajectory(double t) {
    return polynomial_trajectory::evaluatePolynomial(traj_params_, t);
}

bool CartesianTrajectoryPlanner::getNextJointAngles(double* q_out, double * q_in, double t) {
    if (!arm_moving_ && !traj_queue_.empty()) {
        initializeNewTrajectory();
    }
    
    if (!arm_moving_) {
        return false;
    }
    
    if (t < current_traj_.duration) {
        double v, a;
        std::tie(interpolation_factor, v, a) = evaluateTrajectory(t);
        double current_cartesian[6];
        // Cartesian space interpolation
        for (int i = 0; i < 3; i++) {
            current_cartesian[i] = current_traj_.cartesian_start[i] +
                            (current_traj_.cartesian_end[i] - 
                             current_traj_.cartesian_start[i]) * interpolation_factor;
        }
        
        // Orientation interpolation using quaternion slerp
        double start_quat[4];
        robotics_math::Eul2Quat(current_traj_.cartesian_start[5], 
                              current_traj_.cartesian_start[4], 
                              current_traj_.cartesian_start[3], 
                              start_quat);
        double end_quat[4];
        robotics_math::Eul2Quat(current_traj_.cartesian_end[5], 
                              current_traj_.cartesian_end[4], 
                              current_traj_.cartesian_end[3], 
                              end_quat);
        double current_quat[4];
        robotics_math::slerpInsert(start_quat, end_quat, interpolation_factor, current_quat);
        double current_eula[3];
        robotics_math::Quat2Eul(current_quat, current_eula[2], current_eula[1], current_eula[0]);
        
        // Create target in world frame
        double target_in_world[6]{
            current_cartesian[0], current_cartesian[1], current_cartesian[2], 
            current_eula[0], current_eula[1], current_eula[2]
        };
        
        // Inverse kinematics
        double target_l7_in_world[6];
        kinematics::removeTool(target_l7_in_world, target_in_world);
        
        if (!kinematics::inverseKinematicsSingularityRobust(arm_index_, target_l7_in_world, 1e-6, 20, 
                                                         q_in, q_out)) 
        {                                          
            return true;
        } else {
            std::cerr << (arm_index_ ? "Right" : "Left")
                      << " arm trajectory execution failed at t="
                      << t << std::endl;
            t = current_traj_.duration;
        }
    }
    
    if (t >= current_traj_.duration) {
        arm_moving_ = false;
        std::cout << (arm_index_ ? "Right" : "Left") << " arm trajectory completed." << std::endl;
    }
    
    return arm_moving_;
}

bool CartesianTrajectoryPlanner::isTrajectoryActive() const {
    return arm_moving_ || !traj_queue_.empty();
}