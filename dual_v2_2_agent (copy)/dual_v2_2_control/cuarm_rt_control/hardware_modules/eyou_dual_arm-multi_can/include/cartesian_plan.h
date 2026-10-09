#ifndef CARTESIAN_TRAJECTORY_H
#define CARTESIAN_TRAJECTORY_H

#include <vector>
#include <queue>
#include <tuple>

struct CartesianTrajectory {
    double cartesian_start[6];
    double cartesian_end[6];
    double duration;
};

class CartesianTrajectoryPlanner {
public:
    CartesianTrajectoryPlanner(int arm_index=0, double dt=0);
    
    void addTrajectory(const CartesianTrajectory& traj);
    bool getNextJointAngles(double* q_out, double * q_in, double t);
    bool isTrajectoryActive() const;
    
private:
    int arm_index_;
    double dt_;
    std::queue<CartesianTrajectory> traj_queue_;
    CartesianTrajectory current_traj_;
    bool arm_moving_;
    double traj_progress_;
    std::vector<double> traj_params_;
    double interpolation_factor;
    void initializeNewTrajectory();
    std::tuple<double, double, double> evaluateTrajectory(double t);
};

#endif // CARTESIAN_TRAJECTORY_H