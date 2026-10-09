#include "interpolation.h"
#include <math.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <type_traits>

#ifndef M_PI
    #define M_PI 3.14159265358979323846  /* pi */
#endif
const float kTolerance = 1e-4f;

InterpolatorFactory::InterpolatorFactory():interpolation_method_(InterpolationMethod::kCos), dt_(0.05f){

}

InterpolatorFactory::InterpolatorFactory(InterpolationMethod m, float dt, int joint_size, const std::string& name):
interpolation_method_(m), dt_(dt), joint_size_(joint_size), name_(name){
    resetInterpolator(interpolation_method_, acc_t_, VectorXd::Zero(joint_size_), VectorXd::Zero(joint_size_));
}

void InterpolatorFactory::replaceEmptyVector(VectorXd& v){
    if (v.size() == 0)
        v = VectorXd::Zero(joint_size_);
}

void InterpolatorFactory::updateParam(InterpolationMethod m, float acc_duration, float vel_duration, const VectorXd& leading_joint_max_vel){
    interpolator_ptr_.reset(); //Reset pointer to nullptr to flag that we need to create a new interpolator.
    path_interpolator_ptr_.reset(); //Reset pointer to nullptr to flag that we need to create a new interpolator.
    abstract_interpolator_ptr_ = nullptr;
    filter_.reset();
    interpolation_method_ = m;
    acc_t_ = acc_duration;
    vel_t_ = vel_duration;
    leading_joint_max_vel_ = leading_joint_max_vel;
    state_ = InterpolationMotionPhase::kInitialized;
}

//Reset single point interpolator
bool InterpolatorFactory::resetInterpolator(InterpolationMethod m, float T, VectorXd x0, VectorXd x1, VectorXd v0, VectorXd v1, VectorXd a0, VectorXd a1){
    replaceEmptyVector(v0);
    replaceEmptyVector(v1);
    replaceEmptyVector(a0);
    replaceEmptyVector(a1);
    if (T <= 0 ){
        std::cout <<"ERROR: Acceleration time must be >= 0\n";
        return false;
    }

    switch (m){
        case InterpolationMethod::kLinear:
            interpolator_ptr_ = std::make_unique<LinearInterpolator>(T, x0, x1);
            break;
        case InterpolationMethod::kCos:
            interpolator_ptr_ = std::make_unique<CosInterpolator>(T, x0, x1);
            break;
        case InterpolationMethod::kCubic:
            interpolator_ptr_ = std::make_unique<CubicInterpolator>(T, x0, x1, v0, v1);
            break;
        case InterpolationMethod::kQuintic:
            interpolator_ptr_ = std::make_unique<QuinticInterpolator>(T, x0, x1, v0, v1, a0, a1);
            break;
        default:
            throw std::invalid_argument("Reset single point interpolator Unknown interpolation method " + std::to_string((int)m) + "\n.");
    }
    interpolation_method_ = m;
    i_=0;
    abstract_interpolator_ptr_ = interpolator_ptr_.get();
    is_waypoint_interpolator_ = false;
    return true;
}

//Reset waypoint interpolator
bool InterpolatorFactory::resetInterpolator(InterpolationMethod m, VectorXd x0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time, VectorXd v0, VectorXd a0){
    if (waypoint_position.size() <= 0) return false;
    replaceEmptyVector(v0);
    replaceEmptyVector(a0);
    switch (m){
        case InterpolationMethod::kQuinticPath:
            path_interpolator_ptr_ = std::make_unique<QuinticPathInterpolator>(x0, v0, a0, waypoint_position, waypoint_time);
            break;
        default:
            throw std::invalid_argument("Reset waypoint interpolator Unknown interpolation method " + std::to_string((int)m) + "\n.");
    }
    interpolation_method_ = m;
    i_=0;
    abstract_interpolator_ptr_ = path_interpolator_ptr_.get();
    is_waypoint_interpolator_ = true;
    return true;
}

VectorXd InterpolatorFactory::getPos(AbstractInterpolator* interpolator) const{
    float t = (i_*dt_ > interpolator->getT())? interpolator->getT(): i_*dt_;
    return interpolator->getPos(t);
}

int InterpolatorFactory::getLeadingJointIndex(const VectorXd& x0, const VectorXd& x1) const {
    int leading_joint_index = 0;
    float max_time_needed = -1.0f;

    for (int i = 0; i < x0.size(); ++i) {
        float delta = std::abs(static_cast<float>(x1[i] - x0[i]));
        float max_vel = getLeadingJointMaxVelValue(i);

        // Avoid division by zero if a joint limit is invalid or zero
        if (max_vel <= kTolerance) {
            continue; 
        }

        // Time needed for this joint under pure linear scaling
        float time_needed = delta / max_vel;
        if (time_needed > max_time_needed) {
            max_time_needed = time_needed;
            leading_joint_index = i;
        }
    }
    return leading_joint_index;
}

float InterpolatorFactory::getLeadingJointPeakVel(InterpolationMethod m, float T, int leading_joint_index, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1, const VectorXd& a0, const VectorXd& a1) const{
    CubicInterpolator cubic(T, x0, x1, v0, v1);
    QuinticInterpolator quintic(T, x0, x1, v0, v1, a0, a1);
    const AbstractPointInterpolator* interpolator_ptr = nullptr;
    switch (m){
        case InterpolationMethod::kCubic:
            interpolator_ptr = &cubic;
            break;
        case InterpolationMethod::kQuintic:
            interpolator_ptr = &quintic;
            break;
        default:
            throw std::invalid_argument("getLeadingJointPeakVel: Unknown interpolation method " + std::to_string((int)m) + "\n.");
    }

    constexpr int kSamples = 60;
    float max_abs_vel = 0.0f;
    for (int i = 0; i <= kSamples; ++i){
        float t = T * static_cast<float>(i) / static_cast<float>(kSamples);
        VectorXd v = interpolator_ptr->getVel(t);
        max_abs_vel = std::max(max_abs_vel, std::abs(static_cast<float>(v[leading_joint_index])));
    }
    return max_abs_vel;
}

float InterpolatorFactory::getLeadingJointMaxVelValue(int leading_joint_index) const{
    if (leading_joint_max_vel_.size() == 0){
        return -1.0f;
    }
    if (leading_joint_max_vel_.size() <= leading_joint_index){
        return -1.0f;
    }
    return static_cast<float>(leading_joint_max_vel_[leading_joint_index]);
}

float InterpolatorFactory::getMinAllowedTime(InterpolationMethod m, double x0, double x1, double v0, double v1, double max_vel) const{
    double delta = std::abs(x1 - x0);

    // Catch pure velocity pass-throughs when position delta is zero
    if (delta <= kTolerance) {
        double delta_v = std::abs(v1 - v0);
        if (delta_v <= kTolerance) {
            // Constant velocity cruise or perfectly stationary, zero time needed
            return 0.0f; 
        } else {
            // Velocity is changing over zero distance! Calculate safe kinetic time.
            // Using a conservative estimated max acceleration limit (e.g., 15.0 rad/s^2)
            warning_flags_ |= DiagnosticFlags::kPlanVelocitySnap;
            constexpr float kEstimatedMaxAcc = 15.0f; 
            return static_cast<float>(delta_v / kEstimatedMaxAcc);
        }
    }

    if (m == InterpolationMethod::kLinear) {
        return static_cast<float>(delta / max_vel);
    }else if (m == InterpolationMethod::kCos) {
        return static_cast<float>((delta * M_PI) / (2.0f * max_vel));
    }else if (m == InterpolationMethod::kCubic || m == InterpolationMethod::kQuintic) {
        double term1 = 15.0f * delta;
        double term2 = std::sqrt(225.0f * delta * delta + 320.0f * std::max(v0, v1) * delta);
        return static_cast<float>((term1 + term2) / (8.0f * max_vel));
    }else{
        throw std::invalid_argument("getMinAllowedTime: Unknown interpolation method " + std::to_string((int)m) + "\n.");
    }
}

bool InterpolatorFactory::isSinglePointDurationValid(InterpolationMethod m, float user_T, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1) const {
    // 1. Time must be a positive, real number
    if (user_T <= kTolerance) return false;

    // 2. Loop through all joints to ensure none of them break their limits
    for (int j = 0; j < x0.size(); ++j) {
        float joint_max_vel = getLeadingJointMaxVelValue(j); 
        if (joint_max_vel <= kTolerance) continue;
        if (std::abs(v0[j]) > joint_max_vel || std::abs(v1[j]) > joint_max_vel) return false;

        float min_allowed_T = getMinAllowedTime(m, x0[j], x1[j], v0[j], v1[j], joint_max_vel);
        if (user_T < min_allowed_T) {
            return false;
        }
    }
    return true;
}

float InterpolatorFactory::resolveSinglePointDuration(InterpolationMethod m, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1, const VectorXd& a0, const VectorXd& a1) const{
    if (acc_t_ > 0  && isSinglePointDurationValid(m, acc_t_, x0, x1, v0, v1)){
        return acc_t_;
    }
    if (acc_t_ > 0) warning_flags_ |= DiagnosticFlags::kPlanTimelineExtended;
    

    const int leading_joint_index = getLeadingJointIndex(x0, x1);
    const float leading_joint_max_vel = getLeadingJointMaxVelValue(leading_joint_index);
    if (leading_joint_max_vel <= kTolerance){
        warning_flags_ |= DiagnosticFlags::kPlanVelLimitInvalid;
        return kDefaultAccT;
    }
    
    const float leading_joint_delta = std::abs(static_cast<float>(x1[leading_joint_index] - x0[leading_joint_index]));
    if (leading_joint_delta <= kTolerance){
        return std::max(dt_, kTolerance);
    }

    // For linear and cosine interpolation, we can use the leading joint max velocity to resolve the duration.
    if (m == InterpolationMethod::kLinear) {
        return std::max({dt_, kTolerance, leading_joint_delta / leading_joint_max_vel});
    }else if (m == InterpolationMethod::kCos) {
        float T_cos = (leading_joint_delta * static_cast<float>(M_PI)) / (2.0f * leading_joint_max_vel);
        return std::max({dt_, kTolerance, T_cos});
    }

    // Extract the leading joint's data and create 1D vectors for the interpolator.
    VectorXd x0_1d = VectorXd::Constant(1, x0[leading_joint_index]);
    VectorXd x1_1d = VectorXd::Constant(1, x1[leading_joint_index]);
    VectorXd v0_1d = VectorXd::Constant(1, v0[leading_joint_index]);
    VectorXd v1_1d = VectorXd::Constant(1, v1[leading_joint_index]);
    VectorXd a0_1d = VectorXd::Constant(1, a0[leading_joint_index]);
    VectorXd a1_1d = VectorXd::Constant(1, a1[leading_joint_index]);

    float min_kinematic_T = std::max({
        std::abs(static_cast<float>(v1[leading_joint_index] - v0[leading_joint_index])) / 10.0f, // Rough max acceleration estimation limit
        leading_joint_delta / leading_joint_max_vel
    });

    float low_T = std::max({dt_, kTolerance, min_kinematic_T});
    float high_T = std::max(kDefaultAccT, low_T);
    float peak_vel = getLeadingJointPeakVel(m, high_T, 0, x0_1d, x1_1d, v0_1d, v1_1d, a0_1d, a1_1d);

    // Expansion Phase
    constexpr int kExpandSteps = 40;
    for (int i = 0; i < kExpandSteps && peak_vel > leading_joint_max_vel; ++i){
        low_T = high_T;
        high_T *= 1.5f;
        peak_vel = getLeadingJointPeakVel(m, high_T, 0, x0_1d, x1_1d, v0_1d, v1_1d, a0_1d, a1_1d);
    }

    if (peak_vel > leading_joint_max_vel){
        warning_flags_ |= DiagnosticFlags::kPlanDeltaTooLarge;
        return kDefaultAccT;
    }

    // Bisection Phase
    constexpr int kBisectionSteps = 30;
    float best_T = high_T;
    for (int i = 0; i < kBisectionSteps; ++i){
        float mid_T = 0.5f * (low_T + high_T);
        float mid_peak_vel = getLeadingJointPeakVel(m, mid_T, 0, x0_1d, x1_1d, v0_1d, v1_1d, a0_1d, a1_1d);
        if (mid_peak_vel <= leading_joint_max_vel){
            best_T = mid_T;
            high_T = mid_T;
        }else{
            low_T = mid_T;
        }
    }
    return best_T;
}

bool InterpolatorFactory::isFluidWaypointDurationValid(InterpolationMethod m, const VectorXd& x0, const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& pos, const std::vector<float>& waypoint_time, std::vector<VectorXd> max_vel_limits) const {
    QuinticPathInterpolator quintic_path(x0, v0, a0, pos, waypoint_time);
    const AbstractPathInterpolator* interpolator_ptr = nullptr;
    switch (m){
        case InterpolationMethod::kQuinticPath:
            interpolator_ptr = &quintic_path;
            break;
        default:
            throw std::invalid_argument("isFluidWaypointDurationValid: Unknown interpolation method " + std::to_string((int)m) + "\n.");
    }

    size_t segment_count = pos.size() - 1;
    constexpr int kSamplesPerSegment = 40; 
    for (size_t i = 0; i < segment_count; ++i) {
        double T = waypoint_time[i];
        for (int step = 0; step <= kSamplesPerSegment; ++step) {
            float t = static_cast<float>(T * static_cast<float>(step) / kSamplesPerSegment);
            VectorXd actual_vel = interpolator_ptr->getVel(t);

            for (int joint_j = 0; joint_j < actual_vel.size(); ++joint_j) {
                double max_allowed = max_vel_limits[i][joint_j];
                if (std::abs(actual_vel[joint_j]) > max_allowed) {
                    return false; 
                }
            }
        }
    }
    return true; 
}

// Estimates fluid continuous velocities using predictive time buffers
std::vector<VectorXd> InterpolatorFactory::estimateFluidWaypointVelocities(
    InterpolationMethod m, 
    const std::vector<VectorXd>& pos, 
    const VectorXd& v0, 
    const std::vector<VectorXd>& max_vel) const {
    if (m != InterpolationMethod::kQuinticPath) {
        throw std::invalid_argument("estimateFluidWaypointVelocities: Unknown interpolation method " + std::to_string((int)m) + "\n.");
    }

    size_t total_points = pos.size();
    size_t segment_count = total_points - 1;
    int joint_size = static_cast<int>(pos[0].size());
    
    std::vector<float> initial_dt(segment_count, 5.0f);
    for (size_t i = 0; i < segment_count; ++i) {
        float max_linear_time = 0.1f;
        for (int j = 0; j < joint_size; ++j) {
            float dist = std::abs(static_cast<float>(pos[i+1][j] - pos[i][j]));
            float max_v = static_cast<float>(max_vel[i][j]);
            if (max_v > kTolerance) max_linear_time = std::max(max_linear_time, dist / max_v);
        }
        initial_dt[i] = max_linear_time;
    }

    std::vector<VectorXd> estimated_vel(total_points, VectorXd::Zero(joint_size));
    estimated_vel.front() = v0; 
    estimated_vel.back() = VectorXd::Zero(joint_size); 

    for (size_t i = 1; i < segment_count; ++i) {
        double total_dt = initial_dt[i-1] + initial_dt[i];
        if (total_dt > kTolerance) {
            estimated_vel[i] = (pos[i+1] - pos[i-1]) / total_dt;
        }
    }
    return estimated_vel;
}

// Optimized continuous evaluator function
std::vector<float> InterpolatorFactory::evaluateDtFromLeadingJointMaxVelContinuous(
    InterpolationMethod m, 
    const std::vector<VectorXd>& pos, 
    const std::vector<VectorXd>& vel,
    const std::vector<VectorXd>& leading_joint_max_vel) const {

    InterpolationMethod single_point_method;
    if (m == InterpolationMethod::kQuinticPath) {
        single_point_method = InterpolationMethod::kQuintic;
    }else{
        throw std::invalid_argument("evaluateDtFromLeadingJointMaxVelContinuous: Unknown interpolation method " + std::to_string((int)m) + "\n.");
    }


    const size_t segment_count = pos.size() > 0 ? pos.size() - 1 : 0;
    std::vector<float> waypoint_time(segment_count, kDefaultAccT);

    for (size_t i = 0; i < segment_count; ++i) {
        float max_time_needed = -1.0f;
        for (int joint_j = 0; joint_j < pos[i].size(); ++joint_j) {
            float min_time = getMinAllowedTime(single_point_method, pos[i][joint_j], pos[i+1][joint_j], vel[i][joint_j], vel[i+1][joint_j], leading_joint_max_vel[i][joint_j]);
            if (min_time > max_time_needed) {
                max_time_needed = min_time;
            }
        }

        if (max_time_needed <= kTolerance) {
            waypoint_time[i] = std::max(dt_, kTolerance);
        }else{
            waypoint_time[i] = std::max(kTolerance, max_time_needed);
        }
    }
    return waypoint_time;
}

void InterpolatorFactory::filterWaypoints(std::vector<VectorXd>& waypoint_pos, std::vector<float>& waypoint_time, std::vector<VectorXd>& leading_joint_max_vel) const {
    if (waypoint_pos.size() < 2) return;

    // Cache initial sizes before modifying the vectors
    const size_t initial_size = waypoint_pos.size();
    const bool has_time = (waypoint_time.size() == initial_size);
    const bool has_vel  = (leading_joint_max_vel.size() == initial_size);

    std::vector<size_t> filtered_indices;
    filtered_indices.reserve(initial_size);
    filtered_indices.push_back(0); // Always keep the starting waypoint

    for (size_t i = 1; i < initial_size; ++i) {
        size_t last_accepted = filtered_indices.back();
        double delta = (waypoint_pos[i] - waypoint_pos[last_accepted]).norm();
        bool is_last_point = (i == initial_size - 1);

        if (delta > kTolerance) {
            filtered_indices.push_back(i);
        }else {
            // Force-overwrite the last accepted point with the exact terminal destination
            // to preserve the user's explicit terminal goal orientation boundary.
            if (is_last_point) filtered_indices.back() = i; 
            warning_flags_ |= DiagnosticFlags::kPlanPointSkipped;
        }
    }
    // If nothing was skipped, drop out early
    if (filtered_indices.size() == initial_size) return;

    //Extract filtered elements
    auto extract_filtered = [&](auto& target_vector) {
        using T = typename std::decay_t<decltype(target_vector)>::value_type;
        std::vector<T> temp;
        temp.reserve(filtered_indices.size());
        for (size_t idx : filtered_indices) {
            temp.push_back(std::move(target_vector[idx]));
        }
        target_vector = std::move(temp);
    };

    extract_filtered(waypoint_pos);
    if (has_time) extract_filtered(waypoint_time);
    if (has_vel)  extract_filtered(leading_joint_max_vel);
}

std::vector<float> InterpolatorFactory::resolveFluidWaypointDuration(InterpolationMethod m, const VectorXd& x0, const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time, std::vector<VectorXd> leading_joint_max_vel) const{
    // Insert current position at the beginning
    std::vector<VectorXd> waypoint_position1 = waypoint_position;
    waypoint_position1.insert(waypoint_position1.begin(), x0);

    std::vector<float> waypoint_duration;
    if (waypoint_time.size() == waypoint_position.size()) {
        bool is_safe = isFluidWaypointDurationValid(m, x0, v0, a0, waypoint_position1, waypoint_time, leading_joint_max_vel);

        if (is_safe) {
            waypoint_duration = waypoint_time;
        } else {
            warning_flags_ |= DiagnosticFlags::kPlanTimelineExtended;
            
            // Auto-Correction Fallback: Extrapolate continuous velocity matrices
            std::vector<VectorXd> estimated_vel = estimateFluidWaypointVelocities(m, waypoint_position1, v0, leading_joint_max_vel);
            waypoint_duration = evaluateDtFromLeadingJointMaxVelContinuous(m, waypoint_position1, estimated_vel, leading_joint_max_vel);
        }
    }else if (leading_joint_max_vel.size() == waypoint_position.size()) {
        std::vector<VectorXd> estimated_vel = estimateFluidWaypointVelocities(m, waypoint_position1, v0, leading_joint_max_vel);
        waypoint_duration = evaluateDtFromLeadingJointMaxVelContinuous(m, waypoint_position1, estimated_vel, leading_joint_max_vel);
    }else {
        throw std::invalid_argument("WARNING: waypoint_time and leading_joint_max_vel size mismatch\n");
    }
    return waypoint_duration;
}

bool InterpolatorFactory::isCurrentTarget(const VectorXd& x1, const VectorXd& v1, const VectorXd& a1) const{
    if (current_x1_.size() != x1.size() || current_v1_.size() != v1.size() || current_a1_.size() != a1.size()){
        return false;
    }

    return current_x1_.isApprox(x1, kTolerance) && current_v1_.isApprox(v1, kTolerance) && current_a1_.isApprox(a1, kTolerance);
}

bool InterpolatorFactory::isCurrentTarget(std::vector<VectorXd> waypoint_position, const std::vector<float>& waypoint_time, std::vector<VectorXd> leading_joint_max_vel) const{
    if (current_waypoint_position_.size() != waypoint_position.size()){
        return false;
    }

    for (size_t i = 0; i < waypoint_position.size(); ++i){
        if (current_waypoint_position_[i].size() != waypoint_position[i].size() || !current_waypoint_position_[i].isApprox(waypoint_position[i], kTolerance)){
            return false;
        }
    }
    
    if (waypoint_time.size() > 0){
        if (current_waypoint_time_.size() != waypoint_time.size()){
            return false;
        }
        for (size_t i = 0; i < waypoint_position.size(); ++i){
            if (std::abs(current_waypoint_time_[i] - waypoint_time[i]) > kTolerance){
                return false;
            }
        }
    }else{
        if (current_waypoint_leading_joint_max_vel_.size() != leading_joint_max_vel.size()){
            return false;
        }
        for (size_t i = 0; i < waypoint_position.size(); ++i){
            if (current_waypoint_leading_joint_max_vel_[i].size() != leading_joint_max_vel[i].size() || !current_waypoint_leading_joint_max_vel_[i].isApprox(leading_joint_max_vel[i], kTolerance)){
                return false;
            }
        }
    }
    return true;
}

//The curve stays at final position after acceleration, used for waypoint interpolation.
VectorXd InterpolatorFactory::getNextPos(VectorXd x0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time, std::vector<VectorXd> leading_joint_max_vel, VectorXd v0, VectorXd a0){
    if (waypoint_position.size() == 0 || (waypoint_time.size() == 0 && leading_joint_max_vel.size() == 0) ||
        (waypoint_time.size() > 0 && waypoint_position.size() != waypoint_time.size()) ||
        (leading_joint_max_vel.size() > 0 && leading_joint_max_vel.size() != waypoint_position.size())){
        return x0;
    }
    if (getInterpolationMethod() == InterpolationMethod::kNone){
        throw std::invalid_argument("getNextPos for waypoint dont support InterpolationMethod::kNone\n");
    }
    replaceEmptyVector(v0);
    replaceEmptyVector(a0);

    //If not current target, reset interpolator
    if (!abstract_interpolator_ptr_ || !path_interpolator_ptr_ || !isCurrentTarget(waypoint_position, waypoint_time, leading_joint_max_vel)){
        warning_flags_ = DiagnosticFlags::kNone;

        //Remove duplicate waypoints
        std::vector<VectorXd> filter_waypoint_position = waypoint_position;
        std::vector<float> filter_waypoint_time = waypoint_time;
        std::vector<VectorXd> filter_leading_joint_max_vel = leading_joint_max_vel;
        filterWaypoints(filter_waypoint_position, filter_waypoint_time, filter_leading_joint_max_vel);

        //Resolve duration
        std::vector<float> reallocated_dt = resolveFluidWaypointDuration(interpolation_method_, x0, v0, a0, filter_waypoint_position, filter_waypoint_time, filter_leading_joint_max_vel);
        state_ = InterpolationMotionPhase::kInitialized;
        if (resetInterpolator(interpolation_method_, x0, filter_waypoint_position, reallocated_dt, v0, a0)){
            current_waypoint_position_ = waypoint_position;
            current_waypoint_time_ = waypoint_time;
            current_waypoint_leading_joint_max_vel_ = leading_joint_max_vel;
        }else{
            return x0;
        }
    }

    if (isInterrupted()){
        return x0;
    }

    if (isCompleted(path_interpolator_ptr_->getT())){
        state_ = InterpolationMotionPhase::kFinished;
    }else{
        i_+=1;
        state_ = InterpolationMotionPhase::kAcceleration;
    }
    return getPos(path_interpolator_ptr_.get());
}

//The curve stays at final position after acceleration, used for single point interpolation.
VectorXd InterpolatorFactory::getNextPos(VectorXd x0, VectorXd x1, VectorXd v0, VectorXd v1, VectorXd a0, VectorXd a1){
    if (getInterpolationMethod() == InterpolationMethod::kNone){
        if (!filter_){
            float cutoff_freq = 2;
            float output_freq = static_cast<float>(1.0 / dt_);
            filter_ = std::make_unique<LowPass2ndFilter>(x0, cutoff_freq, output_freq, leading_joint_max_vel_, acc_t_);
        }
        return filter_->getPos(x1);
    }
    
    replaceEmptyVector(v0);
    replaceEmptyVector(v1);
    replaceEmptyVector(a0);
    replaceEmptyVector(a1);
    if (!abstract_interpolator_ptr_ || !interpolator_ptr_ || !isCurrentTarget(x1, v1, a1)){ 
        warning_flags_ = DiagnosticFlags::kNone;

        float interpolation_T = resolveSinglePointDuration(interpolation_method_, x0, x1, v0, v1, a0, a1);
        state_ = InterpolationMotionPhase::kInitialized;
        if (resetInterpolator(interpolation_method_, interpolation_T, x0, x1, v0, v1, a0, a1)){
            current_x1_ = x1;
            current_v1_ = v1;
            current_a1_ = a1;
        }else{
            return x0;
        }
    }
    
    if (isInterrupted()){
        return x0;
    }

    if ( x0.isApprox(x1, kTolerance) || interpolator_ptr_ && isCompleted(interpolator_ptr_->getT())){
        state_ = InterpolationMotionPhase::kFinished;
    }else{
        i_+=1;
        state_ = InterpolationMotionPhase::kAcceleration;
    }
    return getPos(interpolator_ptr_.get());
}

void InterpolatorFactory::interrupt(){
    state_ = InterpolationMotionPhase::kInterrupted;
}

bool InterpolatorFactory::isInterrupted() const{
    return state_ == InterpolationMotionPhase::kInterrupted;
}

bool InterpolatorFactory::isCompleted(float t){
    return (i_*dt_ >= t)? true: false;
}

InterpolationMotionPhase InterpolatorFactory::getState() const{
    return state_;
}

InterpolationMethod InterpolatorFactory::getInterpolationMethod() const{
    return interpolation_method_;
}

int InterpolatorFactory::getInterpolatorIndex() const{
    return i_;
}

void InterpolatorFactory::reset(){
    abstract_interpolator_ptr_ = nullptr;
    filter_.reset();
}

void InterpolatorFactory::getPositionTarget(VectorXd& x1) const{
    if (getInterpolationMethod() == InterpolationMethod::kNone){
        if (!filter_) return;
        x1 = filter_->getPositionTarget();
    }
    
    if (!abstract_interpolator_ptr_) return;
    float t = (i_*dt_ > abstract_interpolator_ptr_->getT())? abstract_interpolator_ptr_->getT(): i_*dt_;
    abstract_interpolator_ptr_->getPositionTarget(t, x1);
}

int InterpolatorFactory::getPositionTargetIndex() const{
    if (!abstract_interpolator_ptr_ || !is_waypoint_interpolator_) return 0;
    float t = (i_*dt_ > path_interpolator_ptr_->getT())? path_interpolator_ptr_->getT(): i_*dt_;
    return path_interpolator_ptr_->getPositionTargetIndex(t);
}

uint32_t InterpolatorFactory::getWarningFlags() const{
    return warning_flags_;
}

float InterpolatorFactory::getInterpolationT() const{
    return (is_waypoint_interpolator_)? path_interpolator_ptr_->getT(): interpolator_ptr_->getT();
}

VectorXd InterpolatorFactory::getPosAtTime(float t) const{
    return (is_waypoint_interpolator_)? path_interpolator_ptr_->getPos(t): interpolator_ptr_->getPos(t);
}
//////////////////////////////////LinearInterpolator////////////////////////////////////////
LinearInterpolator::LinearInterpolator(float T, const VectorXd& x0, const VectorXd& x1): AbstractPointInterpolator(T){
    // Linear interpolation: p(t) = x0 + (x1 - x0) * (t / T)
    x0_ = x0;
    x1_ = x1;
}

VectorXd LinearInterpolator::getPos(float t) const{
    return x0_ + (x1_ - x0_) * (t / T_);
}

VectorXd LinearInterpolator::getVel(float t) const{
    return (x1_ - x0_) / T_;
}

void LinearInterpolator::getPositionTarget(float t, VectorXd& x1) const{
    x1 = x1_;
}

//////////////////////////////////CosInterpolator////////////////////////////////////////
CosInterpolator::CosInterpolator(float T, const VectorXd& x0, const VectorXd& x1): AbstractPointInterpolator(T){
    // Cosine interpolation: p(t) = x0 + (x1 - x0) * (1 - cos(pi * t / T)) / 2
    x0_ = x0;
    x1_ = x1;
}

VectorXd CosInterpolator::getPos(float t) const{
    return x0_ + (x1_ - x0_) * (1.0 - cos(M_PI * t / T_)) * 0.5;
}

VectorXd CosInterpolator::getVel(float t) const{
    return (x1_ - x0_) * sin(M_PI * t / T_) * 0.5 * M_PI / T_;
}

void CosInterpolator::getPositionTarget(float t, VectorXd& x1) const{
    x1 = x1_;
}

//////////////////////////////////CubicInterpolator////////////////////////////////////////
CubicInterpolator::CubicInterpolator(float T, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1): AbstractPointInterpolator(T){
    // Cubic polynomial: p(t) = A + B*t + C*t^2 + D*t^3
    x0_ = x0;
    x1_ = x1;
    v0_ = v0;
    v1_ = v1;
    A_ = x0;
    B_ = v0;
    C_ = (3 * (x1 - x0) / (T * T)) - (2 * v0 / T) - (v1 / T);
    D_ = (2 * (x0 - x1) / (T * T * T)) + ( (v0 + v1) / (T * T));
}

VectorXd CubicInterpolator::getPos(float t) const{
    return A_ + B_ * t + C_ * t * t + D_ * t * t * t;
}

VectorXd CubicInterpolator::getVel(float t) const{
    return B_ + 2 * C_ * t + 3 * D_ * t * t;
}

void CubicInterpolator::getPositionTarget(float t, VectorXd& x1) const{
    x1 = x1_;
}

//////////////////////////////////QuinticInterpolator////////////////////////////////////////
QuinticInterpolator::QuinticInterpolator(float T, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1, const VectorXd& a0, const VectorXd& a1): AbstractPointInterpolator(T){
    // 5th order polynomial: p(t) = A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5
    x0_ = x0;
    x1_ = x1;
    v0_ = v0;
    v1_ = v1;
    a0_ = a0;
    a1_ = a1;
    
    VectorXd dx = x1_ - (x0_ + v0_ * T + 0.5 * a0_ * T * T);
    VectorXd dv = v1_ - (v0_ + a0_ * T);
    VectorXd da = a1_ - a0_;
    A_ = x0_;
    B_ = v0_;
    C_ = a0_ * 0.5;
    D_ = (10.0 * dx / (T * T * T)) - (4.0 * dv / (T * T)) + (0.5 * da / T);
    E_ = (-15.0 * dx / (T * T * T * T)) + (7.0 * dv / (T * T * T)) - (da / (T * T));
    F_ = (6.0 * dx / (T * T * T * T * T)) - (3.0 * dv / (T * T * T * T)) + (0.5 * da / (T * T * T));
}

QuinticInterpolator::QuinticInterpolator(float T, const std::vector<VectorXd>& coefficients): AbstractPointInterpolator(T){
    A_ = coefficients[0];
    B_ = coefficients[1];
    C_ = coefficients[2];
    D_ = coefficients[3];
    E_ = coefficients[4];
    F_ = coefficients[5];
    x0_ = A_;
    v0_ = B_;
    a0_ = 2*C_;
    x1_ = getPos(T);
    v1_ = getVel(T);
    a1_ = getAcc(T);
}

VectorXd QuinticInterpolator::getPos(float t) const{
    return A_ + B_ * t + C_ * t * t + D_ * t * t * t + E_ * t * t * t * t + F_ * t * t * t * t * t;
}

VectorXd QuinticInterpolator::getVel(float t) const{
    return B_ + 2 * C_ * t + 3 * D_ * t * t + 4 * E_ * t * t * t + 5 * F_ * t * t * t * t;
}

VectorXd QuinticInterpolator::getAcc(float t) const{
    return 2 * C_ + 6 * D_ * t + 12 * E_ * t * t + 20 * F_ * t * t * t;
}

void QuinticInterpolator::getPositionTarget(float t, VectorXd& x1) const{
    x1 = x1_;
}

//////////////////////////////////QuinticPathInterpolator////////////////////////////////////////
QuinticPathInterpolator::QuinticPathInterpolator(const VectorXd& x0, const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time)
    : AbstractPathInterpolator(0.0f), x0_(x0), v0_(v0), a0_(a0), waypoint_position_(waypoint_position), waypoint_time_(waypoint_time){
    
    // Insert start position at the beginning
    std::vector<VectorXd> waypoint_position1 = waypoint_position;
    waypoint_position1.insert(waypoint_position1.begin(), x0);
    
    std::vector<std::vector<VectorXd>> coefficients = evaluateCoefficients(v0, a0, waypoint_position1, waypoint_time_);

    for (size_t i = 0; i < coefficients.size(); ++i){
        float T = waypoint_time_[i];
        segment_interpolators_.emplace_back(std::make_unique<QuinticInterpolator>(T, coefficients[i]));
        T_ += T;
    }
}

std::vector<std::vector<VectorXd>> QuinticPathInterpolator::evaluateCoefficients(const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& pos, const std::vector<float>& waypoint_time){
    int N = static_cast<int>(pos.size()) - 1;
    int unknowns = 6 * N;
    int joint_size = static_cast<int>(pos[0].size());
    std::vector<std::vector<VectorXd>> coefficients(N, std::vector<VectorXd>(6, VectorXd::Zero(joint_size))); // shape: [waypoint_size, 6, joint_size] 

    for (int joint_i=0; joint_i < joint_size; joint_i++){
        // Build global system
        MatrixXd M = MatrixXd::Zero(unknowns, unknowns);
        VectorXd b = VectorXd::Zero(unknowns);

        int row = 0;
        // For each segment, add position constraints at start and end
        double current_time = 0;
        for (size_t i = 0; i < N; ++i) {
            double T = waypoint_time[i];

            // Start position
            M(row, 6*i+0) = 1;
            b(row) = pos[i][joint_i];
            row++;

            // End position
            M(row, 6*i+0) = 1;
            M(row, 6*i+1) = T;
            M(row, 6*i+2) = T*T;
            M(row, 6*i+3) = pow(T,3);
            M(row, 6*i+4) = pow(T,4);
            M(row, 6*i+5) = pow(T,5);
            b(row) = pos[i+1][joint_i];
            row++;
        }

        // Continuity constraints between segments
        for (size_t i = 0; i < N-1; ++i) {
            double T = waypoint_time[i];

            // Velocity continuity: v_i(T) - v_{i+1}(0) = 0
            M(row, 6*i+1) = 1;
            M(row, 6*i+2) = 2*T;
            M(row, 6*i+3) = 3*T*T;
            M(row, 6*i+4) = 4*pow(T,3);
            M(row, 6*i+5) = 5*pow(T,4);
            M(row, 6*(i+1)+1) = -1; // subtract v_{i+1}(0) = a1_{i+1}
            row++;

            // Acceleration continuity: a_i(T) - a_{i+1}(0) = 0
            M(row, 6*i+2) = 2;
            M(row, 6*i+3) = 6*T;
            M(row, 6*i+4) = 12*T*T;
            M(row, 6*i+5) = 20*pow(T,3);
            M(row, 6*(i+1)+2) = -2; // subtract a_{i+1}(0) = 2*a2_{i+1}
            row++;
        }

        // Boundary conditions: set velocity and acceleration at start
        M(row, 1) = 1; //v(0) = v0
        b[row] = v0[joint_i];
        row++;
        M(row, 2) = 2; //a(0) = a0
        b[row] = a0[joint_i];
        row++;

        // Boundary conditions: zero velocity and acceleration at end
        double Tlast = waypoint_time.back();
        M(row, 6*(N-1)+1) = 1;
        M(row, 6*(N-1)+2) = 2*Tlast;
        M(row, 6*(N-1)+3) = 3*Tlast*Tlast;
        M(row, 6*(N-1)+4) = 4*pow(Tlast,3);
        M(row, 6*(N-1)+5) = 5*pow(Tlast,4);
        row++;
        M(row, 6*(N-1)+2) = 2;
        M(row, 6*(N-1)+3) = 6*Tlast;
        M(row, 6*(N-1)+4) = 12*Tlast*Tlast;
        M(row, 6*(N-1)+5) = 20*pow(Tlast,3);
        row++;

        // Solve
        VectorXd x = M.fullPivLu().solve(b);
        for (int pt_i = 0; pt_i < N; pt_i++) {
            for (int coe_i=0; coe_i<6; coe_i++){
                coefficients[pt_i][coe_i][joint_i] = x[pt_i * 6 + coe_i];
            }
        }
    }
    return coefficients;
}

VectorXd QuinticPathInterpolator::getPos(float t) const{
    auto [i, t1] = getSegmentIndexAndTime(t);
    return segment_interpolators_[i]->getPos(t1);
}

VectorXd QuinticPathInterpolator::getVel(float t) const{
    auto [i, t1] = getSegmentIndexAndTime(t);
    return segment_interpolators_[i]->getVel(t1);
}

std::tuple<int, float> QuinticPathInterpolator::getSegmentIndexAndTime(float t) const{
    float accumulated_time = 0.0f;
    int ind = static_cast<int>(waypoint_time_.size()) - 1;
    float t1 = waypoint_time_.back();
    for (int i = 0; i < waypoint_time_.size(); ++i){
        if (t <= accumulated_time + waypoint_time_[i]){
            ind = i;
            t1 = t - accumulated_time;
            break;
        }
        accumulated_time += waypoint_time_[i];
    }
    return std::make_tuple(ind, t1);
}

void QuinticPathInterpolator::getPositionTarget(float t, VectorXd& x1) const{
    if (segment_interpolators_.size() == 0) 
        return;
    auto [i, t1] = getSegmentIndexAndTime(t);
    segment_interpolators_[i]->getPositionTarget(t1, x1);
}

int QuinticPathInterpolator::getPositionTargetIndex(float t) const{
    if (segment_interpolators_.size() == 0) return -1;
    auto [i, t1] = getSegmentIndexAndTime(t);
    return i;
}

//////////////////////////////////LowPassFilter////////////////////////////////////////
LowPassFilter::LowPassFilter(const VectorXd& x0, float cutoff_frequency, float output_frequency){
    alpha_ = calculateTimeConstant(cutoff_frequency, output_frequency);
    last_output_ = x0;
}

VectorXd LowPassFilter::getPos(VectorXd x0){
    last_input_ = x0;
    last_output_ = (alpha_ * x0) + ((1.0f - alpha_) * last_output_);
    return last_output_;
}

VectorXd LowPassFilter::getPositionTarget(){
    return last_input_;
}

float LowPassFilter::calculateTimeConstant(float fc, float fs){
    if (fs <= 0.0f || fc <= 0.0f) return 1.0f;
    
    float tau = static_cast<float>(1.0f / (2.0f * M_PI * fc));
    float dt = 1.0f / fs;
    
    return dt / (tau + dt);
}

//////////////////////////////////LowPass2ndFilter////////////////////////////////////////
LowPass2ndFilter::LowPass2ndFilter(const VectorXd& x0, float cutoff_frequency, float output_frequency, const VectorXd& max_velocity_limits, float T){
    calculateTimeConstant(cutoff_frequency, output_frequency);
    x1_ = x0;
    x2_ = x0;
    y1_ = x0;
    y2_ = x0;
    last_input_ = x0;
    reach_target_time_ = T;
    remaining_target_time_ = T;
    dt_ = 1.0 / output_frequency;


    // For a Butterworth filter (Q=0.707), peak output velocity = 2.45 * fc * delta_x
    // Therefore, max allowed input delta_x = max_velocity / (2.45 * fc)
    double response_factor = 2.45;
    max_input_step_.resize(max_velocity_limits.size());
    for (int i = 0; i < max_velocity_limits.size(); ++i) {
        double max_step_per_second = max_velocity_limits[i] / response_factor;
        max_input_step_[i] = max_step_per_second * dt_; 
    }
}

VectorXd LowPass2ndFilter::getPos(VectorXd x0){
    if (!x0.isApprox(last_input_, kTolerance)) {
        remaining_target_time_ = reach_target_time_;
    }

    Eigen::VectorXd clamped_x0 = x0;
    
    for (int i = 0; i < x0.size(); ++i) {
        // Calculate the raw distance remaining to the ultimate target
        double delta = x0[i] - x1_[i]; 
        
        // Default to our hard hardware velocity limit step
        double active_max_step = max_input_step_[i];
        
        // If a valid target time is provided, evaluate the time-based step profile
        if (remaining_target_time_ > 0.0) {
            // Velocity needed to reach target on time = distance / time
            double required_velocity = std::abs(delta) / (remaining_target_time_);
            
            // Convert required velocity to a step size per control loop cycle
            double time_based_step = required_velocity * dt_;
            
            // Choose the smaller step (Time constraint vs Hard Safety Speed Limit)
            active_max_step = std::min(active_max_step, time_based_step);
            remaining_target_time_ -= static_cast<int>(dt_);
            if (remaining_target_time_ < 0.0) remaining_target_time_ = 0.0; 
        }
        
        // Apply the chosen step constraint
        if (std::abs(delta) > active_max_step) {
            clamped_x0[i] = (delta > 0) ? x1_[i] + active_max_step : x1_[i] - active_max_step;
        }
    }

    last_input_ = x0;
    last_clamped_input_ = clamped_x0;

    // 2. Standard 2nd-Order Biquad Difference Equation using the safely clamped input
    Eigen::VectorXd y0 = (b0_ * clamped_x0) + (b1_ * x1_) + (b2_ * x2_) - (a1_ * y1_) - (a2_ * y2_);

    // 3. Update history (shift values back)
    x2_ = x1_;
    x1_ = clamped_x0;

    y2_ = y1_;
    y1_ = y0;

    return y0;
}

VectorXd LowPass2ndFilter::getPositionTarget(){
    return last_input_;
}

void LowPass2ndFilter::calculateTimeConstant(float fc, float fs){
    double omega = 2.0 * M_PI * fc / fs;
    double alpha = std::sin(omega) / std::sqrt(2.0); // Q factor = 0.707
    double cos_w = std::cos(omega);
    double a0 = 1.0 + alpha;
    
    // Calculate b coefficients (numerator)
    b0_ = ((1.0 - cos_w) / 2.0) / a0;
    b1_ = (1.0 - cos_w) / a0;
    b2_ = ((1.0 - cos_w) / 2.0) / a0;
    
    // Calculate a coefficients (denominator)
    a1_ = (-2.0 * cos_w) / a0;
    a2_ = (1.0 - alpha) / a0;
}