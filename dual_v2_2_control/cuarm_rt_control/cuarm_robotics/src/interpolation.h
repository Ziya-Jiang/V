#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>
#include <memory>
#include <tuple>

#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"

class AbstractInterpolator{
    public:
        AbstractInterpolator(float T): T_(T){};
        virtual VectorXd getPos(float t) const = 0;
        virtual VectorXd getVel(float t) const = 0;
        virtual void getPositionTarget(float t, VectorXd& x1) const = 0;
        float getT() const { return T_; };
    protected:
        float T_;
};

class AbstractPointInterpolator: public AbstractInterpolator{
    public:
        AbstractPointInterpolator(float T): AbstractInterpolator(T){};
};

class AbstractPathInterpolator: public AbstractInterpolator{
    public:
        AbstractPathInterpolator(float T): AbstractInterpolator(T){};
        virtual int getPositionTargetIndex(float t) const = 0;
};

class LinearInterpolator: public AbstractPointInterpolator{
    public:
        LinearInterpolator(float T, const VectorXd& x0, const VectorXd& x1);
        VectorXd getPos(float t) const override;
        VectorXd getVel(float t) const override;
        void getPositionTarget(float t, VectorXd& x1) const override;
    protected:
        VectorXd x0_;
        VectorXd x1_;
};

class CosInterpolator: public AbstractPointInterpolator{
    public:
        CosInterpolator(float T, const VectorXd& x0, const VectorXd& x1);
        VectorXd getPos(float t) const override;
        VectorXd getVel(float t) const override;
        void getPositionTarget(float t, VectorXd& x1) const override;
    protected:
        VectorXd x0_;
        VectorXd x1_;
};

class CubicInterpolator: public AbstractPointInterpolator{
    public:
        CubicInterpolator(float T, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1);
        VectorXd getPos(float t) const override;
        VectorXd getVel(float t) const override;
        void getPositionTarget(float t, VectorXd& x1) const override;
    protected:
        VectorXd x0_, x1_, v0_, v1_;
        VectorXd A_, B_, C_, D_;
};

class QuinticInterpolator: public AbstractPointInterpolator{
    public:
        QuinticInterpolator(float T, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1, const VectorXd& a0, const VectorXd& a1);
        QuinticInterpolator(float T, const std::vector<VectorXd>& coefficients);
        VectorXd getPos(float t) const override;
        VectorXd getVel(float t) const override;
        VectorXd getAcc(float t) const;
        void getPositionTarget(float t, VectorXd& x1) const override;
    protected:
        VectorXd x0_, x1_, v0_, v1_, a0_, a1_;
        VectorXd A_, B_, C_, D_, E_, F_;
};

class QuinticPathInterpolator: public AbstractPathInterpolator{
    public:
        QuinticPathInterpolator(const VectorXd& x0, const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time);
        VectorXd getPos(float t) const override;
        VectorXd getVel(float t) const override;
        void getPositionTarget(float t, VectorXd& x1) const override;
        int getPositionTargetIndex(float t) const override;
    protected:
        std::vector<std::vector<VectorXd>> evaluateCoefficients(const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& pos, const std::vector<float>& waypoint_time);
        std::tuple<int, float> getSegmentIndexAndTime(float t) const;
        std::vector<std::unique_ptr<QuinticInterpolator>> segment_interpolators_;
        VectorXd x0_, v0_, a0_;
        std::vector<VectorXd> waypoint_position_;
        std::vector<float> waypoint_time_;
};

class LowPassFilter{
    public: 
        LowPassFilter(const VectorXd& x0, float cutoff_frequency, float output_frequency);
        VectorXd getPos(VectorXd x0);
        VectorXd getPositionTarget();
    protected:
        VectorXd last_output_;
        VectorXd last_input_;
        float alpha_;
        float calculateTimeConstant(float cutoff_frequency, float output_frequency);
};

class LowPass2ndFilter{
    public: 
        LowPass2ndFilter(const VectorXd& x0, float cutoff_frequency, float output_frequency, const VectorXd& max_velocity_limits, float T);
        VectorXd getPos(VectorXd x0);
        VectorXd getPositionTarget();
    protected:
        // Filter coefficients
        double b0_, b1_, b2_, a1_, a2_;
        // History buffers
        VectorXd x1_, x2_, y1_, y2_, last_input_, last_clamped_input_;

        double dt_;
        VectorXd max_input_step_;
        float reach_target_time_;
        float remaining_target_time_;
        void calculateTimeConstant(float cutoff_frequency, float output_frequency);
};

class InterpolatorFactory{
    public:
        InterpolatorFactory();
        InterpolatorFactory(InterpolationMethod m, float dt, int joint_size, const std::string& name="");
        void updateParam(InterpolationMethod m, float acc_duration, float vel_duration, const VectorXd& leading_joint_max_vel = VectorXd());
        bool resetInterpolator(InterpolationMethod m, float T, VectorXd x0, VectorXd x1, VectorXd v0=VectorXd(), VectorXd v1=VectorXd(), VectorXd a0=VectorXd(), VectorXd a1=VectorXd());
        bool resetInterpolator(InterpolationMethod m, VectorXd x0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time, VectorXd v0=VectorXd(), VectorXd a0=VectorXd());
        VectorXd getNextPos(VectorXd x0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time, std::vector<VectorXd> leading_joint_max_vel=std::vector<VectorXd>(), VectorXd v0=VectorXd(), VectorXd a0=VectorXd());
        VectorXd getNextPos(VectorXd x0, VectorXd x1, VectorXd v0=VectorXd(), VectorXd v1=VectorXd(), VectorXd a0=VectorXd(), VectorXd a1=VectorXd());
        void interrupt();
        bool isInterrupted() const;
        bool isCompleted(float t);
        InterpolationMotionPhase getState() const;
        InterpolationMethod getInterpolationMethod() const;
        int getInterpolatorIndex() const;
        void reset();
        void getPositionTarget(VectorXd& x1) const;
        int getPositionTargetIndex() const;
        uint32_t getWarningFlags() const;
        float getInterpolationT() const;
        VectorXd getPosAtTime(float t) const;
    protected:
        VectorXd getPos(AbstractInterpolator* interpolator) const;
        bool isCurrentTarget(const VectorXd& x1, const VectorXd& v1, const VectorXd& a1) const;
        bool isCurrentTarget(std::vector<VectorXd> waypoint_position , const std::vector<float>& waypoint_time, std::vector<VectorXd> leading_joint_max_vel) const;
        void replaceEmptyVector(VectorXd& v);
        float getMinAllowedTime(InterpolationMethod m, double x0, double x1, double v0, double v1, double max_vel) const;

        // Single point duration resolution
        int getLeadingJointIndex(const VectorXd& x0, const VectorXd& x1) const;
        float getLeadingJointPeakVel(InterpolationMethod m, float T, int leading_joint_index, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1, const VectorXd& a0, const VectorXd& a1) const;
        float getLeadingJointMaxVelValue(int leading_joint_index) const;
        bool isSinglePointDurationValid(InterpolationMethod m, float user_T, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1) const;
        float resolveSinglePointDuration(InterpolationMethod m, const VectorXd& x0, const VectorXd& x1, const VectorXd& v0, const VectorXd& v1, const VectorXd& a0, const VectorXd& a1) const;
        
        // Fluid waypoint duration resolution
        void filterWaypoints(std::vector<VectorXd>& waypoint_pos, std::vector<float>& waypoint_time, std::vector<VectorXd>& leading_joint_max_vel) const;
        bool isFluidWaypointDurationValid(InterpolationMethod m, const VectorXd& x0, const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time, std::vector<VectorXd> leading_joint_max_vel) const;
        std::vector<VectorXd> estimateFluidWaypointVelocities(InterpolationMethod m, const std::vector<VectorXd>& pos, const VectorXd& v0, const std::vector<VectorXd>& max_vel) const;
        std::vector<float> evaluateDtFromLeadingJointMaxVelContinuous(InterpolationMethod m, const std::vector<VectorXd>& pos, const std::vector<VectorXd>& vel, const std::vector<VectorXd>& leading_joint_max_vel) const;
        std::vector<float> resolveFluidWaypointDuration(InterpolationMethod m, const VectorXd& x0, const VectorXd& v0, const VectorXd& a0, const std::vector<VectorXd>& waypoint_position, const std::vector<float>& waypoint_time, std::vector<VectorXd> leading_joint_max_vel) const;

        bool is_waypoint_interpolator_ = false;
        float dt_;
        std::string name_;
        InterpolationMethod interpolation_method_;
        std::unique_ptr<AbstractPointInterpolator> interpolator_ptr_;
        std::unique_ptr<AbstractPathInterpolator> path_interpolator_ptr_;
        std::unique_ptr<LowPass2ndFilter> filter_;
        InterpolationMotionPhase state_ = InterpolationMotionPhase::kInitialized;
        AbstractInterpolator* abstract_interpolator_ptr_ = nullptr;
        static constexpr float kDefaultAccT = 5.0f;
        int i_ = 0;
        float acc_t_ = kDefaultAccT;
        float vel_t_ = 0.0f;
        VectorXd leading_joint_max_vel_;
        int joint_size_;
        mutable uint32_t warning_flags_ = DiagnosticFlags::kNone;

        // current single point information
        VectorXd current_x1_;
        VectorXd current_v1_;
        VectorXd current_a1_;

        // current waypoint information
        std::vector<VectorXd> current_waypoint_position_;
        std::vector<float>    current_waypoint_time_;
        std::vector<VectorXd> current_waypoint_leading_joint_max_vel_;
};

#endif