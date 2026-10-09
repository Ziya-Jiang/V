#ifndef ROBOTICS_BASE_H
#define ROBOTICS_BASE_H

#include <string>
#include "robot_platform_utils/cpp/include/eigen_vector.h"
#include "robotics/robotics_library/robotics_library.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846  /* pi */
#endif

class RoboticsBase{
    public:
        RoboticsBase(const std::string& library_name, const std::string& urdf_file, const std::vector<std::string>& joint_names,
            const std::string& ee_link_name, const Vector3d& tool_offset);
        virtual ~RoboticsBase();
        MatrixX7d FK(const VectorXd& joint_pos, Vector7d* ee_pose=nullptr, Vector7d* tool_pose=nullptr) const;
        Matrix4d FK(const VectorXd& joint_pos, int joint_id) const;
        bool IK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const;
        bool QuIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const ;
        void hessianProduct(const Matrix6Xd& J, const VectorXd& dQ, Matrix6Xd& A) const ;
        VectorXd ID(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc) const;
        VectorXd IDWithExternalForces(const VectorXd& joint_pos,const VectorXd&  joint_vel,const VectorXd& joint_acc) const;
        Matrix6Xd getJointJacobian(const VectorXd& joint_pos, int joint_id) const;
        void getToolPose(Vector3d* pos, Eigen::Quaterniond* rot) const;
        Vector3d getToolOffset() const;
        void setToolOffset(const Vector3d& tool_offset);
        
        //Setters
        void setExternalForce(const Vector3d& force_linear, const Vector3d& force_angular);

        //Static functions
        static Vector3d toEulerAngle(const Matrix3d& R);
        static Vector3d toEulerAngle(const Eigen::Quaterniond& quat);
        static Eigen::Quaterniond toQuat(const Vector3d& RPY);
        static Eigen::Quaterniond toQuat(const Matrix3d& R);
        static Matrix3d toR(const Vector3d& RPY);
        static Matrix3d toR(const Vector4d& quat);
        static Matrix3d MDH_R(double alpha, double theta);
        static Matrix4d MDH(double a, double alpha, double d, double theta);
        static Vector7d eeToTool(const Vector7d& ee_pose, const Vector3d& tool_offset);
        static Vector7d toolToEE(const Vector7d& tool_pose, const Vector3d& tool_offset);

        template <typename T> 
        static int  Sgn(T val){return (T(0) < val) - (val < T(0));};
        static bool anyLarger(const VectorXd& v, float x);
        static bool anyLarger(const VectorXd& v1, const VectorXd& v2, const VectorXi& cmp_ji);
        static bool anySameSign(const VectorXd& v1, const VectorXd& v2, const VectorXi& cmp_ji);

    protected:
        RoboticsLibrary* lib_ = nullptr;
        int joint_size_;
        Eigen::ArrayXf link_lengths_;
        Eigen::Array<bool, Eigen::Dynamic, 1> joint_revolute_; // true if it is a revolute joint;
        Vector3d tool_offset_;
        VectorXd joint_lower_limit_;
        VectorXd joint_upper_limit_;
        VectorXd joint_limit_mid_;
};

#endif