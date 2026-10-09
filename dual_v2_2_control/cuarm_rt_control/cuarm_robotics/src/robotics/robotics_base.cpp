#include "robotics_base.h"

#include "iostream"
#include "robotics/robotics_library/pinocchio_wrapper.h"

// using namespace planner;
bool RoboticsBase::anyLarger(const VectorXd& v, float x){
    for (int i =0; i<v.size(); i++){
        if (v[i] > x) return true; 
    }
    return false;
}

bool RoboticsBase::anyLarger(const VectorXd& v1, const VectorXd& v2, const VectorXi& cmp_ji){
    for (int i =0; i<v1.size(); i++){
        if (cmp_ji[i] == -1) continue; //Not the target for comparison
        if ((abs(v1[i]) > abs(v2[i]) && Sgn(v1[i]) == Sgn(v2[i])) || (Sgn(v1[i]) != Sgn(v2[i]) && v1[i] > v2[i])) return true; 
    }
    return false;
}

bool RoboticsBase::anySameSign(const VectorXd& v1, const VectorXd& v2, const VectorXi& cmp_ji){
    for (int i =0; i<v1.size(); i++){
        if (cmp_ji[i] == -1) continue; //Not the target for comparison
        if (Sgn(v1[i]) == Sgn(v2[i])) return true; 
    }
    return false;
}

RoboticsBase::RoboticsBase(const std::string& library_name, const std::string& urdf_file, const std::vector<std::string>& joint_names, 
    const std::string& ee_link_name, const Vector3d& tool_offset) : tool_offset_(tool_offset){
    if (library_name == "pinocchio"){
        lib_ = new PinocchioWrapper(urdf_file, joint_names, ee_link_name, tool_offset);
    }else{
        std::cout <<"Error: Input the robotics library name.\n";
    }
    joint_size_ = lib_->joint_size_;
    link_lengths_ = lib_->link_lengths_;
    joint_revolute_ = lib_->joint_revolute_;
    joint_lower_limit_ = lib_->getJointLowerLimit();
    joint_upper_limit_ = lib_->getJointUpperLimit();
    joint_limit_mid_.resize(joint_size_);
    for (int j = 0; j < joint_size_; ++j) {
        joint_limit_mid_[j] = (joint_lower_limit_[j] + joint_upper_limit_[j]) / 2.0;
    }
}

RoboticsBase::~RoboticsBase(){
    delete lib_;
    lib_ = nullptr;
}

MatrixX7d RoboticsBase::FK(const VectorXd& joint_pos, Vector7d* ee_pose, Vector7d* tool_pose) const {
    return lib_->FK(joint_pos, ee_pose, tool_pose);
}

Matrix4d RoboticsBase::FK(const VectorXd& joint_pos, int joint_id) const{
    return lib_->FK(joint_pos, joint_id);
}

bool RoboticsBase::IK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const {
    return lib_->IK(R, P, res);
}

VectorXd RoboticsBase::ID(const VectorXd& joint_pos, const VectorXd& joint_vel, const VectorXd& joint_acc) const {
    return lib_->ID(joint_pos, joint_vel, joint_acc);
}

/** Calculate inverse kinematics with Quick Inverse Kinematics (i.e. third-order Jacobian-based).
 * 
 * \param R     target 3x3 orientation matrix from base to tool frame.
 * \param P     target 3x1 translation vector from base to tool frame.
 * \param res   input current joint position, return the calculated joint postion.
 * \returns     state of IK
 */
bool RoboticsBase::QuIK(const Matrix3d& R, const Vector3d& P, VectorXd& res) const {
    const int last_joint_id = joint_size_-1;
    const double eps  = 2e-4;
    const int IT_MAX  = 200;
    const double DT   = 3e-1;
    const double damp = 1e-6;

    int i=0;
    VectorXd q = res;
    bool success = false;
    Vector6d err;
    Matrix3d R1;
    Vector3d P1;
    VectorXd v(joint_size_);
    Matrix6Xd J(6, joint_size_), A(6, joint_size_);
    J.setZero();
    Matrix66 JJt, AAt;
    Matrix4d T;
    while (i < IT_MAX ){
        T = FK(q, last_joint_id);
        R1 = T.block<3,3>(0,0);
        P1 = T.block<3,1>(0,3);
        err = lib_->calPoseDifference(R, P, R1, P1);
        if(err.norm() < eps){
            success = true;
            break;
        }
        if (anyLarger(q-res, static_cast<float>(M_PI/10))) break; //Prevent large jump
        
        //Perform newton step and store the halved value in v.
        J = getJointJacobian(q, last_joint_id);
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = J.transpose() * JJt.ldlt().solve(err);
        v *= -0.5;

        //Get gradient product
        A = J;
        hessianProduct(J, v, A);

        //Resolve
        AAt.noalias() = A * A.transpose();
        AAt.diagonal().array() += damp;
        v.noalias() = A.transpose() * AAt.ldlt().solve(err);
        v *= -1;
        
        q = q + v;
        i++;
    }

    if (success) res = q;
    return success;    
}

/**
 *  Computes the product H*dQ, where H is the geometric hessian of the robot. This
 *	function never computes H explicitely, and instead just reduces the computed values
 * 	through multiplication as they are computed. The result is added to A.
 *	Note, A must be pre-initialized with 0's before calling this function!
 *
 * @param J The Jacobian of the manipulator in a given pose
 * @param dQ The vector dQ
 * @param A The matrix A to store the hessian product
 */
void RoboticsBase::hessianProduct( const Matrix6Xd& J, const VectorXd& dQ, Matrix6Xd& A) const {
	int dof = joint_size_;
	Vector3d cp, jvk, jwk, Aw_k_sum;
	if (joint_revolute_.all()){
		// Special code for revolute joint robots only (most common, can
		// skip branched coding which is
		
		for (int k = 0; k < dof; k++){
			// First, iterate over off-diagonal terms
			jvk = J.template block<3,1>(0,k);
			jwk = J.template block<3,1>(3,k);
            
            // Rotational terms can be simplified by avoiding half the cross products
			// Initiate summer, then sum in loop before doing cross product.
            Aw_k_sum << 0,0,0;
            
			for (int i = 0; i < k; i++){
				// A(4:6, k) += jwi x jwk * dQi
				//           += (jwi * dQi) x jwk
				// Can sum the first term in cross product first before computing cross product.
                Aw_k_sum += J.template block<3,1>(3,i) * dQ(i);
                
				// A(1:3, k) += jwi x jvk*dQi
				cp = J.template block<3,1>(3, i).cross( jvk );
				A.template block<3,1>(0, k) += cp * dQ(i);
				A.template block<3,1>(0, i) += cp * dQ(k); // Symmetry
			}
            
            // Do final cross product for rotational term
            A.template block<3,1>(3, k) += Aw_k_sum.cross( jwk );
			
			// For diagonal entries, can skip the omega term since jwk x jwk = 0
			// A(4:6, k) += jwi x jwk * dQi = 0
			//
			// A(1:3, k) += jwi x jvk*dQi
			A.template block<3,1>(0, k) += jwk.cross( jvk ) * dQ(k);
		}
		
	}else{
		// General case
		
		for (int k = 0; k < dof; k++){
			// First, iterate over off-diagonal terms
			jvk = J.template block<3,1>(0,k);
			jwk = J.template block<3,1>(3,k);
			
			for (int i = 0; i < k; i++){
				if (joint_revolute_[i]){ // link i is revolute
					if (joint_revolute_[k]){	// link k is revolute
						// A(4:6, k) += jwi x jwk * dQi
						A.template block<3,1>(3, k) += (J.template block<3,1>(3, i)).cross( jwk ) * dQ(i);
					}
					
					// A(1:3, k) += jwi x jvk*dQi
					cp = J.template block<3,1>(3, i).cross( jvk );
					A.template block<3,1>(0, k) += cp * dQ(i);
					A.template block<3,1>(0, i) += cp * dQ(k); // Symmetry
				}
				
				if (joint_revolute_[k]){
					// For diagonal entries, can skip the omega term since jwk x jwk = 0
					// A(4:6, k) += jwi x jwk * dQi = 0
					//
					// A(1:3, k) += jwi x jvk*dQi
					A.template block<3,1>(0, k) += jwk.cross( jvk ) * dQ(k);
				}
			} // end of inner for loop
		} // end of outer for loop
	} // end of branched if
}

VectorXd RoboticsBase::IDWithExternalForces(const VectorXd& joint_pos,const VectorXd&  joint_vel,const VectorXd& joint_acc) const {
    return lib_->IDWithExternalForces(joint_pos, joint_vel, joint_acc);
}

Matrix6Xd RoboticsBase::getJointJacobian(const VectorXd& joint_pos, int joint_id) const {
    return lib_->getJointJacobian(joint_pos, joint_id);
}

void RoboticsBase::getToolPose(Vector3d* pos, Eigen::Quaterniond* rot) const {
    return lib_->getToolPose(pos, rot);
}

Vector3d RoboticsBase::getToolOffset() const{
    return tool_offset_;
}

void RoboticsBase::setToolOffset(const Vector3d& tool_offset){
    lib_->updateToolFrame(tool_offset);
    tool_offset_ = tool_offset;
}

void RoboticsBase::setExternalForce(const Vector3d& force_linear, const Vector3d& force_angular){
    return lib_->setExternalForce(force_linear, force_angular);
}

Vector3d RoboticsBase::toEulerAngle(const Matrix3d& R){
    return R.eulerAngles(0, 1, 2); 
}

Vector3d RoboticsBase::toEulerAngle(const Eigen::Quaterniond& quat){
    return toEulerAngle(quat.toRotationMatrix());
}

Eigen::Quaterniond RoboticsBase::toQuat(const Vector3d& RPY){
    return Eigen::AngleAxisd(RPY[0], Vector3d::UnitX())
            * Eigen::AngleAxisd(RPY[1], Vector3d::UnitY())
            * Eigen::AngleAxisd(RPY[2], Vector3d::UnitZ());
}

Eigen::Quaterniond RoboticsBase::toQuat(const Matrix3d& R){
    Eigen::Quaterniond q(R);
    q.normalize();
    return q;
}

Matrix3d RoboticsBase::toR(const Vector3d& RPY){
    return toQuat(RPY).toRotationMatrix();
}

Matrix3d RoboticsBase::toR(const Vector4d& quat){
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);    
    q.normalize(); 
    return q.toRotationMatrix();
}

Matrix3d RoboticsBase::MDH_R(double alpha, double theta){
    Matrix3d Rx, Rz;
    Rx <<   1,           0,           0,
            0, +cos(alpha), -sin(alpha),
            0, +sin(alpha), +cos(alpha);
    Rz <<   +cos(theta), -sin(theta), 0,
            +sin(theta), +cos(theta), 0,
                      0,           0, 1;
    return Rx*Rz;
}

Matrix4d RoboticsBase::MDH(double a, double alpha, double d, double theta){
    Matrix4d A1, A2;
    A1 <<   1,           0,           0,    a,
            0, +cos(alpha), -sin(alpha),    0,
            0, +sin(alpha), +cos(alpha),    0,
            0,           0,           0,    1;
    A2 <<   +cos(theta), -sin(theta), 0,    0,
            +sin(theta), +cos(theta), 0,    0,
                      0,           0, 1,    d,
                      0,           0, 0,    1;
    return A1*A2;
}


Vector7d RoboticsBase::eeToTool(const Vector7d& ee_pose, const Vector3d& tool_offset){
    Vector3d p_ee = ee_pose.head<3>();
    Vector4d quat = ee_pose.tail<4>();
    Matrix3d R = toR(quat);
    Vector3d p_tool = p_ee + (R * tool_offset);
    Vector7d result;
    result << p_tool(0), p_tool(1), p_tool(2), quat(0), quat(1), quat(2), quat(3);
    return result;
}

Vector7d RoboticsBase::toolToEE(const Vector7d& tool_pose, const Vector3d& tool_offset){
    Eigen::Vector3d p_tool = tool_pose.head<3>();
    Vector4d quat = tool_pose.tail<4>();
    Matrix3d R = toR(quat);
    Vector3d p_ee = p_tool - (R * tool_offset);
    Vector7d result;
    result << p_ee(0), p_ee(1), p_ee(2), quat(0), quat(1), quat(2), quat(3);
    return result;
}