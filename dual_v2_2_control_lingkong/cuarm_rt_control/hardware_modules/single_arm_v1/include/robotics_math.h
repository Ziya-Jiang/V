/*
 * @Author:
 * @Date: 
 * @LastEditors:
 * @LastEditTime: 
 * @FilePath: /ros2_demo/src/gac_arm_cpp/src/
 * @Description: 
 * @Description: 错误码区间100-199
 */

 #pragma once
 #ifndef ROBOTICS_MATH_
 #define ROBOTICS_MATH_
 
 #define ZERO_VALUE (1e-6)
 
 #include "robot_struct.h"
 #include "Matrix.hpp"
 #include "SquareMatrix.hpp"
 
 namespace robotics_math
 {
	 /*
	  * 求解齐次变换矩阵
	  */
	 void getTransformationMatrixMDH(double theta, double alpha, double a, double d, matrix::SquareMatrix<double, 4>& T);
	 void getTransformationMatrixMDHdot(double theta, double alpha, double a, double d, matrix::SquareMatrix<double, 4>& T);
	 /*
	  * 求解旋转矩阵
	  */
	 void getRotationMatrixMDH(double theta, double alpha, matrix::SquareMatrix<double, 3>& R);
 
	 /*
	  * 通用雅可比计算函数
	  */
	 int getJacobianGeneral(const double* q_in,const double* alpha, const double* a,const double* d, const int dim, double Jacob[6][num_joints_]);
 
	 /*
	  * 笛卡尔空间位置转变换矩阵
	  */
	 void Eula2Mat(matrix::Matrix<double, 4, 4>& mat, const double* P);
 
	 /*
	  * 笛卡尔空间位置转变换矩阵
	  */
	 void Eula2Mat(matrix::Matrix<double, 4, 4>& mat, const matrix::Matrix<double, 1, 6>& P);
 
	 /*
	  * 变换矩阵转笛卡尔空间位置
	  */
	 void Mat2Eula(const matrix::Matrix<double, 4, 4>& mat, double* P);
 
	 /*
	  * 变换矩阵转笛卡尔空间位置
	  */
	 void Mat2Eula(const matrix::Matrix<double, 4, 4>& mat, matrix::Matrix<double, 1, 6>& P);
 
	 /*
	  * 笛卡尔空间位置转四元数
	  */
	 void Eul2Quat(double rz, double ry, double rx, double(&quat)[4]);
	 void Rot2Quat(const matrix::Matrix<double, 3, 3>& mat, double(&quat)[4]);
	 /*
	  * 四元数转笛卡尔空间位置
	  */
	 void Quat2Eul(const double(&q)[4], double& rz, double& ry, double& rx);
 
	 /*
	  * 四元数转变换矩阵
	  */
	 void Quat2Mat(const double(&q)[4], matrix::Matrix<double, 4, 4>& mat);
 
	 void Quat2Rot(const double(&q)[4], matrix::Matrix<double, 3, 3>& mat);
 
	 void slerp(const double(&q1)[4], const double(&q2)[4], const double& q, double(&quat)[4]);
 
	 void slerpInsert(const double(&p)[4], const double(&q)[4], const double& real_dis, double(&quat)[4]);
 
	 /*
	  * 由变换矩阵提取旋转矩阵
	  */
	 matrix::SquareMatrix<double, 3> frameToRotation(const matrix::SquareMatrix<double, 4>& frame_in);
 
	 /*
	  * 由变换矩阵提取笛卡尔空间位置向量
	  */
	 matrix::Vector<double, 3> frameToPosition(const matrix::SquareMatrix<double, 4>& frame_in);
 
	 matrix::SquareMatrix<double, 4> rotAndPosToFrame(const matrix::SquareMatrix<double, 3>& rot_in, const matrix::Vector<double, 3>& pos_in);
 
	 /*
	  * 向量叉乘
	  */
	 matrix::Vector<double, 3> cross(const matrix::Vector<double, 3>& A, const matrix::Vector<double, 3>& B);
 
	 void arrayCross(const double* p1, const double* p2, double* result);
 
	 void InvQuat(const double(&q)[4], double(&InvQuat)[4]);
 
	 void GetQuaAxis(const double(&quat)[4], double(&QuaAxis)[4]);
 
	 void MultiQuat(const double(&q)[4], const double(&r)[4], double(&result)[4]);
 

 

	 void mat4Inv(const matrix::Matrix<double, 4, 4>& src_T, matrix::Matrix<double, 4, 4>& des_T);
 
	 void addTcp(const matrix::Matrix<double, 4, 4>& src_matrix, matrix::Matrix<double, 4, 4>& des_matrix, const double* tcp);
 
	 void removeTcp(const matrix::Matrix<double, 4, 4>& src_matrix, matrix::Matrix<double, 4, 4>& des_matrix, const double* tcp);
 }
 #endif