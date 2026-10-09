#ifndef EIGEN_VECTOR_H
#define EIGEN_VECTOR_H
#include <Eigen/Dense> // Must be before robotics library includes so Eigen plugin load correctly
#include <Eigen/Geometry>

// typedef Eigen::Array<double,7,1> Array7;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<float, 7, 1>  Vector7f;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<float, 6, 1>  Vector6f;
typedef Eigen::Matrix<double, 6, 6> Matrix66;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic>  Matrix2Xf;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;
typedef Eigen::Matrix<float, Eigen::Dynamic, 3>  MatrixX3f;
typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixX6d;
typedef Eigen::Matrix<float, Eigen::Dynamic, 6>  MatrixX6f;
typedef Eigen::Matrix<double, Eigen::Dynamic, 7> MatrixX7d;
typedef Eigen::Matrix<float, Eigen::Dynamic, 7>  MatrixX7f;
typedef Eigen::Array2d Array2d;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector4d Vector4d;
typedef Eigen::Vector4f Vector4f;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::VectorXf VectorXf;
typedef Eigen::VectorXi VectorXi;
typedef Eigen::Matrix2d Matrix2d;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Matrix4f Matrix4f;
typedef Eigen::MatrixXd MatrixXd;
#endif