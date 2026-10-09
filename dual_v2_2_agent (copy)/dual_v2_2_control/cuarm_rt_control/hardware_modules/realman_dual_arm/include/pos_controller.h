#pragma once
#include <cmath>

#include <iostream>

namespace PController {

// 单关节插值控制器
class CosInterpolation {
public:
    // 构造函数
    CosInterpolation(double error_tol=0.05,
                     double dt=0.02, int T=10);
    
    // 更新状态并计算控制输出
    double update(double target_position, 
                 double current_position,
                 double kp_value,
                 double output_limit);
    
    // 检查是否收敛
    int isConverged();
    
    // 重置状态
    void reset();
    
    // 获取当前误差分量
    double current_t;
    double getCurrentError() const { return current_error; }
    // 获取收敛计数

private:
    double dt;
    int T;
    double current_error = 0.0;
    int converge_flag = 0;
    double error_tol;
    double velocity_factor;
    double x0 = 0.0;
    double remaining_error = 0.0;
    int linear_step = 500;
    double total_error;
};
// 基本PID计算函数
double computePIDOutput(double kp, double error, double pid_limits);

} // namespace PController