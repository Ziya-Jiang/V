#include "pos_controller.h"
namespace PController {

    CosInterpolation::CosInterpolation(double error_tol,
                                       double dt,
                                       int T)
    : error_tol(error_tol),
      dt(dt), 
      T(T)
    {
        reset();
    }

// 更新控制器状态并计算输出
    double CosInterpolation::update(
        double target_position, 
        double current_position,
        double kp_value,
        double output_limit) 
    {
        // 计算总误差
        total_error = target_position - current_position;
        std::cout << "Total Error: " << total_error << "|";
        std::cout << "Current Time: " << current_t << "s|"<< std::endl;
        if (current_t <= T) {
            velocity_factor = 0.5*(1.0 - std::cos(2*M_PI * (current_t) / T));
            current_error = total_error * velocity_factor;
        }
        else {
            // 直接使用总误差
            if (linear_step>0)
            {
                current_error = remaining_error / linear_step;
        
        // 更新状态
                remaining_error -= current_error;  // 减少剩余误差
                linear_step--;
            }else{
                current_error = total_error;
            }             // 减少剩余步数
        }
        current_t+=dt;
        // 更新收敛计数
        if (std::fabs(total_error) <= error_tol) {  // 使用较小的收敛阈值
            converge_flag = 1;
            std::cout << "Converged! Total Error: " << total_error << std::endl;
        } else {
            converge_flag = 0;
        }
        // std::cout << "Current Error: " << current_error << std::endl;
        // 计算并返回控制输出
        return computePIDOutput(kp_value, current_error, output_limit);
    }
    // 检查是否收敛
    int CosInterpolation::isConverged()
    {
        return converge_flag;
    }

    // 重置状态
    void CosInterpolation::reset() {
        current_t = 0.0;
        converge_flag = 0;
        current_error = 0.0;
    }

    // 基本PID计算函数
    double computePIDOutput(double kp, double error, double pid_limits) {
        double output = kp * error;
        
        // 应用输出限制
        if (output > pid_limits) {
            return pid_limits;
        } else if (output < -pid_limits) {
            return -pid_limits;
        }
        return output;
    }

}