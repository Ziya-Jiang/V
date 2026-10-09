#ifndef QUINTIC_TRAJ_H
#define QUINTIC_TRAJ_H

class QuinticTraj {
public:
    QuinticTraj(int num_joints);

    // 析构函数，释放动态分配的内存
    ~QuinticTraj();

    // 生成五次多项式轨迹，q0 默认为 0，v0, vf, a0, af 也默认为 0
    void generate_trajectory(const double* qf, double T);

    // 计算给定时间 t 的位置
    void calculate_position(double t, double* positions) const;
    void calculate_velocity(double t, double* velocity) const;

private:
    // 五次多项式结构体
    struct QuinticPolynomial {
        double a0, a1, a2, a3, a4, a5;
    };

    int num_joints_;               // 关节数量
    QuinticPolynomial* polynomials_; // 多项式系数数组

    // 计算给定时间 t 的位置
    double calculate_position(const QuinticPolynomial& poly, double t) const;
    double calculate_velocity(const QuinticPolynomial& poly, double t) const;
    
    // 生成五次多项式的系数，q0 默认为 0，v0, vf, a0, af 也默认为 0
    QuinticPolynomial generate_quintic_polynomial(double qf, double T) const;
};

#endif // QuinticTraj_H