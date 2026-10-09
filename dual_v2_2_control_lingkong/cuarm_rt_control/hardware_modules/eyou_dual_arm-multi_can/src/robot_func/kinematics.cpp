#define ROBOT_GLOBAL  // 必须在 #include 之前！

#include "kinematics.h"
// Robot g_robot[ROBOT_NUM];
// in world


int kinematics::forwardKinematics(const int &robot_index, const double q_in[AXISNUM], double cartesian_position[6])
{
    double q[AXISNUM];

    for (int i = 0; i < AXISNUM; i++)
    {
        // if ((q_in[i] > g_robot[robot_index].m_joint_limit.joint_positive_limit[i]) || (q_in[i] < g_robot[robot_index].m_joint_limit.joint_negative_limit[i]))
        //     return 1; // 输入关节角超限位

        q[i] = q_in[i] + g_robot[robot_index].m_mdh.offset[i];
    }

    matrix::Matrix<double, 4, 4> T;
    T.identity();

    matrix::SquareMatrix<double, 4> Ti;
    for (int i = 0; i < AXISNUM; i++)
    {
        robotics_math::getTransformationMatrixMDH(q[i], g_robot[robot_index].m_mdh.alpha[i], g_robot[robot_index].m_mdh.a[i], g_robot[robot_index].m_mdh.d[i], Ti);
        T = T * Ti;
    }
    double cartesian_base_position[6] = {0, 0, 0, 0, 0, 0};
    robotics_math::Mat2Eula(T, cartesian_base_position);
    // for (int j = 0; j < 6; j++)
    // {
    //     cartesian_position[j] = cartesian_base_position[j];
    // }
    baseToWorld(robot_index,cartesian_position, cartesian_base_position);
    return 0;
}
// in world

int kinematics::inverseKinematicsSingularityRobust(const int &robot_index, const double destination_cartesian[6], const double tolerance, const int max_iterative_num, const double last_q_in[AXISNUM], double q_out[AXISNUM])
{
    int itr_num = 0;

    // 迭代解变量初始化
    matrix::Matrix<double, 6, 1> dxe;
    matrix::Matrix<double, 6, AXISNUM> jacobian;

    double lambda = 1; // 步长
    double sum = 0;
    double sum_dq[AXISNUM]{};

    matrix::Matrix<double, AXISNUM, 6> pinvJ;

    double q_in[7]{};
    // 将关节角赋值为当前关节角
    for (int i = 0; i < AXISNUM; i++)
    {
        q_in[i] = last_q_in[i] + g_robot[robot_index].m_mdh.offset[i];
        q_out[i] = q_in[i];
    }

    double destination_base_cartesian[6] = {0, 0, 0, 0, 0, 0};
    worldToBase(robot_index,destination_base_cartesian, destination_cartesian);
    // for (int j = 0; j < 7; j++)
    // {
    //     destination_base_cartesian[j]=destination_cartesian[j];
    // }

    // 第一次计算误差
    kinematics::calculateErrorJacobian(robot_index, q_out, destination_base_cartesian, dxe, jacobian);

    for (int i = 0; i < 6; i++)
        sum += dxe(i, 0) * dxe(i, 0);

    double norm_result = sqrt(sum);

    double m_0 = 100000;
    double sr_k = 0;

    // 计算可操作性度量
    matrix::SquareMatrix<double, 6> indictor = jacobian * jacobian.T();
    double manipulate_measure = sqrt(matrix::det(indictor));

    // 如果在奇异区域，则进入奇异鲁棒逆求解，否则不做任何处理，直接返回
    if (manipulate_measure < m_0)
    {
        sr_k = 1.5 * (1.0 - manipulate_measure / m_0) * (1.0 - manipulate_measure / m_0); // 计算阻尼因子调整方程 k

        auto I6 = matrix::eye<double, 6>();
        matrix::SquareMatrix<double, 6> temp = indictor + sr_k * I6;
        matrix::Matrix<double, AXISNUM, 6> J_sr_inv = jacobian.T() * matrix::inv(temp);

        matrix::Matrix<double, AXISNUM, 1> dq;
        dq = J_sr_inv * dxe;

        for (int i = 0; i < AXISNUM; i++)
        {
            q_out[i] = q_out[i] + lambda * dq(i, 0);
            sum_dq[i] = sum_dq[i] + lambda * dq(i, 0);
        }
    }
    else
    {
        while (norm_result > tolerance && itr_num < max_iterative_num)
        {
            // std::cout << "generalJ=======" << std::endl;
            // for (int i = 0; i < 6; i++)
            // {
            //     for (int j = 0; j < 7; j++)
            //     {
            //         std::cout << jacobian(i, j) << "  ";
            //     }
            //     std::cout << std::endl;
            // }

            pinvJ.setZero();

            // matrix::SquareMatrix<double, 6>  J_I(J * J.T());
            pinvJ = jacobian.T() * indictor.I();

            matrix::Matrix<double, AXISNUM, 1> dq;
            dq = pinvJ * dxe;

            for (int i = 0; i < AXISNUM; i++)
            {
                q_out[i] = q_out[i] + lambda * dq(i, 0);
                sum_dq[i] = sum_dq[i] + lambda * dq(i, 0);
            }

            kinematics::calculateErrorJacobian(robot_index, q_out, destination_base_cartesian, dxe, jacobian);

            indictor = jacobian * jacobian.T();
            manipulate_measure = sqrt(matrix::det(indictor));
            if (manipulate_measure < m_0)
            {
                break;
            }

            sum = 0;
            for (int i = 0; i < 6; i++)
            {
                sum += dxe(i, 0) * dxe(i, 0);
            }

            norm_result = sqrt(sum);
            itr_num = itr_num + 1;
        }
        printf("%s%d%s%d\n"," robot_index: ", robot_index, " itnum: ", itr_num);
        if (norm_result >= tolerance && itr_num >= max_iterative_num)
        {
            printf("%s\n", "!!!!!!!!!!!!超过迭代次数，无解!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
            return 4; // 超过迭代次数，无解
        }
    }

    for (int i = 0; i < AXISNUM; i++)
    {
        // if(abs(sum_dq[i])>0.026)
        // {
        //     printf("%s/n", "!!!!!!!!!!!!逆解迭代步长过长（会超速）!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
        //     return 7;  //逆解迭代步长过长（会超速）
        // }
        q_out[i] = q_out[i] - g_robot[robot_index].m_mdh.offset[i];
        if (q_out[i] > PI)
        {
            q_out[i] = q_out[i]- 2 * PI;
        }
        else if (q_out[i] < -PI)
        {
            q_out[i] = q_out[i]+ 2 * PI;
        }
        if (abs(q_out[i] - last_q_in[i]) < 0.000001)
        {
            q_out[i] = last_q_in[i];
        }
    }
    printf("%s,%f,%f,%f,%f,%f,%f,%f\n\n", "SRq_out ", q_out[0], q_out[1], q_out[2], q_out[3], q_out[4], q_out[5], q_out[6]);

    double next_cartesian_position[6]{};
    kinematics::forwardKinematics(robot_index, q_out, next_cartesian_position);

    if (fabs(next_cartesian_position[0] - destination_cartesian[0]) > 50 ||
        fabs(next_cartesian_position[1] - destination_cartesian[1]) > 50 ||
        fabs(next_cartesian_position[2] - destination_cartesian[2]) > 50)
    {
        printf("%s\n", "!!!!!!!!!!!!位置误差过大，无解!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
        return 5; // 奇异规避失败：位置误差过大
    }

    // if (fabs(next_cartesian_position[3] - destination_base_cartesian[3]) > 5 * DEGREE2RAD ||
    //     fabs(next_cartesian_position[4] - destination_base_cartesian[4]) > 5 * DEGREE2RAD ||
    //     fabs(next_cartesian_position[5] - destination_base_cartesian[5]) > 5 * DEGREE2RAD)
    // {
    //     printf("%s\n", "!!!!!!!!!!!!姿态误差过大，无解!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
    //     return 6; // 奇异规避失败：姿态误差过大
    // }

    return 0;
}

void kinematics::calculateErrorJacobian(const int &robot_index, const double *q, const double *desEular, matrix::Matrix<double, 6, 1> &dxe, matrix::Matrix<double, 6, AXISNUM> &jacobian_matrix)
{
    // 先求解出当前关节角下的齐次变换矩阵
    matrix::Matrix<double, 4, 4> T_cur;
    T_cur.identity();
    matrix::SquareMatrix<double, 4> Ti[AXISNUM];
    for (int i = 0; i < AXISNUM; i++)
    {
        robotics_math::getTransformationMatrixMDH(q[i], g_robot[robot_index].m_mdh.alpha[i], g_robot[robot_index].m_mdh.a[i], g_robot[robot_index].m_mdh.d[i], Ti[i]);
        T_cur = T_cur * Ti[i];
    }

    // 将该齐次变换矩阵转换为欧拉角
    double curEular[6];
    robotics_math::Mat2Eula(T_cur, curEular);

    // 求解位置误差
    for (int i = 0; i < 3; i++)
        dxe(i, 0) = desEular[i] - curEular[i];
    ////////求解姿态误差
    // 将欧拉角转四元数
    double qc[4], qd[4], invQc[4], deltaQ[4], QuatAxis[4]; // qc-当前	//qd--目标
    robotics_math::Eul2Quat(curEular[5], curEular[4], curEular[3], qc);
    robotics_math::Eul2Quat(desEular[5], desEular[4], desEular[3], qd);
    robotics_math::InvQuat(qc, invQc);
    robotics_math::MultiQuat(qd, invQc, deltaQ);
    robotics_math::GetQuaAxis(deltaQ, QuatAxis);
    // 姿态误差赋值
    dxe(3, 0) = QuatAxis[0] * QuatAxis[1];
    dxe(4, 0) = QuatAxis[0] * QuatAxis[2];
    dxe(5, 0) = QuatAxis[0] * QuatAxis[3];

    //////求解雅可比矩阵
    double jacobian[6][AXISNUM]{};

    robotics_math::getJacobianGeneral(q, g_robot[robot_index].m_mdh.alpha, g_robot[robot_index].m_mdh.a, g_robot[robot_index].m_mdh.d, AXISNUM, jacobian);

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < AXISNUM; j++)
        {
            jacobian_matrix(i, j) = jacobian[i][j];
        }
    }
}

// void kinematics::worldToBase(const double world_cartesian[6], double base_cartesian[6])
// {
//     matrix::Matrix<double, 4, 4> T_worldToBase;
//     T_worldToBase.identity();
//     T_worldToBase(1, 1) = std::cos(-30 * DEGREE2RAD);
//     T_worldToBase(1, 2) = -std::sin(-30 * DEGREE2RAD);
//     T_worldToBase(2, 1) = std::sin(-30 * DEGREE2RAD);
//     T_worldToBase(2, 2) = std::cos(-30 * DEGREE2RAD);

//     matrix::Matrix<double, 4, 4> T_world;
//     T_world.identity();
//     robotics_math::Eula2Mat(T_world, world_cartesian);
//     matrix::Matrix<double, 4, 4> T_base;
//     T_base.identity();
//     T_base = T_worldToBase * T_world;
//     show(T_world);
//     robotics_math::Mat2Eula(T_base, base_cartesian);
// }

void kinematics::worldToBase(const int &robot_index,double base_cartesian[6], const double world_cartesian[6])
{
    // 创建从Base到World的变换矩阵
    matrix::Matrix<double, 4, 4> T_trans;
    T_trans.identity();
    matrix::Matrix<double, 4, 4> T_rots;
    T_rots.identity();
    // 设置旋转部分 (给定的旋转矩阵)

    // 设置平移部分 (给定的平移向量)
    T_trans(1, 3) = -g_robot[robot_index].m_mdh.base_to_world[1]; // y translation
    T_trans(2, 3) = -g_robot[robot_index].m_mdh.base_to_world[2]; // z translation
    T_rots(1, 1) = std::cos(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(1, 2) = -std::sin(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(2, 1) = std::sin(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(2, 2) = std::cos(g_robot[robot_index].m_mdh.base_to_world[3]);
    // 创建Base坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T_world2base;
    T_world2base.identity();
    T_world2base = T_rots.transpose() * T_trans;

    matrix::Matrix<double, 4, 4> T_world;
    robotics_math::Eula2Mat(T_world, world_cartesian);

    // 计算World坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T_base;
    T_base.identity();
    T_base = T_world2base * T_world;

    // 转换回欧拉角表示
    robotics_math::Mat2Eula(T_base, base_cartesian);
}

void kinematics::baseToWorld(const int &robot_index,double world_cartesian[6], const double base_cartesian[6])
{
    // 创建从Base到World的变换矩阵
    matrix::Matrix<double, 4, 4> T_trans;
    T_trans.identity();
    matrix::Matrix<double, 4, 4> T_rots;
    T_rots.identity();
    // 设置旋转部分 (给定的旋转矩阵)
    //std::cout<<"robotindex:"<<robot_index<<",base_to_world:"<< g_robot[robot_index].m_mdh.base_to_world[1]<<std::endl;
    // 设置平移部分 (给定的平移向量)
    T_trans(1, 3) = g_robot[robot_index].m_mdh.base_to_world[1]; // y translation
    T_trans(2, 3) = g_robot[robot_index].m_mdh.base_to_world[2]; // z translation
    T_rots(1, 1) = std::cos(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(1, 2) = -std::sin(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(2, 1) = std::sin(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(2, 2) = std::cos(g_robot[robot_index].m_mdh.base_to_world[3]);
    // 创建Base坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T_base2word;
    T_base2word.identity();
    T_base2word = T_trans * T_rots;

    matrix::Matrix<double, 4, 4> T_base;
    robotics_math::Eula2Mat(T_base, base_cartesian);

    // 计算World坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T_world;
    T_world.identity();
    T_world = T_base2word * T_base;

    // 转换回欧拉角表示
    robotics_math::Mat2Eula(T_world, world_cartesian);
}

void kinematics::addTool(double EEF_world_cartesian[6], const double link7_world_cartesian[6])
{
    // 创建从Base到World的变换矩阵
    matrix::Matrix<double, 4, 4> T_trans;
    T_trans.identity();
    matrix::Matrix<double, 4, 4> T_rots;
    T_rots.identity();

    // 设置平移部分 (给定的平移向量)
    // T_trans(0, 3) = 50; // x translation
    T_trans(0, 3) = 76.335; // x translation
    // 创建Base坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T;
    T.identity();
    T = T_trans * T_rots;
    matrix::Matrix<double, 4, 4> T_lin7_world;
    robotics_math::Eula2Mat(T_lin7_world, link7_world_cartesian);

    // 计算World坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T_EEF_world;
    T_EEF_world.identity();
    T_EEF_world = T_lin7_world * T;
    // 转换回欧拉角表示
    robotics_math::Mat2Eula(T_EEF_world, EEF_world_cartesian);
}

void kinematics::removeTool(double link7_world_cartesian[6], const double EEF_world_cartesian[6])
{
    // 创建从Base到World的变换矩阵
    matrix::Matrix<double, 4, 4> T_trans;
    T_trans.identity();
    matrix::Matrix<double, 4, 4> T_rots;
    T_rots.identity();

    // 设置平移部分 (给定的平移向量)
    // T_trans(0, 3) = -50; // x translation
    T_trans(0, 3) = -76.335; // x translation
    // 创建Base坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T;
    T.identity();
    T = T_trans * T_rots;
    matrix::Matrix<double, 4, 4> T_EEF_world;
    robotics_math::Eula2Mat(T_EEF_world, EEF_world_cartesian);

    //
    matrix::Matrix<double, 4, 4> T_link7_world;
    T_link7_world.identity();
    T_link7_world = T_EEF_world * T;
    // 转换回欧拉角表示
    robotics_math::Mat2Eula(T_link7_world, link7_world_cartesian);
}

void kinematics::show(double cartesian_position[6])
{
    std::cout << "cartesian_position=" << std::endl;
    for (int i = 0; i < 6; i++)
    {
        std::cout << cartesian_position[i] << std::endl;
    }
}

void kinematics::showjoint(double joint[AXISNUM])
{
    std::cout << "joint=" << std::endl;
    for (int i = 0; i < AXISNUM; i++)
    {
        std::cout << joint[i] << std::endl;
    }
}

void kinematics::show(matrix::Matrix<double, 4, 4> T)
{
    std::cout << "T_matrix=" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            std::cout << T(i, j) << " , ";
        }
        std::cout << std::endl;
    }
}

void kinematics::computeRegression(const int &robot_index, double q_in[AXISNUM], matrix::Matrix<double, 7, 28> &Y_G)
{
    double q[AXISNUM];
    double g = 9.81;

    // 创建从Base到World的变换矩阵
    matrix::Matrix<double, 4, 4> T_trans;
    T_trans.identity();
    matrix::Matrix<double, 4, 4> T_rots;
    T_rots.identity();
    // 设置旋转部分 (给定的旋转矩阵)
    // std::cout<<"robotindex:"<<robot_index<<",base_to_world:"<< g_robot[robot_index].m_mdh.base_to_world[1]<<std::endl;
    // 设置平移部分 (给定的平移向量)
    // T_trans(1, 3) = g_robot[robot_index].m_mdh.base_to_world[1]; // y translation
    // T_trans(2, 3) = g_robot[robot_index].m_mdh.base_to_world[2]; // z translation
    T_rots(1, 1) = std::cos(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(1, 2) = -std::sin(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(2, 1) = std::sin(g_robot[robot_index].m_mdh.base_to_world[3]);
    T_rots(2, 2) = std::cos(g_robot[robot_index].m_mdh.base_to_world[3]);
    // 创建Base坐标系下的变换矩阵
    matrix::Matrix<double, 4, 4> T_base2word;
    T_base2word.identity();
    T_base2word = T_trans * T_rots;

    matrix::Matrix<double, 4, 4> T;
    T.identity();
    for (int i = 0; i < AXISNUM; i++)
    {
        // if ((q_in[i] > g_robot[robot_index].m_joint_limit.joint_positive_limit[i]) || (q_in[i] < g_robot[robot_index].m_joint_limit.joint_negative_limit[i]))
        //     return 1; // 输入关节角超限位

        q[i] = q_in[i] + g_robot[robot_index].m_mdh.offset[i];
    }
    for (int k = 0; k < 7; k++)
    {
        for (int j = 0; j < 7; j++)
        {

            T = T_base2word;
            matrix::SquareMatrix<double, 4> Ti;
            if (j < k)
            {
                T.zero();
            }
            else
            {
                for (int i = 0; i <= j; i++)
                {
                    if (i == k)
                    {
                        robotics_math::getTransformationMatrixMDHdot(q[i], g_robot[robot_index].m_mdh.alpha[i], g_robot[robot_index].m_mdh.a[i], g_robot[robot_index].m_mdh.d[i], Ti);
                      
                    }
                    else
                    {
                        robotics_math::getTransformationMatrixMDH(q[i], g_robot[robot_index].m_mdh.alpha[i], g_robot[robot_index].m_mdh.a[i], g_robot[robot_index].m_mdh.d[i], Ti);
                        
                    }
                    T = T * Ti;
                }
            }
            matrix::Matrix<double, 4, 4> Tz;
            Tz(0, 0) = T(2, 0) * g;
            Tz(0, 1) = T(2, 1) * g;
            Tz(0, 2) = T(2, 2) * g;
            Tz(0, 3) = T(2, 3) * g;
           
            Y_G(k, 4 * j) = Tz(0, 0);
            Y_G(k, 4 * j + 1) = Tz(0, 1);
            Y_G(k, 4 * j + 2) = Tz(0, 2);
            Y_G(k, 4 * j + 3) = Tz(0, 3);

            // double cartesian_base_position[6] = {0, 0, 0, 0, 0, 0};
            // robotics_math::Mat2Eula(T, cartesian_base_position);
            // for (int j = 0; j < 6; j++)
            // {
            //     cartesian_position[j] = cartesian_base_position[j];
            // }
            // baseToWorld(robot_index,cartesian_position, cartesian_base_position);
        }
    }
}