/*
 * @Author:
 * @Date:
 * @LastEditors:
 * @LastEditTime:
 * @FilePath: /ros2_demo/src/gac_arm_cpp/src
 * @Description:
 */
#pragma once
#ifndef ROBOT_STRUCT_H_
#define ROBOT_STRUCT_H_

#define PI 3.1415926
#define AXISNUM 7
#define ROBOT_NUM 2
#define DEGREE2RAD PI / 180
#define RAD2DEGREE 180 / PI
#define RATE 8
#define PERIOD 0.008
#define MAXDOF 10


#include <iostream>
#include <list>
#include <deque>
#include <algorithm>
#include <array>
#include <string.h>

struct MDH
{

	// HKCLR ARM left
	//  double a[AXISNUM]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 62.0 };
	//  double alpha[AXISNUM]{ -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD };
	//  double d[AXISNUM]{ 85, 0.0, 250, 0.0, 210, 0.0, 0.0 };
	//  double offset[AXISNUM]{ 90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD+90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, 180.0 * DEGREE2RAD-60.0*DEGREE2RAD, 180.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, 0.0 * DEGREE2RAD };

	double a[AXISNUM]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 62.0};
	double alpha[AXISNUM]{0.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, 90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD, -90.0 * DEGREE2RAD};
	double d[AXISNUM]{85, 0.0, 250, 0.0, 210, 0.0, 0.0};
	double offset[AXISNUM]{0, 0, 0, 0, 0, -90 * DEGREE2RAD, 0};
	double sign[AXISNUM]{1, 1, 1, -1, 1, -1, -1};
	double base_to_world[6]{0.0, 143.31, 332.44, -60 * DEGREE2RAD, 0.0, 0.0};
	double m_init_mdhpos[AXISNUM]{-3.23179e+06, -151971, -3.05434e+06, -150993, 1.5849e+06, -382888, 1.91445e+06};
};

struct JointLimit
{
	double joint_positive_limit[AXISNUM]{PI, PI/2, PI, PI/2, PI, PI/2, PI/2};
	double joint_negative_limit[AXISNUM]{-PI, -PI/2, -PI, -PI/2, -PI, -PI/2, -PI/2};
};

struct CartesianSpaceLimit
{
	double cartesian_positive_limit[6]{};
	double cartesian_negative_limit[6]{};
};

struct Ratio
{
	double joint_ratio[AXISNUM]{121, 121, 101, 101, 101, 101, 101};
};





class Robot
{
public:
	Robot() {};
	~Robot() {};

public:

	//	结构相关参数（DH参数，关节限位，末端结构尺寸等）
	MDH m_mdh;
	JointLimit m_joint_limit;
	CartesianSpaceLimit m_cartesian_space_limit;
	Ratio m_ratio;
	double m_ts = PERIOD;
};

#ifdef ROBOT_GLOBAL // 只在interface接口层cpp定义
#define ROBOT_EXT
#else
#define ROBOT_EXT extern
#endif
ROBOT_EXT Robot g_robot[ROBOT_NUM];

#endif