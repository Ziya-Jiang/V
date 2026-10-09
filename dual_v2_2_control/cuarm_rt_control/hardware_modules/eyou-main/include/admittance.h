#ifndef ADMITTANCE_H
#define ADMITTANCE_H

#include <cstdint>
#include <cstdio>
#include <csignal>
#include <thread>
#include <iomanip>
#include "torque_sensor.h"
#include "eyou_joints.h"
#include <cmath>

#define CLAMP(x, low_lim, high_lim) ((x) < (low_lim) ? low_lim : ((x) > (high_lim) ? high_lim : (x)))

extern uint32_t motor_id[14];
extern int enable_motor_index[14];
extern uint32_t sensor_id[14];
extern int enable_sensor_index[14];

extern EyouJoints dual_arm;
extern EyoTorSenor dual_arm_sensors;
void admittance(int * enable_motor_index, float* gain, float* damping, float *limits, int data_size, int time_interval_us);

#endif // ADMITTANCE_H