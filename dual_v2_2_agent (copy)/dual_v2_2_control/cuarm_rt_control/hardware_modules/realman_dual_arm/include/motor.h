#ifndef MOTOR_H
#define MOTOR_H
#include "can_fd.h"
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include "logger.h"
#ifdef __cplusplus
extern "C"
{
#endif
typedef enum{
    open_loop = 0,
    cur_control = 1,
    vel_control = 2,
    pos_control = 3
}CONTROL_MODE;

#define ADDR_IAP_FLAG 0x49
#define ADDR_ENABLE_FLAG 0x0A
#define ADDR_WORK_MODE 0x30
#define ADDR_CLEAR_ERROR 0x0F
class WhjMotor {
public:
    WhjMotor(uint32_t motor_id=1, int can_dev_index=0, double gear_ratio=81, int sign=1, double init_mdh_rad=0.0);
    int set_IAP();
    int enable();
    int disable();
    int clear_error();
    int select_mode(uint8_t control_mode);
    int set_velocity(double vel_rpm);
    int set_current(double cur_ma);
    int set_absposition(double pos_rad);
    int read_status(void);
    int decode_motor_data(uint8_t* resp_max_data);
    int decode_motor_data2(uint8_t* resp_max_data);
    int servo_control(uint32_t can_id, int32_t control_target);
    int write_register(uint8_t addr, uint8_t values, uint8_t dlc);

    uint32_t motor_id_;
    float pos_;
    float vel_;
    float cur_;
    float last_pos_;
    int can_dev_index_;
    int control_success_;
    int write_success_;
    int error_code_;
    int enable_status_;
    int brake_status_;
    double gear_ratio_;    // Added
    int sign_;             // Added
    double init_mdh_rad_; // Added

private:
};
#ifdef __cplusplus
}
#endif
#endif