#ifndef HKCLR_JOINT_INTERFACE_H
#define HKCLR_JOINT_INTERFACE_H

#include <cstdint>

enum class JointKind {
    TlCanopen,
    Rmd,
};

enum class JointControlMode {
    OpenLoop,
    Position,
    Velocity,
    Current,
};

class JointActuatorBase {
public:
    virtual ~JointActuatorBase() = default;

    virtual int request_iap_mode() = 0;
    virtual int set_absolute_position_rad(double pos_rad) = 0;
    virtual int set_velocity(double vel_rad_s) = 0;
    virtual int set_current(double cur_A) = 0;
    virtual int read_status() = 0;
    virtual int enable() = 0;
    virtual int disable() = 0;
    virtual int clear_error() = 0;
    virtual int select_mode(JointControlMode mode) = 0;
    virtual int decode_feedback(uint32_t can_id, const uint8_t *data, uint8_t dlc) = 0;

    virtual double get_position() const = 0;
    virtual double get_velocity() const = 0;
    virtual double get_current() const = 0;
    virtual int is_control_success() const = 0;
    virtual JointKind joint_kind() const = 0;
    virtual uint32_t can_node_id() const = 0;
    virtual bool accepts_can_id(uint32_t can_id) const = 0;
};

#endif
