#ifndef HKCLR_JOINT_ARM_H
#define HKCLR_JOINT_ARM_H

#include <memory>
#include <string>
#include <vector>

#include "../arm_base/arm_base.h"
#include "../cxt_can/cxt_can.h"
#include "../joint_interface/joint_interface.h"
#include "spark2_config_loader.h"

/**
 * Lite Arm V2：关节由外部注入；构造时从 `config_path`（默认 `spark2_config.yaml`）读取硬件 YAML。
 * `hardware.motor.enabled` 决定 `existing_motors_` / `enable_motors_` 初值；`sign` 与 `motor_offset` 在用户域与执行器域之间做位置/速度/电流换算；`motor_type` 与注入关节的 `JointKind` 一致时校验通过。
 * 驱动器在 `open()` 时硬件使能一次，`close()` 时关断；周期内 `enableMotors` 仅更新 panel 门控，不重复发使能。
 * `getState` 对每个已使能关节每周期只 `read_status` 一次，再取位置/速度/力矩。
 * 失败或与关节数不一致时抛 `std::runtime_error`。
 */
class Spark2 : public ArmBase {
public:
    explicit Spark2(const std::string &name,
                       const std::string &config_path = "spark2_config.yaml");
    Spark2(const std::string &name, std::vector<std::unique_ptr<JointActuatorBase>> joints,
              const std::string &config_path = "spark2_config.yaml");
    ~Spark2();

    const std::string &config_path() const { return config_path_; }
    const spark2::HardwareConfig &hardware() const { return hardware_; }

    RetCode open() override;
    RetCode close() override;
    RetCode setMode(ControlType mode) override;
    RetCode setCommand(int joint_i, float val) override;
    RetCode getPosition(int joint_i, float *val) override;
    RetCode getVelocity(int joint_i, float *val) override;
    RetCode getTorque(int joint_i, float *val) override;
    RetCode updateEncoderZeroOffset(int joint_i, float offset) override;
    RetCode enableMotor(int joint_i, bool enable) override;
    RetCode enableMotors(const std::vector<bool> &enable) override;
    RetCode getState(int joint_i, float *pos = nullptr, float *vel = nullptr, float *tor = nullptr,
                     int read_delay_us = 0) override;
    RetCode getState(std::vector<float> *pos = nullptr, std::vector<float> *vel = nullptr,
                     std::vector<float> *tor = nullptr, int read_delay_us = 0) override;
    RetCode setCommandAndGetState(const std::vector<float> &set_val, std::vector<float> *pos = nullptr,
                                  std::vector<float> *vel = nullptr, std::vector<float> *tor = nullptr,
                                  int set_delay_us = 0, int read_delay_us = 0) override;

private:
    const std::string config_path_;
    cxt_can bus_{};
    bool owns_bus_{false};
    bool bus_opened_{false};

    static std::string resolve_config_path(const std::string &config_path);
    static int load_joint_count_or_throw(const std::string &config_path);
    static int logical_channel_index(const int *chn_idx, int chn_num, int phys_channel);
    static int phys_channel_for_driver(const spark2::HardwareConfig &hw,
                                       spark2::MotorDriverKind driver);
    static RetCode joint_rc(int r);
    static JointControlMode control_type_to_joint_mode(ControlType mode);
    int init_can_bus(const spark2::HardwareConfig &hw, int *chn_idx_out, int *chn_num_out);
    std::vector<std::unique_ptr<JointActuatorBase>> build_joints(const spark2::HardwareConfig &hw,
                                                                  const int *chn_idx, int chn_num);
    void load_hardware_config();
    void initialize_from_hardware_config();

    double user_position_to_actuator_rad(int joint_i, float user_rad) const;
    double actuator_position_to_user_rad(int joint_i, double actuator_rad) const;
    double user_velocity_to_actuator(int joint_i, float user_vel) const;
    double actuator_velocity_to_user(int joint_i, double actuator_vel) const;
    double user_current_to_actuator(int joint_i, float user_cur) const;
    double actuator_current_to_user(int joint_i, double actuator_cur) const;

    /** 已对驱动器执行过 enable()（与 panel 的 enable_motors_ 请求分离）。 */
    RetCode hardware_enable_joint(int joint_i);
    RetCode hardware_disable_joint(int joint_i);
    bool is_tl_joint(int joint_i) const;

    std::vector<std::unique_ptr<JointActuatorBase>> joints_;
    std::vector<bool> hardware_enabled_;
    std::vector<ControlType> joint_modes_;
    spark2::HardwareConfig hardware_{};
};

#endif
