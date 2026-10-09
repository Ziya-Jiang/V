#ifndef HKCLR_LITE_ARM_V2_CONFIG_LOADER_H
#define HKCLR_LITE_ARM_V2_CONFIG_LOADER_H

#include <string>

#include "spark2_hardware.h"

namespace spark2 {

/**
 * 从 YAML 文件解析 `version` / `num_joints` / `hardware` 到 `HardwareConfig`。
 * @param path  配置文件路径
 * @param out   成功时写入；失败时内容未定义
 * @param err   可选，失败时写入人类可读原因
 * @return      是否解析且校验通过
 */
bool load_hardware_yaml(const std::string &path, HardwareConfig *out, std::string *err = nullptr);

} // namespace spark2

#endif
