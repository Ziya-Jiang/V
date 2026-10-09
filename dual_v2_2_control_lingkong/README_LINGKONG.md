# 独立领控夹爪控制副本

本目录和 `/mnt/ssd/user/V/dual_v2_2_configuration_lingkong` 是独立副本，
原 `dual_v2_2_control`、`dual_v2_2_configuration` 未修改。

构建：

```bash
cmake -S cuarm_rt_control -B cuarm_rt_control/build_lingkong
cmake --build cuarm_rt_control/build_lingkong -j2
```

启动独立规划器时必须显式选择独立配置：

```bash
cd cuarm_upper_planner
CUARM_CONFIG_ROOT=/mnt/ssd/user/V/dual_v2_2_configuration_lingkong \
  /mnt/ssd/robotarm/env/curobo/bin/python dual_v2_2_upper_planner.py
```

代理程序也应设置同一个 `CUARM_CONFIG_ROOT`。右夹爪配置当前保持
`calibrated: false`，确认 Erazor 节点 ID、方向和机械端点前不得解除。
