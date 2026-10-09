# Erazor 右侧领控夹爪独立适配

本目录只新增平行构建入口，不修改 Erazor 的：

- `/home/erazer/cuarm_panel_control`
- `/home/erazer/Seldon/dual_arm_cockpit/rt`
- `/home/erazer/Seldon/dual_arm_cockpit/runtime/build/seldon_rt_control`

部署目标为 `/home/erazer/Seldon/dual_arm_cockpit/rt_lingkong`，构建产物为
`runtime/build_lingkong/seldon_rt_control`。回退时继续启动原
`runtime/build/seldon_rt_control` 即可。

## 安全状态

`right_gripper.yaml` 默认 `calibrated: false`。此时驱动只读取状态，任何
位置命令都会失败，RT 也会拒绝进入硬件控制。只有完成供电、接线、节点
ID、方向和机械端点确认后才能改为 `true`。

2026-10-06 的只读检查结果：

- Erazor `can1`：1 Mbps、ERROR-ACTIVE、无总线错误。
- 节点 1–32 对 `0x9A` 均无回复，因此尚未确认节点 ID 和当前角度。
- 当前配置中的节点 8、张开 −800°、闭合 −125°仅来自 `.175` 参考代码，
  不能视为 Erazor 实机标定结果。

## 只读探测

确认原 RT 已停止后执行：

```bash
python3 rt_lingkong/probe_lingkong.py --interface can1
```

该脚本只发送 `0x9A/0x92/0x9C` 状态查询，不发送使能、位置或转矩写入。

## 构建

```bash
cd /home/erazer/Seldon/dual_arm_cockpit
python3 rt_lingkong/build_lingkong.py
```

先用仿真检查独立入口：

```bash
rt_lingkong/run_lingkong_rt.sh --simulation strict
```

真机只能在节点与端点确认、`calibrated: true` 且没有其他机器人控制进程时
显式运行：

```bash
rt_lingkong/run_lingkong_rt.sh --hardware strict
```

构建不会覆盖原二进制。首次真机运动前应先通过协议单测和虚拟 CAN 测试，
再以 `speed_dps: 50`、`torque_limit_raw: 200` 空载小步确认方向。
