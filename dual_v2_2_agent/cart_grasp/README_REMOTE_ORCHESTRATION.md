# 21 主机运行原 Agent，155 提供 RGB-D

脚本已经部署到 21 主机：

```text
/mnt/ssd/user/V/dual_v2_2_agent/cart_grasp/run_remote_agent.sh
```

在 21 上运行：

```bash
ssh -tt vlmdemo@10.30.13.21
cd /mnt/ssd/user/V/dual_v2_2_agent
./cart_grasp/run_remote_agent.sh --ensure-camera
```

脚本会：

1. 在 21 建立到 `erazer@10.30.3.155` 的临时 SSH ControlMaster；密码只在终端提示，不写入脚本。
2. 检查 155 的 `CV2TC5100060` RGB-D 快照；使用 `--ensure-camera` 时，若快照过期会切换到 155 的共享 RGB-D 采集入口。
3. 复用或启动 21 的 `dual_v2_2_upper_planner.py`，使用 `/tmp/dual_v2_2_configuration_remote`，GPU 0，BLAS/OpenMP 单线程。
4. 启动原始 `/mnt/ssd/user/V/dual_v2_2_agent/dual_v2_2_agent_pcl.py`，通过 `--remote-rgbd` 获取 155 图像，使用原有 `10165/10166` Agent↔Planner 协议。

启动后由原 Agent 提示选择 `bowl`、`cup` 等任务类型。脚本不会自动输入任务，也不会绕过双臂页面的 Planner 连接、RT 反馈和安全租约检查。执行前需在 155 双臂页面确认 Planner 地址为 `10.30.13.21:30241` 并按页面流程允许 Planner 控制。

只做链路检查、不启动 Agent：

```bash
./cart_grasp/run_remote_agent.sh --check-only
```

如果不希望脚本切换 155 相机所有权，省略 `--ensure-camera`；此时 155 必须已有不超过 3 秒的新鲜 `runtime/rgbd/status.json`。
