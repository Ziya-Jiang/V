# 仅规划 Robot Agent

[English](README.md) | 中文

该 overlay 将已发布的 Web Profile 连接到现有 Python Robot Agent，同时不暴露硬件执行能力。

启动一个专用的仅规划 Python Agent。不要在此模式下使用 `launch_robot_assistant.sh`，因为生产 Supervisor 会启动 eager 硬件 bootstrap。

```sh
cd /mnt/ssd/user/V/dual_v2_2_agent
ROBOT_AGENT_PLAN_ONLY=1 \
ROBOT_AGENT_TOKEN="choose-a-long-random-token" \
ROBOT_AGENT_PORT=15555 \
python3 -m robot_agent.server
```

在第二个终端中使用 opt-in overlay 启动 Harness：

```sh
cd /mnt/ssd/user/V/dual_v2_2_agent/deepseek-harness
ROBOT_AGENT_HOST=127.0.0.1 \
ROBOT_AGENT_PORT=15555 \
ROBOT_AGENT_TOKEN="choose-a-long-random-token" \
ROBOT_AGENT_OUTPUT_DIR=/absolute/path/to/scene-output \
npm run dsh -- web --patch examples/robot-dry-run/cordis.yml
```

模型只能看到 `robot_status`、`robot_manifest` 和 `robot_plan`。每次 manifest 或 plan 调用都会先验证 Python 进程仍处于仅规划模式、从未 bootstrap 硬件且没有活跃任务。当 `ROBOT_AGENT_PLAN_ONLY=1` 时，Python 服务端自身也会拒绝 `execute`、`task_status`、`task_events`、`cancel` 和旧 `call` 请求。

返回的 `planId` 是仅存储在 Python Agent 进程内的预览标识符。该 overlay 不包含任何执行它的路径。
