# @deepseek-ai/dsh-robot

[English](README.md) | 中文

`RobotRuntime` 定义 `ctx.robot` 能力接口。它将提供方传输与模型工具分离，并默认保持仅规划：

- `status(signal?)`：读取提供方健康状态和仅规划就绪状态。
- `manifest(signal?)`：读取已声明的 Robot Tool 和 Skill。
- `plan({ instruction }, signal?)`：创建不可变的服务端计划预览。
- `requestExecution({ planId }, signal?)`：创建一个不会产生动作、等待独立人工审批面的逐 Plan 请求。
- `executionStatus({ requestId }, signal?)`：读取由本提供方创建的审批与 Task 权威状态。

该约定仍没有直接执行、审批、确认、任意 RPC、关节空间、力、速度或 shell 操作。模型可以请求执行，但不能批准自己的请求；只有硬件所有者的独立操作界面才能把该精确 Plan 变成 Task。

## 模型体验

本包不直接影响模型。`@deepseek-ai/dsh-tool-robot` 负责模型可见的名称、schema、指令、结果和呈现。

#### KV Cache 影响

不会直接导致失效；本 Service Definition 不贡献提示词文本。

## 已知限制与暂缓事项

- 当前接口支持一个提供方，并且尚未把 Robot Plan 持久化进 Harness Session 日志。丰富任务事件流、场景版本、多机器人提供方选择和待审批请求的持久化恢复仍暂缓。
