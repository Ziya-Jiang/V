# @deepseek-ai/dsh-tool-robot

[English](README.md) | 中文

本消费方默认在 `ctx.robot` 上注册三个面向模型的工具：

- `robot_status`：验证进程级仅规划就绪状态。
- `robot_manifest`：检查已声明的 Robot Tool 和 Skill。
- `robot_plan`：创建语义化服务端计划预览。

启用 `approvalGatedExecution: true` 后，还会注册：

- `robot_request_execution`：请求现场人员批准一个精确 Plan。
- `robot_execution_status`：读取权威审批和 Task 状态。

它仍不会注册直接执行、审批、确认、任意 RPC、shell、原子 Tool 或低层运动工具。模型可以请求动作，但不能批准自己的请求。

## 模型体验

模型会收到结构化的状态、manifest、Plan、审批请求和 Task 摘要。`robot_plan` 只接受语义指令；调用 `robot_request_execution` 后，模型必须告知用户正在等待独立人工审批，并通过 `robot_execution_status` 获取状态，不能自行宣称成功。

#### KV Cache 影响

加载或卸载本包会改变工具 schema 和一个具名提示词段，因此下一次模型请求会收到不同前缀。

## 已知限制与暂缓事项

- 结果目前使用通用工具卡片，Plan 尚未投影为持久 Robot Session 事件。丰富任务事件流、场景版本绑定、审批持久恢复和模型发起取消仍暂缓。
