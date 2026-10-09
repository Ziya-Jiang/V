# @deepseek-ai/dsh-robot-python-agent

[English](README.md) | 中文

本包通过现有的 loopback NDJSON Python Robot Agent 提供 `ctx.robot`，默认保持仅规划模式：

- 传输代码只能发送 `health`、`manifest` 和 `plan`。
- `status`、`manifest` 和 `plan` 均保留调用方取消信号，并限制响应大小和超时。
- `manifest` 和 `plan` 首先要求健康响应声明精确的仅规划方法集合、`planOnly: true`、没有 eager/bootstrap 活动并且没有活跃任务。
- 输出目录属于提供方配置，模型不能选择任意文件系统路径。
- 端点必须是数值 loopback 地址。

Python Agent 必须以 `ROBOT_AGENT_PLAN_ONLY=1` 启动。该服务端模式会禁用硬件 bootstrap，并在分派前拒绝所有执行面方法。

`approvalGatedExecution: true` 是连接已报告硬件 `ready` 的生产 Agent 的独立显式模式。它只向 Harness 传输增加 `request_execution` 和 `execution_request_status`。提供方没有 `approve_execution`、确认口令、直接 execute 或任意调用方法。模型创建的请求保持无动作，直到操作者通过 Robot Soul DSH 批准该精确 Plan。

## 配置

- `host`（默认 `127.0.0.1`）
- `port`（默认 `15555`）
- `token`（可选共享 token）
- `timeoutMs`（默认 `30000`）
- `maxResponseBytes`（默认 1 MiB）
- `maxInstructionBytes`（默认 16 KiB）
- `outputDir`（默认当前目录；必须为绝对路径）
- `approvalGatedExecution`（默认 `false`）

## 模型体验

通过 `@deepseek-ai/dsh-tool-robot` 间接影响。提供方失败会保留稳定的 `RobotError.code` 分类，包括配置、连接、超时、取消、协议、远端和仅规划策略失败。

#### KV Cache 影响

不会直接导致失效；本提供方不贡献提示词文本。

## 已知限制与暂缓事项

- 本提供方面向仓库特有的单请求单连接 NDJSON 协议，并在模式敏感调用前执行健康校验。连接池、协议版本协商、Plan/审批持久恢复和多机器人路由仍暂缓。
