# Robot Soul DSH 互动介绍网站

[English](README.md) | 中文

面向非技术用户与开发者的单页统一 GUI。它把完整 DeepSeek Harness AI
对话、可消费的 Skill 产品、Plan、架构探索器和独立真机审批面放在同一页面，
自顶向下解释并操作 DeepSeek Harness、Python Agent 与 RobotRAM。

## 启动

在 `dual_v2_2_agent` 根目录先启动本地网页操作台：

```bash
./launch_robot_soul.sh start
```

此时不会初始化真实机器人。打开 <http://127.0.0.1:3091/#playground>，选择
“Real robot”，完成现场确认后点击网页中的启动按钮。Harness AI 在隔离的
`127.0.0.1:3081` 进程中运行并通过 iframe 嵌入。状态和停止使用
`./launch_robot_soul.sh status|stop`。需要跳过网页、直接启动完整真机栈时，
可显式执行 `./launch_robot_soul.sh start-all`。

纯模拟开发可从 `deepseek-harness` 根目录直接运行：

```bash
./node_modules/.bin/tsx apps/robot-soul-dsh/src/index.ts
```

## 体验边界

- 默认体验完全模拟。`POST /api/simulate/plan` 只生成不可执行的展示 Plan。
- 可选 Live 模式只读取本机 plan-only 或显式启用的生产 Python Agent 的
  health/status 与 `manifest`。
- 真机写入 API 默认锁定；只有服务器双门同时开启后，独立审批面才能使用
  `plan`、`execute`、`task_status`、`task_events` 和 `cancel`。
- 网站永不暴露 `call`、原子 Tool、CAN、关节或 shell 控制入口。
- Live 断线时自动保留完整静态目录与浏览器模拟体验。
- token 与 `outputDir` 仅留在服务端，不会返回浏览器。
- 常规网页请求不会启动控制栈。统一启动器模式下，只有同源 loopback 的
  `/api/ops/recover`、精确现场口令和 supervisor 的可恢复故障状态同时满足时，
  操作员按钮才能请求启动器重新初始化硬件面；AI 没有该入口。生产 Python
  Agent 始终是唯一硬件所有者。

Live 只读连接默认使用 `127.0.0.1:15555`，可通过以下服务端环境变量调整：

- `ROBOT_SOUL_AGENT_HOST`：必须为数字 loopback 地址。
- `ROBOT_SOUL_AGENT_PORT`：Agent TCP 端口。
- `ROBOT_SOUL_HARDWARE_READONLY`：设为 `1` 时选择生产 Agent 只读网关；
  除 `0`、`1` 外的值会被拒绝。
- `ROBOT_SOUL_APPROVED_EXECUTION`：阶段二服务器门；必须精确设为
  `ENABLE_DUAL_V2_2_HARDWARE`。
- `ROBOT_SOUL_AGENT_TOKEN`：可选共享 token，不发送给浏览器。
- `ROBOT_SOUL_AGENT_TIMEOUT_MS`：Provider 请求超时。
- `ROBOT_SOUL_AGENT_MAX_RESPONSE_BYTES`：响应大小上限。
- `ROBOT_SOUL_OUTPUT_DIR`：Agent 任务输出目录，不暴露给网页。
- `ROBOT_SOUL_HARDWARE_PLAN_TTL_MS`：网站审批 Plan 的寿命，默认十分钟。
- `ROBOT_SOUL_HARNESS_HOST` / `ROBOT_SOUL_HARNESS_PORT`：嵌入式 Harness Web
  地址；网站独立启动默认 `127.0.0.1:3080`，统一启动器默认使用 `3081`，
  只允许数字 loopback。

## 分阶段硬件访问

阶段一只读必须显式启用：

```bash
ROBOT_SOUL_HARDWARE_READONLY=1 \
  ./node_modules/.bin/tsx apps/robot-soul-dsh/src/index.ts
```

它要求 loopback 上已有由操作者管理的 Python Agent。此时网站只获得
`health()` 与 `manifest()` 窄门面，`/api/hardware/policy` 报告
`executeEnabled: false`。

阶段二必须在生产 Agent 已完成硬件 bootstrap 并报告 `ready` 后，由现场
操作者显式打开两个服务器门：

```bash
ROBOT_SOUL_HARDWARE_READONLY=1 \
ROBOT_SOUL_APPROVED_EXECUTION=ENABLE_DUAL_V2_2_HARDWARE \
ROBOT_SOUL_OUTPUT_DIR=/absolute/path/to/task-output \
  ./node_modules/.bin/tsx apps/robot-soul-dsh/src/index.ts
```

即使服务器门已开启，每次动作仍需满足全部条件：

1. 只接受语义目标，拒绝关节角、速度、CAN、shell 等低层命令。
2. Agent 必须为非 plan-only、`bootstrapState=ready` 且没有活动任务。
3. Plan 只能包含 `perceive_scene`、`pick_and_place`、`tidy_table`、
   `safety_reset`；`start_robot` 与所有原子 Tool 均被拒绝。
4. Skill 参数必须通过严格白名单、偏移范围、12 步和 8 个动作单元上限。
5. Plan 必须由当前网站进程签发、仍在有效期内，且一次只允许一个活动 Task。
6. 操作者必须逐项确认在场、工作区清空、急停可用、机器人已检查，并输入
   精确口令 `ENABLE_DUAL_V2_2_HARDWARE`。
7. 真机 POST 只接受同源 loopback 的 `application/json` 请求。

`cancel` 是协作取消，不是物理急停。任何异常运动都必须使用独立急停。

## 故障与人工恢复

运动中或状态不明确时，单次 CAN 命令/状态交换失败仍会立即停止并锁定硬件面。
只有控制器确认处于静止位置保持时，才允许最多两次连续瞬态失败；第三次失败仍
锁定。硬件面故障不会再关闭 Harness 和本网页：状态栏会显示原因，现场人员检查
硬件与急停后可点击“重新初始化机器人”。恢复只重启控制栈与 Python Agent，
不会重启 Harness，也不会把恢复能力交给模型。若 `sudo` 授权已过期，请先在
终端执行 `sudo -v`，再点击恢复。

当 DeepSeek Harness 使用 `examples/robot-approved/cordis.yml` 时，模型通过
`robot_request_execution` 只能创建 `awaiting_approval` 请求。网站的 AI
审批队列读取该请求并显示原始服务端 Plan；确认口令只发送到 Python Agent，
不会进入 Harness 上下文。批准后 Python Agent 自动创建 Task，模型通过
`robot_execution_status` 读取权威结果。

## 与 Robot Assistant 的区别

`apps/robot-assistant` 是旧的独立任务界面。Robot Soul DSH 是新的统一入口：
`launch_robot_soul.sh` 负责启动隔离进程，网页同时承载 Harness 对话、人工
审批与任务事件。网页本身仍不持有硬件，也不提供任意调用或原子 Tool 执行。

## API

只读：

- `GET /api/content`
- `GET /api/catalog`
- `GET /api/architecture`
- `GET /api/hardware/policy`
- `GET /api/live/status`
- `GET /api/live/manifest`
- `GET /api/ops/status`，聚合 Harness、Python Agent、硬件策略与故障状态

本地模拟：

- `POST /api/simulate/plan`，请求体为
  `{ "instruction": "语义目标" }`

阶段二审批执行（默认 403 锁定）：

- `POST /api/hardware/plan`
- `POST /api/hardware/execute`
- `GET /api/hardware/approvals`
- `POST /api/hardware/approvals/approve`
- `POST /api/hardware/approvals/deny`
- `GET /api/hardware/task/status?taskId=...`
- `GET /api/hardware/task/events?taskId=...&cursor=...`
- `POST /api/hardware/task/cancel`

仅现场人工恢复：

- `POST /api/ops/recover`，仅接受同源 loopback JSON、精确确认口令和
  supervisor 已标记的可恢复故障；不注册为 Harness Tool

## 验证

```bash
./node_modules/.bin/tsc -p apps/robot-soul-dsh/tsconfig.json --noEmit
./node_modules/.bin/vitest run apps/robot-soul-dsh/tests
```

交互验收应覆盖桌面与移动端、键盘卡片导航、Dialog 的 Escape 关闭、深浅色、
`prefers-reduced-motion`、模拟回退、架构节点、协议拒绝和安全判断游戏。
