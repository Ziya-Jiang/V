# 人工审批 Robot Agent

[English](README.md) | 中文

该 overlay 允许 DeepSeek Harness 模型请求真实执行一个服务端 Plan，但不会把
execute 或审批凭据交给模型。生产 Python Agent 始终是唯一硬件所有者，每个
Plan 都必须由现场人员在 Robot Soul DSH 中独立批准。

## 一键启动

先停止旧的 Robot Assistant 栈，确保现场无人进入工作区且物理急停可立即触达：

```sh
cd /mnt/ssd/user/V/dual_v2_2_agent
./launch_robot_assistant.sh stop

./launch_robot_soul.sh start
```

启动器会自动生成只在服务端共享的 token，并先启动 DeepSeek Harness、统一
Robot Soul GUI 和安全监督器，不会立即初始化真实机器人。首次运行会构建
Harness Web，因此不需要运行 `pnpm install`，也不需要管理三个终端。

只打开一个页面：

<http://127.0.0.1:3091/#playground>

页面左侧是完整 DeepSeek Harness AI 对话，右侧解释操作步骤；“人工审批”区会
显示模型提交的原始服务端 Plan、四项现场确认、Task 状态与 Python Agent
事件。AI 页面仍运行在隔离的 `127.0.0.1:3081` 进程中，但已嵌入统一 GUI。

状态和停止同样只使用一个脚本：

```sh
./launch_robot_soul.sh status
./launch_robot_soul.sh stop
```

在页面选择“Real robot”，完成现场确认并点击启动按钮后，才会启动控制栈和唯一
Python Agent。需要从终端直接启动完整真机栈时，显式执行
`./launch_robot_soul.sh start-all`。任何组件启动失败都会逆序回滚。运行期间若
控制栈或 Agent 故障，硬件面会安全停止并锁定，但 Harness 与统一网页继续运行；
页面会显示故障及仅供现场人员使用的“重新初始化机器人”按钮。若网页提示
`sudo` 授权过期，先在终端执行 `sudo -v` 再重试。

## 模型执行流程

模型会获得五个工具：

- `robot_status`
- `robot_manifest`
- `robot_plan`
- `robot_request_execution`
- `robot_execution_status`

预期流程为：Plan → 模型执行请求 → 人工检查/批准 → Python Agent Task。批准
之前不会创建任何硬件动作。模型永远不会获得 `approve_execution`、确认口令、
直接 execute、旧 `call`、原子 Tool、CAN、关节、力、速度或 shell 控制。

首次真机联调只允许“左臂安全复位”一类最小 Plan。确认完整 Plan 仅包含预期的
`safety_reset` 后再批准。协作取消不是物理急停。
