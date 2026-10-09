# Human-approved Robot Agent

English | [中文](README.zh.md)

This overlay lets the DeepSeek Harness model request real execution of one
server-side Plan. It does not give the model an execute or approval credential.
The production Python Agent remains the sole hardware owner, and every Plan
must be independently approved by an on-site human in Robot Soul DSH.

## One-command startup

Stop the legacy Robot Assistant stack first. Keep the workspace clear and the
physical emergency stop immediately reachable:

```sh
cd /mnt/ssd/user/V/dual_v2_2_agent
./launch_robot_assistant.sh stop

./launch_robot_soul.sh start
```

The launcher generates one server-only token and starts DeepSeek Harness, the
unified Robot Soul GUI, and the safety supervisor without initializing the real
robot. On first use it builds Harness Web, so no `pnpm install` or
three-terminal workflow is required.

Open one page:

<http://127.0.0.1:3091/#playground>

The full DeepSeek Harness conversation is embedded beside the operator guide.
The Human Approval section shows the model-created server Plan, four on-site
checks, Task state, and Python Agent events. Harness remains process-isolated on
`127.0.0.1:3081` even though it appears inside the unified GUI.

Use the same launcher for status and shutdown:

```sh
./launch_robot_soul.sh status
./launch_robot_soul.sh stop
```

Select **Real robot**, complete the on-site confirmation, and click the startup
button before the control stack and sole Python Agent are initialized. To start
the complete hardware stack directly from a terminal, explicitly run
`./launch_robot_soul.sh start-all`. Startup failures roll back in reverse
order. During operation, a control-stack or Agent fault safely stops and locks
the hardware plane while Harness and the unified page stay online. If the page
reports expired `sudo` authorization, run `sudo -v` in a terminal and retry.

## Model execution flow

The model receives five tools:

- `robot_status`
- `robot_manifest`
- `robot_plan`
- `robot_request_execution`
- `robot_execution_status`

The expected flow is Plan → model execution request → human review/approval →
Python Agent Task. Before approval, no hardware action is created. The model
never receives `approve_execution`, the confirmation phrase, direct execute,
legacy `call`, atomic Tools, CAN, joints, force, velocity, or shell control.

The first live integration should use only a minimal left-arm safety-reset Plan.
Approve only after the complete Plan contains the expected `safety_reset`.
Cooperative cancellation is not an emergency stop.
