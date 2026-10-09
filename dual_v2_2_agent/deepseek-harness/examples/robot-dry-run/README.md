# Plan-only Robot Agent

English | [中文](README.zh.md)

This overlay connects the shipped Web profile to the existing Python Robot
Agent without exposing hardware execution.

Start a dedicated plan-only Python Agent. Do not use
`launch_robot_assistant.sh` for this mode because the production supervisor
starts eager hardware bootstrap.

```sh
cd /mnt/ssd/user/V/dual_v2_2_agent
ROBOT_AGENT_PLAN_ONLY=1 \
ROBOT_AGENT_TOKEN="choose-a-long-random-token" \
ROBOT_AGENT_PORT=15555 \
python3 -m robot_agent.server
```

In a second terminal, start Harness with the opt-in overlay:

```sh
cd /mnt/ssd/user/V/dual_v2_2_agent/deepseek-harness
ROBOT_AGENT_HOST=127.0.0.1 \
ROBOT_AGENT_PORT=15555 \
ROBOT_AGENT_TOKEN="choose-a-long-random-token" \
ROBOT_AGENT_OUTPUT_DIR=/absolute/path/to/scene-output \
npm run dsh -- web --patch examples/robot-dry-run/cordis.yml
```

The model receives only `robot_status`, `robot_manifest`, and `robot_plan`.
Every manifest or plan call first verifies the Python process is still in
plan-only mode, has never bootstrapped hardware, and has no active task. The
Python server itself rejects `execute`, `task_status`, `task_events`, `cancel`,
and legacy `call` requests while `ROBOT_AGENT_PLAN_ONLY=1`.

The returned `planId` is a preview identifier stored only in the Python Agent
process. This overlay contains no route that can execute it.
