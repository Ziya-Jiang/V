# Robot Assistant MVP

Robot Assistant is a three-layer embodied-agent scaffold:

- The outer layer serves a browser interface, task API, and live execution events.
- The middle layer converts natural-language requests into a serializable TaskIR.
- The inner layer discovers constrained skills and is the only layer allowed to call a robot driver.

The first task is table cleaning. It decomposes each perceived object into `approach`, `grasp`, and `place`, followed by `home`. The bundled driver is intentionally dry-run only and never opens robot hardware.

## Run

From the repository root:

```sh
pnpm --filter @deepseek-ai/dsh-robot-assistant dev
```

Open `http://127.0.0.1:3090`. Set `ROBOT_ASSISTANT_PORT` to use another loopback port.

## Python Agent runtime

The production GUI can connect directly to the Python Agent's one-request-per-connection
NDJSON server. The server is required to listen on a numeric loopback address; no shell
process or RobotRAM HTTP bridge is used in this runtime.

```sh
ROBOT_ASSISTANT_RUNTIME=python-agent \
ROBOT_AGENT_HOST=127.0.0.1 \
ROBOT_AGENT_PORT=15555 \
ROBOT_AGENT_TOKEN="optional-shared-token" \
ROBOT_AGENT_TIMEOUT_MS=30000 \
ROBOT_AGENT_OUTPUT_DIR=/absolute/path/to/agent-output \
pnpm --filter @deepseek-ai/dsh-robot-assistant dev
```

`ROBOT_AGENT_HOST`, `ROBOT_AGENT_PORT`, and `ROBOT_AGENT_TIMEOUT_MS` default to
`127.0.0.1`, `15555`, and `30000`. `ROBOT_AGENT_TOKEN` is omitted from requests when
unset. `ROBOT_AGENT_OUTPUT_DIR` defaults to the server process working directory.

The browser workflow is deliberately two-phase:

1. Submit an instruction to `POST /api/tasks`. The GUI displays the returned Python
   Plan, constraints, and steps without executing it.
2. Inspect the Plan. If it is wrong, discard it through `DELETE /api/tasks/:planId`.
   If it is correct and the Agent reports READY, type `ENABLE_DUAL_V2_2_HARDWARE` and confirm through
   `POST /api/tasks/:planId/confirm`.
3. Follow Python `task_events` on `/api/events`, or request cooperative cancellation
   through `POST /api/tasks/:taskId/cancel`.

Only one Plan preview or execution task may be active at a time. `GET /api/state`
reports `runtime`, Python Agent readiness (`loading`, `ready`, or `error`), the active
preview, and the active task. Cancellation is not a physical emergency stop; use the
robot's physical emergency-stop device whenever safety requires it.

The Python transport supports `health`, `manifest`, `plan`, `execute`, `task_status`,
`task_events`, and `cancel`. Responses are size-limited, timeout-bounded, and validated
before entering the HTTP/SSE layer.

For the complete real-robot stack, use the repository-level supervisor instead of
starting these services individually:

```sh
cd /mnt/ssd/user/V/dual_v2_2_agent
export ROBOT_ASSISTANT_HARDWARE_CONFIRM=ENABLE_DUAL_V2_2_HARDWARE
./launch_robot_assistant.sh check
./launch_robot_assistant.sh start
```

It starts Control first, then the persistent Python Agent in the `robotram_wx`
environment, and finally the GUI. Use `./launch_robot_assistant.sh status` and
`./launch_robot_assistant.sh stop` to inspect or cleanly stop the complete stack.
The supervisor generates a local shared token and refuses conflicting robot owners
or occupied ports. A background safety supervisor also watches Control logs,
processes, Agent health, and GUI health; a Control `ERROR`/`[FAULT]` or repeated
service-health failure stops GUI, Agent, Planner, Upper, and RT in safe order.

The Python planner uses its deterministic fallback by default. To enable its LLM
fallback, set `DEEPSEEK_API_KEY`; optional settings are `DEEPSEEK_API_URL` and
`DEEPSEEK_MODEL`.

## Agent planner

The deterministic planner is the default. Enable the DeepSeek planner explicitly:

```sh
ROBOT_ASSISTANT_PLANNER=deepseek \
DEEPSEEK_API_KEY=... \
pnpm --filter @deepseek-ai/dsh-robot-assistant dev
```

The model may select registered skill identifiers and semantic inputs only. Runtime validation rejects coordinates, force, speed, shell commands, unknown skills, oversized plans, and plans that do not end with the home skill. Constraints always come from the trusted registry.

## RobotRAM provider

The RobotRAM provider targets a separately supervised atomic-operation bridge on loopback. It fails during startup unless hardware mode and its exact confirmation are both present:

```sh
export ROBOTRAM_BRIDGE_TOKEN="replace-with-at-least-24-random-characters"
python ../RobotRAM/robot_assistant_bridge.py --token "$ROBOTRAM_BRIDGE_TOKEN"

# Replace the dry-run command above with this only after the control stack and
# current scene manifests are ready:
ROBOT_ASSISTANT_HARDWARE_CONFIRM=ENABLE_DUAL_V2_2_HARDWARE \
python ../RobotRAM/robot_assistant_bridge.py \
  --mode hardware \
  --token "$ROBOTRAM_BRIDGE_TOKEN"

ROBOT_ASSISTANT_DRIVER=robotram \
ROBOT_ASSISTANT_HARDWARE_CONFIRM=ENABLE_DUAL_V2_2_HARDWARE \
ROBOTRAM_BRIDGE_URL=http://127.0.0.1:8091 \
ROBOTRAM_BRIDGE_TOKEN="$ROBOTRAM_BRIDGE_TOKEN" \
pnpm --filter @deepseek-ai/dsh-robot-assistant dev
```

The bundled bridge defaults to dry-run and enforces `approach -> grasp -> place` ordering. Hardware mode additionally requires `--mode hardware`, the exact hardware confirmation, the running RT/Upper/Planner stack, and current `detailed_subtasks_manual.json` plus `vlm_categories.json` manifests. The default per-operation timeout is five minutes and can be changed with `ROBOTRAM_OPERATION_TIMEOUT_MS`. Direct subprocess invocation of `dual_v2_2_agent.py` is prohibited because that program owns an interactive whole-task loop rather than an atomic-operation API.

## Safety model

Natural-language output is never treated as a robot command. The planner emits a TaskIR containing registered skill identifiers and constraints. The deterministic executor resolves every identifier again, verifies its atomic operation, executes serially, and stops at the first failure.

The RobotRAM bridge provider enforces explicit hardware enablement, loopback-only transport, cancellation, timeout, and response validation. The bridge server itself remains deferred and must enforce feedback verification and emergency-stop behavior before hardware use.

## Known Limitations and Deferred Work

Object extraction in deterministic mode uses a small vocabulary rather than perception output. The UI is process-local and task history is not durable. Python Agent polling and cancellation are cooperative and do not replace safety-rated controls. The RobotRAM atomic bridge server remains disabled until feedback verification, approval, and emergency-stop behavior are implemented.
