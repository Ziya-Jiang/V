# Robot Soul DSH Interactive Introduction

English | [中文](README.zh.md)

A single-page unified GUI for non-technical users and developers. It combines
the complete DeepSeek Harness AI conversation, consumable Skill products,
Plans, an architecture explorer, and an isolated hardware approval surface.

## Start

Start the local operator console from the `dual_v2_2_agent` root:

```bash
./launch_robot_soul.sh start
```

This does not initialize the real robot. Open
<http://127.0.0.1:3091/#playground>, select **Real robot**, complete the on-site
confirmation, and click the startup button. Harness stays isolated on
`127.0.0.1:3081` and is embedded through an iframe. Use
`./launch_robot_soul.sh status|stop` for lifecycle management. To bypass the
page and explicitly start the complete hardware stack, run
`./launch_robot_soul.sh start-all`.

For simulation-only development, run from the `deepseek-harness` root:

```bash
./node_modules/.bin/tsx apps/robot-soul-dsh/src/index.ts
```

## Experience boundary

- The default experience is fully simulated. `POST /api/simulate/plan` only
  creates a non-executable presentation Plan.
- Optional Live mode reads only health/status and `manifest` from a local
  plan-only or explicitly enabled production Python Agent.
- Hardware-write APIs are locked by default. Only when both server-side gates
  are open can the isolated approval surface use `plan`, `execute`,
  `task_status`, `task_events`, and `cancel`.
- The website never exposes `call`, atomic Tools, CAN, joint, or shell control.
- If Live disconnects, the full static catalogue and browser simulation remain
  available.
- The token and `outputDir` stay on the server and are never returned to the
  browser.
- Normal web requests never start the control stack. In unified-launcher mode,
  only the same-origin loopback `/api/ops/recover` route, the exact on-site
  phrase, and a recoverable supervisor fault together let the operator button
  ask the launcher to reinitialize the hardware plane. AI has no such route.
  The production Python Agent remains the sole hardware owner.

The Live read-only connection defaults to `127.0.0.1:15555`. These server-only
environment variables configure it:

- `ROBOT_SOUL_AGENT_HOST`: must be a numeric loopback address.
- `ROBOT_SOUL_AGENT_PORT`: Agent TCP port.
- `ROBOT_SOUL_HARDWARE_READONLY`: set to `1` to select the production Agent
  read-only gateway; any other value than `0` or `1` is rejected.
- `ROBOT_SOUL_APPROVED_EXECUTION`: stage-two server gate; it must equal
  `ENABLE_DUAL_V2_2_HARDWARE` exactly.
- `ROBOT_SOUL_AGENT_TOKEN`: optional shared token, never sent to the browser.
- `ROBOT_SOUL_AGENT_TIMEOUT_MS`: Provider request timeout.
- `ROBOT_SOUL_AGENT_MAX_RESPONSE_BYTES`: response size limit.
- `ROBOT_SOUL_OUTPUT_DIR`: Agent task output directory, never exposed to the
  page.
- `ROBOT_SOUL_HARDWARE_PLAN_TTL_MS`: website approval-Plan lifetime; ten
  minutes by default.
- `ROBOT_SOUL_HARNESS_HOST` / `ROBOT_SOUL_HARNESS_PORT`: embedded Harness Web
  endpoint. Standalone site startup defaults to `127.0.0.1:3080`; the unified
  launcher uses `3081`. Only numeric loopback is accepted.

## Staged hardware access

Stage-one read-only access remains opt-in:

```bash
ROBOT_SOUL_HARDWARE_READONLY=1 \
  ./node_modules/.bin/tsx apps/robot-soul-dsh/src/index.ts
```

This expects an operator-managed Python Agent on loopback. The website receives
only a `health()` and `manifest()` facade, and `/api/hardware/policy` reports
`executeEnabled: false`.

Stage two requires a production Agent that has completed hardware bootstrap and
reports `ready`. An on-site operator must explicitly open both server gates:

```bash
ROBOT_SOUL_HARDWARE_READONLY=1 \
ROBOT_SOUL_APPROVED_EXECUTION=ENABLE_DUAL_V2_2_HARDWARE \
ROBOT_SOUL_OUTPUT_DIR=/absolute/path/to/task-output \
  ./node_modules/.bin/tsx apps/robot-soul-dsh/src/index.ts
```

Even then, every motion requires all of these conditions:

1. Only semantic goals are accepted; joint, speed, CAN, shell, and other
   low-level commands are rejected.
2. The Agent must be non-plan-only, report `bootstrapState=ready`, and have no
   active task.
3. The Plan may contain only `perceive_scene`, `pick_and_place`, `tidy_table`,
   and `safety_reset`. `start_robot` and all atomic Tools are rejected.
4. Skill arguments must pass strict key, offset, 12-step, and eight-motion-unit
   limits.
5. The Plan must have been issued by this website process, remain unexpired,
   and only one Task may be active.
6. The operator must confirm presence, a clear workspace, a reachable physical
   emergency stop, and robot inspection, then enter
   `ENABLE_DUAL_V2_2_HARDWARE` exactly.
7. Hardware POST routes accept only same-origin loopback `application/json`.

`cancel` is cooperative cancellation, not an emergency stop. Use the
independent physical emergency stop for unexpected motion.

## Faults and operator recovery

One CAN command/state exchange failure still stops and locks the hardware plane
while moving or whenever state is uncertain. Only a confirmed stationary
position hold tolerates at most two consecutive transient failures; the third
failure locks it. A hardware-plane fault no longer terminates Harness or this
page. The status area shows the fault, and an on-site operator can select
**Reinitialize robot** after checking the hardware and emergency stop. Recovery
restarts only the control stack and Python Agent, never Harness, and is not
available to the model. If the cached `sudo` authorization has expired, run
`sudo -v` in a terminal before retrying.

When DeepSeek Harness uses `examples/robot-approved/cordis.yml`, the model can
only create an `awaiting_approval` record through `robot_request_execution`.
The site's AI approval queue reads that record and displays the original
server-side Plan. The confirmation phrase is sent only to the Python Agent and
never enters Harness context. After approval, the Python Agent creates the Task
and the model reads authoritative state through `robot_execution_status`.

## Difference from Robot Assistant

`apps/robot-assistant` is the legacy independent task UI. Robot Soul DSH is the
new unified entry: `launch_robot_soul.sh` starts the isolated processes, while
one page carries Harness chat, human approval, and Task events. The page still
owns no hardware and exposes no arbitrary call or atomic Tool execution.

## API

Read-only:

- `GET /api/content`
- `GET /api/catalog`
- `GET /api/architecture`
- `GET /api/hardware/policy`
- `GET /api/live/status`
- `GET /api/live/manifest`
- `GET /api/ops/status`, aggregating Harness, Python Agent, policy, and fault
  state

Local simulation:

- `POST /api/simulate/plan` with
  `{ "instruction": "semantic robot goal" }`

Stage-two approval execution (locked with 403 by default):

- `POST /api/hardware/plan`
- `POST /api/hardware/execute`
- `GET /api/hardware/approvals`
- `POST /api/hardware/approvals/approve`
- `POST /api/hardware/approvals/deny`
- `GET /api/hardware/task/status?taskId=...`
- `GET /api/hardware/task/events?taskId=...&cursor=...`
- `POST /api/hardware/task/cancel`

On-site operator recovery only:

- `POST /api/ops/recover`, requiring same-origin loopback JSON, the exact
  confirmation phrase, and a recoverable supervisor fault; it is not registered
  as a Harness Tool

## Verify

```bash
./node_modules/.bin/tsc -p apps/robot-soul-dsh/tsconfig.json --noEmit
./node_modules/.bin/vitest run apps/robot-soul-dsh/tests
```

Interactive acceptance covers desktop and mobile layouts, keyboard card
navigation, closing dialogs with Escape, light and dark themes,
`prefers-reduced-motion`, simulation fallback, architecture nodes, protocol
rejection, and the safety decision game.
