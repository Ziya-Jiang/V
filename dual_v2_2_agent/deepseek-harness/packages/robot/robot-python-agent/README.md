# @deepseek-ai/dsh-robot-python-agent

English | [中文](README.zh.md)

This package provides `ctx.robot` over the existing loopback NDJSON Python
Robot Agent. It defaults to plan-only:

- The transport code can send only `health`, `manifest`, and `plan`.
- `status`, `manifest`, and `plan` all preserve caller cancellation and apply
  response-size and timeout limits.
- `manifest` and `plan` first require health to advertise the exact plan-only
  method set, `planOnly: true`, no eager/bootstrap activity, and no active task.
- The output directory is provider configuration; the model cannot choose an
  arbitrary filesystem path.
- The endpoint must be a numeric loopback address.

Start the Python Agent with `ROBOT_AGENT_PLAN_ONLY=1`. That server mode disables
hardware bootstrap and rejects every execution-plane method before dispatch.

Setting `approvalGatedExecution: true` is a separate opt-in mode for a
production Agent that already reports hardware `ready`. It adds only
`request_execution` and `execution_request_status` to the Harness transport.
The provider has no `approve_execution`, confirmation, direct execute, or
arbitrary-call method. A model-created request stays inert until the operator
approves that exact Plan through Robot Soul DSH.

## Configuration

- `host` (default `127.0.0.1`)
- `port` (default `15555`)
- `token` (optional shared token)
- `timeoutMs` (default `30000`)
- `maxResponseBytes` (default 1 MiB)
- `maxInstructionBytes` (default 16 KiB)
- `outputDir` (default current directory; must be absolute)
- `approvalGatedExecution` (default `false`)

## Model Experience

Indirectly through `@deepseek-ai/dsh-tool-robot`. Provider failures retain
stable `RobotError.code` categories for configuration, connection, timeout,
cancellation, protocol, remote, and plan-only-policy failures.

#### KV Cache effect

No direct invalidation; this provider contributes no prompt text.

## Known Limitations and Deferred Work

- The provider targets the repository-specific one-request-per-connection
  NDJSON protocol and performs health validation before mode-sensitive calls.
  Reconnection pooling, protocol version negotiation, durable Plan/approval
  recovery, and multi-robot routing are deferred.
