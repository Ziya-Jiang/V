# @deepseek-ai/dsh-tool-robot

English | [中文](README.zh.md)

By default this consumer registers three model-facing tools over `ctx.robot`:

- `robot_status`: verify process-level plan-only readiness.
- `robot_manifest`: inspect advertised Robot Tools and Skills.
- `robot_plan`: create a semantic server-side plan preview.

With `approvalGatedExecution: true`, it additionally registers:

- `robot_request_execution`: ask an on-site human to approve one exact Plan.
- `robot_execution_status`: read authoritative approval and Task state.

It still registers no direct execute, approval, confirmation, arbitrary RPC,
shell, atomic Tool, or low-level motion tool. The model can request but cannot
approve hardware action.

## Model Experience

The model receives structured status, manifest, Plan, approval-request, and Task
summary values. `robot_plan` accepts semantic instructions only; after
`robot_request_execution`, it must tell the user that a separate human approval
is pending and use `robot_execution_status` rather than claiming success.

#### KV Cache effect

Loading or unloading the package changes the tool schemas and one named prompt
section, so the next model request receives a different prefix.

## Known Limitations and Deferred Work

- Results currently use generic tool cards and the Plan is not projected into
  a durable Robot Session event. Rich task-event streaming, scene-revision
  binding, durable approval recovery, and model-initiated cancellation are
  deferred.
