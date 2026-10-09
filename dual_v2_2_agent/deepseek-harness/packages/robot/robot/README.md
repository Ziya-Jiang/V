# @deepseek-ai/dsh-robot

English | [中文](README.zh.md)

`RobotRuntime` defines the `ctx.robot` capability seam. It separates provider
transport from model-facing tools and defaults to plan-only:

- `status(signal?)`: read provider health and plan-only readiness.
- `manifest(signal?)`: read advertised Robot Tools and Skills.
- `plan({ instruction }, signal?)`: create an immutable server-side preview.
- `requestExecution({ planId }, signal?)`: create an inert per-Plan request
  that waits for a separate human approval surface.
- `executionStatus({ requestId }, signal?)`: read authoritative approval and
  Task state for a request created by the provider.

The contract still has no direct execute, approval, confirmation, arbitrary
RPC, joint-space, force, velocity, or shell operation. The model can request
execution, but it cannot approve its own request; only the hardware owner's
separate operator surface can turn that exact Plan into a Task.

## Model Experience

None directly. `@deepseek-ai/dsh-tool-robot` owns model-visible names, schemas,
instructions, results, and presentation.

#### KV Cache effect

No direct invalidation; this Service Definition contributes no prompt text.

## Known Limitations and Deferred Work

- The seam supports one provider and does not persist Robot Plans into the
  Harness Session log. Rich task-event streaming, scene revisions, multi-robot
  provider selection, and durable recovery of pending approvals are deferred.
