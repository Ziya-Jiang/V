# Human-approved Robot

English | [中文](robot.zh.md)

The robot seam is a deliberately narrow capability boundary: Service
Definition [`@deepseek-ai/dsh-robot`](../../packages/robot/robot) owns
`ctx.robot`, the loopback Python provider
[`@deepseek-ai/dsh-robot-python-agent`](../../packages/robot/robot-python-agent)
owns protocol validation, and
[`@deepseek-ai/dsh-tool-robot`](../../packages/robot/tool-robot) owns the
model-visible tools. The existing Python Agent remains the only process that
can own cameras, arms, perception models, and RobotRAM integration.

The default remains strict plan-only mode with `ROBOT_AGENT_PLAN_ONLY=1`.
Approval-gated mode is a separate opt-in: the model may create an inert
execution request for one immutable Plan, but it receives no confirmation or
approval method. Robot Soul DSH independently displays and validates the Plan;
only an on-site human can approve it. The Python Agent then creates the Task and
remains the sole hardware owner.

## Status and manifest

```ts type-equiv
/** Provider status and proof that its process-level plan-only guard is active. */
interface RobotStatus {
  readonly mode: RobotMode
  readonly provider: string
  readonly planOnlyReady: boolean
  readonly executionReady: boolean
  readonly reason?: string
  readonly health: RobotJsonObject
}
```

```ts type-equiv
/** Tool and Skill capabilities advertised by the Python Robot Agent. */
interface RobotManifest {
  readonly mode: RobotMode
  readonly provider: string
  readonly tools: RobotJsonValue[]
  readonly skills: RobotJsonValue[]
}
```

`status()` reports remote health even when the safety check fails. In default
mode `planOnlyReady` must be true; in explicit approval mode
`executionReady` requires a production Agent with completed bootstrap, the
approval-request protocol, and no active Task.

## Planning

```ts type-equiv
/** Input to one hardware-free semantic planning request. */
interface RobotPlanRequest {
  readonly instruction: string
}
```

```ts type-equiv
/** Immutable server-side plan preview returned to the Harness. */
interface RobotPlan {
  readonly mode: RobotMode
  readonly provider: string
  readonly planId: string
  readonly instruction: string
  readonly outputDir: string | null
  readonly createdAt?: string
  readonly plan: RobotJsonObject
}
```

The model supplies only a semantic instruction. The provider owns the absolute
output directory and bounds the instruction, timeout, and response size.
`planId` is remote in-memory state. It remains inert until the model creates an
execution request and an on-site human separately approves that exact Plan.

## Per-Plan human approval

```ts type-equiv
/** Per-Plan execution request created by the model and awaiting a human. */
interface RobotExecutionRequest {
  readonly mode: 'approval-gated'
  readonly provider: string
  readonly requestId: string
  readonly planId: string
  readonly status: string
  readonly instruction: string
  readonly createdAt: string
  readonly expiresInSeconds: number
  readonly taskId?: string
  readonly plan: RobotJsonObject
  readonly task?: RobotJsonObject
}
```

`requestExecution()` can only create an awaiting-approval record.
`executionStatus()` can only read requests created by the same provider
instance. Neither method can approve, confirm, directly execute, call an atomic
Tool, or issue low-level motion.

## Model-facing surface

- `robot_status` reads readiness.
- `robot_manifest` reads registered Tool and Skill descriptions.
- `robot_plan` creates a preview.
- `robot_request_execution` optionally creates a per-Plan human approval
  request.
- `robot_execution_status` reads approval and Task state.

The optional tools are registered only when `approvalGatedExecution: true` is
configured in both provider and consumer. The package exposes no direct
execute, approve, confirmation, arbitrary RPC, shell, joint, force, speed,
atomic Tool, or direct motion authority.

<!-- BEGIN GENERATED cordis-surface (gen-cordis-catalog.ts) — do not edit between markers -->

<a id="cordis-surface"></a>

## Cordis API

Generated from source by `scripts/gen-cordis-catalog.ts` (verified fresh by `pnpm run verify-cordis-catalog` in doc-sync; regenerate with `pnpm run gen-cordis-catalog`) — this section is byte-identical in both language sides of the page. Signature blocks use a `ts cordis-catalog` fence and keep the original source JSDoc; dispatch modes are defined in the [primer](../cordis-primer.md#dispatch-modes), and the framework-inherited `ctx` API lives in [cordis-api/inherited.md](../cordis-api/inherited.md).

<a id="ctxrobot--robotruntime-abstract-seam"></a>

### `ctx.robot` — `RobotRuntime` (abstract seam)

Robot service with plan-only defaults and separate approval-request methods. Concrete providers own transport and process validation; model-facing names and schemas belong to a separate consumer.

```ts cordis-catalog
/**
 * Read provider health and configured safety-mode readiness.
 * @param signal - Optional caller cancellation forwarded to the provider.
 * @returns Provider health plus plan-only and execution readiness evidence.
 */
abstract status(signal?: AbortSignal): Promise<RobotStatus>

/**
 * Read the currently registered Robot Tools and Skills.
 * @param signal - Optional caller cancellation forwarded to the provider.
 * @returns The provider's advertised hardware-free capability manifest.
 */
abstract manifest(signal?: AbortSignal): Promise<RobotManifest>

/**
 * Create a hardware-free, server-side plan preview.
 * @param request - Semantic task instruction to plan.
 * @param signal - Optional caller cancellation forwarded to the provider.
 * @returns An immutable plan identifier and preview body.
 */
abstract plan( request: RobotPlanRequest, signal?: AbortSignal, ): Promise<RobotPlan>

/**
 * Ask the hardware owner to hold a Plan for per-task human approval.
 * This call never contains the human confirmation and cannot move hardware
 * before a separate operator surface approves the exact server-side Plan.
 * @param request - Identifier returned by {@link plan}.
 * @param signal - Optional caller cancellation forwarded to the provider.
 * @returns Pending execution request visible to the operator approval UI.
 */
requestExecution( request: RobotExecutionRequestInput, signal?: AbortSignal, ): Promise<RobotExecutionRequest>

/**
 * Read approval and Task state for a model-created execution request.
 * @param request - Request identifier returned by {@link requestExecution}.
 * @param signal - Optional caller cancellation forwarded to the provider.
 * @returns Approval state and Task summary when execution has started.
 */
executionStatus( request: RobotExecutionStatusInput, signal?: AbortSignal, ): Promise<RobotExecutionRequest>
```

Source: [`packages/robot/robot/src/index.ts:119`](../../packages/robot/robot/src/index.ts)
<!-- END GENERATED cordis-surface -->
