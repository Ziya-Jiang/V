# 人工审批机器人

[English](robot.md) | 中文

机器人接口是一个被刻意收窄的能力边界：Service Definition [`@deepseek-ai/dsh-robot`](../../packages/robot/robot) 拥有 `ctx.robot`；loopback Python 提供方 [`@deepseek-ai/dsh-robot-python-agent`](../../packages/robot/robot-python-agent) 负责协议校验；[`@deepseek-ai/dsh-tool-robot`](../../packages/robot/tool-robot) 负责模型可见工具。现有 Python Agent 仍是唯一可以拥有相机、机械臂、感知模型和 RobotRAM 集成的进程。

默认仍是带 `ROBOT_AGENT_PLAN_ONLY=1` 的严格仅规划模式。带审批模式是独立的显式选项：模型可以为一个不可变 Plan 创建不会产生动作的执行请求，但不会获得确认口令或审批方法。Robot Soul DSH 独立展示并校验 Plan；只有现场人员可以批准。随后 Python Agent 创建 Task，并始终保持唯一硬件所有者。

## 状态与 manifest

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

即使安全检查失败，`status()` 仍会报告远端健康状态。默认模式要求 `planOnlyReady` 为 true；显式审批模式中的 `executionReady` 要求生产 Agent 已完成 bootstrap、声明审批请求协议且没有活动 Task。

## 规划

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

模型只提供语义指令。提供方拥有绝对输出目录，并限制指令、超时和响应大小。`planId` 是远端内存状态；只有模型创建执行请求且现场人员独立批准该精确 Plan 后，它才能变成 Task。

## 逐 Plan 人工审批

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

`requestExecution()` 只能创建等待审批记录；`executionStatus()` 只能读取同一提供方实例创建的请求。两者都不能审批、确认、直接执行、调用原子 Tool 或发送低层运动。

## 面向模型的接口

- `robot_status` 读取就绪状态。
- `robot_manifest` 读取已注册的 Tool 与 Skill 描述。
- `robot_plan` 创建预览。
- `robot_request_execution` 可选地创建逐 Plan 人工审批请求。
- `robot_execution_status` 读取审批与 Task 状态。

只有提供方和消费方都配置 `approvalGatedExecution: true` 时，两个可选工具才会注册。本包不暴露直接 execute、approve、确认口令、任意 RPC、shell、关节、力、速度、原子 Tool 或直接运动权限。

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
