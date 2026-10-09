/**
 * Service Definition for the robot capability seam (`ctx.robot`).
 * The default remains plan-only. The optional execution API creates inert
 * per-Plan approval requests; it never carries human confirmation or low-level
 * hardware operations.
 * @module @deepseek-ai/dsh-robot
 */

import { Context, Service } from '@deepseek-ai/cordis'

/** One recursively JSON-serializable value crossing the robot process boundary. */
export type RobotJsonValue =
  | null
  | boolean
  | number
  | string
  | RobotJsonValue[]
  | { [key: string]: RobotJsonValue }

/** A JSON object returned by the robot provider. */
export type RobotJsonObject = { [key: string]: RobotJsonValue }

/** Operating mode exposed by the selected Robot provider. */
export type RobotMode = 'dry-run' | 'approval-gated'

/** Provider status and proof of the configured safety mode's readiness. */
export interface RobotStatus {
  readonly mode: RobotMode
  readonly provider: string
  readonly planOnlyReady: boolean
  readonly executionReady: boolean
  readonly reason?: string
  readonly health: RobotJsonObject
}

/** Tool and Skill capabilities advertised by the Python Robot Agent. */
export interface RobotManifest {
  readonly mode: RobotMode
  readonly provider: string
  readonly tools: RobotJsonValue[]
  readonly skills: RobotJsonValue[]
}

/** Input to one hardware-free semantic planning request. */
export interface RobotPlanRequest {
  readonly instruction: string
}

/** Immutable server-side plan preview returned to the Harness. */
export interface RobotPlan {
  readonly mode: RobotMode
  readonly provider: string
  readonly planId: string
  readonly instruction: string
  readonly outputDir: string | null
  readonly createdAt?: string
  readonly plan: RobotJsonObject
}

/** Per-Plan execution request created by the model and awaiting a human. */
export interface RobotExecutionRequest {
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

/** Input for one model-initiated, human-approved execution request. */
export interface RobotExecutionRequestInput {
  readonly planId: string
}

/** Input for reading a previously created execution request. */
export interface RobotExecutionStatusInput {
  readonly requestId: string
}

/** Stable error categories shared by providers and consumers. */
export type RobotErrorCode =
  | 'ROBOT_CONFIGURATION'
  | 'ROBOT_CONNECTION'
  | 'ROBOT_TIMEOUT'
  | 'ROBOT_ABORTED'
  | 'ROBOT_RESPONSE_TOO_LARGE'
  | 'ROBOT_PROTOCOL'
  | 'ROBOT_REMOTE'
  | 'ROBOT_PLAN_ONLY_REQUIRED'
  | 'ROBOT_EXECUTION_NOT_ENABLED'

/** Robot capability failure with a routable category. */
export class RobotError extends Error {
  readonly code: RobotErrorCode

  constructor(code: RobotErrorCode, message: string, options?: ErrorOptions) {
    super(message, options)
    this.name = 'RobotError'
    this.code = code
  }
}

declare module '@deepseek-ai/cordis' {
  interface Context {
    robot: RobotRuntime
  }
}

/**
 * Robot service with plan-only defaults and separate approval-request methods.
 * Concrete providers own transport and process validation; model-facing names
 * and schemas belong to a separate consumer.
 */
export abstract class RobotRuntime extends Service {
  constructor(ctx: Context) {
    super(ctx, 'robot')
  }

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
  abstract plan(
    request: RobotPlanRequest,
    signal?: AbortSignal,
  ): Promise<RobotPlan>

  /**
   * Ask the hardware owner to hold a Plan for per-task human approval.
   * This call never contains the human confirmation and cannot move hardware
   * before a separate operator surface approves the exact server-side Plan.
   * @param request - Identifier returned by {@link plan}.
   * @param signal - Optional caller cancellation forwarded to the provider.
   * @returns Pending execution request visible to the operator approval UI.
   */
  requestExecution(
    request: RobotExecutionRequestInput,
    signal?: AbortSignal,
  ): Promise<RobotExecutionRequest> {
    void request
    void signal
    return Promise.reject(new RobotError(
      'ROBOT_EXECUTION_NOT_ENABLED',
      'Robot provider does not enable approval-gated execution',
    ))
  }

  /**
   * Read approval and Task state for a model-created execution request.
   * @param request - Request identifier returned by {@link requestExecution}.
   * @param signal - Optional caller cancellation forwarded to the provider.
   * @returns Approval state and Task summary when execution has started.
   */
  executionStatus(
    request: RobotExecutionStatusInput,
    signal?: AbortSignal,
  ): Promise<RobotExecutionRequest> {
    void request
    void signal
    return Promise.reject(new RobotError(
      'ROBOT_EXECUTION_NOT_ENABLED',
      'Robot provider does not enable execution request status',
    ))
  }
}

export default RobotRuntime
