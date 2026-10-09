/**
 * Service Definition for the plan-only robot capability seam (`ctx.robot`).
 * The seam deliberately omits execute/cancel operations: this first integration
 * can inspect capabilities, create server-side plan previews, and observe
 * provider status without exposing a hardware action path.
 * @module @deepseek-ai/dsh-robot
 */
import { Context, Service } from '@deepseek-ai/cordis';
/** One recursively JSON-serializable value crossing the robot process boundary. */
export type RobotJsonValue = null | boolean | number | string | RobotJsonValue[] | {
    [key: string]: RobotJsonValue;
};
/** A JSON object returned by the robot provider. */
export type RobotJsonObject = {
    [key: string]: RobotJsonValue;
};
/** Operating mode exposed by the selected Robot provider. */
export type RobotMode = 'dry-run' | 'approval-gated';
/** Provider status and proof that its process-level plan-only guard is active. */
export interface RobotStatus {
    readonly mode: RobotMode;
    readonly provider: string;
    readonly planOnlyReady: boolean;
    readonly executionReady: boolean;
    readonly reason?: string;
    readonly health: RobotJsonObject;
}
/** Tool and Skill capabilities advertised by the Python Robot Agent. */
export interface RobotManifest {
    readonly mode: RobotMode;
    readonly provider: string;
    readonly tools: RobotJsonValue[];
    readonly skills: RobotJsonValue[];
}
/** Input to one hardware-free semantic planning request. */
export interface RobotPlanRequest {
    readonly instruction: string;
}
/** Immutable server-side plan preview returned to the Harness. */
export interface RobotPlan {
    readonly mode: RobotMode;
    readonly provider: string;
    readonly planId: string;
    readonly instruction: string;
    readonly outputDir: string | null;
    readonly createdAt?: string;
    readonly plan: RobotJsonObject;
}
/** Per-Plan execution request created by the model and awaiting a human. */
export interface RobotExecutionRequest {
    readonly mode: 'approval-gated';
    readonly provider: string;
    readonly requestId: string;
    readonly planId: string;
    readonly status: string;
    readonly instruction: string;
    readonly createdAt: string;
    readonly expiresInSeconds: number;
    readonly taskId?: string;
    readonly plan: RobotJsonObject;
    readonly task?: RobotJsonObject;
}
/** Input for one model-initiated, human-approved execution request. */
export interface RobotExecutionRequestInput {
    readonly planId: string;
}
/** Input for reading a previously created execution request. */
export interface RobotExecutionStatusInput {
    readonly requestId: string;
}
/** Stable error categories shared by providers and consumers. */
export type RobotErrorCode = 'ROBOT_CONFIGURATION' | 'ROBOT_CONNECTION' | 'ROBOT_TIMEOUT' | 'ROBOT_ABORTED' | 'ROBOT_RESPONSE_TOO_LARGE' | 'ROBOT_PROTOCOL' | 'ROBOT_REMOTE' | 'ROBOT_PLAN_ONLY_REQUIRED' | 'ROBOT_EXECUTION_NOT_ENABLED';
/** Robot capability failure with a routable category. */
export declare class RobotError extends Error {
    readonly code: RobotErrorCode;
    constructor(code: RobotErrorCode, message: string, options?: ErrorOptions);
}
declare module '@deepseek-ai/cordis' {
    interface Context {
        robot: RobotRuntime;
    }
}
/**
 * Plan-only robot service. Concrete providers own transport and process
 * validation; model-facing names and schemas belong to a separate consumer.
 */
export declare abstract class RobotRuntime extends Service {
    constructor(ctx: Context);
    /**
     * Read provider health and whether its plan-only safety guard is active.
     * @param signal - Optional caller cancellation forwarded to the provider.
     * @returns Provider health and plan-only readiness evidence.
     */
    abstract status(signal?: AbortSignal): Promise<RobotStatus>;
    /**
     * Read the currently registered Robot Tools and Skills.
     * @param signal - Optional caller cancellation forwarded to the provider.
     * @returns The provider's advertised hardware-free capability manifest.
     */
    abstract manifest(signal?: AbortSignal): Promise<RobotManifest>;
    /**
     * Create a hardware-free, server-side plan preview.
     * @param request - Semantic task instruction to plan.
     * @param signal - Optional caller cancellation forwarded to the provider.
     * @returns An immutable plan identifier and preview body.
     */
    abstract plan(request: RobotPlanRequest, signal?: AbortSignal): Promise<RobotPlan>;
    /**
     * Ask the hardware owner to hold a Plan for per-task human approval.
     * This call never contains the human confirmation and cannot move hardware
     * before a separate operator surface approves the exact server-side Plan.
     * @param request - Identifier returned by {@link plan}.
     * @param signal - Optional caller cancellation forwarded to the provider.
     * @returns Pending execution request visible to the operator approval UI.
     */
    requestExecution(_request: RobotExecutionRequestInput, _signal?: AbortSignal): Promise<RobotExecutionRequest>;
    /**
     * Read approval and Task state for a model-created execution request.
     * @param request - Request identifier returned by {@link requestExecution}.
     * @param signal - Optional caller cancellation forwarded to the provider.
     * @returns Approval state and Task summary when execution has started.
     */
    executionStatus(_request: RobotExecutionStatusInput, _signal?: AbortSignal): Promise<RobotExecutionRequest>;
}
export default RobotRuntime;
//# sourceMappingURL=index.d.ts.map