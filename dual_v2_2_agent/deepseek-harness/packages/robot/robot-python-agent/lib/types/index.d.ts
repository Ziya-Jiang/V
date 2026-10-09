/**
 * Loopback NDJSON Service Provider for the plan-only robot seam.
 * It accepts only Python Agents that advertise process-level plan-only mode;
 * no execute, task, cancel, legacy call, or arbitrary RPC method exists on
 * this provider's public or private API.
 * @module @deepseek-ai/dsh-robot-python-agent
 */
import { Context } from '@deepseek-ai/cordis';
import z from '@deepseek-ai/schemastery';
import { RobotRuntime, type RobotManifest, type RobotPlan, type RobotPlanRequest, type RobotStatus } from '@deepseek-ai/dsh-robot';
/** Connection and input limits for the plan-only Python Agent provider. */
export interface Config {
    /** Numeric loopback address of the Python Agent. */
    readonly host?: string;
    /** TCP port of the Python Agent. */
    readonly port?: number;
    /** Optional shared token included in every NDJSON request. */
    readonly token?: string;
    /** Per-request connection and response deadline in milliseconds. */
    readonly timeoutMs?: number;
    /** Maximum accepted NDJSON response size in bytes. */
    readonly maxResponseBytes?: number;
    /** Maximum semantic planning instruction size in UTF-8 bytes. */
    readonly maxInstructionBytes?: number;
    /** Fixed absolute scene/output directory sent with every planning request. */
    readonly outputDir?: string;
}
/** Schemastery configuration for the Python Agent provider. */
export declare const Config: z<Config>;
/** Plan-only RobotRuntime implementation over the existing Python Agent. */
export declare class PythonAgentRobotRuntime extends RobotRuntime {
    #private;
    static Config: z<Config>;
    constructor(ctx: Context, config?: Config);
    status(signal?: AbortSignal): Promise<RobotStatus>;
    manifest(signal?: AbortSignal): Promise<RobotManifest>;
    plan(request: RobotPlanRequest, signal?: AbortSignal): Promise<RobotPlan>;
}
export default PythonAgentRobotRuntime;
//# sourceMappingURL=index.d.ts.map