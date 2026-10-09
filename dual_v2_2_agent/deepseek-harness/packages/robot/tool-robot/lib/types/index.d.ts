/**
 * Model-facing read/plan tools over the plan-only robot seam.
 * No execute, task, cancel, confirmation, or arbitrary-operation tool is
 * registered by this package.
 * @module @deepseek-ai/dsh-tool-robot
 */
import type { Context } from '@deepseek-ai/cordis';
import z from '@deepseek-ai/schemastery';
/** Cooperative timeout budgets for plan-only robot tool calls. */
export interface Config {
    /** Timeout in milliseconds for status and manifest reads. */
    readonly readTimeoutMs?: number;
    /** Timeout in milliseconds for semantic plan preview creation. */
    readonly planTimeoutMs?: number;
}
/** Schemastery configuration for the plan-only robot tools. */
export declare const Config: z<Config>;
/** Cordis plugin name used by loader diagnostics. */
export declare const name = "tool-robot";
/** Services required by the plan-only robot tool suite. */
export declare const inject: string[];
/** Register status, manifest, and planning tools without an execution surface. */
export declare function apply(ctx: Context, config?: Config): void;
//# sourceMappingURL=index.d.ts.map