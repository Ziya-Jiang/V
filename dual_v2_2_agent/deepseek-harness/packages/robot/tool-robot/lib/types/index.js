/**
 * Model-facing read/plan tools over the plan-only robot seam.
 * No execute, task, cancel, confirmation, or arbitrary-operation tool is
 * registered by this package.
 * @module @deepseek-ai/dsh-tool-robot
 */
import z from '@deepseek-ai/schemastery';
import { defineTool } from '@deepseek-ai/dsh-tools';
/** Schemastery configuration for the plan-only robot tools. */
export const Config = z.object({
    readTimeoutMs: z.number().default(30_000),
    planTimeoutMs: z.number().default(60_000),
});
/** Cordis plugin name used by loader diagnostics. */
export const name = 'tool-robot';
/** Services required by the plan-only robot tool suite. */
export const inject = ['tools', 'robot', 'systemPrompt'];
function positiveInteger(name, value) {
    if (!Number.isSafeInteger(value) || value < 1) {
        throw new Error(`tool-robot: ${name} must be a positive safe integer`);
    }
    return value;
}
function renderJson(label, value) {
    return [{
            type: 'text',
            text: `${label}\n${JSON.stringify(value, null, 2)}`,
        }];
}
/** Register status, manifest, and planning tools without an execution surface. */
export function apply(ctx, config = {}) {
    const readTimeoutMs = positiveInteger('readTimeoutMs', config.readTimeoutMs ?? 30_000);
    const planTimeoutMs = positiveInteger('planTimeoutMs', config.planTimeoutMs ?? 60_000);
    ctx.systemPrompt.section({
        name: 'tool:robot-plan-only',
        order: 119,
        text: 'Robot access is PLAN-ONLY and DRY-RUN. Use robot_status to verify the isolated Python Agent, robot_manifest to inspect registered capabilities, and robot_plan to create a preview. These tools cannot execute, cancel, or control hardware.',
    });
    ctx.tools.register(defineTool({
        name: 'robot_status',
        description: 'Read Python Robot Agent health and verify that its process-level plan-only guard is active. This never controls hardware.',
        parameters: {},
        output: {
            schema: {
                type: 'object',
                additionalProperties: false,
                properties: {
                    mode: { type: 'string', required: true, const: 'dry-run' },
                    provider: { type: 'string', required: true },
                    planOnlyReady: { type: 'boolean', required: true },
                    reason: { type: 'string' },
                    health: {
                        type: 'object',
                        required: true,
                        additionalProperties: true,
                    },
                },
            },
            render: (_args, value) => renderJson('Robot plan-only status:', value),
        },
        timeoutMs: readTimeoutMs,
        isConcurrencySafe: () => true,
        execute: (_args, exec) => ctx.robot.status(exec.signal),
        presentCall: () => ({
            card: 'generic',
            title: 'Check robot plan-only status',
            kind: 'other',
        }),
    }));
    ctx.tools.register(defineTool({
        name: 'robot_manifest',
        description: 'List Robot Tools and Skills advertised by the isolated plan-only Python Agent. This never controls hardware.',
        parameters: {},
        output: {
            schema: {
                type: 'object',
                additionalProperties: false,
                properties: {
                    mode: { type: 'string', required: true, const: 'dry-run' },
                    provider: { type: 'string', required: true },
                    tools: {
                        type: 'array',
                        required: true,
                        items: { type: 'json' },
                    },
                    skills: {
                        type: 'array',
                        required: true,
                        items: { type: 'json' },
                    },
                },
            },
            render: (_args, value) => renderJson('Robot capability manifest:', value),
        },
        timeoutMs: readTimeoutMs,
        isConcurrencySafe: () => true,
        execute: (_args, exec) => ctx.robot.manifest(exec.signal),
        presentCall: () => ({
            card: 'generic',
            title: 'Read robot capability manifest',
            kind: 'read',
        }),
    }));
    ctx.tools.register(defineTool({
        name: 'robot_plan',
        description: 'Create an immutable server-side robot plan PREVIEW from a semantic instruction. The result is dry-run only and cannot be executed by this Harness integration.',
        parameters: {
            instruction: {
                type: 'string',
                required: true,
                description: 'A semantic robot task such as "把杯子放到盘子旁边". Do not provide joint angles, forces, speeds, shell commands, or arbitrary motion coordinates.',
            },
        },
        output: {
            schema: {
                type: 'object',
                additionalProperties: false,
                properties: {
                    mode: { type: 'string', required: true, const: 'dry-run' },
                    provider: { type: 'string', required: true },
                    planId: { type: 'string', required: true },
                    instruction: { type: 'string', required: true },
                    outputDir: {
                        required: true,
                        oneOf: [
                            { type: 'string' },
                            { type: 'null' },
                        ],
                    },
                    createdAt: { type: 'string' },
                    plan: {
                        type: 'object',
                        required: true,
                        additionalProperties: true,
                    },
                },
            },
            render: (_args, value) => renderJson('Robot plan preview (DRY-RUN; no hardware action was exposed):', value),
        },
        timeoutMs: planTimeoutMs,
        execute: (args, exec) => ctx.robot.plan({ instruction: args.instruction }, exec.signal),
        presentCall: args => ({
            card: 'generic',
            title: 'Create robot plan preview',
            kind: 'other',
            rawInput: { instruction: args.instruction },
        }),
    }));
}
//# sourceMappingURL=index.js.map