/**
 * Model-facing tools over the robot seam. Execution requests are opt-in and
 * never contain human confirmation; hardware remains inert until a separate
 * operator surface approves the exact server-side Plan.
 * @module @deepseek-ai/dsh-tool-robot
 */

import type { Context } from '@deepseek-ai/cordis'
import z from '@deepseek-ai/schemastery'
import type {} from '@deepseek-ai/dsh-robot'
import { defineTool } from '@deepseek-ai/dsh-tools'

/** Cooperative mode and timeout policy for robot tool calls. */
export interface Config {
  /** Timeout in milliseconds for status and manifest reads. */
  readonly readTimeoutMs?: number
  /** Timeout in milliseconds for semantic plan preview creation. */
  readonly planTimeoutMs?: number
  /** Register per-Plan approval-request and status tools. */
  readonly approvalGatedExecution?: boolean
  /** Timeout in milliseconds for approval-request operations. */
  readonly executionTimeoutMs?: number
}

/** Schemastery configuration for the robot model-tool surface. */
export const Config: z<Config> = z.object({
  readTimeoutMs: z.number().default(30_000),
  planTimeoutMs: z.number().default(60_000),
  approvalGatedExecution: z.boolean().default(false),
  executionTimeoutMs: z.number().default(30_000),
})

/** Cordis plugin name used by loader diagnostics. */
export const name = 'tool-robot'
/** Services required by the robot model-tool suite. */
export const inject = ['tools', 'robot', 'systemPrompt']

function positiveInteger(name: string, value: number): number {
  if (!Number.isSafeInteger(value) || value < 1) {
    throw new Error(`tool-robot: ${name} must be a positive safe integer`)
  }
  return value
}

function renderJson(label: string, value: unknown): {
  readonly type: 'text'
  readonly text: string
}[] {
  return [{
    type: 'text',
    text: `${label}\n${JSON.stringify(value, null, 2)}`,
  }]
}

/** Register base planning tools plus optional approval-request tools. */
export function apply(ctx: Context, config: Config = {}): void {
  const readTimeoutMs = positiveInteger(
    'readTimeoutMs',
    config.readTimeoutMs ?? 30_000,
  )
  const planTimeoutMs = positiveInteger(
    'planTimeoutMs',
    config.planTimeoutMs ?? 60_000,
  )
  const executionTimeoutMs = positiveInteger(
    'executionTimeoutMs',
    config.executionTimeoutMs ?? 30_000,
  )
  const approvalGatedExecution = config.approvalGatedExecution ?? false

  ctx.systemPrompt.section({
    name: 'tool:robot',
    order: 119,
    text: approvalGatedExecution
      ? 'Robot access is approval-gated. Use robot_plan with semantic goals only, inspect the complete Plan, then use robot_request_execution to ask an on-site human to approve that exact Plan. The request cannot move hardware by itself. Never claim approval or success; use robot_execution_status for authoritative state. Never request joints, speed, CAN, shell, atomic Tools, or start_robot.'
      : 'Robot access is PLAN-ONLY and DRY-RUN. Use robot_status to verify the isolated Python Agent, robot_manifest to inspect registered capabilities, and robot_plan to create a preview. These tools cannot execute, cancel, or control hardware.',
  })

  ctx.tools.register(defineTool({
    name: 'robot_status',
    description: approvalGatedExecution
      ? 'Read Python Robot Agent health and verify that its approval-gated execution boundary is ready. This never controls hardware.'
      : 'Read Python Robot Agent health and verify that its process-level plan-only guard is active. This never controls hardware.',
    parameters: {},
    output: {
      schema: {
        type: 'object',
        additionalProperties: false,
        properties: {
          mode: {
            required: true,
            oneOf: [
              { type: 'string', const: 'dry-run' },
              { type: 'string', const: 'approval-gated' },
            ],
          },
          provider: { type: 'string', required: true },
          planOnlyReady: { type: 'boolean', required: true },
          executionReady: { type: 'boolean', required: true },
          reason: { type: 'string' },
          health: {
            type: 'object',
            required: true,
            additionalProperties: true,
          },
        },
      },
      render: (_args, value) => renderJson('Robot safety status:', value),
    },
    timeoutMs: readTimeoutMs,
    isConcurrencySafe: () => true,
    execute: (_args, exec) => ctx.robot.status(exec.signal),
    presentCall: () => ({
      card: 'generic',
      title: 'Check robot safety status',
      kind: 'other',
    }),
  }))

  ctx.tools.register(defineTool({
    name: 'robot_manifest',
    description: approvalGatedExecution
      ? 'List Robot Tools and Skills advertised by the production Python Agent behind the approval boundary. This never controls hardware.'
      : 'List Robot Tools and Skills advertised by the isolated plan-only Python Agent. This never controls hardware.',
    parameters: {},
    output: {
      schema: {
        type: 'object',
        additionalProperties: false,
        properties: {
          mode: {
            required: true,
            oneOf: [
              { type: 'string', const: 'dry-run' },
              { type: 'string', const: 'approval-gated' },
            ],
          },
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
  }))

  ctx.tools.register(defineTool({
    name: 'robot_plan',
    description: 'Create an immutable server-side robot Plan from a semantic instruction. It remains inert unless the optional approval-request tool is enabled and an on-site human separately approves this exact Plan.',
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
          mode: {
            required: true,
            oneOf: [
              { type: 'string', const: 'dry-run' },
              { type: 'string', const: 'approval-gated' },
            ],
          },
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
      render: (_args, value) => renderJson(
        'Robot Plan (inert until separate per-Plan human approval):',
        value,
      ),
    },
    timeoutMs: planTimeoutMs,
    execute: (args, exec) => ctx.robot.plan(
      { instruction: args.instruction },
      exec.signal,
    ),
    presentCall: args => ({
      card: 'generic',
      title: 'Create robot plan preview',
      kind: 'other',
      rawInput: { instruction: args.instruction },
    }),
  }))

  if (!approvalGatedExecution) return

  ctx.tools.register(defineTool({
    name: 'robot_request_execution',
    description: 'Submit one existing server-side robot Plan for separate per-task human approval. This call does not contain approval credentials and cannot move hardware until an on-site operator approves the exact Plan.',
    parameters: {
      planId: {
        type: 'string',
        required: true,
        description: 'The immutable planId returned by robot_plan.',
      },
    },
    output: {
      schema: {
        type: 'object',
        additionalProperties: false,
        properties: {
          mode: {
            type: 'string',
            required: true,
            const: 'approval-gated',
          },
          provider: { type: 'string', required: true },
          requestId: { type: 'string', required: true },
          planId: { type: 'string', required: true },
          status: { type: 'string', required: true },
          instruction: { type: 'string', required: true },
          createdAt: { type: 'string', required: true },
          expiresInSeconds: { type: 'number', required: true },
          plan: {
            type: 'object',
            required: true,
            additionalProperties: true,
          },
          taskId: { type: 'string' },
          task: {
            type: 'object',
            additionalProperties: true,
          },
        },
      },
      render: (_args, value) => renderJson(
        'Robot execution request (awaiting independent human approval):',
        value,
      ),
    },
    timeoutMs: executionTimeoutMs,
    execute: (args, exec) => ctx.robot.requestExecution(
      { planId: args.planId },
      exec.signal,
    ),
    presentCall: args => ({
      card: 'generic',
      title: 'Request human approval for robot Plan',
      kind: 'other',
      rawInput: { planId: args.planId },
    }),
  }))

  ctx.tools.register(defineTool({
    name: 'robot_execution_status',
    description: 'Read authoritative approval and Task state for an execution request previously created by this Harness provider.',
    parameters: {
      requestId: {
        type: 'string',
        required: true,
        description: 'The requestId returned by robot_request_execution.',
      },
    },
    output: {
      schema: {
        type: 'object',
        additionalProperties: false,
        properties: {
          mode: {
            type: 'string',
            required: true,
            const: 'approval-gated',
          },
          provider: { type: 'string', required: true },
          requestId: { type: 'string', required: true },
          planId: { type: 'string', required: true },
          status: { type: 'string', required: true },
          instruction: { type: 'string', required: true },
          createdAt: { type: 'string', required: true },
          expiresInSeconds: { type: 'number', required: true },
          plan: {
            type: 'object',
            required: true,
            additionalProperties: true,
          },
          taskId: { type: 'string' },
          task: {
            type: 'object',
            additionalProperties: true,
          },
        },
      },
      render: (_args, value) => renderJson(
        'Robot execution approval and Task status:',
        value,
      ),
    },
    timeoutMs: executionTimeoutMs,
    isConcurrencySafe: () => true,
    execute: (args, exec) => ctx.robot.executionStatus(
      { requestId: args.requestId },
      exec.signal,
    ),
    presentCall: args => ({
      card: 'generic',
      title: 'Read robot execution request status',
      kind: 'read',
      rawInput: { requestId: args.requestId },
    }),
  }))
}
