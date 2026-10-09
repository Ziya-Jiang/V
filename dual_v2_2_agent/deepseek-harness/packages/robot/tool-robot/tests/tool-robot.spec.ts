import { Context } from '@deepseek-ai/cordis'
import { CallId } from '@deepseek-ai/dsh-llm'
import {
  RobotRuntime,
  type RobotExecutionRequest,
  type RobotExecutionRequestInput,
  type RobotExecutionStatusInput,
  type RobotManifest,
  type RobotPlan,
  type RobotPlanRequest,
  type RobotStatus,
} from '@deepseek-ai/dsh-robot'
import SystemPrompt from '@deepseek-ai/dsh-system-prompt'
import ToolRuntime from '@deepseek-ai/dsh-tools'
import { describe, expect, it } from 'vitest'

import { apply } from '../src/index.ts'

class StubRobotRuntime extends RobotRuntime {
  readonly calls: string[] = []

  status(signal?: AbortSignal): Promise<RobotStatus> {
    this.calls.push(`status:${signal?.aborted === true}`)
    return Promise.resolve({
      mode: 'dry-run',
      provider: 'stub',
      planOnlyReady: true,
      executionReady: false,
      health: {
        planOnly: true,
        bootstrapState: 'not_started',
      },
    })
  }

  manifest(signal?: AbortSignal): Promise<RobotManifest> {
    this.calls.push(`manifest:${signal?.aborted === true}`)
    return Promise.resolve({
      mode: 'dry-run',
      provider: 'stub',
      tools: [{ name: 'localize' }],
      skills: [{ name: 'tidy_table' }],
    })
  }

  plan(
    request: RobotPlanRequest,
    signal?: AbortSignal,
  ): Promise<RobotPlan> {
    this.calls.push(`plan:${request.instruction}:${signal?.aborted === true}`)
    return Promise.resolve({
      mode: 'dry-run',
      provider: 'stub',
      planId: 'plan-1',
      instruction: request.instruction,
      outputDir: '/tmp/robot-scene',
      createdAt: '2026-08-23T06:00:00Z',
      plan: {
        instruction: request.instruction,
        steps: [],
      },
    })
  }

  override requestExecution(
    request: RobotExecutionRequestInput,
    signal?: AbortSignal,
  ): Promise<RobotExecutionRequest> {
    this.calls.push(
      `requestExecution:${request.planId}:${signal?.aborted === true}`,
    )
    return Promise.resolve({
      mode: 'approval-gated',
      provider: 'stub',
      requestId: 'request-1',
      planId: request.planId,
      status: 'awaiting_approval',
      instruction: '整理桌面',
      createdAt: '2026-08-24T06:00:00Z',
      expiresInSeconds: 600,
      plan: {
        instruction: '整理桌面',
        steps: [{ skill: 'tidy_table', args: {} }],
      },
    })
  }

  override executionStatus(
    request: RobotExecutionStatusInput,
    signal?: AbortSignal,
  ): Promise<RobotExecutionRequest> {
    this.calls.push(
      `executionStatus:${request.requestId}:${signal?.aborted === true}`,
    )
    return Promise.resolve({
      mode: 'approval-gated',
      provider: 'stub',
      requestId: request.requestId,
      planId: 'plan-1',
      status: 'approved',
      instruction: '整理桌面',
      createdAt: '2026-08-24T06:00:00Z',
      expiresInSeconds: 600,
      taskId: 'task-1',
      plan: {
        instruction: '整理桌面',
        steps: [{ skill: 'tidy_table', args: {} }],
      },
      task: { taskId: 'task-1', status: 'queued' },
    })
  }
}

function setup(config: Parameters<typeof apply>[1] = {}) {
  const ctx = new Context()
  new SystemPrompt(ctx, {})
  new ToolRuntime(ctx)
  const robot = new StubRobotRuntime(ctx)
  apply(ctx, config)
  return { ctx, robot }
}

function execute(
  ctx: Context,
  name: string,
  args: unknown,
) {
  return ctx.tools.execute({
    callId: CallId(`call-${name}`),
    name,
    arguments: args,
    signal: new AbortController().signal,
  })
}

describe('tool-robot', () => {
  it('registers exactly the three plan-only robot tools', () => {
    const { ctx } = setup()
    const names = ctx.tools.schemas().map(schema => schema.name)
      .filter(name => name.startsWith('robot_'))
      .sort()

    expect(names).toEqual([
      'robot_manifest',
      'robot_plan',
      'robot_status',
    ])
    expect(ctx.tools.get('robot_execute')).toBeUndefined()
    expect(ctx.tools.get('robot_cancel')).toBeUndefined()
    expect(ctx.tools.get('robot_plan')?.timeoutMs).toBe(60_000)
    expect(ctx.tools.get('robot_status')?.timeoutMs).toBe(30_000)
  })

  it('routes status, manifest, and semantic planning through ctx.robot', async () => {
    const { ctx, robot } = setup()

    const status = await execute(ctx, 'robot_status', {})
    const manifest = await execute(ctx, 'robot_manifest', {})
    const plan = await execute(ctx, 'robot_plan', {
      instruction: '把杯子放到盘子旁边',
    })

    expect(status).toMatchObject({
      isError: false,
      value: { mode: 'dry-run', planOnlyReady: true },
    })
    expect(manifest).toMatchObject({
      isError: false,
      value: { skills: [{ name: 'tidy_table' }] },
    })
    expect(plan).toMatchObject({
      isError: false,
      value: {
        mode: 'dry-run',
        planId: 'plan-1',
        instruction: '把杯子放到盘子旁边',
      },
    })
    expect(robot.calls).toEqual([
      'status:false',
      'manifest:false',
      'plan:把杯子放到盘子旁边:false',
    ])
    expect(plan.content[0]).toMatchObject({
      type: 'text',
    })
    if (plan.content[0]?.type !== 'text') {
      throw new Error('robot_plan must render text')
    }
    expect(plan.content[0].text).toContain('inert')
  })

  it('contributes an explicit plan-only instruction to the system prompt', async () => {
    const { ctx } = setup()

    const prompt = await ctx.systemPrompt.assemble()
    const text = prompt.sections.map(section => section.text).join('\n')

    expect(text).toContain('Robot access is PLAN-ONLY and DRY-RUN')
    expect(text).toContain('cannot execute, cancel, or control hardware')
  })

  it('adds model request/status tools only behind the approval gate', async () => {
    const { ctx, robot } = setup({
      approvalGatedExecution: true,
      executionTimeoutMs: 4321,
    })
    const names = ctx.tools.schemas().map(schema => schema.name)
      .filter(name => name.startsWith('robot_'))
      .sort()
    expect(names).toEqual([
      'robot_execution_status',
      'robot_manifest',
      'robot_plan',
      'robot_request_execution',
      'robot_status',
    ])
    expect(ctx.tools.get('robot_request_execution')?.timeoutMs).toBe(4321)

    const requested = await execute(ctx, 'robot_request_execution', {
      planId: 'plan-1',
    })
    const status = await execute(ctx, 'robot_execution_status', {
      requestId: 'request-1',
    })
    expect(requested).toMatchObject({
      isError: false,
      value: {
        requestId: 'request-1',
        status: 'awaiting_approval',
      },
    })
    expect(status).toMatchObject({
      isError: false,
      value: {
        taskId: 'task-1',
        task: { status: 'queued' },
      },
    })
    expect(robot.calls).toEqual([
      'requestExecution:plan-1:false',
      'executionStatus:request-1:false',
    ])

    const prompt = await ctx.systemPrompt.assemble()
    const text = prompt.sections.map(section => section.text).join('\n')
    expect(text).toContain('Robot access is approval-gated')
    expect(text).toContain('on-site human')
  })

  it('uses configured timeout budgets and rejects invalid values', () => {
    const configured = setup({
      readTimeoutMs: 1234,
      planTimeoutMs: 5678,
    })
    expect(configured.ctx.tools.get('robot_manifest')?.timeoutMs).toBe(1234)
    expect(configured.ctx.tools.get('robot_plan')?.timeoutMs).toBe(5678)

    expect(() => setup({ readTimeoutMs: 0 })).toThrow(
      'readTimeoutMs must be a positive safe integer',
    )
    expect(() => setup({ planTimeoutMs: 1.5 })).toThrow(
      'planTimeoutMs must be a positive safe integer',
    )
    expect(() => setup({ executionTimeoutMs: 0 })).toThrow(
      'executionTimeoutMs must be a positive safe integer',
    )
  })
})
