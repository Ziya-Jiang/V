import { createServer, type AddressInfo } from 'node:net'

import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'

import {
  createHardwareExecutionController,
  HardwareAccessError,
  HardwareExecutionController,
  type HardwareAgentApi,
  validateExecutionPlan,
} from '../src/hardware.ts'
import { HARDWARE_CONFIRMATION } from '../src/live.ts'

const VALID_PLAN = {
  steps: [
    {
      skill: 'perceive_scene',
      args: {
        instruction: '看桌面',
        selectors: [
          { category: 'cup', color: 'blue' },
          { category: 'plate', color: 'yellow' },
        ],
      },
    },
    {
      skill: 'pick_and_place',
      args: {
        object: { category: 'cup', color: 'blue' },
        target: { category: 'plate', color: 'yellow' },
        placement_type: 'beside',
        offset: [100, 120, 0],
      },
    },
    { skill: 'safety_reset', args: { arms: 'both' } },
  ],
}

function createAgent(overrides: Partial<HardwareAgentApi> = {}) {
  const api: HardwareAgentApi = {
    health: vi.fn(async () => ({
      planOnly: false,
      bootstrapState: 'ready',
      activeTaskId: null,
    })),
    plan: vi.fn(async () => ({
      planId: 'plan-1',
      plan: VALID_PLAN,
    })),
    execute: vi.fn(async () => ({
      taskId: 'task-1',
      status: 'queued',
    })),
    pendingExecutionRequests: vi.fn(async () => ({ requests: [] })),
    approveExecution: vi.fn(async () => ({
      requestId: 'request-1',
      taskId: 'task-1',
      status: 'approved',
    })),
    denyExecution: vi.fn(async () => ({
      requestId: 'request-1',
      status: 'denied',
    })),
    taskStatus: vi.fn(async () => ({ status: 'running' })),
    taskEvents: vi.fn(async () => ({
      events: [{ type: 'skill.started', message: 'perceive_scene' }],
      nextCursor: 1,
    })),
    cancel: vi.fn(async () => ({ accepted: true })),
    ...overrides,
  }
  return api
}

function createController(
  api = createAgent(),
  options: { readonly now?: () => number; readonly planTtlMs?: number } = {},
) {
  return new HardwareExecutionController(api, {
    outputDir: '/tmp/robot-soul-test',
    ...options,
  })
}

function enableExecution(): void {
  vi.stubEnv('ROBOT_SOUL_HARDWARE_READONLY', '1')
  vi.stubEnv('ROBOT_SOUL_APPROVED_EXECUTION', HARDWARE_CONFIRMATION)
}

const CHECKLIST = {
  operatorPresent: true,
  workspaceClear: true,
  emergencyStopReady: true,
  robotInspected: true,
}

describe('hardware approval controller', () => {
  beforeEach(() => {
    vi.unstubAllEnvs()
  })

  afterEach(() => {
    vi.unstubAllEnvs()
  })

  it('stays fail-closed unless both server-side gates are enabled', async () => {
    const api = createAgent()
    const controller = createController(api)
    await expect(controller.preview('整理桌面')).rejects.toMatchObject({
      code: 'HARDWARE_DISABLED',
      status: 403,
    })
    expect(api.health).not.toHaveBeenCalled()

    vi.stubEnv('ROBOT_SOUL_HARDWARE_READONLY', '1')
    await expect(controller.preview('整理桌面')).rejects.toBeInstanceOf(
      HardwareAccessError,
    )
    expect(api.health).not.toHaveBeenCalled()
  })

  it('rejects low-level instructions and non-whitelisted Skills', async () => {
    enableExecution()
    const api = createAgent({
      plan: vi.fn(async () => ({
        planId: 'plan-start',
        plan: { steps: [{ skill: 'start_robot' }] },
      })),
    })
    const controller = createController(api)
    await expect(controller.preview('设置关节角为 30 度')).rejects.toThrow(
      'joint angles',
    )
    await expect(controller.preview('启动机器人')).rejects.toMatchObject({
      code: 'PLAN_REJECTED',
    })
    expect(api.execute).not.toHaveBeenCalled()
  })

  it('requires a ready production Agent with no active task', async () => {
    enableExecution()
    const planOnly = createController(createAgent({
      health: vi.fn(async () => ({
        planOnly: true,
        bootstrapState: 'disabled',
        activeTaskId: null,
      })),
    }))
    await expect(planOnly.preview('整理桌面')).rejects.toMatchObject({
      code: 'AGENT_NOT_READY',
    })

    const busy = createController(createAgent({
      health: vi.fn(async () => ({
        planOnly: false,
        bootstrapState: 'ready',
        activeTaskId: 'other-task',
      })),
    }))
    await expect(busy.preview('整理桌面')).rejects.toMatchObject({
      code: 'TASK_CONFLICT',
    })

    const launcherCompatible = createController(createAgent({
      health: vi.fn(async () => ({
        planOnly: false,
        ready: true,
        activeTaskId: null,
      })),
    }))
    await expect(launcherCompatible.preview('整理桌面')).resolves.toMatchObject({
      mode: 'hardware-approval-required',
    })
  })

  it('requires current-process Plan, full checklist and exact phrase', async () => {
    enableExecution()
    const api = createAgent()
    const controller = createController(api)
    const preview = await controller.preview('整理桌面')
    expect(preview).toMatchObject({
      mode: 'hardware-approval-required',
      planId: 'plan-1',
      hardwareOwner: 'python-agent',
    })
    await expect(controller.execute({
      planId: 'plan-1',
      confirmation: 'yes',
      checklist: CHECKLIST,
    })).rejects.toMatchObject({ code: 'APPROVAL_REQUIRED' })
    await expect(controller.execute({
      planId: 'plan-1',
      confirmation: HARDWARE_CONFIRMATION,
      checklist: { ...CHECKLIST, workspaceClear: false },
    })).rejects.toMatchObject({ code: 'APPROVAL_REQUIRED' })
    await expect(controller.execute({
      planId: 'unknown-plan',
      confirmation: HARDWARE_CONFIRMATION,
      checklist: CHECKLIST,
    })).rejects.toMatchObject({ code: 'UNKNOWN_PLAN' })
    expect(api.execute).not.toHaveBeenCalled()
  })

  it('executes once, scopes task reads and forwards cooperative cancel', async () => {
    enableExecution()
    const api = createAgent()
    const controller = createController(api)
    await controller.preview('整理桌面')
    await expect(controller.execute({
      planId: 'plan-1',
      confirmation: HARDWARE_CONFIRMATION,
      checklist: CHECKLIST,
    })).resolves.toMatchObject({
      mode: 'hardware-execution',
      taskId: 'task-1',
    })
    expect(api.execute).toHaveBeenCalledWith({
      planId: 'plan-1',
      confirmation: HARDWARE_CONFIRMATION,
    })
    await expect(controller.status('not-issued')).rejects.toMatchObject({
      code: 'UNKNOWN_TASK',
    })
    await expect(controller.status('task-1')).resolves.toMatchObject({
      taskId: 'task-1',
      status: 'running',
    })
    await expect(controller.events('task-1', '0')).resolves.toMatchObject({
      nextCursor: 1,
    })
    expect(api.taskEvents).toHaveBeenCalledWith({ taskId: 'task-1', cursor: 0 })
    await expect(controller.events('task-1', 'invalid')).rejects.toMatchObject({
      code: 'INVALID_REQUEST',
    })
    await expect(controller.cancel('task-1')).resolves.toMatchObject({
      taskId: 'task-1',
      cancelRequested: true,
    })
  })

  it('expires previews before execute', async () => {
    enableExecution()
    let now = 1000
    const api = createAgent()
    const controller = createController(api, {
      now: () => now,
      planTtlMs: 10,
    })
    await controller.preview('整理桌面')
    now = 1010
    await expect(controller.execute({
      planId: 'plan-1',
      confirmation: HARDWARE_CONFIRMATION,
      checklist: CHECKLIST,
    })).rejects.toMatchObject({ code: 'UNKNOWN_PLAN' })
    expect(api.execute).not.toHaveBeenCalled()
  })

  it('approves only validated DeepSeek Harness requests', async () => {
    enableExecution()
    const api = createAgent({
      pendingExecutionRequests: vi.fn(async () => ({
        requests: [{
          requestId: 'request-1',
          planId: 'plan-1',
          source: 'deepseek-harness',
          status: 'awaiting_approval',
          plan: VALID_PLAN,
        }],
      })),
    })
    const controller = createController(api)
    await expect(controller.pendingApprovals()).resolves.toMatchObject({
      requests: [{ requestId: 'request-1' }],
    })
    await expect(controller.approveRequest({
      requestId: 'request-1',
      confirmation: HARDWARE_CONFIRMATION,
      checklist: CHECKLIST,
    })).resolves.toMatchObject({
      requestId: 'request-1',
      taskId: 'task-1',
    })
    expect(api.approveExecution).toHaveBeenCalledWith({
      requestId: 'request-1',
      confirmation: HARDWARE_CONFIRMATION,
      checklist: CHECKLIST,
    })
  })
})

describe('execution Plan validator', () => {
  it('accepts only reviewed high-level Skills', () => {
    expect(validateExecutionPlan(VALID_PLAN)).toBe(VALID_PLAN)
    expect(() => validateExecutionPlan({
      steps: [{ skill: 'joint_move' }],
    })).toThrow('does not allow Skill')
    expect(() => validateExecutionPlan({
      steps: Array.from({ length: 33 }, () => ({ skill: 'safety_reset' })),
    })).toThrow('1 to 12')
    expect(() => validateExecutionPlan({
      steps: [{
        skill: 'pick_and_place',
        args: {
          object: { category: 'cup' },
          target: { category: 'plate' },
          offset: [1000, 0, 0],
        },
      }],
    })).toThrow('bounded')
  })
})

describe('zero-motion production protocol smoke', () => {
  it('uses only health and plan against the NDJSON Agent seam', async () => {
    enableExecution()
    const methods: string[] = []
    const agent = createServer((socket) => {
      let buffered = ''
      socket.setEncoding('utf8')
      socket.on('data', (chunk: string) => {
        buffered += chunk
        const newline = buffered.indexOf('\n')
        if (newline < 0) return
        const request = JSON.parse(buffered.slice(0, newline)) as {
          method: string
        }
        methods.push(request.method)
        const data = request.method === 'health'
          ? {
            planOnly: false,
            bootstrapState: 'ready',
            activeTaskId: null,
          }
          : request.method === 'plan'
            ? { planId: 'protocol-plan', plan: VALID_PLAN }
            : { unexpected: request.method }
        socket.end(`${JSON.stringify({ ok: true, data })}\n`)
      })
    })
    await new Promise<void>((resolve, reject) => {
      agent.once('error', reject)
      agent.listen(0, '127.0.0.1', resolve)
    })
    try {
      const address = agent.address() as AddressInfo
      vi.stubEnv('ROBOT_SOUL_AGENT_PORT', String(address.port))
      vi.stubEnv('ROBOT_SOUL_OUTPUT_DIR', '/tmp/robot-soul-protocol-smoke')
      const controller = await createHardwareExecutionController()
      await expect(controller.preview('整理桌面')).resolves.toMatchObject({
        planId: 'protocol-plan',
        mode: 'hardware-approval-required',
      })
      expect(methods).toEqual(['health', 'plan'])
      expect(methods).not.toContain('execute')
    } finally {
      await new Promise<void>((resolve, reject) => {
        agent.close((error) => {
          if (error) reject(error)
          else resolve()
        })
      })
    }
  })
})
