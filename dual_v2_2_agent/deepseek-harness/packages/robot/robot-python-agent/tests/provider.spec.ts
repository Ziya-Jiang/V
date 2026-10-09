import { once } from 'node:events'
import {
  createServer,
  type Server as NetServer,
  type Socket,
} from 'node:net'

import { Context } from '@deepseek-ai/cordis'
import { describe, expect, it } from 'vitest'

import { PythonAgentRobotRuntime } from '../src/index.ts'

type JsonRecord = Record<string, unknown>
type RequestHandler = (request: JsonRecord, socket: Socket) => void

const PLAN_ONLY_HEALTH = {
  planOnly: true,
  allowedMethods: ['health', 'manifest', 'plan'],
  eagerBootstrap: false,
  bootstrapState: 'not_started',
  activeTaskId: null,
}
const APPROVAL_HEALTH = {
  planOnly: false,
  allowedMethods: [
    'health',
    'manifest',
    'plan',
    'request_execution',
    'execution_request_status',
  ],
  eagerBootstrap: true,
  bootstrapState: 'ready',
  activeTaskId: null,
}

async function startFakeAgent(handler: RequestHandler): Promise<{
  readonly port: number
  readonly close: () => Promise<void>
}> {
  const sockets = new Set<Socket>()
  const server: NetServer = createServer({ allowHalfOpen: true }, (socket) => {
    sockets.add(socket)
    socket.once('close', () => sockets.delete(socket))
    let request = Buffer.alloc(0)
    socket.on('data', (chunk: Buffer) => {
      request = Buffer.concat([request, chunk])
      const newline = request.indexOf(0x0A)
      if (newline < 0) return
      const value = JSON.parse(
        request.subarray(0, newline).toString('utf8'),
      ) as JsonRecord
      handler(value, socket)
    })
  })
  server.listen(0, '127.0.0.1')
  await once(server, 'listening')
  const address = server.address()
  if (address === null || typeof address === 'string') {
    throw new Error('Fake Python Agent has no TCP port')
  }
  return {
    port: address.port,
    close: async () => {
      for (const socket of sockets) socket.destroy()
      await new Promise<void>((resolve, reject) => {
        server.close((error) => {
          if (error === undefined) resolve()
          else reject(error)
        })
      })
    },
  }
}

function methodOf(request: JsonRecord): string {
  if (typeof request.method !== 'string') throw new Error('missing method')
  return request.method
}

describe('PythonAgentRobotRuntime', () => {
  it('exposes only status, manifest, and plan against a verified plan-only Agent', async () => {
    const requests: JsonRecord[] = []
    const fake = await startFakeAgent((request, socket) => {
      requests.push(request)
      const method = methodOf(request)
      if (method === 'health') {
        socket.end(`${JSON.stringify({ ok: true, data: PLAN_ONLY_HEALTH })}\n`)
        return
      }
      if (method === 'manifest') {
        socket.end(`${JSON.stringify({
          ok: true,
          data: {
            tools: [{ type: 'function', function: { name: 'localize' } }],
            skills: [{ name: 'tidy_table' }],
          },
        })}\n`)
        return
      }
      if (method === 'plan') {
        const args = request.args as JsonRecord
        socket.end(`${JSON.stringify({
          ok: true,
          data: {
            planId: 'plan-1',
            instruction: args.instruction,
            output_dir: args.output_dir,
            createdAt: '2026-08-23T06:00:00Z',
            plan: {
              instruction: args.instruction,
              steps: [{ skill: 'tidy_table', args: {} }],
            },
          },
        })}\n`)
        return
      }
      socket.end(`${JSON.stringify({ ok: false, error: 'method denied' })}\n`)
    })

    try {
      const ctx = new Context()
      const robot = new PythonAgentRobotRuntime(ctx, {
        host: '127.0.0.1',
        port: fake.port,
        token: 'shared-token',
        outputDir: '/tmp/robot-scene',
        timeoutMs: 500,
      })

      await expect(robot.status()).resolves.toMatchObject({
        mode: 'dry-run',
        provider: 'python-agent',
        planOnlyReady: true,
      })
      await expect(robot.manifest()).resolves.toMatchObject({
        mode: 'dry-run',
        skills: [{ name: 'tidy_table' }],
      })
      await expect(robot.plan({ instruction: '  整理桌面  ' })).resolves.toEqual({
        mode: 'dry-run',
        provider: 'python-agent',
        planId: 'plan-1',
        instruction: '整理桌面',
        outputDir: '/tmp/robot-scene',
        createdAt: '2026-08-23T06:00:00Z',
        plan: {
          instruction: '整理桌面',
          steps: [{ skill: 'tidy_table', args: {} }],
        },
      })

      expect(requests.map(methodOf)).toEqual([
        'health',
        'health',
        'manifest',
        'health',
        'plan',
      ])
      expect(requests.every(request => request.token === 'shared-token')).toBe(true)
      expect(requests.map(methodOf)).not.toContain('execute')
      expect(requests.at(-1)?.args).toEqual({
        instruction: '整理桌面',
        output_dir: '/tmp/robot-scene',
      })
    } finally {
      await fake.close()
    }
  })

  it('reports unsafe status and refuses capability reads without process plan-only mode', async () => {
    const fake = await startFakeAgent((_request, socket) => {
      socket.end(`${JSON.stringify({
        ok: true,
        data: {
          ...PLAN_ONLY_HEALTH,
          planOnly: false,
          eagerBootstrap: true,
          bootstrapState: 'ready',
        },
      })}\n`)
    })

    try {
      const robot = new PythonAgentRobotRuntime(new Context(), {
        port: fake.port,
        outputDir: '/tmp/robot-scene',
        timeoutMs: 500,
      })
      await expect(robot.status()).resolves.toMatchObject({
        planOnlyReady: false,
        reason: 'Python Agent must be started with ROBOT_AGENT_PLAN_ONLY=1',
      })
      await expect(robot.manifest()).rejects.toMatchObject({
        code: 'ROBOT_PLAN_ONLY_REQUIRED',
      })
    } finally {
      await fake.close()
    }
  })

  it('fails closed on an expanded method set and an active task', async () => {
    const healthResponses = [
      {
        ...PLAN_ONLY_HEALTH,
        allowedMethods: [...PLAN_ONLY_HEALTH.allowedMethods, 'execute'],
      },
      {
        ...PLAN_ONLY_HEALTH,
        activeTaskId: 'task-1',
      },
    ]
    const fake = await startFakeAgent((_request, socket) => {
      socket.end(`${JSON.stringify({
        ok: true,
        data: healthResponses.shift(),
      })}\n`)
    })

    try {
      const robot = new PythonAgentRobotRuntime(new Context(), {
        port: fake.port,
        outputDir: '/tmp/robot-scene',
        timeoutMs: 500,
      })
      await expect(robot.status()).resolves.toMatchObject({
        planOnlyReady: false,
        reason: 'Python Agent did not advertise the exact plan-only method set',
      })
      await expect(robot.status()).resolves.toMatchObject({
        planOnlyReady: false,
        reason: 'Python Agent reports an active or unverifiable hardware task',
      })
    } finally {
      await fake.close()
    }
  })

  it('creates only human-gated execution requests in explicit approval mode', async () => {
    const requests: JsonRecord[] = []
    const fake = await startFakeAgent((request, socket) => {
      requests.push(request)
      const method = methodOf(request)
      if (method === 'health') {
        socket.end(`${JSON.stringify({
          ok: true,
          data: APPROVAL_HEALTH,
        })}\n`)
        return
      }
      if (method === 'plan') {
        const args = request.args as JsonRecord
        socket.end(`${JSON.stringify({
          ok: true,
          data: {
            planId: 'plan-approved-1',
            instruction: args.instruction,
            output_dir: args.output_dir,
            createdAt: '2026-08-24T06:00:00Z',
            plan: {
              instruction: args.instruction,
              steps: [{ skill: 'tidy_table', args: {} }],
            },
          },
        })}\n`)
        return
      }
      if (
        method === 'request_execution'
        || method === 'execution_request_status'
      ) {
        const status = method === 'request_execution'
          ? 'awaiting_approval'
          : 'approved'
        socket.end(`${JSON.stringify({
          ok: true,
          data: {
            requestId: 'request-1',
            planId: 'plan-approved-1',
            status,
            instruction: '整理桌面',
            createdAt: '2026-08-24T06:00:00Z',
            expiresInSeconds: 600,
            plan: {
              instruction: '整理桌面',
              steps: [{ skill: 'tidy_table', args: {} }],
            },
            taskId: status === 'approved' ? 'task-1' : null,
            ...(status === 'approved'
              ? { task: { taskId: 'task-1', status: 'queued' } }
              : {}),
          },
        })}\n`)
        return
      }
      socket.end(`${JSON.stringify({ ok: false, error: 'method denied' })}\n`)
    })

    try {
      const robot = new PythonAgentRobotRuntime(new Context(), {
        port: fake.port,
        outputDir: '/tmp/robot-scene',
        timeoutMs: 500,
        approvalGatedExecution: true,
      })
      await expect(robot.status()).resolves.toMatchObject({
        mode: 'approval-gated',
        planOnlyReady: false,
        executionReady: true,
      })
      const plan = await robot.plan({ instruction: '整理桌面' })
      const execution = await robot.requestExecution({
        planId: plan.planId,
      })
      expect(execution).toMatchObject({
        mode: 'approval-gated',
        requestId: 'request-1',
        status: 'awaiting_approval',
      })
      await expect(robot.executionStatus({
        requestId: execution.requestId,
      })).resolves.toMatchObject({
        status: 'approved',
        taskId: 'task-1',
        task: { status: 'queued' },
      })
      expect(requests.map(methodOf)).not.toContain('execute')
      expect(requests.map(methodOf)).not.toContain('approve_execution')
    } finally {
      await fake.close()
    }
  })

  it('rejects unsafe endpoints, invalid ports and paths, and empty instructions', async () => {
    expect(() => new PythonAgentRobotRuntime(new Context(), {
      host: '0.0.0.0',
      outputDir: '/tmp/robot-scene',
    })).toThrow(expect.objectContaining({ code: 'ROBOT_CONFIGURATION' }))
    expect(() => new PythonAgentRobotRuntime(new Context(), {
      port: 65_536,
      outputDir: '/tmp/robot-scene',
    })).toThrow(expect.objectContaining({ code: 'ROBOT_CONFIGURATION' }))
    expect(() => new PythonAgentRobotRuntime(new Context(), {
      outputDir: 'relative/scene',
    })).toThrow(expect.objectContaining({ code: 'ROBOT_CONFIGURATION' }))

    const robot = new PythonAgentRobotRuntime(new Context(), {
      outputDir: '/tmp/robot-scene',
    })
    await expect(robot.plan({ instruction: '   ' })).rejects.toMatchObject({
      code: 'ROBOT_CONFIGURATION',
    })
    await expect(robot.requestExecution({ planId: 'plan-1' }))
      .rejects.toMatchObject({
        code: 'ROBOT_EXECUTION_NOT_ENABLED',
      })
  })

  it('honors caller cancellation while waiting for a response', async () => {
    const fake = await startFakeAgent(() => {
      // Keep the half-open connection silent until the caller aborts it.
    })
    try {
      const robot = new PythonAgentRobotRuntime(new Context(), {
        port: fake.port,
        outputDir: '/tmp/robot-scene',
        timeoutMs: 5_000,
      })
      const controller = new AbortController()
      const status = robot.status(controller.signal)
      controller.abort()
      await expect(status).rejects.toMatchObject({ code: 'ROBOT_ABORTED' })
    } finally {
      await fake.close()
    }
  })
})
