import { once } from 'node:events'
import { get, type ClientRequest } from 'node:http'

import { describe, expect, it } from 'vitest'

import type {
  PythonAgentApi,
  PythonAgentCursor,
} from '../src/python-agent-client.ts'
import { startPythonAgentServer } from '../src/server.ts'

interface RecordedCall {
  readonly method: string
  readonly args: Readonly<Record<string, unknown>>
}

function createFakeClient(
  calls: RecordedCall[],
  health: Readonly<Record<string, unknown>> = { bootstrapState: 'ready' },
): PythonAgentApi {
  let sentEvent = false
  return {
    async health() {
      calls.push({ method: 'health', args: {} })
      return health
    },
    async manifest() {
      calls.push({ method: 'manifest', args: {} })
      return { skills: [] }
    },
    async plan(args) {
      calls.push({ method: 'plan', args })
      return {
        planId: 'plan-http-1',
        plan: {
          instruction: args.instruction,
          constraints: ['workspace-clear'],
          steps: [{ skill: 'pick', args: { object: 'cup' } }],
        },
      }
    },
    async execute(args) {
      calls.push({ method: 'execute', args })
      return { taskId: 'task-http-1' }
    },
    async taskStatus(args) {
      calls.push({ method: 'task_status', args })
      return { status: 'running' }
    },
    async taskEvents(args: { readonly taskId: string, readonly cursor: PythonAgentCursor }) {
      calls.push({ method: 'task_events', args })
      if (sentEvent) return { events: [], nextCursor: 1 }
      sentEvent = true
      return {
        events: [{
          type: 'skill.started',
          skill: 'pick',
          status: 'running',
          message: 'Picking cup.',
        }],
        nextCursor: 1,
      }
    },
    async cancel(args) {
      calls.push({ method: 'cancel', args })
      return { accepted: true }
    },
  }
}

async function waitFor(
  predicate: () => boolean | Promise<boolean>,
  timeoutMs = 1_000,
): Promise<void> {
  const deadline = Date.now() + timeoutMs
  while (!await predicate()) {
    if (Date.now() >= deadline) throw new Error('Timed out waiting for condition.')
    await new Promise(resolve => setTimeout(resolve, 5))
  }
}

async function openEventStream(
  url: string,
  events: Array<Record<string, unknown>>,
): Promise<ClientRequest> {
  let buffer = ''
  let markReady: (() => void) | undefined
  const ready = new Promise<void>(resolve => {
    markReady = resolve
  })
  const request = get(url, (response) => {
    markReady?.()
    response.setEncoding('utf8')
    response.on('data', (chunk: string) => {
      buffer += chunk
      let boundary = buffer.indexOf('\n\n')
      while (boundary >= 0) {
        const block = buffer.slice(0, boundary)
        buffer = buffer.slice(boundary + 2)
        const data = block.split('\n').find(line => line.startsWith('data: '))
        if (data !== undefined) {
          const value: unknown = JSON.parse(data.slice('data: '.length))
          if (typeof value === 'object' && value !== null && !Array.isArray(value)) {
            events.push(value as Record<string, unknown>)
          }
        }
        boundary = buffer.indexOf('\n\n')
      }
    })
  })
  await ready
  return request
}

describe('Python Agent HTTP facade', () => {
  it('does not promote a loading Agent to ready after planning', async () => {
    const calls: RecordedCall[] = []
    const server = startPythonAgentServer(
      createFakeClient(calls, { bootstrapState: 'not_started' }),
      '/tmp/fake-output',
      0,
      { pollIntervalMs: 10 },
    )
    await once(server, 'listening')
    const address = server.address()
    if (address === null || typeof address === 'string') {
      throw new Error('HTTP server has no TCP port.')
    }
    const baseUrl = `http://127.0.0.1:${address.port}`

    try {
      await waitFor(async () => {
        const response = await fetch(`${baseUrl}/api/state`)
        const state = await response.json() as {
          readonly agent?: { readonly status?: string }
        }
        return state.agent?.status === 'loading'
      })
      const previewResponse = await fetch(`${baseUrl}/api/tasks`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ text: '整理杯子' }),
      })
      expect(previewResponse.status).toBe(202)
      const preview = await previewResponse.json() as { readonly planId: string }

      const confirmResponse = await fetch(
        `${baseUrl}/api/tasks/${preview.planId}/confirm`,
        {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ confirmation: 'ENABLE_DUAL_V2_2_HARDWARE' }),
        },
      )
      expect(confirmResponse.status).toBe(409)
      expect(calls.some(call => call.method === 'execute')).toBe(false)

      const discardResponse = await fetch(
        `${baseUrl}/api/tasks/${preview.planId}`,
        { method: 'DELETE' },
      )
      expect(discardResponse.status).toBe(200)
      const replacementResponse = await fetch(`${baseUrl}/api/tasks`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ text: '重新整理杯子' }),
      })
      expect(replacementResponse.status).toBe(202)
    } finally {
      await new Promise<void>(resolve => server.close(() => resolve()))
    }
  })

  it('previews, confirms, forwards SSE events, and cancels', async () => {
    const calls: RecordedCall[] = []
    const server = startPythonAgentServer(
      createFakeClient(calls),
      '/tmp/fake-output',
      0,
      { pollIntervalMs: 10 },
    )
    await once(server, 'listening')
    const address = server.address()
    if (address === null || typeof address === 'string') {
      throw new Error('HTTP server has no TCP port.')
    }
    const baseUrl = `http://127.0.0.1:${address.port}`
    const streamedEvents: Array<Record<string, unknown>> = []
    const eventRequest = await openEventStream(`${baseUrl}/api/events`, streamedEvents)

    try {
      const previewResponse = await fetch(`${baseUrl}/api/tasks`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ text: '整理杯子' }),
      })
      expect(previewResponse.status).toBe(202)
      const preview = await previewResponse.json() as Record<string, unknown>
      expect(preview).toMatchObject({ planId: 'plan-http-1', preview: true })

      const busyResponse = await fetch(`${baseUrl}/api/tasks`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ text: '第二个任务' }),
      })
      expect(busyResponse.status).toBe(409)

      const wrongConfirmation = await fetch(
        `${baseUrl}/api/tasks/plan-http-1/confirm`,
        {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ confirmation: 'yes' }),
        },
      )
      expect(wrongConfirmation.status).toBe(400)

      const confirmResponse = await fetch(
        `${baseUrl}/api/tasks/plan-http-1/confirm`,
        {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ confirmation: 'ENABLE_DUAL_V2_2_HARDWARE' }),
        },
      )
      expect(confirmResponse.status).toBe(202)
      await expect(confirmResponse.json()).resolves.toMatchObject({
        taskId: 'task-http-1',
        planId: 'plan-http-1',
      })

      await waitFor(() => streamedEvents.some(event => event.type === 'skill.started'))
      expect(streamedEvents).toContainEqual(expect.objectContaining({
        taskId: 'task-http-1',
        type: 'skill.started',
        skill: 'pick',
      }))

      const state = await fetch(`${baseUrl}/api/state`).then(
        async response => response.json() as Promise<Record<string, unknown>>,
      )
      expect(state).toMatchObject({
        runtime: 'python-agent',
        agent: { status: 'ready', ready: true },
        activePreview: null,
        activeTask: { taskId: 'task-http-1' },
      })

      const cancelResponse = await fetch(`${baseUrl}/api/tasks/task-http-1/cancel`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: '{}',
      })
      expect(cancelResponse.status).toBe(202)
      await expect(cancelResponse.json()).resolves.toMatchObject({
        taskId: 'task-http-1',
        cancelRequested: true,
      })

      expect(calls).toContainEqual({
        method: 'plan',
        args: { instruction: '整理杯子', output_dir: '/tmp/fake-output' },
      })
      expect(calls).toContainEqual({
        method: 'execute',
        args: {
          planId: 'plan-http-1',
          confirmation: 'ENABLE_DUAL_V2_2_HARDWARE',
        },
      })
      expect(calls).toContainEqual({
        method: 'cancel',
        args: { taskId: 'task-http-1' },
      })
    } finally {
      eventRequest.destroy()
      await new Promise<void>(resolve => server.close(() => resolve()))
    }
  })
})
