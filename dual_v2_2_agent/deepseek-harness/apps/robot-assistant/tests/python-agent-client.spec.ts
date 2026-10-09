import { once } from 'node:events'
import {
  createServer,
  type Server as NetServer,
  type Socket,
} from 'node:net'

import { describe, expect, it } from 'vitest'

import { PythonAgentClient } from '../src/python-agent-client.ts'

type RequestHandler = (request: Record<string, unknown>, socket: Socket) => void

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
      const value: unknown = JSON.parse(request.subarray(0, newline).toString('utf8'))
      if (typeof value !== 'object' || value === null || Array.isArray(value)) {
        throw new Error('Fake server received a non-object request.')
      }
      handler(value as Record<string, unknown>, socket)
    })
  })
  server.listen(0, '127.0.0.1')
  await once(server, 'listening')
  const address = server.address()
  if (address === null || typeof address === 'string') {
    throw new Error('Fake server has no TCP port.')
  }
  return {
    port: address.port,
    close: async () => {
      for (const socket of sockets) socket.destroy()
      await new Promise<void>((resolve, reject) => {
        server.close(error => error === undefined ? resolve() : reject(error))
      })
    },
  }
}

describe('PythonAgentClient', () => {
  it('sends tokenized NDJSON and validates a fragmented plan response', async () => {
    let received: Record<string, unknown> | undefined
    const fake = await startFakeAgent((request, socket) => {
      received = request
      const response = JSON.stringify({
        ok: true,
        data: {
          planId: 'plan-1',
          plan: { constraints: ['loopback'], steps: [{ skill: 'inspect' }] },
        },
      }) + '\n'
      const midpoint = Math.floor(response.length / 2)
      socket.write(response.slice(0, midpoint))
      setImmediate(() => socket.end(response.slice(midpoint)))
    })
    try {
      const client = new PythonAgentClient({
        host: '127.0.0.1',
        port: fake.port,
        token: 'test-token',
        timeoutMs: 500,
      })
      const result = await client.plan({
        instruction: '整理桌面',
        output_dir: '/tmp/fake-output',
      })
      expect(result.planId).toBe('plan-1')
      expect(received).toEqual({
        method: 'plan',
        args: {
          instruction: '整理桌面',
          output_dir: '/tmp/fake-output',
        },
        token: 'test-token',
      })
    } finally {
      await fake.close()
    }
  })

  it('rejects invalid method data with a protocol error', async () => {
    const fake = await startFakeAgent((_request, socket) => {
      socket.end(JSON.stringify({ ok: true, data: { taskId: 42 } }) + '\n')
    })
    try {
      const client = new PythonAgentClient({
        host: '127.0.0.1',
        port: fake.port,
        timeoutMs: 500,
      })
      await expect(client.execute({
        planId: 'plan-1',
        confirmation: 'ENABLE_DUAL_V2_2_HARDWARE',
      })).rejects.toMatchObject({ code: 'protocol' })
    } finally {
      await fake.close()
    }
  })

  it('enforces the response byte limit', async () => {
    const fake = await startFakeAgent((_request, socket) => {
      socket.end(JSON.stringify({
        ok: true,
        data: { ready: true, padding: 'x'.repeat(256) },
      }) + '\n')
    })
    try {
      const client = new PythonAgentClient({
        host: '127.0.0.1',
        port: fake.port,
        timeoutMs: 500,
        maxResponseBytes: 128,
      })
      await expect(client.health())
        .rejects.toMatchObject({ code: 'response-too-large' })
    } finally {
      await fake.close()
    }
  })

  it('times out when the Agent never responds', async () => {
    const fake = await startFakeAgent(() => {
      // Intentionally keep the half-open connection silent.
    })
    try {
      const client = new PythonAgentClient({
        host: '127.0.0.1',
        port: fake.port,
        timeoutMs: 25,
      })
      await expect(client.health())
        .rejects.toMatchObject({ code: 'timeout' })
    } finally {
      await fake.close()
    }
  })
})
