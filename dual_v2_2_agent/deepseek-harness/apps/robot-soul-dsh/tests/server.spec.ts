import type { AddressInfo } from 'node:net'
import type { Server } from 'node:http'

import { afterAll, beforeAll, describe, expect, it, vi } from 'vitest'

import type {
  HardwareRecoveryLauncher,
  HardwareStopLauncher,
  HardwareStartupLauncher,
  HarnessStatusReader,
  LiveReader,
  StackStatusReader,
} from '../src/contracts.ts'
import {
  HardwareExecutionController,
  type HardwareAgentApi,
} from '../src/hardware.ts'
import { HARDWARE_CONFIRMATION } from '../src/live.ts'
import { createRobotSoulServer } from '../src/server.ts'

async function listen(server: Server): Promise<string> {
  await new Promise<void>((resolve, reject) => {
    server.once('error', reject)
    server.listen(0, '127.0.0.1', resolve)
  })
  const address = server.address() as AddressInfo
  return `http://127.0.0.1:${address.port}`
}

async function close(server: Server): Promise<void> {
  await new Promise<void>((resolve, reject) => {
    server.close((error) => {
      if (error) reject(error)
      else resolve()
    })
  })
}

async function readJson<T>(response: Response): Promise<T> {
  const value: unknown = await response.json()
  return value as T
}

function createHardwareAgent(): HardwareAgentApi {
  return {
    health: vi.fn(async () => ({
      planOnly: false,
      bootstrapState: 'ready',
      activeTaskId: null,
    })),
    plan: vi.fn(async () => ({
      planId: 'plan-http-1',
      plan: {
        steps: [
          { skill: 'perceive_scene', args: { instruction: '整理桌面' } },
          { skill: 'tidy_table', args: { instruction: '整理桌面' } },
          { skill: 'safety_reset', args: { arms: 'both' } },
        ],
      },
    })),
    execute: vi.fn(async () => ({
      taskId: 'task-http-1',
      status: 'queued',
    })),
    pendingExecutionRequests: vi.fn(async () => ({ requests: [] })),
    approveExecution: vi.fn(async () => ({
      requestId: 'request-http-1',
      taskId: 'task-http-1',
      status: 'approved',
    })),
    denyExecution: vi.fn(async () => ({
      requestId: 'request-http-1',
      status: 'denied',
    })),
    taskStatus: vi.fn(async () => ({ status: 'running' })),
    taskEvents: vi.fn(async () => ({
      events: [{ type: 'task.started' }],
      nextCursor: 1,
    })),
    cancel: vi.fn(async () => ({ accepted: true })),
  }
}

describe('robot operations HTTP server', () => {
  const status = vi.fn(async () => ({
    mode: 'dry-run',
    provider: 'python-agent',
    planOnlyReady: true,
  }))
  const manifest = vi.fn(async () => ({
    mode: 'dry-run',
    tools: [{ name: 'ground' }],
    skills: [{ name: 'perceive_scene' }],
  }))
  const liveReader: LiveReader = { status, manifest }
  const harnessStatusReader: HarnessStatusReader = {
    url: 'http://127.0.0.1:3080',
    read: vi.fn(async () => ({
      connected: true,
      url: 'http://127.0.0.1:3080',
      checkedAt: '2026-08-24T07:00:00.000Z',
    })),
  }
  const hardwareController = new HardwareExecutionController(
    createHardwareAgent(),
    { outputDir: '/tmp/robot-soul-http-test' },
  )
  const server = createRobotSoulServer({
    liveReader,
    hardwareController,
    harnessStatusReader,
  })
  let baseUrl: string

  beforeAll(async () => {
    baseUrl = await listen(server)
  })

  afterAll(async () => {
    await close(server)
  })

  it('serves the interactive page with security headers', async () => {
    const response = await fetch(baseUrl)
    const html = await response.text()
    expect(response.status).toBe(200)
    expect(response.headers.get('content-type')).toContain('text/html')
    expect(response.headers.get('content-security-policy')).toContain("default-src 'self'")
    expect(response.headers.get('content-security-policy'))
      .toContain("frame-src 'none'")
    expect(response.headers.get('x-frame-options')).toBe('DENY')
    expect(html).toContain('HKCLR Harness Agent')
    expect(html).toContain('The agentic operating layer')
    expect(html).toContain('/assets/hkclr-logo.jpeg')
    expect(html).toContain('ROBOT STATUS &amp; OPERATIONS')
    expect(html).toContain('Active hardware Task')
    expect(html).toContain('DUAL-MODE PLAN WORKSPACE')
    expect(html).toContain('data-workspace-mode="simulation"')
    expect(html).toContain('data-workspace-mode="hardware"')
    expect(html).toContain('class="playground" id="playground" hidden aria-hidden="true"')
    expect(html).toContain('class="hardware-bay" id="hardware" hidden aria-hidden="true"')
    expect(html).not.toContain('href="#playground"')
    expect(html).not.toContain('href="#hardware"')
    expect(html).toContain('id="workspaceStartHardware"')
    expect(html).toContain('id="hardwarePowerToggle"')
    expect(html).toMatch(/\.hardware-power-toggle\s*\{\s*position:\s*fixed/)
    expect(html).toContain("requestJson('/api/ops/stop'")
    expect(html).toContain('id="hardwareBootProgress"')
    expect(html).toContain('id="hardwareBootPercent"')
    expect(html).toContain('id="hardwareBootOperation"')
    expect(html).toContain('id="hardwareBootError"')
    expect(html).toContain('id="demoModeToggle"')
    expect(html).toContain('id="demoWizard"')
    expect(html).toContain('id="memoryModeToggle"')
    expect(html).toContain('id="memoryOverlay"')
    expect(html).toContain('class="memory-brand-logo" src="/assets/hkclr-logo.jpeg"')
    expect(html).toContain('id="memoryZoneList"')
    expect(html).toContain('id="memorySlotGrid"')
    expect(html).toContain('id="memoryAvailableList"')
    expect(html).toMatch(/\.memory-mode-toggle\s*\{\s*position:\s*fixed/)
    expect(html).toContain('font-size: 16px; font-weight: 760')
    expect(html).toContain('class="memory-store-hero"')
    expect(html).toContain('class="memory-hero-robot" src="/assets/memory-robot-minimal.png"')
    expect(html).toContain('More capable.<br>Uniquely yours.')
    expect(html).toContain('Choose a system.')
    expect(html).toContain('class="memory-product-grid"')
    expect(html).toContain('class="memory-add"')
    expect(html).toContain('class="memory-price-original"')
    expect(html).toContain('class="memory-product-detail"')
    expect(html).toContain('.memory-product-card:hover .memory-product-detail')
    expect(html).toContain('background: #f5f5f7')
    expect(html).toContain('.memory-overlay::before, .memory-overlay::after { display: none; }')
    expect(html).not.toContain('class="memory-robot-image"')
    expect(html).not.toContain('class="memory-body-zone"')
    expect(html).not.toContain('class="memory-skill-detail"')
    expect(html).not.toContain('class="memory-app-group"')
    expect(html).toContain('function renderMemoryLoadout()')
    expect(html).toContain('function memoryCapabilityMarkup(product, equipped, slots)')
    expect(html).toContain('function memoryZoneIcon(zoneId)')
    expect(html).toContain('class="memory-zone-icon" data-system="')
    expect(html).not.toContain("icon: '◉'")
    expect(html).toContain('function memoryProductIcon(product)')
    expect(html).toContain('function memoryPriceFor(product)')
    expect(html).toContain("var discount = product.kind === 'tool' ? 80 : 60")
    expect(html).toContain('function memorySkillDetailMarkup(product)')
    expect(html).toContain('function equipMemoryProduct(name)')
    expect(html).toContain('function restoreMemoryLoadout()')
    expect(html).toContain("'robot-soul-memory-loadout-v1'")
    expect(html).toMatch(/\.demo-shell\s*\{[^}]*width:\s*100%;[^}]*min-height:\s*100svh;/)
    expect(html).toMatch(/\.hardware-boot-panel\s*\{[^}]*width:\s*100%;[^}]*min-height:\s*100svh;/)
    expect(html).toContain('.demo-wizard::-webkit-scrollbar, .hardware-boot-overlay::-webkit-scrollbar')
    expect(html).toContain('body:has(.demo-wizard:not([hidden]))')
    expect(html).toContain('body:has(.memory-overlay:not([hidden]))')
    expect(html).toContain('function enterImmersiveFullscreen()')
    expect(html).toContain('document.documentElement.requestFullscreen()')
    expect(html).toContain('function startDemoPlanning()')
    expect(html).toContain('function pollDemoHardwareTask()')
    expect(html).toContain('id="demoStopHardwareNow"')
    expect(html).toContain('function stopDemoHardwareNow()')
    expect(html).toContain('function demoActiveStepFraction(events)')
    expect(html).toContain("event.type === 'operation.stage.started'")
    expect(html).toContain('function pauseDemoForHardwareStartup()')
    expect(html).toContain('function resumeDemoAfterHardwareStartup()')
    expect(html).toContain('setTimeout(resumeDemoAfterHardwareStartup, 1100)')
    expect(html).not.toMatch(/#(?:5e5ce6|bf5af2|30d158|ff453a)/i)
    expect(html).toContain('data-demo-safety')
    expect(html).not.toContain('window.prompt')
    expect(html).toContain("hardwareMode ? '/api/hardware/plan' : '/api/simulate/plan'")
    expect(html).toContain('eventsNode.scrollTop = eventsNode.scrollHeight')
    expect(html).not.toContain('<iframe')
    expect(html).toContain('Reinitialize hardware')
    expect(html).toContain('CAPABILITY CATALOG')
    expect(html).toContain('SAFETY MODEL')
    expect(html).toContain('OPERATOR APPROVAL')
    expect(html).not.toMatch(/\p{Script=Han}/u)
  })

  it('serves the HKCLR logos and generated Memory robot asset', async () => {
    const [avifResponse, jpegResponse, memoryRobotResponse] = await Promise.all([
      fetch(`${baseUrl}/assets/hkclr-logo.avif`),
      fetch(`${baseUrl}/assets/hkclr-logo.jpeg`),
      fetch(`${baseUrl}/assets/memory-robot-minimal.png`),
    ])
    expect(avifResponse.status).toBe(200)
    expect(avifResponse.headers.get('content-type')).toBe('image/avif')
    expect((await avifResponse.arrayBuffer()).byteLength).toBeGreaterThan(100)
    expect(jpegResponse.status).toBe(200)
    expect(jpegResponse.headers.get('content-type')).toBe('image/jpeg')
    expect((await jpegResponse.arrayBuffer()).byteLength).toBeGreaterThan(100)
    expect(memoryRobotResponse.status).toBe(200)
    expect(memoryRobotResponse.headers.get('content-type')).toBe('image/png')
    expect((await memoryRobotResponse.arrayBuffer()).byteLength).toBeGreaterThan(100)
  })

  it('aggregates safe service status for the unified GUI', async () => {
    const response = await fetch(`${baseUrl}/api/ops/status`)
    const body = await readJson<{
      data: {
        policy: { hardwareOwner: string; executeEnabled: boolean }
        agent: { connected: boolean; readonly: boolean }
        bootstrap: { state: string; current: number; total: number }
        harness: { connected: boolean; url: string }
        stack: { state: string; recoverable: boolean }
      }
    }>(response)
    expect(response.status).toBe(200)
    expect(body.data).toMatchObject({
      policy: { hardwareOwner: 'python-agent', executeEnabled: false },
      agent: { connected: true, readonly: true },
      bootstrap: { state: 'unavailable', current: 0, total: 1 },
      harness: { connected: true, url: 'http://127.0.0.1:3080' },
      stack: { state: 'healthy', recoverable: false },
    })
    expect(JSON.stringify(body)).not.toContain('ROBOT_SOUL_AGENT_TOKEN')
    expect(JSON.stringify(body)).not.toContain('/tmp/robot-soul-http-test')
  })

  it('serves catalog and architecture as structured content', async () => {
    const [catalogResponse, architectureResponse, policyResponse] = await Promise.all([
      fetch(`${baseUrl}/api/catalog`),
      fetch(`${baseUrl}/api/architecture`),
      fetch(`${baseUrl}/api/hardware/policy`),
    ])
    const catalog = await readJson<{
      data: {
        counts: { skills: number; tools: number; harnessTools: number }
        products: unknown[]
      }
    }>(catalogResponse)
    const architecture = await readJson<{
      data: { nodes: unknown[]; edges: unknown[] }
    }>(architectureResponse)
    const policy = await readJson<{
      data: {
        executeEnabled: boolean
        hardwareOwner: string
        allowedMethods: string[]
        blockedMethods: string[]
      }
    }>(policyResponse)
    expect(catalog.data.counts).toEqual({
      skills: 5,
      tools: 8,
      harnessTools: 5,
    })
    expect(catalog.data.products).toHaveLength(18)
    expect(architecture.data.nodes).toHaveLength(8)
    expect(architecture.data.edges).toHaveLength(7)
    expect(policy.data).toMatchObject({
      executeEnabled: false,
      hardwareOwner: 'python-agent',
      allowedMethods: ['health', 'manifest'],
    })
    expect(policy.data.blockedMethods).toContain('execute')
  })

  it('creates only a simulated semantic plan', async () => {
    const response = await fetch(`${baseUrl}/api/simulate/plan`, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ instruction: '把蓝色杯子放到盘子旁边' }),
    })
    const body = await readJson<{
      data: {
        mode: string
        provider: string
        constraints: {
          hardwareAccess: boolean
          approvalRequiredForFutureExecution: boolean
        }
        steps: Array<{ skill: string }>
      }
    }>(response)
    expect(response.status).toBe(200)
    expect(body.data).toMatchObject({
      mode: 'simulation',
      provider: 'robot-soul-dsh',
      constraints: {
        hardwareAccess: false,
        approvalRequiredForFutureExecution: true,
      },
    })
    expect(body.data.steps.map((step: { skill: string }) => step.skill))
      .toEqual(['perceive_scene', 'pick_and_place', 'safety_reset'])
  })

  it('rejects low-level commands and oversized request bodies', async () => {
    const unsafe = await fetch(`${baseUrl}/api/simulate/plan`, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ instruction: '发送关节角 [0, 30, 60]' }),
    })
    const oversized = await fetch(`${baseUrl}/api/simulate/plan`, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ instruction: 'a'.repeat(17 * 1024) }),
    })
    expect(unsafe.status).toBe(400)
    const unsafeBody = await readJson<{ error: { message: string } }>(unsafe)
    expect(unsafeBody.error.message).toContain('rejected')
    expect(oversized.status).toBe(413)
  })

  it('uses the Provider seam for status and manifest reads only', async () => {
    status.mockClear()
    manifest.mockClear()
    const [statusResponse, manifestResponse] = await Promise.all([
      fetch(`${baseUrl}/api/live/status`),
      fetch(`${baseUrl}/api/live/manifest`),
    ])
    const statusBody = await readJson<{
      mode: string
      fallback: string
      data: { connected: boolean; readonly: boolean }
    }>(statusResponse)
    const manifestBody = await readJson<{
      data: {
        connected: boolean
        data: { products: unknown[] }
      }
    }>(manifestResponse)
    expect(statusBody).toMatchObject({
      mode: 'live-readonly',
      fallback: 'simulation',
      data: { connected: true, readonly: true },
    })
    expect(manifestBody.data.connected).toBe(true)
    expect(manifestBody.data.data.products).toHaveLength(18)
    expect(status).toHaveBeenCalledOnce()
    expect(manifest).toHaveBeenCalledOnce()
  })

  it('falls back cleanly when the loopback Agent is unavailable', async () => {
    const offlineServer = createRobotSoulServer({
      liveReader: {
        status: async () => { throw new Error('connect ECONNREFUSED 127.0.0.1:15555') },
        manifest: async () => { throw new Error('connect ECONNREFUSED 127.0.0.1:15555') },
      },
    })
    const offlineUrl = await listen(offlineServer)
    try {
      const response = await fetch(`${offlineUrl}/api/live/status`)
      const body = await readJson<{
        data: { connected: boolean; readonly: boolean; reason: string }
      }>(response)
      expect(response.status).toBe(200)
      expect(body.data).toMatchObject({
        connected: false,
        readonly: true,
      })
      expect(body.data.reason).toContain('ECONNREFUSED')
    } finally {
      await close(offlineServer)
    }
  })

  it.each([
    '/api/execute',
    '/api/cancel',
    '/api/task',
    '/api/call',
    '/api/live/execute',
    '/api/live/cancel',
    '/api/live/task',
    '/api/live/call',
  ])('does not expose execution-plane route %s', async (route) => {
    const response = await fetch(`${baseUrl}${route}`, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: '{}',
    })
    expect(response.status).toBe(404)
    const body = await readJson<{ error: { code: string } }>(response)
    expect(body.error.code).toBe('NOT_FOUND')
  })

  it('keeps namespaced hardware Plan locked by default', async () => {
    const response = await fetch(`${baseUrl}/api/hardware/plan`, {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ instruction: '整理桌面' }),
    })
    expect(response.status).toBe(403)
    const body = await readJson<{ error: { code: string } }>(response)
    expect(body.error.code).toBe('HARDWARE_DISABLED')
  })

  it('rejects simple-content and cross-origin hardware writes', async () => {
    const simple = await fetch(`${baseUrl}/api/hardware/plan`, {
      method: 'POST',
      headers: { 'content-type': 'text/plain' },
      body: JSON.stringify({ instruction: '整理桌面' }),
    })
    const crossOrigin = await fetch(`${baseUrl}/api/hardware/plan`, {
      method: 'POST',
      headers: {
        'content-type': 'application/json',
        origin: 'https://example.com',
      },
      body: JSON.stringify({ instruction: '整理桌面' }),
    })
    expect(simple.status).toBe(415)
    expect(crossOrigin.status).toBe(403)
    expect(
      (await readJson<{ error: { code: string } }>(simple)).error.code,
    ).toBe('ORIGIN_REJECTED')
    expect(
      (await readJson<{ error: { code: string } }>(crossOrigin)).error.code,
    ).toBe('ORIGIN_REJECTED')
  })

  it('keeps hardware recovery operator-only and outside Harness tools', async () => {
    const stackStatusReader: StackStatusReader = {
      read: vi.fn(async () => ({
        state: 'faulted' as const,
        reason: 'hardware_stack_unhealthy',
        detail: 'CAN command/state exchange failed for arm 0.',
        recoverable: true,
        updatedAt: '2026-08-24T08:00:00.000Z',
      })),
    }
    const start = vi.fn(async () => ({ accepted: true as const, pid: 42 }))
    const recoveryLauncher: HardwareRecoveryLauncher = { start }
    const recoveryServer = createRobotSoulServer({
      liveReader,
      harnessStatusReader,
      stackStatusReader,
      recoveryLauncher,
    })
    const recoveryUrl = await listen(recoveryServer)
    try {
      const crossOrigin = await fetch(`${recoveryUrl}/api/ops/recover`, {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: 'https://example.com',
        },
        body: '{}',
      })
      const accepted = await fetch(`${recoveryUrl}/api/ops/recover`, {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: recoveryUrl,
        },
        body: '{}',
      })
      const body = await readJson<{
        data: { accepted: boolean; state: string }
      }>(accepted)

      expect(crossOrigin.status).toBe(403)
      expect(accepted.status).toBe(202)
      expect(body.data).toMatchObject({ accepted: true, state: 'recovering' })
      expect(start).toHaveBeenCalledOnce()
    } finally {
      await close(recoveryServer)
    }
  })

  it('keeps one-click robot startup local without a phrase prompt', async () => {
    vi.stubEnv('ROBOT_SOUL_HARDWARE_READONLY', '1')
    vi.stubEnv('ROBOT_SOUL_APPROVED_EXECUTION', HARDWARE_CONFIRMATION)
    const start = vi.fn(async () => ({ accepted: true as const, pid: 43 }))
    const startupLauncher: HardwareStartupLauncher = { start }
    const stackStatusReader: StackStatusReader = {
      read: vi.fn(async () => ({
        state: 'offline' as const,
        recoverable: false,
        updatedAt: '2026-08-24T08:00:00.000Z',
      })),
    }
    const startupServer = createRobotSoulServer({
      liveReader,
      harnessStatusReader,
      stackStatusReader,
      startupLauncher,
    })
    const startupUrl = await listen(startupServer)
    try {
      const crossOrigin = await fetch(`${startupUrl}/api/ops/start`, {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: 'https://example.com',
        },
        body: '{}',
      })
      const accepted = await fetch(`${startupUrl}/api/ops/start`, {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: startupUrl,
        },
        body: '{}',
      })
      const body = await readJson<{
        data: { accepted: boolean; state: string }
      }>(accepted)

      expect(crossOrigin.status).toBe(403)
      expect(accepted.status).toBe(202)
      expect(body.data).toMatchObject({ accepted: true, state: 'starting' })
      expect(start).toHaveBeenCalledOnce()
    } finally {
      vi.unstubAllEnvs()
      await close(startupServer)
    }
  })

  it('stops only the hardware plane from the same-origin toggle', async () => {
    const stop = vi.fn(async () => ({ accepted: true as const, pid: 44 }))
    const stopLauncher: HardwareStopLauncher = { stop }
    const stackStatusReader: StackStatusReader = {
      read: vi.fn(async () => ({
        state: 'healthy' as const,
        recoverable: false,
        updatedAt: '2026-08-24T08:00:00.000Z',
      })),
    }
    const stopServer = createRobotSoulServer({
      liveReader: {
        status: async () => ({ ready: true, activeTask: null }),
        manifest,
      },
      harnessStatusReader,
      stackStatusReader,
      stopLauncher,
    })
    const stopUrl = await listen(stopServer)
    try {
      const crossOrigin = await fetch(`${stopUrl}/api/ops/stop`, {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: 'https://example.com',
        },
        body: '{}',
      })
      const accepted = await fetch(`${stopUrl}/api/ops/stop`, {
        method: 'POST',
        headers: {
          'content-type': 'application/json',
          origin: stopUrl,
        },
        body: '{}',
      })
      const body = await readJson<{
        data: { accepted: boolean; state: string }
      }>(accepted)

      expect(crossOrigin.status).toBe(403)
      expect(accepted.status).toBe(202)
      expect(body.data).toMatchObject({ accepted: true, state: 'stopping' })
      expect(stop).toHaveBeenCalledOnce()
    } finally {
      await close(stopServer)
    }
  })

  it('serves the complete approval-gated task lifecycle', async () => {
    vi.stubEnv('ROBOT_SOUL_HARDWARE_READONLY', '1')
    vi.stubEnv('ROBOT_SOUL_APPROVED_EXECUTION', HARDWARE_CONFIRMATION)
    const agent = createHardwareAgent()
    const approvalServer = createRobotSoulServer({
      liveReader,
      hardwareController: new HardwareExecutionController(agent, {
        outputDir: '/tmp/robot-soul-http-approval',
      }),
    })
    const approvalUrl = await listen(approvalServer)
    try {
      const planResponse = await fetch(`${approvalUrl}/api/hardware/plan`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ instruction: '整理桌面' }),
      })
      const planBody = await readJson<{
        data: { planId: string; mode: string }
      }>(planResponse)
      expect(planResponse.status).toBe(200)
      expect(planBody.data).toMatchObject({
        planId: 'plan-http-1',
        mode: 'hardware-approval-required',
      })

      const denied = await fetch(`${approvalUrl}/api/hardware/execute`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({
          planId: planBody.data.planId,
          confirmation: HARDWARE_CONFIRMATION,
          checklist: {},
        }),
      })
      expect(denied.status).toBe(403)
      expect(agent.execute).not.toHaveBeenCalled()

      const executeResponse = await fetch(`${approvalUrl}/api/hardware/execute`, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({
          planId: planBody.data.planId,
          confirmation: HARDWARE_CONFIRMATION,
          checklist: {
            operatorPresent: true,
            workspaceClear: true,
            emergencyStopReady: true,
            robotInspected: true,
          },
        }),
      })
      const executeBody = await readJson<{
        data: { taskId: string }
      }>(executeResponse)
      expect(executeResponse.status).toBe(202)
      expect(executeBody.data.taskId).toBe('task-http-1')

      const query = 'taskId=task-http-1'
      const [statusResponse, eventsResponse, cancelResponse] = await Promise.all([
        fetch(`${approvalUrl}/api/hardware/task/status?${query}`),
        fetch(`${approvalUrl}/api/hardware/task/events?${query}&cursor=0`),
        fetch(`${approvalUrl}/api/hardware/task/cancel`, {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ taskId: 'task-http-1' }),
        }),
      ])
      expect(statusResponse.status).toBe(200)
      expect(eventsResponse.status).toBe(200)
      expect(cancelResponse.status).toBe(202)
      expect(agent.taskStatus).toHaveBeenCalledWith({ taskId: 'task-http-1' })
      expect(agent.taskEvents).toHaveBeenCalledWith({
        taskId: 'task-http-1',
        cursor: 0,
      })
      expect(agent.cancel).toHaveBeenCalledWith({ taskId: 'task-http-1' })
    } finally {
      await close(approvalServer)
      vi.unstubAllEnvs()
    }
  })

  it('lets a human approve a validated DeepSeek Harness Plan', async () => {
    vi.stubEnv('ROBOT_SOUL_HARDWARE_READONLY', '1')
    vi.stubEnv('ROBOT_SOUL_APPROVED_EXECUTION', HARDWARE_CONFIRMATION)
    const agent: HardwareAgentApi = {
      ...createHardwareAgent(),
      pendingExecutionRequests: vi.fn(async () => ({
        requests: [{
          requestId: 'request-http-1',
          planId: 'plan-http-1',
          source: 'deepseek-harness',
          status: 'awaiting_approval',
          instruction: '整理桌面',
          plan: {
            steps: [{ skill: 'tidy_table', args: {} }],
          },
        }],
      })),
    }
    const approvalServer = createRobotSoulServer({
      liveReader,
      hardwareController: new HardwareExecutionController(agent, {
        outputDir: '/tmp/robot-soul-harness-approval',
      }),
    })
    const approvalUrl = await listen(approvalServer)
    try {
      const pending = await fetch(
        `${approvalUrl}/api/hardware/approvals`,
      )
      expect(pending.status).toBe(200)
      const pendingBody = await readJson<{
        data: { requests: Array<{ requestId: string }> }
      }>(pending)
      expect(pendingBody.data.requests[0]?.requestId)
        .toBe('request-http-1')

      const approved = await fetch(
        `${approvalUrl}/api/hardware/approvals/approve`,
        {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({
            requestId: 'request-http-1',
            confirmation: HARDWARE_CONFIRMATION,
            checklist: {
              operatorPresent: true,
              workspaceClear: true,
              emergencyStopReady: true,
              robotInspected: true,
            },
          }),
        },
      )
      expect(approved.status).toBe(202)
      expect(agent.approveExecution).toHaveBeenCalledOnce()
      expect(agent.execute).not.toHaveBeenCalled()
    } finally {
      await close(approvalServer)
      vi.unstubAllEnvs()
    }
  })

  it('returns 405 for a known endpoint with the wrong method', async () => {
    const response = await fetch(`${baseUrl}/api/live/status`, { method: 'POST' })
    expect(response.status).toBe(405)
    expect(response.headers.get('allow')).toBe('GET')
  })
})
