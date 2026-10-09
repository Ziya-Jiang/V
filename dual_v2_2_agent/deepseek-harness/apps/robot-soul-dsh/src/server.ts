import { readFile } from 'node:fs/promises'
import { createServer, type IncomingMessage, type ServerResponse } from 'node:http'
import { isIP } from 'node:net'

import { PRODUCT_CATALOG } from './catalog.ts'
import {
  ARCHITECTURE_EDGES,
  ARCHITECTURE_NODES,
  SITE_CONTENT,
} from './content.ts'
import type {
  AgentBootstrapStatusReader,
  HardwareRecoveryLauncher,
  HardwareStopLauncher,
  HardwareStartupLauncher,
  HarnessStatusReader,
  LiveReader,
  StackStatusReader,
} from './contracts.ts'
import {
  createHardwareExecutionController,
  HardwareAccessError,
  type HardwareExecutionController,
} from './hardware.ts'
import {
  createConfiguredLiveReader,
  hardwareAccessPolicy,
  readLive,
} from './live.ts'
import {
  createAgentBootstrapStatusReader,
  createHardwareRecoveryLauncher,
  createHardwareStopLauncher,
  createHardwareStartupLauncher,
  createHarnessStatusReader,
  createStackStatusReader,
} from './ops.ts'
import { simulatePlan } from './simulator.ts'
import { ROBOT_SOUL_HTML } from './ui.ts'

const MAX_REQUEST_BYTES = 16 * 1024
const HKCLR_LOGO_JPEG = new URL('../../../../hkclr_logo.jpeg', import.meta.url)
const HKCLR_LOGO_AVIF = new URL('../../../../hkclr-logo-large.avif', import.meta.url)
const MEMORY_ROBOT_PNG = new URL(
  'file:///home/vlmdemo/.cursor/projects/mnt-ssd-user-V/assets/memory-robot-minimal.png',
)
const GET_ROUTES = new Set([
  '/',
  '/assets/hkclr-logo.jpeg',
  '/assets/hkclr-logo.avif',
  '/assets/memory-robot-minimal.png',
  '/api/content',
  '/api/catalog',
  '/api/architecture',
  '/api/hardware/policy',
  '/api/hardware/approvals',
  '/api/hardware/task/status',
  '/api/hardware/task/events',
  '/api/live/status',
  '/api/live/manifest',
  '/api/ops/status',
])
const POST_ROUTES = new Set([
  '/api/simulate/plan',
  '/api/hardware/plan',
  '/api/hardware/execute',
  '/api/hardware/approvals/approve',
  '/api/hardware/approvals/deny',
  '/api/hardware/task/cancel',
  '/api/ops/start',
  '/api/ops/stop',
  '/api/ops/recover',
])

export interface RobotSoulServerOptions {
  readonly liveReader?: LiveReader
  readonly hardwareController?: HardwareExecutionController
  readonly bootstrapStatusReader?: AgentBootstrapStatusReader
  readonly harnessStatusReader?: HarnessStatusReader
  readonly stackStatusReader?: StackStatusReader
  readonly startupLauncher?: HardwareStartupLauncher
  readonly stopLauncher?: HardwareStopLauncher
  readonly recoveryLauncher?: HardwareRecoveryLauncher
}

type JsonRecord = Readonly<Record<string, unknown>>

function commonHeaders(contentType: string): Record<string, string> {
  return {
    'content-type': contentType,
    'cache-control': 'no-store',
    'x-content-type-options': 'nosniff',
    'x-frame-options': 'DENY',
    'referrer-policy': 'no-referrer',
    'content-security-policy': [
      "default-src 'self'",
      "script-src 'unsafe-inline'",
      "style-src 'unsafe-inline'",
      "img-src 'self' data:",
      "connect-src 'self'",
      "frame-src 'none'",
      "font-src 'self'",
      "object-src 'none'",
      "base-uri 'none'",
      "frame-ancestors 'none'",
      "form-action 'self'",
    ].join('; '),
  }
}

function sendJson(response: ServerResponse, status: number, value: unknown): void {
  response.writeHead(status, commonHeaders('application/json; charset=utf-8'))
  response.end(JSON.stringify(value))
}

function sendHtml(response: ServerResponse): void {
  response.writeHead(200, commonHeaders('text/html; charset=utf-8'))
  response.end(ROBOT_SOUL_HTML)
}

async function sendImage(
  response: ServerResponse,
  source: URL,
  contentType: 'image/avif' | 'image/jpeg' | 'image/png',
): Promise<void> {
  const image = await readFile(source)
  response.writeHead(200, {
    ...commonHeaders(contentType),
    'cache-control': 'public, max-age=86400',
  })
  response.end(image)
}

function isRecord(value: unknown): value is JsonRecord {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

async function readJsonObject(request: IncomingMessage): Promise<JsonRecord> {
  const chunks: Buffer[] = []
  let bytes = 0
  for await (const chunk of request) {
    const buffer = Buffer.isBuffer(chunk) ? chunk : Buffer.from(String(chunk))
    bytes += buffer.length
    if (bytes > MAX_REQUEST_BYTES) {
      throw new RangeError(`Request body cannot exceed ${MAX_REQUEST_BYTES} bytes.`)
    }
    chunks.push(buffer)
  }
  let value: unknown
  try {
    value = JSON.parse(Buffer.concat(chunks).toString('utf8'))
  } catch (error) {
    throw new TypeError('Request body must be valid JSON.', { cause: error })
  }
  if (!isRecord(value)) throw new TypeError('Request body must be a JSON object.')
  return value
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error)
}

function isLoopbackHost(hostname: string): boolean {
  const unwrapped = hostname.startsWith('[') && hostname.endsWith(']')
    ? hostname.slice(1, -1)
    : hostname
  const version = isIP(unwrapped)
  if (version === 4) return unwrapped.split('.')[0] === '127'
  return version === 6 && unwrapped === '::1'
}

function requireOperatorPostBoundary(
  request: IncomingMessage,
  requireOrigin = false,
): void {
  const contentType = request.headers['content-type']
    ?.split(';', 1)[0]
    ?.trim()
    .toLowerCase()
  if (contentType !== 'application/json') {
    throw new HardwareAccessError(
      'ORIGIN_REJECTED',
      415,
      'Hardware write endpoints accept application/json only.',
    )
  }
  const fetchSite = request.headers['sec-fetch-site']
  if (
    fetchSite !== undefined
    && fetchSite !== 'same-origin'
    && fetchSite !== 'none'
  ) {
    throw new HardwareAccessError(
      'ORIGIN_REJECTED',
      403,
      'Hardware write endpoints reject cross-site browser requests.',
    )
  }
  const origin = request.headers.origin
  if (origin === undefined) {
    if (requireOrigin) {
      throw new HardwareAccessError(
        'ORIGIN_REJECTED',
        403,
        'Hardware recovery accepts only same-origin requests from this page.',
      )
    }
    return
  }
  let parsed: URL
  try {
    parsed = new URL(origin)
  } catch {
    throw new HardwareAccessError(
      'ORIGIN_REJECTED',
      403,
      'Origin must be a valid local HTTP address.',
    )
  }
  const originPort = parsed.port === '' ? 80 : Number(parsed.port)
  if (
    parsed.protocol !== 'http:'
    || !isLoopbackHost(parsed.hostname)
    || originPort !== request.socket.localPort
  ) {
    throw new HardwareAccessError(
      'ORIGIN_REJECTED',
      403,
      'Hardware write endpoints accept only same-origin requests from this loopback page.',
    )
  }
}

function methodNotAllowed(response: ServerResponse, pathname: string): boolean {
  if (!GET_ROUTES.has(pathname) && !POST_ROUTES.has(pathname)) return false
  const allow = GET_ROUTES.has(pathname) ? 'GET' : 'POST'
  response.writeHead(405, {
    ...commonHeaders('application/json; charset=utf-8'),
    allow,
  })
  response.end(JSON.stringify({
    ok: false,
    error: { code: 'METHOD_NOT_ALLOWED', message: `This endpoint accepts ${allow} only.` },
  }))
  return true
}

/** Create the staged Robot Soul introduction server without listening. */
export function createRobotSoulServer(options: RobotSoulServerOptions = {}) {
  const liveReader = options.liveReader ?? createConfiguredLiveReader()
  const bootstrapStatusReader = options.bootstrapStatusReader
    ?? createAgentBootstrapStatusReader()
  const harnessStatusReader = options.harnessStatusReader
    ?? createHarnessStatusReader()
  const stackStatusReader = options.stackStatusReader ?? createStackStatusReader()
  const startupLauncher = options.startupLauncher
    ?? createHardwareStartupLauncher()
  const stopLauncher = options.stopLauncher ?? createHardwareStopLauncher()
  const recoveryLauncher = options.recoveryLauncher
    ?? createHardwareRecoveryLauncher()
  const startupAvailable = options.startupLauncher !== undefined
    || process.env.ROBOT_SOUL_STARTUP_LAUNCHER !== undefined
  const stopAvailable = options.stopLauncher !== undefined
    || process.env.ROBOT_SOUL_STARTUP_LAUNCHER !== undefined
  let controllerPromise: Promise<HardwareExecutionController> | undefined
  const hardwareController = async (): Promise<HardwareExecutionController> => {
    if (options.hardwareController !== undefined) {
      return options.hardwareController
    }
    controllerPromise ??= createHardwareExecutionController()
    return controllerPromise
  }
  const handleRequest = async (
    request: IncomingMessage,
    response: ServerResponse,
  ): Promise<void> => {
    const url = new URL(
      request.url ?? '/',
      `http://${request.headers.host ?? '127.0.0.1'}`,
    )
    const pathname = url.pathname

    try {
      if (
        request.method === 'POST'
        && (
          pathname.startsWith('/api/hardware/')
          || pathname === '/api/ops/start'
          || pathname === '/api/ops/stop'
          || pathname === '/api/ops/recover'
        )
      ) {
        requireOperatorPostBoundary(
          request,
          pathname === '/api/ops/start'
            || pathname === '/api/ops/stop'
            || pathname === '/api/ops/recover',
        )
      }
      if (request.method === 'GET' && pathname === '/') {
        sendHtml(response)
        return
      }
      if (request.method === 'GET' && pathname === '/assets/hkclr-logo.jpeg') {
        await sendImage(response, HKCLR_LOGO_JPEG, 'image/jpeg')
        return
      }
      if (request.method === 'GET' && pathname === '/assets/hkclr-logo.avif') {
        await sendImage(response, HKCLR_LOGO_AVIF, 'image/avif')
        return
      }
      if (request.method === 'GET' && pathname === '/assets/memory-robot-minimal.png') {
        await sendImage(response, MEMORY_ROBOT_PNG, 'image/png')
        return
      }
      if (request.method === 'GET' && pathname === '/api/content') {
        sendJson(response, 200, { ok: true, data: SITE_CONTENT })
        return
      }
      if (request.method === 'GET' && pathname === '/api/catalog') {
        sendJson(response, 200, {
          ok: true,
          data: {
            products: PRODUCT_CATALOG,
            counts: {
              skills: PRODUCT_CATALOG.filter(item => item.kind === 'skill').length,
              tools: PRODUCT_CATALOG.filter(item => item.kind === 'tool').length,
              harnessTools: PRODUCT_CATALOG.filter(
                item => item.kind === 'harness-tool',
              ).length,
            },
          },
        })
        return
      }
      if (request.method === 'GET' && pathname === '/api/architecture') {
        sendJson(response, 200, {
          ok: true,
          data: { nodes: ARCHITECTURE_NODES, edges: ARCHITECTURE_EDGES },
        })
        return
      }
      if (request.method === 'GET' && pathname === '/api/ops/status') {
        const [agent, bootstrap, harness, stack] = await Promise.all([
          readLive('status', liveReader),
          bootstrapStatusReader.read(),
          harnessStatusReader.read(),
          stackStatusReader.read(),
        ])
        sendJson(response, 200, {
          ok: true,
          data: {
            policy: hardwareAccessPolicy(),
            agent,
            bootstrap,
            harness,
            stack,
            operations: { startupAvailable, stopAvailable },
            checkedAt: new Date().toISOString(),
          },
        })
        return
      }
      if (request.method === 'POST' && pathname === '/api/ops/start') {
        await readJsonObject(request)
        const policy = hardwareAccessPolicy()
        if (!policy.executeEnabled) {
          throw new HardwareAccessError(
            'HARDWARE_DISABLED',
            403,
            'Robot startup is unavailable until both server-side hardware gates are enabled.',
          )
        }
        const [agent, stack] = await Promise.all([
          readLive('status', liveReader),
          stackStatusReader.read(),
        ])
        const agentData = isRecord(agent.data) ? agent.data : {}
        if (agent.connected && agentData.ready === true) {
          throw new HardwareAccessError(
            'TASK_CONFLICT',
            409,
            'The supervised Python Agent already reports ready.',
          )
        }
        if (stack.state === 'faulted') {
          throw new HardwareAccessError(
            'TASK_CONFLICT',
            409,
            'The hardware stack is faulted; use recovery instead of cold start.',
          )
        }
        if (
          stack.state === 'recovering'
          || stack.state === 'starting'
          || stack.state === 'stopping'
        ) {
          throw new HardwareAccessError(
            'TASK_CONFLICT',
            409,
            'A hardware lifecycle operation is already in progress.',
          )
        }
        let startup: Awaited<ReturnType<HardwareStartupLauncher['start']>>
        try {
          startup = await startupLauncher.start()
        } catch (error) {
          throw new HardwareAccessError(
            'AGENT_NOT_READY',
            503,
            `Could not start the supervised robot stack: ${errorMessage(error)}`,
          )
        }
        controllerPromise = undefined
        sendJson(response, 202, {
          ok: true,
          data: {
            ...startup,
            state: 'starting',
            message: 'The supervised robot stack startup request was accepted.',
          },
        })
        return
      }
      if (request.method === 'POST' && pathname === '/api/ops/stop') {
        await readJsonObject(request)
        if (!stopAvailable) {
          throw new HardwareAccessError(
            'HARDWARE_DISABLED',
            403,
            'No reviewed local hardware stop launcher is configured.',
          )
        }
        const [agent, stack] = await Promise.all([
          readLive('status', liveReader),
          stackStatusReader.read(),
        ])
        if (stack.state === 'faulted') {
          throw new HardwareAccessError(
            'TASK_CONFLICT',
            409,
            'The hardware stack is faulted; inspect it and use recovery instead of clearing the fault.',
          )
        }
        if (
          stack.state === 'starting'
          || stack.state === 'recovering'
          || stack.state === 'stopping'
        ) {
          throw new HardwareAccessError(
            'TASK_CONFLICT',
            409,
            'A hardware lifecycle operation is already in progress.',
          )
        }
        if (stack.state === 'offline' && !agent.connected) {
          throw new HardwareAccessError(
            'TASK_CONFLICT',
            409,
            'The supervised robot stack is already offline.',
          )
        }
        let stopped: Awaited<ReturnType<HardwareStopLauncher['stop']>>
        try {
          stopped = await stopLauncher.stop()
        } catch (error) {
          throw new HardwareAccessError(
            'AGENT_NOT_READY',
            503,
            `Could not stop the supervised robot stack: ${errorMessage(error)}`,
          )
        }
        controllerPromise = undefined
        sendJson(response, 202, {
          ok: true,
          data: {
            ...stopped,
            state: 'stopping',
            message: 'Hardware shutdown was accepted; this page and Harness remain online.',
          },
        })
        return
      }
      if (request.method === 'POST' && pathname === '/api/ops/recover') {
        await readJsonObject(request)
        const stack = await stackStatusReader.read()
        if (stack.state !== 'faulted' || !stack.recoverable) {
          throw new HardwareAccessError(
            'TASK_CONFLICT',
            409,
            stack.state === 'recovering'
              ? 'Hardware recovery is already in progress.'
              : 'The hardware stack is not in a recoverable fault state.',
          )
        }
        let recovery: Awaited<ReturnType<HardwareRecoveryLauncher['start']>>
        try {
          recovery = await recoveryLauncher.start()
        } catch (error) {
          throw new HardwareAccessError(
            'AGENT_NOT_READY',
            503,
            `Could not start hardware recovery: ${errorMessage(error)}`,
          )
        }
        controllerPromise = undefined
        sendJson(response, 202, {
          ok: true,
          data: {
            ...recovery,
            state: 'recovering',
            message: 'Recovery was delegated to the local launcher; Harness and this page remain online.',
          },
        })
        return
      }
      if (request.method === 'GET' && pathname === '/api/hardware/policy') {
        sendJson(response, 200, {
          ok: true,
          data: hardwareAccessPolicy(),
        })
        return
      }
      if (request.method === 'POST' && pathname === '/api/hardware/plan') {
        const body = await readJsonObject(request)
        sendJson(response, 200, {
          ok: true,
          data: await (await hardwareController()).preview(body.instruction),
        })
        return
      }
      if (request.method === 'POST' && pathname === '/api/hardware/execute') {
        const body = await readJsonObject(request)
        sendJson(response, 202, {
          ok: true,
          data: await (await hardwareController()).execute({
            planId: body.planId,
            confirmation: body.confirmation,
            checklist: body.checklist,
          }),
        })
        return
      }
      if (request.method === 'GET' && pathname === '/api/hardware/approvals') {
        sendJson(response, 200, {
          ok: true,
          data: await (await hardwareController()).pendingApprovals(),
        })
        return
      }
      if (
        request.method === 'POST'
        && pathname === '/api/hardware/approvals/approve'
      ) {
        const body = await readJsonObject(request)
        sendJson(response, 202, {
          ok: true,
          data: await (await hardwareController()).approveRequest({
            requestId: body.requestId,
            confirmation: body.confirmation,
            checklist: body.checklist,
          }),
        })
        return
      }
      if (
        request.method === 'POST'
        && pathname === '/api/hardware/approvals/deny'
      ) {
        const body = await readJsonObject(request)
        sendJson(response, 200, {
          ok: true,
          data: await (await hardwareController()).denyRequest(body.requestId),
        })
        return
      }
      if (
        request.method === 'GET'
        && pathname === '/api/hardware/task/status'
      ) {
        sendJson(response, 200, {
          ok: true,
          data: await (await hardwareController()).status(
            url.searchParams.get('taskId'),
          ),
        })
        return
      }
      if (
        request.method === 'GET'
        && pathname === '/api/hardware/task/events'
      ) {
        sendJson(response, 200, {
          ok: true,
          data: await (await hardwareController()).events(
            url.searchParams.get('taskId'),
            url.searchParams.get('cursor') ?? 0,
          ),
        })
        return
      }
      if (
        request.method === 'POST'
        && pathname === '/api/hardware/task/cancel'
      ) {
        const body = await readJsonObject(request)
        sendJson(response, 202, {
          ok: true,
          data: await (await hardwareController()).cancel(body.taskId),
        })
        return
      }
      if (request.method === 'POST' && pathname === '/api/simulate/plan') {
        const body = await readJsonObject(request)
        sendJson(response, 200, { ok: true, data: simulatePlan(body.instruction) })
        return
      }
      if (request.method === 'GET' && pathname === '/api/live/status') {
        sendJson(response, 200, {
          ok: true,
          mode: 'live-readonly',
          fallback: 'simulation',
          data: await readLive('status', liveReader),
        })
        return
      }
      if (request.method === 'GET' && pathname === '/api/live/manifest') {
        sendJson(response, 200, {
          ok: true,
          mode: 'live-readonly',
          fallback: 'static-catalog',
          data: await readLive('manifest', liveReader),
        })
        return
      }
      if (methodNotAllowed(response, pathname)) return
      sendJson(response, 404, {
        ok: false,
        error: {
          code: 'NOT_FOUND',
          message: 'Endpoint not found. Direct execute, cancel, task, and arbitrary call routes are not exposed here.',
        },
      })
    } catch (error) {
      const status = error instanceof HardwareAccessError
        ? error.status
        : error instanceof RangeError
          ? 413
          : 400
      sendJson(response, status, {
        ok: false,
        error: {
          code: error instanceof HardwareAccessError
            ? error.code
            : status === 413
              ? 'PAYLOAD_TOO_LARGE'
              : 'INVALID_REQUEST',
          message: errorMessage(error),
        },
      })
    }
  }
  return createServer((request, response) => {
    void handleRequest(request, response)
  })
}
