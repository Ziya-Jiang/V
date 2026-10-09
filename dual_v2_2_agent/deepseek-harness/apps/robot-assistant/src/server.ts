import { createServer, type ServerResponse } from 'node:http'

import type {
  ExecutionEvent,
  RobotDriver,
  TaskPlanningProvider,
  TaskRequest,
} from './contracts.ts'
import { TaskExecutor } from './executor.ts'
import {
  PythonAgentClientError,
  type PythonAgentApi,
  type PythonAgentCursor,
} from './python-agent-client.ts'
import { SkillRegistry } from './skill-registry.ts'
import { ROBOT_ASSISTANT_HTML } from './ui.ts'

const MAX_REQUEST_BYTES = 16 * 1024
const HARDWARE_CONFIRMATION = 'ENABLE_DUAL_V2_2_HARDWARE'

type JsonRecord = Readonly<Record<string, unknown>>

interface StreamEvent extends JsonRecord {
  readonly timestamp: string
  readonly taskId: string
  readonly type: string
  readonly message: string
  readonly status?: string
}

interface AgentReadiness {
  readonly status: 'loading' | 'ready' | 'error'
  readonly ready: boolean
  readonly details?: JsonRecord
  readonly error?: string
}

interface ActivePreview {
  readonly status: 'planning' | 'ready' | 'confirming'
  readonly instruction: string
  readonly planId?: string
  readonly plan?: unknown
}

interface ActivePythonTask {
  readonly taskId: string
  readonly planId: string
  readonly status: string
  readonly startedAt: string
}

export interface PythonAgentServerOptions {
  readonly pollIntervalMs?: number
}

function sendJson(response: ServerResponse, status: number, value: unknown): void {
  response.writeHead(status, { 'content-type': 'application/json; charset=utf-8' })
  response.end(JSON.stringify(value))
}

function isRecord(value: unknown): value is JsonRecord {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

async function readJsonObject(request: NodeJS.ReadableStream): Promise<JsonRecord> {
  const chunks: Buffer[] = []
  let bytes = 0
  for await (const chunk of request) {
    const buffer = Buffer.isBuffer(chunk) ? chunk : Buffer.from(String(chunk))
    bytes += buffer.length
    if (bytes > MAX_REQUEST_BYTES) {
      throw new Error('Request exceeds 16 KiB.')
    }
    chunks.push(buffer)
  }
  let value: unknown
  try {
    value = JSON.parse(Buffer.concat(chunks).toString('utf8'))
  } catch (error) {
    throw new Error('Request body must be valid JSON.', { cause: error })
  }
  if (!isRecord(value)) throw new Error('Request body must be a JSON object.')
  return value
}

async function readTaskRequest(request: NodeJS.ReadableStream): Promise<TaskRequest> {
  const value = await readJsonObject(request)
  if (typeof value.text !== 'string') {
    throw new Error('Request must contain a string text field.')
  }
  return { text: value.text }
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error)
}

function pythonAgentHttpStatus(error: unknown): number {
  if (!(error instanceof PythonAgentClientError)) return 400
  return error.code === 'timeout' ? 504 : 502
}

function normalizePythonEvent(event: JsonRecord, taskId: string): StreamEvent {
  const type = typeof event.type === 'string' ? event.type : 'task.event'
  const message = typeof event.message === 'string'
    ? event.message
    : typeof event.detail === 'string'
      ? event.detail
      : type
  return {
    ...event,
    timestamp: typeof event.timestamp === 'string'
      ? event.timestamp
      : new Date().toISOString(),
    taskId: typeof event.taskId === 'string' ? event.taskId : taskId,
    type,
    message,
    ...typeof event.status === 'string' ? { status: event.status } : {},
  }
}

function statusFromData(data: JsonRecord, fallback: string): string {
  const value = data.status ?? data.state ?? data.bootstrapState
  return typeof value === 'string' && value.length > 0 ? value : fallback
}

function isTerminalStatus(status: string): boolean {
  return new Set([
    'succeeded',
    'success',
    'completed',
    'failed',
    'error',
    'cancelled',
    'canceled',
  ]).has(status.toLowerCase())
}

/** Start the outer-layer HTTP/SSE interface around the planner and executor. */
export function startRobotAssistantServer(
  registry: SkillRegistry,
  planner: TaskPlanningProvider,
  plannerMode: 'deterministic' | 'deepseek',
  driver: RobotDriver,
  driverMode: 'dry-run' | 'robotram',
  port: number,
): ReturnType<typeof createServer> {
  const eventHistory: ExecutionEvent[] = []
  const clients = new Set<ServerResponse>()
  let activeTaskId: string | undefined
  const emit = (event: ExecutionEvent): void => {
    eventHistory.push(event)
    if (eventHistory.length > 200) eventHistory.shift()
    const payload = `data: ${JSON.stringify(event)}\n\n`
    for (const client of clients) client.write(payload)
  }
  const executor = new TaskExecutor(registry, driver, emit)

  const server = createServer(async (request, response) => {
    const url = new URL(request.url ?? '/', `http://${request.headers.host ?? 'localhost'}`)
    if (request.method === 'GET' && url.pathname === '/') {
      response.writeHead(200, { 'content-type': 'text/html; charset=utf-8' })
      response.end(ROBOT_ASSISTANT_HTML)
      return
    }
    if (request.method === 'GET' && url.pathname === '/api/state') {
      sendJson(response, 200, {
        activeTaskId,
        driverMode,
        plannerMode,
        runtime: 'mvp',
        skills: registry.list(),
      })
      return
    }
    if (request.method === 'GET' && url.pathname === '/api/events') {
      response.writeHead(200, {
        'content-type': 'text/event-stream',
        'cache-control': 'no-cache',
        connection: 'keep-alive',
      })
      for (const event of eventHistory) {
        response.write(`data: ${JSON.stringify(event)}\n\n`)
      }
      clients.add(response)
      request.once('close', () => clients.delete(response))
      return
    }
    if (request.method === 'POST' && url.pathname === '/api/tasks') {
      if (activeTaskId !== undefined) {
        sendJson(response, 409, {
          error: `Task ${activeTaskId} is still active.`,
        })
        return
      }
      activeTaskId = 'planning'
      try {
        const taskRequest = await readTaskRequest(request)
        const plan = await planner.plan(taskRequest)
        activeTaskId = plan.id
        emit({
          timestamp: new Date().toISOString(),
          taskId: plan.id,
          type: 'plan.created',
          status: 'ready',
          message: `TaskIR created with ${plan.steps.length} atomic operations.`,
        })
        sendJson(response, 202, { plan })
        void executor.execute(plan, new AbortController().signal).finally(() => {
          if (activeTaskId === plan.id) activeTaskId = undefined
        })
      } catch (error) {
        activeTaskId = undefined
        sendJson(response, 400, {
          error: error instanceof Error ? error.message : String(error),
        })
      }
      return
    }
    sendJson(response, 404, { error: 'Not found.' })
  })

  server.listen(port, '127.0.0.1')
  return server
}

/** Start the confirmation-gated HTTP/SSE facade for the loopback Python Agent. */
export function startPythonAgentServer(
  client: PythonAgentApi,
  outputDir: string,
  port: number,
  options: PythonAgentServerOptions = {},
): ReturnType<typeof createServer> {
  const pollIntervalMs = options.pollIntervalMs ?? 350
  if (!Number.isInteger(pollIntervalMs) || pollIntervalMs < 1) {
    throw new Error('Python Agent polling interval must be a positive integer.')
  }

  const eventHistory: StreamEvent[] = []
  const clients = new Set<ServerResponse>()
  let agent: AgentReadiness = { status: 'loading', ready: false }
  let activePreview: ActivePreview | undefined
  let activeTask: ActivePythonTask | undefined
  let closed = false
  const delays = new Map<NodeJS.Timeout, () => void>()

  const emit = (event: StreamEvent): void => {
    eventHistory.push(event)
    if (eventHistory.length > 200) eventHistory.shift()
    const payload = `data: ${JSON.stringify(event)}\n\n`
    for (const stream of clients) stream.write(payload)
  }
  const delay = async (): Promise<void> => {
    await new Promise<void>((resolve) => {
      const timer = setTimeout(() => {
        delays.delete(timer)
        resolve()
      }, pollIntervalMs)
      delays.set(timer, resolve)
    })
  }
  const setAgentError = (error: unknown): void => {
    agent = { status: 'error', ready: false, error: errorMessage(error) }
  }
  const refreshHealth = async (): Promise<void> => {
    try {
      const details = await client.health()
      const status = statusFromData(details, details.ready === false ? 'loading' : 'ready')
      if (status.toLowerCase() === 'error' || typeof details.error === 'string') {
        agent = {
          status: 'error',
          ready: false,
          details,
          error: typeof details.error === 'string' ? details.error : 'Python Agent reported an error.',
        }
      } else if (
        details.ready === false
        || ['loading', 'not_started', 'initializing'].includes(status.toLowerCase())
      ) {
        agent = { status: 'loading', ready: false, details }
      } else {
        agent = { status: 'ready', ready: true, details }
      }
    } catch (error) {
      setAgentError(error)
    }
  }
  const pollTask = async (taskId: string): Promise<void> => {
    let cursor: PythonAgentCursor = 0
    while (!closed && activeTask?.taskId === taskId) {
      try {
        const batch = await client.taskEvents({ taskId, cursor })
        for (const event of batch.events) emit(normalizePythonEvent(event, taskId))
        cursor = batch.nextCursor

        const statusData = await client.taskStatus({ taskId })
        const current = activeTask
        if (current?.taskId !== taskId) return
        const observedStatus = statusFromData(statusData, current.status)
        const nextStatus = current.status === 'canceling' && !isTerminalStatus(observedStatus)
          ? current.status
          : observedStatus
        if (nextStatus !== current.status) {
          activeTask = { ...current, status: nextStatus }
          emit({
            timestamp: new Date().toISOString(),
            taskId,
            type: 'task.status',
            status: nextStatus,
            message: `Python Agent task is ${nextStatus}.`,
          })
        }
        if (isTerminalStatus(nextStatus)) {
          activeTask = undefined
          return
        }
        await delay()
      } catch (error) {
        if (closed || activeTask?.taskId !== taskId) return
        setAgentError(error)
        emit({
          timestamp: new Date().toISOString(),
          taskId,
          type: 'task.status',
          status: 'failed',
          message: errorMessage(error),
        })
        activeTask = undefined
        return
      }
    }
  }

  const server = createServer(async (request, response) => {
    const url = new URL(request.url ?? '/', 'http://127.0.0.1')
    if (request.method === 'GET' && url.pathname === '/') {
      response.writeHead(200, { 'content-type': 'text/html; charset=utf-8' })
      response.end(ROBOT_ASSISTANT_HTML)
      return
    }
    if (request.method === 'GET' && url.pathname === '/api/state') {
      sendJson(response, 200, {
        runtime: 'python-agent',
        agent,
        activePreview: activePreview ?? null,
        activeTask: activeTask ?? null,
        confirmation: HARDWARE_CONFIRMATION,
        outputDir,
      })
      return
    }
    if (request.method === 'GET' && url.pathname === '/api/events') {
      response.writeHead(200, {
        'content-type': 'text/event-stream',
        'cache-control': 'no-cache',
        connection: 'keep-alive',
      })
      response.write(': connected\n\n')
      for (const event of eventHistory) {
        response.write(`data: ${JSON.stringify(event)}\n\n`)
      }
      clients.add(response)
      request.once('close', () => clients.delete(response))
      return
    }
    if (request.method === 'POST' && url.pathname === '/api/tasks') {
      if (activePreview !== undefined || activeTask !== undefined) {
        const activeId = activeTask?.taskId ?? activePreview?.planId ?? 'planning'
        sendJson(response, 409, { error: `Another preview or task is active (${activeId}).` })
        return
      }
      activePreview = { status: 'planning', instruction: '' }
      let instruction = ''
      try {
        const body = await readJsonObject(request)
        const value = body.instruction ?? body.text
        if (typeof value !== 'string' || value.trim().length === 0) {
          throw new Error('Request must contain a non-empty instruction or text field.')
        }
        if (value.length > 4_096) throw new Error('Instruction exceeds 4096 characters.')
        instruction = value.trim()
      } catch (error) {
        activePreview = undefined
        sendJson(response, 400, { error: errorMessage(error) })
        return
      }

      activePreview = { status: 'planning', instruction }
      try {
        const result = await client.plan({ instruction, output_dir: outputDir })
        activePreview = {
          status: 'ready',
          instruction,
          planId: result.planId,
          plan: result.plan,
        }
        emit({
          timestamp: new Date().toISOString(),
          taskId: result.planId,
          type: 'plan.created',
          status: 'ready',
          message: 'Python Agent plan is ready for explicit confirmation.',
        })
        sendJson(response, 202, { ...result, preview: true })
      } catch (error) {
        activePreview = undefined
        setAgentError(error)
        sendJson(response, pythonAgentHttpStatus(error), { error: errorMessage(error) })
      }
      return
    }

    const discardMatch = /^\/api\/tasks\/([^/]+)$/.exec(url.pathname)
    if (request.method === 'DELETE' && discardMatch !== null) {
      let planId: string
      try {
        planId = decodeURIComponent(discardMatch[1] ?? '')
      } catch {
        sendJson(response, 400, { error: 'Plan id is not valid URL encoding.' })
        return
      }
      if (activePreview?.planId !== planId || activeTask !== undefined) {
        sendJson(response, 409, { error: `Plan ${planId} is not the active preview.` })
        return
      }
      activePreview = undefined
      sendJson(response, 200, { planId, discarded: true })
      return
    }

    const confirmMatch = /^\/api\/tasks\/([^/]+)\/confirm$/.exec(url.pathname)
    if (request.method === 'POST' && confirmMatch !== null) {
      let planId: string
      try {
        planId = decodeURIComponent(confirmMatch[1] ?? '')
      } catch {
        sendJson(response, 400, { error: 'Plan id is not valid URL encoding.' })
        return
      }
      const preview = activePreview
      if (preview?.status !== 'ready' || preview.planId !== planId || activeTask !== undefined) {
        sendJson(response, 409, { error: `Plan ${planId} is not the active ready preview.` })
        return
      }
      if (!agent.ready) {
        sendJson(response, 409, {
          error: 'Python Agent is not ready for hardware execution.',
        })
        return
      }
      activePreview = { ...preview, status: 'confirming' }
      let confirmation: string
      try {
        const body = await readJsonObject(request)
        if (body.confirmation !== HARDWARE_CONFIRMATION) {
          throw new Error(`confirmation must equal ${HARDWARE_CONFIRMATION}.`)
        }
        confirmation = body.confirmation
      } catch (error) {
        activePreview = preview
        sendJson(response, 400, { error: errorMessage(error) })
        return
      }

      try {
        const result = await client.execute({ planId, confirmation })
        activePreview = undefined
        activeTask = {
          taskId: result.taskId,
          planId,
          status: 'queued',
          startedAt: new Date().toISOString(),
        }
        emit({
          timestamp: new Date().toISOString(),
          taskId: result.taskId,
          type: 'task.status',
          status: 'queued',
          message: 'Execution accepted by Python Agent.',
        })
        sendJson(response, 202, { ...result, planId })
        void pollTask(result.taskId)
      } catch (error) {
        activePreview = preview
        setAgentError(error)
        sendJson(response, pythonAgentHttpStatus(error), { error: errorMessage(error) })
      }
      return
    }

    const cancelMatch = /^\/api\/tasks\/([^/]+)\/cancel$/.exec(url.pathname)
    if (request.method === 'POST' && cancelMatch !== null) {
      let taskId: string
      try {
        taskId = decodeURIComponent(cancelMatch[1] ?? '')
      } catch {
        sendJson(response, 400, { error: 'Task id is not valid URL encoding.' })
        return
      }
      const task = activeTask
      if (task?.taskId !== taskId) {
        sendJson(response, 409, { error: `Task ${taskId} is not active.` })
        return
      }
      if (task.status === 'canceling') {
        sendJson(response, 409, { error: `Task ${taskId} cancellation is already pending.` })
        return
      }
      activeTask = { ...task, status: 'canceling' }
      try {
        const data = await client.cancel({ taskId })
        emit({
          timestamp: new Date().toISOString(),
          taskId,
          type: 'task.status',
          status: 'canceling',
          message: 'Cancellation requested. This is not a physical emergency stop.',
        })
        sendJson(response, 202, { taskId, cancelRequested: true, data })
      } catch (error) {
        activeTask = task
        setAgentError(error)
        sendJson(response, pythonAgentHttpStatus(error), { error: errorMessage(error) })
      }
      return
    }

    sendJson(response, 404, { error: 'Not found.' })
  })

  server.once('close', () => {
    closed = true
    for (const [timer, resolve] of delays) {
      clearTimeout(timer)
      resolve()
    }
    delays.clear()
    for (const stream of clients) stream.end()
    clients.clear()
  })
  server.listen(port, '127.0.0.1')
  void refreshHealth()
  return server
}
