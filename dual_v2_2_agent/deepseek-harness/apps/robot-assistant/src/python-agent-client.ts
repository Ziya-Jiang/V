import { createConnection, isIP, type Socket } from 'node:net'

const DEFAULT_MAX_RESPONSE_BYTES = 1024 * 1024

type JsonRecord = Readonly<Record<string, unknown>>

export type PythonAgentCursor = number

export interface PythonAgentClientConfig {
  readonly host: string
  readonly port: number
  readonly token?: string
  readonly timeoutMs: number
  readonly maxResponseBytes?: number
}

export interface PythonAgentPlanData extends JsonRecord {
  readonly planId: string
  readonly plan: unknown
}

export interface PythonAgentTaskData extends JsonRecord {
  readonly taskId: string
}

export interface PythonAgentEventsData extends JsonRecord {
  readonly events: readonly JsonRecord[]
  readonly nextCursor: PythonAgentCursor
}

export interface PythonAgentApi {
  health(): Promise<JsonRecord>
  manifest(): Promise<JsonRecord>
  plan(args: { readonly instruction: string, readonly output_dir: string }): Promise<PythonAgentPlanData>
  execute(args: { readonly planId: string, readonly confirmation: string }): Promise<PythonAgentTaskData>
  pendingExecutionRequests(): Promise<JsonRecord>
  approveExecution(args: {
    readonly requestId: string
    readonly confirmation: string
    readonly checklist: JsonRecord
  }): Promise<JsonRecord>
  denyExecution(args: { readonly requestId: string }): Promise<JsonRecord>
  taskStatus(args: { readonly taskId: string }): Promise<JsonRecord>
  taskEvents(args: {
    readonly taskId: string
    readonly cursor: PythonAgentCursor
  }): Promise<PythonAgentEventsData>
  cancel(args: { readonly taskId: string }): Promise<unknown>
}

export type PythonAgentErrorCode =
  | 'configuration'
  | 'connection'
  | 'timeout'
  | 'response-too-large'
  | 'protocol'
  | 'remote'

/** Error carrying a stable category suitable for an HTTP boundary. */
export class PythonAgentClientError extends Error {
  readonly code: PythonAgentErrorCode

  constructor(code: PythonAgentErrorCode, message: string, options?: ErrorOptions) {
    super(message, options)
    this.name = 'PythonAgentClientError'
    this.code = code
  }
}

function isRecord(value: unknown): value is JsonRecord {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

function isLoopbackAddress(host: string): boolean {
  const ipVersion = isIP(host)
  if (ipVersion === 4) return host.split('.')[0] === '127'
  return ipVersion === 6 && host === '::1'
}

function requireRecord(value: unknown, method: string): JsonRecord {
  if (!isRecord(value)) {
    throw new PythonAgentClientError(
      'protocol',
      `Python Agent ${method} response data must be an object.`,
    )
  }
  return value
}

function requireIdentifier(
  value: unknown,
  field: 'planId' | 'taskId',
  method: string,
): string {
  if (typeof value !== 'string' || value.length === 0 || value.length > 256) {
    throw new PythonAgentClientError(
      'protocol',
      `Python Agent ${method} response has an invalid ${field}.`,
    )
  }
  return value
}

/** One-request-per-connection NDJSON client for the loopback Python Agent. */
export class PythonAgentClient implements PythonAgentApi {
  readonly #config: Required<Omit<PythonAgentClientConfig, 'token'>> & Pick<PythonAgentClientConfig, 'token'>

  constructor(config: PythonAgentClientConfig) {
    if (!isLoopbackAddress(config.host)) {
      throw new PythonAgentClientError(
        'configuration',
        'ROBOT_AGENT_HOST must be a numeric loopback address (127.0.0.0/8 or ::1).',
      )
    }
    if (!Number.isInteger(config.port) || config.port < 1 || config.port > 65_535) {
      throw new PythonAgentClientError(
        'configuration',
        'ROBOT_AGENT_PORT must be an integer from 1 to 65535.',
      )
    }
    if (!Number.isInteger(config.timeoutMs) || config.timeoutMs < 1) {
      throw new PythonAgentClientError(
        'configuration',
        'ROBOT_AGENT_TIMEOUT_MS must be a positive integer.',
      )
    }
    const maxResponseBytes = config.maxResponseBytes ?? DEFAULT_MAX_RESPONSE_BYTES
    if (!Number.isInteger(maxResponseBytes) || maxResponseBytes < 1) {
      throw new PythonAgentClientError(
        'configuration',
        'Python Agent response limit must be a positive integer.',
      )
    }
    if (config.token !== undefined && config.token.length === 0) {
      throw new PythonAgentClientError(
        'configuration',
        'ROBOT_AGENT_TOKEN must be non-empty when provided.',
      )
    }
    this.#config = { ...config, maxResponseBytes }
  }

  async health(): Promise<JsonRecord> {
    return requireRecord(await this.#request('health', {}), 'health')
  }

  async manifest(): Promise<JsonRecord> {
    return requireRecord(await this.#request('manifest', {}), 'manifest')
  }

  async plan(args: {
    readonly instruction: string
    readonly output_dir: string
  }): Promise<PythonAgentPlanData> {
    const data = requireRecord(await this.#request('plan', args), 'plan')
    if (!('plan' in data)) {
      throw new PythonAgentClientError(
        'protocol',
        'Python Agent plan response must contain a plan field.',
      )
    }
    return {
      ...data,
      planId: requireIdentifier(data.planId, 'planId', 'plan'),
      plan: data.plan,
    }
  }

  async execute(args: {
    readonly planId: string
    readonly confirmation: string
  }): Promise<PythonAgentTaskData> {
    const data = requireRecord(await this.#request('execute', args), 'execute')
    return {
      ...data,
      taskId: requireIdentifier(data.taskId, 'taskId', 'execute'),
    }
  }

  async pendingExecutionRequests(): Promise<JsonRecord> {
    return requireRecord(
      await this.#request('pending_execution_requests', {}),
      'pending_execution_requests',
    )
  }

  async approveExecution(args: {
    readonly requestId: string
    readonly confirmation: string
    readonly checklist: JsonRecord
  }): Promise<JsonRecord> {
    return requireRecord(
      await this.#request('approve_execution', args),
      'approve_execution',
    )
  }

  async denyExecution(args: { readonly requestId: string }): Promise<JsonRecord> {
    return requireRecord(
      await this.#request('deny_execution', args),
      'deny_execution',
    )
  }

  async taskStatus(args: { readonly taskId: string }): Promise<JsonRecord> {
    return requireRecord(await this.#request('task_status', args), 'task_status')
  }

  async taskEvents(args: {
    readonly taskId: string
    readonly cursor: PythonAgentCursor
  }): Promise<PythonAgentEventsData> {
    const data = requireRecord(await this.#request('task_events', args), 'task_events')
    if (!Array.isArray(data.events) || !data.events.every(isRecord)) {
      throw new PythonAgentClientError(
        'protocol',
        'Python Agent task_events response must contain an events object array.',
      )
    }
    const nextCursor = data.nextCursor
    if (
      typeof nextCursor !== 'number'
      || !Number.isSafeInteger(nextCursor)
      || nextCursor < 0
    ) {
      throw new PythonAgentClientError(
        'protocol',
        'Python Agent task_events response has an invalid nextCursor.',
      )
    }
    return { ...data, events: data.events, nextCursor }
  }

  async cancel(args: { readonly taskId: string }): Promise<unknown> {
    return this.#request('cancel', args)
  }

  async #request(method: string, args: JsonRecord): Promise<unknown> {
    const request = {
      method,
      args,
      ...this.#config.token === undefined ? {} : { token: this.#config.token },
    }
    const payload = `${JSON.stringify(request)}\n`
    const response = await this.#exchange(method, payload)
    return this.#parseEnvelope(method, response)
  }

  #exchange(method: string, payload: string): Promise<string> {
    return new Promise<string>((resolve, reject) => {
      let socket: Socket | undefined
      let settled = false
      let received = Buffer.alloc(0)
      const timeout = setTimeout(() => {
        fail(new PythonAgentClientError(
          'timeout',
          `Python Agent ${method} timed out after ${this.#config.timeoutMs} ms.`,
        ))
      }, this.#config.timeoutMs)

      const cleanup = (): void => {
        clearTimeout(timeout)
        socket?.removeAllListeners()
        socket?.destroy()
      }
      const fail = (error: Error): void => {
        if (settled) return
        settled = true
        cleanup()
        reject(error)
      }
      const succeed = (value: string): void => {
        if (settled) return
        settled = true
        cleanup()
        resolve(value)
      }

      socket = createConnection({
        host: this.#config.host,
        port: this.#config.port,
      })
      socket.once('connect', () => socket?.end(payload))
      socket.on('data', (chunk: Buffer) => {
        received = Buffer.concat([received, chunk])
        const newlineIndex = received.indexOf(0x0A)
        if (received.length > this.#config.maxResponseBytes + 1) {
          fail(new PythonAgentClientError(
            'response-too-large',
            `Python Agent ${method} response exceeds ${this.#config.maxResponseBytes} bytes.`,
          ))
          return
        }
        if (newlineIndex < 0) return
        const trailing = received.subarray(newlineIndex + 1).toString('utf8').trim()
        if (trailing.length > 0) {
          fail(new PythonAgentClientError(
            'protocol',
            `Python Agent ${method} returned more than one NDJSON response.`,
          ))
          return
        }
        try {
          const decoder = new TextDecoder('utf-8', { fatal: true })
          succeed(decoder.decode(received.subarray(0, newlineIndex)))
        } catch (error) {
          fail(new PythonAgentClientError(
            'protocol',
            `Python Agent ${method} returned invalid UTF-8.`,
            { cause: error },
          ))
        }
      })
      socket.once('end', () => {
        if (!settled) {
          fail(new PythonAgentClientError(
            'protocol',
            `Python Agent ${method} closed the connection before a complete NDJSON response.`,
          ))
        }
      })
      socket.once('error', (error: NodeJS.ErrnoException) => {
        fail(new PythonAgentClientError(
          'connection',
          `Could not reach Python Agent at ${this.#config.host}:${this.#config.port}: ${error.code ?? error.message}.`,
          { cause: error },
        ))
      })
    })
  }

  #parseEnvelope(method: string, text: string): unknown {
    let value: unknown
    try {
      value = JSON.parse(text)
    } catch (error) {
      throw new PythonAgentClientError(
        'protocol',
        `Python Agent ${method} returned invalid JSON.`,
        { cause: error },
      )
    }
    if (!isRecord(value) || typeof value.ok !== 'boolean') {
      throw new PythonAgentClientError(
        'protocol',
        `Python Agent ${method} response must contain a boolean ok field.`,
      )
    }
    if (value.message !== undefined && typeof value.message !== 'string') {
      throw new PythonAgentClientError(
        'protocol',
        `Python Agent ${method} response has an invalid message field.`,
      )
    }
    if (value.error !== undefined && typeof value.error !== 'string') {
      throw new PythonAgentClientError(
        'protocol',
        `Python Agent ${method} response has an invalid error field.`,
      )
    }
    if (!value.ok) {
      const detail = value.error ?? value.message ?? 'request was rejected'
      throw new PythonAgentClientError(
        'remote',
        `Python Agent ${method} failed: ${detail}`,
      )
    }
    return value.data
  }
}
