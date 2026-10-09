/**
 * Loopback NDJSON Service Provider for the robot capability seam.
 * It defaults to strict plan-only mode. An explicit configuration gate can
 * additionally expose model-created execution requests that remain inert until
 * a separate human approval surface approves the exact server-side Plan.
 * @module @deepseek-ai/dsh-robot-python-agent
 */

import { createConnection, isIP } from 'node:net'
import { isAbsolute } from 'node:path'

import { Context } from '@deepseek-ai/cordis'
import z from '@deepseek-ai/schemastery'
import {
  RobotError,
  RobotRuntime,
  type RobotExecutionRequest,
  type RobotExecutionRequestInput,
  type RobotExecutionStatusInput,
  type RobotJsonObject,
  type RobotJsonValue,
  type RobotManifest,
  type RobotPlan,
  type RobotPlanRequest,
  type RobotStatus,
} from '@deepseek-ai/dsh-robot'

const PROVIDER_ID = 'python-agent'
const DEFAULT_MAX_RESPONSE_BYTES = 1024 * 1024
const DEFAULT_MAX_INSTRUCTION_BYTES = 16 * 1024
const PLAN_ONLY_METHODS = ['health', 'manifest', 'plan'] as const

/** Connection, mode, and input limits for the Python Agent provider. */
export interface Config {
  /** Numeric loopback address of the Python Agent. */
  readonly host?: string
  /** TCP port of the Python Agent. */
  readonly port?: number
  /** Optional shared token included in every NDJSON request. */
  readonly token?: string
  /** Per-request connection and response deadline in milliseconds. */
  readonly timeoutMs?: number
  /** Maximum accepted NDJSON response size in bytes. */
  readonly maxResponseBytes?: number
  /** Maximum semantic planning instruction size in UTF-8 bytes. */
  readonly maxInstructionBytes?: number
  /** Fixed absolute scene/output directory sent with every planning request. */
  readonly outputDir?: string
  /** Allow model-created requests that still require separate human approval. */
  readonly approvalGatedExecution?: boolean
}

interface ResolvedConfig {
  readonly host: string
  readonly port: number
  readonly token?: string
  readonly timeoutMs: number
  readonly maxResponseBytes: number
  readonly maxInstructionBytes: number
  readonly outputDir: string
  readonly approvalGatedExecution: boolean
}

/** Schemastery configuration for the Python Agent provider. */
export const Config: z<Config> = z.object({
  host: z.string().default('127.0.0.1'),
  port: z.number().default(15_555),
  token: z.string(),
  timeoutMs: z.number().default(30_000),
  maxResponseBytes: z.number().default(DEFAULT_MAX_RESPONSE_BYTES),
  maxInstructionBytes: z.number().default(DEFAULT_MAX_INSTRUCTION_BYTES),
  outputDir: z.string().default(process.cwd()),
  approvalGatedExecution: z.boolean().default(false),
})

function isLoopbackAddress(host: string): boolean {
  const version = isIP(host)
  if (version === 4) return host.split('.')[0] === '127'
  return version === 6 && host === '::1'
}

function positiveInteger(name: string, value: number): number {
  if (!Number.isSafeInteger(value) || value < 1) {
    throw new RobotError(
      'ROBOT_CONFIGURATION',
      `robot-python-agent: ${name} must be a positive safe integer`,
    )
  }
  return value
}

function resolveConfig(config: Config): ResolvedConfig {
  const host = config.host ?? '127.0.0.1'
  if (!isLoopbackAddress(host)) {
    throw new RobotError(
      'ROBOT_CONFIGURATION',
      'robot-python-agent: host must be a numeric loopback address (127.0.0.0/8 or ::1)',
    )
  }
  const token = config.token
  if (token !== undefined && token.length === 0) {
    throw new RobotError(
      'ROBOT_CONFIGURATION',
      'robot-python-agent: token must be non-empty when configured',
    )
  }
  const outputDir = config.outputDir ?? process.cwd()
  if (!isAbsolute(outputDir)) {
    throw new RobotError(
      'ROBOT_CONFIGURATION',
      'robot-python-agent: outputDir must be an absolute path',
    )
  }
  const port = positiveInteger('port', config.port ?? 15_555)
  if (port > 65_535) {
    throw new RobotError(
      'ROBOT_CONFIGURATION',
      'robot-python-agent: port must be at most 65535',
    )
  }
  return {
    host,
    port,
    timeoutMs: positiveInteger('timeoutMs', config.timeoutMs ?? 30_000),
    maxResponseBytes: positiveInteger(
      'maxResponseBytes',
      config.maxResponseBytes ?? DEFAULT_MAX_RESPONSE_BYTES,
    ),
    maxInstructionBytes: positiveInteger(
      'maxInstructionBytes',
      config.maxInstructionBytes ?? DEFAULT_MAX_INSTRUCTION_BYTES,
    ),
    outputDir,
    approvalGatedExecution: config.approvalGatedExecution ?? false,
    ...token === undefined ? {} : { token },
  }
}

function isObject(value: unknown): value is RobotJsonObject {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

function requireObject(
  value: RobotJsonValue,
  method: string,
): RobotJsonObject {
  if (!isObject(value)) {
    throw new RobotError(
      'ROBOT_PROTOCOL',
      `Python Agent ${method} response data must be an object`,
    )
  }
  return value
}

function requireString(
  value: RobotJsonValue | undefined,
  field: string,
  method: string,
  maxLength = 4096,
): string {
  if (
    typeof value !== 'string'
    || value.length === 0
    || value.length > maxLength
  ) {
    throw new RobotError(
      'ROBOT_PROTOCOL',
      `Python Agent ${method} response has an invalid ${field}`,
    )
  }
  return value
}

function planOnlyReason(health: RobotJsonObject): string | undefined {
  if (health.planOnly !== true) {
    return 'Python Agent must be started with ROBOT_AGENT_PLAN_ONLY=1'
  }
  const allowed = health.allowedMethods
  if (
    !Array.isArray(allowed)
    || allowed.length !== PLAN_ONLY_METHODS.length
    || !PLAN_ONLY_METHODS.every(method => allowed.includes(method))
  ) {
    return 'Python Agent did not advertise the exact plan-only method set'
  }
  if (health.eagerBootstrap !== false) {
    return 'Python Agent plan-only mode must disable eager hardware bootstrap'
  }
  if (health.bootstrapState !== 'not_started') {
    const state = health.bootstrapState
    const renderedState = typeof state === 'string'
      ? state
      : state === undefined
        ? 'missing'
        : JSON.stringify(state)
    return `Python Agent hardware bootstrap state is ${renderedState}`
  }
  if (health.activeTaskId !== null) {
    return 'Python Agent reports an active or unverifiable hardware task'
  }
  return undefined
}

function approvalExecutionReason(health: RobotJsonObject): string | undefined {
  if (health.planOnly !== false) {
    return 'Approval-gated execution requires a production Python Agent'
  }
  const allowed = health.allowedMethods
  if (
    !Array.isArray(allowed)
    || ![
      'health',
      'manifest',
      'plan',
      'request_execution',
      'execution_request_status',
    ].every(method => allowed.includes(method))
  ) {
    return 'Python Agent does not advertise the approval-request protocol'
  }
  if (health.bootstrapState !== 'ready') {
    return 'Python Agent hardware bootstrap must report ready'
  }
  if (health.activeTaskId !== null) {
    return 'Python Agent reports an active or unverifiable hardware task'
  }
  return undefined
}

/** RobotRuntime implementation over the existing loopback Python Agent. */
export class PythonAgentRobotRuntime extends RobotRuntime {
  static Config = Config

  readonly #config: ResolvedConfig
  readonly #executionRequestIds = new Set<string>()

  constructor(ctx: Context, config: Config = {}) {
    super(ctx)
    this.#config = resolveConfig(config)
  }

  async status(signal?: AbortSignal): Promise<RobotStatus> {
    const health = requireObject(
      await this.#request('health', {}, signal),
      'health',
    )
    const reason = planOnlyReason(health)
    const executionReason = approvalExecutionReason(health)
    const mode = this.#config.approvalGatedExecution
      ? 'approval-gated'
      : 'dry-run'
    return {
      mode,
      provider: PROVIDER_ID,
      planOnlyReady: !this.#config.approvalGatedExecution && reason === undefined,
      executionReady: this.#config.approvalGatedExecution
        && executionReason === undefined,
      ...this.#config.approvalGatedExecution
        ? executionReason === undefined ? {} : { reason: executionReason }
        : reason === undefined ? {} : { reason },
      health,
    }
  }

  async manifest(signal?: AbortSignal): Promise<RobotManifest> {
    await this.#requireConfiguredMode(signal)
    const data = requireObject(
      await this.#request('manifest', {}, signal),
      'manifest',
    )
    if (!Array.isArray(data.tools) || !Array.isArray(data.skills)) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        'Python Agent manifest response must contain tools and skills arrays',
      )
    }
    return {
      mode: this.#config.approvalGatedExecution
        ? 'approval-gated'
        : 'dry-run',
      provider: PROVIDER_ID,
      tools: data.tools,
      skills: data.skills,
    }
  }

  async plan(
    request: RobotPlanRequest,
    signal?: AbortSignal,
  ): Promise<RobotPlan> {
    const instruction = request.instruction.trim()
    if (instruction.length === 0) {
      throw new RobotError(
        'ROBOT_CONFIGURATION',
        'robot plan instruction must be non-empty',
      )
    }
    if (Buffer.byteLength(instruction, 'utf8') > this.#config.maxInstructionBytes) {
      throw new RobotError(
        'ROBOT_CONFIGURATION',
        `robot plan instruction exceeds ${this.#config.maxInstructionBytes} UTF-8 bytes`,
      )
    }
    await this.#requireConfiguredMode(signal)
    const data = requireObject(
      await this.#request('plan', {
        instruction,
        output_dir: this.#config.outputDir,
      }, signal),
      'plan',
    )
    const planId = requireString(data.planId, 'planId', 'plan', 256)
    const echoedInstruction = requireString(
      data.instruction,
      'instruction',
      'plan',
      this.#config.maxInstructionBytes,
    )
    if (echoedInstruction !== instruction) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        'Python Agent plan response instruction does not match the request',
      )
    }
    if (data.output_dir !== this.#config.outputDir) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        'Python Agent plan response output_dir does not match provider configuration',
      )
    }
    if (!isObject(data.plan)) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        'Python Agent plan response must contain a plan object',
      )
    }
    const createdAt = requireString(data.createdAt, 'createdAt', 'plan')
    return {
      mode: this.#config.approvalGatedExecution
        ? 'approval-gated'
        : 'dry-run',
      provider: PROVIDER_ID,
      planId,
      instruction,
      outputDir: this.#config.outputDir,
      createdAt,
      plan: data.plan,
    }
  }

  override async requestExecution(
    request: RobotExecutionRequestInput,
    signal?: AbortSignal,
  ): Promise<RobotExecutionRequest> {
    if (!this.#config.approvalGatedExecution) {
      throw new RobotError(
        'ROBOT_EXECUTION_NOT_ENABLED',
        'robot-python-agent: approvalGatedExecution is not enabled',
      )
    }
    const planId = request.planId.trim()
    if (planId.length === 0 || planId.length > 256) {
      throw new RobotError(
        'ROBOT_CONFIGURATION',
        'robot execution planId must be a non-empty string up to 256 characters',
      )
    }
    await this.#requireConfiguredMode(signal)
    const data = requireObject(
      await this.#request('request_execution', { planId }, signal),
      'request_execution',
    )
    const result = this.#parseExecutionRequest(data, 'request_execution')
    if (result.planId !== planId) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        'Python Agent execution request planId does not match the request',
      )
    }
    this.#executionRequestIds.add(result.requestId)
    return result
  }

  override async executionStatus(
    request: RobotExecutionStatusInput,
    signal?: AbortSignal,
  ): Promise<RobotExecutionRequest> {
    if (!this.#config.approvalGatedExecution) {
      throw new RobotError(
        'ROBOT_EXECUTION_NOT_ENABLED',
        'robot-python-agent: approvalGatedExecution is not enabled',
      )
    }
    const requestId = request.requestId.trim()
    if (!this.#executionRequestIds.has(requestId)) {
      throw new RobotError(
        'ROBOT_CONFIGURATION',
        'robot execution requestId was not created by this provider',
      )
    }
    const data = requireObject(
      await this.#request(
        'execution_request_status',
        { requestId },
        signal,
      ),
      'execution_request_status',
    )
    const result = this.#parseExecutionRequest(
      data,
      'execution_request_status',
    )
    if (result.requestId !== requestId) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        'Python Agent execution status requestId does not match the request',
      )
    }
    return result
  }

  #parseExecutionRequest(
    data: RobotJsonObject,
    method: string,
  ): RobotExecutionRequest {
    const requestId = requireString(data.requestId, 'requestId', method, 256)
    const planId = requireString(data.planId, 'planId', method, 256)
    const status = requireString(data.status, 'status', method, 64)
    const instruction = requireString(
      data.instruction,
      'instruction',
      method,
      this.#config.maxInstructionBytes,
    )
    const createdAt = requireString(data.createdAt, 'createdAt', method)
    if (
      typeof data.expiresInSeconds !== 'number'
      || !Number.isSafeInteger(data.expiresInSeconds)
      || data.expiresInSeconds < 1
    ) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        `Python Agent ${method} response has an invalid expiresInSeconds`,
      )
    }
    if (!isObject(data.plan)) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        `Python Agent ${method} response must contain a plan object`,
      )
    }
    const taskId = data.taskId === null || data.taskId === undefined
      ? undefined
      : requireString(data.taskId, 'taskId', method, 256)
    if (data.task !== undefined && !isObject(data.task)) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        `Python Agent ${method} response task must be an object`,
      )
    }
    return {
      mode: 'approval-gated',
      provider: PROVIDER_ID,
      requestId,
      planId,
      status,
      instruction,
      createdAt,
      expiresInSeconds: data.expiresInSeconds,
      plan: data.plan,
      ...taskId === undefined ? {} : { taskId },
      ...data.task === undefined ? {} : { task: data.task },
    }
  }

  async #requireConfiguredMode(signal?: AbortSignal): Promise<void> {
    const status = await this.status(signal)
    const ready = this.#config.approvalGatedExecution
      ? status.executionReady
      : status.planOnlyReady
    if (!ready) {
      throw new RobotError(
        this.#config.approvalGatedExecution
          ? 'ROBOT_EXECUTION_NOT_ENABLED'
          : 'ROBOT_PLAN_ONLY_REQUIRED',
        status.reason ?? 'Python Agent readiness could not be verified',
      )
    }
  }

  async #request(
    method:
      | 'health'
      | 'manifest'
      | 'plan'
      | 'request_execution'
      | 'execution_request_status',
    args: RobotJsonObject,
    signal?: AbortSignal,
  ): Promise<RobotJsonValue> {
    if (signal?.aborted === true) {
      throw new RobotError(
        'ROBOT_ABORTED',
        `Python Agent ${method} request was aborted before connection`,
      )
    }
    const request: RobotJsonObject = {
      method,
      args,
      ...this.#config.token === undefined ? {} : { token: this.#config.token },
    }
    const payload = `${JSON.stringify(request)}\n`
    const response = await this.#exchange(method, payload, signal)
    let parsed: RobotJsonValue
    try {
      parsed = JSON.parse(response) as RobotJsonValue
    } catch (error: unknown) {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        `Python Agent ${method} returned invalid JSON`,
        { cause: error },
      )
    }
    const envelope = requireObject(parsed, method)
    if (typeof envelope.ok !== 'boolean') {
      throw new RobotError(
        'ROBOT_PROTOCOL',
        `Python Agent ${method} response must contain a boolean ok field`,
      )
    }
    if (!envelope.ok) {
      const detail = typeof envelope.error === 'string'
        ? envelope.error
        : typeof envelope.message === 'string'
          ? envelope.message
          : 'request was rejected'
      throw new RobotError(
        'ROBOT_REMOTE',
        `Python Agent ${method} failed: ${detail}`,
      )
    }
    return envelope.data ?? null
  }

  #exchange(
    method: string,
    payload: string,
    signal?: AbortSignal,
  ): Promise<string> {
    return new Promise<string>((resolve, reject) => {
      let settled = false
      let received = Buffer.alloc(0)
      const socket = createConnection({
        host: this.#config.host,
        port: this.#config.port,
      })

      const cleanup = (): void => {
        clearTimeout(timeout)
        signal?.removeEventListener('abort', abort)
        socket.removeAllListeners()
        socket.destroy()
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
      const abort = (): void => {
        fail(new RobotError(
          'ROBOT_ABORTED',
          `Python Agent ${method} request was aborted`,
        ))
      }
      const timeout = setTimeout(() => {
        fail(new RobotError(
          'ROBOT_TIMEOUT',
          `Python Agent ${method} timed out after ${this.#config.timeoutMs} ms`,
        ))
      }, this.#config.timeoutMs)

      signal?.addEventListener('abort', abort, { once: true })
      socket.once('connect', () => socket.end(payload))
      socket.on('data', (chunk: Buffer) => {
        received = Buffer.concat([received, chunk])
        if (received.length > this.#config.maxResponseBytes + 1) {
          fail(new RobotError(
            'ROBOT_RESPONSE_TOO_LARGE',
            `Python Agent ${method} response exceeds ${this.#config.maxResponseBytes} bytes`,
          ))
          return
        }
        const newline = received.indexOf(0x0A)
        if (newline < 0) return
        if (received.subarray(newline + 1).toString('utf8').trim().length > 0) {
          fail(new RobotError(
            'ROBOT_PROTOCOL',
            `Python Agent ${method} returned more than one NDJSON response`,
          ))
          return
        }
        try {
          const decoder = new TextDecoder('utf-8', { fatal: true })
          succeed(decoder.decode(received.subarray(0, newline)))
        } catch (error: unknown) {
          fail(new RobotError(
            'ROBOT_PROTOCOL',
            `Python Agent ${method} returned invalid UTF-8`,
            { cause: error },
          ))
        }
      })
      socket.once('end', () => {
        fail(new RobotError(
          'ROBOT_PROTOCOL',
          `Python Agent ${method} closed before a complete NDJSON response`,
        ))
      })
      socket.once('error', (error: NodeJS.ErrnoException) => {
        fail(new RobotError(
          'ROBOT_CONNECTION',
          `Could not reach Python Agent at ${this.#config.host}:${this.#config.port}: ${error.code ?? error.message}`,
          { cause: error },
        ))
      })
    })
  }
}

export default PythonAgentRobotRuntime
