import { resolve } from 'node:path'

import {
  HARDWARE_CONFIRMATION,
  hardwareAccessPolicy,
  isProductionAgentReady,
} from './live.ts'
import { validateSemanticInstruction } from './simulator.ts'

type JsonRecord = Readonly<Record<string, unknown>>
type AgentCursor = number

const DEFAULT_HOST = '127.0.0.1'
const DEFAULT_PORT = 15_555
const DEFAULT_PLAN_TTL_MS = 10 * 60 * 1000
const ALLOWED_EXECUTION_SKILLS = new Set([
  'perceive_scene',
  'pick_and_place',
  'tidy_table',
  'safety_reset',
])
const productionClientSource = new URL(
  '../../robot-assistant/src/python-agent-client.ts',
  import.meta.url,
).href

export interface HardwareAgentApi {
  readonly health: () => Promise<JsonRecord>
  readonly plan: (args: {
    readonly instruction: string
    readonly output_dir: string
  }) => Promise<JsonRecord & { readonly planId: string; readonly plan: unknown }>
  readonly execute: (args: {
    readonly planId: string
    readonly confirmation: string
  }) => Promise<JsonRecord & { readonly taskId: string }>
  readonly pendingExecutionRequests: () => Promise<JsonRecord>
  readonly approveExecution: (args: {
    readonly requestId: string
    readonly confirmation: string
    readonly checklist: JsonRecord
  }) => Promise<JsonRecord>
  readonly denyExecution: (args: {
    readonly requestId: string
  }) => Promise<JsonRecord>
  readonly taskStatus: (args: { readonly taskId: string }) => Promise<JsonRecord>
  readonly taskEvents: (args: {
    readonly taskId: string
    readonly cursor: AgentCursor
  }) => Promise<JsonRecord & {
    readonly events: readonly JsonRecord[]
    readonly nextCursor: AgentCursor
  }>
  readonly cancel: (args: { readonly taskId: string }) => Promise<unknown>
}

interface ProductionClientModule {
  readonly PythonAgentClient: new (config: {
    readonly host: string
    readonly port: number
    readonly token?: string
    readonly timeoutMs: number
    readonly maxResponseBytes: number
  }) => HardwareAgentApi
}

interface PlanRecord {
  readonly planId: string
  readonly instruction: string
  readonly plan: JsonRecord
  readonly expiresAt: number
}

export interface HardwareChecklist {
  readonly operatorPresent: boolean
  readonly workspaceClear: boolean
  readonly emergencyStopReady: boolean
  readonly robotInspected: boolean
}

export type HardwareErrorCode =
  | 'HARDWARE_DISABLED'
  | 'ORIGIN_REJECTED'
  | 'INVALID_REQUEST'
  | 'AGENT_NOT_READY'
  | 'PLAN_REJECTED'
  | 'APPROVAL_REQUIRED'
  | 'TASK_CONFLICT'
  | 'UNKNOWN_PLAN'
  | 'UNKNOWN_TASK'

export class HardwareAccessError extends Error {
  readonly code: HardwareErrorCode
  readonly status: number

  constructor(code: HardwareErrorCode, status: number, message: string) {
    super(message)
    this.name = 'HardwareAccessError'
    this.code = code
    this.status = status
  }
}

function integerFromEnv(name: string, fallback: number): number {
  const raw = process.env[name]
  if (raw === undefined) return fallback
  const value = Number(raw)
  if (!Number.isSafeInteger(value) || value < 1) {
    throw new HardwareAccessError(
      'INVALID_REQUEST',
      500,
      `${name} must be a positive integer.`,
    )
  }
  return value
}

function isRecord(value: unknown): value is JsonRecord {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
}

function requireIdentifier(value: unknown, field: string): string {
  if (typeof value !== 'string' || value.length === 0 || value.length > 256) {
    throw new HardwareAccessError(
      'INVALID_REQUEST',
      400,
      `${field} must be a non-empty string no longer than 256 characters.`,
    )
  }
  return value
}

function statusFromHealth(health: JsonRecord): string {
  return typeof health.bootstrapState === 'string'
    ? health.bootstrapState
    : 'unknown'
}

function taskStatus(data: JsonRecord): string {
  const value = data.status ?? data.state
  return typeof value === 'string' ? value.toLowerCase() : 'unknown'
}

function isTerminalTaskStatus(status: string): boolean {
  return new Set([
    'succeeded',
    'success',
    'completed',
    'failed',
    'error',
    'cancelled',
    'canceled',
  ]).has(status)
}

function rejectPlan(message: string, status = 400): never {
  throw new HardwareAccessError('PLAN_REJECTED', status, message)
}

function requireAllowedKeys(
  value: JsonRecord,
  keys: readonly string[],
  label: string,
): void {
  const allowed = new Set(keys)
  const unknown = Object.keys(value).find(key => !allowed.has(key))
  if (unknown !== undefined) {
    rejectPlan(`${label} does not allow parameter ${unknown}.`)
  }
}

function requireShortString(value: unknown, label: string): void {
  if (typeof value !== 'string' || value.length < 1 || value.length > 2000) {
    rejectPlan(`${label} must be a string from 1 to 2000 characters.`)
  }
}

function validateSelector(value: unknown, label: string): void {
  if (typeof value === 'string') {
    if (!/^object_[0-9]+$/.test(value)) {
      rejectPlan(`${label} has an invalid object id.`)
    }
    return
  }
  if (!isRecord(value)) rejectPlan(`${label} must be a semantic object selector.`)
  requireAllowedKeys(value, ['category', 'color'], label)
  requireShortString(value.category, `${label}.category`)
  if (value.color !== undefined) requireShortString(value.color, `${label}.color`)
}

function validatePlacement(value: unknown, label: string): void {
  if (!isRecord(value)) rejectPlan(`${label} must be an object.`)
  requireAllowedKeys(
    value,
    ['object', 'target', 'placement_type', 'offset', 'plan_only'],
    label,
  )
  validateSelector(value.object, `${label}.object`)
  validateSelector(value.target, `${label}.target`)
  if (
    value.placement_type !== undefined
    && value.placement_type !== 'on_center'
    && value.placement_type !== 'beside'
  ) {
    rejectPlan(`${label}.placement_type must be on_center or beside.`)
  }
  if (value.offset !== undefined) {
    if (
      !Array.isArray(value.offset)
      || value.offset.length !== 3
      || !value.offset.every(item => typeof item === 'number' && Number.isFinite(item))
      || Math.abs(value.offset[0] as number) > 200
      || Math.abs(value.offset[1] as number) > 200
      || Math.abs(value.offset[2] as number) > 100
    ) {
      rejectPlan(`${label}.offset must be a bounded [x,y,z] millimeter offset.`)
    }
  }
  if (value.plan_only !== undefined && typeof value.plan_only !== 'boolean') {
    rejectPlan(`${label}.plan_only must be boolean.`)
  }
}

function validateSkillArguments(skill: string, value: unknown): number {
  if (!isRecord(value)) rejectPlan(`${skill}.args must be an object.`, 502)
  if (JSON.stringify(value).length > 8192) {
    rejectPlan(`${skill}.args exceeds the 8 KiB limit.`)
  }
  if (skill === 'perceive_scene') {
    requireAllowedKeys(
      value,
      ['instruction', 'targets', 'categories', 'selectors'],
      skill,
    )
    if (value.instruction !== undefined) {
      requireShortString(value.instruction, `${skill}.instruction`)
    }
    for (const key of ['targets', 'categories'] as const) {
      const items = value[key]
      if (
        items !== undefined
        && (
          !Array.isArray(items)
          || items.length > 32
          || !items.every(isRecord)
        )
      ) {
        rejectPlan(`${skill}.${key} must be an array containing at most 32 objects.`)
      }
    }
    if (value.selectors !== undefined) {
      if (!Array.isArray(value.selectors) || value.selectors.length > 32) {
        rejectPlan(`${skill}.selectors must be an array containing at most 32 selectors.`)
      }
      value.selectors.forEach((selector, index) => {
        validateSelector(selector, `${skill}.selectors[${index}]`)
      })
    }
    return 0
  }
  if (skill === 'pick_and_place') {
    validatePlacement(value, skill)
    return 1
  }
  if (skill === 'tidy_table') {
    requireAllowedKeys(value, ['instruction', 'placements'], skill)
    if (value.instruction !== undefined) {
      requireShortString(value.instruction, `${skill}.instruction`)
    }
    if (value.placements === undefined) return 4
    if (!Array.isArray(value.placements) || value.placements.length > 8) {
      rejectPlan(`${skill}.placements may contain at most 8 placement actions.`)
    }
    for (const [index, placement] of value.placements.entries()) {
      validatePlacement(placement, `${skill}.placements[${index}]`)
    }
    return value.placements.length
  }
  if (skill === 'safety_reset') {
    requireAllowedKeys(value, ['arms', 'kind'], skill)
    if (
      value.arms !== undefined
      && (
        typeof value.arms !== 'string'
        || !['left', 'right', 'both'].includes(value.arms)
      )
    ) {
      rejectPlan(`${skill}.arms must be left, right, or both.`)
    }
    if (value.kind !== undefined && value.kind !== 'safe') {
      rejectPlan(`${skill}.kind must be safe on the website approval surface.`)
    }
    return 1
  }
  rejectPlan(`The hardware approval surface does not allow Skill ${skill}.`)
}

/** Enforce a high-level Skill-only execution plan. */
export function validateExecutionPlan(value: unknown): JsonRecord {
  if (!isRecord(value) || !Array.isArray(value.steps)) {
    throw new HardwareAccessError(
      'PLAN_REJECTED',
      502,
      'A Python Agent Plan must contain a steps array.',
    )
  }
  if (value.steps.length < 1 || value.steps.length > 12) {
    throw new HardwareAccessError(
      'PLAN_REJECTED',
      400,
      'A hardware Plan must contain 1 to 12 high-level Skills.',
    )
  }
  let motionBudget = 0
  for (const [index, step] of value.steps.entries()) {
    if (!isRecord(step) || typeof step.skill !== 'string') {
      throw new HardwareAccessError(
        'PLAN_REJECTED',
        502,
        `Plan step ${index + 1} is missing skill.`,
      )
    }
    if (!ALLOWED_EXECUTION_SKILLS.has(step.skill)) {
      throw new HardwareAccessError(
        'PLAN_REJECTED',
        400,
        `The hardware approval surface does not allow Skill ${step.skill}.`,
      )
    }
    motionBudget += validateSkillArguments(step.skill, step.args)
    if (motionBudget > 8) {
      rejectPlan('The hardware Plan exceeds the 8-motion-unit limit.')
    }
  }
  return value
}

/** Approval-gated controller; it never exposes arbitrary Python Agent calls. */
export class HardwareExecutionController {
  readonly #api: HardwareAgentApi
  readonly #outputDir: string
  readonly #planTtlMs: number
  readonly #now: () => number
  readonly #plans = new Map<string, PlanRecord>()
  readonly #issuedTaskIds = new Set<string>()
  #activeTaskId: string | undefined

  constructor(
    api: HardwareAgentApi,
    options: {
      readonly outputDir: string
      readonly planTtlMs?: number
      readonly now?: () => number
    },
  ) {
    this.#api = api
    this.#outputDir = options.outputDir
    this.#planTtlMs = options.planTtlMs ?? DEFAULT_PLAN_TTL_MS
    this.#now = options.now ?? Date.now
  }

  async preview(rawInstruction: unknown): Promise<JsonRecord> {
    this.#requireEnabled()
    if (this.#activeTaskId !== undefined) {
      throw new HardwareAccessError(
        'TASK_CONFLICT',
        409,
        'Another hardware Task is already active.',
      )
    }
    const instruction = validateSemanticInstruction(rawInstruction)
    await this.#requireReadyAgent()
    const result = await this.#api.plan({
      instruction,
      output_dir: this.#outputDir,
    })
    const planId = requireIdentifier(result.planId, 'planId')
    const plan = validateExecutionPlan(result.plan)
    const expiresAt = this.#now() + this.#planTtlMs
    this.#plans.clear()
    this.#plans.set(planId, {
      planId,
      instruction,
      plan,
      expiresAt,
    })
    return {
      mode: 'hardware-approval-required',
      planId,
      instruction,
      plan,
      expiresAt: new Date(expiresAt).toISOString(),
      allowedSkills: [...ALLOWED_EXECUTION_SKILLS],
      hardwareOwner: 'python-agent',
    }
  }

  async execute(input: {
    readonly planId: unknown
    readonly confirmation: unknown
    readonly checklist: unknown
  }): Promise<JsonRecord> {
    this.#requireEnabled()
    if (this.#activeTaskId !== undefined) {
      throw new HardwareAccessError(
        'TASK_CONFLICT',
        409,
        'Another hardware Task is already active.',
      )
    }
    const planId = requireIdentifier(input.planId, 'planId')
    if (input.confirmation !== HARDWARE_CONFIRMATION) {
      throw new HardwareAccessError(
        'APPROVAL_REQUIRED',
        403,
        `The confirmation phrase must exactly equal ${HARDWARE_CONFIRMATION}.`,
      )
    }
    this.#requireChecklist(input.checklist)
    this.#removeExpiredPlans()
    const record = this.#plans.get(planId)
    if (record === undefined) {
      throw new HardwareAccessError(
        'UNKNOWN_PLAN',
        404,
        'The Plan does not exist, has expired, or was not created by this website process.',
      )
    }
    await this.#requireReadyAgent()
    const result = await this.#api.execute({
      planId,
      confirmation: HARDWARE_CONFIRMATION,
    })
    const taskId = requireIdentifier(result.taskId, 'taskId')
    this.#plans.delete(planId)
    this.#issuedTaskIds.add(taskId)
    this.#activeTaskId = taskId
    return {
      mode: 'hardware-execution',
      taskId,
      planId,
      status: typeof result.status === 'string' ? result.status : 'queued',
      warning: 'Cooperative cancellation is not an emergency stop. Use the independent physical emergency stop for unexpected motion.',
    }
  }

  async pendingApprovals(): Promise<JsonRecord> {
    this.#requireEnabled()
    const result = await this.#api.pendingExecutionRequests()
    if (!Array.isArray(result.requests)) {
      throw new HardwareAccessError(
        'PLAN_REJECTED',
        502,
        'The Python Agent pending-approval response must contain a requests array.',
      )
    }
    const requests = result.requests.map((value) => {
      if (
        !isRecord(value)
        || value.source !== 'deepseek-harness'
        || typeof value.requestId !== 'string'
        || typeof value.planId !== 'string'
      ) {
        throw new HardwareAccessError(
          'PLAN_REJECTED',
          502,
          'The Python Agent returned an invalid Harness approval request.',
        )
      }
      validateExecutionPlan(value.plan)
      return value
    })
    return { requests }
  }

  async approveRequest(input: {
    readonly requestId: unknown
    readonly confirmation: unknown
    readonly checklist: unknown
  }): Promise<JsonRecord> {
    this.#requireEnabled()
    const requestId = requireIdentifier(input.requestId, 'requestId')
    if (input.confirmation !== HARDWARE_CONFIRMATION) {
      throw new HardwareAccessError(
        'APPROVAL_REQUIRED',
        403,
        `The confirmation phrase must exactly equal ${HARDWARE_CONFIRMATION}.`,
      )
    }
    this.#requireChecklist(input.checklist)
    await this.#requireReadyAgent()
    const pending = await this.pendingApprovals()
    const request = (pending.requests as readonly unknown[]).find(value =>
      isRecord(value) && value.requestId === requestId)
    if (!isRecord(request)) {
      throw new HardwareAccessError(
        'UNKNOWN_PLAN',
        404,
        'No matching Harness Agent approval request was found.',
      )
    }
    validateExecutionPlan(request.plan)
    const result = await this.#api.approveExecution({
      requestId,
      confirmation: HARDWARE_CONFIRMATION,
      checklist: { ...input.checklist },
    })
    const taskId = requireIdentifier(result.taskId, 'taskId')
    this.#issuedTaskIds.add(taskId)
    this.#activeTaskId = taskId
    return {
      ...result,
      requestId,
      taskId,
      warning: 'An operator approved the AI request. Cooperative cancellation is not an emergency stop.',
    }
  }

  async denyRequest(rawRequestId: unknown): Promise<JsonRecord> {
    this.#requireEnabled()
    const requestId = requireIdentifier(rawRequestId, 'requestId')
    const result = await this.#api.denyExecution({ requestId })
    return { ...result, requestId }
  }

  async status(rawTaskId: unknown): Promise<JsonRecord> {
    this.#requireEnabled()
    const taskId = this.#requireIssuedTask(rawTaskId)
    const result = await this.#api.taskStatus({ taskId })
    if (isTerminalTaskStatus(taskStatus(result)) && this.#activeTaskId === taskId) {
      this.#activeTaskId = undefined
    }
    return { ...result, taskId }
  }

  async events(
    rawTaskId: unknown,
    rawCursor: unknown,
  ): Promise<JsonRecord> {
    this.#requireEnabled()
    const taskId = this.#requireIssuedTask(rawTaskId)
    const cursor = this.#requireCursor(rawCursor)
    return this.#api.taskEvents({ taskId, cursor })
  }

  async cancel(rawTaskId: unknown): Promise<JsonRecord> {
    this.#requireEnabled()
    const taskId = this.#requireIssuedTask(rawTaskId)
    const result = await this.#api.cancel({ taskId })
    return {
      taskId,
      cancelRequested: true,
      result,
      warning: 'Cooperative cancellation is not an emergency stop.',
    }
  }

  #requireEnabled(): void {
    if (!hardwareAccessPolicy().executeEnabled) {
      throw new HardwareAccessError(
        'HARDWARE_DISABLED',
        403,
        'Approval-gated execution is disabled; both the read-only and server execution gates must be open.',
      )
    }
  }

  async #requireReadyAgent(): Promise<void> {
    const health = await this.#api.health()
    if (health.planOnly !== false) {
      throw new HardwareAccessError(
        'AGENT_NOT_READY',
        409,
        'Hardware execution requires a production Python Agent, not a plan-only Agent.',
      )
    }
    const bootstrapState = statusFromHealth(health)
    if (!isProductionAgentReady(health)) {
      throw new HardwareAccessError(
        'AGENT_NOT_READY',
        409,
        `Python Agent bootstrap state must be ready; current state is ${bootstrapState}.`,
      )
    }
    if (health.activeTaskId !== null) {
      throw new HardwareAccessError(
        'TASK_CONFLICT',
        409,
        'The Python Agent reports an active Task.',
      )
    }
  }

  #requireChecklist(value: unknown): asserts value is HardwareChecklist {
    if (
      !isRecord(value)
      || value.operatorPresent !== true
      || value.workspaceClear !== true
      || value.emergencyStopReady !== true
      || value.robotInspected !== true
    ) {
      throw new HardwareAccessError(
        'APPROVAL_REQUIRED',
        403,
        'Confirm operator presence, a clear workspace, a ready emergency stop, and a completed robot inspection.',
      )
    }
  }

  #requireIssuedTask(value: unknown): string {
    const taskId = requireIdentifier(value, 'taskId')
    if (!this.#issuedTaskIds.has(taskId)) {
      throw new HardwareAccessError(
        'UNKNOWN_TASK',
        404,
        'This Task was not submitted by the current website process.',
      )
    }
    return taskId
  }

  #requireCursor(value: unknown): AgentCursor {
    if (value === undefined) return 0
    if (typeof value === 'number' && Number.isSafeInteger(value) && value >= 0) {
      return value
    }
    if (typeof value === 'string' && /^(?:0|[1-9]\d*)$/.test(value)) {
      const parsed = Number(value)
      if (Number.isSafeInteger(parsed)) return parsed
    }
    throw new HardwareAccessError(
      'INVALID_REQUEST',
      400,
      'cursor must be a non-negative safe integer.',
    )
  }

  #removeExpiredPlans(): void {
    const now = this.#now()
    for (const [planId, record] of this.#plans) {
      if (record.expiresAt <= now) this.#plans.delete(planId)
    }
  }
}

/** Build the production controller lazily used by namespaced HTTP routes. */
export async function createHardwareExecutionController(): Promise<HardwareExecutionController> {
  const module = await import(productionClientSource) as ProductionClientModule
  const api = new module.PythonAgentClient({
    host: process.env.ROBOT_SOUL_AGENT_HOST ?? DEFAULT_HOST,
    port: integerFromEnv('ROBOT_SOUL_AGENT_PORT', DEFAULT_PORT),
    timeoutMs: integerFromEnv('ROBOT_SOUL_AGENT_TIMEOUT_MS', 30_000),
    maxResponseBytes: integerFromEnv(
      'ROBOT_SOUL_AGENT_MAX_RESPONSE_BYTES',
      1024 * 1024,
    ),
    ...process.env.ROBOT_SOUL_AGENT_TOKEN
      ? { token: process.env.ROBOT_SOUL_AGENT_TOKEN }
      : {},
  })
  return new HardwareExecutionController(api, {
    outputDir: resolve(process.env.ROBOT_SOUL_OUTPUT_DIR ?? process.cwd()),
    planTtlMs: integerFromEnv(
      'ROBOT_SOUL_HARDWARE_PLAN_TTL_MS',
      DEFAULT_PLAN_TTL_MS,
    ),
  })
}
