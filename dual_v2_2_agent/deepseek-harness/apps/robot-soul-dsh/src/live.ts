import { resolve } from 'node:path'

import { PRODUCT_CATALOG, STATIC_SAFETY } from './catalog.ts'
import type { LiveReader, LiveSnapshot } from './contracts.ts'

const DEFAULT_HOST = '127.0.0.1'
const DEFAULT_PORT = 15_555
export const HARDWARE_CONFIRMATION = 'ENABLE_DUAL_V2_2_HARDWARE'
const cordisSource = new URL('../../../vendor/cordis/src/index.ts', import.meta.url).href
const providerSource = new URL(
  '../../../packages/robot/robot-python-agent/src/index.ts',
  import.meta.url,
).href
const productionClientSource = new URL(
  '../../robot-assistant/src/python-agent-client.ts',
  import.meta.url,
).href

interface ContextModule {
  readonly Context: new () => unknown
}

interface ProviderConfig {
  readonly host: string
  readonly port: number
  readonly token?: string
  readonly timeoutMs: number
  readonly maxResponseBytes: number
  readonly outputDir: string
}

interface ProviderModule {
  readonly PythonAgentRobotRuntime: new (
    context: unknown,
    config: ProviderConfig,
  ) => LiveReader
}

interface ProductionReadOnlyClient {
  health(): Promise<Readonly<Record<string, unknown>>>
  manifest(): Promise<Readonly<Record<string, unknown>>>
}

interface ProductionClientModule {
  readonly PythonAgentClient: new (config: {
    readonly host: string
    readonly port: number
    readonly token?: string
    readonly timeoutMs: number
    readonly maxResponseBytes: number
  }) => ProductionReadOnlyClient
}

export interface HardwareAccessPolicy {
  readonly stage: 'simulation' | 'hardware-readonly' | 'approval-gated-execute'
  readonly readonlyEnabled: boolean
  readonly executeEnabled: boolean
  readonly hardwareOwner: 'python-agent'
  readonly allowedMethods: readonly string[]
  readonly blockedMethods: readonly string[]
  readonly nextStage: 'approval-gated-execute-after-validation' | 'operator-approved-high-level-skills'
}

function integerFromEnv(name: string, fallback: number): number {
  const raw = process.env[name]
  if (raw === undefined) return fallback
  const value = Number(raw)
  if (!Number.isSafeInteger(value) || value < 1) {
    throw new Error(`${name} must be a positive integer.`)
  }
  return value
}

function booleanFromEnv(name: string, fallback = false): boolean {
  const raw = process.env[name]
  if (raw === undefined) return fallback
  if (raw === '1') return true
  if (raw === '0') return false
  throw new Error(`${name} must be either 0 or 1.`)
}

/** Public policy shown to the browser; no credential or filesystem value is included. */
export function hardwareAccessPolicy(): HardwareAccessPolicy {
  const readonlyEnabled = booleanFromEnv('ROBOT_SOUL_HARDWARE_READONLY')
  const executionGate = process.env.ROBOT_SOUL_APPROVED_EXECUTION
  if (
    executionGate !== undefined
    && executionGate !== HARDWARE_CONFIRMATION
  ) {
    throw new Error(
      `ROBOT_SOUL_APPROVED_EXECUTION must exactly equal ${HARDWARE_CONFIRMATION}.`,
    )
  }
  const executeEnabled = readonlyEnabled
    && executionGate === HARDWARE_CONFIRMATION
  return {
    stage: executeEnabled
      ? 'approval-gated-execute'
      : readonlyEnabled
        ? 'hardware-readonly'
        : 'simulation',
    readonlyEnabled,
    executeEnabled,
    hardwareOwner: 'python-agent',
    allowedMethods: executeEnabled
      ? [
        'health',
        'manifest',
        'plan',
        'execute',
        'pending_execution_requests',
        'approve_execution',
        'deny_execution',
        'task_status',
        'task_events',
        'cancel',
      ]
      : ['health', 'manifest'],
    blockedMethods: executeEnabled
      ? ['call']
      : ['plan', 'execute', 'task_status', 'task_events', 'cancel', 'call'],
    nextStage: executeEnabled
      ? 'operator-approved-high-level-skills'
      : 'approval-gated-execute-after-validation',
  }
}

/**
 * Lazily load the existing Cordis Provider. The website remains fully usable
 * when workspace packages have not been built or no Python Agent is running.
 */
export function createProviderLiveReader(): LiveReader {
  let readerPromise: Promise<LiveReader> | undefined

  const load = async (): Promise<LiveReader> => {
    const [cordis, provider] = await Promise.all([
      import(cordisSource) as Promise<ContextModule>,
      import(providerSource) as Promise<ProviderModule>,
    ])
    const { Context } = cordis
    const { PythonAgentRobotRuntime } = provider
    const runtime = new PythonAgentRobotRuntime(new Context(), {
      host: process.env.ROBOT_SOUL_AGENT_HOST ?? DEFAULT_HOST,
      port: integerFromEnv('ROBOT_SOUL_AGENT_PORT', DEFAULT_PORT),
      timeoutMs: integerFromEnv('ROBOT_SOUL_AGENT_TIMEOUT_MS', 2_500),
      maxResponseBytes: integerFromEnv(
        'ROBOT_SOUL_AGENT_MAX_RESPONSE_BYTES',
        1024 * 1024,
      ),
      outputDir: resolve(process.env.ROBOT_SOUL_OUTPUT_DIR ?? process.cwd()),
      ...process.env.ROBOT_SOUL_AGENT_TOKEN
        ? { token: process.env.ROBOT_SOUL_AGENT_TOKEN }
        : {},
    })
    return {
      status: signal => runtime.status(signal),
      manifest: signal => runtime.manifest(signal),
    }
  }

  const reader = async (): Promise<LiveReader> => {
    readerPromise ??= load()
    try {
      return await readerPromise
    } catch (error) {
      readerPromise = undefined
      throw error
    }
  }

  return {
    status: async signal => (await reader()).status(signal),
    manifest: async signal => (await reader()).manifest(signal),
  }
}

/** Match the production readiness rule enforced by launch_robot_assistant.sh. */
export function isProductionAgentReady(
  health: Readonly<Record<string, unknown>>,
): boolean {
  return health.bootstrapState === 'ready' || health.ready === true
}

/** Normalize production Agent health without surfacing an execution method. */
export function normalizeHardwareStatus(
  health: Readonly<Record<string, unknown>>,
): Readonly<Record<string, unknown>> {
  return {
    mode: 'hardware-readonly',
    provider: 'python-agent-readonly',
    hardwareOwner: 'python-agent',
    readonly: true,
    planOnly: health.planOnly === true,
    bootstrapState: typeof health.bootstrapState === 'string'
      ? health.bootstrapState
      : 'unknown',
    ready: isProductionAgentReady(health),
    activeTask: health.activeTaskId !== null && health.activeTaskId !== undefined,
    health,
  }
}

/** Normalize the production manifest while keeping website permissions read-only. */
export function normalizeHardwareManifest(
  manifest: Readonly<Record<string, unknown>>,
): Readonly<Record<string, unknown>> {
  return {
    mode: 'hardware-readonly',
    provider: 'python-agent-readonly',
    readonly: true,
    tools: Array.isArray(manifest.tools) ? manifest.tools : [],
    skills: Array.isArray(manifest.skills) ? manifest.skills : [],
  }
}

/**
 * Read health and manifest from a production Agent through the already-reviewed
 * loopback client. The returned facade has no plan, execute, task, cancel, or
 * arbitrary-call method.
 */
export function createHardwareReadOnlyReader(): LiveReader {
  let clientPromise: Promise<ProductionReadOnlyClient> | undefined
  const load = async (): Promise<ProductionReadOnlyClient> => {
    const module = await import(productionClientSource) as ProductionClientModule
    return new module.PythonAgentClient({
      host: process.env.ROBOT_SOUL_AGENT_HOST ?? DEFAULT_HOST,
      port: integerFromEnv('ROBOT_SOUL_AGENT_PORT', DEFAULT_PORT),
      timeoutMs: integerFromEnv('ROBOT_SOUL_AGENT_TIMEOUT_MS', 2_500),
      maxResponseBytes: integerFromEnv(
        'ROBOT_SOUL_AGENT_MAX_RESPONSE_BYTES',
        1024 * 1024,
      ),
      ...process.env.ROBOT_SOUL_AGENT_TOKEN
        ? { token: process.env.ROBOT_SOUL_AGENT_TOKEN }
        : {},
    })
  }
  const client = async (): Promise<ProductionReadOnlyClient> => {
    clientPromise ??= load()
    try {
      return await clientPromise
    } catch (error) {
      clientPromise = undefined
      throw error
    }
  }
  return {
    status: async () => normalizeHardwareStatus(await (await client()).health()),
    manifest: async () =>
      normalizeHardwareManifest(await (await client()).manifest()),
  }
}

/** Select the narrow production reader only after the explicit stage-one gate. */
export function createConfiguredLiveReader(): LiveReader {
  return hardwareAccessPolicy().readonlyEnabled
    ? createHardwareReadOnlyReader()
    : createProviderLiveReader()
}

function safeReason(error: unknown): string {
  const raw = error instanceof Error ? error.message : String(error)
  return raw
    .replaceAll(process.env.ROBOT_SOUL_AGENT_TOKEN ?? '\0', '[redacted]')
    .slice(0, 500)
}

/** Read one live value and turn connection failures into an offline snapshot. */
export async function readLive(
  operation: 'status' | 'manifest',
  reader: LiveReader,
): Promise<LiveSnapshot> {
  const controller = new AbortController()
  const timeout = setTimeout(() => {
    controller.abort()
  }, 3_000)
  timeout.unref()
  try {
    const data = await reader[operation](controller.signal)
    return {
      connected: true,
      readonly: true,
      data: operation === 'manifest' ? mergeLiveManifest(data) : data,
      checkedAt: new Date().toISOString(),
    }
  } catch (error) {
    return {
      connected: false,
      readonly: true,
      reason: safeReason(error),
      checkedAt: new Date().toISOString(),
    }
  } finally {
    clearTimeout(timeout)
  }
}

function manifestNames(value: unknown): ReadonlySet<string> {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    return new Set()
  }
  const record = value as Record<string, unknown>
  const arrayOrEmpty = (candidate: unknown): readonly unknown[] =>
    Array.isArray(candidate) ? candidate as readonly unknown[] : []
  const items = [
    ...arrayOrEmpty(record.tools),
    ...arrayOrEmpty(record.skills),
  ]
  const names = items.flatMap((item): string[] => {
    if (typeof item === 'string') return [item]
    if (typeof item !== 'object' || item === null || Array.isArray(item)) return []
    const candidate = item as Record<string, unknown>
    if (typeof candidate.name === 'string') return [candidate.name]
    const fn = candidate.function
    if (typeof fn !== 'object' || fn === null || Array.isArray(fn)) return []
    const name = (fn as Record<string, unknown>).name
    return typeof name === 'string' ? [name] : []
  })
  return new Set(names)
}

/**
 * Runtime data only contributes availability. Reviewed source-controlled
 * safety levels remain authoritative and cannot be lowered by Agent payloads.
 */
export function mergeLiveManifest(value: unknown): unknown {
  const available = manifestNames(value)
  return {
    raw: value,
    products: PRODUCT_CATALOG.map(product => ({
      name: product.name,
      available: available.has(product.name),
      safety: STATIC_SAFETY.get(product.name),
      safetySource: 'static-reviewed-catalog',
    })),
  }
}
