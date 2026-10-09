import { spawn } from 'node:child_process'
import { readFile } from 'node:fs/promises'
import { isIP } from 'node:net'
import { isAbsolute } from 'node:path'

import type {
  AgentBootstrapStatusReader,
  HardwareRecoveryLauncher,
  HardwareStopLauncher,
  HardwareStartupLauncher,
  HarnessStatusReader,
  HarnessStatusSnapshot,
  StackLifecycleState,
  StackStatusReader,
  StackStatusSnapshot,
} from './contracts.ts'
import { HARDWARE_CONFIRMATION } from './live.ts'

const DEFAULT_HARNESS_PORT = 3_080

function loopbackHost(value: string | undefined): string {
  const host = value ?? '127.0.0.1'
  const version = isIP(host)
  const allowed = version === 4
    ? host.startsWith('127.')
    : version === 6 && host === '::1'
  if (!allowed) {
    throw new Error('ROBOT_SOUL_HARNESS_HOST must be a numeric loopback address.')
  }
  return host
}

function portNumber(value: string | undefined): number {
  if (value === undefined) return DEFAULT_HARNESS_PORT
  const port = Number(value)
  if (!Number.isSafeInteger(port) || port < 1 || port > 65_535) {
    throw new Error('ROBOT_SOUL_HARNESS_PORT must be an integer from 1 to 65535.')
  }
  return port
}

/** Return the validated loopback origin of the embedded Harness Web UI. */
export function harnessWebOrigin(): string {
  const host = loopbackHost(process.env.ROBOT_SOUL_HARNESS_HOST)
  const port = portNumber(process.env.ROBOT_SOUL_HARNESS_PORT)
  return `http://${host.includes(':') ? `[${host}]` : host}:${port}`
}

/** Create a narrow health reader for the independently isolated Harness UI. */
export function createHarnessStatusReader(): HarnessStatusReader {
  const url = harnessWebOrigin()
  return {
    url,
    async read(): Promise<HarnessStatusSnapshot> {
      const checkedAt = new Date().toISOString()
      try {
        const response = await fetch(url, {
          redirect: 'manual',
          signal: AbortSignal.timeout(2_000),
        })
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`)
        }
        return { connected: true, url, checkedAt }
      } catch (error) {
        return {
          connected: false,
          url,
          checkedAt,
          reason: error instanceof Error ? error.message : String(error),
        }
      }
    },
  }
}

function healthyStackStatus(): StackStatusSnapshot {
  return {
    state: 'healthy',
    recoverable: false,
    updatedAt: new Date().toISOString(),
  }
}

function stackState(value: unknown): StackLifecycleState | undefined {
  return value === 'offline'
    || value === 'healthy'
    || value === 'faulted'
    || value === 'recovering'
    || value === 'starting'
    || value === 'stopping'
    ? value
    : undefined
}

/** Read the supervisor-owned hardware lifecycle status without exposing paths. */
export function createStackStatusReader(): StackStatusReader {
  const path = process.env.ROBOT_SOUL_STACK_STATUS_FILE
  return {
    async read(): Promise<StackStatusSnapshot> {
      if (path === undefined) return healthyStackStatus()
      try {
        const parsed: unknown = JSON.parse(await readFile(path, 'utf8'))
        if (typeof parsed !== 'object' || parsed === null || Array.isArray(parsed)) {
          throw new TypeError('status payload must be an object')
        }
        const record = parsed as Readonly<Record<string, unknown>>
        const state = stackState(record.state)
        if (state === undefined) throw new TypeError('invalid stack state')
        const progress = record.progress
        if (
          progress !== undefined
          && (
            typeof progress !== 'number'
            || !Number.isFinite(progress)
            || progress < 0
            || progress > 100
          )
        ) {
          throw new TypeError('invalid stack progress')
        }
        return {
          state,
          ...(typeof record.reason === 'string' ? { reason: record.reason } : {}),
          ...(typeof record.detail === 'string' ? { detail: record.detail } : {}),
          ...(typeof progress === 'number' ? { progress } : {}),
          ...(typeof record.operation === 'string'
            ? { operation: record.operation }
            : {}),
          recoverable: state === 'faulted' && record.recoverable === true,
          updatedAt: typeof record.updatedAt === 'string'
            ? record.updatedAt
            : new Date().toISOString(),
        }
      } catch (error) {
        if (
          typeof error === 'object'
          && error !== null
          && 'code' in error
          && error.code === 'ENOENT'
        ) {
          return healthyStackStatus()
        }
        return {
          state: 'faulted',
          reason: 'invalid_supervisor_status',
          detail: 'The hardware supervisor status is unreadable; recovery is disabled to fail closed.',
          recoverable: false,
          updatedAt: new Date().toISOString(),
        }
      }
    },
  }
}

/** Read model-loading progress written by the supervised Python Agent. */
export function createAgentBootstrapStatusReader(): AgentBootstrapStatusReader {
  const path = process.env.ROBOT_SOUL_AGENT_STATUS_FILE
  return {
    async read() {
      if (path === undefined) {
        return { state: 'unavailable', current: 0, total: 1 }
      }
      try {
        const parsed: unknown = JSON.parse(await readFile(path, 'utf8'))
        if (typeof parsed !== 'object' || parsed === null || Array.isArray(parsed)) {
          throw new TypeError('bootstrap status payload must be an object')
        }
        const record = parsed as Readonly<Record<string, unknown>>
        const state = record.state
        if (state !== 'loading' && state !== 'ready' && state !== 'error') {
          throw new TypeError('invalid bootstrap state')
        }
        const current = record.current
        const total = record.total
        if (
          typeof current !== 'number'
          || !Number.isSafeInteger(current)
          || current < 0
          || typeof total !== 'number'
          || !Number.isSafeInteger(total)
          || total < 1
          || current > total
        ) {
          throw new TypeError('invalid bootstrap progress')
        }
        return {
          state,
          current,
          total,
          ...(typeof record.label === 'string' && record.label !== ''
            ? { label: record.label }
            : {}),
          ...(typeof record.detail === 'string' && record.detail !== ''
            ? { detail: record.detail }
            : {}),
          ...(typeof record.error === 'string' && record.error !== ''
            ? { error: record.error }
            : {}),
        }
      } catch (error) {
        if (
          typeof error === 'object'
          && error !== null
          && 'code' in error
          && error.code === 'ENOENT'
        ) {
          return { state: 'unavailable', current: 0, total: 1 }
        }
        return {
          state: 'error',
          current: 0,
          total: 1,
          error: 'Python Agent startup progress is unreadable.',
        }
      }
    },
  }
}

/** Launch the operator-only recovery subprocess; it is never exposed to Cordis. */
export function createHardwareRecoveryLauncher(): HardwareRecoveryLauncher {
  const launcher = process.env.ROBOT_SOUL_RECOVERY_LAUNCHER
  return {
    async start() {
      if (launcher === undefined || !isAbsolute(launcher)) {
        throw new Error('No absolute hardware recovery launcher path is configured.')
      }
      return await new Promise((resolve, reject) => {
        const child = spawn(launcher, ['recover-assistant'], {
          detached: true,
          env: {
            ...process.env,
            ROBOT_ASSISTANT_HARDWARE_CONFIRM: HARDWARE_CONFIRMATION,
          },
          stdio: 'ignore',
        })
        child.once('error', reject)
        child.once('spawn', () => {
          child.unref()
          resolve({
            accepted: true as const,
            ...(child.pid === undefined ? {} : { pid: child.pid }),
          })
        })
      })
    },
  }
}

/** Launch a local operator-confirmed cold start; it is never exposed to Cordis. */
export function createHardwareStartupLauncher(): HardwareStartupLauncher {
  const launcher = process.env.ROBOT_SOUL_STARTUP_LAUNCHER
  return {
    async start() {
      if (launcher === undefined || !isAbsolute(launcher)) {
        throw new Error('No absolute hardware startup launcher path is configured.')
      }
      return await new Promise((resolve, reject) => {
        const child = spawn(launcher, ['start-assistant'], {
          detached: true,
          env: {
            ...process.env,
            ROBOT_ASSISTANT_HARDWARE_CONFIRM: HARDWARE_CONFIRMATION,
          },
          stdio: 'ignore',
        })
        child.once('error', reject)
        child.once('spawn', () => {
          child.unref()
          resolve({
            accepted: true as const,
            ...(child.pid === undefined ? {} : { pid: child.pid }),
          })
        })
      })
    },
  }
}

/** Stop only the hardware plane; the local operator page and Harness stay online. */
export function createHardwareStopLauncher(): HardwareStopLauncher {
  const launcher = process.env.ROBOT_SOUL_STARTUP_LAUNCHER
  return {
    async stop() {
      if (launcher === undefined || !isAbsolute(launcher)) {
        throw new Error('No absolute hardware lifecycle launcher path is configured.')
      }
      return await new Promise((resolve, reject) => {
        const child = spawn(launcher, ['stop-assistant'], {
          detached: true,
          env: process.env,
          stdio: 'ignore',
        })
        child.once('error', reject)
        child.once('spawn', () => {
          child.unref()
          resolve({
            accepted: true as const,
            ...(child.pid === undefined ? {} : { pid: child.pid }),
          })
        })
      })
    },
  }
}
