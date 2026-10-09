import { mkdtemp, rm, writeFile } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import { join } from 'node:path'

import { afterEach, describe, expect, it, vi } from 'vitest'

import {
  createAgentBootstrapStatusReader,
  createStackStatusReader,
} from '../src/ops.ts'

describe('hardware stack lifecycle status', () => {
  const temporaryDirectories: string[] = []

  afterEach(async () => {
    vi.unstubAllEnvs()
    await Promise.all(temporaryDirectories.splice(0).map(
      directory => rm(directory, { force: true, recursive: true }),
    ))
  })

  it('treats a missing supervisor status as healthy', async () => {
    const directory = await mkdtemp(join(tmpdir(), 'robot-soul-ops-'))
    temporaryDirectories.push(directory)
    vi.stubEnv('ROBOT_SOUL_STACK_STATUS_FILE', join(directory, 'stack.json'))

    await expect(createStackStatusReader().read()).resolves.toMatchObject({
      state: 'healthy',
      recoverable: false,
    })
  })

  it('reports a console-only hardware plane as offline and startable', async () => {
    const directory = await mkdtemp(join(tmpdir(), 'robot-soul-ops-'))
    temporaryDirectories.push(directory)
    const path = join(directory, 'stack.json')
    await writeFile(path, JSON.stringify({
      state: 'offline',
      reason: 'operator_startup_available',
      detail: 'The local operator console is ready.',
      recoverable: false,
      updatedAt: '2026-08-24T08:00:00.000Z',
    }))
    vi.stubEnv('ROBOT_SOUL_STACK_STATUS_FILE', path)

    await expect(createStackStatusReader().read()).resolves.toMatchObject({
      state: 'offline',
      reason: 'operator_startup_available',
      recoverable: false,
    })
  })

  it('returns a recoverable fault without exposing its file path', async () => {
    const directory = await mkdtemp(join(tmpdir(), 'robot-soul-ops-'))
    temporaryDirectories.push(directory)
    const path = join(directory, 'stack.json')
    await writeFile(path, JSON.stringify({
      state: 'faulted',
      reason: 'hardware_stack_unhealthy',
      detail: 'CAN exchange failed.',
      recoverable: true,
      updatedAt: '2026-08-24T08:00:00.000Z',
    }))
    vi.stubEnv('ROBOT_SOUL_STACK_STATUS_FILE', path)

    const snapshot = await createStackStatusReader().read()
    expect(snapshot).toEqual({
      state: 'faulted',
      reason: 'hardware_stack_unhealthy',
      detail: 'CAN exchange failed.',
      recoverable: true,
      updatedAt: '2026-08-24T08:00:00.000Z',
    })
    expect(JSON.stringify(snapshot)).not.toContain(directory)
  })

  it('reports operator startup as a non-recoverable in-progress state', async () => {
    const directory = await mkdtemp(join(tmpdir(), 'robot-soul-ops-'))
    temporaryDirectories.push(directory)
    const path = join(directory, 'stack.json')
    await writeFile(path, JSON.stringify({
      state: 'starting',
      reason: 'operator_startup',
      detail: 'Starting the robot stack.',
      progress: 15,
      operation: 'Starting the control stack',
      recoverable: false,
      updatedAt: '2026-08-24T08:00:00.000Z',
    }))
    vi.stubEnv('ROBOT_SOUL_STACK_STATUS_FILE', path)

    await expect(createStackStatusReader().read()).resolves.toMatchObject({
      state: 'starting',
      reason: 'operator_startup',
      progress: 15,
      operation: 'Starting the control stack',
      recoverable: false,
    })
  })

  it('reports operator shutdown as a non-recoverable in-progress state', async () => {
    const directory = await mkdtemp(join(tmpdir(), 'robot-soul-ops-'))
    temporaryDirectories.push(directory)
    const path = join(directory, 'stack.json')
    await writeFile(path, JSON.stringify({
      state: 'stopping',
      reason: 'operator_shutdown',
      detail: 'Stopping the hardware plane.',
      recoverable: false,
      updatedAt: '2026-08-24T08:00:00.000Z',
    }))
    vi.stubEnv('ROBOT_SOUL_STACK_STATUS_FILE', path)

    await expect(createStackStatusReader().read()).resolves.toMatchObject({
      state: 'stopping',
      reason: 'operator_shutdown',
      recoverable: false,
    })
  })

  it('reads the Python Agent model-loading stage without exposing its path', async () => {
    const directory = await mkdtemp(join(tmpdir(), 'robot-soul-ops-'))
    temporaryDirectories.push(directory)
    const path = join(directory, 'agent-status.json')
    await writeFile(path, JSON.stringify({
      state: 'loading',
      current: 3,
      total: 6,
      label: 'GroundingDINO + SAM2',
      detail: 'Detection and segmentation',
      stages: ['arm', 'calibration', 'camera', 'grounding', 'vggt', 'ram'],
      error: null,
    }))
    vi.stubEnv('ROBOT_SOUL_AGENT_STATUS_FILE', path)

    const snapshot = await createAgentBootstrapStatusReader().read()
    expect(snapshot).toEqual({
      state: 'loading',
      current: 3,
      total: 6,
      label: 'GroundingDINO + SAM2',
      detail: 'Detection and segmentation',
    })
    expect(JSON.stringify(snapshot)).not.toContain(directory)
  })

  it('fails closed when the supervisor status is malformed', async () => {
    const directory = await mkdtemp(join(tmpdir(), 'robot-soul-ops-'))
    temporaryDirectories.push(directory)
    const path = join(directory, 'stack.json')
    await writeFile(path, '{not-json')
    vi.stubEnv('ROBOT_SOUL_STACK_STATUS_FILE', path)

    await expect(createStackStatusReader().read()).resolves.toMatchObject({
      state: 'faulted',
      reason: 'invalid_supervisor_status',
      recoverable: false,
    })
  })
})
