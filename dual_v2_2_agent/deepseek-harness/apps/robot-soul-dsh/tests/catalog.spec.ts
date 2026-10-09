import { afterEach, describe, expect, it, vi } from 'vitest'

import {
  PRODUCT_CATALOG,
  STATIC_SAFETY,
  validateCatalog,
} from '../src/catalog.ts'
import {
  HARDWARE_CONFIRMATION,
  hardwareAccessPolicy,
  isProductionAgentReady,
  mergeLiveManifest,
  normalizeHardwareManifest,
  normalizeHardwareStatus,
} from '../src/live.ts'

afterEach(() => {
  vi.unstubAllEnvs()
})

describe('robot capability catalog', () => {
  it('covers all reviewed Skills, atomic Tools and Harness Tools', () => {
    expect(PRODUCT_CATALOG.filter(item => item.kind === 'skill').map(item => item.name))
      .toEqual([
        'perceive_scene',
        'pick_and_place',
        'tidy_table',
        'safety_reset',
        'start_robot',
      ])
    expect(PRODUCT_CATALOG.filter(item => item.kind === 'tool').map(item => item.name))
      .toEqual([
        'ground',
        'estimate_grasp_poses',
        'localize',
        'approach',
        'grasp',
        'place',
        'transport',
        'reset',
      ])
    expect(PRODUCT_CATALOG.filter(item => item.kind === 'harness-tool').map(item => item.name))
      .toEqual([
        'robot_status',
        'robot_manifest',
        'robot_plan',
        'robot_request_execution',
        'robot_execution_status',
      ])
  })

  it('has unique names, valid dependencies and complete product metadata', () => {
    expect(validateCatalog()).toEqual([])
    expect(new Set(PRODUCT_CATALOG.map(item => item.name)).size).toBe(18)
    for (const item of PRODUCT_CATALOG) {
      expect(item.description.length).toBeGreaterThan(10)
      expect(item.source.length).toBeGreaterThan(0)
      expect(['L0', 'L1', 'L2', 'L3']).toContain(item.safety)
      for (const property of Object.values(item.parameters)) {
        expect(property.type.length).toBeGreaterThan(0)
        expect(property.description.length).toBeGreaterThan(0)
      }
    }
  })

  it('keeps static safety authoritative when live manifest disagrees', () => {
    const merged = mergeLiveManifest({
      tools: [{ name: 'reset', safety: 'L0' }],
      skills: [{ name: 'start_robot', safety: 'L0' }],
    }) as {
      products: Array<{
        name: string
        available: boolean
        safety: string
        safetySource: string
      }>
    }
    const reset = merged.products.find(item => item.name === 'reset')
    const start = merged.products.find(item => item.name === 'start_robot')
    expect(reset).toMatchObject({
      available: true,
      safety: STATIC_SAFETY.get('reset'),
      safetySource: 'static-reviewed-catalog',
    })
    expect(start).toMatchObject({
      available: true,
      safety: 'L3',
      safetySource: 'static-reviewed-catalog',
    })
  })

  it('opens only the explicit stage-one read-only hardware boundary', () => {
    vi.stubEnv('ROBOT_SOUL_HARDWARE_READONLY', '1')
    expect(hardwareAccessPolicy()).toEqual({
      stage: 'hardware-readonly',
      readonlyEnabled: true,
      executeEnabled: false,
      hardwareOwner: 'python-agent',
      allowedMethods: ['health', 'manifest'],
      blockedMethods: [
        'plan',
        'execute',
        'task_status',
        'task_events',
        'cancel',
        'call',
      ],
      nextStage: 'approval-gated-execute-after-validation',
    })
  })

  it('opens approval execution only when both gates match exactly', () => {
    vi.stubEnv('ROBOT_SOUL_HARDWARE_READONLY', '1')
    vi.stubEnv('ROBOT_SOUL_APPROVED_EXECUTION', HARDWARE_CONFIRMATION)
    expect(hardwareAccessPolicy()).toMatchObject({
      stage: 'approval-gated-execute',
      readonlyEnabled: true,
      executeEnabled: true,
      hardwareOwner: 'python-agent',
      blockedMethods: ['call'],
      nextStage: 'operator-approved-high-level-skills',
    })
    expect(hardwareAccessPolicy().allowedMethods).toContain('execute')

    vi.stubEnv('ROBOT_SOUL_APPROVED_EXECUTION', 'yes')
    expect(() => hardwareAccessPolicy()).toThrow('must exactly equal')
  })

  it('normalizes production Agent data without exporting execution capability', () => {
    expect(normalizeHardwareStatus({
      planOnly: false,
      bootstrapState: 'ready',
      activeTaskId: null,
      allowedMethods: ['execute'],
    })).toMatchObject({
      mode: 'hardware-readonly',
      provider: 'python-agent-readonly',
      hardwareOwner: 'python-agent',
      readonly: true,
      planOnly: false,
      bootstrapState: 'ready',
      ready: true,
      activeTask: false,
    })
    expect(normalizeHardwareManifest({
      tools: [{ name: 'reset' }],
      skills: [{ name: 'safety_reset' }],
    })).toMatchObject({
      mode: 'hardware-readonly',
      readonly: true,
      tools: [{ name: 'reset' }],
      skills: [{ name: 'safety_reset' }],
    })
  })

  it('uses the launcher-compatible production readiness rule', () => {
    expect(isProductionAgentReady({ bootstrapState: 'ready' })).toBe(true)
    expect(isProductionAgentReady({ ready: true })).toBe(true)
    expect(isProductionAgentReady({ bootstrapState: 'bootstrapping' })).toBe(false)
    expect(isProductionAgentReady({})).toBe(false)
  })
})
