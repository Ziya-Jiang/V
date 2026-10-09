import { Context } from '@deepseek-ai/cordis'
import { describe, expect, it } from 'vitest'

import {
  RobotError,
  RobotRuntime,
  type RobotManifest,
  type RobotPlan,
  type RobotPlanRequest,
  type RobotStatus,
} from '../src/index.ts'

class TestRobotRuntime extends RobotRuntime {
  status(): Promise<RobotStatus> {
    return Promise.resolve({
      mode: 'dry-run',
      provider: 'test',
      planOnlyReady: true,
      health: {},
    })
  }

  manifest(): Promise<RobotManifest> {
    return Promise.resolve({
      mode: 'dry-run',
      provider: 'test',
      tools: [],
      skills: [],
    })
  }

  plan(request: RobotPlanRequest): Promise<RobotPlan> {
    return Promise.resolve({
      mode: 'dry-run',
      provider: 'test',
      planId: 'plan-1',
      instruction: request.instruction,
      outputDir: null,
      plan: {},
    })
  }
}

describe('RobotRuntime', () => {
  it('registers one plan-only robot service on the Cordis context', async () => {
    const ctx = new Context()
    new TestRobotRuntime(ctx)

    expect(ctx.robot).toBeDefined()
    await expect(ctx.robot.status()).resolves.toMatchObject({
      mode: 'dry-run',
      planOnlyReady: true,
    })
    await expect(ctx.robot.plan({ instruction: 'inspect the table' }))
      .resolves.toMatchObject({ planId: 'plan-1' })
  })

  it('rejects a second provider through Cordis service ownership', () => {
    const ctx = new Context()
    new TestRobotRuntime(ctx)

    expect(() => new TestRobotRuntime(ctx)).toThrow()
  })
})

describe('RobotError', () => {
  it('retains its stable category and cause', () => {
    const cause = new Error('socket failed')
    const error = new RobotError(
      'ROBOT_CONNECTION',
      'could not reach the robot agent',
      { cause },
    )

    expect(error).toMatchObject({
      name: 'RobotError',
      code: 'ROBOT_CONNECTION',
      message: 'could not reach the robot agent',
      cause,
    })
  })
})
