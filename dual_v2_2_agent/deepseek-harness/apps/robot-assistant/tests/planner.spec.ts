import { describe, expect, it } from 'vitest'

import { taskPlanFromAgentJson } from '../src/agent-planner.ts'
import { TaskPlanner } from '../src/planner.ts'
import { RobotRamBridgeDriver } from '../src/robotram-driver.ts'
import { SkillRegistry } from '../src/skill-registry.ts'
import { CLEAN_TABLE_SKILLS } from '../src/skills/clean-table.ts'

function createPlanner(): TaskPlanner {
  const registry = new SkillRegistry()
  for (const skill of CLEAN_TABLE_SKILLS) registry.register(skill)
  return new TaskPlanner(registry)
}

describe('TaskPlanner', () => {
  it('decomposes a clean-table request into object operations and home', () => {
    const plan = createPlanner().plan({ text: '整理桌面上的杯子和碗' })
    expect(plan.steps.map(step => [step.operation, step.input.object])).toEqual([
      ['approach', 'cup'],
      ['grasp', 'cup'],
      ['place', 'cup'],
      ['approach', 'bowl'],
      ['grasp', 'bowl'],
      ['place', 'bowl'],
      ['home', undefined],
    ])
    expect(plan.steps.every(step => step.constraints.length > 0)).toBe(true)
  })

  it('rejects unsupported intent before execution', () => {
    expect(() => createPlanner().plan({ text: '打开窗户' }))
      .toThrow('clean-table')
  })
})

describe('RobotRamBridgeDriver', () => {
  it('rejects hardware mode without the explicit confirmation', () => {
    expect(() => new RobotRamBridgeDriver({
      endpoint: new URL('http://127.0.0.1:8091'),
      confirmation: '',
      bridgeToken: 'test-token-with-24-characters',
      timeoutMs: 30_000,
    })).toThrow('ENABLE_DUAL_V2_2_HARDWARE')
  })

  it('rejects non-loopback bridge endpoints', () => {
    expect(() => new RobotRamBridgeDriver({
      endpoint: new URL('https://robot.example.com'),
      confirmation: 'ENABLE_DUAL_V2_2_HARDWARE',
      bridgeToken: 'test-token-with-24-characters',
      timeoutMs: 30_000,
    })).toThrow('loopback')
  })
})

describe('taskPlanFromAgentJson', () => {
  it('injects registry constraints and accepts no model-authored robot command', () => {
    const registry = new SkillRegistry()
    for (const skill of CLEAN_TABLE_SKILLS) registry.register(skill)
    const plan = taskPlanFromAgentJson({
      steps: [
        { skillId: 'robot.approach-object', input: { object: 'cup' } },
        { skillId: 'robot.grasp-object', input: { object: 'cup' } },
        { skillId: 'robot.place-object', input: { object: 'cup', destination: 'cup-zone' } },
        { skillId: 'robot.return-home', input: {} },
      ],
    }, { text: '整理杯子' }, registry)
    expect(plan.steps[1]?.constraints.some(item => item.kind === 'force')).toBe(true)
  })

  it('rejects model-authored coordinates', () => {
    const registry = new SkillRegistry()
    for (const skill of CLEAN_TABLE_SKILLS) registry.register(skill)
    expect(() => taskPlanFromAgentJson({
      steps: [
        {
          skillId: 'robot.grasp-object',
          input: { object: 'cup', x: 120 },
        },
        { skillId: 'robot.return-home', input: {} },
      ],
    }, { text: '整理杯子' }, registry)).toThrow('unsupported input field')
  })
})
