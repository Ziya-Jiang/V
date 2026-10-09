import { createHash } from 'node:crypto'

import type { SimulatedEvent, SimulatedPlan, SimulatedStep } from './contracts.ts'

const MAX_INSTRUCTION_CHARS = 2_000
const FORBIDDEN_LOW_LEVEL = [
  /关节角/i,
  /\bjoint\s*(?:angle|position)/i,
  /\b(?:rpm|torque|pwm)\b/i,
  /\bsudo\b/i,
  /\b(?:bash|shell)\b/i,
  /\bcan(?:0|1|able|fd)\b/i,
]

function constraints(): Readonly<Record<string, unknown>> {
  return {
    mode: 'simulation',
    hardwareAccess: false,
    approvalRequiredForFutureExecution: true,
    lowLevelCommandsAccepted: false,
    maxVelocity: 'not applicable in simulation',
  }
}

function sceneStep(instruction: string): SimulatedStep {
  return {
    ref: 'step-01',
    skill: 'perceive_scene',
    args: { instruction },
    constraints: { readOnly: true, frameSource: 'simulated' },
    tools: ['ground', 'estimate_grasp_poses', 'localize'],
  }
}

function resetStep(ref: string): SimulatedStep {
  return {
    ref,
    skill: 'safety_reset',
    args: { arms: 'both', kind: 'safe' },
    constraints: { recoveryOnly: true, execution: false },
    tools: ['place', 'reset'],
  }
}

function chooseSteps(instruction: string): readonly SimulatedStep[] {
  const normalized = instruction.toLowerCase()
  if (/复位|归位|回预备位|reset/.test(normalized)) {
    return [resetStep('step-01')]
  }
  if (/整理|收拾|清理|tidy|clean/.test(normalized)) {
    return [
      sceneStep(instruction),
      {
        ref: 'step-02',
        skill: 'tidy_table',
        args: { instruction },
        constraints: { planOnly: true, placements: 4 },
        tools: ['approach', 'grasp', 'transport', 'place', 'reset'],
      },
      resetStep('step-03'),
    ]
  }
  if (/看|找|识别|定位|观察|see|find|locate|detect/.test(normalized)
    && !/拿|抓|放|移动|pick|place|move/.test(normalized)) {
    return [sceneStep(instruction)]
  }
  return [
    sceneStep(instruction),
    {
      ref: 'step-02',
      skill: 'pick_and_place',
      args: {
        object: 'semantic:source_object',
        target: 'semantic:destination_object',
        placement_type: /旁边|beside|near/.test(normalized) ? 'beside' : 'on_center',
        plan_only: true,
      },
      constraints: { semanticReferences: true, planOnly: true },
      tools: ['approach', 'grasp', 'transport', 'place', 'reset'],
    },
    resetStep('step-03'),
  ]
}

function buildEvents(steps: readonly SimulatedStep[]) {
  const events: SimulatedEvent[] = [{
    offsetMs: 0,
    type: 'plan.created',
    status: 'ready',
    message: 'The read-only simulator created a Plan without connecting to or initializing hardware.',
  }]
  steps.forEach((step, index) => {
    events.push({
      offsetMs: 420 + index * 680,
      type: 'skill.preview',
      status: 'simulated',
      message: `Previewed ${step.skill} with ${step.tools.length} Tool dependencies.`,
      skill: step.skill,
    })
  })
  events.push({
    offsetMs: 420 + steps.length * 680,
    type: 'plan.completed',
    status: 'dry-run',
    message: 'Simulation complete. The Plan contains no execution credential and cannot move hardware.',
  })
  return events
}

function planIdFor(instruction: string): string {
  return `sim_${createHash('sha256').update(instruction).digest('hex').slice(0, 12)}`
}

/** Validate that an instruction is semantic rather than a low-level command. */
export function validateSemanticInstruction(rawInstruction: unknown): string {
  if (typeof rawInstruction !== 'string') {
    throw new TypeError('instruction must be a string.')
  }
  const instruction = rawInstruction.trim()
  if (!instruction) throw new TypeError('Enter a robot task goal first.')
  if (instruction.length > MAX_INSTRUCTION_CHARS) {
    throw new RangeError(`instruction cannot exceed ${MAX_INSTRUCTION_CHARS} characters.`)
  }
  if (FORBIDDEN_LOW_LEVEL.some(pattern => pattern.test(instruction))) {
    throw new TypeError(
      'Only semantic task goals are accepted; joint angles, velocity, CAN, shell, and other low-level control commands are rejected.',
    )
  }
  return instruction
}

/** Create a deterministic, browser-demo Plan without contacting the robot. */
export function simulatePlan(rawInstruction: unknown): SimulatedPlan {
  const instruction = validateSemanticInstruction(rawInstruction)
  const steps = chooseSteps(instruction)
  const events = buildEvents(steps)
  const createdAt = new Date().toISOString()
  const planId = planIdFor(instruction)
  const ndjson = [
    JSON.stringify({ id: 'demo-1', method: 'plan', params: { instruction } }),
    JSON.stringify({
      id: 'demo-1',
      ok: true,
      data: {
        mode: 'dry-run',
        planId,
        steps: steps.map(step => ({ skill: step.skill, args: step.args })),
      },
    }),
  ]

  return {
    mode: 'simulation',
    provider: 'robot-soul-dsh',
    planId,
    instruction,
    createdAt,
    constraints: constraints(),
    steps,
    events,
    ndjson,
  }
}
