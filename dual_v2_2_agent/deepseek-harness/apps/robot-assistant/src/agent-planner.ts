import { randomUUID } from 'node:crypto'

import type {
  SkillInput,
  SkillInvocation,
  TaskPlan,
  TaskPlanningProvider,
  TaskRequest,
} from './contracts.ts'
import { SkillRegistry } from './skill-registry.ts'

interface DeepSeekPlannerConfig {
  readonly apiKey: string
  readonly baseUrl: URL
  readonly model: string
  readonly timeoutMs: number
}

function parseSkillInput(value: unknown): SkillInput {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw new Error('Agent skill input must be an object.')
  }
  const record = value as Record<string, unknown>
  const allowedKeys = new Set(['object', 'destination'])
  for (const key of Object.keys(record)) {
    if (!allowedKeys.has(key)) throw new Error(`Agent emitted unsupported input field: ${key}`)
  }
  if (record.object !== undefined && typeof record.object !== 'string') {
    throw new Error('Agent object input must be a string.')
  }
  if (record.destination !== undefined && typeof record.destination !== 'string') {
    throw new Error('Agent destination input must be a string.')
  }
  return {
    ...record.object === undefined ? {} : { object: record.object },
    ...record.destination === undefined ? {} : { destination: record.destination },
  }
}

/** Convert untrusted model JSON into a registry-owned TaskIR. */
export function taskPlanFromAgentJson(
  value: unknown,
  request: TaskRequest,
  registry: SkillRegistry,
): TaskPlan {
  if (typeof value !== 'object' || value === null || Array.isArray(value)
    || !('steps' in value) || !Array.isArray(value.steps)) {
    throw new Error('Agent response must contain a steps array.')
  }
  if (value.steps.length < 1 || value.steps.length > 32) {
    throw new Error('Agent plan must contain 1 to 32 steps.')
  }

  const taskId = randomUUID()
  const steps: SkillInvocation[] = value.steps.map((rawStep, index) => {
    if (typeof rawStep !== 'object' || rawStep === null || Array.isArray(rawStep)
      || !('skillId' in rawStep) || typeof rawStep.skillId !== 'string'
      || !('input' in rawStep)) {
      throw new Error(`Agent step ${index} is invalid.`)
    }
    const skill = registry.get(rawStep.skillId)
    return {
      id: `${taskId}:${index}`,
      skillId: skill.id,
      operation: skill.operation,
      input: parseSkillInput(rawStep.input),
      constraints: skill.constraints,
    }
  })
  if (steps.at(-1)?.operation !== 'home') {
    throw new Error('Agent plan must end with the registered home skill.')
  }
  return {
    id: taskId,
    intent: request.text.trim(),
    createdAt: new Date().toISOString(),
    steps,
  }
}

/** DeepSeek provider that may select skills but cannot author robot commands or constraints. */
export class DeepSeekTaskPlanner implements TaskPlanningProvider {
  readonly #registry: SkillRegistry
  readonly #config: DeepSeekPlannerConfig

  constructor(registry: SkillRegistry, config: DeepSeekPlannerConfig) {
    if (config.apiKey.length === 0) throw new Error('DEEPSEEK_API_KEY is required.')
    if (config.baseUrl.protocol !== 'https:' && config.baseUrl.hostname !== '127.0.0.1') {
      throw new Error('DeepSeek base URL must use HTTPS or loopback HTTP.')
    }
    if (!Number.isInteger(config.timeoutMs) || config.timeoutMs < 1_000) {
      throw new Error('Agent planner timeout must be at least 1000 ms.')
    }
    this.#registry = registry
    this.#config = config
  }

  async plan(request: TaskRequest): Promise<TaskPlan> {
    const skills = this.#registry.list().map(skill => ({
      id: skill.id,
      operation: skill.operation,
      description: skill.description,
    }))
    const response = await fetch(new URL('/chat/completions', this.#config.baseUrl), {
      method: 'POST',
      headers: {
        authorization: `Bearer ${this.#config.apiKey}`,
        'content-type': 'application/json',
      },
      body: JSON.stringify({
        model: this.#config.model,
        temperature: 0,
        response_format: { type: 'json_object' },
        messages: [
          {
            role: 'system',
            content: [
              'You are a robot task planner.',
              'Return JSON only: {"steps":[{"skillId":"...","input":{...}}]}.',
              'Use only listed skills. Never emit coordinates, force, speed, shell, or robot commands.',
              'Every plan must end with robot.return-home.',
              `Skills: ${JSON.stringify(skills)}`,
            ].join('\n'),
          },
          { role: 'user', content: request.text },
        ],
      }),
      signal: AbortSignal.timeout(this.#config.timeoutMs),
    })
    if (!response.ok) {
      throw new Error(`DeepSeek planner failed: HTTP ${response.status}`)
    }
    const payload: unknown = await response.json()
    if (typeof payload !== 'object' || payload === null || !('choices' in payload)
      || !Array.isArray(payload.choices)) {
      throw new Error('DeepSeek planner response is invalid.')
    }
    const choice = payload.choices[0]
    if (typeof choice !== 'object' || choice === null || !('message' in choice)
      || typeof choice.message !== 'object' || choice.message === null
      || !('content' in choice.message) || typeof choice.message.content !== 'string') {
      throw new Error('DeepSeek planner response has no message content.')
    }
    return taskPlanFromAgentJson(JSON.parse(choice.message.content), request, this.#registry)
  }
}
