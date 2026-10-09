import { randomUUID } from 'node:crypto'

import type {
  AtomicOperation,
  SkillInvocation,
  TaskPlan,
  TaskPlanningProvider,
  TaskRequest,
} from './contracts.ts'
import { SkillRegistry } from './skill-registry.ts'

const OBJECT_ALIASES = new Map<string, string>([
  ['杯子', 'cup'],
  ['水杯', 'cup'],
  ['cup', 'cup'],
  ['碗', 'bowl'],
  ['bowl', 'bowl'],
  ['面包', 'bread'],
  ['bread', 'bread'],
  ['叉子', 'fork'],
  ['fork', 'fork'],
])

const SKILL_BY_OPERATION: Readonly<Record<AtomicOperation, string>> = {
  approach: 'robot.approach-object',
  grasp: 'robot.grasp-object',
  place: 'robot.place-object',
  home: 'robot.return-home',
}

/** Replaceable middle-layer planner that emits a deterministic TaskIR. */
export class TaskPlanner implements TaskPlanningProvider {
  readonly #registry: SkillRegistry

  constructor(registry: SkillRegistry) {
    this.#registry = registry
  }

  /** Convert one natural-language MVP request into validated atomic operations. */
  plan(request: TaskRequest): TaskPlan {
    const intent = request.text.trim()
    if (intent.length === 0) {
      throw new Error('Task text must not be empty.')
    }
    if (!/(整理|清理|收拾|clean)/iu.test(intent)) {
      throw new Error('MVP currently supports clean-table requests only.')
    }

    const objects = this.#extractObjects(intent)
    const taskId = randomUUID()
    const steps: SkillInvocation[] = []
    for (const object of objects) {
      steps.push(
        this.#invocation(taskId, steps.length, 'approach', { object }),
        this.#invocation(taskId, steps.length + 1, 'grasp', { object }),
        this.#invocation(taskId, steps.length + 2, 'place', {
          object,
          destination: `${object}-zone`,
        }),
      )
    }
    steps.push(this.#invocation(taskId, steps.length, 'home', {}))

    return {
      id: taskId,
      intent,
      createdAt: new Date().toISOString(),
      steps,
    }
  }

  #extractObjects(intent: string): readonly string[] {
    const objects: string[] = []
    for (const [alias, object] of OBJECT_ALIASES) {
      if (intent.toLocaleLowerCase().includes(alias.toLocaleLowerCase())
        && !objects.includes(object)) {
        objects.push(object)
      }
    }
    return objects.length > 0 ? objects : ['scene-object']
  }

  #invocation(
    taskId: string,
    index: number,
    operation: AtomicOperation,
    input: SkillInvocation['input'],
  ): SkillInvocation {
    const skillId = SKILL_BY_OPERATION[operation]
    const skill = this.#registry.get(skillId)
    return {
      id: `${taskId}:${index}`,
      skillId,
      operation,
      input,
      constraints: skill.constraints,
    }
  }
}
