import type {
  ExecutionEvent,
  OperationResult,
  RobotDriver,
  SkillInput,
  TaskPlan,
} from './contracts.ts'
import { SkillRegistry } from './skill-registry.ts'

export type EventSink = (event: ExecutionEvent) => void

/** Safe default driver that demonstrates the complete flow without hardware I/O. */
export class DryRunRobotDriver implements RobotDriver {
  async run(
    operation: Parameters<RobotDriver['run']>[0],
    input: SkillInput,
    signal: AbortSignal,
  ): Promise<OperationResult> {
    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(resolve, 350)
      signal.addEventListener('abort', () => {
        clearTimeout(timer)
        reject(signal.reason)
      }, { once: true })
    })
    return {
      ok: true,
      detail: `dry-run ${operation} ${input.object ?? 'robot'}`,
    }
  }
}

/** Deterministic bottom-layer executor; it is the sole owner of driver calls. */
export class TaskExecutor {
  readonly #registry: SkillRegistry
  readonly #driver: RobotDriver
  readonly #emit: EventSink

  constructor(
    registry: SkillRegistry,
    driver: RobotDriver,
    emit: EventSink,
  ) {
    this.#registry = registry
    this.#driver = driver
    this.#emit = emit
  }

  /** Execute a validated TaskIR serially and stop at the first failed step. */
  async execute(plan: TaskPlan, signal: AbortSignal): Promise<void> {
    this.#event(plan.id, 'task.status', 'running', 'Task execution started.')
    for (const step of plan.steps) {
      if (signal.aborted) {
        throw signal.reason
      }
      const skill = this.#registry.get(step.skillId)
      if (skill.operation !== step.operation) {
        throw new Error(`Skill operation mismatch: ${step.skillId}`)
      }
      this.#event(plan.id, 'step.started', undefined, skill.title, step.id)
      try {
        const result = await skill.execute({ driver: this.#driver, signal }, step.input)
        if (!result.ok) {
          throw new Error(result.detail)
        }
        this.#event(plan.id, 'step.completed', undefined, result.detail, step.id)
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error)
        this.#event(plan.id, 'step.failed', 'failed', message, step.id)
        this.#event(plan.id, 'task.status', 'failed', 'Task execution failed.')
        return
      }
    }
    this.#event(plan.id, 'task.status', 'succeeded', 'Task execution completed.')
  }

  #event(
    taskId: string,
    type: ExecutionEvent['type'],
    status: ExecutionEvent['status'],
    message: string,
    stepId?: string,
  ): void {
    this.#emit({
      timestamp: new Date().toISOString(),
      taskId,
      type,
      message,
      ...status === undefined ? {} : { status },
      ...stepId === undefined ? {} : { stepId },
    })
  }
}
