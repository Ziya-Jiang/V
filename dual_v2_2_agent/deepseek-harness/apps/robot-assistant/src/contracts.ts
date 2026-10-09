/** Atomic robot operations exposed by the execution layer. */
export type AtomicOperation = 'approach' | 'grasp' | 'place' | 'home'

/** Task and step lifecycle states projected to the user interface. */
export type RunStatus = 'queued' | 'planning' | 'ready' | 'running' | 'succeeded' | 'failed'

/** A deterministic safety rule attached to a skill invocation. */
export interface Constraint {
  readonly kind: 'workspace' | 'speed' | 'force' | 'collision' | 'approval'
  readonly description: string
  readonly value?: number | string
  readonly unit?: string
}

/** Model-independent input passed from the middle layer to one skill. */
export interface SkillInput {
  readonly object?: string
  readonly destination?: string
}

/** One validated skill invocation in the task intermediate representation. */
export interface SkillInvocation {
  readonly id: string
  readonly skillId: string
  readonly operation: AtomicOperation
  readonly input: SkillInput
  readonly constraints: readonly Constraint[]
}

/** Serializable task intermediate representation produced before execution. */
export interface TaskPlan {
  readonly id: string
  readonly intent: string
  readonly createdAt: string
  readonly steps: readonly SkillInvocation[]
}

/** User request accepted by the outer communication layer. */
export interface TaskRequest {
  readonly text: string
}

/** Replaceable middle-layer provider; model planners and deterministic planners share this API. */
export interface TaskPlanningProvider {
  plan(request: TaskRequest): Promise<TaskPlan> | TaskPlan
}

/** Event stream shared by the executor and graphical interface. */
export interface ExecutionEvent {
  readonly timestamp: string
  readonly taskId: string
  readonly type: 'plan.created' | 'task.status' | 'step.started' | 'step.completed' | 'step.failed'
  readonly status?: RunStatus
  readonly stepId?: string
  readonly message: string
}

/** Result returned by a robot driver for one atomic operation. */
export interface OperationResult {
  readonly ok: boolean
  readonly detail: string
}

/** Narrow driver interface; skills never invoke robot processes directly. */
export interface RobotDriver {
  run(operation: AtomicOperation, input: SkillInput, signal: AbortSignal): Promise<OperationResult>
}

/** Execution context provided to a registered skill. */
export interface SkillContext {
  readonly driver: RobotDriver
  readonly signal: AbortSignal
}

/** Plug-in unit discovered by the bottom-layer skill registry. */
export interface SkillDefinition {
  readonly id: string
  readonly title: string
  readonly description: string
  readonly operation: AtomicOperation
  readonly constraints: readonly Constraint[]
  execute(context: SkillContext, input: SkillInput): Promise<OperationResult>
}
