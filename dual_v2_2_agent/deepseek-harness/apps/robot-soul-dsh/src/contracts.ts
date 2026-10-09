export type SafetyLevel = 'L0' | 'L1' | 'L2' | 'L3'
export type ProductKind = 'harness-tool' | 'skill' | 'tool'
export type ProductCategory =
  | 'interface'
  | 'perception'
  | 'manipulation'
  | 'task'
  | 'startup'

export interface SchemaProperty {
  readonly type: string
  readonly description: string
  readonly required?: boolean
  readonly default?: unknown
  readonly options?: readonly string[]
}

export interface SoulProduct {
  readonly id: string
  readonly name: string
  readonly title: string
  readonly tagline: string
  readonly description: string
  readonly kind: ProductKind
  readonly category: ProductCategory
  readonly safety: SafetyLevel
  readonly simulated: boolean
  readonly liveReadonly: boolean
  readonly parameters: Readonly<Record<string, SchemaProperty>>
  readonly returns: string
  readonly dependsOn: readonly string[]
  readonly poweredBy: readonly string[]
  readonly source: string
  readonly example: Readonly<Record<string, unknown>>
}

export interface ArchitectureNode {
  readonly id: string
  readonly title: string
  readonly eyebrow: string
  readonly summary: string
  readonly owns: readonly string[]
  readonly refuses: readonly string[]
  readonly process: string
  readonly source: string
}

export interface ArchitectureEdge {
  readonly from: string
  readonly to: string
  readonly label: string
  readonly protocol: string
}

export interface StorySection {
  readonly id: string
  readonly eyebrow: string
  readonly title: string
  readonly copy: string
}

export interface SimulatedStep {
  readonly ref: string
  readonly skill: string
  readonly args: Readonly<Record<string, unknown>>
  readonly constraints: Readonly<Record<string, unknown>>
  readonly tools: readonly string[]
}

export interface SimulatedPlan {
  readonly mode: 'simulation'
  readonly provider: 'robot-soul-dsh'
  readonly planId: string
  readonly instruction: string
  readonly createdAt: string
  readonly constraints: Readonly<Record<string, unknown>>
  readonly steps: readonly SimulatedStep[]
  readonly events: readonly SimulatedEvent[]
  readonly ndjson: readonly string[]
}

export interface SimulatedEvent {
  readonly offsetMs: number
  readonly type: string
  readonly status: string
  readonly message: string
  readonly skill?: string
}

export interface LiveReader {
  status(signal?: AbortSignal): Promise<unknown>
  manifest(signal?: AbortSignal): Promise<unknown>
}

export interface LiveSnapshot {
  readonly connected: boolean
  readonly readonly: true
  readonly data?: unknown
  readonly reason?: string
  readonly checkedAt: string
}

export interface HarnessStatusSnapshot {
  readonly connected: boolean
  readonly url: string
  readonly checkedAt: string
  readonly reason?: string
}

export interface HarnessStatusReader {
  readonly url: string
  read(): Promise<HarnessStatusSnapshot>
}

export type StackLifecycleState =
  | 'offline'
  | 'healthy'
  | 'faulted'
  | 'recovering'
  | 'starting'
  | 'stopping'

export interface StackStatusSnapshot {
  readonly state: StackLifecycleState
  readonly reason?: string
  readonly detail?: string
  readonly progress?: number
  readonly operation?: string
  readonly recoverable: boolean
  readonly updatedAt: string
}

export interface StackStatusReader {
  read(): Promise<StackStatusSnapshot>
}

export interface AgentBootstrapStatusSnapshot {
  readonly state: 'error' | 'loading' | 'ready' | 'unavailable'
  readonly current: number
  readonly total: number
  readonly label?: string
  readonly detail?: string
  readonly error?: string
}

export interface AgentBootstrapStatusReader {
  read(): Promise<AgentBootstrapStatusSnapshot>
}

export interface HardwareRecoveryLauncher {
  start(): Promise<{ readonly accepted: true; readonly pid?: number }>
}

export interface HardwareStartupLauncher {
  start(): Promise<{ readonly accepted: true; readonly pid?: number }>
}

export interface HardwareStopLauncher {
  stop(): Promise<{ readonly accepted: true; readonly pid?: number }>
}
