import { DeepSeekTaskPlanner } from './agent-planner.ts'
import type { RobotDriver, TaskPlanningProvider } from './contracts.ts'
import { DryRunRobotDriver } from './executor.ts'
import { TaskPlanner } from './planner.ts'
import { PythonAgentClient } from './python-agent-client.ts'
import { RobotRamBridgeDriver } from './robotram-driver.ts'
import { startPythonAgentServer, startRobotAssistantServer } from './server.ts'
import { SkillRegistry } from './skill-registry.ts'
import { CLEAN_TABLE_SKILLS } from './skills/clean-table.ts'

const portValue = Number.parseInt(process.env.ROBOT_ASSISTANT_PORT ?? '3090', 10)
if (!Number.isInteger(portValue) || portValue < 1 || portValue > 65_535) {
  throw new Error('ROBOT_ASSISTANT_PORT must be an integer from 1 to 65535.')
}

const registry = new SkillRegistry()
for (const skill of CLEAN_TABLE_SKILLS) registry.register(skill)

function createDriver(mode: 'dry-run' | 'robotram'): RobotDriver {
  if (mode === 'dry-run') return new DryRunRobotDriver()
  const timeoutMs = Number.parseInt(
    process.env.ROBOTRAM_OPERATION_TIMEOUT_MS ?? '300000',
    10,
  )
  return new RobotRamBridgeDriver({
    endpoint: new URL(process.env.ROBOTRAM_BRIDGE_URL ?? 'http://127.0.0.1:8091'),
    confirmation: process.env.ROBOT_ASSISTANT_HARDWARE_CONFIRM ?? '',
    bridgeToken: process.env.ROBOTRAM_BRIDGE_TOKEN ?? '',
    timeoutMs,
  })
}

function createPlanner(mode: 'deterministic' | 'deepseek'): TaskPlanningProvider {
  if (mode === 'deterministic') return new TaskPlanner(registry)
  return new DeepSeekTaskPlanner(registry, {
    apiKey: process.env.DEEPSEEK_API_KEY ?? '',
    baseUrl: new URL(process.env.DEEPSEEK_BASE_URL ?? 'https://api.deepseek.com'),
    model: process.env.DEEPSEEK_MODEL ?? 'deepseek-chat',
    timeoutMs: Number.parseInt(process.env.DEEPSEEK_TIMEOUT_MS ?? '30000', 10),
  })
}

const configuredRuntime = process.env.ROBOT_ASSISTANT_RUNTIME ?? 'mvp'
if (configuredRuntime !== 'mvp' && configuredRuntime !== 'python-agent') {
  throw new Error('ROBOT_ASSISTANT_RUNTIME must be mvp or python-agent.')
}

let server: ReturnType<typeof startRobotAssistantServer>
if (configuredRuntime === 'python-agent') {
  const agentPort = Number.parseInt(process.env.ROBOT_AGENT_PORT ?? '15555', 10)
  const timeoutMs = Number.parseInt(process.env.ROBOT_AGENT_TIMEOUT_MS ?? '30000', 10)
  const token = process.env.ROBOT_AGENT_TOKEN
  const client = new PythonAgentClient({
    host: process.env.ROBOT_AGENT_HOST ?? '127.0.0.1',
    port: agentPort,
    timeoutMs,
    ...token === undefined || token.length === 0 ? {} : { token },
  })
  const outputDir = process.env.ROBOT_AGENT_OUTPUT_DIR ?? process.cwd()
  server = startPythonAgentServer(client, outputDir, portValue)
  server.once('listening', () => {
    console.log(`Robot Assistant: http://127.0.0.1:${portValue}`)
    console.log(`Runtime: python-agent (${process.env.ROBOT_AGENT_HOST ?? '127.0.0.1'}:${agentPort})`)
  })
} else {
  const configuredDriverMode = process.env.ROBOT_ASSISTANT_DRIVER ?? 'dry-run'
  if (configuredDriverMode !== 'dry-run' && configuredDriverMode !== 'robotram') {
    throw new Error('ROBOT_ASSISTANT_DRIVER must be dry-run or robotram.')
  }
  const driverMode: 'dry-run' | 'robotram' = configuredDriverMode

  const configuredPlannerMode = process.env.ROBOT_ASSISTANT_PLANNER ?? 'deterministic'
  if (configuredPlannerMode !== 'deterministic' && configuredPlannerMode !== 'deepseek') {
    throw new Error('ROBOT_ASSISTANT_PLANNER must be deterministic or deepseek.')
  }
  const plannerMode: 'deterministic' | 'deepseek' = configuredPlannerMode
  const driver = createDriver(driverMode)
  const planner = createPlanner(plannerMode)
  server = startRobotAssistantServer(
    registry,
    planner,
    plannerMode,
    driver,
    driverMode,
    portValue,
  )
  server.once('listening', () => {
    console.log(`Robot Assistant MVP: http://127.0.0.1:${portValue}`)
    console.log(`Planner mode: ${plannerMode}`)
    console.log(`Driver mode: ${driverMode}`)
  })
}

server.once('listening', () => {
  console.log(`Runtime mode: ${configuredRuntime}`)
})

let shutdownStarted = false
for (const signal of ['SIGINT', 'SIGTERM'] as const) {
  process.once(signal, () => {
    if (shutdownStarted) return
    shutdownStarted = true

    const forceCloseTimer = setTimeout(() => {
      server.closeAllConnections()
    }, 2_000)
    forceCloseTimer.unref()

    const hardExitTimer = setTimeout(() => {
      console.error('Robot Assistant HTTP server did not close within 5 seconds.')
      process.exit(1)
    }, 5_000)
    hardExitTimer.unref()

    server.closeIdleConnections()
    server.close(() => {
      clearTimeout(forceCloseTimer)
      clearTimeout(hardExitTimer)
      process.exit(0)
    })
  })
}
