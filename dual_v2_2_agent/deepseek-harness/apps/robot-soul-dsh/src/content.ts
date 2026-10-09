import type {
  ArchitectureEdge,
  ArchitectureNode,
  StorySection,
} from './contracts.ts'

export const STORY: readonly StorySection[] = [
  {
    id: 'hero',
    eyebrow: 'HKCLR · AGENTIC ROBOT RUNTIME',
    title: 'The agentic operating layer for robots.',
    copy: 'Harness Agent interprets intent, gathers world context, selects verified Skills, builds an auditable Plan, and coordinates supervised execution.',
  },
  {
    id: 'store',
    eyebrow: 'CAPABILITY CATALOG',
    title: 'Inspect every capability before you use it.',
    copy: 'Browse Skills, atomic Tools, inputs, outputs, dependencies, and reviewed safety levels in one searchable catalog.',
  },
  {
    id: 'playground',
    eyebrow: 'PLAN PLAYGROUND',
    title: 'Preview the execution structure without moving hardware.',
    copy: 'The simulator shows selected Skills, underlying Tools, constraints, events, and the raw protocol envelope.',
  },
  {
    id: 'architecture',
    eyebrow: 'SYSTEM ARCHITECTURE',
    title: 'Follow one goal through the complete agentic operating stack.',
    copy: 'Harness Agent owns intent and orchestration. The supervised Python Agent and RobotRAM own execution state and hardware resources.',
  },
  {
    id: 'protocol',
    eyebrow: 'PROTOCOL LAB',
    title: 'See the exact messages exchanged between processes.',
    copy: 'Each request is one NDJSON envelope. Every response carries an id, an outcome, and either data or a structured error.',
  },
  {
    id: 'safety',
    eyebrow: 'SAFETY MODEL',
    title: 'Execution requires independent checks at every boundary.',
    copy: 'The model may request execution but cannot approve itself. The operator, website, Python Agent, supervisor, and physical emergency stop each enforce a separate boundary.',
  },
  {
    id: 'extend',
    eyebrow: 'DEVELOPER GUIDE',
    title: 'Add a capability without widening hardware permissions.',
    copy: 'Define a typed atomic Tool, compose it into a Skill, then connect it through the RobotRuntime Provider while preserving existing policy checks.',
  },
]

export const ARCHITECTURE_NODES: readonly ArchitectureNode[] = [
  {
    id: 'user',
    title: 'User',
    eyebrow: 'GOAL',
    summary: 'Describes the desired outcome in plain language instead of sending joint or motor commands.',
    owns: ['Task goal', 'Plan review', 'Final approval for each hardware Plan'],
    refuses: ['Direct motor control', 'Bypassing execution policy'],
    process: 'Browser or CLI',
    source: 'apps/robot-soul-dsh/src/ui.ts',
  },
  {
    id: 'harness',
    title: 'Harness Agent',
    eyebrow: 'AGENTIC ORCHESTRATION',
    summary: 'Interprets intent, acquires environment context, selects model-facing Tools and Skills, and records observable planning events.',
    owns: ['Intent and context', 'Skill orchestration', 'Model session and plugin lifecycle'],
    refuses: ['Acting as a hardware driver', 'Holding robot device handles'],
    process: 'Node.js Harness process',
    source: 'packages/core/runtime',
  },
  {
    id: 'tool-robot',
    title: 'dsh-tool-robot',
    eyebrow: 'MODEL INTERFACE',
    summary: 'Exposes three read/plan Tools by default and two narrow request/status Tools only when explicitly enabled.',
    owns: ['Model-facing schemas', 'Timeout budgets', 'Approval instructions'],
    refuses: ['approve', 'Direct execute', 'cancel', 'Arbitrary call'],
    process: 'Cordis plugin inside Harness',
    source: 'packages/robot/tool-robot/src/index.ts',
  },
  {
    id: 'robot-service',
    title: 'ctx.robot',
    eyebrow: 'SERVICE CONTRACT',
    summary: 'A stable RobotRuntime interface that separates planning code from the concrete transport Provider.',
    owns: ['manifest()', 'plan()', 'status()', 'requestExecution()', 'executionStatus()'],
    refuses: ['Leaking transport details', 'Granting hardware access by default'],
    process: 'Cordis Service inside Harness',
    source: 'packages/robot/robot/src/index.ts',
  },
  {
    id: 'provider',
    title: 'Cordis Provider',
    eyebrow: 'TRANSPORT ADAPTER',
    summary: 'Translates RobotRuntime calls into loopback NDJSON and verifies the Agent operating mode.',
    owns: ['127.0.0.1 transport', 'Request ids', 'Response-size and timeout limits'],
    refuses: ['Non-loopback hosts', 'Unapproved production Agents', 'Human approval and direct execution'],
    process: 'Provider inside Harness',
    source: 'packages/robot/robot-python-agent/src/index.ts',
  },
  {
    id: 'python-agent',
    title: 'Python Agent',
    eyebrow: 'SOLE HARDWARE OWNER',
    summary: 'Owns RobotContext, the Skill registry, approval records, and task state. launch_robot_assistant.sh starts it only after control-stack preflight and supervises its health.',
    owns: ['Skill and Tool registry', 'Immutable Plans', 'Execution lock, task state, and cooperative cancellation'],
    refuses: ['Hardware bootstrap in plan-only mode', 'Concurrent hardware owners', 'Execution without confirmation'],
    process: 'Supervised Python process',
    source: 'robot_agent/server.py',
  },
  {
    id: 'robotram',
    title: 'RobotRAM',
    eyebrow: 'ROBOT CAPABILITIES',
    summary: 'Packages perception, pose estimation, motion planning, and existing control code as composable capabilities.',
    owns: ['Vision models', 'Scene state', 'Pick-and-place algorithms'],
    refuses: ['Accepting unvalidated browser commands'],
    process: 'Python runtime and model services',
    source: 'robot_agent/context.py',
  },
  {
    id: 'hardware',
    title: 'Control / Hardware',
    eyebrow: 'PHYSICAL PLANE',
    summary: 'Runs real-time control, motion planning, arms, grippers, and cameras. The physical emergency stop remains independent of software.',
    owns: ['Real-time loops', 'CAN and UDP transport', 'Physical motion'],
    refuses: ['Natural language', 'Browser requests', 'Unapproved execution'],
    process: 'rt_control / upper_software / upper_planner',
    source: '../dual_v2_2_control',
  },
]

export const ARCHITECTURE_EDGES: readonly ArchitectureEdge[] = [
  { from: 'user', to: 'harness', label: 'Goal', protocol: 'UI / Tool intent' },
  { from: 'harness', to: 'tool-robot', label: 'Select Tool', protocol: 'Cordis Tools' },
  { from: 'tool-robot', to: 'robot-service', label: 'Typed call', protocol: 'RobotRuntime' },
  { from: 'robot-service', to: 'provider', label: 'Resolve service', protocol: 'Cordis Service' },
  { from: 'provider', to: 'python-agent', label: 'Loopback request', protocol: 'NDJSON / TCP' },
  { from: 'python-agent', to: 'robotram', label: 'Run Skill', protocol: 'Python calls' },
  { from: 'robotram', to: 'hardware', label: 'Approved motion', protocol: 'Robot SDK / UDP / CAN' },
]

export const PROTOCOL_EXAMPLES = {
  health: {
    request: { id: 'req-health-1', method: 'health', params: {} },
    response: {
      id: 'req-health-1',
      ok: true,
      data: {
        status: 'ready',
        mode: 'plan-only',
        planOnly: true,
        hardwareInitialized: false,
      },
    },
  },
  manifest: {
    request: { id: 'req-manifest-1', method: 'manifest', params: {} },
    response: {
      id: 'req-manifest-1',
      ok: true,
      data: {
        mode: 'plan-only',
        tools: ['ground', 'estimate_grasp_poses', 'localize', 'approach', 'grasp', 'place', 'transport', 'reset'],
        skills: ['start_robot', 'perceive_scene', 'pick_and_place', 'tidy_table', 'safety_reset'],
      },
    },
  },
  plan: {
    request: {
      id: 'req-plan-1',
      method: 'plan',
      params: { instruction: 'Place the blue cup beside the yellow plate' },
    },
    response: {
      id: 'req-plan-1',
      ok: true,
      data: {
        mode: 'dry-run',
        planId: 'plan_preview_only',
        steps: ['perceive_scene', 'pick_and_place', 'safety_reset'],
      },
    },
  },
} as const

export const SAFETY_SCENARIOS = [
  {
    id: 'read-status',
    prompt: 'The website reads local Agent health.',
    expected: 'allow',
    explanation: 'This is a read-only loopback request and does not initialize hardware.',
  },
  {
    id: 'plan-semantic',
    prompt: 'Preview a Plan for “place the cup beside the plate.”',
    expected: 'allow',
    explanation: 'Planning does not move hardware. A Task exists only after separate per-Plan approval.',
  },
  {
    id: 'joint-command',
    prompt: 'Send left-arm joint angles [0, 30, 60…] from the website.',
    expected: 'deny',
    explanation: 'The website exposes no low-level control route, and model-facing Tools do not accept joint angles.',
  },
  {
    id: 'execute-live',
    prompt: 'Call execute(planId) without operator approval.',
    expected: 'deny',
    explanation: 'The model Provider has no direct approve or execute method. Hardware writes require both server gates, same-origin requests, four checks, and the exact phrase.',
  },
  {
    id: 'physical-estop',
    prompt: 'Rely only on the web Cancel button during unexpected motion.',
    expected: 'deny',
    explanation: 'Web cancellation is cooperative, not an emergency stop. Use the independent physical emergency stop for unexpected motion.',
  },
] as const

export const EXTENSION_STEPS = [
  {
    number: '01',
    title: 'Define a Tool',
    copy: 'Start with the smallest testable operation. Specify its parameter schema, result, and failure conditions.',
    source: 'robot_agent/base.py',
  },
  {
    number: '02',
    title: 'Compose a Skill',
    copy: 'Combine Tools around a task goal, including recovery, progress reporting, and cancellation boundaries.',
    source: 'robot_agent/registry.py',
  },
  {
    number: '03',
    title: 'Connect the Provider',
    copy: 'Implement the stable RobotRuntime interface. The Provider handles transport and invariants without expanding permissions.',
    source: 'packages/robot/robot/src/index.ts',
  },
] as const

export const SITE_CONTENT = {
  productName: 'HKCLR Harness Agent',
  hero: STORY[0],
  story: STORY,
  architecture: {
    nodes: ARCHITECTURE_NODES,
    edges: ARCHITECTURE_EDGES,
  },
  protocol: PROTOCOL_EXAMPLES,
  safety: SAFETY_SCENARIOS,
  extension: EXTENSION_STEPS,
  docs: 'docs/subsystems/robot.md',
  guarantee: 'The model interface exposes no approve, direct execute, cancel, arbitrary call, or confirmation-phrase endpoint.',
} as const
