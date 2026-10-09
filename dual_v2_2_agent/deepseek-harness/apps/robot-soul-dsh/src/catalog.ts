import type {
  ProductCategory,
  ProductKind,
  SafetyLevel,
  SchemaProperty,
  SoulProduct,
} from './contracts.ts'

const perceptionSource = 'robot_agent/tools/perception.py'
const manipulationSource = 'robot_agent/tools/manipulation.py'
const toolRobotSource = 'packages/robot/tool-robot/src/index.ts'

function product(
  id: string,
  title: string,
  tagline: string,
  description: string,
  kind: ProductKind,
  category: ProductCategory,
  safety: SafetyLevel,
  parameters: Readonly<Record<string, SchemaProperty>>,
  returns: string,
  dependsOn: readonly string[],
  poweredBy: readonly string[],
  source: string,
  example: Readonly<Record<string, unknown>>,
  liveReadonly = false,
): SoulProduct {
  return {
    id,
    name: id,
    title,
    tagline,
    description,
    kind,
    category,
    safety,
    simulated: true,
    liveReadonly,
    parameters,
    returns,
    dependsOn,
    poweredBy,
    source,
    example,
  }
}

const arm: SchemaProperty = {
  type: 'string',
  description: 'Select the left or right arm.',
  required: true,
  options: ['left', 'right'],
}
const position: SchemaProperty = {
  type: 'number[3]',
  description: '[x, y, z] in the robot base frame, in millimeters.',
  required: true,
}
const rotation: SchemaProperty = {
  type: 'number[3]',
  description: '[rx, ry, rz] Euler angles, in degrees.',
  required: true,
}

/**
 * Static safety catalogue. A live manifest may add runtime availability, but
 * must never lower these reviewed safety classifications.
 */
export const PRODUCT_CATALOG: readonly SoulProduct[] = [
  product(
    'robot_status',
    'Runtime Status',
    'Check whether the supervised Agent is ready.',
    'Reads Python Agent health and verifies its process-level planning or approval-gated mode. This Tool cannot control hardware.',
    'harness-tool',
    'interface',
    'L0',
    {},
    'Operating mode, readiness, and health details',
    [],
    [],
    toolRobotSource,
    {},
    true,
  ),
  product(
    'robot_manifest',
    'Capability Manifest',
    'See exactly which Tools and Skills are available.',
    'Reads the Tool and Skill manifest published by the Python Agent. Runtime availability never overrides reviewed permissions.',
    'harness-tool',
    'interface',
    'L0',
    {},
    'Tool and Skill manifest',
    ['robot_status'],
    [],
    toolRobotSource,
    {},
    true,
  ),
  product(
    'robot_plan',
    'Plan Preview',
    'Turn one plain-language goal into a reviewable plan.',
    'Creates an immutable server-side Plan. It cannot cause motion unless the model requests execution and an on-site operator independently approves it.',
    'harness-tool',
    'interface',
    'L0',
    {
      instruction: {
        type: 'string',
        description: 'A semantic task goal. Joint, force, velocity, and shell commands are rejected.',
        required: true,
      },
    },
    'Immutable RobotPlan awaiting independent approval',
    ['robot_status', 'robot_manifest'],
    ['perceive_scene', 'pick_and_place', 'tidy_table', 'safety_reset'],
    toolRobotSource,
    { instruction: 'Place the blue cup beside the yellow plate' },
  ),
  product(
    'robot_request_execution',
    'Request Execution',
    'The model may request execution, but it cannot approve itself.',
    'Places an immutable Plan from robot_plan into the on-site approval queue. This Tool has no confirmation phrase and cannot directly create a hardware Task.',
    'harness-tool',
    'interface',
    'L1',
    {
      planId: {
        type: 'string',
        description: 'Immutable Plan ID returned by robot_plan.',
        required: true,
      },
    },
    'Execution request in awaiting_approval state',
    ['robot_plan'],
    [],
    toolRobotSource,
    { planId: 'plan-…' },
  ),
  product(
    'robot_execution_status',
    'Execution Status',
    'Read authoritative approval and Task state.',
    'Reads an execution request created by the same Harness Provider and its approved Task summary. It cannot approve or modify state.',
    'harness-tool',
    'interface',
    'L0',
    {
      requestId: {
        type: 'string',
        description: 'Request ID returned by robot_request_execution.',
        required: true,
      },
    },
    'Authoritative approval state and Task summary',
    ['robot_request_execution'],
    [],
    toolRobotSource,
    { requestId: 'request-…' },
    true,
  ),
  product(
    'perceive_scene',
    'Perceive Scene',
    'Detect objects and locate them in robot coordinates.',
    'Runs semantic detection, 6-DoF pose estimation, and base-frame localization to produce traceable scene data for manipulation.',
    'skill',
    'perception',
    'L2',
    {
      instruction: { type: 'string', description: 'Scene-level natural-language goal.' },
      targets: { type: 'object[]', description: 'Optional target-object definitions.' },
      categories: { type: 'object[]', description: 'Optional category metadata.' },
    },
    'objects_info in the robot base frame',
    ['ground', 'estimate_grasp_poses', 'localize'],
    ['ground', 'estimate_grasp_poses', 'localize'],
    'robot_agent/skills/perceive.py',
    { instruction: 'Find the blue cup and yellow plate' },
  ),
  product(
    'pick_and_place',
    'Pick & Place',
    'Move one detected object to a reviewed target.',
    'Expands “place A on or beside B” into approach, grasp, transport, place, and reset operations.',
    'skill',
    'manipulation',
    'L2',
    {
      object: { type: 'string', description: 'object_<id> to move.', required: true },
      target: { type: 'string', description: 'Destination object_<id>.', required: true },
      placement_type: {
        type: 'string',
        description: 'Place on the center or beside the target.',
        default: 'on_center',
        options: ['on_center', 'beside'],
      },
      offset: { type: 'number[3]', description: 'Millimeter offset used by beside placement.' },
      plan_only: { type: 'boolean', description: 'Plan without executing.', default: true },
    },
    'Detailed motion subtasks and result',
    ['perceive_scene', 'approach', 'grasp', 'transport', 'place', 'reset'],
    ['approach', 'grasp', 'transport', 'place', 'reset'],
    'robot_agent/skills/pick_and_place.py',
    {
      object: 'object_0',
      target: 'object_1',
      placement_type: 'beside',
      offset: [100, 120, 0],
      plan_only: true,
    },
  ),
  product(
    'tidy_table',
    'Tidy Table',
    'Run a multi-object table organization plan.',
    'Combines scene perception with repeated pick-and-place operations to arrange bowls, bread, forks, and cups by semantic rules.',
    'skill',
    'task',
    'L2',
    {
      instruction: { type: 'string', description: 'Table organization goal.' },
      placements: { type: 'object[]', description: 'Optional explicit placement list.' },
    },
    'Combined result for each placement',
    ['perceive_scene', 'pick_and_place', 'safety_reset'],
    ['perceive_scene', 'pick_and_place'],
    'robot_agent/skills/tidy_table.py',
    { instruction: 'Tidy the table' },
  ),
  product(
    'safety_reset',
    'Safety Reset',
    'Release held objects, then return to a known pose.',
    'Opens both grippers in a fixed recovery sequence and returns the selected arms to a known preparation pose after an execution failure.',
    'skill',
    'manipulation',
    'L2',
    {
      arms: {
        type: 'string',
        description: 'Reset the left arm, right arm, or both.',
        default: 'both',
        options: ['left', 'right', 'both'],
      },
      kind: {
        type: 'string',
        description: 'Named reset pose.',
        default: 'safe',
        options: ['safe', 'first', 'pos', 'fork'],
      },
    },
    'Recovery completion result',
    ['place', 'reset'],
    ['place', 'reset'],
    'robot_agent/skills/safety_reset.py',
    { arms: 'both', kind: 'safe' },
  ),
  product(
    'start_robot',
    'Start Control Stack',
    'Start the real-time hardware control services.',
    'Starts rt_control, upper_software, and upper_planner. The website never calls this side-effecting Skill; launch_robot_assistant.sh owns startup, health checks, rollback, and supervision.',
    'skill',
    'startup',
    'L3',
    {
      monitor_after_start: {
        type: 'boolean',
        description: 'Whether the legacy Skill keeps monitoring after startup.',
        default: false,
      },
    },
    'Control-stack component status',
    [],
    [],
    'robot_agent/skills/start_robot.py',
    { monitor_after_start: false },
  ),
  product(
    'ground',
    'Semantic Grounding',
    'Detect the objects named in the goal.',
    'Uses Grounding-DINO and SAM2 to detect semantic objects and return masks, bounding boxes, and class ids.',
    'tool',
    'perception',
    'L2',
    {
      instruction: { type: 'string', description: 'Scene prompt.', required: true },
      targets: { type: 'object[]', description: 'Target-object definitions.' },
    },
    'masks, xyxys, and class_ids',
    [],
    [],
    perceptionSource,
    { instruction: 'clean the table' },
  ),
  product(
    'estimate_grasp_poses',
    'Estimate Grasp Poses',
    'Estimate where each object is and how to grasp it.',
    'Estimates 6-DoF object and grasp poses from the grounded detections.',
    'tool',
    'perception',
    'L2',
    { categories: { type: 'object[]', description: 'Object-category metadata.' } },
    'objects_meta',
    ['ground'],
    [],
    perceptionSource,
    {},
  ),
  product(
    'localize',
    'Localize to Robot Base',
    'Convert camera poses into robot coordinates.',
    'Uses hand-eye calibration to transform object and grasp poses from the camera frame to the robot base frame.',
    'tool',
    'perception',
    'L2',
    {},
    'objects_info',
    ['estimate_grasp_poses'],
    [],
    perceptionSource,
    {},
  ),
  product(
    'approach',
    'Approach',
    'Move an open gripper toward the grasp pose.',
    'Moves the selected open gripper to the pre-grasp waypoint or down to the grasp point.',
    'tool',
    'manipulation',
    'L2',
    {
      arm,
      position,
      rotation,
      phase: {
        type: 'string',
        description: 'Move above the target or descend to the grasp point.',
        default: 'above',
        options: ['above', 'descend'],
      },
    },
    'ToolResult',
    [],
    [],
    manipulationSource,
    { arm: 'right', position: [420, -120, 280], rotation: [180, 0, 90], phase: 'above' },
  ),
  product(
    'grasp',
    'Grasp',
    'Close the selected gripper around an object.',
    'Closes one arm’s gripper and optionally carries an object id for execution tracking.',
    'tool',
    'manipulation',
    'L2',
    {
      arm,
      object_id: { type: 'string', description: 'Optional object_<id> for tracking.' },
      object_category: { type: 'string', description: 'Object category.', default: 'cup' },
    },
    'ToolResult',
    [],
    [],
    manipulationSource,
    { arm: 'right', object_id: 'object_0' },
  ),
  product(
    'place',
    'Place',
    'Open the selected gripper to release an object.',
    'Opens one gripper and releases the currently held object.',
    'tool',
    'manipulation',
    'L2',
    {
      arm,
      object_id: { type: 'string', description: 'Optional object_<id> for tracking.' },
      object_category: { type: 'string', description: 'Object category.', default: 'cup' },
    },
    'ToolResult',
    [],
    [],
    manipulationSource,
    { arm: 'right', object_id: 'object_0' },
  ),
  product(
    'transport',
    'Transport',
    'Move the gripper between planned waypoints.',
    'Moves the gripper through lift, transfer, lower, or retreat phases.',
    'tool',
    'manipulation',
    'L2',
    {
      arm,
      position,
      rotation,
      phase: {
        type: 'string',
        description: 'Transport phase.',
        default: 'move',
        options: ['lift', 'move', 'lower', 'retreat'],
      },
    },
    'ToolResult',
    [],
    [],
    manipulationSource,
    { arm: 'right', position: [500, 90, 330], rotation: [180, 0, 90], phase: 'move' },
  ),
  product(
    'reset',
    'Reset',
    'Return one or both arms to a known pose.',
    'Moves the selected arms to a predefined joint-space reset pose.',
    'tool',
    'manipulation',
    'L2',
    {
      arms: {
        type: 'string',
        description: 'Arm or arms to reset.',
        default: 'both',
        options: ['left', 'right', 'both'],
      },
      kind: {
        type: 'string',
        description: 'Named reset pose.',
        default: 'safe',
        options: ['safe', 'first', 'pos', 'fork'],
      },
    },
    'ToolResult',
    [],
    [],
    manipulationSource,
    { arms: 'both', kind: 'safe' },
  ),
]

export const STATIC_SAFETY = new Map(
  PRODUCT_CATALOG.map(item => [item.name, item.safety] as const),
)

/** Validate catalogue invariants used by both tests and the website server. */
export function validateCatalog(
  catalog: readonly SoulProduct[] = PRODUCT_CATALOG,
): readonly string[] {
  const errors: string[] = []
  const names = new Set<string>()
  for (const item of catalog) {
    if (names.has(item.name)) errors.push(`duplicate product: ${item.name}`)
    names.add(item.name)
    if (!item.title || !item.description || !item.source) {
      errors.push(`incomplete product: ${item.name}`)
    }
    for (const dependency of item.dependsOn) {
      if (dependency === item.name) errors.push(`self dependency: ${item.name}`)
    }
  }
  for (const item of catalog) {
    for (const dependency of item.dependsOn) {
      if (!names.has(dependency)) {
        errors.push(`unknown dependency: ${item.name} -> ${dependency}`)
      }
    }
  }
  return errors
}

const validationErrors = validateCatalog()
if (validationErrors.length > 0) {
  throw new Error(`Invalid robot capability catalog:\n${validationErrors.join('\n')}`)
}
