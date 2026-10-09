import type {
  AtomicOperation,
  Constraint,
  SkillDefinition,
} from '../contracts.ts'

const SHARED_MOTION_CONSTRAINTS: readonly Constraint[] = [
  {
    kind: 'workspace',
    description: 'Target pose must remain inside the calibrated robot workspace.',
  },
  {
    kind: 'collision',
    description: 'Planner preview must report a collision-free path.',
  },
  {
    kind: 'speed',
    description: 'Limit tool speed while people may share the workspace.',
    value: 0.2,
    unit: 'm/s',
  },
]

function atomicSkill(
  id: string,
  title: string,
  description: string,
  operation: AtomicOperation,
  constraints: readonly Constraint[],
): SkillDefinition {
  return {
    id,
    title,
    description,
    operation,
    constraints,
    async execute(context, input) {
      return context.driver.run(operation, input, context.signal)
    },
  }
}

/** First MVP skill set used by the clean-table task planner. */
export const CLEAN_TABLE_SKILLS: readonly SkillDefinition[] = [
  atomicSkill(
    'robot.approach-object',
    'Approach object',
    'Move to a perception-derived pre-grasp pose without closing the gripper.',
    'approach',
    SHARED_MOTION_CONSTRAINTS,
  ),
  atomicSkill(
    'robot.grasp-object',
    'Grasp object',
    'Close the gripper at a validated grasp pose and verify retention.',
    'grasp',
    [
      ...SHARED_MOTION_CONSTRAINTS,
      {
        kind: 'force',
        description: 'Use the object profile force limit and stop on overcurrent.',
        value: 'object-profile',
      },
    ],
  ),
  atomicSkill(
    'robot.place-object',
    'Place object',
    'Transport the retained object to its semantic destination and release it.',
    'place',
    SHARED_MOTION_CONSTRAINTS,
  ),
  atomicSkill(
    'robot.return-home',
    'Return home',
    'Return both arms to the configured safe home state.',
    'home',
    [
      {
        kind: 'collision',
        description: 'Use the validated joint-space recovery trajectory.',
      },
    ],
  ),
]
