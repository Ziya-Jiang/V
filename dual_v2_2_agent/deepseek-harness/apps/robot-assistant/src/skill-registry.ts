import type { SkillDefinition } from './contracts.ts'

/** Disposable registry that makes robot skills independently replaceable. */
export class SkillRegistry {
  readonly #skills = new Map<string, SkillDefinition>()

  /** Register one unique skill and return its disposer. */
  register(skill: SkillDefinition): () => void {
    if (this.#skills.has(skill.id)) {
      throw new Error(`Duplicate skill id: ${skill.id}`)
    }
    this.#skills.set(skill.id, skill)
    return () => {
      this.#skills.delete(skill.id)
    }
  }

  /** Resolve a skill or fail before any robot command is issued. */
  get(skillId: string): SkillDefinition {
    const skill = this.#skills.get(skillId)
    if (skill === undefined) {
      throw new Error(`Unknown skill: ${skillId}`)
    }
    return skill
  }

  /** Return stable skill metadata for the graphical interface. */
  list(): readonly Omit<SkillDefinition, 'execute'>[] {
    return [...this.#skills.values()]
      .sort((left, right) => left.id.localeCompare(right.id))
      .map(({ execute: _execute, ...metadata }) => metadata)
  }
}
