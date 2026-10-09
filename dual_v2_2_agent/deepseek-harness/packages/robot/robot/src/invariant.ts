/**
 * Package-owned invariant companion for `@deepseek-ai/dsh-robot`.
 * @module @deepseek-ai/dsh-robot/invariant
 */

import type { Context } from '@deepseek-ai/cordis'
import type { InvariantInstaller } from '@deepseek-ai/dsh-invariants'

const PACKAGE_NAME = '@deepseek-ai/dsh-robot'

/** Cordis companion plugin name. */
export const name = 'robot-invariant'
/** Service required before the companion can reserve package ownership. */
export const inject = ['invariants']

/**
 * No runtime invariant: Cordis owns the single-provider service relation and
 * the abstract seam publishes no independent mutable registry.
 */
const install: InvariantInstaller = () => {}

/** Register this package's invariant companion. */
export const apply = (ctx: Context): Promise<() => void> =>
  Promise.resolve(ctx.invariants.register(PACKAGE_NAME, install))
