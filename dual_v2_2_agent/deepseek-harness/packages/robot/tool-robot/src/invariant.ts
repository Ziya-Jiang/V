/**
 * Package-owned invariant companion for `@deepseek-ai/dsh-tool-robot`.
 * @module @deepseek-ai/dsh-tool-robot/invariant
 */

import type { Context } from '@deepseek-ai/cordis'
import type { InvariantInstaller } from '@deepseek-ai/dsh-invariants'

const PACKAGE_NAME = '@deepseek-ai/dsh-tool-robot'

/** Cordis companion plugin name. */
export const name = 'tool-robot-invariant'
/** Service required before the companion can reserve package ownership. */
export const inject = ['invariants']

/**
 * No runtime invariant: the registered surface is statically limited to
 * status, manifest, and plan; ToolRuntime owns registration lifecycle.
 */
const install: InvariantInstaller = () => {}

/** Register this package's invariant companion. */
export const apply = (ctx: Context): Promise<() => void> =>
  Promise.resolve(ctx.invariants.register(PACKAGE_NAME, install))
