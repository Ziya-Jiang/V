/**
 * Package-owned invariant companion for `@deepseek-ai/dsh-tool-robot`.
 * @module @deepseek-ai/dsh-tool-robot/invariant
 */
import type { Context } from '@deepseek-ai/cordis';
/** Cordis companion plugin name. */
export declare const name = "tool-robot-invariant";
/** Service required before the companion can reserve package ownership. */
export declare const inject: string[];
/** Register this package's invariant companion. */
export declare const apply: (ctx: Context) => Promise<() => void>;
//# sourceMappingURL=invariant.d.ts.map