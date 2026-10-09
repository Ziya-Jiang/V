/**
 * Package-owned invariant companion for `@deepseek-ai/dsh-robot-python-agent`.
 * @module @deepseek-ai/dsh-robot-python-agent/invariant
 */
const PACKAGE_NAME = '@deepseek-ai/dsh-robot-python-agent';
/** Cordis companion plugin name. */
export const name = 'robot-python-agent-invariant';
/** Service required before the companion can reserve package ownership. */
export const inject = ['invariants'];
/**
 * No runtime invariant: plan-only readiness is revalidated from the remote
 * health response before every manifest or plan request.
 */
const install = () => { };
/** Register this package's invariant companion. */
export const apply = (ctx) => Promise.resolve(ctx.invariants.register(PACKAGE_NAME, install));
//# sourceMappingURL=invariant.js.map