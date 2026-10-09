/**
 * Service Definition for the plan-only robot capability seam (`ctx.robot`).
 * The seam deliberately omits execute/cancel operations: this first integration
 * can inspect capabilities, create server-side plan previews, and observe
 * provider status without exposing a hardware action path.
 * @module @deepseek-ai/dsh-robot
 */
import { Service } from '@deepseek-ai/cordis';
/** Robot capability failure with a routable category. */
export class RobotError extends Error {
    code;
    constructor(code, message, options) {
        super(message, options);
        this.name = 'RobotError';
        this.code = code;
    }
}
/**
 * Plan-only robot service. Concrete providers own transport and process
 * validation; model-facing names and schemas belong to a separate consumer.
 */
export class RobotRuntime extends Service {
    constructor(ctx) {
        super(ctx, 'robot');
    }
    /**
     * Ask the hardware owner to hold a Plan for per-task human approval.
     * This call never contains the human confirmation and cannot move hardware
     * before a separate operator surface approves the exact server-side Plan.
     * @param request - Identifier returned by {@link plan}.
     * @param signal - Optional caller cancellation forwarded to the provider.
     * @returns Pending execution request visible to the operator approval UI.
     */
    requestExecution(_request, _signal) {
        return Promise.reject(new RobotError('ROBOT_EXECUTION_NOT_ENABLED', 'Robot provider does not enable approval-gated execution'));
    }
    /**
     * Read approval and Task state for a model-created execution request.
     * @param request - Request identifier returned by {@link requestExecution}.
     * @param signal - Optional caller cancellation forwarded to the provider.
     * @returns Approval state and Task summary when execution has started.
     */
    executionStatus(_request, _signal) {
        return Promise.reject(new RobotError('ROBOT_EXECUTION_NOT_ENABLED', 'Robot provider does not enable execution request status'));
    }
}
export default RobotRuntime;
//# sourceMappingURL=index.js.map