/**
 * Loopback NDJSON Service Provider for the plan-only robot seam.
 * It accepts only Python Agents that advertise process-level plan-only mode;
 * no execute, task, cancel, legacy call, or arbitrary RPC method exists on
 * this provider's public or private API.
 * @module @deepseek-ai/dsh-robot-python-agent
 */
import { createConnection, isIP } from 'node:net';
import { isAbsolute } from 'node:path';
import z from '@deepseek-ai/schemastery';
import { RobotError, RobotRuntime, } from '@deepseek-ai/dsh-robot';
const PROVIDER_ID = 'python-agent';
const DEFAULT_MAX_RESPONSE_BYTES = 1024 * 1024;
const DEFAULT_MAX_INSTRUCTION_BYTES = 16 * 1024;
const PLAN_ONLY_METHODS = ['health', 'manifest', 'plan'];
/** Schemastery configuration for the Python Agent provider. */
export const Config = z.object({
    host: z.string().default('127.0.0.1'),
    port: z.number().default(15_555),
    token: z.string(),
    timeoutMs: z.number().default(30_000),
    maxResponseBytes: z.number().default(DEFAULT_MAX_RESPONSE_BYTES),
    maxInstructionBytes: z.number().default(DEFAULT_MAX_INSTRUCTION_BYTES),
    outputDir: z.string().default(process.cwd()),
});
function isLoopbackAddress(host) {
    const version = isIP(host);
    if (version === 4)
        return host.split('.')[0] === '127';
    return version === 6 && host === '::1';
}
function positiveInteger(name, value) {
    if (!Number.isSafeInteger(value) || value < 1) {
        throw new RobotError('ROBOT_CONFIGURATION', `robot-python-agent: ${name} must be a positive safe integer`);
    }
    return value;
}
function resolveConfig(config) {
    const host = config.host ?? '127.0.0.1';
    if (!isLoopbackAddress(host)) {
        throw new RobotError('ROBOT_CONFIGURATION', 'robot-python-agent: host must be a numeric loopback address (127.0.0.0/8 or ::1)');
    }
    const token = config.token;
    if (token !== undefined && token.length === 0) {
        throw new RobotError('ROBOT_CONFIGURATION', 'robot-python-agent: token must be non-empty when configured');
    }
    const outputDir = config.outputDir ?? process.cwd();
    if (!isAbsolute(outputDir)) {
        throw new RobotError('ROBOT_CONFIGURATION', 'robot-python-agent: outputDir must be an absolute path');
    }
    const port = positiveInteger('port', config.port ?? 15_555);
    if (port > 65_535) {
        throw new RobotError('ROBOT_CONFIGURATION', 'robot-python-agent: port must be at most 65535');
    }
    return {
        host,
        port,
        timeoutMs: positiveInteger('timeoutMs', config.timeoutMs ?? 30_000),
        maxResponseBytes: positiveInteger('maxResponseBytes', config.maxResponseBytes ?? DEFAULT_MAX_RESPONSE_BYTES),
        maxInstructionBytes: positiveInteger('maxInstructionBytes', config.maxInstructionBytes ?? DEFAULT_MAX_INSTRUCTION_BYTES),
        outputDir,
        ...token === undefined ? {} : { token },
    };
}
function isObject(value) {
    return typeof value === 'object' && value !== null && !Array.isArray(value);
}
function requireObject(value, method) {
    if (!isObject(value)) {
        throw new RobotError('ROBOT_PROTOCOL', `Python Agent ${method} response data must be an object`);
    }
    return value;
}
function requireString(value, field, method, maxLength = 4096) {
    if (typeof value !== 'string'
        || value.length === 0
        || value.length > maxLength) {
        throw new RobotError('ROBOT_PROTOCOL', `Python Agent ${method} response has an invalid ${field}`);
    }
    return value;
}
function planOnlyReason(health) {
    if (health.planOnly !== true) {
        return 'Python Agent must be started with ROBOT_AGENT_PLAN_ONLY=1';
    }
    const allowed = health.allowedMethods;
    if (!Array.isArray(allowed)
        || allowed.length !== PLAN_ONLY_METHODS.length
        || !PLAN_ONLY_METHODS.every(method => allowed.includes(method))) {
        return 'Python Agent did not advertise the exact plan-only method set';
    }
    if (health.eagerBootstrap !== false) {
        return 'Python Agent plan-only mode must disable eager hardware bootstrap';
    }
    if (health.bootstrapState !== 'not_started') {
        const state = health.bootstrapState;
        const renderedState = typeof state === 'string'
            ? state
            : state === undefined
                ? 'missing'
                : JSON.stringify(state);
        return `Python Agent hardware bootstrap state is ${renderedState}`;
    }
    if (health.activeTaskId !== null) {
        return 'Python Agent reports an active or unverifiable hardware task';
    }
    return undefined;
}
/** Plan-only RobotRuntime implementation over the existing Python Agent. */
export class PythonAgentRobotRuntime extends RobotRuntime {
    static Config = Config;
    #config;
    constructor(ctx, config = {}) {
        super(ctx);
        this.#config = resolveConfig(config);
    }
    async status(signal) {
        const health = requireObject(await this.#request('health', {}, signal), 'health');
        const reason = planOnlyReason(health);
        return {
            mode: 'dry-run',
            provider: PROVIDER_ID,
            planOnlyReady: reason === undefined,
            ...reason === undefined ? {} : { reason },
            health,
        };
    }
    async manifest(signal) {
        await this.#requirePlanOnly(signal);
        const data = requireObject(await this.#request('manifest', {}, signal), 'manifest');
        if (!Array.isArray(data.tools) || !Array.isArray(data.skills)) {
            throw new RobotError('ROBOT_PROTOCOL', 'Python Agent manifest response must contain tools and skills arrays');
        }
        return {
            mode: 'dry-run',
            provider: PROVIDER_ID,
            tools: data.tools,
            skills: data.skills,
        };
    }
    async plan(request, signal) {
        const instruction = request.instruction.trim();
        if (instruction.length === 0) {
            throw new RobotError('ROBOT_CONFIGURATION', 'robot plan instruction must be non-empty');
        }
        if (Buffer.byteLength(instruction, 'utf8') > this.#config.maxInstructionBytes) {
            throw new RobotError('ROBOT_CONFIGURATION', `robot plan instruction exceeds ${this.#config.maxInstructionBytes} UTF-8 bytes`);
        }
        await this.#requirePlanOnly(signal);
        const data = requireObject(await this.#request('plan', {
            instruction,
            output_dir: this.#config.outputDir,
        }, signal), 'plan');
        const planId = requireString(data.planId, 'planId', 'plan', 256);
        const echoedInstruction = requireString(data.instruction, 'instruction', 'plan', this.#config.maxInstructionBytes);
        if (echoedInstruction !== instruction) {
            throw new RobotError('ROBOT_PROTOCOL', 'Python Agent plan response instruction does not match the request');
        }
        if (data.output_dir !== this.#config.outputDir) {
            throw new RobotError('ROBOT_PROTOCOL', 'Python Agent plan response output_dir does not match provider configuration');
        }
        if (!isObject(data.plan)) {
            throw new RobotError('ROBOT_PROTOCOL', 'Python Agent plan response must contain a plan object');
        }
        const createdAt = requireString(data.createdAt, 'createdAt', 'plan');
        return {
            mode: 'dry-run',
            provider: PROVIDER_ID,
            planId,
            instruction,
            outputDir: this.#config.outputDir,
            createdAt,
            plan: data.plan,
        };
    }
    async #requirePlanOnly(signal) {
        const status = await this.status(signal);
        if (!status.planOnlyReady) {
            throw new RobotError('ROBOT_PLAN_ONLY_REQUIRED', status.reason ?? 'Python Agent plan-only readiness could not be verified');
        }
    }
    async #request(method, args, signal) {
        if (signal?.aborted === true) {
            throw new RobotError('ROBOT_ABORTED', `Python Agent ${method} request was aborted before connection`);
        }
        const request = {
            method,
            args,
            ...this.#config.token === undefined ? {} : { token: this.#config.token },
        };
        const payload = `${JSON.stringify(request)}\n`;
        const response = await this.#exchange(method, payload, signal);
        let parsed;
        try {
            parsed = JSON.parse(response);
        }
        catch (error) {
            throw new RobotError('ROBOT_PROTOCOL', `Python Agent ${method} returned invalid JSON`, { cause: error });
        }
        const envelope = requireObject(parsed, method);
        if (typeof envelope.ok !== 'boolean') {
            throw new RobotError('ROBOT_PROTOCOL', `Python Agent ${method} response must contain a boolean ok field`);
        }
        if (!envelope.ok) {
            const detail = typeof envelope.error === 'string'
                ? envelope.error
                : typeof envelope.message === 'string'
                    ? envelope.message
                    : 'request was rejected';
            throw new RobotError('ROBOT_REMOTE', `Python Agent ${method} failed: ${detail}`);
        }
        return envelope.data ?? null;
    }
    #exchange(method, payload, signal) {
        return new Promise((resolve, reject) => {
            let settled = false;
            let received = Buffer.alloc(0);
            const socket = createConnection({
                host: this.#config.host,
                port: this.#config.port,
            });
            const cleanup = () => {
                clearTimeout(timeout);
                signal?.removeEventListener('abort', abort);
                socket.removeAllListeners();
                socket.destroy();
            };
            const fail = (error) => {
                if (settled)
                    return;
                settled = true;
                cleanup();
                reject(error);
            };
            const succeed = (value) => {
                if (settled)
                    return;
                settled = true;
                cleanup();
                resolve(value);
            };
            const abort = () => {
                fail(new RobotError('ROBOT_ABORTED', `Python Agent ${method} request was aborted`));
            };
            const timeout = setTimeout(() => {
                fail(new RobotError('ROBOT_TIMEOUT', `Python Agent ${method} timed out after ${this.#config.timeoutMs} ms`));
            }, this.#config.timeoutMs);
            signal?.addEventListener('abort', abort, { once: true });
            socket.once('connect', () => socket.end(payload));
            socket.on('data', (chunk) => {
                received = Buffer.concat([received, chunk]);
                if (received.length > this.#config.maxResponseBytes + 1) {
                    fail(new RobotError('ROBOT_RESPONSE_TOO_LARGE', `Python Agent ${method} response exceeds ${this.#config.maxResponseBytes} bytes`));
                    return;
                }
                const newline = received.indexOf(0x0A);
                if (newline < 0)
                    return;
                if (received.subarray(newline + 1).toString('utf8').trim().length > 0) {
                    fail(new RobotError('ROBOT_PROTOCOL', `Python Agent ${method} returned more than one NDJSON response`));
                    return;
                }
                try {
                    const decoder = new TextDecoder('utf-8', { fatal: true });
                    succeed(decoder.decode(received.subarray(0, newline)));
                }
                catch (error) {
                    fail(new RobotError('ROBOT_PROTOCOL', `Python Agent ${method} returned invalid UTF-8`, { cause: error }));
                }
            });
            socket.once('end', () => {
                fail(new RobotError('ROBOT_PROTOCOL', `Python Agent ${method} closed before a complete NDJSON response`));
            });
            socket.once('error', (error) => {
                fail(new RobotError('ROBOT_CONNECTION', `Could not reach Python Agent at ${this.#config.host}:${this.#config.port}: ${error.code ?? error.message}`, { cause: error }));
            });
        });
    }
}
export default PythonAgentRobotRuntime;
//# sourceMappingURL=index.js.map