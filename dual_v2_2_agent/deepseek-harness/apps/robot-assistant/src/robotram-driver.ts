import type {
  AtomicOperation,
  OperationResult,
  RobotDriver,
  SkillInput,
} from './contracts.ts'

const HARDWARE_CONFIRMATION = 'ENABLE_DUAL_V2_2_HARDWARE'

/** Configuration required before the RobotRAM bridge provider can load. */
export interface RobotRamDriverConfig {
  readonly endpoint: URL
  readonly confirmation: string
  readonly bridgeToken: string
  readonly timeoutMs: number
}

/** RobotRAM provider using a narrow, local atomic-operation HTTP bridge. */
export class RobotRamBridgeDriver implements RobotDriver {
  readonly #endpoint: URL
  readonly #bridgeToken: string
  readonly #timeoutMs: number

  constructor(config: RobotRamDriverConfig) {
    if (config.confirmation !== HARDWARE_CONFIRMATION) {
      throw new Error(
        `RobotRAM hardware mode requires ROBOT_ASSISTANT_HARDWARE_CONFIRM=${HARDWARE_CONFIRMATION}`,
      )
    }
    if (config.endpoint.protocol !== 'http:'
      || !['127.0.0.1', 'localhost'].includes(config.endpoint.hostname)) {
      throw new Error('RobotRAM bridge must use loopback HTTP.')
    }
    if (!Number.isInteger(config.timeoutMs) || config.timeoutMs < 1_000) {
      throw new Error('RobotRAM operation timeout must be at least 1000 ms.')
    }
    if (config.bridgeToken.length < 24) {
      throw new Error('ROBOTRAM_BRIDGE_TOKEN must contain at least 24 characters.')
    }
    this.#endpoint = new URL('/v1/operations', config.endpoint)
    this.#bridgeToken = config.bridgeToken
    this.#timeoutMs = config.timeoutMs
  }

  /** Send one validated atomic operation to the separately supervised bridge. */
  async run(
    operation: AtomicOperation,
    input: SkillInput,
    signal: AbortSignal,
  ): Promise<OperationResult> {
    const timeoutSignal = AbortSignal.timeout(this.#timeoutMs)
    const response = await fetch(this.#endpoint, {
      method: 'POST',
      headers: {
        'content-type': 'application/json',
        'x-robot-bridge-token': this.#bridgeToken,
      },
      body: JSON.stringify({ operation, input }),
      signal: AbortSignal.any([signal, timeoutSignal]),
    })
    const body: unknown = await response.json()
    if (typeof body !== 'object' || body === null
      || !('ok' in body) || typeof body.ok !== 'boolean'
      || !('detail' in body) || typeof body.detail !== 'string') {
      throw new Error('RobotRAM bridge returned an invalid operation result.')
    }
    if (!response.ok) {
      throw new Error(
        `RobotRAM bridge rejected ${operation}: HTTP ${response.status}: ${body.detail}`,
      )
    }
    return { ok: body.ok, detail: body.detail }
  }
}
