import { isIP } from 'node:net'

import { hardwareAccessPolicy } from './live.ts'
import { harnessWebOrigin } from './ops.ts'
import { createRobotSoulServer } from './server.ts'

function loopbackHost(value: string | undefined): string {
  const host = value ?? '127.0.0.1'
  const version = isIP(host)
  const allowed = version === 4 ? host.startsWith('127.') : version === 6 && host === '::1'
  if (!allowed) throw new Error('ROBOT_SOUL_HOST must be a numeric loopback address.')
  return host
}

function portNumber(value: string | undefined): number {
  if (value === undefined) return 3091
  const port = Number(value)
  if (!Number.isSafeInteger(port) || port < 1 || port > 65_535) {
    throw new Error('ROBOT_SOUL_PORT must be an integer from 1 to 65535.')
  }
  return port
}

const host = loopbackHost(process.env.ROBOT_SOUL_HOST)
const port = portNumber(process.env.ROBOT_SOUL_PORT)
const policy = hardwareAccessPolicy()
const server = createRobotSoulServer()

server.listen(port, host, () => {
  console.log(`HKCLR Harness Agent: http://${host}:${port}/#control`)
  console.log(`Harness orchestration service: ${harnessWebOrigin()} (not embedded)`)
  console.log(`Hardware access stage: ${policy.stage}`)
  console.log(
    policy.executeEnabled
      ? 'Execution boundary: approval-gated execution enabled; only high-level Skills are allowed and call remains blocked'
      : 'Execution boundary: hardware execution locked; simulation and read-only Agent status remain available',
  )
})

let shutdownStarted = false
for (const signal of ['SIGINT', 'SIGTERM'] as const) {
  process.once(signal, () => {
    if (shutdownStarted) return
    shutdownStarted = true

    const forceCloseTimer = setTimeout(() => {
      server.closeAllConnections()
    }, 2_000)
    forceCloseTimer.unref()
    const hardExitTimer = setTimeout(() => {
      console.error('HKCLR Harness Agent did not shut down cleanly within 5 seconds.')
      process.exit(1)
    }, 5_000)
    hardExitTimer.unref()

    server.closeIdleConnections()
    server.close(() => {
      clearTimeout(forceCloseTimer)
      clearTimeout(hardExitTimer)
      process.exit(0)
    })
  })
}
