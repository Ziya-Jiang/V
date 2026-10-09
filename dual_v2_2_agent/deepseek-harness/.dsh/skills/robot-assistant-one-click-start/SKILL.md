---
name: robot-assistant-one-click-start
description: Safely check, start, inspect, restart, or stop the complete Dual V2.2 Robot Assistant hardware stack and expose its GUI.
whenToUse: Use when the user asks for one-click robot startup, full Agent startup, GUI startup, robot status, restart, or safe shutdown.
disable-model-invocation: false
user-invocable: true
metadata:
  product: robot-assistant
  hardware: dual-v2-2
---

# Robot Assistant one-click startup

Manage the complete stack only through:

```bash
cd /mnt/ssd/user/V/dual_v2_2_agent
./launch_robot_assistant.sh <command>
```

Valid commands are `check`, `start`, `status`, `stop`, and `restart`.

## Route the request

- Startup, power-on, or open GUI: follow **Start**.
- Status: run `status` only.
- Safe shutdown: run `stop`, then verify with `status`.
- Restart: follow the same hardware confirmation as Start, then use `restart`.

## Start

1. Run `./launch_robot_assistant.sh status`.
2. If all six components are already `RUNNING`, do not start a duplicate. Return
   `http://127.0.0.1:3090`.
3. Before real hardware startup, require the user to explicitly confirm all three:
   - no person or obstacle is inside the robot workspace;
   - the physical emergency stop is immediately reachable;
   - no independent control terminal or other robot Agent owns the hardware.
4. After confirmation, execute:

```bash
ROBOT_ASSISTANT_HARDWARE_CONFIRM=ENABLE_DUAL_V2_2_HARDWARE \
  ./launch_robot_assistant.sh check

ROBOT_ASSISTANT_HARDWARE_CONFIRM=ENABLE_DUAL_V2_2_HARDWARE \
  ./launch_robot_assistant.sh start
```

5. Run `./launch_robot_assistant.sh status`. Startup succeeds only when
   `supervisor`, `python_agent`, `gui`, `rt_control`, `upper_software`, and
   `upper_planner` are all `RUNNING`.
6. Return the GUI URL and the Agent, GUI, and supervisor log paths.

## Fail closed

If any command reports `ERROR`, `[FAULT]`, `Traceback`, `Aborted`, a component
exit, or a failed health check, immediately run:

```bash
./launch_robot_assistant.sh stop
./launch_robot_assistant.sh status
```

Keep the stack stopped unless every component is confirmed healthy. Retry the
same RT/CAN startup failure at most once; after a second failure, report the
first root cause and do not retry.

## Safety boundary

Startup only initializes Control, the persistent Python Agent, the GUI, and the
safety supervisor. It must not submit, confirm, or execute a grasping task.
Robot motion still requires separate Plan review and the explicit GUI
confirmation `ENABLE_DUAL_V2_2_HARDWARE`. Cooperative cancellation is not a
physical emergency stop.
