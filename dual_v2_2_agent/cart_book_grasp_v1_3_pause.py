"""Cooperative operator interruption; keyboard thread only sets an event.

All target replacement, opening and resetting occur on the execution thread.
This is a planner-mediated stop, not a hardware emergency stop.
"""
from contextlib import contextmanager
import time


class PauseRequested(KeyboardInterrupt):
    pass


def interruptible_sleep(robot, seconds):
    sleeper = getattr(robot, 'operator_sleep', time.sleep)
    sleeper(seconds)


@contextmanager
def operator_robot(agent, loop):
    original = agent.RobotArm

    class OperatorRobot(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            loop.robot = self

        def operator_checkpoint(self):
            if loop.armed and not loop.recovering and loop.pause.is_set():
                raise PauseRequested()

        def operator_sleep(self, seconds):
            end = time.monotonic() + seconds
            while True:
                self.operator_checkpoint()
                remaining = end-time.monotonic()
                if remaining <= 0:
                    return
                time.sleep(min(.05, remaining))

        def get_current_state(self):
            self.operator_checkpoint()
            return super().get_current_state()

        def master_control(self, *args, **kwargs):
            self.operator_checkpoint()
            return super().master_control(*args, **kwargs)

        def control_joint_position(self, *args, **kwargs):
            self.operator_checkpoint()
            return super().control_joint_position(*args, **kwargs)

        def _wait_for_robot_stop(self, *args, **kwargs):
            self.operator_checkpoint()
            return super()._wait_for_robot_stop(*args, **kwargs)

    agent.RobotArm = OperatorRobot
    try:
        yield
    finally:
        agent.RobotArm = original


def open_and_reset(robot, reload_ready):
    # A forced, fresh measured-joint goal replaces the previous planner target.
    # Keep the gripper command until that replacement is confirmed complete.
    if not robot.control_joint_position(left_pos=False, right_pos=False,
                                        action='book_operator_hold', force=True):
        print('p 中断：实测关节保持未确认，取消松爪和复位；等待人工检查。', flush=True)
        return False
    if robot.master_control(right_gripper=0, action='book_operator_open', object_category='book') is not True:
        print('p 中断：打开右夹爪命令失败，取消复位。', flush=True)
        return False
    interruptible_sleep(robot, 1.5)
    reload_ready(robot)
    success = robot.control_joint_position(left_pos=False, right_pos=True,
                                          action='first_reset', force=True)
    print('p 中断：右夹爪已发送打开命令，右臂 first_reset ' + ('完成。' if success else '未确认完成。'), flush=True)
    return bool(success)
