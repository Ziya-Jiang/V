"""Cart-only repeated book grasp. Press s in this terminal to stop after a cycle.

    python cart_book_grasp_v1.py --assume-head-camera-calibrated

Reuses 1.2 perception, J7 inverse rotation, visualization, release and reset.
Does not move hardware when imported. Failed/incomplete cycles are not retried.
"""
from contextlib import contextmanager
import os
from pathlib import Path
import select
import sys
import termios
import threading
import tty


class AutoCartLoop:
    def __init__(self, delay_seconds=5., stop_event=None):
        self.stop = stop_event if stop_event is not None else threading.Event()
        self.delay_seconds = delay_seconds
        self.cycles = 0
        self.last_success = None

    def request_stop(self):
        if not self.stop.is_set():
            self.stop.set()
            print('\n收到 s：当前轮次结束后退出，不再启动下一轮。', flush=True)

    def record_result(self, result):
        self.last_success = bool(result[1])

    def next_task(self):
        if self.stop.is_set():
            return None
        if self.cycles:
            if self.last_success is not True:
                print('本轮感知或执行未完成，自动循环已停止。请检查现场后重新启动。', flush=True)
                return None
            print(f'第 {self.cycles} 轮完成；等待 {self.delay_seconds:g} 秒。按 s 停止。', flush=True)
            if self.stop.wait(self.delay_seconds):
                return None
        if self.stop.is_set():
            return None
        self.cycles += 1
        self.last_success = None
        print(f'开始第 {self.cycles} 轮 cart+。按 s 可停止后续循环。', flush=True)
        return 'cart+'


@contextmanager
def stop_key_listener(loop, stream=None):
    """Read keys without Enter; restore terminal settings on every exit path."""
    stream = sys.stdin if stream is None else stream
    if not stream.isatty():
        raise RuntimeError('请在交互终端运行，以便按 s 停止自动循环。')
    fd = stream.fileno()
    original = termios.tcgetattr(fd)
    finished = threading.Event()

    def listen():
        try:
            while not finished.is_set() and not loop.stop.is_set():
                ready, _, _ = select.select([fd], [], [], .1)
                if not ready:
                    continue
                data = os.read(fd, 64)
                if not data:
                    loop.stop.set()
                    return
                if b's' in data.lower():
                    loop.request_stop()
                    return
        except (OSError, ValueError):
            loop.stop.set()
            print('停止按键监听失效，取消后续自动循环。', flush=True)

    thread = threading.Thread(target=listen, name='cart-book-stop-key', daemon=True)
    try:
        tty.setcbreak(fd)
        thread.start()
        yield
    finally:
        finished.set()
        if thread.ident is not None:
            thread.join(timeout=.5)
        termios.tcsetattr(fd, termios.TCSADRAIN, original)


@contextmanager
def automatic_cart_mode(agent, loop):
    provider = agent.TASK_TYPE_PROVIDER
    execute = agent.execute_main

    def tracked_execute(*args, **kwargs):
        result = execute(*args, **kwargs)
        loop.record_result(result)
        if result[0] == getattr(agent, 'OPERATOR_RESET_REQUESTED', object()):
            loop.last_success = False
        return result

    agent.TASK_TYPE_PROVIDER = loop.next_task
    agent.execute_main = tracked_execute
    try:
        yield
    finally:
        agent.TASK_TYPE_PROVIDER = provider
        agent.execute_main = execute


def main():
    loop = AutoCartLoop(delay_seconds=5.)
    # Set up a usable stop key before any robot or camera initialization.
    with stop_key_listener(loop):
        import dual_v2_2_agent_pcl as agent
        import book_grasp_1_2_local as book

        if loop.stop.is_set():
            return
        print('自动 cart+：每轮完成后等 5 秒再运行；终端按 s（无需回车）停止后续循环。', flush=True)
        try:
            with automatic_cart_mode(agent, loop):
                book.main(task_types=['cart+'],
                    config_path=Path(__file__).with_name('cart_book_grasp_v1_config.json'),
                    wrist_config_path=Path(__file__).with_name('cart_book_grasp_v1_wrist_config.json'))
        finally:
            agent.cleanup_runtime_resources()


if __name__ == '__main__':
    main()
