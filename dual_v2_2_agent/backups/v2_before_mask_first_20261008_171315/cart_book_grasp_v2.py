"""GPT multi-book grasp: type a request, select from the live image, grasp once.

    python cart_book_grasp_v2.py --assume-head-camera-calibrated

Base-X approach with conditional depth-recovery Z compensation, p reset and Enter retry.
Does not move hardware when imported. Failed cycles wait for Enter to retry.
"""
from contextlib import contextmanager
import os
from pathlib import Path
import select
import sys
import termios
import threading
import tty
import time
import codecs
import unicodedata
from cart_book_grasp_v2_pause import PauseRequested, operator_robot, open_and_reset


class AutoCartLoop:
    def __init__(self, delay_seconds=5., stop_event=None):
        self.stop = stop_event if stop_event is not None else threading.Event()
        self.delay_seconds = delay_seconds
        self.pause = threading.Event()
        self.robot = None
        self.armed = False
        self.recovering = False
        self.cycles = 0
        self.last_success = None
        self.waiting_for_retry = threading.Event()
        self._retry_condition = threading.Condition()
        self._retry_requested = False

    def request_pause(self):
        if not self.recovering:
            self.pause.set()
            print('\n收到 p：中断本轮，保持后打开右夹爪并回 first_reset；随后等待回车。', flush=True)

    def handle_interrupt(self, error):
        if not isinstance(error, PauseRequested):
            return False
        self.recovering = True
        self.last_success = False
        self._retry_requested = False
        try:
            if self.robot is None:
                print('机器人尚未初始化，已取消本轮。', flush=True)
            else:
                from cart_book_grasp_v2_motion import _reload_initial_right_j6
                open_and_reset(self.robot, _reload_initial_right_j6)
        except Exception as error:
            print(f'p 中断恢复失败：{error}；保持暂停，等待人工检查。', flush=True)
        finally:
            self.pause.clear()
            self.recovering = False
        return True

    def check_pause(self):
        if self.pause.is_set():
            raise PauseRequested()

    def request_stop(self):
        if not self.stop.is_set():
            self.stop.set()
            print('\n收到 s：当前轮次结束后退出，不再启动下一轮。', flush=True)

    def record_result(self, result):
        self.last_success = bool(result[1])

    def request_retry(self):
        # Discard Enter during motion/success delay; never queue a future retry.
        with self._retry_condition:
            if self.waiting_for_retry.is_set():
                self._retry_requested = True
                self._retry_condition.notify_all()

    def record_failure(self, error):
        self.last_success = False
        print(f'本轮发生异常：{type(error).__name__}: {error}', flush=True)

    def wait_for_retry(self):
        with self._retry_condition:
            self._retry_requested = False
            print('本轮感知或执行未完成，已暂停；保持夹爪当前指令，不自动松手或复位。\n'
                  '请检查现场，按回车从新一轮开始（会先执行 first_reset，再重新感知）；按 s 退出。', flush=True)
            self.waiting_for_retry.set()
            try:
                while not self.stop.is_set() and not self._retry_requested:
                    self.check_pause()
                    self._retry_condition.wait(timeout=.1)
                self.check_pause()
                return not self.stop.is_set()
            finally:
                self.waiting_for_retry.clear()
                self._retry_requested = False

    def next_task(self):
        self.armed = True
        self.check_pause()
        if self.stop.is_set():
            return None
        if self.cycles or self.last_success is False:
            if self.last_success is not True:
                if not self.wait_for_retry():
                    return None
            else:
                print(f'第 {self.cycles} 轮完成；等待 {self.delay_seconds:g} 秒。p 复位暂停，s 停止。', flush=True)
                end = time.monotonic()+self.delay_seconds
                while time.monotonic() < end:
                    self.check_pause()
                    if self.stop.wait(min(.05, max(0., end-time.monotonic()))):
                        return None
        self.check_pause()
        if self.stop.is_set():
            return None
        self.cycles += 1
        self.last_success = None
        print(f'开始第 {self.cycles} 轮 cart+。p 中断并复位，s 停止后续循环。', flush=True)
        return 'cart+'


class InstructionCartLoop(AutoCartLoop):
    """Exactly one fresh perception/grasp per submitted instruction."""
    def __init__(self, selector):
        super().__init__(delay_seconds=0.)
        self.selector=selector
        self.accepting_instruction=False
        self.input_buffer=''
        self.pending_instruction=None
        self.previous_instruction=None

    def feed_character(self, character):
        with self._retry_condition:
            if self.accepting_instruction:
                if character in ('\r','\n'):
                    line=self.input_buffer.strip();self.input_buffer=''
                    print('',flush=True)
                    if line.lower() in ('s','quit','exit'):
                        self.request_stop()
                    elif line.lower()=='p':
                        self.request_pause()
                    else:
                        self.pending_instruction=line
                    self.accepting_instruction=False
                    self._retry_condition.notify_all()
                elif character in ('\x7f','\b'):
                    if self.input_buffer:
                        last=self.input_buffer[-1];self.input_buffer=self.input_buffer[:-1]
                        sys.stdout.write('\b \b'*(2 if unicodedata.east_asian_width(last) in ('W','F') else 1));sys.stdout.flush()
                elif character=='\x04':
                    self.request_stop()
                elif character.isprintable() and len(self.input_buffer)<500:
                    self.input_buffer+=character;sys.stdout.write(character);sys.stdout.flush()
            else:
                if character.lower()=='p':self.request_pause()
                elif character.lower()=='s':self.request_stop()

    def next_task(self):
        self.armed=True
        while not self.stop.is_set():
            self.check_pause()
            retry_allowed=self.cycles>0 and self.last_success is not True
            with self._retry_condition:
                self.pending_instruction=None;self.input_buffer=''
                print('\n输入选书指令（如“给我中间的书”“给我黄色的书”）；左右以机器人视角为准。',flush=True)
                if retry_allowed:print('上一轮未完成：可输入更明确的描述，或直接回车重试上一条。',flush=True)
                print('输入 s 回车退出；动作中 p 中断并松爪复位，s 停止后续任务。\n> ',end='',flush=True)
                self.accepting_instruction=True
                try:
                    while self.pending_instruction is None and not self.stop.is_set():
                        self.check_pause();self._retry_condition.wait(timeout=.05)
                    self.check_pause()
                    if self.stop.is_set():return None
                    instruction=self.pending_instruction
                finally:
                    self.accepting_instruction=False
            if not instruction:
                if retry_allowed and self.previous_instruction:
                    instruction=self.previous_instruction
                else:
                    print('请输入新的选书指令；不会自动重复已完成的抓取。',flush=True)
                    continue
            self.selector.set_instruction(instruction)
            self.previous_instruction=instruction
            self.cycles+=1;self.last_success=None
            print(f'第 {self.cycles} 轮 GPT 选书：{instruction}',flush=True)
            return 'cart+'
        return None


@contextmanager
def stop_key_listener(loop, stream=None):
    """One terminal reader: UTF-8 text at prompts; p/s keys during actions."""
    stream=sys.stdin if stream is None else stream
    if not stream.isatty():raise RuntimeError('请在交互终端运行 v2，以输入指令及使用 p/s。')
    fd=stream.fileno();original=termios.tcgetattr(fd);finished=threading.Event()
    decoder=codecs.getincrementaldecoder('utf-8')(errors='replace')
    def listen():
        try:
            while not finished.is_set() and not loop.stop.is_set():
                ready,_,_=select.select([fd],[],[],.1)
                if not ready:continue
                data=os.read(fd,256)
                if not data:loop.request_stop();return
                # Do not interpret pasted command remainder as motion hotkeys.
                was_prompt=loop.accepting_instruction
                for character in decoder.decode(data):
                    if was_prompt and not loop.accepting_instruction:break
                    loop.feed_character(character)
        except (OSError,ValueError):
            loop.request_stop()
            print('终端输入监听失效，停止后续任务。',flush=True)
    thread=threading.Thread(target=listen,name='book-language-input',daemon=True)
    try:
        tty.setcbreak(fd);thread.start();yield
    finally:
        finished.set()
        if thread.ident is not None:thread.join(timeout=.5)
        termios.tcsetattr(fd,termios.TCSADRAIN,original)


@contextmanager
def automatic_cart_mode(agent, loop):
    provider = agent.TASK_TYPE_PROVIDER
    failure_handler = getattr(agent, 'TASK_FAILURE_HANDLER', None)
    keyboard_handler = getattr(agent, 'TASK_KEYBOARD_INTERRUPT_HANDLER', None)
    execute = agent.execute_main

    def tracked_execute(*args, **kwargs):
        result = execute(*args, **kwargs)
        loop.record_result(result)
        if result[0] == getattr(agent, 'OPERATOR_RESET_REQUESTED', object()):
            loop.last_success = False
        return result

    agent.TASK_TYPE_PROVIDER = loop.next_task
    agent.TASK_FAILURE_HANDLER = loop.record_failure
    agent.TASK_KEYBOARD_INTERRUPT_HANDLER = loop.handle_interrupt
    agent.execute_main = tracked_execute
    try:
        yield
    finally:
        agent.TASK_TYPE_PROVIDER = provider
        agent.TASK_FAILURE_HANDLER = failure_handler
        agent.TASK_KEYBOARD_INTERRUPT_HANDLER = keyboard_handler
        agent.execute_main = execute


def main(*, book_entry=None, config_path=None, wrist_config_path=None):
    from cart_book_grasp_v2_agent import BookSelectionAgent
    selector=BookSelectionAgent()
    selector.check_ready()  # Missing key/config fails before robot initialization.
    loop=InstructionCartLoop(selector)
    selector.checkpoint=loop.check_pause
    # Set up a usable stop key before any robot or camera initialization.
    with stop_key_listener(loop):
        import dual_v2_2_agent_pcl as agent
        import cart_book_grasp_v2_perception as book
        run_book = book.main if book_entry is None else book_entry

        if loop.stop.is_set():
            return
        print('GPT 多书抓取：每条文字指令选择一本，完成后等待下一条；不自动连续抓取。',flush=True)
        import cart_book_grasp_v2_motion as motion
        saved_selector=motion._SELECTION_AGENT
        motion._SELECTION_AGENT=selector
        try:
            with operator_robot(agent, loop), automatic_cart_mode(agent, loop):
                run_book(task_types=['cart+'],
                    config_path=config_path or Path(__file__).with_name('cart_book_grasp_v2_config.json'),
                    wrist_config_path=wrist_config_path or Path(__file__).with_name('cart_book_grasp_v2_wrist_config.json'))
        finally:
            motion._SELECTION_AGENT=saved_selector
            agent.cleanup_runtime_resources()


def check_gpt_connection():
    """Test credentials/model without importing the robot or opening a camera."""
    from cart_book_grasp_v2_agent import load_config
    from cart_book_grasp_v2_gpt import GPTBookClient, GPTSelectionError
    config=load_config(Path(__file__).with_name('cart_book_grasp_v2_agent_config.json'))
    payload=dict(model=config['model'],store=False,max_output_tokens=100,
        input='Return ready=true to verify this API connection. No robot action is requested.',
        text={'format':{'type':'json_schema','name':'connection_check','strict':True,'schema':{
            'type':'object','properties':{'ready':{'type':'boolean'}},'required':['ready'],'additionalProperties':False}}})
    result=GPTBookClient().select(payload,config['request_timeout_seconds'])
    if result != {'ready':True}:raise GPTSelectionError('GPT 连通测试未返回 ready=true。')
    print(f"GPT 接口已连通：{config['model']}。未启动相机或机械臂。",flush=True)


if __name__ == '__main__':
    from cart_book_grasp_v2_gpt import GPTSelectionError
    try:
        if sys.argv[1:] == ['--check-gpt']:
            check_gpt_connection()
        else:
            main()
    except GPTSelectionError as error:
        print(str(error),file=sys.stderr)
        raise SystemExit(2)
