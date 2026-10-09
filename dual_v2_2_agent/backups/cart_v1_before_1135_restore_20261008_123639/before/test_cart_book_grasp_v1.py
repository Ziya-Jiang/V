"""Offline auto-loop tests, including a simulated terminal for the s key."""
from contextlib import nullcontext
import ast
from pathlib import Path
import os
import pty
import sys
import termios
import types
import threading
import unittest
from unittest.mock import Mock, patch

import cart_book_grasp_v1 as cart


class AutoCartTests(unittest.TestCase):
    def test_immediate_cart_then_five_second_delay_after_success(self):
        loop = cart.AutoCartLoop()
        with patch.object(loop.stop, 'wait', return_value=False) as wait:
            self.assertEqual(loop.next_task(), 'cart+')
            wait.assert_not_called()
            loop.record_result(('book0', True))
            self.assertEqual(loop.next_task(), 'cart+')
            wait.assert_called_once_with(5.)
            self.assertEqual(loop.cycles, 2)
            self.assertIsNone(loop.last_success)

    def test_s_during_cycle_prevents_next_cycle(self):
        loop = cart.AutoCartLoop()
        self.assertEqual(loop.next_task(), 'cart+')
        loop.request_stop()
        loop.record_result(('book0', True))
        self.assertIsNone(loop.next_task())
        self.assertEqual(loop.cycles, 1)

    def test_s_interrupts_intercycle_delay(self):
        loop = cart.AutoCartLoop()
        loop.next_task()
        loop.record_result(('book0', True))
        def stop_during_wait(seconds):
            loop.request_stop()
            return True
        with patch.object(loop.stop, 'wait', side_effect=stop_during_wait):
            self.assertIsNone(loop.next_task())
        self.assertEqual(loop.cycles, 1)

    def test_failed_execution_or_incomplete_perception_waits_for_enter(self):
        for result in (None, ('book0', False)):
            loop = cart.AutoCartLoop()
            loop.next_task()
            if result is not None:
                loop.record_result(result)
            loop.request_retry()  # Enter before the prompt must not be saved.
            returned = []
            thread = threading.Thread(target=lambda: returned.append(loop.next_task()), daemon=True)
            thread.start()
            try:
                self.assertTrue(loop.waiting_for_retry.wait(1.))
                self.assertTrue(thread.is_alive())
                self.assertEqual(loop.cycles, 1)
                loop.request_retry()
                thread.join(1.)
                self.assertFalse(thread.is_alive())
                self.assertEqual(returned, ['cart+'])
                self.assertEqual(loop.cycles, 2)
                self.assertFalse(loop.waiting_for_retry.is_set())
            finally:
                loop.request_stop()
                thread.join(1.)

    def test_failure_wait_accepts_terminal_enter_and_s(self):
        for key in (b'\r', b'\n', b's'):
            master, slave = pty.openpty()
            loop = cart.AutoCartLoop()
            loop.next_task()
            loop.record_result(('book0', False))
            returned = []
            thread = threading.Thread(target=lambda: returned.append(loop.next_task()), daemon=True)
            try:
                stream = types.SimpleNamespace(isatty=lambda: True, fileno=lambda: slave)
                original = termios.tcgetattr(slave)
                with cart.stop_key_listener(loop, stream):
                    thread.start()
                    self.assertTrue(loop.waiting_for_retry.wait(1.))
                    os.write(master, key)
                    thread.join(1.)
                    self.assertFalse(thread.is_alive())
                    self.assertEqual(returned, [None if key == b's' else 'cart+'])
                self.assertEqual(termios.tcgetattr(slave), original)
            finally:
                loop.request_stop()
                thread.join(1.)
                os.close(master)
                os.close(slave)

    def test_cycle_exception_hook_returns_to_provider_without_reinitialization(self):
        # Execute the real main-loop AST with injected dependencies, never its
        # hardware/model initialization or imports.
        tree = ast.parse(Path(cart.__file__).with_name('dual_v2_2_agent_pcl.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        body = next(n for n in main.body if isinstance(n, ast.While))
        code = compile(ast.fix_missing_locations(ast.Module(body=[body], type_ignores=[])), '<offline-cycle>', 'exec')
        loop = cart.AutoCartLoop()
        loop.last_success = True
        handler = Mock(side_effect=loop.record_failure)
        provider = Mock(side_effect=['cart+', None])
        namespace = dict(EXCLUSIVE_EXTENSION_MODE=True, EXTRA_TASK_TYPES=['cart+'],
                         TASK_TYPE_PROVIDER=provider, valid_task_types=['cart+'],
                         EXTRA_TASK_BUILDER=Mock(side_effect=RuntimeError('perception error')),
                         vlm_categories=[], TASK_FAILURE_HANDLER=handler)
        exec(code, namespace)
        self.assertFalse(loop.last_success)
        self.assertEqual(provider.call_count, 2)
        handler.assert_called_once()
        namespace.update(TASK_FAILURE_HANDLER=None, TASK_TYPE_PROVIDER=Mock(return_value='cart+'))
        with self.assertRaisesRegex(RuntimeError, 'perception error'):
            exec(code, namespace)

    def test_mode_hooks_track_result_and_restore_on_error(self):
        executor = Mock(return_value=('book0', True))
        agent = types.SimpleNamespace(TASK_TYPE_PROVIDER=None, execute_main=executor)
        loop = cart.AutoCartLoop()
        with self.assertRaises(RuntimeError):
            with cart.automatic_cart_mode(agent, loop):
                self.assertEqual(agent.TASK_TYPE_PROVIDER(), 'cart+')
                self.assertEqual(agent.execute_main(['tasks']), ('book0', True))
                self.assertTrue(loop.last_success)
                agent.TASK_FAILURE_HANDLER(RuntimeError('cycle failed'))
                self.assertFalse(loop.last_success)
                raise RuntimeError('test cleanup')
        self.assertIsNone(agent.TASK_TYPE_PROVIDER)
        self.assertIs(agent.execute_main, executor)
        self.assertIsNone(agent.TASK_FAILURE_HANDLER)

    def test_single_s_without_enter_and_terminal_restore(self):
        master, slave = pty.openpty()
        try:
            original = termios.tcgetattr(slave)
            stream = types.SimpleNamespace(isatty=lambda: True, fileno=lambda: slave)
            loop = cart.AutoCartLoop()
            with cart.stop_key_listener(loop, stream):
                os.write(master, b's')
                self.assertTrue(loop.stop.wait(1.))
            self.assertEqual(termios.tcgetattr(slave), original)
        finally:
            os.close(master)
            os.close(slave)

    def test_no_terminal_fails_before_running(self):
        with self.assertRaises(RuntimeError):
            with cart.stop_key_listener(cart.AutoCartLoop(), types.SimpleNamespace(isatty=lambda: False)):
                self.fail('must not start an unstoppable loop')

    def test_entrypoint_selects_cart_and_own_configs_and_cleans_up(self):
        agent = types.SimpleNamespace(TASK_TYPE_PROVIDER=None, execute_main=Mock(), cleanup_runtime_resources=Mock())
        book = types.SimpleNamespace(main=Mock())
        with (patch.dict(sys.modules, {'dual_v2_2_agent_pcl': agent, 'book_grasp_1_2_local': book}),
              patch.object(cart, 'stop_key_listener', return_value=nullcontext())):
            cart.main()
        kwargs = book.main.call_args.kwargs
        self.assertEqual(kwargs['task_types'], ['cart+'])
        self.assertEqual(kwargs['config_path'].name, 'cart_book_grasp_v1_config.json')
        self.assertEqual(kwargs['wrist_config_path'].name, 'cart_book_grasp_v1_wrist_config.json')
        agent.cleanup_runtime_resources.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
