"""Offline auto-loop tests, including a simulated terminal for the s key."""
from contextlib import nullcontext
import os
import pty
import sys
import termios
import types
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

    def test_failed_execution_or_incomplete_perception_never_retries(self):
        for result in (None, ('book0', False)):
            loop = cart.AutoCartLoop()
            loop.next_task()
            if result is not None:
                loop.record_result(result)
            with patch.object(loop.stop, 'wait') as wait:
                self.assertIsNone(loop.next_task())
                wait.assert_not_called()

    def test_mode_hooks_track_result_and_restore_on_error(self):
        executor = Mock(return_value=('book0', True))
        agent = types.SimpleNamespace(TASK_TYPE_PROVIDER=None, execute_main=executor)
        loop = cart.AutoCartLoop()
        with self.assertRaises(RuntimeError):
            with cart.automatic_cart_mode(agent, loop):
                self.assertEqual(agent.TASK_TYPE_PROVIDER(), 'cart+')
                self.assertEqual(agent.execute_main(['tasks']), ('book0', True))
                self.assertTrue(loop.last_success)
                raise RuntimeError('test cleanup')
        self.assertIsNone(agent.TASK_TYPE_PROVIDER)
        self.assertIs(agent.execute_main, executor)

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
