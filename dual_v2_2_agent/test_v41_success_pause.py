"""Offline input gate: no hardware or perception imports."""
import threading
import unittest
from cart_book_grasp_v1_3 import AutoCartLoop
from cart_book_grasp_v1_3_pause import PauseRequested

class SuccessPause(unittest.TestCase):
    def test_success_waits_enter_stop_or_pause_without_new_cycle(self):
        for key in ('enter','s','p'):
            with self.subTest(key=key):
                loop=AutoCartLoop(delay_seconds=0.,wait_for_enter_after_success=True)
                self.assertEqual(loop.next_task(),'cart+')
                loop.record_result(('book',True))
                loop.request_retry()  # Enter before waiting must not queue a cycle.
                result=[]
                def run():
                    try:result.append(loop.next_task())
                    except PauseRequested:result.append('pause')
                worker=threading.Thread(target=run,daemon=True);worker.start()
                try:
                    self.assertTrue(loop.waiting_for_retry.wait(1.))
                    worker.join(.05)
                    self.assertTrue(worker.is_alive());self.assertEqual(loop.cycles,1)
                    {'enter':loop.request_retry,'s':loop.request_stop,'p':loop.request_pause}[key]()
                    worker.join(1.)
                    self.assertFalse(worker.is_alive())
                    self.assertEqual(result,[{'enter':'cart+','s':None,'p':'pause'}[key]])
                    self.assertEqual(loop.cycles,2 if key=='enter' else 1)
                finally:
                    loop.request_stop();worker.join(1.)
    def test_original_success_default_is_unchanged(self):
        loop=AutoCartLoop(delay_seconds=0.)
        loop.next_task();loop.record_result(('book',True))
        self.assertEqual(loop.next_task(),'cart+')
        self.assertFalse(loop.waiting_for_retry.is_set())

if __name__=='__main__':unittest.main()
