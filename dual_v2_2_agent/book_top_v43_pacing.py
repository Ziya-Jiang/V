"""V4.3 progress pacing; final image tolerances remain unchanged."""
import numpy as np


class LiftPacing:
    def __init__(self, config):
        self.c = config
        self.reset()

    def reset(self):
        self.advancing = False
        self.aligned_steps = 0

    def step(self, error, tolerance, remaining):
        rho = float(np.max(np.abs(error)/tolerance))
        if not np.isfinite(rho):
            raise ValueError('Invalid lift pacing error')
        pause = self.c.get('progress_pause_error_ratio', 2.)
        if rho >= pause:
            self.advancing = False
        elif rho <= 1.:
            self.advancing = True
        self.aligned_steps = self.aligned_steps+1 if rho <= 1. else 0
        base = min(self.c['max_step_mm'], self.c.get('base_step_mm', 10.))
        cap = base
        if remaining <= self.c.get('near_goal_mm', 20.):
            cap = min(cap, self.c.get('near_step_mm', 5.))
        elif self.aligned_steps >= self.c.get('fast_aligned_steps', 2):
            cap = self.c['max_step_mm']
        if rho > 1.:
            # Small errors reduce progress without immediately switching to a
            # full stop/correction cycle. Never synthesize tiny sub-tolerance Z.
            cap *= (pause-rho)/(pause-1.)
        if cap <= self.c['max_pose_error_mm'] and remaining > self.c['x_tolerance_mm']:
            allowed = False
        else:
            allowed = self.advancing
        return allowed, max(0., cap), rho

    def history_step(self, remaining):
        # No fresh image confidence: historical continuation never gets 15 mm.
        cap = min(self.c['max_step_mm'], self.c.get('base_step_mm', 10.))
        if remaining <= self.c.get('near_goal_mm', 20.):
            cap = min(cap, self.c.get('near_step_mm', 5.))
        return cap
