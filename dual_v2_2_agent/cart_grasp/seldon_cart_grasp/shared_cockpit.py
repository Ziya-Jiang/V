"""Use the unmodified cockpit controller with a single RGB-D camera owner.

This wrapper does not claim control, enable motors, or launch RT. The existing
cockpit's startup/shutdown and all control interlocks remain its own behavior.
"""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'keyboard_cockpit'))

def main():
    from seldon_cockpit import webapp
    from .orbbec_capture import run_camera
    webapp.run_camera=run_camera
    webapp.main()
if __name__=='__main__':main()
