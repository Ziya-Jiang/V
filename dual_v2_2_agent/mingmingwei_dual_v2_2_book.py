"""Old entry name. The shelf and cart choices now live in book_grasp.py."""

import runpy
import sys
from pathlib import Path

if __name__ == "__main__":
    target = Path(__file__).with_name("book_grasp.py")
    sys.argv = [str(target), *sys.argv[1:]]
    runpy.run_path(str(target), run_name="__main__")
