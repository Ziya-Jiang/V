#!/usr/bin/env python3
"""Build a test against native upper objects without launching MainWindow/RT."""
from pathlib import Path
import os
import shlex
import subprocess
import resource

resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

root = Path('/mnt/ssd/user/V/dual_v2_2_agent/cart_grasp')
build = Path('/mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_software/build')
flags = {}
for line in (build/'CMakeFiles/dual_v2_2_upper_software.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_INCLUDES = ', 'CXX_FLAGS = ')):
        name, value = line.split(' = ', 1)
        flags[name] = shlex.split(value)
obj = root/'runtime/test_native_remote_widget.o'
binary = root/'runtime/test_native_remote_widget'
subprocess.run(['/usr/bin/c++', *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
                '-c', str(root/'tests/test_native_remote_widget.cpp'), '-o', str(obj)], check=True)
link = shlex.split((build/'CMakeFiles/dual_v2_2_upper_software.dir/link.txt').read_text())
index = link.index('CMakeFiles/dual_v2_2_upper_software.dir/test/panel.cpp.o')
link[index] = str(obj)
link[link.index('-o')+1] = str(binary)
subprocess.run(link, cwd=build, check=True)
subprocess.run([str(binary)], env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen'}, check=True, timeout=10)
