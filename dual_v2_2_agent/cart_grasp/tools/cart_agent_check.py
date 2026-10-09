#!/usr/bin/env python3
"""Check the selected original Agent environment without constructing RobotArm."""
import argparse
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--v-root", type=Path, default=Path("/mnt/ssd/user/V"))
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    agent = args.v_root / "dual_v2_2_agent"
    config = args.v_root / "dual_v2_2_configuration"
    planner = args.v_root / "dual_v2_2_control/cuarm_upper_planner/dual_v2_2_upper_planner.py"
    files = [agent / "dual_v2_2_agent_pcl.py", agent / "RobotRAM/controllor/hkstp_arm.py", planner,
             config / "config.yaml", config / "share/left_hand_right_gripper_half/dual_arm_half.urdf"]
    missing = [str(f) for f in files if not f.is_file()]
    if missing:
        raise RuntimeError("Missing original Agent dependencies: " + str(missing))
    os.environ["CUARM_CONFIG_ROOT"] = str(config)
    sys.path.insert(0, str(agent / "RobotRAM"))
    module = importlib.import_module("controllor.hkstp_arm")
    if Path(module.__file__).resolve() != files[1].resolve():
        raise RuntimeError("Imported a different RobotArm implementation")
    # Import class only; constructor binds UDP, so deliberately do not call it.
    report = {"agent_root": str(agent), "planner_entry": str(planner),
              "python": sys.executable, "robotarm_source": module.__file__,
              "network_packets_sent": 0, "hardware_started": False,
              "using_miniagent": False,
              "dependencies": {n: importlib.util.find_spec(n) is not None for n in ("torch", "curobo", "cv2", "scipy")},
              "source_sha256": {str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in files},
              "next_required": ["remote registered RGB-D", "Upper Software/RT protocol connection",
                                "current-state staged grasp planning with cart obstacles"]}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
