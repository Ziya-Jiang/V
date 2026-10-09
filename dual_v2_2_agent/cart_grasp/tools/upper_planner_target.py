#!/usr/bin/env python3
"""Encode original Agent preview goals without starting any hardware."""
import argparse
import hashlib
import json
from pathlib import Path
from seldon_cart_grasp.upper_target import protocol, preview_command, outward_stage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-root", type=Path, required=True)
    parser.add_argument("--state", type=Path, required=True,
                        help="Recorded CuroboState JSON; never treated as live feedback")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--stage", type=Path)
    group.add_argument("--outward-limits", type=Path,
                       help="2x7x2 active V2 soft-limit JSON for an outward candidate")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    types, codec = protocol(args.config_root)
    state = json.loads(args.state.read_text())
    stage = (json.loads(args.stage.read_text()) if args.stage else
             outward_stage(state["arm_joint_position"], json.loads(args.outward_limits.read_text())))
    cmd = preview_command(types, state, stage)
    payload = codec.pack_target_command(cmd)
    recovered = codec.unpack_target_command(payload)
    if recovered.description != cmd.description:
        raise RuntimeError("Canonical protocol round-trip failed")
    report = {"hardware_executable": False, "network_packets_sent": 0,
              "source": "recorded_state_only", "stage": stage,
              "command_description": cmd.description, "target_command_wire": payload,
              "wire_sha256": hashlib.sha256(payload.encode("ISO-8859-1")).hexdigest(),
              "chain": ["dual_v2_2_agent TargetCommand", "dual_v2_2_upper_planner.py",
                        "RemotePanelCommand", "native Upper Software", "RT"],
              "requires": ["live PanelState and active RT limits", "cart/environment in planner world",
                           "whole-stage trajectory review", "validated tool/contact transforms",
                           "verified stop and stale-target handling"]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "network_packets_sent": 0,
                      "hardware_executable": False, "stage": stage["name"]}))


if __name__ == "__main__":
    main()
