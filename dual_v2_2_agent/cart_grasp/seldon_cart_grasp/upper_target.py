"""Original dual_v2_2_agent TargetCommand adapter for the V2.2 upper planner.

This module encodes goals; it never opens sockets or starts a controller.
Public commands are previews until the physical execution path is commissioned.
The wire protocol is imported from the same configuration tree as the planner.
"""
from pathlib import Path
import importlib
import sys
import uuid
import numpy as np


def array(value, shape, label):
    out = np.asarray(value, dtype=float)
    if out.shape != shape or not np.isfinite(out).all():
        raise ValueError(f"Invalid {label}: expected finite {shape}")
    return out.copy()


def poses(value):
    out = array(value, (2, 7), "dual tool poses [m,wxyz]")
    if np.max(np.abs(out[:, :3])) > 3:
        raise ValueError("Tool positions exceed local workspace; check mm/m")
    if not np.allclose(np.linalg.norm(out[:, 3:], axis=1), 1, atol=1e-5, rtol=0):
        raise ValueError("Tool quaternions must be unit wxyz")
    return out


def protocol(config_root):
    root = Path(config_root).resolve()
    scripts = root / "robot_platform_utils/python/scripts"
    udp = root / "curi_udp/python"
    for path in (scripts / "cuarm_state.py", scripts / "cuarm_udp.py", udp / "udp_socket.py"):
        if not path.is_file():
            raise FileNotFoundError(path)
    for name, directory in (("cuarm_state", scripts), ("cuarm_udp", scripts), ("udp_socket", udp)):
        loaded = sys.modules.get(name)
        if loaded and Path(loaded.__file__).resolve().parent != directory:
            raise ValueError("Conflicting protocol already imported: " + name)
    sys.path[:0] = [str(scripts), str(udp)]
    state = importlib.import_module("cuarm_state")
    udp_module = importlib.import_module("cuarm_udp")
    # Codec only. Do NOT call CuarmUdp.__init__, which creates sockets.
    codec = object.__new__(udp_module.CuarmUdp)
    return state, codec


def tool_pose_from_upper_tip(upper_T_tip, half_T_upper, offset_from_ee):
    """Match CuRobo FK: half_T_tip + rotated config offset, NOT contact TCP.

    The contact-to-flange conversion must already have been applied upstream.
    FrameGraph.relative('dummy_world', 'SJ4Z') supplies half_T_upper.
    """
    from scipy.spatial.transform import Rotation
    from .geometry import transform
    t = transform(half_T_upper) @ transform(upper_T_tip)
    xyz = t[:3, 3] + t[:3, :3] @ array(offset_from_ee, (3,), "planner tool offset")
    xyzw = Rotation.from_matrix(t[:3, :3]).as_quat()
    return np.r_[xyz, xyzw[3], xyzw[:3]].tolist()


def preview_command(types, state, stage):
    """Preserve both measured arms and both normalized grippers by default."""
    if state.get("arm_size") != 2 or state.get("arm_joint_size") != [7, 7]:
        raise ValueError("Expected two V2.2 arms with 7 joints each")
    if state.get("gripper_size") != 2 or state.get("gripper_joint_size") != [6, 1]:
        raise ValueError("Expected left 6-joint hand and right 1-joint claw")
    q = array(state["arm_joint_position"], (2, 7), "joint feedback")
    if np.max(np.abs(q)) > 2 * np.pi + .01:
        raise ValueError("Joint feedback is outside the V2.2 angular range")
    v = array(state["arm_joint_velocity"], (2, 7), "joint speed")
    if np.max(np.abs(v)) > .02:
        raise ValueError("Preview requires stationary arms")
    tool = poses(state["arm_tool_pose"])
    if len(state["gripper_joint_position"]) != 2:
        raise ValueError("Expected feedback for exactly two end effectors")
    grippers = [array(g, (n,), "normalized gripper feedback") for g, n in
                zip(state["gripper_joint_position"], [6, 1])]
    if len(grippers) != 2 or any(np.any(g < 0) or np.any(g > 1) for g in grippers):
        raise ValueError("Missing or unnormalized gripper feedback")
    if set(stage) - {"name", "kind", "targets", "frame"}:
        raise ValueError("Unknown stage fields; no gripper closure in approach preview")
    name = stage.get("name", "")
    if not name or len(name) > 80 or not name.isascii() or "#" in name:
        raise ValueError("Use a short ASCII stage name without protocol separators")
    targets = stage.get("targets", {})
    if not targets or set(targets) - {"left", "right"}:
        raise ValueError("Explicit left/right targets are required")
    cmd = types.TargetCommand()
    cmd.planner_mode = types.TargetCommandPlannerMode.PLANNING
    cmd.interpolation_acceleration_time = 5.0
    cmd.no_interpolation_max_velocity_ratio = .05
    cmd.arm_size, cmd.arm_joint_size = 2, [7, 7]
    if stage["kind"] == "joint":
        if stage.get("frame") != "V2 calibrated":
            raise ValueError("Joint targets require V2 calibrated radians")
        cmd.arm_command_type = types.TargetCommandArmCommandType.JOINT
        for side, label in enumerate(("left", "right")):
            if label in targets:
                q[side] = array(targets[label], (7,), "joint target in radians")
        if np.max(np.abs(q)) > 2 * np.pi + .01:
            raise ValueError("Joint targets exceed V2 range; check degrees/radians")
    elif stage["kind"] == "cartesian":
        if stage.get("frame") != "half_model::dummy_world/planner_tool":
            raise ValueError("Cartesian goal must use half-root planner tool frame")
        cmd.arm_command_type = types.TargetCommandArmCommandType.CARTESIAN
        for side, label in enumerate(("left", "right")):
            if label in targets:
                tool[side] = array(targets[label], (7,), "tool target")
        tool = poses(tool)
    else:
        raise ValueError("Unknown stage kind")
    cmd.arm_joint_command, cmd.arm_tool_command = q.tolist(), tool.tolist()
    cmd.gripper_command_type = types.TargetCommandGripperCommandType.JOINT
    cmd.gripper_size, cmd.gripper_joint_size = 2, [6, 1]
    # MODE values are protocol placeholders; JOINT preserves feedback values.
    cmd.gripper_mode_command = [types.GripperMode.OPEN] * 2
    cmd.gripper_joint_command = [g.tolist() for g in grippers]
    cmd.description = "__preview__:cart:" + name + ":" + uuid.uuid4().hex
    return cmd


def outward_stage(q, bounds, delta_rad=.15):
    """A candidate for both shoulders moving outward; NOT a safe preset.

    Only applies to the verified V2 arms-down branch. Other poses must use a
    new route search. The original planner must check the full trajectory.
    """
    q = array(q, (2, 7), "current V2 joints")
    bounds = array(bounds, (2, 7, 2), "active joint limits")
    if not .01 <= delta_rad <= .3:
        raise ValueError("Outward step must be 0.01..0.3 rad")
    if np.any(q[:, 0] < 1.3) or np.any(q[:, 0] > 1.8) or np.any(q[:, 1] < 1.0):
        raise ValueError("Not the verified arms-down branch; no fixed reset fallback")
    target = q.copy()
    target[:, 1] -= delta_rad
    if np.any(bounds[:, :, 0] >= bounds[:, :, 1]):
        raise ValueError("Invalid active limits")
    for sample in (q, target):
        if np.any(sample < bounds[:, :, 0]) or np.any(sample > bounds[:, :, 1]):
            raise ValueError("Outward candidate exceeds active joint limits")
    return {"name": "both_arms_outward", "kind": "joint", "frame": "V2 calibrated",
            "targets": {"left": target[0].tolist(), "right": target[1].tolist()}}
