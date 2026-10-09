import copy
import os
from pathlib import Path
import numpy as np
import pytest
from seldon_cart_grasp.upper_target import protocol, preview_command, outward_stage, tool_pose_from_upper_tip


@pytest.fixture
def wire():
    return protocol(os.environ["CUARM_CONFIG_ROOT"])


@pytest.fixture
def feedback():
    return {"arm_size": 2, "arm_joint_size": [7, 7],
            "arm_joint_position": [[1.57, 1.4, .17, .23, .17, -.17, 0]] * 2,
            "arm_joint_velocity": [[0] * 7] * 2,
            "arm_tool_pose": [[.1, .25, -.5, 1, 0, 0, 0], [.1, -.25, -.5, 1, 0, 0, 0]],
            "gripper_size": 2, "gripper_joint_size": [6, 1],
            "gripper_joint_position": [[1, .1, .2, .3, .4, .5], [.25]]}


def goal():
    return {"name": "right_hover", "kind": "cartesian",
            "frame": "half_model::dummy_world/planner_tool",
            "targets": {"right": [.55, -.2, .1, 1, 0, 0, 0]}}


def test_original_protocol_roundtrip_preserves_inactive_arm_and_grippers(wire, feedback):
    types, codec = wire
    c = preview_command(types, feedback, goal())
    decoded = codec.unpack_target_command(codec.pack_target_command(c))
    assert decoded.planner_mode == types.TargetCommandPlannerMode.PLANNING
    assert decoded.description.startswith("__preview__:cart:")
    assert decoded.arm_tool_command[0] == feedback["arm_tool_pose"][0]
    assert decoded.arm_joint_command == feedback["arm_joint_position"]
    assert decoded.gripper_joint_command == feedback["gripper_joint_position"]
    assert decoded.gripper_command_type == types.TargetCommandGripperCommandType.JOINT


@pytest.mark.parametrize("bad", ["mm", "frame", "quaternion", "nan", "delimiter", "closure"])
def test_rejects_ambiguous_or_malformed_goals(wire, feedback, bad):
    types, _ = wire
    stage = goal()
    if bad == "mm": stage["targets"]["right"][0] = 550
    if bad == "frame": stage["frame"] = "SJ4Z"
    if bad == "quaternion": stage["targets"]["right"][3] = 0
    if bad == "nan": stage["targets"]["right"][0] = float("nan")
    if bad == "delimiter": stage["name"] = "preview#execute"
    if bad == "closure": stage["close_gripper"] = True
    with pytest.raises(ValueError): preview_command(types, feedback, stage)


@pytest.mark.parametrize("bad", ["moving", "missing_hand", "extra_hand", "unnormalized", "sentinel"])
def test_rejects_invalid_feedback(wire, feedback, bad):
    types, _ = wire
    state = copy.deepcopy(feedback)
    if bad == "moving": state["arm_joint_velocity"][0][0] = .1
    if bad == "missing_hand": state["gripper_joint_position"] = [[0]]
    if bad == "extra_hand": state["gripper_joint_position"].append([0])
    if bad == "unnormalized": state["gripper_joint_position"][1] = [1000]
    if bad == "sentinel": state["arm_joint_position"][0][0] = -999
    with pytest.raises(ValueError): preview_command(types, state, goal())


def test_outward_uses_measured_pose_and_both_arms(feedback):
    q = np.array(feedback["arm_joint_position"])
    limits = np.tile([-3.14, 3.14], (2, 7, 1))
    stage = outward_stage(q, limits)
    expected = q.copy()
    expected[:, 1] -= .15
    assert np.allclose([stage["targets"][s] for s in ("left", "right")], expected)
    with pytest.raises(ValueError): outward_stage(np.zeros((2, 7)), limits)
    limits[:, 1, 0] = 1.35
    with pytest.raises(ValueError): outward_stage(q, limits)


def test_planner_tool_offset_and_half_root_are_applied_once():
    root = np.eye(4)
    root[2, 3] = .342
    tip = np.eye(4)
    tip[:3, 3] = [.5, -.2, -.3]
    tip[:3, :3] = np.diag([1., -1., -1.])
    pose = tool_pose_from_upper_tip(tip, root, [0, 0, -.2])
    assert np.allclose(pose[:3], [.5, -.2, .242])
    assert np.isclose(abs(pose[4]), 1.)
