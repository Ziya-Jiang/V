from dataclasses import dataclass
from importlib.machinery import EXTENSION_SUFFIXES
from pathlib import Path
import os
import platform
import sys

import numpy as np

# Detect architecture and load the ABI-compatible configuration extension.
default_config_root = (
    Path(__file__).resolve().parents[3] / "dual_v2_2_configuration"
)
config_root = Path(
    os.environ.get("CUARM_CONFIG_ROOT", str(default_config_root))
).expanduser().resolve()
arch = platform.machine().lower()
if arch in ("x86_64", "amd64"):
    lib_path = config_root / "robot_platform_utils" / "python" / "lib" / "x86"
elif arch in ("aarch64", "arm64"):
    lib_path = config_root / "robot_platform_utils" / "python" / "lib" / "arm64"
else:
    raise RuntimeError(f"Unsupported arch: {arch}")

config_loader_module = next(
    (
        lib_path / f"config_loader{suffix}"
        for suffix in EXTENSION_SUFFIXES
        if (lib_path / f"config_loader{suffix}").is_file()
    ),
    None,
)
if config_loader_module is None:
    raise RuntimeError(
        "No config_loader extension matches this Python ABI: "
        f"{lib_path}/config_loader{EXTENSION_SUFFIXES[0]}"
    )
sys.path.insert(0, str(lib_path))

import config_loader # cuarm_configuration.share.python.lib

@dataclass
class CuroboConfig:
    """
    Configuration class for the Curobo robot system.
    
    Attributes:
        config (dict): Full configuration dictionary loaded from YAML.
        world_cfg (dict): World/environment configuration.
        robot_cfg (dict): Robot-specific configuration.
        target_link_names (list): List of target link names in the robot.
        target_size (int): Number of target links.
        ee_link_name (str): Name of the end-effector link.
        ee_link_idx_in_target_link_names (int): Index of the end-effector link in target_link_names.
        joint_names (list): List of joint names in the robot's configuration space.
        joint_size (int): Number of joints.
        arm_size (int): Number of arms in the robot.
        arm_joint_size (list): List of joint counts per arm.
        total_arm_joint_names (list): Flattened list of all arm joint names.
        total_arm_joint_size (int): Total number of arm joints.
        gripper_size (int): Number of grippers.
        gripper_joint_size (list): List of joint counts per gripper.
        total_gripper_joint_names (list): Flattened list of all gripper joint names.
        total_gripper_joint_size (int): Total number of gripper joints.
        end_effector_offset (np.ndarray): Offsets for the end-effectors of each arm.
    """
    
    config: dict
    world_cfg: dict
    robot_cfg: dict
    target_link_names: list
    target_size: int
    ee_link_name: str
    ee_link_idx_in_target_link_names: int
    joint_names: list
    joint_size: int
    arm_size: int
    arm_joint_size: list
    total_arm_joint_names: list
    total_arm_joint_size: int
    gripper_size: int
    gripper_types: list
    gripper_joint_size: list
    total_gripper_joint_names: list
    total_gripper_joint_size: int
    end_effector_offset: np.ndarray
    
    @staticmethod
    def load_from_yaml_path(config_path: str):
        config_path = Path(config_path).expanduser().resolve()
        if not config_path.is_file():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")

        yaml_node = config_loader.load_yaml(str(config_path))
        config = yaml_node.as_dict()
        try:
            world_cfg = config["curobo"]["world_cfg"]
            robot_cfg = config["curobo"]["robot_cfg"]
            robot_arms = config["robot"]["arm"]
        except (KeyError, TypeError) as exc:
            raise ValueError(
                f"Incomplete CuRobo configuration: {config_path}"
            ) from exc
        
        # Ensure absolute path for URDF file
        if not os.path.isabs(robot_cfg["kinematics"]["urdf_path"]):
            robot_cfg["kinematics"]["urdf_path"] = str(
                (config_path.parent / robot_cfg["kinematics"]["urdf_path"]).resolve()
            )
        urdf_path = Path(robot_cfg["kinematics"]["urdf_path"])
        if not urdf_path.is_file():
            raise FileNotFoundError(f"Robot URDF not found: {urdf_path}")
        
        target_link_names = robot_cfg["kinematics"]["link_names"]
        target_size = len(target_link_names) if target_link_names else 1
        ee_link_name = robot_cfg["kinematics"]["ee_link"]
        if target_link_names and ee_link_name not in target_link_names:
            raise ValueError(
                f"End-effector link {ee_link_name!r} is absent from link_names"
            )
        ee_link_idx_in_target_link_names = 0 if not target_link_names else target_link_names.index(ee_link_name)
        joint_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
        joint_size = len(joint_names)
        arm_size = len(robot_arms)
        arm_joint_size = [arm["joint_size"] for arm in robot_arms]
        total_arm_joint_names = [name for arm in robot_arms for name in arm["urdf_joints"]]
        total_arm_joint_size = len(total_arm_joint_names)

        gripper_size = len(config["robot"]["gripper"]) if "gripper" in config["robot"] else 0
        gripper_types, gripper_joint_size, total_gripper_joint_names, total_gripper_joint_size = [], [], [], 0
        end_effector_offset = np.zeros((arm_size, 3))
        if (gripper_size > 0):
            for gripper in config["robot"]["gripper"]:
                gripper_types.append(gripper["type"])
            gripper_joint_size = [gripper["joint_size"] for gripper in config["robot"]["gripper"]]
            total_gripper_joint_names = [name for gripper in config["robot"]["gripper"] for name in gripper["urdf_joints"]] if gripper_size > 0 else []
            total_gripper_joint_size = len(total_gripper_joint_names)
            
            for i, gripper in enumerate(config["robot"]["gripper"]):
                if "offset_from_ee" in gripper:
                    end_effector_offset[i, :] = gripper["offset_from_ee"]

        total_robot_joint_size = total_arm_joint_size + total_gripper_joint_size
        if total_robot_joint_size != joint_size:
            raise ValueError(
                "Robot joint count does not match CuRobo cspace: "
                f"{total_robot_joint_size} != {joint_size} "
                f"(arms={total_arm_joint_size}, grippers={total_gripper_joint_size})"
            )
                
        return CuroboConfig(
            config=config,
            world_cfg=world_cfg,
            robot_cfg=robot_cfg,
            target_link_names=target_link_names,
            target_size=target_size,
            ee_link_name=ee_link_name,
            ee_link_idx_in_target_link_names=ee_link_idx_in_target_link_names,
            joint_names=joint_names,
            joint_size=joint_size,
            arm_size=arm_size,
            arm_joint_size=arm_joint_size,
            total_arm_joint_names=total_arm_joint_names,
            total_arm_joint_size=total_arm_joint_size,
            gripper_size=gripper_size,
            gripper_types=gripper_types,
            gripper_joint_size=gripper_joint_size,
            total_gripper_joint_names=total_gripper_joint_names,
            total_gripper_joint_size=total_gripper_joint_size,
            end_effector_offset=end_effector_offset,
        )