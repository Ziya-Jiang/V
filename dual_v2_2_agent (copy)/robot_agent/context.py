"""Shared runtime context for the grasping system.

``RobotContext`` is the single place that owns the resources every tool reads --
robot arm, cameras, perception models, calibration -- plus the transient *scene*
state threaded between perception tools and the per-object ``object_id_to_type``
mapping needed by the manipulator.

The context is deliberately **lazy**: constructing it imports nothing heavy
(no torch, no camera SDK, no arm controller). Everything that touches hardware
or GPU models lives in :meth:`RobotContext.bootstrap`, so the Agent's planning
path stays importable and testable without a robot attached.
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from typing import Any, Dict, Optional

AGENT_ROOT = Path(__file__).resolve().parent.parent
ROBOTRAM_ROOT = Path(
    os.environ.get("ROBOTRAM_ROOT", AGENT_ROOT / "RobotRAM")
).expanduser().resolve()
ROBOTRAM_SCRIPTS_ROOT = ROBOTRAM_ROOT / "scripts"

DEFAULT_ORBBEC_CALIBRATION_PATH = Path(
    os.environ.get(
        "ROBOTRAM_ORBBEC_CALIBRATION_PATH",
        AGENT_ROOT
        / "calibration"
        / "session_demo"
        / "base_to_camera"
        / "results"
        / "machine-readable results.json",
    )
)

# UDP ports aligned with cuarm_curobo/tutorial/planner_to_panel.py
DEFAULT_TARGET_LOCAL_PORT = 10165
DEFAULT_TARGET_REMOTE_PORT = 10166
DEFAULT_ORBBEC_PRESET = os.environ.get("ROBOTRAM_ORBBEC_PRESET", "Dual Color Streams")

# Bootstrap stage labels, reported to the model-loading progress visualizer.
BOOTSTRAP_STAGES = (
    ("arm", "连接机械臂", "运动规划 UDP"),
    ("calibration", "加载标定", "相机外参"),
    ("camera", "初始化相机", "Orbbec + RealSense"),
    ("grounding", "GroundingDINO + SAM2", "检测与分割"),
    ("vggt", "VGGT-1B", "深度与位姿"),
    ("ram", "RAM", "抓取位姿"),
)

# Workspace-local status file (not /tmp): the server writes it unconfined, and
# the sandboxed Host half reads it back through the workspace bind mount.
STATUS_PATH = str(AGENT_ROOT / ".robot_agent_status.json")


def write_status(state, current=0, label="", detail="", error=None):
    """Persist one model-loading progress snapshot for the GUI visualizer."""
    import json

    total = len(BOOTSTRAP_STAGES)
    payload = {
        "state": state,          # "loading" | "ready" | "error"
        "current": int(current),
        "total": total,
        "label": label,
        "detail": detail,
        "stages": [s[1] for s in BOOTSTRAP_STAGES],
        "error": error,
    }
    try:
        with open(STATUS_PATH, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
    except Exception:  # noqa: BLE001
        pass


def ensure_robotram_on_path() -> None:
    """Make the RobotRAM package importable *lazily*, only when tools run."""
    for path in (str(ROBOTRAM_SCRIPTS_ROOT), str(ROBOTRAM_ROOT)):
        if path not in sys.path:
            sys.path.insert(0, path)
    os.environ.setdefault("ROBOTRAM_ROOT", str(ROBOTRAM_ROOT))


def load_orbbec_calibration(calibration_path: Optional[Path] = None) -> Dict[str, Any]:
    """Load and validate the camera-to-robot-base calibration (mirrors the
    legacy ``dual_v2_2_agent.load_orbbec_calibration`` without importing it)."""
    import numpy as np

    path = Path(calibration_path or DEFAULT_ORBBEC_CALIBRATION_PATH)
    with open(path, "r", encoding="utf-8") as f:
        calibration = json_load(f)

    try:
        params = calibration["left"]
        camera_matrix = np.asarray(params["K"], dtype=float)
        rotation = np.asarray(params["R"], dtype=float)
        translation = np.asarray(params["T"], dtype=float).reshape(3, 1)
        distortion = np.asarray(params["D"], dtype=float)
        width = int(params["w"])
        height = int(params["h"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Invalid Orbbec calibration file: {path}") from error

    if camera_matrix.shape != (3, 3) or rotation.shape != (3, 3):
        raise ValueError(f"Invalid K/R matrix shape in calibration file: {path}")
    if not all(
        np.isfinite(v).all()
        for v in (camera_matrix, rotation, translation, distortion)
    ):
        raise ValueError(f"Calibration contains non-finite values: {path}")
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise ValueError(f"Calibration focal lengths must be positive: {path}")

    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
    if orthogonality_error > 1e-3 or abs(float(np.linalg.det(rotation)) - 1.0) > 1e-3:
        raise ValueError(f"Calibration R is not a valid rotation matrix: {path}")

    transform_base_from_camera = np.eye(4, dtype=float)
    transform_base_from_camera[:3, :3] = rotation
    transform_base_from_camera[:3, 3] = translation.reshape(3)

    return {
        "path": str(path),
        "K": camera_matrix,
        "R": rotation,
        "T": translation,
        "D": distortion,
        "w": width,
        "h": height,
        "T_base_cam": transform_base_from_camera,
    }


def json_load(f):
    import json

    return json.load(f)


class RobotContext:
    """Owns runtime resources and the transient scene state shared by tools."""

    def __init__(
        self,
        output_dir: Optional[Path] = None,
        instruction: str = "clean the table",
        plan_only: bool = False,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.instruction = instruction
        self.output_dir = Path(output_dir) if output_dir else ROBOTRAM_ROOT / "output" / "scene"
        self.plan_only = plan_only
        self.logger = logger or logging.getLogger("robot_agent")

        # Heavy resources -- populated by bootstrap().
        self.robot = None
        self.extern_camera = None       # Orbbec (left/right RGB) for grounding
        self.extern_camera2 = None      # RealSense (third RGB) for VGGT
        self.extern_camera_class = None  # Orbbec carrying RGB/depth into RAM
        self.vgm = None                 # DinoGrounding
        self.vggt_model = None
        self.vggt_device = None
        self.vggt_dtype = None
        self.ram_model = None
        self.ram_template = None
        self.ram_io = None

        self.calibration: Dict[str, Any] = {}
        self.object_id_to_type: Dict[str, str] = {}

        # Transient scene state threaded between perception tools.
        self.scene: Dict[str, Any] = {}

    # ------------------------------------------------------------------ #
    # Bootstrap / teardown
    # ------------------------------------------------------------------ #
    def bootstrap(self) -> "RobotContext":
        """Initialize arm + cameras + models. Heavy; requires hardware/GPU."""
        if self.plan_only:
            self.logger.info("plan_only: skipping hardware/model bootstrap")
            return self

        ensure_robotram_on_path()
        import atexit
        import copy

        import numpy as np
        import torch

        from utils.other_utils import set_random_seeds

        set_random_seeds(2)

        # --- Robot arm --------------------------------------------------- #
        write_status("loading", 0, "连接机械臂", "运动规划 UDP")
        from controllor.hkstp_arm import RobotArm

        self.robot = RobotArm(
            target_host="127.0.0.1",
            target_local_port=DEFAULT_TARGET_LOCAL_PORT,
            target_remote_port=DEFAULT_TARGET_REMOTE_PORT,
        )
        self.robot.initialize_target_command()

        # --- Calibration -------------------------------------------------- #
        write_status("loading", 1, "加载标定", "相机外参")
        self.calibration = load_orbbec_calibration()
        self.head_camera_reference_calibration = copy.deepcopy(self.calibration)

        # --- Cameras ------------------------------------------------------ #
        write_status("loading", 2, "初始化相机", "Orbbec + RealSense")
        from controllor.extern_camera import (
            ExternCameraCallbacks,
            OrbbecCamera,
            RealSense455Camera,
        )

        self.extern_camera2 = RealSense455Camera(color_only=True, warmup_frames=5)
        self.extern_camera = ExternCameraCallbacks["Orbbec"](
            preset_name=DEFAULT_ORBBEC_PRESET,
            warmup_frames=5,
        )
        self.extern_camera_class = OrbbecCamera(
            is_warmup=False,
            preset_name=DEFAULT_ORBBEC_PRESET,
        )
        for camera in (self.extern_camera, self.extern_camera_class):
            camera.camparas_file = self.calibration["path"]
            camera.K = self.calibration["K"].copy()
            camera.R = self.calibration["R"].copy()
            camera.T = self.calibration["T"].copy()
            camera.D = self.calibration["D"].copy()
            camera.w = self.calibration["w"]
            camera.h = self.calibration["h"]

        # --- Perception models ------------------------------------------- #
        from visions.grounding.dino_grounding import DinoGrounding
        from visions.ram.ram import RAMIO, RAMTemplate, RAMModel
        from visions.vggt.load_model import load_vggt_model

        write_status("loading", 3, "GroundingDINO + SAM2", "检测与分割")

        # GPU selection: ROBOTRAM_CUDA_DEVICE=0/1/... (or "cpu"). Defaults to
        # "1" here so the heavy inference avoids the training job on GPU 0.
        device_name = os.environ.get("ROBOTRAM_CUDA_DEVICE", "1")
        if device_name.lower() in ("cpu", "none"):
            self.vggt_device = torch.device("cpu")
            device_label = "cpu"
        else:
            self.vggt_device = torch.device(f"cuda:{int(device_name)}")
            device_label = f"cuda:{int(device_name)}"

        if self.vggt_device.type == "cuda":
            capability = torch.cuda.get_device_capability(self.vggt_device.index)
            self.vggt_dtype = torch.bfloat16 if capability[0] >= 8 else torch.float16
        else:
            self.vggt_dtype = torch.float16

        self.vgm = DinoGrounding(device=device_label)

        write_status("loading", 4, "VGGT-1B", "深度与位姿")
        self.vggt_model = load_vggt_model("facebook/VGGT-1B").to(self.vggt_device)

        write_status("loading", 5, "RAM", "抓取位姿")
        config_path = str(ROBOTRAM_ROOT / "visions" / "ram" / "assets" / "ram.json")
        self.ram_model = RAMModel(config_path, device=device_label)
        self.ram_template = RAMTemplate(config_path)
        self.ram_io = RAMIO()

        write_status("ready", len(BOOTSTRAP_STAGES), "就绪", "模型已加载")

        atexit.register(self.cleanup)
        self.logger.info("RobotContext bootstrapped (arm, cameras, models, calibration)")
        return self

    def cleanup(self) -> None:
        """Release SDK/GPU resources in a deterministic order."""
        for name, obj in (
            ("robot", self.robot),
            ("extern_camera", self.extern_camera),
            ("extern_camera2", self.extern_camera2),
            ("extern_camera_class", self.extern_camera_class),
        ):
            if obj is None:
                continue
            closer = getattr(obj, "shutdown", None) or getattr(obj, "close", None)
            if callable(closer):
                try:
                    closer()
                except Exception as error:  # noqa: BLE001
                    self.logger.warning("failed to close %s: %s", name, error)
            setattr(self, name, None)

    # ------------------------------------------------------------------ #
    # Scene helpers
    # ------------------------------------------------------------------ #
    def find_object_id(self, category: str, color: Optional[str] = None) -> Optional[str]:
        """Resolve a semantic object spec to its ``object_<id>`` key."""
        categories = self.scene.get("vlm_categories_dict") or {}
        for obj_key, info in categories.items():
            if (info.get("category") or "").lower() != (category or "").lower():
                continue
            if color and (info.get("color") or "").lower() != color.lower():
                continue
            return obj_key
        return None

    def load_categories_from_output(self) -> bool:
        """Hydrate semantic object categories from ``vlm_categories.json``.

        Pure file I/O (no hardware/models): lets a *fresh* process resolve
        semantic ``{category, color}`` specs to ``object_<id>`` without
        re-running perception. Returns True when categories were loaded.
        """
        categories_path = self.output_dir / "vlm_categories.json"
        if not categories_path.exists():
            return False
        try:
            categories = json_load(open(categories_path, "r", encoding="utf-8"))
        except Exception:  # noqa: BLE001
            return False

        vlm_categories_dict: Dict[str, Any] = {}
        for item in categories:
            vlm_categories_dict[f"object_{item['id']}"] = {
                "name": item.get("name", ""),
                "color": item.get("color", ""),
                "category": item.get("category", ""),
            }
        self.scene.setdefault("vlm_categories_dict", {}).update(vlm_categories_dict)
        for obj_key, info in vlm_categories_dict.items():
            self.object_id_to_type[obj_key] = info["category"]
        return bool(vlm_categories_dict)
