# This demo runs camera/UDP workers in addition to the main control loop.
# Matplotlib otherwise selects TkAgg when a desktop session is present; Tk
# images created by worker-owned debug plots can then be finalized outside the
# Tk main thread and emit "main thread is not in main loop".  All debug output
# in this pipeline is written to files/Rerun, so a non-interactive backend is
# the correct one.
import os
os.environ["MPLBACKEND"] = os.environ.get("ROBOTRAM_MPLBACKEND", "Agg")

import atexit
import copy
import gc
import time
import sys
import numpy as np
import torch  
from os.path import join as pjoin
import json
import argparse
import re
import logging
from pathlib import Path

# Resolve dependencies from the RobotRAM tree copied beside this entrypoint.
AGENT_ROOT = Path(__file__).resolve().parent
ROBOTRAM_ROOT = Path(
    os.environ.get("ROBOTRAM_ROOT", AGENT_ROOT / "RobotRAM")
).expanduser().resolve()
ROBOTRAM_SCRIPTS_ROOT = ROBOTRAM_ROOT / "scripts"
VLM_DEMO_ROOT = Path(
    os.environ.get("VLM_DEMO_ROOT", AGENT_ROOT)
).expanduser().resolve()

required_robotram_paths = (
    ROBOTRAM_ROOT / "utils",
    ROBOTRAM_ROOT / "controllor",
    ROBOTRAM_ROOT / "visions",
    ROBOTRAM_SCRIPTS_ROOT,
)
missing_robotram_paths = [
    path for path in required_robotram_paths if not path.exists()
]
if missing_robotram_paths:
    raise RuntimeError(
        "RobotRAM dependency tree is incomplete:\n"
        + "\n".join(f"  - {path}" for path in missing_robotram_paths)
    )

os.environ.setdefault("ROBOTRAM_ROOT", str(ROBOTRAM_ROOT))
sys.path[:0] = [str(ROBOTRAM_SCRIPTS_ROOT), str(ROBOTRAM_ROOT)]

# Keep the legacy variable names for downstream stage functions.
repo_path = str(ROBOTRAM_ROOT)
parent_path = str(VLM_DEMO_ROOT)
DEFAULT_RERUN_WORKSPACE_CACHE = (
    VLM_DEMO_ROOT
    / "collision_classifier"
    / "point_cloud_data"
    / "dual_v2_2"
    / "workspace_slice_cache.npz"
)

from utils.other_utils import set_random_seeds
from controllor.hkstp_arm import RobotArm
from controllor.extern_camera import (
    ExternCameraCallbacks,
    OrbbecCamera,
    RealSense455Camera,
)
from visions.grounding.dino_grounding import DinoGrounding
from visions.ram.ram import RAMIO, RAMTemplate, RAMModel
from visions.vggt.load_model import load_vggt_model

from step1_grounding import grounding_create_output_dir, grounding_main
from step2_ram import ram_main
from step3_read_object_info import read_object_main
from step4_manual_planning import planning_main
from step5_execute import (
    OPERATOR_RESET_REQUESTED,
    execute_main,
    load_object_categories,
)
from utils.head_camera_correction import (
    correct_head_camera_calibration,
    save_correction_outputs,
)
from utils.rerun_pipeline_logger import RerunPipelineLogger

sys.stdout.reconfigure(encoding='utf-8')

# UDP ports aligned with cuarm_curobo/tutorial/planner_to_panel.py get_dual_v2_2_setup()
LOCALHOST = "127.0.0.1"
TARGET_LOCAL_PORT = 10165   # RobotArm binds here, receives CuroboState from planner
TARGET_REMOTE_PORT = 10166  # RobotArm sends TargetCommand to planner
PANEL_LOCAL_PORT = 30241    # planner_to_panel binds here for panel state
PANEL_REMOTE_PORT = 30240   # planner_to_panel sends RemotePanelCommand to panel

# Set ROBOTRAM_ORBBEC_CALIBRATION_PATH to override this default without
# changing code. The file is the latest verified camera-to-robot-base result.
LATEST_ORBBEC_CALIBRATION_PATH = os.environ.get(
    "ROBOTRAM_ORBBEC_CALIBRATION_PATH",
    os.path.join(
        parent_path,
        "calibration",
        "session_demo",
        "base_to_camera",
        "results",
        "machine-readable results.json",
    ),
)
# Gemini 305 outputs left/right RGB only after switching its device preset.
# Override with ROBOTRAM_ORBBEC_PRESET when a depth+color run is required.
ORBBEC_PRESET = os.environ.get(
    "ROBOTRAM_ORBBEC_PRESET",
    "Dual Color Streams",
)


def load_orbbec_calibration(calibration_path):
    """Load RobotRAM-compatible K/R/T data exported by calibrate.py."""
    with open(calibration_path, "r", encoding="utf-8") as calibration_file:
        calibration = json.load(calibration_file)

    try:
        params = calibration["left"]
        camera_matrix = np.asarray(params["K"], dtype=float)
        rotation = np.asarray(params["R"], dtype=float)
        translation = np.asarray(params["T"], dtype=float).reshape(3, 1)
        distortion = np.asarray(params["D"], dtype=float)
        width = int(params["w"])
        height = int(params["h"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"Invalid Orbbec calibration file: {calibration_path}"
        ) from error

    if camera_matrix.shape != (3, 3) or rotation.shape != (3, 3):
        raise ValueError(
            f"Invalid K/R matrix shape in calibration file: {calibration_path}"
        )
    if not all(
        np.isfinite(value).all()
        for value in (camera_matrix, rotation, translation, distortion)
    ):
        raise ValueError(
            f"Calibration contains non-finite values: {calibration_path}"
        )
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise ValueError(
            f"Calibration focal lengths must be positive: {calibration_path}"
        )
    orthogonality_error = float(
        np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro")
    )
    rotation_determinant = float(np.linalg.det(rotation))
    if (
        orthogonality_error > 1e-3
        or abs(rotation_determinant - 1.0) > 1e-3
    ):
        raise ValueError(
            "Calibration R is not a valid rotation matrix "
            f"(orthogonality_error={orthogonality_error:.3e}, "
            f"det={rotation_determinant:.6f}): {calibration_path}"
        )
    if width <= 0 or height <= 0:
        raise ValueError(
            f"Invalid image resolution in calibration file: {calibration_path}"
        )

    transform_base_from_camera = np.eye(4, dtype=float)
    transform_base_from_camera[:3, :3] = rotation
    transform_base_from_camera[:3, 3] = translation.reshape(3)
    return {
        "path": calibration_path,
        "K": camera_matrix,
        "R": rotation,
        "T": translation,
        "D": distortion,
        "w": width,
        "h": height,
        "T_base_cam": transform_base_from_camera,
    }


def apply_orbbec_calibration(camera, calibration):
    """Make an OrbbecCamera use the supplied calibrated geometry."""
    camera.camparas_file = calibration["path"]
    camera.K = calibration["K"].copy()
    camera.R = calibration["R"].copy()
    camera.T = calibration["T"].copy()
    camera.D = calibration["D"].copy()
    camera.w = calibration["w"]
    camera.h = calibration["h"]


class ColoredFormatter(logging.Formatter):
    COLORS = {
        'INFO': '\033[94m',  # Blue
        'DEBUG': '\033[92m',  # Green
        'WARNING': '\033[93m',  # Yellow
        'ERROR': '\033[91m',  # Red
        'CRITICAL': '\033[95m',  # Magenta
    }
    RESET = '\033[0m'

    def format(self, record):
        color = self.COLORS.get(record.levelname, self.RESET)
        record.msg = f"{color}{record.msg}{self.RESET}"
        return super().format(record)

# 配置根 logger
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s',
    handlers=[logging.StreamHandler()]
)

# 获取根 logger 并设置自定义格式化器
logger = logging.getLogger()
logger.handlers[0].setFormatter(ColoredFormatter('%(levelname)s: %(message)s'))


def parse_rerun_args():
    parser = argparse.ArgumentParser(
        description="运行 SR/RAM 演示，并将中间图像、点云和规划结果记录到 Rerun。"
    )
    parser.add_argument(
        "--no-rerun",
        action="store_true",
        help="禁用 Rerun 可视化。",
    )
    parser.add_argument(
        "--no-rerun-viewer",
        action="store_true",
        help="仅写入 .rrd 录制文件，不自动打开 Rerun Viewer。",
    )
    parser.add_argument(
        "--rerun-recording",
        default=None,
        help="Rerun .rrd 保存路径；默认保存到当前 output 目录。",
    )
    parser.add_argument(
        "--rerun-max-points",
        type=int,
        default=100_000,
        help="每个点云写入 Rerun 的最大点数。",
    )
    parser.add_argument(
        "--rerun-workspace-cache",
        type=Path,
        default=DEFAULT_RERUN_WORKSPACE_CACHE,
        help="dual_v2_2 可达工作空间缓存；按每个规划点的 Z 高度生成二维切片。",
    )
    parser.add_argument(
        "--head-camera-correction",
        action="store_true",
        help=(
            "使用 VGGT 深度中的桌面平面自动修正头部相机俯仰外参；"
            "校正不可信时会在规划和执行前中止。"
        ),
    )
    parser.add_argument(
        "--assume-head-camera-calibrated",
        action="store_true",
        help=(
            "明确确认头部相机仍处于手眼标定姿态并使用原始外参；"
            "该选项与运行时校正互斥。"
        ),
    )
    parser.add_argument(
        "--head-camera-correction-only",
        action="store_true",
        help=(
            "只完成一次头部相机外参估计并保存结果，不进入 Step3、规划或机器人执行。"
        ),
    )
    parser.add_argument(
        "--head-camera-table-z-mm",
        type=float,
        default=0.0,
        help="机器人基座坐标系中的桌面 Z 高度，单位 mm，默认 0。",
    )
    parser.add_argument(
        "--head-camera-roi",
        type=float,
        nargs=4,
        default=(0.08, 0.92, 0.30, 0.95),
        help="桌面深度 ROI 的归一化 x_min x_max y_min y_max。",
    )
    parser.add_argument(
        "--head-camera-depth-range-m",
        type=float,
        nargs=2,
        default=(0.25, 2.0),
        help="参与桌面拟合的最小和最大深度，单位 m。",
    )
    parser.add_argument(
        "--head-camera-pitch-limit-deg",
        type=float,
        default=20.0,
        help="相对标定姿态允许搜索的最大俯仰变化，单位度。",
    )
    parser.add_argument(
        "--head-camera-pitch-axis-base",
        type=float,
        nargs=3,
        default=None,
        help=(
            "可选：机器人基座坐标系中的机械俯仰轴向量；"
            "未提供时根据桌面法向使用最小调平旋转。"
        ),
    )
    parser.add_argument(
        "--head-camera-pivot-base-mm",
        type=float,
        nargs=3,
        default=None,
        help="可选：机器人基座坐标系中的相机俯仰转轴点 X Y Z，单位 mm。",
    )
    parser.add_argument(
        "--head-camera-anchor",
        type=float,
        nargs=5,
        action="append",
        default=None,
        help=(
            "可重复指定已知标记点：图像 u v 和机器人基座 X Y Z(mm)；"
            "执行模式至少提供两个，以完整修正并交叉验证 XYZ 平移。"
        ),
    )
    parser.add_argument(
        "--head-camera-xy-offset-mm",
        type=float,
        nargs=2,
        default=(0.0, 0.0),
        help="可选：完成自动校正后附加的基座 X/Y 微调，单位 mm。",
    )
    args = parser.parse_args()
    camera_pose_modes = (
        int(args.head_camera_correction)
        + int(args.head_camera_correction_only)
        + int(args.assume_head_camera_calibrated)
    )
    if camera_pose_modes != 1:
        parser.error(
            "必须且只能选择一个相机外参模式："
            "--head-camera-correction、--head-camera-correction-only 或 "
            "--assume-head-camera-calibrated。"
        )
    if args.head_camera_correction:
        if args.head_camera_pitch_axis_base is None:
            parser.error(
                "机器人执行模式必须通过 --head-camera-pitch-axis-base "
                "提供已验证的机械俯仰轴；可先运行 "
                "--head-camera-correction-only 获取诊断值。"
            )
        anchor_count = len(args.head_camera_anchor or [])
        if anchor_count == 1:
            parser.error(
                "机器人执行模式不能使用单个 --head-camera-anchor；"
                "请不使用 anchor 并依赖已验证的 pivot，或至少提供两个不同 anchor。"
            )
        if (
            args.head_camera_pivot_base_mm is None
            and anchor_count < 2
        ):
            parser.error(
                "机器人执行模式必须提供 --head-camera-pivot-base-mm，"
                "或至少两个 --head-camera-anchor，以修正并交叉验证 XYZ 平移。"
            )
    return args


def close_camera_resources(*named_cameras):
    """Release every camera SDK stream before the Python process exits."""
    for camera_name, camera in named_cameras:
        if camera is None:
            continue
        close = getattr(camera, "close", None)
        if not callable(close):
            continue
        try:
            close()
            print(f"已释放相机资源: {camera_name}")
        except Exception as error:
            print(f"关闭相机 {camera_name} 时出现异常: {error}")


_runtime_cleanup_done = False


def cleanup_runtime_resources():
    """Close native resources once, in a deterministic shutdown order."""
    global _runtime_cleanup_done
    if _runtime_cleanup_done:
        return
    _runtime_cleanup_done = True

    resources = globals()
    rerun_logger = resources.get("rerun_logger")
    if rerun_logger is not None:
        try:
            rerun_logger.close()
            print("已断开 Rerun 记录流")
        except Exception as error:
            print(f"关闭 Rerun 时出现异常: {error}")

    close_camera_resources(
        ("Orbbec grounding", resources.get("extern_camera")),
        ("RealSense RAM", resources.get("extern_camera2")),
        ("Orbbec RAM", resources.get("extern_camera_class")),
    )

    robot = resources.get("robot")
    if robot is not None:
        try:
            robot.shutdown()
        except Exception as error:
            print(f"关闭机器人通信时出现异常: {error}")

    # Drop the final Python references after each SDK has synchronously stopped
    # its pipeline. This forces native destructors to run now, instead of in an
    # undefined order during interpreter teardown.
    for resource_name in (
        "rerun_logger",
        "extern_camera",
        "extern_camera2",
        "extern_camera_class",
        "robot",
    ):
        resources[resource_name] = None
    gc.collect()


if __name__ == "__main__":
    args = parse_rerun_args()
    if args.rerun_max_points <= 0:
        raise ValueError("--rerun-max-points 必须大于 0。")

    enable_logging = True
    enable_debug = True
    if not enable_logging: logger = None
    set_random_seeds(2) 

    robot = None
    extern_camera = None
    extern_camera2 = None
    extern_camera_class = None
    rerun_logger = None
    atexit.register(cleanup_runtime_resources)

    # Robot Arm
    robot = RobotArm(
        target_host=LOCALHOST,
        target_local_port=TARGET_LOCAL_PORT,
        target_remote_port=TARGET_REMOTE_PORT,
    )
    if logger:
        logger.info(
            f"UDP target ports: local={TARGET_LOCAL_PORT}, remote={TARGET_REMOTE_PORT} "
            f"(planner_to_panel panel ports: {PANEL_LOCAL_PORT}/{PANEL_REMOTE_PORT})"
        )
    robot.initialize_target_command()

    orbbec_calibration = load_orbbec_calibration(
        LATEST_ORBBEC_CALIBRATION_PATH
    )
    # Keep the hand-eye result immutable. Every frame estimates head motion
    # relative to this reference instead of accumulating corrections.
    head_camera_reference_calibration = copy.deepcopy(orbbec_calibration)
    if logger:
        logger.info(
            "Loaded latest Orbbec calibration: %s (%dx%d, T_base_cam=%s mm)",
            orbbec_calibration["path"],
            orbbec_calibration["w"],
            orbbec_calibration["h"],
            np.round(orbbec_calibration["T"].reshape(3), 2).tolist(),
        )

    vlm = None

    # cameras
    # extern_camera = ExternCameraCallbacks['ZED']()
    # SR/VGGT uses Orbbec left/right RGB plus the RealSense RGB image.  The
    # Dual Color Streams preset intentionally returns a zero depth placeholder;
    # step2_ram obtains metric depth from VGGT instead.
    extern_camera2 = RealSense455Camera(color_only=True, warmup_frames=5)
    atexit.unregister(extern_camera2.close)
    extern_camera = ExternCameraCallbacks['Orbbec'](
        preset_name=ORBBEC_PRESET,
        warmup_frames=5,
    )
    atexit.unregister(extern_camera.close)
    apply_orbbec_calibration(extern_camera, orbbec_calibration)
    if ORBBEC_PRESET == "Dual Color Streams" and not extern_camera.use_dual_color:
        raise RuntimeError(
            "Gemini 305 did not start dual RGB streams. "
            "Run RobotRAM/examples/test_orbbec_dual_color_vggt.py first."
        )
    if logger:
        logger.info(
            "Orbbec preset=%s, dual_rgb=%s",
            ORBBEC_PRESET,
            extern_camera.use_dual_color,
        )

    # visual grounding model
    vgm = DinoGrounding()

    # load vggt model
    vggt_device=torch.device("cuda:0")
    vggt_dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    vggt_model = load_vggt_model("facebook/VGGT-1B").to(vggt_device)

    # load RAM
    config_path = pjoin(repo_path, './visions/ram/assets/ram.json')
    RAM_Model = RAMModel(config_path)
    ObjectTemplate = RAMTemplate(config_path)
    RAM_IO = RAMIO()

    #prompt
    prompt = "clean the table"
    promp_name = re.sub(r'\W+', '_', prompt.lower())  # 将 args.prompt 转换为小写并替代所有空格为下划线，特殊符号也变为下划线
    if logger: logger.info(f"Prompt name: {promp_name}")

    #Create directories
    # output = "./output/scene_56_forkdiagonalcam"
    output = os.path.join(repo_path, "output", "scene_57")
    # output = "./output/dex_wineglass_01"
    # output = "./output/scene_59"

    output_dir, grounding_debug_dir = grounding_create_output_dir(prompt, output, logger)
    ram_debug_dir = debug_dir = os.path.join(output_dir, "ram")
    os.makedirs(debug_dir, exist_ok=True)
    cur_dir = repo_path
    if not args.no_rerun:
        recording_path = (
            Path(args.rerun_recording).expanduser().resolve()
            if args.rerun_recording
            else Path(output_dir) / "sr_demo_rerun.rrd"
        )
        rerun_logger = RerunPipelineLogger(
            application_id="robotram_sr_demo",
            recording_path=recording_path,
            spawn_viewer=not args.no_rerun_viewer,
            max_points=args.rerun_max_points,
        )
        atexit.unregister(rerun_logger.close)
        rerun_logger.send_default_blueprint()
        rerun_logger.begin_stage(
            "setup",
            "Loaded calibrated Orbbec camera and initialized SR/RAM demo",
        )
        rerun_logger.log_camera(
            c2w_mm=orbbec_calibration["T_base_cam"],
            intrinsic=orbbec_calibration["K"],
            resolution=(orbbec_calibration["w"], orbbec_calibration["h"]),
        )
        workspace_summary = rerun_logger.log_workspace_slice_cache(
            args.rerun_workspace_cache
        )
        rerun_logger.log_json_document(
            "setup/calibration",
            {
                "calibration_path": orbbec_calibration["path"],
                "orbbec_preset": ORBBEC_PRESET,
                "image_resolution": [
                    orbbec_calibration["w"],
                    orbbec_calibration["h"],
                ],
                "T_base_cam_mm": orbbec_calibration["T_base_cam"],
                "intrinsic_K": orbbec_calibration["K"],
            },
        )
        if logger:
            logger.info("Rerun recording: %s", recording_path)
            if workspace_summary is None:
                logger.warning(
                    "Rerun workspace cache not found: %s",
                    args.rerun_workspace_cache,
                )
            else:
                logger.info(
                    "Rerun workspace cache: thickness=%.3fm, cells=%s",
                    workspace_summary["slice_thickness_m"],
                    workspace_summary["cell_counts"],
                )

    ########################Preprocess for Grounding
    vlm_object_file = os.path.join(output_dir, "vlm_objects.json")
    with open(vlm_object_file, 'r') as f:
        related_objects = json.load(f)
        
    ########################Preprocess for Ram
    # load VLM category results
    vlm_category_json = pjoin(output_dir, 'vlm_categories.json')
    with open(vlm_category_json, 'r') as f:
        vlm_categories = json.load(f)

    # 转换为以object_id为key的字典
    vlm_categories_dict = {}
    for item in vlm_categories:
        obj_key = f"object_{item['id']}"
        vlm_categories_dict[obj_key] = {
            'name': item['name'],
            'color': item['color'], 
            'category': item['category']
        }
        
    # loading the camera parameters
    # extern_camera_class = ZEDCamera(is_warmup=False)
    extern_camera_class = OrbbecCamera(
        is_warmup=False,
        preset_name=ORBBEC_PRESET,
    )
    atexit.unregister(extern_camera_class.close)
    apply_orbbec_calibration(extern_camera_class, orbbec_calibration)

    head_camera_correction_report = (
        Path(output_dir) / "head_camera_correction.json"
    )
    head_camera_corrected_calibration = (
        Path(output_dir) / "head_camera_corrected_calibration.json"
    )

    def correct_head_camera_pose(depth_m, intrinsic, valid_mask):
        corrected_calibration, diagnostics = correct_head_camera_calibration(
            head_camera_reference_calibration,
            depth_m,
            intrinsic,
            valid_mask=valid_mask,
            table_z_mm=args.head_camera_table_z_mm,
            roi=args.head_camera_roi,
            depth_range_m=args.head_camera_depth_range_m,
            pitch_axis_base=args.head_camera_pitch_axis_base,
            pitch_limit_deg=args.head_camera_pitch_limit_deg,
            pivot_base_mm=args.head_camera_pivot_base_mm,
            anchors=args.head_camera_anchor,
            xy_offset_mm=args.head_camera_xy_offset_mm,
        )
        orbbec_calibration.update(corrected_calibration)
        apply_orbbec_calibration(extern_camera, orbbec_calibration)
        apply_orbbec_calibration(extern_camera_class, orbbec_calibration)
        save_correction_outputs(
            head_camera_reference_calibration["path"],
            corrected_calibration,
            diagnostics,
            head_camera_correction_report,
            head_camera_corrected_calibration,
        )
        if logger:
            logger.info(
                "头部相机外参已修正: pitch=%+.2f°, table tilt %.2f°→%.2f°, "
                "T=%s mm",
                diagnostics["delta_pitch_deg"],
                diagnostics["table_tilt_before_deg"],
                diagnostics["table_tilt_after_deg"],
                np.round(corrected_calibration["T"].reshape(3), 2).tolist(),
            )
            if diagnostics.get("warning"):
                logger.warning(diagnostics["warning"])
        if rerun_logger:
            rerun_logger.log_camera(
                c2w_mm=corrected_calibration["T_base_cam"],
                intrinsic=orbbec_calibration["K"],
                resolution=(
                    orbbec_calibration["w"],
                    orbbec_calibration["h"],
                ),
            )
            rerun_logger.log_json_document(
                "step2/head_camera_correction",
                diagnostics,
            )
        return diagnostics

    #######################Preprocess for Execute
    object_id_to_type = load_object_categories(vlm_categories)

    #######################Start loop
    total_planning_time = 0
    total_running_time = 0
    planning_cnt = 0
    running_cnt = 0
    valid_task_types = [
        'all', 'step', 'skip-fork', 'fork', 'bowl', 'cup', 'bread', '0',
        'wineglass',
    ]
    while True:
        try:
            task_type = input(
                "Enter 'all', 'step' (step-by-step, skips fork), "
                "'skip-fork', 'fork', 'bowl', 'cup', "
                "'bread' or 'wineglass' for task type: "
            )
            if task_type not in valid_task_types:
                print(f"Your input: {task_type}. Please input valid_task_types: {valid_task_types}")
                continue
            if task_type == "fork":
                predefined_tasks = [
                    # ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                    # ("object_3", "object_1", 0, 0, 0),        # 把面包放在碗里  
                    ("object_4", "object_1", 0, -120, 0),        # 把叉子放在盘子旁边
                    # ("object_2", "object_1", 100, 120, 0),     # 把杯子放在盘子旁边
                    # ("object_1", "object_0", 0, -120, 0),        # 把叉子放在盘子旁边
                ]
            elif task_type in ("all", "0"):
                predefined_tasks = [
                    ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                    ("object_3", "object_1", 0, 0, 0),        # 把面包放在碗里
                    ("object_4", "object_1", 0, -120, 0),     # 把叉子放在盘子旁边
                    ("object_2", "object_1", 100, 120, 0),    # 把杯子放在盘子旁边
                ]
            elif task_type in ("step", "skip-fork"):
                predefined_tasks = [
                    ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                    ("object_3", "object_1", 0, 0, 0),        # 把面包放在碗里
                    ("object_2", "object_1", 100, 120, 0),    # 把杯子放在盘子旁边
                ]
            elif task_type=="bowl":
                predefined_tasks = [
                    ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                    # ("object_3", "object_1", 0, 0, 0),        # 把面包放在碗里  
                    # ("object_4", "object_1", 0, -120, 0),        # 把叉子放在盘子旁边
                    # ("object_2", "object_1", 100, 120, 0),     # 把杯子放在盘子旁边
                    # ("object_1", "object_0", 0, -120, 0),        # 把叉子放在盘子旁边
                ]
            elif task_type=="cup":
                predefined_tasks = [
                    # ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                    # ("object_3", "object_1", 0, 0, 0),        # 把面包放在碗里  
                    # ("object_4", "object_1", 0, -120, 0),        # 把叉子放在盘子旁边
                    ("object_2", "object_1", 100, 120, 0),     # 把杯子放在盘子旁边
                    # ("object_1", "object_0", 0, -120, 0),        # 把叉子放在盘子旁边
                ]
            elif task_type=="bread":
                predefined_tasks = [
                    # ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                    ("object_3", "object_1", 0, 0, 0),        # 把面包放在碗里  
                    # ("object_4", "object_1", 0, -120, 0),        # 把叉子放在盘子旁边
                    # ("object_2", "object_1", 100, 120, 0),     # 把杯子放在盘子旁边
                    # ("object_1", "object_0", 0, -120, 0),        # 把叉子放在盘子旁边
                ]
            # elif task_type=="wineglass": #Left golden
            #     predefined_tasks = [
            #         # ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
            #         ("object_1", "object_0", 100, -120, 0),       # 把面包放在碗里  
            #         # ("object_4", "object_1", 0, -120, 0),        # 把叉子放在盘子旁边
            #         # ("object_2", "object_1", 100, 120, 0),     # 把杯子放在盘子旁边
            #         # ("object_1", "object_0", 0, -120, 0),        # 把叉子放在盘子旁边
            #     ]
            elif task_type=="wineglass": #Right black
                predefined_tasks = [
                    # ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                    ("object_1", "object_0", 100, 120, 0),       # 把面包放在碗里  
                    # ("object_4", "object_1", 0, -120, 0),        # 把叉子放在盘子旁边
                    # ("object_2", "object_1", 100, 120, 0),     # 把杯子放在盘子旁边
                    # ("object_1", "object_0", 0, -120, 0),        # 把叉子放在盘子旁边
                ]

            if task_type in ("step", "skip-fork"):
                active_related_objects = [
                    obj for obj in related_objects
                    if obj.get("category", "").lower() != "fork"
                ]
                active_vlm_categories = [
                    obj for obj in vlm_categories
                    if obj.get("category", "").lower() != "fork"
                ]
            else:
                active_related_objects = related_objects
                active_vlm_categories = vlm_categories

            if rerun_logger:
                rerun_logger.begin_run(task_type)
            t0 = time.time()
            #Step 1: Grounding
            if rerun_logger:
                rerun_logger.begin_stage(
                    "step1_grounding",
                    f"Capturing Orbbec RGB streams and grounding task={task_type}",
                )
            rgb_ext, left_image, right_image, depth_ext, masks, xyxys, class_ids = \
                grounding_main(prompt, output_dir, grounding_debug_dir, active_related_objects, robot, vlm, extern_camera,
                               vgm, only_grounding=True, logger=logger, enable_debug=enable_debug)
            if rerun_logger:
                rerun_logger.log_detection_artifacts(Path(output_dir))
            # print("Grounding time: ", time.time() - t0) #1.5s

            #Step 2: Ram
            if rerun_logger:
                rerun_logger.begin_stage(
                    "step2_ram",
                    "Running VGGT depth estimation and RAM pose estimation",
                )
            extern_camera_class.rgb_image = rgb_ext
            extern_camera_class.left_image = left_image
            extern_camera_class.right_image = right_image
            extern_camera_class.depth_map  = depth_ext
            objects_meta, rgb_ext2_original = ram_main(prompt, output_dir, ram_debug_dir, cur_dir, extern_camera2, extern_camera_class,
                    active_vlm_categories, masks, xyxys, class_ids, RAM_Model, ObjectTemplate, RAM_IO,
                    device=vggt_device, dtype=vggt_dtype, vggt_model=vggt_model, use_vggt_service=False, 
                    vggt_service_url="http://localhost:5001", logger=logger, enable_debug=enable_debug,
                    camera_pose_correction_callback=(
                        correct_head_camera_pose
                        if (
                            args.head_camera_correction
                            or args.head_camera_correction_only
                        )
                        else None
                    ),
                    stop_after_camera_pose_correction=args.head_camera_correction_only)
            if not args.head_camera_correction_only:
                objects_meta_path = Path(output_dir) / "ram_objects_meta.npy"
                np.save(objects_meta_path, objects_meta)
            if rerun_logger:
                rerun_logger.log_ram_artifacts(
                    Path(output_dir),
                    c2w_mm=orbbec_calibration["T_base_cam"],
                    coordinate_unit="m",
                )
            if args.head_camera_correction_only:
                print(
                    "头部相机外参校正完成，未进入规划和执行。\n"
                    f"诊断报告: {head_camera_correction_report}\n"
                    f"修正标定: {head_camera_corrected_calibration}"
                )
                break
            # print("Ram time: ", time.time() - t0) # almost equal planning time

            #Step 3: Read object info
            if rerun_logger:
                rerun_logger.begin_stage(
                    "step3_object_world",
                    "Transforming RAM object poses into robot-base coordinates",
                )
            objects_info = read_object_main(
                objects_meta,
                output_dir,
                enable_debug,
                c2w_matrix=orbbec_calibration["T_base_cam"],
                camera_pose_unit="m",
            )
            if rerun_logger:
                rerun_logger.log_json_document(
                    "step3/objects/world_metadata",
                    objects_info,
                )

            # yiyao_pose_input_path = 'output/dex_wineglass_01/wineglass2_grasp_pose.json'
            # R_obj_in_robot = objects_info['object_5']['center_rotation']  # @yiyao: get object rotation in robot base frame
            # p_obj_in_robot = objects_info['object_5']['center_position']  # @yiyao: get object position in robot base frame
            # p_obj_in_robot = np.array([p_obj_in_robot['x'], p_obj_in_robot['y'], p_obj_in_robot['z']]).astype(np.float32)

            # # let R_Obj_in_robot is a eye matrix for test (no rotation)
            # R_obj_in_robot = np.eye(3).astype(np.float32)

            # print('Grab object position in robot base frame: ', p_obj_in_robot)
            # print('Grab object rotation in robot base frame: ', R_obj_in_robot)
            # with open(yiyao_pose_input_path, 'r') as f:
            #     yiyao_grasp_poses = json.load(f)
            #     T_grasp_in_obj = yiyao_grasp_poses['position']
            #     T_grasp_in_obj = np.array([T_grasp_in_obj['x'], T_grasp_in_obj['y'], T_grasp_in_obj['z']]).astype(np.float32)
            #     R_grasp_in_world = np.array(yiyao_grasp_poses['rotation']).astype(np.float32)          
            
            #Step 4: Manual planning
            if rerun_logger:
                rerun_logger.begin_stage(
                    "step4_manual_plan",
                    "Generating robot tool target poses",
                )
            detailed_task_list = planning_main(rgb_ext, output_dir, objects_info, vlm_categories_dict, masks,
                        orbbec_calibration["K"], orbbec_calibration["R"], orbbec_calibration["T"],
                        logger=logger, predefined_tasks=predefined_tasks)
            if rerun_logger:
                rerun_logger.log_manual_planning_tasks(detailed_task_list)

            t1 = time.time()
            planning_time = t1 - t0
            total_planning_time += planning_time
            planning_cnt += 1
            print("Planning time: ", planning_time)

            # #Step 5: Execute
            # input()
            if rerun_logger:
                rerun_logger.begin_stage(
                    "step5_execution",
                    f"Executing {len(detailed_task_list)} planned robot actions",
                )
            object_id, success = execute_main(
                detailed_task_list,
                object_id_to_type,
                robot,
                rerun_logger=rerun_logger,
                pause_between_tasks=(task_type == "step"),
            )
            if rerun_logger:
                rerun_logger.log_json_document(
                    "step5/execution/result",
                    {
                        "success": bool(success),
                        "object_id": object_id,
                        "executed_task_count": len(detailed_task_list),
                    },
                )
            if object_id == OPERATOR_RESET_REQUESTED:
                print("当前任务序列已由操作员中止，返回任务类型选择。")
                continue
            if success == False:
                print(f"Task failed when executing object {object_id}, please check the scene and try again!")
                continue

            t2 = time.time()
            running_time = t2 - t0
            total_running_time += running_time
            running_cnt += 1
            print("Running time: ", running_time)
        except KeyboardInterrupt:
            print("Ctrl+C detected. Exiting...")
            break

    if planning_cnt > 0:
        print("Average planning time: ", total_planning_time / planning_cnt)
    else:
        print("Average planning time: N/A")
    if running_cnt > 0:
        print("Average running time: ", total_running_time / running_cnt)
    else:
        print("Average running time: N/A")
    print(f"Planning cnt: {planning_cnt}, running_cnt: {running_cnt}")
    cleanup_runtime_resources()



    

