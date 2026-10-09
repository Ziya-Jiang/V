#!/usr/bin/env python3
"""Estimate a head-camera correction from saved VGGT depth without a robot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from utils.head_camera_correction import (
    correct_head_camera_calibration,
    save_correction_outputs,
)


ROBOTRAM_ROOT = Path(__file__).resolve().parent
AGENT_ROOT = ROBOTRAM_ROOT.parent
DEFAULT_CALIBRATION = (
    AGENT_ROOT
    / "calibration"
    / "session_demo"
    / "base_to_camera"
    / "results"
    / "machine-readable results.json"
)


def load_calibration(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as source:
        payload = json.load(source)
    left = payload["left"]
    rotation = np.asarray(left["R"], dtype=float)
    translation = np.asarray(left["T"], dtype=float).reshape(3, 1)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return {
        "path": str(path),
        "R": rotation,
        "T": translation,
        "K": np.asarray(left["K"], dtype=float),
        "D": np.asarray(left["D"], dtype=float),
        "w": int(left["w"]),
        "h": int(left["h"]),
        "T_base_cam": transform,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "使用已保存的 vggt_depth_ext.npy 和 vggt_K.npy 拟合桌面，"
            "离线生成头部相机修正外参；不会连接或控制机器人。"
        )
    )
    parser.add_argument("--depth", type=Path, required=True, help="VGGT 米制深度 .npy")
    parser.add_argument("--intrinsic", type=Path, required=True, help="VGGT K 矩阵 .npy")
    parser.add_argument("--valid-mask", type=Path, default=None, help="可选有效深度掩码 .npy")
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help="原始 machine-readable results.json",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="输出目录；默认与深度文件相同。",
    )
    parser.add_argument("--table-z-mm", type=float, default=0.0)
    parser.add_argument(
        "--roi",
        type=float,
        nargs=4,
        default=(0.08, 0.92, 0.30, 0.95),
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
    )
    parser.add_argument(
        "--depth-range-m",
        type=float,
        nargs=2,
        default=(0.25, 2.0),
        metavar=("MIN", "MAX"),
    )
    parser.add_argument("--pitch-limit-deg", type=float, default=20.0)
    parser.add_argument(
        "--pitch-axis-base",
        type=float,
        nargs=3,
        default=None,
        metavar=("AX", "AY", "AZ"),
    )
    parser.add_argument(
        "--pivot-base-mm",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--anchor",
        type=float,
        nargs=5,
        action="append",
        default=None,
        metavar=("U", "V", "X_MM", "Y_MM", "Z_MM"),
    )
    parser.add_argument(
        "--xy-offset-mm",
        type=float,
        nargs=2,
        default=(0.0, 0.0),
        metavar=("X", "Y"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    calibration_path = args.calibration.expanduser().resolve()
    depth_path = args.depth.expanduser().resolve()
    intrinsic_path = args.intrinsic.expanduser().resolve()
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else depth_path.parent
    )

    calibration = load_calibration(calibration_path)
    depth = np.load(depth_path)
    intrinsic = np.load(intrinsic_path)
    valid_mask = (
        np.load(args.valid_mask.expanduser().resolve())
        if args.valid_mask is not None
        else None
    )
    corrected, diagnostics = correct_head_camera_calibration(
        calibration,
        depth,
        intrinsic,
        valid_mask=valid_mask,
        table_z_mm=args.table_z_mm,
        roi=args.roi,
        depth_range_m=args.depth_range_m,
        pitch_axis_base=args.pitch_axis_base,
        pitch_limit_deg=args.pitch_limit_deg,
        pivot_base_mm=args.pivot_base_mm,
        anchors=args.anchor,
        xy_offset_mm=args.xy_offset_mm,
    )

    report_path = output_dir / "head_camera_correction.json"
    corrected_path = output_dir / "head_camera_corrected_calibration.json"
    save_correction_outputs(
        calibration_path,
        corrected,
        diagnostics,
        report_path,
        corrected_path,
    )
    print(
        f"调平角度: {diagnostics['delta_pitch_deg']:+.3f}°\n"
        f"桌面倾角: {diagnostics['table_tilt_before_deg']:.3f}° -> "
        f"{diagnostics['table_tilt_after_deg']:.3f}°\n"
        f"平面内点率: {diagnostics['plane_inlier_ratio']:.3f}\n"
        f"诊断报告: {report_path}\n"
        f"修正标定: {corrected_path}"
    )
    if diagnostics.get("warning"):
        print(f"警告: {diagnostics['warning']}")


if __name__ == "__main__":
    main()
