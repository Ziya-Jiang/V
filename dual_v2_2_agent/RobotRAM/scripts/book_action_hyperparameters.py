"""Hot-reloaded cart/shelf book stage offsets."""

import csv
import math
import os
from functools import lru_cache
from pathlib import Path

DEFAULT_BOOK_ACTION_HYPERPARAMETERS_PATH = (
    Path(__file__).resolve().parents[2] / "book_action_hyperparameters.csv"
)
BOOK_SHELF_STAGES = {
    "move_above_grasp",
    "move_down_to_grasp",
    "close_gripper",
    "lift_object",
    "move_to_target",
    "move_down_to_place",
    "open_gripper",
    "move_up_after_release",
}
BOOK_CART_STAGES = {
    "move_to_safe_entry",
    "move_above_grasp",
    "move_down_to_grasp",
    "close_gripper",
    "lift_object",
    "move_to_target",
    "move_down_to_place",
    "open_gripper",
    "move_up_after_release",
}
BOOK_CART_PLUS_STAGES = {
    "move_above_grasp",
    "move_down_to_grasp",
    "close_gripper",
    "lift_object",
    "move_to_target",
    "move_down_to_place",
    "open_gripper",
    "move_up_after_release",
}
REQUIRED_BOOK_OFFSET_KEYS = {
    ("book_shelf", stage) for stage in BOOK_SHELF_STAGES
} | {
    ("book_cart", stage) for stage in BOOK_CART_STAGES
} | {
    ("book_cart_plus", stage) for stage in BOOK_CART_PLUS_STAGES
}
VALID_REFERENCES = {"computed_grasp", "object_center", "robot_base"}


@lru_cache(maxsize=4)
def _load_book_action_hyperparameters_cached(csv_path, mtime_ns, file_size):
    del mtime_ns, file_size
    offsets = {}
    with open(csv_path, newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required_columns = {
            "object",
            "stage",
            "reference",
            "offset_x_mm",
            "offset_y_mm",
            "target_z_base_mm",
            "max_z_step_mm",
        }
        missing_columns = required_columns.difference(reader.fieldnames or ())
        if missing_columns:
            raise ValueError(
                f"书本动作 CSV 缺少列: {sorted(missing_columns)} ({csv_path})"
            )
        for row_number, row in enumerate(reader, start=2):
            key = (
                (row["object"] or "").strip().lower(),
                (row["stage"] or "").strip(),
            )
            if not all(key) or key in offsets:
                raise ValueError(
                    f"书本动作 CSV 第 {row_number} 行对象/阶段为空或重复: {key}"
                )
            reference = (row["reference"] or "").strip()
            if reference not in VALID_REFERENCES:
                raise ValueError(
                    f"书本动作 CSV 第 {row_number} 行 reference 无效: {reference}"
                )
            values = [
                float(row["offset_x_mm"]),
                float(row["offset_y_mm"]),
                float(row["target_z_base_mm"]),
                float(row["max_z_step_mm"]),
            ]
            if not all(math.isfinite(value) for value in values):
                raise ValueError(
                    f"书本动作 CSV 第 {row_number} 行包含非有限数值"
                )
            offsets[key] = {
                "reference": reference,
                "offset_x_mm": values[0],
                "offset_y_mm": values[1],
                "target_z_base_mm": values[2],
                "max_z_step_mm": values[3],
                "note": (row.get("note") or "").strip(),
            }

    missing_keys = REQUIRED_BOOK_OFFSET_KEYS.difference(offsets)
    if missing_keys:
        raise ValueError(
            "书本动作 CSV 缺少必要对象阶段: "
            + ", ".join(f"{obj}/{stage}" for obj, stage in sorted(missing_keys))
        )
    for action in (
        "move_down_to_grasp",
        "lift_object",
        "move_down_to_place",
        "move_up_after_release",
    ):
        step_mm = offsets[("book_cart", action)]["max_z_step_mm"]
        if step_mm <= 0:
            raise ValueError(
                f"book_cart/{action} 的 max_z_step_mm 必须大于 0"
            )
    return offsets


def load_book_action_hyperparameters(csv_path=None):
    """Load the CSV again whenever its mtime or size changes."""
    configured_path = csv_path or os.environ.get(
        "ROBOTRAM_BOOK_ACTION_HYPERPARAMETERS_PATH",
        DEFAULT_BOOK_ACTION_HYPERPARAMETERS_PATH,
    )
    resolved_path = Path(configured_path).expanduser().resolve()
    stat = resolved_path.stat()
    return _load_book_action_hyperparameters_cached(
        str(resolved_path), stat.st_mtime_ns, stat.st_size
    )


def _book_mode_csv_object(mode):
    normalized = str(mode).lower()
    if normalized == "cart+":
        return "book_cart_plus"
    return f"book_{normalized}"


def apply_book_action_pose(
    parameters,
    mode,
    action,
    position,
    rotation,
    *,
    object_center=None,
):
    """Apply one cart/shelf book row using its declared reference."""
    normalized_mode = str(mode).lower()
    if normalized_mode not in {"cart", "cart+", "shelf"}:
        raise ValueError(f"未知书本抓取模式: {mode}")
    key = (_book_mode_csv_object(normalized_mode), str(action))
    config = parameters[key]
    if config["reference"] == "robot_base":
        adjusted_position = [
            config["offset_x_mm"],
            config["offset_y_mm"],
            config["target_z_base_mm"],
        ]
    elif config["reference"] == "object_center":
        if not isinstance(object_center, dict):
            raise ValueError(
                f"{key[0]}/{action} 使用 object_center，但没有对象中心"
            )
        adjusted_position = [
            float(object_center["x"]) + config["offset_x_mm"],
            float(object_center["y"]) + config["offset_y_mm"],
            config["target_z_base_mm"],
        ]
    else:
        adjusted_position = [
            float(position[0]) + config["offset_x_mm"],
            float(position[1]) + config["offset_y_mm"],
            float(position[2]) + config["target_z_base_mm"],
        ]
    if normalized_mode in {"shelf", "cart+"} and config["reference"] != "computed_grasp":
        raise ValueError(
            f"{key[0]}/{action} 在 book_action_hyperparameters 中应使用 computed_grasp"
        )
    expected_cart_reference = (
        "robot_base" if action == "move_to_safe_entry" else "object_center"
    )
    if (
        normalized_mode == "cart"
        and config["reference"] != expected_cart_reference
    ):
        raise ValueError(
            f"book_cart/{action} 在 book_action_hyperparameters 中应使用 "
            f"{expected_cart_reference}"
        )
    return adjusted_position, list(rotation[:3])
