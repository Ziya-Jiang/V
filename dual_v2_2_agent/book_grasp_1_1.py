"""Cart-book grasp that computes the highest solvable pre-grasp height.

``cart`` still starts from ``book_action_hyperparameters.csv``. Before the
arm moves, this entry asks the planner for the highest ``move_above_grasp``
height, on a 5 mm grid, that keeps every right-arm joint at least 5° inside
its soft limit. ``move_to_safe_entry`` supplies the outside-cart XY waypoint
and the configurable search ceiling. The entry and pre-grasp use the same
computed Z, so the second segment is horizontal and cannot sweep through the
book. If ``lift_object`` is above that height, it is lowered to the same value.

    python book_grasp_1_1.py
"""

import sys

_PREGRASP_HEIGHT_STEP_MM = 5.0


def highest_feasible_height_mm(low_mm, high_mm, is_feasible, step_mm=_PREGRASP_HEIGHT_STEP_MM):
    """Return the highest grid height in ``[low, high]`` accepted by ``is_feasible``.

    ``high`` is tested first. The search assumes that a passing height remains
    passing at every lower grid point, which holds for this vertical approach.
    """
    step = max(int(round(float(step_mm))), 1)
    low = int(round(float(low_mm)))
    high = int(round(float(high_mm)))
    if high <= low:
        return float(high) if is_feasible(float(high)) else None
    if is_feasible(float(high)):
        return float(high)
    if not is_feasible(float(low)):
        return None

    best = low
    while high - low > step:
        mid = low + ((high - low) // 2 // step) * step
        if mid <= low:
            mid = low + step
        if is_feasible(float(mid)):
            best = mid
            low = mid
        else:
            high = mid
    return float(best)


def _cart_pregrasp_candidates(step5, tasks, object_id):
    """Spine and yaw samples used by the cart-book preflight, without tilt."""
    grasp_task = next(
        (
            task for task in tasks
            if task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") == "move_down_to_grasp"
        ),
        None,
    )
    above_task = next(
        (
            task for task in tasks
            if task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") == "move_above_grasp"
        ),
        None,
    )
    if grasp_task is None or above_task is None:
        return None
    grasp_position = grasp_task.get("target_position") or []
    above_position = above_task.get("target_position") or []
    if len(grasp_position) < 3 or len(above_position) < 3:
        return None
    entry_task = next(
        (
            task for task in tasks
            if task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") == "move_to_safe_entry"
        ),
        None,
    )
    entry_position = (
        [] if entry_task is None else entry_task.get("target_position") or []
    )
    lift_task = next(
        (
            task for task in tasks
            if task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") == "lift_object"
        ),
        None,
    )
    lift_position = (
        [] if lift_task is None else lift_task.get("target_position") or []
    )
    place_task = next(
        (
            task for task in tasks
            if task.get("object") == object_id
            and task.get("arm") == "right"
            and task.get("action") == "move_to_target"
        ),
        None,
    )
    place_position = (
        [] if place_task is None else place_task.get("target_position") or []
    )

    detected_yaw = next(
        (
            float(task["book_grasp_yaw_deg"])
            for task in tasks
            if task.get("object") == object_id and "book_grasp_yaw_deg" in task
        ),
        None,
    )
    if detected_yaw is None:
        yaw_candidates = step5.RIGHT_CART_BOOK_GRASP_YAW_CANDIDATES_DEG
    else:
        opposite_yaw = ((detected_yaw + 360.0) % 360.0) - 180.0
        yaw_candidates = (detected_yaw, opposite_yaw)

    spine_xy = next(
        (
            task.get("book_spine_xy")
            for task in tasks
            if task.get("object") == object_id and task.get("book_spine_xy")
        ),
        None,
    )
    spine_length = next(
        (
            task.get("book_length_mm")
            for task in tasks
            if task.get("object") == object_id and task.get("book_length_mm") is not None
        ),
        None,
    )
    if spine_xy is None or spine_length is None:
        spine_offsets = [None]
    else:
        from book_grasp import spine_sample_offsets_mm

        spine_offsets = spine_sample_offsets_mm(spine_xy, spine_length)

    import numpy as np

    grasp_xy = np.asarray(grasp_position[:2], dtype=float)
    candidates = []
    for spine_offset in spine_offsets:
        if spine_offset is None:
            offset = np.zeros(2)
        else:
            offset = np.asarray(spine_offset, dtype=float).reshape(2)
        xy = grasp_xy + offset
        for yaw_deg in yaw_candidates:
            rotation = step5.right_bowl_vertical_pick_rotation(float(yaw_deg))
            candidates.append((xy, offset, float(yaw_deg), rotation))
    return {
        "grasp_z_mm": float(grasp_position[2]),
        "requested_z_mm": float(above_position[2]),
        "search_ceiling_z_mm": (
            float(entry_position[2])
            if len(entry_position) >= 3
            else float(above_position[2])
        ),
        "lift_z_mm": (
            float(lift_position[2])
            if len(lift_position) >= 3
            else float(above_position[2])
        ),
        "entry_xy": (
            np.asarray(entry_position[:2], dtype=float)
            if len(entry_position) >= 2
            else None
        ),
        "place_delta_xy_mm": (
            np.asarray(place_position[:2], dtype=float) - grasp_xy
            if len(place_position) >= 2
            else None
        ),
        "candidates": candidates,
    }


def _pose_is_within_soft_limits(
    step5, robot, object_id, xy, rotation, z_mm, preview_cache
):
    pose = [float(xy[0]), float(xy[1]), float(z_mm), *rotation]
    cache_key = (
        "max_pregrasp",
        round(float(xy[0]), 3),
        round(float(xy[1]), 3),
        round(float(z_mm), 3),
        tuple(round(float(value), 3) for value in rotation),
    )
    success, _, final_joint = step5._preview_right_bowl_pose(
        robot,
        pose,
        (
            f"max_pregrasp:{object_id}:"
            f"xy={float(xy[0]):.1f},{float(xy[1]):.1f}:z={float(z_mm):.1f}"
        ),
        preview_cache=preview_cache,
        cache_key=cache_key,
    )
    if not success or final_joint is None:
        return False
    return step5.right_joint_limit_risk(final_joint) is not None


def maximum_cart_pregrasp_height_mm(step5, robot, tasks, object_id):
    """Highest pre-grasp Z that at least one cart candidate can solve."""
    sampled = _cart_pregrasp_candidates(step5, tasks, object_id)
    if sampled is None:
        return None
    search_ceiling_z = sampled["search_ceiling_z_mm"]
    minimum_pregrasp_z = sampled["requested_z_mm"]
    grasp_z = sampled["grasp_z_mm"]
    lift_z = sampled["lift_z_mm"]
    entry_xy = sampled["entry_xy"]
    place_delta_xy = sampled["place_delta_xy_mm"]
    preview_cache = {}
    best_z = None
    for xy, _offset, _yaw, rotation in sampled["candidates"]:
        # A height is useful only for a candidate that can also complete the
        # fixed descend stage.  Lift is checked below at its effective Z.
        # Without this gate, an otherwise
        # unusable spine/yaw candidate can advertise a high approach Z that
        # every complete sequence later rejects.
        fixed_stages_are_feasible = _pose_is_within_soft_limits(
            step5,
            robot,
            object_id,
            xy,
            rotation,
            grasp_z,
            preview_cache,
        )
        if not fixed_stages_are_feasible:
            continue
        if place_delta_xy is not None:
            place_xy = xy + place_delta_xy
            if not _pose_is_within_soft_limits(
                step5,
                robot,
                object_id,
                place_xy,
                rotation,
                grasp_z,
                preview_cache,
            ):
                continue

        def route_is_feasible(height, xy=xy, rotation=rotation):
            if not _pose_is_within_soft_limits(
                step5,
                robot,
                object_id,
                xy,
                rotation,
                height,
                preview_cache,
            ):
                return False
            if (
                entry_xy is not None
                and not _pose_is_within_soft_limits(
                    step5,
                    robot,
                    object_id,
                    entry_xy,
                    rotation,
                    height,
                    preview_cache,
                )
            ):
                return False
            # lift_object is lowered to the selected approach height when its
            # CSV value is higher, so preview the effective value here.
            if not _pose_is_within_soft_limits(
                step5,
                robot,
                object_id,
                xy,
                rotation,
                min(lift_z, height),
                preview_cache,
            ):
                return False
            if place_delta_xy is None:
                return True
            place_xy = xy + place_delta_xy
            return _pose_is_within_soft_limits(
                step5,
                robot,
                object_id,
                place_xy,
                rotation,
                height,
                preview_cache,
            )

        z_mm = highest_feasible_height_mm(
            minimum_pregrasp_z,
            search_ceiling_z,
            route_is_feasible,
        )
        if z_mm is None:
            continue
        if best_z is None or z_mm > best_z:
            best_z = z_mm
        if abs(best_z - search_ceiling_z) <= 1e-6:
            return best_z
    return best_z


_CART_HIGH_Z_ACTIONS = {
    "lift_object",
    "move_to_target",
    "move_up_after_release",
}


def _lower_lift_to_pregrasp(tasks, object_id, z_mm):
    for task in tasks:
        if (
            task.get("object") != object_id
            or task.get("arm") != "right"
            or task.get("action") not in _CART_HIGH_Z_ACTIONS
        ):
            continue
        position = task.get("target_position")
        if not position or len(position) < 3:
            continue
        current_z = float(position[2])
        if current_z <= z_mm + 1e-6:
            continue
        updated = list(position)
        updated[2] = z_mm
        task["target_position"] = updated
        print(
            f"{task.get('action')} 高度高于最大可解预抓高度，一并降低："
            f"Z={current_z:.1f}mm -> {z_mm:.1f}mm。"
        )


def _synchronize_safe_entry_height(tasks, object_id, z_mm):
    for task in tasks:
        if (
            task.get("object") != object_id
            or task.get("arm") != "right"
            or task.get("action") != "move_to_safe_entry"
        ):
            continue
        position = task.get("target_position")
        if not position or len(position) < 3:
            continue
        updated = list(position)
        updated[2] = z_mm
        task["target_position"] = updated


def _run_full_preflight_at_height(
    step5,
    original,
    robot,
    tasks,
    object_id,
    z_mm,
    *,
    rerun_logger=None,
):
    """Run the authoritative candidate preflight at one approach height."""
    _synchronize_safe_entry_height(tasks, object_id, z_mm)
    _lower_lift_to_pregrasp(tasks, object_id, z_mm)

    original_load = step5.load_book_action_hyperparameters

    def load_with_pregrasp_height(*args, **kwargs):
        offsets = original_load(*args, **kwargs)
        copied = {key: dict(value) for key, value in offsets.items()}
        row = copied.get(("book_cart", "move_above_grasp"))
        if row is not None:
            row["target_z_base_mm"] = z_mm
        for action in ("lift_object", "move_to_target", "move_up_after_release"):
            row = copied.get(("book_cart", action))
            if row is not None and float(row["target_z_base_mm"]) > z_mm:
                row["target_z_base_mm"] = z_mm
        return copied

    step5.load_book_action_hyperparameters = load_with_pregrasp_height
    try:
        return original(
            robot, tasks, object_id, rerun_logger=rerun_logger
        )
    finally:
        step5.load_book_action_hyperparameters = original_load


def _select_with_maximum_pregrasp_height(step5, original):
    def select_safest_right_bowl_candidate(
        robot, tasks, object_id, *, rerun_logger=None
    ):
        is_cart_book = any(
            task.get("object") == object_id
            and task.get("book_grasp_mode") == "cart"
            for task in tasks
        )
        if not is_cart_book:
            return original(
                robot, tasks, object_id, rerun_logger=rerun_logger
            )

        sampled = _cart_pregrasp_candidates(step5, tasks, object_id)
        requested_z = None if sampled is None else sampled["requested_z_mm"]
        search_ceiling_z = (
            None if sampled is None else sampled["search_ceiling_z_mm"]
        )
        z_mm = maximum_cart_pregrasp_height_mm(step5, robot, tasks, object_id)
        if z_mm is None or requested_z is None or search_ceiling_z is None:
            print("没有找到不超出软限位的预抓高度，继续按表内高度预检。")
            return original(
                robot, tasks, object_id, rerun_logger=rerun_logger
            )
        print(
            "自动预抓高度："
            f"原预抓 {requested_z:.1f}mm，"
            f"CSV 搜索上限 {search_ceiling_z:.1f}mm，"
            f"不超出软限位的最大高度 {z_mm:.1f}mm。"
        )
        candidate_z_mm = float(z_mm)
        minimum_z_mm = float(requested_z)
        while candidate_z_mm >= minimum_z_mm - 1e-6:
            if _run_full_preflight_at_height(
                step5,
                original,
                robot,
                tasks,
                object_id,
                candidate_z_mm,
                rerun_logger=rerun_logger,
            ):
                if candidate_z_mm < z_mm - 1e-6:
                    print(
                        "最大高度的完整复检出现不同 IK 分支；"
                        f"最终采用完整预检通过的最高高度 "
                        f"Z={candidate_z_mm:.1f}mm。"
                    )
                return True
            next_z_mm = candidate_z_mm - _PREGRASP_HEIGHT_STEP_MM
            if next_z_mm < minimum_z_mm - 1e-6:
                break
            print(
                f"Z={candidate_z_mm:.1f}mm 未通过完整候选预检；"
                f"降低到 Z={next_z_mm:.1f}mm 后重新检查。"
            )
            candidate_z_mm = next_z_mm
        print(
            f"从 Z={z_mm:.1f}mm 降到表内最低预抓高度 "
            f"Z={minimum_z_mm:.1f}mm 后仍无完整可行路径；拒绝执行。"
        )
        return False

    return select_safest_right_bowl_candidate


def install_maximum_pregrasp_height():
    """Replace the cart preflight so it uses the computed maximum height."""
    import step5_execute as step5

    step5.select_safest_right_bowl_candidate = _select_with_maximum_pregrasp_height(
        step5,
        step5.select_safest_right_bowl_candidate,
    )


def build_book_tasks(task_type, vlm_categories):
    """Build the one-book task after the operator chooses shelf, cart, or cart+."""
    if task_type not in ("shelf", "cart", "cart+"):
        return None
    from book_grasp import set_book_grasp_mode

    set_book_grasp_mode(task_type)
    book_ids = [
        item["id"]
        for item in vlm_categories
        if str(item.get("category", "")).lower() == "book"
    ]
    if not book_ids:
        raise ValueError("场景里没有 category 为 book 的物体。")
    book_key = f"object_{book_ids[0]}"
    return [(book_key, book_key, 0, 0, 0)]


def _use_robot_camera(argv):
    """Read the robot head camera unless the operator asks for a local one."""
    if "--local-camera" in argv:
        return [item for item in argv if item != "--local-camera"]
    if "--remote-rgbd" in argv:
        return argv
    return ["--remote-rgbd", *argv]


def main():
    import dual_v2_2_agent_pcl as agent

    install_maximum_pregrasp_height()
    sys.argv = [sys.argv[0], *_use_robot_camera(sys.argv[1:])]
    agent.EXTRA_TASK_TYPES = ["shelf", "cart", "cart+"]
    agent.EXTRA_TASK_BUILDER = build_book_tasks
    agent.EXCLUSIVE_EXTENSION_MODE = True
    agent.GROUNDING_BOX_THRESHOLD_OVERRIDE = 0.18
    agent.GROUNDING_TEXT_THRESHOLD_OVERRIDE = 0.20
    agent.EXTRA_SCENE_OBJECTS = [
        {
            "name": "book",
            "color": "",
            "category": "book",
        }
    ]
    print(
        "机器人请用启动参数 3：驾驶舱不打开头部相机，Orbbec 图像传到本机。\n"
        "本机把抓书目标发给规划器，由规划器交给远程机器人执行。\n"
        "输入 shelf：右手从侧面抓架子上立着的书。\n"
        "输入 cart：车上立书。预抓高度取表内高度和软限位允许的最大高度中的较低者；"
        "夹爪竖直向下，两指沿点云薄边开合。提起后向左 6cm 放到左侧格子，不复位。\n"
        "输入 cart+：车上立书，从前向后沿基座 +X 抓住书脊（不是从上往下），"
        "提起后向左 6cm 放到左侧格子，不复位。",
        flush=True,
    )
    agent.main()


if __name__ == "__main__":
    main()
