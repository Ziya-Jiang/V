"""Grasp a book with the right-arm parallel gripper.

``shelf`` takes one standing book from the side. ``cart`` takes the nearest
standing book on the cart from above, using the same vertical-down gripper
pose and stage offsets as bowl in ``dual_v2_2_agent_pcl.py``.

The head camera stays on the robot. Start the robot with cockpit parameter 3
so the cockpit does not open the Orbbec; that parameter publishes RGB-D for
this machine. Commands still go to the local planner, which drives the robot:

    python book_grasp.py
"""

import sys

import dual_v2_2_agent_pcl as agent


def build_book_tasks(task_type, vlm_categories):
    """Build the one-book task after the operator chooses shelf or cart."""
    if task_type not in ("shelf", "cart"):
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
    sys.argv = [sys.argv[0], *_use_robot_camera(sys.argv[1:])]
    agent.EXTRA_TASK_TYPES = ["shelf", "cart"]
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
        "输入 cart：车上立书，与抓碗相同——上方接近、竖直向下抓取、原地提起；"
        "夹爪竖直向下，两指沿点云薄边开合。",
        flush=True,
    )
    agent.main()


if __name__ == "__main__":
    main()
