import sys
import os 
import numpy as np
import json
import argparse
import logging

sys.stdout.reconfigure(encoding='utf-8')
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

def parse_args():
    parser = argparse.ArgumentParser(description="parse constraints from VLM subtasks")
    parser.add_argument("--prompt", type=str, default="clean the table", help="Text prompt describing the task")
    parser.add_argument("--output", type=str, default="./output/scene_01", help="Directory to save outputs")
    parser.add_argument("--visualize", action="store_true", help="Run path visualization after execution")
    parser.add_argument("--visualize_only", type=str, help="Only run visualization with specified detailed_tasks.json file")
    parser.add_argument("--add_rotations", type=str, help="Add rotation angles to existing detailed_subtasks.json file")
    parser.add_argument("--scene_data", type=str, help="Directory containing scene data (depth_ext.npy, rgb_ext.png, vggt_K.npy)")
    parser.add_argument("--task_generation_method", type=str, choices=["vlm", "manual"], default="vlm", 
                       help="Method to generate rough tasks: 'vlm' (use GPT4v) or 'manual' (use task_generator function)")
    return parser.parse_args()


def load_objects_info(output_dir):
    """加载物体信息"""
    objects_file = os.path.join(output_dir, "objects_info_world.json")
    if not os.path.exists(objects_file):
        logger.error(f"物体信息文件不存在: {objects_file}")
        return None
    
    with open(objects_file, 'r', encoding='utf-8') as f:
        objects_info = json.load(f)
    
    logger.info(f"成功加载 {len(objects_info)} 个物体的信息")
    return objects_info

def load_vlm_categories(output_dir):
    """加载VLM识别的物体类别和名称"""
    # categories_file = "test_outputs/test_task01_long02/vlm_categories.json"
    categories_file = os.path.join(output_dir, "vlm_categories.json")
    if not os.path.exists(categories_file):
        logger.error(f"VLM类别文件不存在: {categories_file}")
        return None
    
    with open(categories_file, 'r', encoding='utf-8') as f:
        categories = json.load(f)
    
    # 转换为以object_id为key的字典
    categories_dict = {}
    for item in categories:
        obj_key = f"object_{item['id']}"
        categories_dict[obj_key] = {
            'name': item['name'],
            'color': item['color'], 
            'category': item['category']
        }
    
    logger.info(f"成功加载 {len(categories)} 个物体的VLM类别信息")
    return categories_dict

def get_object_info_with_vlm_categories(objects_info, vlm_categories):
    """结合物体信息和VLM类别信息"""
    combined_info = {}
    
    logger.info("物体信息和VLM类别匹配结果:")
    for obj_name, obj_info in objects_info.items():
        if obj_name in vlm_categories:
            vlm_info = vlm_categories[obj_name]
            combined_info[obj_name] = {
                'name': vlm_info['name'],
                'color': vlm_info['color'],
                'category': vlm_info['category'],
                'object_info': obj_info
            }
            logger.info(f"  {obj_name}: {vlm_info['name']} ({vlm_info['category']}) - {vlm_info['color']}")
        else:
            # 如果VLM没有识别到，使用默认信息
            combined_info[obj_name] = {
                'name': f"unknown_object_{obj_info['object_id']}",
                'color': 'unknown',
                'category': 'unknown',
                'object_info': obj_info
            }
            logger.info(f"  {obj_name}: 未识别物体")
    
    return combined_info

def refine_rough_tasks_with_precise_calculation(rough_tasks, combined_objects):
    """
    使用精确计算来优化VLM的粗略任务位置
    
    Args:
        rough_tasks: VLM生成的粗略任务列表
        combined_objects: 物体信息字典
    
    Returns:
        位置精确化后的任务列表
    """
    refined_tasks = []
    
    # 首先找到盘子作为参考物体
    plate_obj_key = None
    plate_obj_info = None
    for obj_key, obj_data in combined_objects.items():
        if obj_data['category'].lower() in ['plate', 'dish', '盘子']:
            plate_obj_key = obj_key
            plate_obj_info = obj_data['object_info']
            break
    
    if not plate_obj_info:
        logger.warning("未找到盘子作为参考物体，保持VLM的粗略位置")
        return rough_tasks
    
    logger.info(f"使用 {plate_obj_key} 作为参考物体进行位置精确化")
    
    for task in rough_tasks:
        obj_key = task['object']
        
        if obj_key not in combined_objects:
            logger.warning(f"物体 {obj_key} 不存在，跳过精确化")
            refined_tasks.append(task)
            continue
        
        obj_data = combined_objects[obj_key]
        obj_info = obj_data['object_info']
        if obj_data['category'].lower() in ['wineglass']:
            refined_tasks.append(task)
            continue
        # 使用VLM提供的放置类型和参数进行精确计算
        placement_type = task.get('placement_type', 'on_center')
        placement_params = task.get('placement_params', {})
        
        # 根据放置类型选择目标物体（通常是盘子）
        if placement_type in ['on_center', 'on_edge', 'beside']:
            target_obj_info = plate_obj_info
        else:
            target_obj_info = plate_obj_info  # 默认使用盘子
        
        # 检查VLM粗略位置是否已经合理
        vlm_pos = task['target_position']
        obj_center = [obj_info['center_position']['x'], obj_info['center_position']['y'], obj_info['center_position']['z']]
        
        # 计算VLM位置与物体实际位置的距离
        distance = ((vlm_pos[0] - obj_center[0])**2 + (vlm_pos[1] - obj_center[1])**2 + (vlm_pos[2] - obj_center[2])**2)**0.5
        
        # if distance < 50:  # 如果VLM位置与物体中心距离小于5cm，认为已经很准确
        # logger.info(f"VLM位置已经很准确 (距离: {distance:.1f}mm)，使用VLM位置加抓取偏移")
        
        # 计算抓取点到中心点的偏移向量
        grasp_to_center_offset = [
            obj_info['center_position']['x'] - obj_info['grasp_point']['x'],
            obj_info['center_position']['y'] - obj_info['grasp_point']['y'],
            obj_info['center_position']['z'] - obj_info['grasp_point']['z']
        ]
        

        # Bowl placement should use the requested plate-relative target
        # directly. Other object categories retain the legacy grasp-point
        # compensation.
        apply_grasp_offset = obj_data['category'].lower() != 'bowl'
        precise_target_pos = (
            [
                vlm_pos[0] - grasp_to_center_offset[0],
                vlm_pos[1] - grasp_to_center_offset[1],
                vlm_pos[2] - grasp_to_center_offset[2],
            ]
            if apply_grasp_offset
            else list(vlm_pos)
        )
        
        refined_task = task.copy()
        refined_task['target_position'] = precise_target_pos
        refined_task['source_position'] = [
            obj_info['grasp_point']['x'],
            obj_info['grasp_point']['y'],
            obj_info['grasp_point']['z']
        ]
        refined_task['description'] += (
            " (VLM位置+抓取偏移)" if apply_grasp_offset else " (VLM位置)"
        )
        refined_tasks.append(refined_task)
        
        adjustment_name = (
            "VLM位置+抓取偏移" if apply_grasp_offset else "VLM位置"
        )
        logger.info(f"调整 {obj_data['name']} 位置 ({adjustment_name}):")
        logger.info(f"  VLM位置: [{vlm_pos[0]:.1f}, {vlm_pos[1]:.1f}, {vlm_pos[2]:.1f}]")
        if apply_grasp_offset:
            logger.info(f"  抓取偏移: [{-grasp_to_center_offset[0]:.1f}, {-grasp_to_center_offset[1]:.1f}]")
        logger.info(f"  最终位置: [{precise_target_pos[0]:.1f}, {precise_target_pos[1]:.1f}, {precise_target_pos[2]:.1f}]")
    
    logger.info(f"完成 {len(refined_tasks)} 个任务的位置精确化")
    return refined_tasks

def prepare_objects_for_prompt(combined_objects):
    """为prompt准备物体信息"""
    prompt_objects = []
    
    for obj_key, obj_data in combined_objects.items():
        obj_info = obj_data['object_info']
        
        if not obj_info.get('grasp_point'):  # 只包含有抓取信息的物体
            continue
            
        prompt_obj = {
            "object_key": obj_key,
            "object_name": obj_data['name'],
            "object_category": obj_data['category'],
            "object_color": obj_data['color'],
            "center_position": {
                "x": obj_info['center_position']['x'],
                "y": obj_info['center_position']['y'],
                "z": obj_info['center_position']['z']
            },
            "grasp_point": {
                "x": obj_info['grasp_point']['x'],
                "y": obj_info['grasp_point']['y'],
                "z": obj_info['grasp_point']['z']
            },
            "dimensions": {
                "width": obj_info['object_size']['width'],
                "height": obj_info['object_size']['height'],
                "depth": obj_info['object_size']['depth']
            }
        }
        prompt_objects.append(prompt_obj)
    
    return prompt_objects

def get_grasp_pose_for_object(obj_info, object_class):
    """根据物体类别获取抓取位姿"""
    grasp_matrix = np.array(obj_info['grasp_pose_matrix'])
    
    # 提取位置和旋转
    position = grasp_matrix[:3, 3]  # [x, y, z]
    
    # 转换为欧拉角 (XYZ顺序)
    from controllor.utils import mat_to_pos
    
    # mat_to_pos返回完整的6DOF位姿[x, y, z, rx, ry, rz]
    full_pose = mat_to_pos(grasp_matrix)
    
    position = full_pose[:3]  # [x, y, z]
    euler_angles = full_pose[3:]  # [rx, ry, rz]
    
    
    return position.tolist() + euler_angles.tolist()  # [x, y, z, rx, ry, rz]


def generate_pick_and_place_sequence(pick_and_place_tasks, combined_objects=None):
    """
    根据pick and place任务生成包含8步流程的完整任务序列，包含完整6DOF位姿
    
    Args:
        pick_and_place_tasks: 每个元素包含 {object, source_position, target_position, description}
        combined_objects: 物体信息字典，用于获取抓取位姿
    
    Returns:
        完整的子任务序列，包括必要的reset操作和旋转角度
    """
    full_task_sequence = []
    last_used_arm = None
    
    for i, pp_task in enumerate(pick_and_place_tasks):
        obj_name = pp_task['object']
        source_pos = pp_task['source_position']
        target_pos = pp_task['target_position']
        description = pp_task['description']
        
        # 获取物体信息和抓取位姿
        obj_info = None
        obj_class = "other"
        grasp_pose_6dof = None
        
        if combined_objects and obj_name in combined_objects:
            obj_data = combined_objects[obj_name]
            obj_info = obj_data['object_info']
            obj_class = obj_data.get('category', 'other')
            
            # 获取完整的6DOF抓取位姿
            if obj_class in ['wineglass']:
                grasp_pose_6dof=[0,0,0,0,0,0]   
            else:
                grasp_pose_6dof = get_grasp_pose_for_object(obj_info, obj_class)
            
        # 默认旋转角度（如果没有物体信息）
        default_rotation = [0, 0, 0]  # [rx, ry, rz] 默认欧拉角
        
        # Match the active dual_v2_2 convention used by Step 4:
        # left/index0 works on positive Y, right/index1 works on negative Y.
        # reset_pos is inserted before the later arm-normalization pass, so
        # this assignment must already be correct or the wrong arm is reset.
        current_arm = 'left' if source_pos[1] >= 0 else 'right'
        
        # 如果切换臂，需要先将之前的臂reset到安全位置
        if last_used_arm is not None and last_used_arm != current_arm:
            full_task_sequence.append({
                "subtask_id": len(full_task_sequence) + 1,
                "action": "reset_pos",
                "target_position": None,
                "target_rotation": None,
                # "target_pose": reset_position + reset_rotation,
                "description": f"重置{last_used_arm}臂到安全位置；实际位姿由RobotArm.control_joint_position()决定",
                "object": None,
                "arm": last_used_arm
            })
        
        # 准备抓取位姿的旋转角度
        grasp_rotation = grasp_pose_6dof[3:] if grasp_pose_6dof and len(grasp_pose_6dof) >= 6 else default_rotation
        
        # 放置时的旋转角度（通常与抓取相同，但可以根据需要调整）
        place_rotation = grasp_rotation.copy()
        
        # 添加8步pick and place流程
        steps = [
            {
                "action": "move_above_grasp",
                "target_position": [source_pos[0], source_pos[1], source_pos[2] + 100],
                "target_rotation": grasp_rotation,
                # "target_pose": [source_pos[0], source_pos[1], source_pos[2] + 100] + grasp_rotation,
                "description": f"使用{current_arm}臂移动到{obj_name}上方10cm并调整抓取位姿",
                "object": obj_name,
                "arm": current_arm
            },
            {
                "action": "move_down_to_grasp",
                "target_position": source_pos[:3] if len(source_pos) >= 3 else source_pos,
                "target_rotation": grasp_rotation,
                # "target_pose": (source_pos[:3] if len(source_pos) >= 3 else source_pos) + grasp_rotation,
                "description": f"使用{current_arm}臂下降10cm到{obj_name}抓取点",
                "object": obj_name,
                "arm": current_arm
            },
            {
                "action": "close_gripper",
                "target_position": source_pos[:3] if len(source_pos) >= 3 else source_pos,
                "target_rotation": grasp_rotation,
                # "target_pose": (source_pos[:3] if len(source_pos) >= 3 else source_pos) + grasp_rotation,
                "description": f"使用{current_arm}臂关闭夹爪抓取{obj_name}",
                "object": obj_name,
                "arm": current_arm
            },
            {
                "action": "lift_object",
                "target_position": [source_pos[0], source_pos[1], source_pos[2] + 100],
                "target_rotation": grasp_rotation,
                # "target_pose": [source_pos[0], source_pos[1], source_pos[2] + 100] + grasp_rotation,
                "description": f"使用{current_arm}臂抬升{obj_name} 10cm",
                "object": obj_name,
                "arm": current_arm
            },
            {
                "action": "move_to_target",
                "target_position": [target_pos[0], target_pos[1], max(target_pos[2] + 100, source_pos[2] + 100)],
                "target_rotation": grasp_rotation,  # 移动时保持抓取姿态
                # "target_pose": [target_pos[0], target_pos[1], max(target_pos[2] + 100, source_pos[2] + 100)] + grasp_rotation,
                "description": f"使用{current_arm}臂将{obj_name}移动到目标位置上方",
                "object": obj_name,
                "arm": current_arm
            },
            {
                "action": "move_down_to_place",
                "target_position": [target_pos[0], target_pos[1], target_pos[2] + 60],
                "target_rotation": place_rotation,
                # "target_pose": [target_pos[0], target_pos[1], target_pos[2] + 60] + place_rotation,
                "description": f"使用{current_arm}臂下降到{obj_name}放置点",
                "object": obj_name,
                "arm": current_arm
            },
            {
                "action": "open_gripper",
                "target_position":[target_pos[0], target_pos[1], target_pos[2] + 60],
                "target_rotation": place_rotation,
                # "target_pose": [target_pos[0], target_pos[1], target_pos[2] + 60] + place_rotation,
                "description": f"使用{current_arm}臂释放{obj_name}",
                "object": obj_name,
                "arm": current_arm
            },
            {   "action": "move_up_after_release",
                "target_position": [target_pos[0], target_pos[1], target_pos[2] + 100],
                "target_rotation": place_rotation,
                # "target_pose": [target_pos[0], target_pos[1], target_pos[2] + 100] + place_rotation,
                "description": f"使用{current_arm}臂释放后抬升10cm",
                "object": obj_name,
                "arm": current_arm
            }
        ]
        
        # 添加步骤到总序列
        for step in steps:
            full_task_sequence.append({
                "subtask_id": len(full_task_sequence) + 1,
                **step
            })
        
        last_used_arm = current_arm
# reset_pos split for left and right
#     reset_right=  {
#     "subtask_id": 17,
#     "action": "reset_pos",
#     "target_position": [
#         215.33098, -722.95459, 753.57861
#     ],
#     "target_rotation": [
#         -152.47, -34.546, -66.899
#     ],
    
#     "description": "重置right臂到安全位置防止碰撞",
#     "object": 'null',
#     "arm": "right"
#    }
#     reset_left=  {
#     "subtask_id": 17,
#     "action": "reset_pos",
#     "target_position": [
#       260.90686, 847.93127, 613.90411
#     ],
#     "target_rotation": [
#       -41.333, -21.694, 69.811
#     ],
#     "description": "重置left臂到安全位置防止碰撞",
#     "object": 'null',
#     "arm": "left"
#    }
#     full_task_sequence.append(reset_left)
#     full_task_sequence.append(reset_right)
    # reset_pos in then end for all
    reset_all=  {
        "subtask_id": 17,
        "action": "reset_all",
        "target_position": None,
        "target_rotation": None,
        "description": "左右臂回最终安全位；实际位姿由RobotArm.get_reset_pos() / self.reset_pos决定",
        "object": 'null',
        "arm": "right"
    }
    full_task_sequence.append(reset_all)
    return full_task_sequence

class GPT4v_simple():
    def __init__(self, api_key="REDACTED", base_url="https://api.poe.com/v1",model_name="gemini-2.5-pro"):
        self.api_key = api_key
        self.base_url = base_url

        self.chat_history = {
            "model": model_name,
            "messages": [],
            "max_tokens": 30000
        }

        assert self.api_key is not None
        assert self.base_url is not None

        self.client = OpenAI(
            base_url=self.base_url, 
            api_key=self.api_key
        )
    def chat(self, prompt):
        response = self.client.chat.completions.create(
            model=self.chat_history["model"],
            messages=[{"role": "user", "content": prompt}],
            max_tokens=self.chat_history["max_tokens"],
            temperature=1.0,  # 降低temperature获得更确定的输出
        )
        return response.choices[0].message.content

def visualize_robot_path_on_pointcloud(detailed_tasks_file, objects_info_file=None, output_dir="./output", scene_data_dir=None):
    """
    将机器人执行路径可视化到点云上
    
    Args:
        detailed_tasks_file: 详细任务文件路径
        objects_info_file: 物体信息文件路径
        output_dir: 输出目录
        scene_data_dir: 场景数据目录（包含depth_ext.npy, rgb_ext.png, vggt_K.npy）
    """
    logger.info("开始生成机器人路径可视化...")
    
    # 1. 读取任务文件
    if not os.path.exists(detailed_tasks_file):
        logger.error(f"任务文件不存在: {detailed_tasks_file}")
        return
    
    with open(detailed_tasks_file, 'r', encoding='utf-8') as f:
        detailed_tasks = json.load(f)
    
    logger.info(f"读取了 {len(detailed_tasks)} 个详细任务步骤")
    
    # 2. 提取路径点
    path_points = []
    point_types = []  # 用于标识点的类型
    point_colors = []
    point_descriptions = []
    
    # 定义颜色映射 - 只保留reset_pos和抓取点相关的颜色
    color_map = {
        'reset_pos': [1.0, 0.0, 0.0],        # 红色 - 重置位置
        'move_above_grasp': [0.0, 1.0, 0.0], # 绿色 - 移动到抓取上方
        'move_down_to_grasp': [0.0, 0.0, 1.0], # 蓝色 - 下降到抓取点
    }
    
    # 存储路径点信息，按物体分组
    path_by_object = {}  # {object_id: {'left': [], 'right': []}}
    object_colors = {}   # {object_id: {'left': color, 'right': color}}
    
    # 定义物体颜色调色板（使用HSV色彩空间生成区分度高的颜色）
    import colorsys
    def generate_object_colors(num_objects):
        colors = []
        for i in range(num_objects):
            hue = i / num_objects
            # 左臂使用较亮的颜色，右臂使用较深的颜色
            left_color = colorsys.hsv_to_rgb(hue, 0.7, 0.9)   # 高亮度
            right_color = colorsys.hsv_to_rgb(hue, 0.9, 0.7)  # 低亮度
            colors.append({'left': left_color, 'right': right_color})
        return colors
    
    # 存储抓取点位姿信息
    grasp_poses = []
    
    # 首先收集所有涉及的物体
    objects_in_tasks = set()
    for task in detailed_tasks:
        obj = task.get('object')
        if obj and obj != 'null':
            objects_in_tasks.add(obj)
    
    objects_list = sorted(list(objects_in_tasks))
    logger.info(f"任务中涉及的物体: {objects_list}")
    
    # 为每个物体生成颜色
    if objects_list:
        colors = generate_object_colors(len(objects_list))
        for i, obj in enumerate(objects_list):
            object_colors[obj] = colors[i]
            path_by_object[obj] = {'left': [], 'right': []}
    
    # 为reset动作创建特殊分组
    path_by_object['reset'] = {'left': [], 'right': []}
    object_colors['reset'] = {'left': [1.0, 0.5, 0.5], 'right': [0.5, 0.0, 0.0]}  # 粉红和深红
    
    for task in detailed_tasks:
        target_pos = task.get('target_position', [0, 0, 0])
        action = task.get('action', 'unknown')
        arm = task.get('arm', 'unknown')
        obj = task.get('object', 'unknown')
        description = task.get('description', 'N/A')
        
        # 确保位置是3D点
        if len(target_pos) >= 3:
            point = [target_pos[0], target_pos[1], target_pos[2]]
            
            # 收集抓取点的位姿信息
            if action in ['move_above_grasp', 'move_down_to_grasp']:
                # 从任务中获取真实的旋转信息（欧拉角，度数）
                target_rotation = task.get('target_rotation', [0, 0, 0])  # [rx, ry, rz] in degrees

                gripper_pos = target_pos + target_rotation
                R = np.array([
                    [0, 0, 1], 
                    [0, -1, 0], 
                    [1, 0, 0]
                ])
                pos_mat = pos_to_mat(gripper_pos.copy())
                pos_rot = pos_mat[:3, :3] @ R  # Convert ZXY to XYZ
                new_pos = np.eye(4)
                new_pos[:3, :3] = pos_rot
                new_pos[:3, 3] = pos_mat[:3, 3]
                pos_rot = mat_to_pos(new_pos)
                target_rotation = pos_rot[3:].copy()
                target_rotation = target_rotation.tolist()
                
                pose_info = {
                    'position': point,
                    'action': action,
                    'arm': arm,
                    'object': obj,
                    'rotation': target_rotation  # 欧拉角 [rx, ry, rz] in degrees
                }
                grasp_poses.append(pose_info)
            
            # 根据物体和机械臂分组路径点
            if action == 'reset_pos':
                # reset动作特殊处理
                if arm in ['left', 'right'] and 'reset' in path_by_object:
                    path_by_object['reset'][arm].append(point)
            elif obj and obj != 'null' and obj in path_by_object:
                # 普通物体操作
                if arm in ['left', 'right']:
                    path_by_object[obj][arm].append(point)
            else:
                # 回退到通用路径（不应该发生，但保持兼容性）
                path_points.append(point)
                point_colors.append([0.7, 0.7, 0.7])  # 灰色
                point_descriptions.append(f"{action}: {description}")
    
    # 统计路径点数量
    total_left_points = sum(len(path_by_object[obj]['left']) for obj in path_by_object)
    total_right_points = sum(len(path_by_object[obj]['right']) for obj in path_by_object)
    
    logger.info(f"按物体分组的路径点统计:")
    for obj in path_by_object:
        left_count = len(path_by_object[obj]['left'])
        right_count = len(path_by_object[obj]['right'])
        if left_count > 0 or right_count > 0:
            logger.info(f"  {obj}: 左臂 {left_count} 点, 右臂 {right_count} 点")
    
    logger.info(f"总计: 左臂 {total_left_points} 点, 右臂 {total_right_points} 点, 通用 {len(path_points)} 点")
    
    # 3. 创建Open3D可视化
    vis = o3d.visualization.Visualizer()
    vis.create_window("机器人执行路径可视化", width=1200, height=800)
    
    # 4. 加载完整场景点云
    scene_pointcloud = None
    if scene_data_dir and os.path.exists(scene_data_dir):
        try:
            # 查找深度图、RGB图像和相机内参文件
            depth_file = pjoin(scene_data_dir, 'vggt_depth_ext.npy')
            rgb_file = pjoin(scene_data_dir, 'rgb_ext.png')
            camera_intrinsics_file = pjoin(scene_data_dir, 'vggt_K.npy')
            
            if os.path.exists(depth_file) and os.path.exists(rgb_file):
                logger.info("加载场景点云数据...")
                
                # 加载深度图
                depth_image = np.load(depth_file)
                logger.info(f"深度图尺寸: {depth_image.shape}")
                
                # 加载RGB图像
                rgb_image = cv2.imread(rgb_file)
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
                logger.info(f"RGB图像尺寸: {rgb_image.shape}")
                
                # 加载相机内参
                if os.path.exists(camera_intrinsics_file):
                    K = np.load(camera_intrinsics_file)
                    logger.info(f"相机内参矩阵: \n{K}")
                else:
                    # 使用默认相机内参（如果没有找到）
                    logger.warning("未找到相机内参文件，使用默认参数")
                    height, width = depth_image.shape
                    K = np.array([[width*0.8, 0, width/2],
                                  [0, height*0.8, height/2],
                                  [0, 0, 1]])
                
                # 从深度图和RGB图像生成点云
                height, width = depth_image.shape
                fx, fy = K[0, 0], K[1, 1]
                cx, cy = K[0, 2], K[1, 2]
                
                # 创建像素坐标网格
                u, v = np.meshgrid(np.arange(width), np.arange(height))
                
                # 过滤有效深度值
                valid_mask = (depth_image > 0) & (depth_image < 2000)  # 深度范围过滤
                
                # 计算3D坐标
                z = depth_image[valid_mask]
                x = (u[valid_mask] - cx) * z / fx
                y = (v[valid_mask] - cy) * z / fy
                
                # 获取对应的颜色
                colors = rgb_image[valid_mask] / 255.0
                
                # 创建Open3D点云
                scene_pointcloud = o3d.geometry.PointCloud()
                points_3d = np.column_stack((x, y, z))
                scene_pointcloud.points = o3d.utility.Vector3dVector(points_3d)
                scene_pointcloud.colors = o3d.utility.Vector3dVector(colors)
                
                # 获取相机到机器人坐标系的变换矩阵
                logger.info("获取相机到机器人坐标系的变换矩阵...")
                try:
                    extern_camera = ZEDCamera(is_warmup=False)
                    c2w_transform = extern_camera.c2w  # 相机到世界(机器人)坐标系的变换矩阵
                    logger.info(f"相机到机器人坐标系变换矩阵:\n{c2w_transform}")
                    
                    # 将点云从相机坐标系变换到机器人坐标系
                    scene_pointcloud.transform(c2w_transform)
                    logger.info("点云已转换到机器人坐标系")
                    
                except Exception as transform_e:
                    logger.warning(f"获取坐标变换失败: {transform_e}")
                    logger.warning("点云将保持在相机坐标系中")
                
                # 点云下采样以提高性能（减少降采样保留更多细节）
                # scene_pointcloud = scene_pointcloud.voxel_down_sample(voxel_size=0.3)
                
                logger.info(f"生成点云包含 {len(scene_pointcloud.points)} 个点")
                vis.add_geometry(scene_pointcloud)
                
            else:
                logger.warning(f"未找到深度图或RGB图像文件: {depth_file}, {rgb_file}")
                
        except Exception as e:
            logger.warning(f"加载场景点云失败: {e}")
            logger.warning("将使用简单的物体表示")
            
            # 回退到简单的物体点云表示
            if objects_info_file and os.path.exists(objects_info_file):
                try:
                    with open(objects_info_file, 'r', encoding='utf-8') as f:
                        objects_info = json.load(f)
                    
                    # 为每个物体创建小的点云表示
                    for obj_key, obj_info in objects_info.items():
                        if 'center_position' in obj_info:
                            center = obj_info['center_position']
                            size = obj_info.get('object_size', {'width': 50, 'height': 50, 'depth': 50})
                            
                            # 创建简单的长方体点云表示物体
                            object_points = []
                            for dx in [-1, 0, 1]:
                                for dy in [-1, 0, 1]:
                                    for dz in [0, 1]:
                                        object_points.append([
                                            center['x'] + dx * size['width'] / 4,
                                            center['y'] + dy * size['depth'] / 4,
                                            center['z'] + dz * size['height'] / 4
                                        ])
                            
                            if object_points:
                                obj_pcd = o3d.geometry.PointCloud()
                                obj_pcd.points = o3d.utility.Vector3dVector(object_points)
                                obj_pcd.paint_uniform_color([0.3, 0.3, 0.3])  # 深灰色表示物体
                                vis.add_geometry(obj_pcd)
                                
                except Exception as e2:
                    logger.warning(f"回退方案也失败: {e2}")
    else:
        logger.info("未提供场景数据目录，跳过点云加载")
    
    # 5. 按物体和机械臂添加路径可视化
    logger.info("添加按物体分组的路径可视化...")
    
    for obj_name, arms_data in path_by_object.items():
        if obj_name not in object_colors:
            continue
            
        # 添加左臂路径
        if arms_data['left']:
            left_points = arms_data['left']
            left_color = object_colors[obj_name]['left']
            
            # 创建点云
            left_pcd = o3d.geometry.PointCloud()
            left_pcd.points = o3d.utility.Vector3dVector(left_points)
            # 所有点使用相同颜色
            left_colors = [left_color] * len(left_points)
            left_pcd.colors = o3d.utility.Vector3dVector(left_colors)
            vis.add_geometry(left_pcd)
            
            # 添加连线
            if len(left_points) > 1:
                lines = []
                for i in range(len(left_points) - 1):
                    lines.append([i, i + 1])
                
                left_line_set = o3d.geometry.LineSet()
                left_line_set.points = o3d.utility.Vector3dVector(left_points)
                left_line_set.lines = o3d.utility.Vector2iVector(lines)
                left_line_set.paint_uniform_color(left_color)
                vis.add_geometry(left_line_set)
            
            logger.info(f"  添加 {obj_name} 左臂路径: {len(left_points)} 点, 颜色 RGB{left_color}")
        
        # 添加右臂路径
        if arms_data['right']:
            right_points = arms_data['right']
            right_color = object_colors[obj_name]['right']
            
            # 创建点云
            right_pcd = o3d.geometry.PointCloud()
            right_pcd.points = o3d.utility.Vector3dVector(right_points)
            # 所有点使用相同颜色
            right_colors = [right_color] * len(right_points)
            right_pcd.colors = o3d.utility.Vector3dVector(right_colors)
            vis.add_geometry(right_pcd)
            
            # 添加连线
            if len(right_points) > 1:
                lines = []
                for i in range(len(right_points) - 1):
                    lines.append([i, i + 1])
                
                right_line_set = o3d.geometry.LineSet()
                right_line_set.points = o3d.utility.Vector3dVector(right_points)
                right_line_set.lines = o3d.utility.Vector2iVector(lines)
                right_line_set.paint_uniform_color(right_color)
                vis.add_geometry(right_line_set)
            
            logger.info(f"  添加 {obj_name} 右臂路径: {len(right_points)} 点, 颜色 RGB{right_color}")
    
    # 7. 添加抓取点位姿可视化
    logger.info(f"添加 {len(grasp_poses)} 个抓取点位姿可视化...")
    for i, pose_info in enumerate(grasp_poses):
        position = pose_info['position']
        action = pose_info['action']
        arm = pose_info['arm']
        rotation = pose_info['rotation']  # [rx, ry, rz] in degrees
        
        # 为每个抓取点创建小的坐标轴来显示位姿
        grasp_frame_size = 30  # 抓取点坐标轴大小
        
        # 创建坐标轴
        grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=grasp_frame_size, 
            origin=[0, 0, 0]  # 先在原点创建
        )
        
        # 根据动作类型调整坐标轴大小
        scale_factor = 0.7 if action == 'move_above_grasp' else 1.2
        grasp_frame.scale(scale_factor, center=[0, 0, 0])
        
        # 使用真实的旋转信息创建变换矩阵
        try:
            # 将欧拉角转换为旋转矩阵
            rotation_matrix = angle2rotation(rotation[0], rotation[1], rotation[2])
            
            # 创建4x4变换矩阵
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = position
            
            # 应用变换
            grasp_frame.transform(transform_matrix)
            
            logger.debug(f"抓取点 {i+1} 位姿: 位置={position}, 旋转={rotation}")
            
        except Exception as pose_e:
            logger.warning(f"抓取点 {i+1} 位姿变换失败: {pose_e}")
            # 回退到简单的平移变换
            translation_matrix = np.eye(4)
            translation_matrix[:3, 3] = position
            grasp_frame.transform(translation_matrix)
        
        vis.add_geometry(grasp_frame)
    
    # 8. 添加全局坐标系
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
    vis.add_geometry(coordinate_frame)
    
    # 9. 设置视角
    view_control = vis.get_view_control()
    view_control.set_front([-0.5, -0.5, -0.7])
    view_control.set_lookat([400, 0, 100])
    view_control.set_up([0, 0, 1])
    view_control.set_zoom(0.3)
    
    # 10. 保存可视化图像
    os.makedirs(output_dir, exist_ok=True)
    vis.poll_events()
    vis.update_renderer()
    
    # 保存截图
    screenshot_path = pjoin(output_dir, "robot_path_visualization.png")
    vis.capture_screen_image(screenshot_path)
    logger.info(f"路径可视化截图已保存到: {screenshot_path}")
    
    # 11. 显示可视化窗口
    logger.info("显示路径可视化窗口...")
    logger.info("颜色说明:")
    logger.info("  🎨 不同颜色代表不同物体的轨迹")
    logger.info("  💡 每个物体有两种色调: 亮色=左臂, 暗色=右臂")
    logger.info("  📐 坐标轴: 抓取点位姿 (较大=实际抓取点, 较小=抓取上方点)")
    
    # 打印物体颜色映射
    logger.info("物体轨迹颜色映射:")
    for obj_name, colors in object_colors.items():
        if obj_name in path_by_object:
            left_count = len(path_by_object[obj_name]['left'])
            right_count = len(path_by_object[obj_name]['right'])
            if left_count > 0 or right_count > 0:
                left_rgb = [f"{c:.2f}" for c in colors['left']]
                right_rgb = [f"{c:.2f}" for c in colors['right']]
                logger.info(f"  {obj_name}:")
                if left_count > 0:
                    logger.info(f"    左臂 ({left_count}点): RGB[{','.join(left_rgb)}]")
                if right_count > 0:
                    logger.info(f"    右臂 ({right_count}点): RGB[{','.join(right_rgb)}]")
    
    vis.run()
    vis.destroy_window()
    
    logger.info("✅ 路径可视化完成")

# Reset任务只保留动作语义；实际执行位姿由RobotArm配置决定。
LEFT_RESET_POS = None
RIGHT_RESET_POS = None


if __name__=="__main__":
    set_random_seeds(42)  # Set random seeds for reproducibility
    args = parse_args()
    output_dir = os.path.join(args.output)

    robot = RobotArm()
    LEFT_RESET_POS = robot.get_reset_pos(0, eular=True)
    RIGHT_RESET_POS = robot.get_reset_pos(1, eular=True)
    
    # 如果只是可视化模式，直接运行可视化
    if args.visualize_only:
        logger.info("=" * 60)
        logger.info("运行路径可视化模式")
        logger.info("=" * 60)
        
        detailed_tasks_file = args.visualize_only
        objects_info_file = os.path.join(output_dir, "objects_info_world.json")
        
        # 检查是否有带旋转信息的任务文件
        if detailed_tasks_file:
            task_dir = os.path.dirname(detailed_tasks_file)
            task_filename = os.path.basename(detailed_tasks_file)
            
            # 优先使用带旋转信息的文件
            # rotation_file = pjoin(task_dir, 'detailed_subtasks_with_rotations.json')
            # if os.path.exists(rotation_file):
            #     detailed_tasks_file = rotation_file
            #     logger.info(f"找到带旋转信息的任务文件: {rotation_file}")
            # else:
            #     logger.info(f"未找到旋转文件，使用原文件: {detailed_tasks_file}")
        
        # 确定场景数据目录
        scene_data_dir = args.scene_data
        if not scene_data_dir and detailed_tasks_file:
            # 尝试从任务文件路径推断场景数据目录
            task_dir = os.path.dirname(detailed_tasks_file)
            # 查找包含深度图的目录
            possible_dirs = [
                task_dir,  # 任务文件同目录
                "test_outputs/test_task01_long02",  # 默认测试目录
            ]
            for possible_dir in possible_dirs:
                if os.path.exists(pjoin(possible_dir, 'vggt_depth_ext.npy')):
                    scene_data_dir = possible_dir
                    logger.info(f"找到场景数据目录: {scene_data_dir}")
                    break
        
        visualize_robot_path_on_pointcloud(detailed_tasks_file, objects_info_file, output_dir, scene_data_dir)
        exit(0)
    
    try:
        # 1. 加载物体信息
        logger.info("=" * 60)
        logger.info("开始机器人任务规划和执行系统")
        logger.info("=" * 60)
        
        objects_info = load_objects_info(output_dir)
        if objects_info is None:
            logger.error("无法加载物体信息，退出程序")
            exit(1)
        
        # 2. 加载VLM类别信息
        vlm_categories = load_vlm_categories(output_dir)
        if vlm_categories is None:
            logger.error("无法加载VLM类别信息，退出程序")
            exit(1)
        
        # 3. 结合物体信息和VLM类别
        combined_objects = get_object_info_with_vlm_categories(objects_info, vlm_categories)
        
        # 4. 准备prompt数据
        prompt_objects = prepare_objects_for_prompt(combined_objects)
        logger.info(f"准备了 {len(prompt_objects)} 个物体用于任务规划")
        
        # 5. 根据参数选择任务生成方法
        vlm_subtasks_json = pjoin(output_dir, 'vlm_rough_tasks.json')
        os.makedirs(output_dir, exist_ok=True)
        
        if args.task_generation_method == "manual":
            logger.info("使用手动任务生成器生成粗略任务...")
            
            # 定义预设任务列表 - 可以根据需要修改这些任务
            predefined_tasks = [
                ("object_0", "object_1", 0, 0, 0),        # 把碗放在盘子上
                ("object_3", "object_1", 0, 0, 0),        # 把面包放在碗里  
                ("object_2", "object_1", 80, 80, 0),     # 把杯子放在盘子旁边
            ]
            
            # 使用task_generator生成粗略任务
            objects_info_path = os.path.join(output_dir, "objects_info_world.json")
            rough_tasks = generate_rough_tasks_batch(predefined_tasks, objects_info_path)
            
            logger.info(f"使用手动任务生成器生成了 {len(rough_tasks)} 个粗略任务")
            
            # 保存粗略任务列表
            with open(vlm_subtasks_json, 'w', encoding='utf-8') as f:
                json.dump(rough_tasks, f, indent=2, ensure_ascii=False)
            logger.info(f"粗略任务列表已保存到: {vlm_subtasks_json}")
            
        else:  # args.task_generation_method == "vlm"
            logger.info("使用VLM生成粗略任务...")
            
            # 初始化VLM进行粗略规划
            vlm = GPT4v_simple()
            
            # 构建完整的prompt进行粗略规划
            task_requirements = """
1. Place the bowl above the plate
2. Place the bread above the bowl on the plate
2. Position the cup 7 cm along the negative y-axis and 7 cm along the positive x-axis of the plate
"""
            
            base_prompt = f"""IMPORTANT: You must respond ONLY with a valid JSON array. Do not include any explanations, thinking process, or additional text.

You are a robot task planner. Generate a JSON array of pick-and-place tasks for these requirements:

TASK REQUIREMENTS:
{task_requirements}

AVAILABLE OBJECTS:
"""

            # 7. 添加物体信息到prompt
            full_prompt = base_prompt + json.dumps(prompt_objects, indent=2, ensure_ascii=False)
            full_prompt += """

RESPOND WITH ONLY A JSON ARRAY IN THIS EXACT FORMAT:
[
  {
    "task_id": 1,
    "object": "object_0",
    "source_position": [x, y, z],
    "target_position": [x, y, z],
    "placement_type": "on_center" / "beside",
    "placement_params": {},
    "description": "Place bowl on plate"
  }
]

NO OTHER TEXT. START WITH [ AND END WITH ]."""
            
            logger.info("发送任务规划请求到VLM进行粗略规划...")
            
            # 8. 调用VLM获取粗略的pick and place任务列表
            response = vlm.chat(full_prompt)
            logger.info("收到VLM粗略规划响应")
            
            # 9. 解析粗略的pick and place任务列表
            rough_tasks = []
            try:
                # 更精确地提取JSON - 查找```json标记或直接查找JSON数组
                task_list_json = None
                
                # 方法1: 查找```json代码块
                if '```json' in response:
                    json_start = response.find('```json') + 7  # 跳过```json
                    json_end = response.find('```', json_start)
                    if json_end != -1:
                        task_list_json = response[json_start:json_end].strip()
                
                # 方法2: 查找最后一个JSON数组
                if not task_list_json:
                    # 找到所有[的位置
                    bracket_positions = []
                    for i, char in enumerate(response):
                        if char == '[':
                            bracket_positions.append(i)
                    
                    # 从最后一个[开始查找完整的JSON数组
                    for start_pos in reversed(bracket_positions):
                        try:
                            # 尝试找到对应的]
                            bracket_count = 0
                            for i in range(start_pos, len(response)):
                                if response[i] == '[':
                                    bracket_count += 1
                                elif response[i] == ']':
                                    bracket_count -= 1
                                    if bracket_count == 0:
                                        potential_json = response[start_pos:i+1]
                                        # 尝试解析这段JSON
                                        json.loads(potential_json)
                                        task_list_json = potential_json
                                        break
                            if task_list_json:
                                break
                        except:
                            continue
                
                if not task_list_json:
                    # 方法3: 简单查找方法作为后备
                    json_start = response.find('[')
                    json_end = response.rfind(']') + 1
                    if json_start != -1 and json_end > json_start:
                        task_list_json = response[json_start:json_end]
                
                if task_list_json:
                    rough_tasks = json.loads(task_list_json)
                    logger.info(f"成功解析粗略任务列表，包含 {len(rough_tasks)} 个pick-and-place任务")
                    
                    # 保存粗略任务列表
                    with open(vlm_subtasks_json, 'w', encoding='utf-8') as f:
                        json.dump(rough_tasks, f, indent=2, ensure_ascii=False)
                    logger.info(f"粗略任务列表已保存到: {vlm_subtasks_json}")
                    
                    # 保存原始VLM响应用于调试
                    raw_response_file = pjoin(output_dir, 'vlm_raw_response.txt')
                    with open(raw_response_file, 'w', encoding='utf-8') as f:
                        f.write(response)
                    logger.info(f"VLM原始响应已保存到: {raw_response_file}")
                    
                else:
                    logger.error("无法从VLM响应中提取有效的JSON任务列表")
                    logger.error(f"VLM响应前500字符: {response[:500]}")
                    exit(1)
                    
            except json.JSONDecodeError as e:
                logger.error(f"解析任务列表JSON时出错: {e}")
                logger.error(f"提取的JSON内容: {task_list_json}")
                logger.error(f"VLM响应前500字符: {response[:500]}")
                exit(1)
        
        # 10. 使用精确计算优化粗略任务的位置
        logger.info("\n" + "=" * 60)
        logger.info("使用精确计算优化粗略任务规划的位置")
        logger.info("=" * 60)
        
        precise_tasks = refine_rough_tasks_with_precise_calculation(rough_tasks, combined_objects)
        
        # 保存精确任务列表
        precise_tasks_json = pjoin(output_dir, 'precise_pick_and_place_tasks.json')
        with open(precise_tasks_json, 'w', encoding='utf-8') as f:
            json.dump(precise_tasks, f, indent=2, ensure_ascii=False)
        logger.info(f"精确任务列表已保存到: {precise_tasks_json}")
        
        # 11. 生成完整的8步流程任务序列
        logger.info("\n" + "=" * 60)
        logger.info("生成完整的8步流程任务序列")
        logger.info("=" * 60)
        
        detailed_task_list = generate_pick_and_place_sequence(precise_tasks, combined_objects)
        logger.info(f"生成了 {len(detailed_task_list)} 个详细子任务，包含完整6DOF位姿信息")
        
        # 保存详细任务列表
        detailed_subtasks_json = pjoin(output_dir, 'detailed_subtasks.json')
        with open(detailed_subtasks_json, 'w', encoding='utf-8') as f:
            json.dump(detailed_task_list, f, indent=2, ensure_ascii=False)
        logger.info(f"详细任务列表已保存到: {detailed_subtasks_json}")
        
        robot.shutdown()
            
    except Exception as e:
        logger.error(f"程序执行过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
    