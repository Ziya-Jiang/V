import json
import os

def generate_rough_tasks(object_a, object_b, x, y, z, objects_info_path=None, task_id=1):
    """
    生成一个pick and place任务，将object_a放置到object_b的指定位置上
    
    Args:
        object_a (str): 要移动的物体ID (如 "object_0")
        object_b (str): 目标物体ID (如 "object_1")
        x (float): 相对于object_b中心的x轴偏移量
        y (float): 相对于object_b中心的y轴偏移量
        z (float): 相对于object_b表面的z轴偏移量
        objects_info_path (str, optional): objects_info_world.json文件路径
        task_id (int): 任务ID，默认为1
    
    Returns:
        dict: 符合vlm_rough_tasks.json格式的任务字典
    """
    
    # 如果没有提供路径，尝试使用默认路径
    if isinstance(objects_info_path, str):
        with open(objects_info_path, 'r', encoding='utf-8') as f:
            objects_info = json.load(f)
    else:
        objects_info=objects_info_path
    
    # 检查物体是否存在
    if object_a not in objects_info:
        raise ValueError(f"找不到物体: {object_a}")
    if object_b not in objects_info:
        raise ValueError(f"找不到物体: {object_b}")
    
    # 获取物体信息
    obj_a_info = objects_info[object_a]
    obj_b_info = objects_info[object_b]
    
    # 计算起始位置 - object_a的中心位置
    source_position = [
        obj_a_info["center_position"]["x"],
        obj_a_info["center_position"]["y"], 
        obj_a_info["center_position"]["z"]
    ]
    
    # 计算目标位置 - object_b的中心位置 + 偏移 + 高度调整
    # z方向需要考虑: object_b的中心 + object_b的高度/2 + object_a的高度/2 + 额外偏移z
    target_z = (obj_b_info["center_position"]["z"] + 
                obj_b_info["object_size"]["depth"] / 2 + 
                obj_a_info["object_size"]["depth"] / 2 + 
                z)
    
    target_position = [
        obj_b_info["center_position"]["x"] + x,
        obj_b_info["center_position"]["y"] + y,
        target_z
    ]
    
    # 生成任务描述
    description = f"Place the {object_a} on the {object_b}"
    if y!=0 or x!= 0:
        placement_type='beside'
    else:
        placement_type='on_center'
    # 构建任务字典
    task = {
        "task_id": task_id,
        "object": object_a,
        "source_position": source_position,
        "target_position": target_position,
        "placement_type": placement_type,
        "placement_params": {},
        "description": description
    }
    
    return task

def generate_rough_tasks_batch(tasks_list, objects_info_path=None):
    """
    批量生成多个pick and place任务
    
    Args:
        tasks_list (list): 任务列表，每个元素为(object_a, object_b, x, y, z)的元组
        objects_info_path (str, optional): objects_info_world.json文件路径
    
    Returns:
        list: 任务字典列表
    """
    results = []
    for i, (object_a, object_b, x, y, z) in enumerate(tasks_list, 1):
        task = generate_rough_tasks(object_a, object_b, x, y, z, 
                                   objects_info_path=objects_info_path, 
                                   task_id=i)
        results.append(task)
    
    return results

# 示例使用
if __name__ == "__main__":
    # 示例1: 把object_0放在object_1的正中心上 (x=0, y=0, z=0)
    task1 = generate_rough_tasks("object_0", "object_1", 0, 0, 0)
    print("任务1:", json.dumps(task1, indent=2, ensure_ascii=False))
    
    # 示例2: 把object_2放在object_1的旁边 (x=70, y=0, z=0)
    task2 = generate_rough_tasks("object_2", "object_1", 70, 0, 0)
    print("任务2:", json.dumps(task2, indent=2, ensure_ascii=False))
    
    # 示例3: 批量生成任务
    tasks_list = [
        ("object_0", "object_1", 0, 0, 0),    # 把碗放在盘子上
        ("object_3", "object_1", 0, 0, 0),   # 把面包放在碗里
        ("object_2", "object_1", 70, -70, 0),    # 把杯子放旁边
    ]
    
    batch_tasks = generate_rough_tasks_batch(tasks_list)
    print("批量任务:")
    for task in batch_tasks:
        print(json.dumps(task, indent=2, ensure_ascii=False))
