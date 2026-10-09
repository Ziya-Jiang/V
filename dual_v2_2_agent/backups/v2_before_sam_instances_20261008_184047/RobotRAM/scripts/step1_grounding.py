"""
this is the demo for task decomposition based on GPT-4V.
* dino grounding for visual grounding, and lable the image with class id
* GPT-4o for task decomposition,
"""
import time
import os
import numpy as np
import json
import re
import logging

from utils.image_utils import save_depth_npy, save_rgb_png, draw_label_at_mask_center
import book_grasp as book_geom

def grounding_create_output_dir(prompt, output, logger=None):
    # Create groundiong output directory
    output_dir = os.path.join(output)
    
    debug_dir = os.path.join(output_dir, 'grounding_dir')
    # clear all files under the debug_dir directory
    if os.path.exists(debug_dir):
        for file in os.listdir(debug_dir):
            file_path = os.path.join(debug_dir, file)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    os.rmdir(file_path)
            except Exception as e:
                if logger: logger.error(f"Failed to delete {file_path}. Reason: {e}")
    
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(debug_dir, exist_ok=True)

    # save prompt in json file
    args_json_path = os.path.join(output_dir, "instruction.json")
    with open(args_json_path, 'w') as f:
        json.dump({"prompt": prompt}, f, indent=4)
    if logger:  logger.info(f">>> OUTPUT DIR: {output_dir}.")
    return output_dir, debug_dir

def grounding_main(prompt, output_dir, debug_dir, related_objects, robot, vlm, extern_camera, vgm, 
                   only_grounding=True, no_color=False, logger=None, enable_debug=False, predefined_tasks=None,
                   target_object_ids=None):
    
    t0 = time.time()
    # grounding all objects in the scene
    rgb_ext, depth_ext = None, None
    for attempt in range(3):
        rgb_ext, depth_ext = extern_camera.capture_image()
        if rgb_ext is not None:
            break
        logging.warning(
            "Orbbec capture returned no RGB frame (%d/3); retrying...",
            attempt + 1,
        )
        time.sleep(0.5)
    if rgb_ext is None:
        raise RuntimeError(
            "Orbbec 未返回有效 RGB 帧。请确认相机已连接；若刚按 Ctrl+C 退出，"
            "请等待数秒后重试或重新插拔相机。"
        )
    selected_object_ids = (
        None
        if target_object_ids is None
        else {int(object_id) for object_id in target_object_ids}
    )
    book_box_first = (book_geom.BOOK_GRASP_MODE == 'cart+'
                      and getattr(book_geom.segment_cart_book_with_sam3, 'box_first', False))
    prepare_image = getattr(vgm, "prepare_boxes_image" if book_box_first else "prepare_image", None)
    grounding_prepared = getattr(vgm, "grounding_prepared", None)
    use_prepared_image = callable(prepare_image) and callable(grounding_prepared)
    if use_prepared_image:
        prepare_image(rgb_ext)
        if logger:
            logger.info(
                "Grounding --> prepared DINO box-only input for the current frame."
                if book_box_first else "Grounding --> prepared one SAM2 image embedding for the current frame."
            )
    t1 = time.time()
    
    if enable_debug:
        rgb_ext_pth = os.path.join(output_dir, "rgb_ext.png")
        save_rgb_png(rgb_ext, rgb_ext_pth)
        save_rgb_png(extern_camera.left_image, os.path.join(output_dir, "rgb_ext_left.png"))
        save_rgb_png(extern_camera.right_image, os.path.join(output_dir, "rgb_ext_right.png"))
        save_depth_npy(depth_ext, os.path.join(output_dir, "depth_ext.npy"))
        save_rgb_png(depth_ext, os.path.join(output_dir, "depth_ext.png"))


    # grounding objects in the scene & label the image with class id
    num_objects = len(related_objects)
    mask_slots = [None] * num_objects
    xyxy_slots = [None] * num_objects
    class_id_slots = [None] * num_objects
    count = 0

    grounding_prompt_list = []
    for obj_idx, object in enumerate(related_objects):
        if selected_object_ids is not None and obj_idx not in selected_object_ids:
            continue
        t1_1 = time.time()
        if (
            book_geom.BOOK_GRASP_MODE == "cart+"
            and str(object.get("category", "")).lower() == "book"
        ):
            segmented = book_geom.segment_cart_book_with_sam3(
                rgb_ext,
                depth_ext,
                getattr(extern_camera, "K", None),
                getattr(extern_camera, "c2w", None),
                **({'box_detector': vgm} if book_box_first else {}),
            )
            if segmented is None:
                if logger:
                    logger.warning(book_geom.BOOK_SELECTION_NOTE)
                continue
            sam3_mask, sam3_box = segmented
            if logger:
                logger.info(book_geom.BOOK_SELECTION_NOTE)
            mask_slots[obj_idx] = sam3_mask
            xyxy_slots[obj_idx] = sam3_box
            class_id_slots[obj_idx] = 0
            continue
        if "other" in object['category'].lower():
            grounding_prompt = object['name']
        else:
            if not no_color:
                color = (object.get('color') or '').strip()
                if color:
                    grounding_prompt = f"{color} {object['category']}"
                else:
                    grounding_prompt = object.get('name') or object['category']
                if "trashcan" in object['category'].lower():
                    grounding_prompt += f". {color or object.get('name', object['category'])} cup"
            else:
                grounding_prompt = f"{object['category']}"
                if "trashcan" in object['category'].lower():
                    grounding_prompt += f". cup"
                
        # if grounding_prompt already in grounding_prompt_list, skip this object
        if grounding_prompt in grounding_prompt_list:
            if logger: logger.info(f"Grounding --> skip object '{grounding_prompt}' as it already exists in the list.")
            continue

        grounding_prompt_list.append(grounding_prompt)
        prompt_candidates = [grounding_prompt]
        if str(object.get("category", "")).lower() == "book":
            prompt_candidates.extend(["book spine", "textbook"])
        detections = None
        last_grounding_error = None
        for candidate_prompt in dict.fromkeys(prompt_candidates):
            if logger:
                logger.info(f"Grounding --> used prompt: {candidate_prompt}")
            try:
                candidate_detections = (
                    grounding_prepared(candidate_prompt)
                    if use_prepared_image
                    else vgm.grounding(rgb_ext, candidate_prompt)
                )
                if len(candidate_detections) == 0:
                    raise ValueError("no boxes above the grounding threshold")
                detections = candidate_detections
                grounding_prompt = candidate_prompt
                break
            except Exception as error:
                last_grounding_error = error
                if logger:
                    logger.warning(
                        "Grounding attempt failed for '%s': %s",
                        candidate_prompt,
                        error,
                    )
        if detections is None:
            if logger:
                logger.error(
                    "Grounding failed for '%s': %s",
                    object.get("category", grounding_prompt),
                    last_grounding_error,
                )
            continue
        mask = detections.mask  # (N, H, W)
        xyxy = detections.xyxy  # (N, 4)
        class_id = detections.class_id  # (N, )
        candidate_index = 0
        confidence = getattr(detections, "confidence", None)
        if confidence is not None:
            confidence_values = np.asarray(confidence, dtype=float).reshape(-1)
            if confidence_values.size == mask.shape[0]:
                finite_scores = np.isfinite(confidence_values)
                if np.any(finite_scores):
                    ranked_scores = np.where(
                        finite_scores,
                        confidence_values,
                        -np.inf,
                    )
                    candidate_index = int(np.argmax(ranked_scores))
                    if logger and mask.shape[0] > 1:
                        logger.info(
                            "Grounding --> selected candidate %d/%d for '%s' "
                            "with confidence %.3f",
                            candidate_index + 1,
                            mask.shape[0],
                            grounding_prompt,
                            confidence_values[candidate_index],
                        )
        book_index = book_geom.select_book_candidate(
            object.get("category", ""),
            mask,
            depth_ext,
            getattr(extern_camera, "K", None),
            getattr(extern_camera, "c2w", None),
        )
        if book_index == -1:
            if logger:
                logger.warning(book_geom.BOOK_SELECTION_NOTE)
            continue
        if logger and book_geom.BOOK_SELECTION_NOTE:
            logger.info(book_geom.BOOK_SELECTION_NOTE)
        if book_index is not None:
            candidate_index = book_index

        # for debug
        if enable_debug:
            for i in range(mask.shape[0]):
                grounding_promp_name = re.sub(r'\W+', '_', grounding_prompt.lower())
                mask_rgb = (mask[i, ..., None] * rgb_ext).astype(np.uint8)
                mask_path = os.path.join(debug_dir, f"grounding_mask_{grounding_promp_name}_{count:02}.png")
                count += 1
                save_rgb_png(mask_rgb, mask_path)

        mask_slots[obj_idx] = mask[candidate_index]
        xyxy_slots[obj_idx] = xyxy[candidate_index]
        class_id_slots[obj_idx] = class_id[candidate_index]
        t1_2 = time.time()
        # print(f"{object['name']} time: ", t1_2 - t1_1)

    valid_masks = [m for m in mask_slots if m is not None]
    if not valid_masks:
        raise RuntimeError("Grounding failed for all objects in the scene.")
    masks = np.stack(
        [m if m is not None else np.zeros_like(valid_masks[0]) for m in mask_slots],
        axis=0,
    )
    xyxys = np.stack(
        [x if x is not None else np.zeros(4, dtype=np.float32) for x in xyxy_slots],
        axis=0,
    )
    class_ids = np.array(
        [c if c is not None else -1 for c in class_id_slots],
        dtype=np.int64,
    )
        
    
    if enable_debug:
        detections_npy_path = os.path.join(output_dir, "detections.npy")   # save detections to npy file
        np.save(detections_npy_path, {
            "masks": masks,
            "xyxys": xyxys,
            "class_ids": class_ids
        })  

        labelled_rgb_ext = rgb_ext.copy()
        for obj_idx in range(masks.shape[0]):
            if not np.any(masks[obj_idx]):
                continue
            labelled_rgb_ext = draw_label_at_mask_center(
                labelled_rgb_ext, masks[obj_idx, ...], 
                label_text=f"{obj_idx}", 
                font_scale=0.5, 
                font_thickness=2,
                text_color=(0, 255, 0),  # Green text
                bg_color=(0, 0, 0)     # (0, 0, 0) for black background, None for no background
            )
        rgb_labelling_image_path = os.path.join(output_dir, "rgb_ext_labeling.png") 
        save_rgb_png(labelled_rgb_ext, rgb_labelling_image_path)
        if logger: logger.info(f">>> save grounding rgb image to {rgb_labelling_image_path}.")

    return rgb_ext, extern_camera.left_image, extern_camera.right_image, depth_ext, masks, xyxys, class_ids



def grounding_main_3wineglass(prompt, output_dir, debug_dir, related_objects, robot, vlm, extern_camera, vgm, 
                   only_grounding=True, no_color=False, logger=None, enable_debug=False, predefined_tasks=None):
    
    t0 = time.time()
    # grounding all objects in the scene
    rgb_ext, depth_ext = extern_camera.capture_image()
    t1 = time.time()
    
    if enable_debug:
        rgb_ext_pth = os.path.join(output_dir, "rgb_ext.png")
        save_rgb_png(rgb_ext, rgb_ext_pth)
        save_rgb_png(extern_camera.left_image, os.path.join(output_dir, "rgb_ext_left.png"))
        save_rgb_png(extern_camera.right_image, os.path.join(output_dir, "rgb_ext_right.png"))
        save_depth_npy(depth_ext, os.path.join(output_dir, "depth_ext.npy"))
        save_rgb_png(depth_ext, os.path.join(output_dir, "depth_ext.png"))


    # grounding objects in the scene & label the image with class id
    masks = []
    xyxys = []
    class_ids = []
    count = 0

    grounding_prompt_list = []
    for object in related_objects:
        t1_1 = time.time()
        if "other" in object['category'].lower():
            grounding_prompt = object['name']
        else:
            if not no_color:
                grounding_prompt = f"{object['color']} {object['category']}"
                if "trashcan" in object['category'].lower():
                    grounding_prompt += f". {object['color']} cup"
            else:
                grounding_prompt = f"{object['category']}"
                if "trashcan" in object['category'].lower():
                    grounding_prompt += f". cup"
                
        # if grounding_prompt already in grounding_prompt_list, skip this object
        if grounding_prompt in grounding_prompt_list:
            if logger: logger.info(f"Grounding --> skip object '{grounding_prompt}' as it already exists in the list.")
            continue

        grounding_prompt_list.append(grounding_prompt)
        if logger: logger.info(f"Grounding --> used prompts: {grounding_prompt}")
        try:
            detections = vgm.grounding(rgb_ext, grounding_prompt)
        except Exception as e:
            if logger: logger.error(f"Grounding failed for '{grounding_prompt}'")
            continue
        mask = detections.mask  # (N, H, W)
        xyxy = detections.xyxy  # (N, 4)
        class_id = detections.class_id  # (N, )

        # 如果检测到多个对象，按照x坐标（从左到右）排序
        if mask.shape[0] > 1:
            # 计算每个检测框的中心x坐标
            x_centers = (xyxy[:, 0] + xyxy[:, 2]) / 2
            sorted_indices = np.argsort(x_centers)
            mask = mask[sorted_indices]
            xyxy = xyxy[sorted_indices]
            class_id = class_id[sorted_indices]
            if logger: logger.info(f"Grounding --> detected {mask.shape[0]} objects, sorted by x coordinate (left to right)")

        # for debug
        if enable_debug:
            for i in range(mask.shape[0]):
                grounding_promp_name = re.sub(r'\W+', '_', grounding_prompt.lower())
                mask_rgb = (mask[i, ..., None] * rgb_ext).astype(np.uint8)
                mask_path = os.path.join(debug_dir, f"grounding_mask_{grounding_promp_name}_{count:02}.png")
                count += 1
                save_rgb_png(mask_rgb, mask_path)

        # 将所有检测到的对象按x坐标顺序添加到列表中
        for i in range(mask.shape[0]):
            masks.append(mask[i:i+1])
            xyxys.append(xyxy[i:i+1])
            class_ids.append(class_id[i:i+1])
            if logger: logger.info(f"Grounding --> appending object {i} for '{grounding_prompt}'")
        break
        t1_2 = time.time()
        # print(f"{object['name']} time: ", t1_2 - t1_1)
    masks = np.concatenate(masks, axis=0)  # (N, H, W)
    xyxys = np.concatenate(xyxys, axis=0)  # (N, 4)
    class_ids = np.concatenate(class_ids, axis=0)  # (N, )
        
    
    if enable_debug:
        detections_npy_path = os.path.join(output_dir, "detections.npy")   # save detections to npy file
        np.save(detections_npy_path, {
            "masks": masks,
            "xyxys": xyxys,
            "class_ids": class_ids
        })  

        labelled_rgb_ext = rgb_ext.copy()
        for i in range(masks.shape[0]):
            labelled_rgb_ext = draw_label_at_mask_center(
                labelled_rgb_ext, masks[i, ...], 
                label_text=f"{i}", 
                font_scale=0.5, 
                font_thickness=2,
                text_color=(0, 255, 0),  # Green text
                bg_color=(0, 0, 0)     # (0, 0, 0) for black background, None for no background
            )
        rgb_labelling_image_path = os.path.join(output_dir, "rgb_ext_labeling.png") 
        save_rgb_png(labelled_rgb_ext, rgb_labelling_image_path)
        if logger: logger.info(f">>> save grounding rgb image to {rgb_labelling_image_path}.")

    return rgb_ext, extern_camera.left_image, extern_camera.right_image, depth_ext, masks, xyxys, class_ids
