"""V2.1 selects an instance; V1.3 owns head geometry and the motion sequence.

Only segmentation/visualization are adapted. No wrist pixels enter a pose.
The V2 interactive shell is reused for language input and p/s/retry handling.
"""
from pathlib import Path
import numpy as np
import cart_book_grasp_v1_3_perception as v13


def prepare_head_target(candidate,rgb,depth,intrinsic,c2w,geometry,config_path,checkpoint=lambda:None):
    """Refine the locked head mask with V1.3's spine fit and whole-book depth."""
    from cart_book_grasp_v2_agent import load_geometry_config,distinct_masks
    from cart_book_grasp_v2_target import restore_crop
    from cart_book_grasp_v1_3_depth import whole_book_depth
    config=load_geometry_config(config_path)
    mask=np.asarray(candidate['book_mask'],bool)
    if mask.shape!=depth.shape or not mask.any(): raise ValueError('无效的选中头部 mask')
    yy,xx=np.nonzero(mask)
    box=np.array([xx.min(),yy.min(),xx.max()+1,yy.max()+1])
    pad=(box[2:]-box[:2])*.15
    crop=np.r_[np.floor(box[:2]-pad),np.ceil(box[2:]+pad)].astype(int)
    crop[[0,2]]=np.clip(crop[[0,2]],0,rgb.shape[1]);crop[[1,3]]=np.clip(crop[[1,3]],0,rgb.shape[0])
    x0,y0,x1,y1=crop
    checkpoint()
    processor,to_numpy,image_cls=geometry._load_sam3_detector()
    raw=geometry._sam3_raw_books(processor,to_numpy,image_cls.fromarray(rgb[y0:y1,x0:x1]),prompt='book spine')
    spines=restore_crop(raw,crop,depth.shape)
    failures=[]
    for score,spine in distinct_masks(spines,depth.shape,config):
        checkpoint()
        if np.count_nonzero(spine&mask)/spine.sum()<config['min_book_spine_coverage']:continue
        try:
            result=v13._spine_center_from_mask(spine,depth,intrinsic)
            result.update(whole_book_depth(mask,depth,intrinsic,c2w),score=float(score))
        except (ValueError,np.linalg.LinAlgError) as error:
            failures.append(str(error));continue
        result.update(selection_box_xyxy=box.tolist(),segmentation_crop_xyxy=crop.tolist())
        candidate.update(geometry=result,geometry_status='ready',selection_box=box.tolist(),segmentation_crop=crop.tolist())
        return candidate
    candidate.update(geometry=None,geometry_status='failed')
    raise ValueError('选中的头部书本未通过 V1_3 书脊检查；不更换目标：'+('; '.join(failures[-3:]) or '书脊不匹配'))


def install_selected_spine(book_geom, visualization=None, on_geometry=None, *, selector):
    """Scope spine segmentation/centering to this entrypoint's cart+ mode."""
    original_fill = book_geom.fill_book_meta
    cache = {"selected": None}

    def segment(rgb, depth, intrinsic, c2w, *, box_detector=None,head_captured_at=None,observation_robot=None):
        cache["selected"] = None
        try:
            if selector is None:
                raise ValueError('V2.1 requires a local selector and a current instruction')
            def publish_recognition():
                if visualization is not None:
                    if selector.last_image is not None:
                        visualization.safe('log_image', 'book/images/candidates', selector.last_image)
                    if selector.last_gate_image is not None:
                        visualization.safe('log_image', 'book/images/cart_filter', selector.last_gate_image)
                    if selector.last_review_image is not None:
                        visualization.safe('log_image', 'book/images/candidate_review', selector.last_review_image)
                    if selector.last_wrist_image is not None:
                        visualization.safe('log_image','book/images/wrist_rgb',selector.last_wrist_image)
                    if selector.last_wrist_candidates_image is not None:
                        visualization.safe('log_image','book/images/wrist_candidates',selector.last_wrist_candidates_image)
                    summary = [dict(id=item['id'], mask_source=item['recognition'],
                        base_y_mm=item['base_y_mm'], geometry_status=item['geometry_status'],
                        geometry_error=item['error'], semantic_book_margin=item.get('semantic_book_margin'),
                        verified_book=item.get('verified_book')) for item in selector.last_candidates]
                    visualization.safe('log_json_document', 'book/selection',
                        dict(decision=selector.last_decision, candidates=summary,
                             cart_filter=selector.last_filter))
                    visualization.safe('flush_book')
            try:
                from cart_book_grasp_v2_dual_view import load_config as load_dual_config,stationary_guard
                dual=load_dual_config();guard=lambda:None
                if hasattr(selector, 'bind_observation_robot'):
                    selector.bind_observation_robot(observation_robot)
                if dual['enabled']:
                    if observation_robot is None:raise ValueError('双视角采集缺少机器人静止状态校验')
                    guard=stationary_guard(observation_robot,dual['max_joint_drift_deg'])
                def prepare_selected(candidate):
                    return prepare_head_target(candidate,rgb,depth,intrinsic,c2w,book_geom,
                        selector.config_path,selector.checkpoint)
                candidate = selector.select_instances(rgb, depth, intrinsic, c2w,
                    book_geom, prepare_selected, on_recognized=publish_recognition,
                    head_captured_at=head_captured_at,dual_config=dual,observation_guard=guard)
            finally:
                publish_recognition()
            selected = candidate['geometry']
            selected['selection_id'] = candidate['id']
            selected['selection_decision'] = dict(selector.last_decision)
            cache['selected'] = selected
            book_mask = candidate['book_mask']
            book_geom.BOOK_SELECTION_NOTE = (
                f"选中 {candidate['id']}；书脊中心 {np.round(selected['uv'],1).tolist()}，"
                f"整书基座 X 中位数 {selected['book_surface_x_mm']:.1f} mm")
            if visualization is not None:
                visualization.capture(rgb, depth, intrinsic, c2w, selected, book_mask)
            return selected["mask"], selected["box"]
        except Exception as error:
            book_geom.BOOK_SELECTION_NOTE = f"选书或目标几何检查未完成：{error}"
            # Reach the v2 cycle-failure handler directly, rather than turning
            # a language clarification into the misleading 'object_0 missing'.
            raise RuntimeError(book_geom.BOOK_SELECTION_NOTE) from error

    def fill(meta, points, c2w):
        if book_geom.BOOK_GRASP_MODE != "cart+":
            return original_fill(meta, points, c2w)
        selected = cache["selected"]
        if selected is None:
            raise ValueError("本轮没有有效书脊中心")
        result = original_fill(meta, points, c2w)
        old_center = meta["centroid"][:3, 3].copy()
        new_center = selected["point_camera_m"]
        seen = set()
        for key in ("centroid", "grasp_0", "bbox_plane_x1", "bbox_plane_x2",
                    "bbox_plane_y1", "bbox_plane_y2", "bbox_plane_z1", "bbox_plane_z2"):
            value = meta[key]
            if id(value) not in seen:
                value[:3, 3] += new_center-old_center
                seen.add(id(value))
        base, _, _ = book_geom._camera_to_base(new_center.reshape(1, 3), c2w)
        result["center_base_mm"] = (base[0]*1000).tolist()
        result["center_kind"] = "visible_book_spine_midline"
        result["book_selection_id"] = selected["selection_id"]
        result["book_selection_decision"] = selected["selection_decision"]
        result["spine_center_uv"] = selected["uv"]
        result["spine_center_depth_mm"] = selected["depth_m"]*1000
        result["spine_depth_samples"] = selected["depth_samples"]
        result["spine_depth_p10_p90_mm"] = selected["depth_p10_p90_mm"]
        for key in ('spine_center_recovery', 'spine_original_center_uv', 'spine_full_fit_failure',
                    'spine_stable_region_uv', 'spine_stable_region_fraction', 'spine_center_shift_px'):
            if key in selected:
                result[key] = selected[key]
        if on_geometry is not None:
            on_geometry(selected, c2w, result)
        if visualization is not None:
            visualization.update_geometry(result, c2w)
        print(f"书脊中心（基座 mm）：{np.round(base[0]*1000, 1).tolist()}", flush=True)
        return result

    segment.instance_first = True
    book_geom.segment_cart_book_with_sam3 = segment
    book_geom.fill_book_meta = fill


def run_v13(*, task_types=None, config_path=None, wrist_config_path=None):
    import cart_book_grasp_v1_3_motion as motion
    import cart_book_grasp_v1_3_visualization as display
    import cart_book_grasp_v2_1_visualization as dual_display
    import cart_book_grasp_v2_motion as shell_motion
    selector=shell_motion._SELECTION_AGENT
    if selector is None: raise ValueError('V2.1 local selector is missing')
    saved_install=motion._install_spine_center
    saved_display=display.install_book_visualization
    saved_pause_config=shell_motion._CART_PLUS_CONFIG_PATH
    def install(geometry, visualization=None, on_geometry=None):
        return install_selected_spine(geometry,visualization,on_geometry,selector=selector)
    try:
        motion._install_spine_center=install
        display.install_book_visualization=dual_display.install_book_visualization
        # The shared p handler reloads this module's reset settings.
        shell_motion._CART_PLUS_CONFIG_PATH=Path(config_path)
        return v13.main(task_types=task_types,config_path=config_path,wrist_config_path=wrist_config_path)
    finally:
        motion._install_spine_center=saved_install
        display.install_book_visualization=saved_display
        shell_motion._CART_PLUS_CONFIG_PATH=saved_pause_config
