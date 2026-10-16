"""One-time wrist selection, with diagnosed, geometry-gated SAM3 recall.

Book masks only locate the object; explicit spine masks define its centerline.
Final boundary recall is explicit and never claims semantic book identity.
"""
import json
import time
from pathlib import Path
import cv2
import numpy as np
from cart_book_grasp_v2_instances import clean_masks
from cart_book_grasp_v3_centerline import visible_centerline
from cart_book_grasp_v3_servo import ServoError


def spine_geometry(mask):
    h,w=mask.shape
    rows=np.flatnonzero(mask.any(axis=1))
    if not len(rows):raise ServoError('empty mask')
    left=mask[rows].argmax(axis=1)
    right=w-1-mask[rows,::-1].argmax(axis=1)
    valid=(left>0)&(right<w-1)&(right-left>=8)
    if valid.mean()<.8:raise ServoError('left/right sides clipped or too narrow')
    rows,left,right=rows[valid],left[valid],right[valid]
    span=float(np.ptp(rows));width=float(np.median(right-left))
    if span<.3*h or span<2*width:raise ServoError('insufficient elongated spine support')
    if not 8<=width<=.25*w:raise ServoError('spine width outside recall range')
    if not rows.min()<=.5*(h-1)<=rows.max():raise ServoError('spine does not cross image center row')
    lines=[]
    for edge in (left,right):
        dx,dy,x,y=cv2.fitLine(np.column_stack([edge,rows]).astype(np.float32),
                            cv2.DIST_HUBER,0,.01,.01).ravel()
        if abs(dy)<.5:raise ServoError('spine side is near horizontal')
        slope=float(dx/dy)
        residual=float(np.median(np.abs(edge-(x+slope*(rows-y)))))
        if residual>4.:raise ServoError('spine side residual above 4 px')
        lines.append((slope,residual))
    parallel_error=abs(float(np.degrees(np.arctan(lines[0][0])-np.arctan(lines[1][0]))))
    from cart_book_grasp_v3_spine_geometry import perspective_width
    perspective=perspective_width(rows,left,right)
    u,angle=visible_centerline(mask)
    if abs(u-(w-1)/2)>.18*w:raise ServoError('low-confidence spine too far from center')
    return dict(center_u=u,angle_deg=angle,width_px=width,span_px=span,
                parallel_error_deg=parallel_error,side_residual_px=[v[1] for v in lines],**perspective)


def inspect_candidates(raw,shape,geometry_validator=spine_geometry):
    raw=list(raw)
    records=[];eligible=[]
    for index,(score,mask,box) in enumerate(raw):
        record=dict(index=index,score=float(score) if np.isfinite(score) else None,
                    detector_box=np.asarray(box).tolist())
        records.append(record)
        if not np.isfinite(score) or score<.15:
            record['reason']='score below recall threshold 0.15';continue
        mask=np.asarray(mask)
        if mask.shape!=shape:
            record['reason']='mask shape mismatch';continue
        cleaned=clean_masks([(score,mask,box)],shape,dict(min_score=.15,
            min_area_px=max(100,int(np.prod(shape)*.003)),mask_nms_iou=.65,nms_min_area_ratio=.65))
        if not cleaned:
            record['reason']='empty or small connected mask';continue
        score,mask,box=cleaned[0]
        # Preserve observed holes/occlusions; contour cleanup must not create texture support.
        mask &= np.asarray(raw[index][1],bool)
        record.update(box=box,area_px=int(mask.sum()))
        try:
            geometry=geometry_validator(mask)
            if 'support_rows' in geometry:
                top,bottom=geometry['support_rows']
                mask=mask.copy();mask[:top]=False;mask[bottom+1:]=False
                yy,xx=np.nonzero(mask)
                box=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)]
                record['support_box']=box
            record.update(geometry=geometry,recall_geometry_valid=True)
            eligible.append(dict(index=index,score=score,mask=mask,box=box,geometry=geometry))
        except ServoError as error:
            record.update(recall_geometry_valid=False,reason=str(error))
        record['strict_score_valid']=score>=.5
        try:visible_centerline(mask);record['strict_centerline_valid']=True
        except ServoError as error:
            record['strict_centerline_valid']=False;record['strict_reason']=str(error)
        if score<.5:record['strict_reason']='score below original threshold 0.5'
    return records,eligible


def select_wrist_book(rgb,detector,*,directory=None,checkpoint=lambda:None,log=lambda event:None,boundary_selector=None):
    start=time.monotonic();report=dict(version='3.2',attempts=[],consensus=[],status='pending')
    folder=Path(directory) if directory is not None else None
    chosen=None
    def save():
        report['seconds']=time.monotonic()-start
        if folder is not None:
            folder.mkdir(parents=True,exist_ok=True)
            (folder/'wrist_detection.json').write_text(json.dumps(report,indent=2)+'\n')
    def attempt(prompt):
        checkpoint();began=time.monotonic()
        raw=list(detector(rgb) if prompt=='book' else detector(rgb,prompt=prompt))
        checkpoint()
        validator=getattr(boundary_selector,'mask_geometry_validator',spine_geometry)
        records,candidates=inspect_candidates(raw,rgb.shape[:2],validator)
        report['attempts'].append(dict(prompt=prompt,seconds=time.monotonic()-began,
                                       raw_count=len(raw),candidates=records))
        if folder is not None:
            folder.mkdir(parents=True,exist_ok=True)
            arrays={f'mask_{i}':np.asarray(d[1],np.uint8) for i,d in enumerate(raw)}
            np.savez_compressed(folder/f'wrist_detection_{prompt.replace(" ","_")}.npz',**arrays)
        save();return raw,candidates
    try:
        raw,_=attempt('book')
        # Whole-book masks may contain a second plane (the cover). They are
        # identity support only, even when their confidence is very high.
        books=clean_masks(raw,rgb.shape[:2],dict(min_score=.15,
            min_area_px=max(100,int(np.prod(rgb.shape[:2])*.003)),
            mask_nms_iou=.65,nms_min_area_ratio=.65))
        _,spines=attempt('book spine')
        accepted=[]
        for spine in spines:
            supported=False
            for index,(score,mask,box) in enumerate(books):
                intersection=np.count_nonzero(mask&spine['mask'])
                containment=float(intersection/max(1,spine['mask'].sum()))
                iou=float(intersection/max(1,np.count_nonzero(mask|spine['mask'])))
                # A narrow spine within a broad book is expected under an
                # oblique view: whole-mask IoU or equal centers are wrong gates.
                ok=containment>=.90
                report['consensus'].append(dict(book_index=index,spine_index=spine['index'],
                    spine_containment=containment,mask_iou=iou,accepted=ok))
                supported|=ok
            if supported or spine['score']>=.5:
                accepted.append((spine,supported))
        if accepted:
            # Prefer the explicit narrower surface over an overlapping mask
            # which includes that surface plus the cover. Never trim one side
            # by image direction: the cover can appear on either side.
            surfaces=[]
            for candidate,supported in accepted:
                broad=False
                for other,_ in accepted:
                    if other is candidate:continue
                    a,b=candidate['mask'],other['mask']
                    if b.sum()<.75*a.sum() and np.count_nonzero(a&b)/max(1,b.sum())>=.95:
                        broad=True;break
                if not broad:surfaces.append((candidate,supported))
            surfaces.sort(key=lambda item:abs(item[0]['geometry']['center_u']-(rgb.shape[1]-1)/2))
            spine,supported=surfaces[0];mask=spine['mask'];ys,_=np.nonzero(mask)
            chosen=dict(score=spine['score'],mask=mask,box=spine['box'],
                uv=[spine['geometry']['center_u'],float(ys.mean())],
                distance_px=abs(spine['geometry']['center_u']-(rgb.shape[1]-1)/2))
            report.update(status='selected',method='book_supported_spine' if supported else 'high_confidence_spine',
                          selected_box=spine['box'],selected_score=spine['score'],
                          selected_geometry=spine['geometry'],centerline_source='spine_mask')
        elif callable(getattr(boundary_selector,'recover',None)):
            report['gpt']={}
            chosen=boundary_selector.recover(rgb,report['gpt'],detector=detector,
                                             directory=folder,checkpoint=checkpoint)
            report.update(status='selected',method='gpt6_polygon_wrist',selected_box=chosen['box'],
                          selected_score=chosen['score'],selected_geometry=report['gpt']['selected_geometry'],
                          centerline_source='gpt_polygon_mask')
        else:
            if boundary_selector is None:
                from cart_book_grasp_v3_2_edges import select_boundary_pair
            else:
                select_boundary_pair=boundary_selector
            report['boundaries']={};checkpoint()
            try:chosen=select_boundary_pair(rgb,report['boundaries'])
            except ServoError as error:
                raise ServoError(f'Wrist book/spine recall lacks mask and centerline agreement; '
                                 f'boundary fallback failed: {error}; see wrist_detection.json') from error
            checkpoint()
            report.update(status='selected',method=report['boundaries'].get('method','paired_image_boundaries'),selected_box=chosen['box'],
                          semantic_identity_verified=False,centerline_source='paired_boundaries')
        chosen['tracking_mask']=spine_tracking_interior(chosen['mask'])
        report['tracking_area_px']=int(chosen['tracking_mask'].sum())
        if folder is not None:
            cv2.imwrite(str(folder/'spine_tracking_mask.png'),np.uint8(chosen['tracking_mask'])*255)
        save();log(dict(event='wrist_selection',method=report['method'],score=chosen['score']))
        return chosen,dict(mode='wrist_center',criterion=report['method'],
            image_center=[(rgb.shape[1]-1)/2,(rgb.shape[0]-1)/2],initial_detection=report)
    except BaseException as error:
        report.update(status='failed',reason=str(error));save();raise
    finally:
        if folder is not None:
            try:
                from cart_book_grasp_wrist_preview import write_preview
                write_preview(rgb,report,folder,chosen)
            except Exception as preview_error:
                # A report/UI error must never replace a perception exception
                # or alter the selected target and motion decision.
                print(f'首次感知可视化保存失败：{preview_error}')


def spine_tracking_interior(mask):
    """Select texture inside the spine, away from the cover seam and silhouette.

    Keep the middle 60% of each visible row plus a full 21px LK window's
    base-resolution clearance. Never enlarge the region for more features.
    """
    mask=np.asarray(mask,bool);h,w=mask.shape
    rows=np.flatnonzero(mask.any(axis=1));result=np.zeros_like(mask)
    left=mask[rows].argmax(axis=1);right=w-1-mask[rows,::-1].argmax(axis=1)
    for y,a,b in zip(rows,left,right):
        margin=max(3,int(np.ceil(.2*(b-a+1))))
        if a+margin<=b-margin:result[y,a+margin:b-margin+1]=mask[y,a+margin:b-margin+1]
    clearance=cv2.distanceTransform(np.pad(mask.astype(np.uint8),1),cv2.DIST_L2,5)[1:-1,1:-1]
    return result & (clearance>=11.)
