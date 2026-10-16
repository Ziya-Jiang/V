"""V3.4 wrist fallback: join short segments, then verify actual outer edges.

No semantic identity is inferred. All joins are hypotheses until supported by
pixels over their full span. Lab chroma preserves equal-luminance boundaries.
"""
import time
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_spine_geometry import perspective_width


def _fit(group, cy):
    rows = [np.linspace(s['top'], s['bottom'], max(3, int((s['bottom']-s['top'])/4))) for s in group]
    xs = np.concatenate([s['u']+s['slope']*(y-cy) for s,y in zip(group, rows)])
    rows = np.concatenate(rows)
    dx,dy,x,y = cv2.fitLine(np.column_stack([xs,rows]).astype(np.float32), cv2.DIST_HUBER,0,.01,.01).ravel()
    slope = float(dx/dy)
    return dict(u=float(x+slope*(cy-y)), slope=slope,
                top=min(s['top'] for s in group), bottom=max(s['bottom'] for s in group),
                segment_count=len(group))


def join_segments(segments, height):
    """Grow local collinear groups; neither direction alone nor length suffices."""
    cy=(height-1)/2; hypotheses=[]
    for seed in sorted(segments,key=lambda s:s['bottom']-s['top'],reverse=True):
        group=[seed]; line=seed
        for _ in range(5):
            additions=[]
            for candidate in segments:
                if any(candidate is member for member in group):continue
                gap=max(0,max(candidate['top'],line['top'])-min(candidate['bottom'],line['bottom']))
                rows=np.array([candidate['top'],candidate['bottom']])
                residual=abs(candidate['u']-line['u']+(candidate['slope']-line['slope'])*(rows-cy))
                if gap<=.16*height and abs(candidate['slope']-line['slope'])<=.12 and residual.max()<=4:
                    additions.append(candidate)
            if not additions:break
            group+=additions;line=_fit(group,cy)
        if line['bottom']-line['top']<.3*height:continue
        if any(abs(line['u']-o['u'])<3 and abs(line['slope']-o['slope'])<.025 for o in hypotheses):continue
        hypotheses.append(dict(line,segment_count=len(group)))
    return hypotheses


def color_boundary_support(edges, lab, rows, xs):
    """Require continuous edge evidence and a persistent *broad* color step.

    A text stroke can have a strong local edge but usually has paper on both
    sides of the wide strip. Two strip scales must agree in polarity/channel.
    """
    h,w=edges.shape;rows=np.asarray(rows,int);xs=np.rint(xs).astype(int)
    if not ((xs>=17)&(xs<w-17)&(rows>=0)&(rows<h)).all():
        raise ServoError('Boundary support outside image')
    hit=np.any([edges[rows,xs+d]>0 for d in range(-3,4)],axis=0)
    longest=int(np.max(np.diff(np.flatnonzero(np.r_[True,hit,True]))-1))
    fraction=float(hit.mean())
    if fraction<.85 or longest>max(10,int(.06*len(rows))):
        raise ServoError('Joined boundary lacks continuous pixel support')
    steps=[]
    for offsets in (range(4,10),range(10,17)):
        steps.append(np.mean([lab[rows,xs+d].astype(float) for d in offsets],axis=0)-
                     np.mean([lab[rows,xs-d].astype(float) for d in offsets],axis=0))
    thresholds=np.array([12.,3.,4.])
    positive=np.mean((steps[0]>thresholds)&(steps[1]>thresholds),axis=0)
    negative=np.mean((steps[0]<-thresholds)&(steps[1]<-thresholds),axis=0)
    support=np.maximum(positive,negative);channel=int(np.argmax(support))
    if support[channel]<.75:
        raise ServoError('Broad color transition inconsistent (possible internal text)')
    return dict(edge_support_fraction=fraction,longest_gap_px=longest,
                broad_color_support=float(support[channel]),color_channel=['L','a','b'][channel],
                color_polarity=1 if positive[channel]>=negative[channel] else -1)



def _band_profile(mask):
    valid=mask.any(axis=1)
    return dict(valid=valid, left=mask.argmax(axis=1),
                right=mask.shape[1]-1-mask[:,::-1].argmax(axis=1),
                rows=int(valid.sum()))


def _duplicate_metrics(a, b):
    """Compare both sides over actual shared rows, not just center distance.

    Partial views need not have equal full-image area. Require substantial
    common extent, close corresponding edges throughout it, and high band IoU.
    """
    common=a['valid'] & b['valid']; n=int(common.sum())
    fraction=n/max(1,min(a['rows'],b['rows']))
    result=dict(common_rows=n,shorter_overlap_fraction=float(fraction),duplicate=False)
    if n<.2*len(common) or fraction<.6:return result
    al,ar=a['left'][common],a['right'][common]
    bl,br=b['left'][common],b['right'][common]
    width=np.minimum(ar-al+1,br-bl+1)
    # Five pixels is an absolute ceiling, not a percentage of a wide cover.
    tolerance=min(5.,.08*float(np.min(width)))
    deviations=[float(np.max(abs(al-bl))),float(np.max(abs(ar-br)))]
    intersection=np.maximum(0,np.minimum(ar,br)-np.maximum(al,bl)+1)
    union=(ar-al+1)+(br-bl+1)-intersection
    iou=float(intersection.sum()/union.sum())
    result.update(side_max_error_px=deviations,side_tolerance_px=tolerance,
                  common_band_iou=iou,duplicate=max(deviations)<=tolerance and iou>=.9)
    return result


def deduplicate_pairs(candidates, diagnostics):
    """Keep the longest supported original fit; do not invent a union mask.

    Complete-link grouping prevents a chain of slightly shifted bands from
    absorbing a genuinely different spine. Retain all edge identities so that
    the later shared-boundary ambiguity check still catches adjacent bands.
    """
    profiles=[_band_profile(c[1]) for c in candidates]
    order=sorted(range(len(candidates)),key=lambda i:(-profiles[i]['rows'],candidates[i][0],i))
    groups=[]; comparisons=[]
    for i in order:
        for group in groups:
            equivalent=True
            for j in group:
                metrics=_duplicate_metrics(profiles[i],profiles[j])
                comparisons.append(dict(pair_indices=[i,j],**metrics))
                if not metrics['duplicate']:
                    equivalent=False;break
            if equivalent:
                group.append(i);break
        else:groups.append([i])
    result=[]
    for group in groups:
        representative=candidates[group[0]]
        identities=set().union(*(candidates[i][3] for i in group))
        result.append((*representative[:3],identities))
    diagnostics.update(candidate_groups=[dict(representative_pair_index=g[0],pair_indices=g) for g in groups],
                       duplicate_comparisons=comparisons,distinct_candidate_count=len(groups))
    return result


def select_boundary_pair(rgb, diagnostics):
    from cart_book_grasp_v3_2_selection import spine_geometry
    started=time.monotonic()
    h,w=rgb.shape[:2];cy=(h-1)/2;cx=(w-1)/2
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);lab=cv2.cvtColor(rgb,cv2.COLOR_RGB2LAB)
    edges=cv2.Canny(cv2.GaussianBlur(gray,(3,3),.6),50,130)
    for channel in (1,2):
        edges=np.maximum(edges,cv2.Canny(cv2.GaussianBlur(lab[:,:,channel],(3,3),.6),15,35))
    raw=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(gray)[0]
    diagnostics.update(method='joined_color_boundaries_v3_4',raw_segments=0 if raw is None else len(raw),
                       segments=[],edges=[],pairs=[],rejected_pairs=[],semantic_identity_verified=False)
    try:
        if raw is None:raise ServoError('No wrist image boundary segments')
        segments=[]
        for x0,y0,x1,y1 in raw[:,0]:
            if abs(y1-y0)<max(12,.035*h):continue
            slope=float((x1-x0)/(y1-y0));u=float(x0+slope*(cy-y0))
            if abs(slope)>.7 or abs(u-cx)>.28*w:continue
            top,bottom=sorted([float(y0),float(y1)])
            segments.append(dict(u=u,slope=slope,top=top,bottom=bottom))
        # Bound pathological texture cost; fail explicitly rather than truncate
        # and inadvertently remove the correct object's outside boundaries.
        if len(segments)>600:raise ServoError('Excessive wrist boundary texture; selection ambiguous')
        diagnostics['segments']=segments
        lines=join_segments(segments,h);diagnostics['edges']=lines
        candidates=[]
        for i,left in enumerate(lines):
            for j,right in enumerate(lines):
                if left['u']>=right['u']:continue
                record=dict(left_index=i,right_index=j)
                try:
                    width=right['u']-left['u'];center=(right['u']+left['u'])/2
                    if not max(12,.025*w)<=width<=.2*w:raise ServoError('Pair width outside range')
                    if abs(center-cx)>.15*w:raise ServoError('Pair too far from image center')
                    top=int(np.ceil(max(left['top'],right['top'])));bottom=int(min(left['bottom'],right['bottom']))
                    if bottom-top<.3*h or not top<=cy<=bottom:
                        raise ServoError('Pair lacks common extent across image center row')
                    rows=np.arange(top,bottom+1)
                    lx=np.rint(left['u']+left['slope']*(rows-cy)).astype(int)
                    rx=np.rint(right['u']+right['slope']*(rows-cy)).astype(int)
                    perspective_width(rows,lx,rx)
                    ls=color_boundary_support(edges,lab,rows,lx)
                    rs=color_boundary_support(edges,lab,rows,rx)
                    mask=np.zeros((h,w),bool)
                    for y,a,b in zip(rows,lx,rx):mask[y,a:b+1]=True
                    geometry=spine_geometry(mask)
                except ServoError as error:
                    diagnostics['rejected_pairs'].append(dict(record,reason=str(error)));continue
                distance=abs(geometry['center_u']-cx)
                diagnostics['pairs'].append(dict(record,center_distance_px=distance,geometry=geometry,
                                                 left_support=ls,right_support=rs))
                candidates.append((distance,mask,geometry,{i,j}))
        if not candidates:raise ServoError('No supported joined wrist outer-boundary pair')
        candidates=deduplicate_pairs(candidates,diagnostics)
        candidates.sort(key=lambda c:c[0])
        if len(candidates)>1 and candidates[1][0]-candidates[0][0]<max(5.,.006*w):
            raise ServoError('Wrist boundary selection ambiguous between multiple pairs')
        for other in candidates[1:]:
            # A central gap can be wider than the neighboring books. Width
            # ratios cannot establish which of two shared-edge bands is solid.
            if candidates[0][3]&other[3]:
                raise ServoError('Wrist boundary selection ambiguous with an adjacent band')
        distance,mask,geometry,_=candidates[0];ys,xs=np.nonzero(mask)
        chosen=dict(mask=mask,box=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],
                    score=None,uv=[geometry['center_u'],float(ys.mean())],distance_px=distance)
        diagnostics.update(selected_box=chosen['box'],selected_geometry=geometry)
        return chosen
    finally:
        diagnostics['seconds']=time.monotonic()-started


def visible_spine_geometry(mask):
    """Use a long, center-crossing observed region when end geometry dominates.

    Only the global perspective-width rejection is recoverable here. Do not
    excuse missing rows or abrupt changes of either side inside the spine.
    """
    from cart_book_grasp_v3_2_selection import spine_geometry
    try:return spine_geometry(mask)
    except ServoError as error:
        if str(error)!='Spine perspective width change excessive':raise
        original_reason=str(error)
    h,w=mask.shape;rows=np.flatnonzero(mask.any(axis=1))
    if len(rows)<.95*(rows[-1]-rows[0]+1):
        raise ServoError('Spine has discontinuous row support')
    left=mask[rows].argmax(axis=1).astype(float)
    right=(w-1-mask[rows,::-1].argmax(axis=1)).astype(float)
    span=rows[-1]-rows[0]
    central=(rows>=rows[0]+.05*span)&(rows<=rows[-1]-.05*span)
    limit=max(4.,.06*float(np.median(right[central]-left[central])))
    for edge in (left[central],right[central],(right-left)[central]):
        smooth=np.median(np.lib.stride_tricks.sliding_window_view(edge,5),axis=1)
        if np.max(np.abs(np.diff(smooth)))>limit:
            raise ServoError('Spine boundary jumps within central observed region')
    for trim in (.05,.10,.15,.20):
        top=int(np.ceil(rows[0]+trim*span));bottom=int(np.floor(rows[-1]-trim*span))
        if not top<=(h-1)/2<=bottom or bottom-top<.3*h:continue
        local=mask.copy();local[:top]=False;local[bottom+1:]=False
        try:geometry=spine_geometry(local)
        except ServoError:continue
        geometry.update(support_rows=[top,bottom],fit_mode='central_observed_region',
                        retained_height_fraction=(bottom-top)/span,
                        full_extent_rejection=original_reason)
        return geometry
    raise ServoError(original_reason+'; no continuous center-crossing support region')


# Optional selector capability; old V3.2/V3.3 selectors retain their own gates.
select_boundary_pair.mask_geometry_validator=visible_spine_geometry
