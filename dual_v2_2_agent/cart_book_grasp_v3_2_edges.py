"""Conservative one-frame spine boundary fallback; no semantic identity claim."""
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_spine_geometry import perspective_width,boundary_support


def select_boundary_pair(rgb,diagnostics):
    from cart_book_grasp_v3_2_selection import spine_geometry
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);h,w=gray.shape;cy=(h-1)/2;cx=(w-1)/2
    edges=cv2.Canny(cv2.GaussianBlur(gray,(3,3),.6),50,130)
    raw=cv2.HoughLinesP(edges,1,np.pi/720,threshold=max(25,int(.08*h)),
                       minLineLength=max(24,int(.25*h)),maxLineGap=max(8,int(.04*h)))
    diagnostics.update(method='paired_image_boundaries',raw_lines=0 if raw is None else len(raw),
                       edges=[],pairs=[],rejected_pairs=[],semantic_identity_verified=False)
    if raw is None:raise ServoError('No long wrist image boundaries')
    lines=[]
    for x0,y0,x1,y1 in raw[:,0]:
        if abs(y1-y0)<.25*h:continue
        slope=float(x1-x0)/float(y1-y0);u=float(x0+slope*(cy-y0))
        if abs(slope)>.7 or abs(u-cx)>.28*w:continue
        top,bottom=sorted([int(y0),int(y1)])
        ys=np.arange(top,bottom+1);xs=np.rint(u+slope*(ys-cy)).astype(int)
        valid=(xs>=10)&(xs<w-10);ys,xs=ys[valid],xs[valid]
        if len(ys)<.25*h:continue
        left=np.mean([gray[ys,xs-d].astype(float) for d in range(4,10)],axis=0)
        right=np.mean([gray[ys,xs+d].astype(float) for d in range(4,10)],axis=0)
        contrast=float(np.median(abs(right-left)));support=float(np.mean(abs(right-left)>12))
        # Text strokes can form long Hough lines; a boundary needs a broad,
        # persistent intensity step, measured away from the stroke itself.
        if contrast<15 or support<.65:continue
        lines.append(dict(u=u,slope=slope,top=top,bottom=bottom,contrast=contrast,support=support))
    lines.sort(key=lambda x:x['bottom']-x['top'],reverse=True)
    unique=[]
    for line in lines:
        duplicate=None
        for other in unique:
            if (abs(line['u']-other['u'])<5 and abs(line['slope']-other['slope'])<.025
                    and max(line['top'],other['top'])<=min(line['bottom'],other['bottom'])+.04*h):
                duplicate=other;break
        if duplicate is not None:
            # Preserve the observed extent of collinear fragments, instead
            # of discarding the bottom half when the top segment is longer.
            duplicate['top']=min(duplicate['top'],line['top'])
            duplicate['bottom']=max(duplicate['bottom'],line['bottom'])
            continue
        unique.append(line)
        if len(unique)>=40:break
    diagnostics['edges']=unique
    candidates=[]
    for i,left in enumerate(unique):
        for j,right in enumerate(unique):
            if left['u']>=right['u']:continue
            top=max(left['top'],right['top']);bottom=min(left['bottom'],right['bottom'])
            width=right['u']-left['u'];center=(right['u']+left['u'])/2
            if not max(12,.025*w)<=width<=.2*w or abs(center-cx)>.15*w:continue
            if bottom-top<.3*h or not top<=cy<=bottom:continue
            rows=np.arange(top,bottom+1)
            lx=np.rint(left['u']+left['slope']*(rows-cy)).astype(int)
            rx=np.rint(right['u']+right['slope']*(rows-cy)).astype(int)
            try:
                perspective_width(rows,lx,rx)
                left_support=boundary_support(edges,gray,rows,lx)
                right_support=boundary_support(edges,gray,rows,rx)
            except ServoError as error:
                diagnostics['rejected_pairs'].append(dict(left_index=i,right_index=j,reason=str(error)))
                continue
            mask=np.zeros((h,w),bool)
            for y,a,b in zip(rows,lx,rx):
                if 0<a<b<w-1:mask[y,a:b+1]=True
            try:geometry=spine_geometry(mask)
            except ServoError as error:
                diagnostics['rejected_pairs'].append(dict(left_index=i,right_index=j,reason=str(error)))
                continue
            distance=abs(geometry['center_u']-cx)
            diagnostics['pairs'].append(dict(left_index=i,right_index=j,center_distance_px=distance,geometry=geometry,left_support=left_support,right_support=right_support))
            candidates.append((distance,mask,geometry,{i,j}))
    if not candidates:raise ServoError('No supported perspective-consistent wrist boundary pair')
    candidates.sort(key=lambda item:item[0])
    if len(candidates)>1 and candidates[1][0]-candidates[0][0]<max(5.,.006*w):
        raise ServoError('Wrist boundary selection ambiguous between multiple pairs')
    for other in candidates[1:]:
        widths=[candidates[0][2]['width_px'],other[2]['width_px']]
        if candidates[0][3]&other[3] and min(widths)/max(widths)>.5:
            # The space between two books also has two parallel edges. When
            # neighboring plausible bands share those edges, do not label
            # the central gap as a book just because it is most centered.
            raise ServoError('Wrist boundary selection ambiguous with an adjacent band')
    distance,mask,geometry,_=candidates[0];ys,xs=np.nonzero(mask)
    chosen=dict(mask=mask,box=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],
                score=None,uv=[geometry['center_u'],float(np.mean(ys))],distance_px=distance)
    diagnostics.update(selected_box=chosen['box'],selected_geometry=geometry)
    return chosen
