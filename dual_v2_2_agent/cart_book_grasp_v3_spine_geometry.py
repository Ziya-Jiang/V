"""Image-plane checks for a spine with perspective convergence/divergence."""
import numpy as np
from cart_book_grasp_v3_servo import ServoError


def perspective_width(rows,left,right):
    rows,left,right=(np.asarray(a,float) for a in (rows,left,right))
    widths=right-left
    if len(rows)<16 or not np.isfinite(widths).all() or np.min(widths)<8:
        raise ServoError('Spine boundaries cross or become too narrow')
    center=np.median(rows);fit=np.polyfit(rows-center,widths,1)
    ends=np.polyval(fit,np.array([rows.min(),rows.max()])-center)
    lo,hi=float(ends.min()),float(ends.max());median=float(np.median(widths))
    if lo<8 or hi/lo>1.8 or hi-lo>.6*median:
        raise ServoError('Spine perspective width change excessive')
    residual=float(np.percentile(abs(widths-np.polyval(fit,rows-center)),90))
    if residual>max(3.,.08*median):
        raise ServoError('Spine width varies discontinuously')
    return dict(width_start_px=float(ends[0]),width_end_px=float(ends[1]),
                width_ratio=hi/lo,width_residual_p90_px=residual,
                width_change_fraction=(hi-lo)/median)


def boundary_support(edge_image,gray,rows,xs):
    """Recheck merged Hough lines against pixels, including any merged gaps."""
    h,w=gray.shape;rows=np.asarray(rows,int);xs=np.rint(xs).astype(int)
    valid=(xs>=10)&(xs<w-10)&(rows>=0)&(rows<h)
    if not valid.all():raise ServoError('Boundary support outside image')
    hit=np.any([edge_image[rows,xs+d]>0 for d in range(-3,4)],axis=0)
    padded=np.r_[True,hit,True];indices=np.flatnonzero(padded)
    longest_gap=int(np.max(np.diff(indices)-1))
    fraction=float(hit.mean())
    delta=np.mean([gray[rows,xs+d].astype(float) for d in range(4,10)],axis=0)-np.mean(
        [gray[rows,xs-d].astype(float) for d in range(4,10)],axis=0)
    polarity=float(max(np.mean(delta>12),np.mean(delta<-12)))
    if fraction<.85 or longest_gap>max(10,int(.06*len(rows))):
        raise ServoError('Boundary lacks continuous edge support')
    if polarity<.75:raise ServoError('Boundary brightness transition inconsistent (possible text)')
    return dict(edge_support_fraction=fraction,longest_gap_px=longest_gap,contrast_polarity_fraction=polarity)
