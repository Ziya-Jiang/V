"""Local, direct spine-side measurement. No corners, descriptors or optical flow."""
import time
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError


def fit_side(rows, columns, cy):
    dx,dy,x,y=cv2.fitLine(np.column_stack([columns,rows]).astype(np.float32),
                         cv2.DIST_HUBER,0,.01,.01).ravel()
    if abs(dy)<.7:raise ServoError('Book side is too close to horizontal')
    slope=float(dx/dy)
    return np.array([float(x+slope*(cy-y)),slope])


class EdgeCenterlineLock:
    """Fit fresh left/right edges in narrow corridors around the last sides.

    The initial instance mask fixes which book is being followed. Each update
    samples image gradients anew; it never retains or matches texture points.
    Two independently supported sides are required; a single missing side is
    not hallucinated from the previous width.
    """
    def __init__(self,rgb,mask):
        if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3:
            raise ServoError('Invalid edge tracker RGB')
        self.shape=rgb.shape;self.h,self.w=rgb.shape[:2];self.cy=(self.h-1)/2
        self.small_h=max(1,self.h//4);self.y_scale=self.small_h/self.h
        mask=np.asarray(mask,bool)
        if mask.shape!=rgb.shape[:2]:raise ServoError('Book mask and image dimensions differ')
        rows=np.flatnonzero(mask.any(axis=1))
        left=mask[rows].argmax(axis=1);right=self.w-1-mask[rows,::-1].argmax(axis=1)
        good=(right-left>=6)&(left>2)&(right<self.w-3)
        rows,left,right=rows[good],left[good],right[good]
        if len(rows)<32 or np.ptp(rows)<40:
            raise ServoError('Too little visible spine height for two-edge tracking')
        self.boundaries=np.array([fit_side(rows,left,self.cy),fit_side(rows,right,self.cy)])
        self.initial_width=float(self.boundaries[1,0]-self.boundaries[0,0])
        if self.initial_width<6:raise ServoError('Spine edges are too close')
        # Use the visible vertical band; it does not require the book's ends.
        low=max(4.,float(np.percentile(rows,5)))
        high=min(self.h-5.,float(np.percentile(rows,95)))
        self.rows=np.linspace(low,high,min(96,max(32,int((high-low)/3)))).astype(np.float32)
        self.anchor=np.array([self.boundaries[:,0].mean(),self.cy])
        self.last_angle=float(np.degrees(np.arctan(self.boundaries[:,1].mean())))
        self.polarities=None
        # Confirm mask sides against the actual image, not just the mask shape.
        self.observe(rgb)
        self.initial_width=float(self.boundaries[1,0]-self.boundaries[0,0])
        self.anchor=np.array([self.boundaries[:,0].mean(),self.cy])

    def _side(self,response,x0,previous,radius):
        offsets=np.arange(2*np.floor((previous[0]-radius)/2),
                          2*np.ceil((previous[0]+radius)/2)+1,2,dtype=np.float32)-previous[0]
        previous_angle=np.degrees(np.arctan(previous[1]))
        angle_grid=np.arange(np.floor(previous_angle)-6,np.floor(previous_angle)+6.01,1.)
        angles=np.radians(angle_grid)
        slopes=np.tan(angles).astype(np.float32)
        xs=previous[0]+offsets[None,:,None]+slopes[:,None,None]*(self.rows[None,None,:]-self.cy)-x0
        ix=np.rint(xs).astype(int);iy=np.clip(np.rint((self.rows+.5)*self.y_scale-.5).astype(int),0,self.small_h-1)
        valid=(ix>=1)&(ix<response.shape[1]-1)
        values=response[iy[None,None,:],np.clip(ix,0,response.shape[1]-1)]
        # Capped evidence makes long edges beat a few very bright text strokes.
        evidence=np.minimum(values/35.,1.)*valid
        support=(values>=10)&valid
        score=evidence.mean(axis=2)-.004*np.abs(offsets)[None,:]-.007*np.abs(angle_grid-previous_angle)[:,None]
        coverage=support.mean(axis=2)
        score[coverage<.55]=-np.inf
        if not np.isfinite(score).any():raise ServoError('Spine boundary lacks continuous edge support')
        ai,oi=np.unravel_index(np.argmax(score),score.shape)
        if score[ai,oi]<.42:raise ServoError('Spine boundary contrast is insufficient')
        fitted=np.array([previous[0]+offsets[oi],slopes[ai]],float)
        # Subpixel peak interpolation along x, then robust line refinement.
        indices=ix[ai,oi];mask=support[ai,oi]
        local=np.clip(indices[:,None]+np.arange(-3,4)[None,:],1,response.shape[1]-2)
        peaks=response[iy[:,None],local]
        indices=local[np.arange(len(iy)),np.argmax(peaks,axis=1)]
        indices=np.clip(indices,1,response.shape[1]-2)
        a=response[iy,indices-1];b=response[iy,indices];d=response[iy,indices+1]
        denominator=a-2*b+d
        sub=np.divide(.5*(a-d),denominator,out=np.zeros_like(a),where=np.abs(denominator)>1e-6)
        sub=np.clip(sub,-.5,.5)
        refined=fit_side(self.rows[mask],indices[mask]+sub[mask]+x0,self.cy)
        if abs(refined[0]-fitted[0])<4 and abs(refined[1]-fitted[1])<.06:fitted=refined
        if np.ptp(self.rows[mask])<.6*np.ptp(self.rows):raise ServoError('Spine side support is too localized')
        return fitted,int(mask.sum()),float(evidence[ai,oi].mean())

    def observe(self,rgb):
        started=time.monotonic()
        if rgb.shape!=self.shape or rgb.dtype!=np.uint8:raise ServoError('Wrist RGB format changed')
        width=float(self.boundaries[1,0]-self.boundaries[0,0])
        radius=int(np.clip(width*.25,20,36))
        ends=self.boundaries[:,0,None]+self.boundaries[:,1,None]*(self.rows[[0,-1]]-self.cy)
        x0=max(0,int(np.floor(ends.min()))-radius-5);x1=min(self.w,int(np.ceil(ends.max()))+radius+6)
        if x1-x0<8:raise ServoError('Spine search region outside image')
        # RGB derivative retains isoluminant colour edges. Blur along y reduces
        # short text strokes; the ROI is only a narrow strip around this book.
        roi=cv2.resize(rgb[:,x0:x1],(x1-x0,self.small_h),interpolation=cv2.INTER_AREA).astype(np.float32)
        smooth=cv2.GaussianBlur(roi,(3,3),0)
        gradient=cv2.Sobel(smooth,cv2.CV_32F,1,0,ksize=3,scale=.25)
        magnitude=np.linalg.norm(gradient,axis=2) if self.polarities is None else None
        sides=[];counts=[];scores=[]
        for i,previous in enumerate(self.boundaries):
            response=magnitude
            if self.polarities is not None:
                # Preserve the local edge colour-transition direction, reducing
                # accidental switches from a side to opposite-polarity letters.
                response=np.maximum(np.einsum('ijk,k->ij',gradient,self.polarities[i]),0.)
            side,count,score=self._side(response,x0,previous,radius)
            sides.append(side);counts.append(count);scores.append(score)
        sides=np.array(sides);widths=(sides[1,0]-sides[0,0])+(sides[1,1]-sides[0,1])*(self.rows-self.cy)
        new_width=float(sides[1,0]-sides[0,0]);center=sides.mean(axis=0)
        angle=float(np.degrees(np.arctan(center[1])))
        if (widths.min()<6 or not .7<=new_width/width<=1.35 or not .35<=new_width/self.initial_width<=6
                or abs(center[0]-self.boundaries[:,0].mean())>radius
                or abs(angle-self.last_angle)>8
                or abs(np.degrees(np.arctan(sides[0,1]))-np.degrees(np.arctan(sides[1,1])))>20):
            raise ServoError('Spine edge pair width/angle/jump inconsistent')
        if self.polarities is None:
            polarities=[]
            for side in sides:
                ix=np.rint(side[0]+side[1]*(self.rows-self.cy)-x0).astype(int)
                iy=np.clip(np.rint((self.rows+.5)*self.y_scale-.5).astype(int),0,self.small_h-1)
                g=gradient[iy,np.clip(ix,0,gradient.shape[1]-1)]
                g=g[np.linalg.norm(g,axis=1)>10]
                v=np.median(g,axis=0)
                if not np.isfinite(v).all() or np.linalg.norm(v)<5:raise ServoError('Spine boundary polarity is ambiguous')
                polarities.append(v/np.linalg.norm(v))
            self.polarities=np.array(polarities)
        self.boundaries=sides;self.last_angle=angle
        y=np.array([0.,self.h-1.]);line=np.column_stack([center[0]+center[1]*(y-self.cy),y])
        edge_lines=np.array([np.column_stack([side[0]+side[1]*(y-self.cy),y]) for side in sides])
        return dict(uv=np.array([center[0],self.cy]),angle_deg=angle,line=line,edge_lines=edge_lines,
                    scale=new_width/self.initial_width,edge_support=counts,edge_scores=scores,
                    inference_seconds=time.monotonic()-started)
