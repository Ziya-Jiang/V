"""Read-only visual references for left hand/right claw; never grip-force evidence."""
import time
import cv2
import numpy as np
from .geometry import Rejected,finite,apply

MARKER_IDS={'left':7,'right':19}

def surface_reference(depth,k,d,polygon,minimum_fraction=.35):
    polygon=finite(polygon)
    if polygon.shape!=(4,2):raise Rejected('Expected four image points')
    mask=np.zeros(depth.shape,np.uint8);cv2.fillConvexPoly(mask,np.rint(polygon).astype(np.int32),255);mask=cv2.erode(mask,np.ones((3,3),np.uint8));total=int(np.count_nonzero(mask))
    if total<8:raise Rejected('Reference image region too small')
    values=depth[mask>0];values=values[np.isfinite(values)&(values>.12)&(values<2.0)]
    if len(values)<8 or len(values)/total<minimum_fraction:raise Rejected('Reference depth missing')
    median=float(np.median(values));mad=float(np.median(abs(values-median)))
    if mad>.012:raise Rejected('Mixed foreground/background depth')
    # Prefer the central patch to avoid taking a background median in a large ROI.
    center=polygon.mean(0);u,v=np.rint(center).astype(int);patch=depth[max(0,v-2):v+3,max(0,u-2):u+3];valid=patch[np.isfinite(patch)&(patch>.12)&(patch<2.0)]
    if len(valid)<3:raise Rejected('Reference center lacks depth')
    z=float(np.median(valid))
    if abs(z-median)>.025:raise Rejected('Center depth disagrees with selected surface')
    ray=cv2.undistortPoints(center.reshape(1,1,2).astype(float),finite(k,(3,3)),finite(d)).reshape(2);point=np.r_[ray*z,z]
    return point,{'depth_m':z,'depth_mad_m':mad,'valid_depth_fraction':len(values)/total}

class VisualFeedback:
    def __init__(self,mode='markers',marker_size_m=.03):
        if mode not in ('markers','manual'):raise Rejected('Unknown feedback mode')
        if not .015<=marker_size_m<=.08:raise Rejected('Marker size must be 15–80 mm')
        self.mode=mode;self.marker_size=marker_size_m;self.trackers={};self.previous={};self.last_sequence=None;self.last_stamp=None
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector=cv2.aruco.DetectorParameters_create();self.detector.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    def clear(self):self.trackers.clear();self.previous.clear();self.last_sequence=None;self.last_stamp=None
    def initialize_manual(self,role,bgr,box):
        if self.mode!='manual' or role not in MARKER_IDS:raise Rejected('Select manual mode and a known side first')
        x,y,w,h=finite(box,(4,));height,width=bgr.shape[:2]
        if min(w,h)<12 or w>200 or h>200 or x<0 or y<0 or x+w>width or y+h>height:raise Rejected('Select a 12–200 pixel box entirely on the visible hand/claw')
        tracker=cv2.TrackerCSRT_create();tracker.init(bgr,tuple(map(int,[x,y,w,h])));self.trackers[role]=tracker;self.previous.pop(role,None)
    def marker_polygons(self,bgr,k,d):
        corners,ids,_=cv2.aruco.detectMarkers(bgr,self.dictionary,parameters=self.detector);result={};ids=[] if ids is None else ids.ravel().tolist()
        for role,marker_id in MARKER_IDS.items():
            matches=[i for i,n in enumerate(ids) if n==marker_id]
            if len(matches)!=1:result[role]={'state':'not_visible' if not matches else 'duplicate_id','marker_id':marker_id};continue
            polygon=corners[matches[0]].reshape(4,2);edges=np.linalg.norm(polygon-np.roll(polygon,1,axis=0),axis=1)
            if edges.min()<8 or cv2.contourArea(polygon.astype(np.float32))<80:result[role]={'state':'marker_too_small','marker_id':marker_id};continue
            result[role]={'state':'candidate','polygon':polygon,'marker_id':marker_id}
        return result
    def analyze(self,data,now_mono,handles=None):
        now_mono=float(now_mono);stamp=float(data['stamp_s']);age=now_mono-float(data['monotonic_s']);seq=int(data['sequence']);base={'mode':self.mode,'image_sequence':seq,'stamp_s':stamp,'frame':'SJ4Z','sample_age_s':age,'hardware_output':False,'grasp_confirmed':False,'feedback_kind':'visual_surface_reference_not_calibrated_TCP','hands':{}}
        if not np.isfinite([age,stamp]).all() or not 0<=age<=1.0:
            self.clear();base['state']='stale_camera';return base
        if self.last_sequence is not None and (seq<=self.last_sequence or stamp<=self.last_stamp):
            self.clear();base['state']='reordered_camera';return base
        bgr=data['bgr'];depth=data['depth'];k=data['k'];d=data['d'];t=data['upper_T_camera'];self.last_sequence=seq;self.last_stamp=stamp
        candidates=self.marker_polygons(bgr,k,d) if self.mode=='markers' else {}
        if self.mode=='manual':
            for role in MARKER_IDS:
                tracker=self.trackers.get(role)
                if tracker is None:candidates[role]={'state':'selection_required'};continue
                ok,bbox=tracker.update(bgr)
                if not ok:self.trackers.pop(role,None);candidates[role]={'state':'tracking_lost'};continue
                x,y,w,h=bbox
                if x<0 or y<0 or x+w>bgr.shape[1] or y+h>bgr.shape[0] or min(w,h)<8:self.trackers.pop(role,None);candidates[role]={'state':'tracking_lost'};continue
                # The operator-selected center is a surface reference, not palm/TCP.
                half=min(w,h)*.2;cx,cy=x+w/2,y+h/2;polygon=np.array([[cx-half,cy-half],[cx+half,cy-half],[cx+half,cy+half],[cx-half,cy+half]])
                candidates[role]={'state':'candidate','polygon':polygon,'tracked_box':list(map(float,bbox))}
        for role,candidate in candidates.items():
            if candidate['state']!='candidate':self.previous.pop(role,None);base['hands'][role]=candidate;continue
            polygon=candidate.pop('polygon')
            try:
                point,quality=surface_reference(depth,k,d,polygon)
                if self.mode=='markers':
                    half=self.marker_size/2;obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],float);ok,rvec,tvec=cv2.solvePnP(obj,polygon.astype(float),k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok or tvec[2,0]<=0:raise Rejected('Marker pose unavailable')
                    projected,_=cv2.projectPoints(obj,rvec,tvec,k,d);reproj=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-polygon)**2,axis=1))))
                    if reproj>1.5 or abs(float(tvec[2,0])-quality['depth_m'])>.035:raise Rejected('Marker size/pose and depth disagree')
                    quality.update(reprojection_rms_px=reproj,pnp_depth_m=float(tvec[2,0]))
                upper=apply(t,point);previous=self.previous.get(role)
                if previous is not None and (stamp-previous[0]>.9 or np.linalg.norm(upper-previous[1])>.06):
                    self.previous.pop(role,None)
                    if self.mode=='manual':self.trackers.pop(role,None)
                    raise Rejected('Reference jumped; reject/reselect rather than extrapolate')
                stable=1 if previous is None else previous[2]+1;self.previous[role]=(stamp,upper,stable)
                item={**candidate,'state':'tracking','polygon':polygon.tolist(),'position_upper_m':upper.tolist(),'quality':quality,'stable_frames':stable,'temporally_stable':stable>=4,'identity_source':'coded_marker' if self.mode=='markers' else 'operator_selected_ROI','identity_reverified':self.mode=='markers'}
                if handles is not None and handles.get('temporal_stable') is True and handles.get('frame')=='SJ4Z' and abs(stamp-float(handles['stamp']))<.8:
                    goal=finite(handles['handles'],(2,3))[0 if role=='left' else 1];delta=goal-upper;item['reference_to_handle_delta_m']=delta.tolist();item['reference_to_handle_distance_m']=float(np.linalg.norm(delta));item['delta_is_tcp_error']=False
                base['hands'][role]=item
            except (Rejected,ValueError,cv2.error) as error:
                self.previous.pop(role,None)
                if self.mode=='manual':self.trackers.pop(role,None)
                base['hands'][role]={'state':'depth_or_tracking_unreliable','reason':str(error)}
        base['state']='observing';return base

def annotate(bgr,report):
    out=bgr.copy()
    for role,item in report.get('hands',{}).items():
        color=(80,255,120) if role=='left' else (0,210,255)
        if 'polygon' in item:
            p=np.rint(item['polygon']).astype(np.int32);cv2.polylines(out,[p],True,color,2);x,y=np.rint(p.mean(0)).astype(int);label=role+' visual reference';cv2.putText(out,label,(max(5,x-70),max(35,y-12)),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1)
    cv2.rectangle(out,(0,0),(out.shape[1],28),(20,20,20),-1);cv2.putText(out,'HAND / CLAW VISUAL FEEDBACK - NO ROBOT COMMANDS',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(240,240,240),1)
    return out
