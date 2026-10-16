"""Fixed-identity textured planar patch features; no robot control or text detector."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageOps


def read_image(path):
    with Image.open(path) as im:
        return cv2.cvtColor(np.asarray(ImageOps.exif_transpose(im).convert('RGB')), cv2.COLOR_RGB2BGR)


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def gray(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image


def project(points, H):
    return cv2.perspectiveTransform(np.float32(points).reshape(-1, 1, 2), H).reshape(-1, 2)


def detector():
    return cv2.SIFT_create(nfeatures=3500, contrastThreshold=.015, edgeThreshold=15)


def hull_area(points):
    return abs(cv2.contourArea(cv2.convexHull(np.float32(points)))) if len(points) >= 3 else 0.


def initialize(image_path, config, out):
    out = Path(out)
    if (out / 'model.json').exists():
        raise ValueError('Template exists; use a new directory to avoid changing point identities.')
    image = read_image(image_path)
    polygon = np.float32(config['patch_polygon'])
    h, w = image.shape[:2]
    if polygon.shape != (4, 2) or not np.isfinite(polygon).all():
        raise ValueError('patch_polygon needs four finite vertices in perimeter order.')
    if np.any(polygon < 0) or np.any(polygon[:, 0] >= w) or np.any(polygon[:, 1] >= h):
        raise ValueError('Patch must lie inside EXIF-oriented image.')
    if not cv2.isContourConvex(polygon) or hull_area(polygon) < 256:
        raise ValueError('Patch must be convex and have area >= 256 pixels^2.')
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, np.int32(polygon), 255)
    kp, desc = detector().detectAndCompute(gray(image), mask)
    if desc is None or len(kp) < 12:
        raise ValueError('insufficient_features: need a larger textured planar patch.')
    points = np.float32([k.pt for k in kp])
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
    candidates = [i for i, q in enumerate(points) if distance[int(q[1]), int(q[0])] >= 6]
    candidates.sort(key=lambda i: (-kp[i].response, points[i, 1], points[i, 0]))
    selected = []
    count = min(16, max(4, int(config.get('point_count', 8))))
    spacing = max(6, np.sqrt(hull_area(polygon)) * .10)
    for i in candidates:
        if not selected or np.min(np.linalg.norm(points[selected] - points[i], axis=1)) >= spacing:
            selected.append(i)
        if len(selected) == count:
            break
    if len(selected) < 4 or hull_area(points[selected]) / hull_area(polygon) < .08:
        raise ValueError('insufficient_features: need spatially distributed interior texture.')
    selected.sort(key=lambda i: (points[i, 1], points[i, 0]))
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / 'reference.png'), image)
    np.savez_compressed(out / 'features.npz', points=points, descriptors=desc,
                        anchors=points[selected], polygon=polygon)
    digest = hashlib.sha256(image.tobytes()+polygon.tobytes()+points[selected].tobytes()).hexdigest()[:20]
    meta = dict(config, template_id=digest, image_size=[w, h], point_count=len(selected),
                coordinate_system='EXIF-oriented pixels; origin top-left; x right; y down',
                reference_source=str(Path(image_path).resolve()), backend='planar-sift-lk-v1')
    write_json(out / 'model.json', meta)
    return meta


class Tracker:
    """Reuse for ordered frames; create separate instances for unrelated images."""
    def __init__(self, model_dir, redetect_every=5):
        root = Path(model_dir)
        self.meta = json.loads((root / 'model.json').read_text())
        with np.load(root / 'features.npz', allow_pickle=False) as z:
            self.points, self.desc, self.anchors, self.polygon = [z[k].copy() for k in ('points','descriptors','anchors','polygon')]
        self.reference = gray(read_image(root / 'reference.png'))
        self.sift = detector()
        self.redetect_every = max(1, int(redetect_every))
        self.prev = self.flow_ref = self.flow_cur = None
        self.frame_index = 0

    def _detect(self, current):
        kp, desc = self.sift.detectAndCompute(current, None)
        if desc is None or len(kp) < 2:
            return None
        matcher = cv2.BFMatcher(cv2.NORM_L2)
        forward = matcher.knnMatch(self.desc, desc, k=2)
        reverse = matcher.knnMatch(desc, self.desc, k=2)
        reverse_good = {a.queryIdx:a.trainIdx for pair in reverse if len(pair)==2
                        for a,b in [pair] if a.distance < .75*b.distance}
        matches = [a for pair in forward if len(pair)==2 for a,b in [pair]
                   if a.distance < .75*b.distance and reverse_good.get(a.trainIdx)==a.queryIdx]
        unique, used_a, used_b = [], set(), set()
        for m in sorted(matches, key=lambda m:m.distance):
            a = tuple(np.round(self.points[m.queryIdx]).astype(int))
            b = tuple(np.round(kp[m.trainIdx].pt).astype(int))
            if a not in used_a and b not in used_b:
                unique.append(m)
                used_a.add(a)
                used_b.add(b)
        if len(unique) < 10:
            return None
        return self.points[[m.queryIdx for m in unique]], np.float32([kp[m.trainIdx].pt for m in unique])

    def _flow(self, current):
        if self.prev is None or self.prev.shape != current.shape or self.flow_cur is None or len(self.flow_cur)<10:
            return None
        p = self.flow_cur.reshape(-1,1,2).astype(np.float32)
        kwargs = dict(winSize=(21,21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
        q, ok, _ = cv2.calcOpticalFlowPyrLK(self.prev, current, p, None, **kwargs)
        if q is None:
            return None
        back, ok_back, _ = cv2.calcOpticalFlowPyrLK(current, self.prev, q, None, **kwargs)
        if back is None:
            return None
        good = (ok.ravel()>0) & (ok_back.ravel()>0)
        good &= np.linalg.norm(back.reshape(-1,2)-p.reshape(-1,2),axis=1)<1.
        good &= np.isfinite(q.reshape(-1,2)).all(axis=1)
        return self.flow_ref[good], q.reshape(-1,2)[good]

    def _fit(self, pairs, current):
        if pairs is None or len(pairs[0])<10:
            return None, 'too_few_correspondences'
        a,b = pairs
        cv2.setRNGSeed(7)
        H, mask = cv2.findHomography(a,b,cv2.RANSAC,3.,maxIters=3000,confidence=.995)
        if H is None or mask is None or not np.isfinite(H).all() or np.linalg.cond(H)>1e10:
            return None, 'homography_failed'
        keep = mask.ravel().astype(bool)
        ar,br = a[keep],b[keep]
        ratio = float(keep.mean())
        if len(ar)<10 or ratio<.55:
            return None, 'insufficient_inliers'
        coverage = hull_area(ar)/hull_area(self.polygon)
        if coverage<.10:
            return None, 'poor_spatial_coverage'
        errors = np.linalg.norm(project(ar,H)-br,axis=1)
        if np.median(errors)>1.8 or np.percentile(errors,90)>3.:
            return None, 'large_reprojection_error'
        quad = project(self.polygon,H)
        h,w = current.shape
        if (not np.isfinite(quad).all() or not cv2.isContourConvex(quad.astype(np.float32))
                or cv2.contourArea(quad,oriented=True)*cv2.contourArea(self.polygon,oriented=True)<=0):
            return None, 'invalid_projected_patch'
        if not .025<hull_area(quad)/hull_area(self.polygon)<40:
            return None, 'scale_out_of_range'
        if np.any(quad<1) or np.any(quad[:,0]>=w-1) or np.any(quad[:,1]>=h-1):
            return None, 'patch_out_of_frame'
        metrics = dict(matches=len(a),inliers=len(ar),inlier_ratio=ratio,
                       reference_coverage=coverage,median_reprojection_px=float(np.median(errors)))
        return (H,ar,br,metrics),None

    def _correlations(self,current,H):
        x0,y0 = np.floor(self.polygon.min(axis=0)).astype(int)
        x1,y1 = np.ceil(self.polygon.max(axis=0)).astype(int)
        T = np.array([[1.,0,-x0],[0,1.,-y0],[0,0,1.]])
        warped = cv2.warpPerspective(current,T@np.linalg.inv(H),(x1-x0+1,y1-y0+1))
        ref = self.reference[y0:y1+1,x0:x1+1]
        scores = []
        for anchor in self.anchors:
            x,y = np.round(anchor-[x0,y0]).astype(int)
            a = ref[y-5:y+6,x-5:x+6].astype(float).ravel()
            b = warped[y-5:y+6,x-5:x+6].astype(float).ravel()
            if len(a)!=121 or len(b)!=121 or a.std()<3 or b.std()<3:
                scores.append(-1.)
            else:
                a -= a.mean()
                b -= b.mean()
                scores.append(float(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))))
        return scores

    def _support_distances(self,inlier_reference):
        hull = cv2.convexHull(np.float32(inlier_reference))
        return np.array([cv2.pointPolygonTest(hull,tuple(float(v) for v in p),True)
                         for p in self.anchors])

    def process(self,image,timestamp=None):
        start = time.perf_counter()
        cur = gray(image)
        H, metrics, reason, accepted = None, {}, 'not_found', None
        sources = ['sift'] if self.prev is None or self.frame_index%self.redetect_every==0 else ['lk','sift']
        for source in sources:
            fit,reason = self._fit(self._flow(cur) if source=='lk' else self._detect(cur),cur)
            if fit is None:
                continue
            candidate,a,b,m = fit
            corr = self._correlations(cur,candidate)
            support = self._support_distances(a)
            # Low residual on a small cluster cannot justify extrapolating a
            # projective transform to distant anchors, even if NCC is high.
            valid = (np.array(corr)>=.65) & (support>=-1.)
            if valid.sum()<4 or hull_area(self.anchors[valid])/hull_area(self.polygon)<.06:
                reason = 'insufficient_verified_anchors'
                continue
            accepted = candidate,a,b,m,corr,valid,source,support
            break
        if accepted is not None:
            H,a,b,metrics,correlations,valid,source,support = accepted
            xy = project(self.anchors,H)
            self.prev,self.flow_ref,self.flow_cur = cur.copy(),a.copy(),b.copy()
            features = [dict(id=f'P{i+1}',xy=p.astype(float).tolist() if ok else None,
                             valid=bool(ok),patch_ncc=score,support_margin_px=float(margin),
                             measurement='homography_projection_with_texture_check')
                        for i,(p,score,ok,margin) in enumerate(zip(xy,correlations,valid,support))]
            status,reason = ('ok' if valid.all() else 'partial'),None
        else:
            self.prev = self.flow_ref = self.flow_cur = None
            source,status = None,'lost'
            features = [dict(id=f'P{i+1}',xy=None,valid=False,patch_ncc=None,support_margin_px=None,measurement=None)
                        for i in range(len(self.anchors))]
        result = dict(schema_version=1,template_id=self.meta['template_id'],frame_index=self.frame_index,
                      timestamp=timestamp,image_size=[cur.shape[1],cur.shape[0]],
                      coordinate_system=self.meta['coordinate_system'],status=status,reason=reason,
                      tracking_valid=accepted is not None,control_ready=False,method=source,features=features,
                      s_px=[v for f in features for v in (f['xy'] if f['valid'] else [None,None])],
                      valid_mask=[f['valid'] for f in features for _ in range(2)],
                      homography_reference_to_frame=H.tolist() if H is not None else None,
                      metrics=metrics,processing_ms=(time.perf_counter()-start)*1000)
        self.frame_index += 1
        return result


def annotate(image,result,title=''):
    out = image.copy()
    scale = .45
    colors = [(255,180,0),(0,140,255),(220,40,220),(70,210,70),
              (30,30,240),(255,140,140),(70,230,230),(210,160,60)]
    visible = [(i,f) for i,f in enumerate(result['features']) if f['valid']]
    # Put labels beside the entire feature cluster; never paint large text over
    # the small texture that the tracker is measuring.
    positions = {}
    ordered = sorted(visible,key=lambda item:item[1]['xy'][0])
    if ordered:
        xy = np.array([f['xy'] for _,f in ordered])
        left,right = ordered[:len(ordered)//2],ordered[len(ordered)//2:]
        for side,items in [('left',left),('right',right)]:
            items = sorted(items,key=lambda item:item[1]['xy'][1])
            label_x = int(xy[:,0].min()-50 if side=='left' else xy[:,0].max()+18)
            label_x = int(np.clip(label_x,2,out.shape[1]-40))
            start_y = int(xy[:,1].mean()-(len(items)-1)*9)
            start_y = int(np.clip(start_y,18,max(18,out.shape[0]-18*len(items))))
            for j,(i,f) in enumerate(items):
                positions[i] = (label_x,start_y+j*18)
    for i,f in visible:
        p = tuple(np.round(f['xy']).astype(int))
        c = colors[i%len(colors)]
        pos = positions[i]
        line_end = (pos[0]+12,pos[1]-5)
        cv2.line(out,p,line_end,c,1,cv2.LINE_AA)
        cv2.circle(out,p,3,(0,0,0),2,cv2.LINE_AA)
        cv2.circle(out,p,2,c,-1,cv2.LINE_AA)
        cv2.putText(out,f['id'],pos,cv2.FONT_HERSHEY_SIMPLEX,scale,(0,0,0),3,cv2.LINE_AA)
        cv2.putText(out,f['id'],pos,cv2.FONT_HERSHEY_SIMPLEX,scale,c,1,cv2.LINE_AA)
    text = f"{title} | {result['status']} | {sum(f['valid'] for f in result['features'])}/{len(result['features'])}"
    cv2.putText(out,text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,scale,(0,0,0),4,cv2.LINE_AA)
    cv2.putText(out,text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,scale,(255,255,255),1,cv2.LINE_AA)
    return out


def compare_results(current,target,target_kind):
    if current['template_id']!=target['template_id']:
        raise ValueError('Cannot compare different template/point identities.')
    if target_kind not in {'real','synthetic','geometric_render'}:
        raise ValueError('Unknown target provenance.')
    mask = [a and b for a,b in zip(current['valid_mask'],target['valid_mask'])]
    same_size = current['image_size']==target['image_size']
    return dict(template_id=current['template_id'],target_kind=target_kind,target_calibrated=False,
                control_ready=False,feature_order=[f['id'] for f in current['features']],
                shared_valid_mask=mask,matching_image_size=same_size,
                error_valid_mask=[valid and same_size for valid in mask],
                e_px=[float(a-b) if valid and same_size else None for a,b,valid in zip(current['s_px'],target['s_px'],mask)],
                note='Same image size is necessary but not sufficient: verify camera geometry. Synthetic targets are illustrative.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    init = sub.add_parser('init')
    for key in ['image','config','out']:
        init.add_argument('--'+key,required=True)
    pair = sub.add_parser('pair')
    for key in ['model','current','target','out']:
        pair.add_argument('--'+key,required=True)
    pair.add_argument('--target-kind',choices=['real','synthetic','geometric_render'],required=True)
    track = sub.add_parser('track')
    track.add_argument('--model',required=True)
    track.add_argument('--out',required=True)
    group = track.add_mutually_exclusive_group(required=True)
    group.add_argument('--video')
    group.add_argument('--frames')
    args = parser.parse_args()
    if args.command=='init':
        print(json.dumps(initialize(args.image,json.loads(Path(args.config).read_text()),args.out),ensure_ascii=False))
        return
    out = Path(args.out)
    out.mkdir(parents=True,exist_ok=True)
    if args.command=='pair':
        results = []
        for label,path in [('current',args.current),('target',args.target)]:
            image = read_image(path)
            result = Tracker(args.model).process(image)
            result['source_path'] = str(Path(path).resolve())
            if label=='target':
                result['target_kind'] = args.target_kind
            write_json(out/f'{label}.json',result)
            cv2.imwrite(str(out/f'{label}.png'),annotate(image,result,label+(' '+args.target_kind if label=='target' else '')))
            results.append(result)
        write_json(out/'comparison.json',compare_results(*results,args.target_kind))
        print(json.dumps({k:r['status'] for k,r in zip(['current','target'],results)}))
        return
    tracker = Tracker(args.model)
    cap = cv2.VideoCapture(args.video) if args.video else None
    if cap is not None and not cap.isOpened():
        raise ValueError('Could not open video.')
    files = sorted(p for p in Path(args.frames).iterdir() if p.suffix.lower() in {'.png','.jpg','.jpeg','.bmp'}) if args.frames else []
    if cap is None and not files:
        raise ValueError('No images found.')
    with (out/'features.jsonl').open('w') as f:
        index = 0
        while True:
            if cap is not None:
                ok,image = cap.read()
                if not ok:
                    break
                timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
            else:
                if index>=len(files):
                    break
                image,timestamp = read_image(files[index]),None
            result = tracker.process(image,timestamp)
            if cap is None:
                result['source_path'] = str(files[index].resolve())
            f.write(json.dumps(result,ensure_ascii=False,allow_nan=False)+'\n')
            cv2.imwrite(str(out/f'{index:06d}.png'),annotate(image,result,'tracking'))
            index += 1
    if cap is not None:
        cap.release()
    print(json.dumps(dict(frames=index,output=str(out.resolve()))))


if __name__=='__main__':
    main()
