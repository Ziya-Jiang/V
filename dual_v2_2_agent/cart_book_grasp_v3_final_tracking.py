"""Multiscale matching of a fixed visible patch in the final grasp image."""
from pathlib import Path
import hashlib
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError


class FinalImageTracker:
    def __init__(self, rgb, roi, points, mask_polygon=None):
        self.shape = rgb.shape
        self.target = np.asarray(points, np.float32)
        if self.target.shape != (4, 2) or not np.isfinite(self.target).all():
            raise ServoError('Four fixed final-image points are required')
        x0, y0, x1, y1 = map(int, roi)
        if not (0 <= x0 < x1 <= rgb.shape[1] and 0 <= y0 < y1 <= rgb.shape[0]):
            raise ServoError('Final reference ROI is outside the image')
        if np.any(self.target <= [x0, y0]) or np.any(self.target >= [x1, y1]):
            raise ServoError('Final reference points must lie inside the ROI')
        if not cv2.isContourConvex(self.target) or abs(cv2.contourArea(self.target)) < 100:
            raise ServoError('Final reference points must form a non-degenerate convex patch')
        self.detector = cv2.SIFT_create(nfeatures=6000, contrastThreshold=.008)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        full_mask = np.zeros(gray.shape, np.uint8)
        if mask_polygon is None:
            full_mask[y0:y1,x0:x1] = 255
        else:
            polygon = np.asarray(mask_polygon, np.float32)
            if (polygon.shape != (4,2) or not np.isfinite(polygon).all() or
                    not cv2.isContourConvex(polygon) or np.any(polygon < [x0,y0]) or
                    np.any(polygon > [x1,y1]) or
                    any(cv2.pointPolygonTest(polygon,tuple(map(float,p)),False)<0 for p in self.target)):
                raise ServoError('Invalid final book mask polygon')
            cv2.fillConvexPoly(full_mask, np.rint(polygon).astype(np.int32), 255)
        self.pyramid = []
        for scale in (.15, .2, .25, .3, .4, .5, .65, 1.):
            small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            mask = cv2.resize(full_mask,(small.shape[1],small.shape[0]),interpolation=cv2.INTER_NEAREST)
            keys, desc = self.detector.detectAndCompute(small, mask)
            if desc is not None:
                self.pyramid.append((scale, keys, desc))
        if sum(len(k) for _,k,_ in self.pyramid) < 16:
            raise ServoError('Final reference patch has insufficient texture')

    @classmethod
    def from_config(cls, config, camera, root):
        ref = config['reference']
        path = Path(ref['image'])
        if not path.is_absolute(): path = Path(root)/path
        if hashlib.sha256(path.read_bytes()).hexdigest() != ref['image_sha256']:
            raise ServoError('Final reference image changed; register it again')
        camera.check_reference(ref['metadata'])
        bgr = cv2.imread(str(path))
        if bgr is None or bgr.shape[:2] != (camera.size[1], camera.size[0]):
            raise ServoError('Final reference image has incorrect resolution')
        return cls(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), ref['roi'], ref['points'], ref.get('mask_polygon'))

    def observe(self, rgb, *, search_mask=None):
        if rgb.shape != self.shape:
            raise ServoError('Wrist resolution changed')
        if search_mask is not None:
            if np.shape(search_mask) != rgb.shape[:2]:
                raise ServoError('Final tracking search mask size mismatch')
            search_mask=(np.asarray(search_mask)!=0).astype(np.uint8)*255
        keys, desc = self.detector.detectAndCompute(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), search_mask)
        if desc is None or len(keys) < 16:
            raise ServoError('Final book patch is not visible')
        matcher = cv2.BFMatcher(cv2.NORM_L2)
        candidates = []
        for scale, reference_keys, reference_desc in self.pyramid:
            pairs = matcher.knnMatch(reference_desc, desc, k=2)
            reverse = matcher.match(desc, reference_desc)
            for pair in pairs:
                if len(pair) != 2: continue
                m, n = pair
                if n.distance <= 0 or m.distance >= .7*n.distance: continue
                if reverse[m.trainIdx].trainIdx != m.queryIdx: continue
                candidates.append((m.distance/n.distance,
                    np.array(reference_keys[m.queryIdx].pt)/scale, np.array(keys[m.trainIdx].pt)))
        unique = []
        for candidate in sorted(candidates, key=lambda item: item[0]):
            if all(np.linalg.norm(candidate[1]-other[1]) > 5 and
                   np.linalg.norm(candidate[2]-other[2]) > 2 for other in unique):
                unique.append(candidate)
        if len(unique) < 12:
            raise ServoError(f'Insufficient independent final-patch matches: {len(unique)}')
        source = np.float32([item[1] for item in unique])
        destination = np.float32([item[2] for item in unique])
        matrix, mask = cv2.findHomography(source, destination, cv2.RANSAC, 2., maxIters=10000, confidence=.999)
        if matrix is None or mask is None or not np.isfinite(matrix).all():
            raise ServoError('Final-patch homography failed')
        keep = mask.ravel().astype(bool)
        self.match_diagnostics = dict(inlier_reference_points=source[keep].tolist(),
                                      inlier_current_points=destination[keep].tolist(),
                                      inliers=int(keep.sum()), candidates=len(keep))
        if keep.sum() < 12 or keep.mean() < .6:
            raise ServoError(f'Ambiguous final-patch match: {keep.sum()}/{len(keep)} inliers')
        hull = cv2.convexHull(source[keep])
        if any(cv2.pointPolygonTest(hull, tuple(map(float, p)), False) < 0 for p in self.target):
            raise ServoError('Final control points lie outside matched texture support')
        predicted = cv2.perspectiveTransform(source[None], matrix)[0]
        rms = float(np.sqrt(np.mean(np.sum((predicted[keep]-destination[keep])**2, axis=1))))
        if rms > 1.5:
            raise ServoError('Final-patch reprojection residual is too large')
        points = cv2.perspectiveTransform(self.target[None], matrix)[0]
        h,w = rgb.shape[:2]
        if (not np.isfinite(points).all() or not cv2.isContourConvex(points) or
                np.any(points < [3,3]) or np.any(points > [w-4,h-4])):
            raise ServoError('Final control patch is clipped or distorted')
        ratio = cv2.contourArea(points, oriented=True)/cv2.contourArea(self.target, oriented=True)
        if not .005 <= ratio <= 2.:
            raise ServoError('Final-patch scale or orientation is invalid')
        return dict(points=points.astype(float), inliers=int(keep.sum()), candidates=len(keep),
                    reprojection_rms_px=rms, area_ratio=float(ratio), homography=matrix)


def draw_final_tracking(rgb, observation, target):
    image = rgb.copy()
    points = np.asarray(observation['points'])
    for current, desired in zip(points, target):
        a,b = tuple(np.rint(current).astype(int)), tuple(np.rint(desired).astype(int))
        cv2.circle(image,a,4,(255,220,0),-1)
        cv2.drawMarker(image,b,(0,255,0),cv2.MARKER_CROSS,14,1)
        cv2.line(image,a,b,(255,220,0),1)
    cv2.polylines(image,[np.rint(points).astype(np.int32)],True,(255,220,0),2)
    cv2.putText(image,'Stage 2 XYZ / yellow: current / green: final target',(12,28),
                cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
    return image
