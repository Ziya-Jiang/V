"""Fixed head RGB-D book-face tracker for V4.1; no I/O at import.

Select one planar physical face in TL, TR, BR, BL order. The bottom edge
must be the actual support edge. Depth is measured inside that same face;
edge points are intersections with its fitted plane, not background depth.
"""
import cv2
import numpy as np


class PlaneGeometryError(ValueError):
    """Measured face cannot support the planar reconstruction model."""


def validate_quad(q, shape):
    q = np.asarray(q, np.float32)
    h, w = shape[:2]
    if (q.shape != (4, 2) or not np.isfinite(q).all() or
            (q < 2).any() or (q[:, 0] >= w-2).any() or (q[:, 1] >= h-2).any() or
            not cv2.isContourConvex(q) or abs(cv2.contourArea(q)) < 400):
        raise ValueError('Book face must be a visible convex quadrilateral, TL/TR/BR/BL')
    return q


def rays(uv, k, d):
    xy = cv2.undistortPoints(np.asarray(uv, float).reshape(-1, 1, 2), k, d).reshape(-1, 2)
    return np.c_[xy, np.ones(len(xy))]


def measure_face(quad, depth, k, d, c2w, edge_midpoints=None, *, min_height_mm=40., min_axis_mm=10.):
    """Robust RGB-D plane with bounded residuals; XYZ output in base mm."""
    q = validate_quad(quad, depth.shape)
    mask = np.zeros(depth.shape, np.uint8)
    # Stay well inside the selected surface to avoid mixed edge/background depth.
    cv2.fillConvexPoly(mask, np.rint(q.mean(0) + .70*(q-q.mean(0))).astype(np.int32), 1)
    y, x = np.nonzero(mask)
    stride = max(1, len(x)//1800)
    x, y = x[::stride], y[::stride]
    z = depth[y, x].astype(float)
    good = np.isfinite(z) & (z > 100) & (z < 2500)
    if good.sum() < 80 or good.mean() < .65:
        raise ValueError('Book face has insufficient measured depth')
    xyz = rays(np.c_[x[good], y[good]], k, d) * z[good, None]
    # Deterministic RANSAC: reject a mixed surface instead of averaging it.
    rng = np.random.default_rng(41)
    best = np.zeros(len(xyz), bool)
    for _ in range(60):
        a, b, c = xyz[rng.choice(len(xyz), 3, replace=False)]
        n = np.cross(b-a, c-a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        inliers = abs((xyz-a) @ n) < 2.
        if inliers.sum() > best.sum():
            best = inliers
    if best.mean() < .8:
        raise PlaneGeometryError('Book face depth has no dominant plane (80% within 2 mm required)')
    points = xyz[best]
    center = points.mean(0)
    _, singular, vh = np.linalg.svd(points-center, full_matrices=False)
    if singular[1]/np.sqrt(len(points)) < 3:
        raise PlaneGeometryError('Book depth support is too narrow to estimate a plane')
    n = vh[-1]
    residual = float(np.sqrt(np.mean(((points-center) @ n)**2)))
    mids = np.array([(q[0]+q[1])/2, (q[2]+q[3])/2]) if edge_midpoints is None else np.asarray(edge_midpoints)
    if mids.shape != (2, 2) or not np.isfinite(mids).all():
        raise ValueError('Invalid tracked book landmarks')
    uv = np.vstack([q, mids])
    rr = rays(uv, k, d)
    denom = rr @ n
    if (abs(denom) < .12).any():
        raise PlaneGeometryError('Book plane too oblique for reliable edge reconstruction')
    zz = np.dot(center, n)/denom
    if (zz < 100).any() or (zz > 2500).any() or residual > 1.5:
        raise PlaneGeometryError('Book edge reconstruction or depth residual invalid')
    base = (rr*zz[:, None]) @ c2w[:3, :3].T + c2w[:3, 3]
    axis = base[2]-base[3]
    if np.linalg.norm(axis) < min_axis_mm or np.linalg.norm(base[4]-base[5]) < min_height_mm:
        raise ValueError('Book support axis/height unobservable')
    return dict(top_base_mm=base[4], bottom_base_mm=base[5],
                axis_base=axis/np.linalg.norm(axis), corners_base_mm=base[:4],
                plane_residual_mm=residual, depth_inlier_fraction=float(best.mean()))


class TrackingLost(ValueError):
    """Only visual tracking errors are recoverable; depth/robot faults are not."""


def project_base(points, k, d, c2w):
    camera = (np.asarray(points, float)-c2w[:3, 3]) @ c2w[:3, :3]
    if not np.isfinite(camera).all() or (camera[:, 2] <= 50).any():
        raise ValueError('Visual landmark behind/too close to camera')
    return cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, d)[0][:, 0]


def physical_midpoints(quad, depth, k, d, c2w, **geometry_options):
    """Initialize actual 3D edge midpoints, not perspective-dependent averages."""
    corners = measure_face(quad, depth, k, d, c2w, **geometry_options)['corners_base_mm']
    return project_base(np.array([(corners[0]+corners[1])/2,
                                  (corners[2]+corners[3])/2]), k, d, c2w)


class FaceTracker:
    """Distributed LK tracks with bounded, same-template SIFT reacquisition.

    Semantic points are transported from the immutable reference; texture
    centroids never define the contact location. Book and finger use separate
    instances. Projected edges are model estimates, not newly measured edges.
    """
    def __init__(self, reference_rgb, quad, landmarks=None, label='book', exclude=None):
        self.label = label
        self.shape = reference_rgb.shape
        self.reference_quad = validate_quad(quad, self.shape).copy()
        q = self.reference_quad
        self.reference_midpoints = np.array([(q[0]+q[1])/2, (q[2]+q[3])/2], np.float32) if landmarks is None else np.asarray(landmarks, np.float32).copy()
        self.edge_midpoints = self.reference_midpoints.copy()
        self.reference_gray = cv2.cvtColor(reference_rgb, cv2.COLOR_RGB2GRAY)
        self.previous = self.reference_gray.copy()
        self.mask = np.zeros(self.previous.shape, np.uint8)
        cv2.fillConvexPoly(self.mask, np.rint(q.mean(0)+.9*(q-q.mean(0))).astype(np.int32), 255)
        if exclude is not None:
            cv2.fillConvexPoly(self.mask, np.rint(exclude).astype(np.int32), 0)
        # Sample all spatial cells, not just whichever text has highest contrast.
        chunks = []
        x0, y0 = np.floor(q.min(0)).astype(int); x1, y1 = np.ceil(q.max(0)).astype(int)
        for yy in np.linspace(y0, y1, 4).astype(int)[:-1]:
            for xx in np.linspace(x0, x1, 4).astype(int)[:-1]:
                cell = np.zeros_like(self.mask)
                cell[yy:min(y1, yy+int(np.ceil((y1-y0)/3))), xx:min(x1, xx+int(np.ceil((x1-x0)/3)))] = 255
                pts = cv2.goodFeaturesToTrack(self.previous, 16, .01, 5, mask=cv2.bitwise_and(cell, self.mask))
                if pts is not None: chunks.append(pts)
        self.points = np.concatenate(chunks) if chunks else None
        self.minimum = 12 if label == 'book' else 8
        if self.points is None or len(self.points) < self.minimum:
            raise TrackingLost(f'{label}: selected face has too little texture')
        self.origin = self.points.copy()
        self.quad = q.copy()
        self.sift = cv2.SIFT_create(nfeatures=800)
        self.template_keys, self.template_desc = self.sift.detectAndCompute(self.reference_gray, self.mask)
        self.method = 'initial'
        self.relocalized = 0

    def _fit(self, src, dst):
        if len(src) < self.minimum:
            raise TrackingLost(f'{self.label}: lost reliable points ({len(src)})')
        h, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 1.5)
        if h is None or inliers is None or inliers.sum() < self.minimum-2 or inliers.mean() < .75:
            raise TrackingLost(f'{self.label}: geometry lacks consensus')
        take = inliers.ravel().astype(bool)
        support = cv2.convexHull(src[take])
        if abs(cv2.contourArea(support)) < .2*abs(cv2.contourArea(self.reference_quad)):
            raise TrackingLost(f'{self.label}: points do not span enough of the selected face')
        # Area alone can pass when only a central text block survives. Bound
        # projection of top/support landmarks away from real texture support.
        support_margin = min(24.,max(8.,.12*np.linalg.norm(np.ptp(self.reference_quad,axis=0))))
        for point in self.reference_midpoints:
            if cv2.pointPolygonTest(support,tuple(map(float,point)),True) < -support_margin:
                raise TrackingLost(f'{self.label}: semantic edge lacks nearby feature support')
        q = cv2.perspectiveTransform(self.reference_quad[None], h)[0]
        try: validate_quad(q, self.shape)
        except ValueError as exc: raise TrackingLost(str(exc)) from exc
        # Bound template reacquisition to continuous local motion, not a similar
        # neighboring book elsewhere in the frame.
        if np.max(np.linalg.norm(q-self.quad, axis=1)) > 40:
            raise TrackingLost(f'{self.label}: identity/geometry jump exceeds local search')
        area_ratio = abs(cv2.contourArea(q))/abs(cv2.contourArea(self.quad))
        if not .7 <= area_ratio <= 1.4:
            raise TrackingLost(f'{self.label}: implausible per-frame scale')
        return h, take, q

    def _lk(self, gray):
        nxt, ok, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, self.points, None, winSize=(31,31), maxLevel=3)
        if nxt is None: raise TrackingLost(f'{self.label}: optical flow lost')
        back, bok, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, nxt, None, winSize=(31,31), maxLevel=3)
        if back is None: raise TrackingLost(f'{self.label}: backward flow lost')
        keep = (ok.ravel()>0) & (bok.ravel()>0) & np.isfinite(nxt).all(axis=(1,2))
        keep &= np.linalg.norm(back-self.points, axis=2).ravel()<.8
        src, dst = self.origin[keep], nxt[keep]
        return src, dst, self._fit(src, dst)

    def _relocalize(self, gray):
        lo = np.maximum(0, np.floor(self.quad.min(0)-40)).astype(int)
        hi = np.minimum([gray.shape[1],gray.shape[0]], np.ceil(self.quad.max(0)+40)).astype(int)
        keys, desc = self.sift.detectAndCompute(gray[lo[1]:hi[1],lo[0]:hi[0]], None)
        if desc is None or self.template_desc is None or min(len(desc),len(self.template_desc))<self.minimum:
            raise TrackingLost(f'{self.label}: no template matches for recovery')
        bf = cv2.BFMatcher()
        forward = bf.knnMatch(self.template_desc, desc, k=2)
        reverse = bf.knnMatch(desc, self.template_desc, k=2)
        reverse_good = {m.queryIdx:m.trainIdx for pair in reverse if len(pair)==2 for m,n in [pair] if m.distance<.7*n.distance}
        matches = [m for pair in forward if len(pair)==2 for m,n in [pair]
                   if m.distance<.7*n.distance and reverse_good.get(m.trainIdx)==m.queryIdx]
        src = np.float32([self.template_keys[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
        dst = (np.float32([keys[m.trainIdx].pt for m in matches]).reshape(-1,1,2)+lo).astype(np.float32)
        return src, dst, self._fit(src,dst)

    def update(self, rgb):
        if rgb.shape != self.shape: raise ValueError('Head image resolution changed')
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        try:
            src, dst, (h,take,q) = self._lk(gray)
            method = 'lk'
        except TrackingLost:
            src, dst, (h,take,q) = self._relocalize(gray)
            method = 'sift_same_template'
        # Atomic commit only after all validity checks; a failed tracker cannot
        # install a new reference accidentally before semantic recovery.
        self.edge_midpoints = cv2.perspectiveTransform(self.reference_midpoints[None],h)[0]
        self.origin, self.points, self.previous, self.quad = src[take],dst[take],gray,q.copy()
        self.method = method
        if method != 'lk': self.relocalized += 1
        return self.quad.copy()


class FingerTracker:
    """Independent visible finger-side texture, anchored to its lower pad edge.

    RGB observes the edge position. Depth of this robot landmark remains FK
    based, not sampled at a mixed foreground/background contour. No claim of
    measured contact force or visually measured pad depth is made.
    """
    def __init__(self, rgb, quad):
        self.tracker = FaceTracker(rgb, quad, label='finger')
        self.local_anchor = None

    def bind(self, tcp, rotation, pad, k, d, c2w):
        side = np.cross(pad.normal_tcp, pad.along_tcp)
        side /= np.linalg.norm(side)
        locals_ = np.array([pad.center_tcp_mm+sign*side*pad.width_mm/2 for sign in (-1,1)])
        pixels = project_base(locals_@rotation.T+tcp, k, d, c2w)
        line = self.tracker.quad[[3,2]].astype(float)
        tangent = line[1]-line[0]
        if np.linalg.norm(tangent)<12:
            raise ValueError('Finger lower long edge is too short')
        t = ((pixels-line[0])@tangent)/np.dot(tangent,tangent)
        feet = line[0]+t[:,None]*tangent
        errors = np.linalg.norm(pixels-feet, axis=1)
        rank = np.argsort(errors); chosen = rank[0]
        if not .1<t[chosen]<.9 or errors[chosen]>8 or errors[rank[1]]-errors[chosen]<1:
            raise ValueError('Visible finger edge disagrees with calibrated pad or side is ambiguous')
        self.local_anchor = locals_[chosen].copy()
        # This point retains its identity through the side-face homography.
        self.tracker.reference_midpoints = np.array([feet[chosen]],np.float32)
        self.tracker.edge_midpoints = self.tracker.reference_midpoints.copy()

    def update(self, rgb):
        self.tracker.update(rgb)
        return dict(finger_edge_uv=self.tracker.quad[[3,2]].copy(),
            finger_reference_uv=None if self.local_anchor is None else self.tracker.edge_midpoints[0].copy(),
            finger_reference_tcp_mm=None if self.local_anchor is None else self.local_anchor.copy(),
            finger_points=len(self.tracker.points), finger_method=self.tracker.method)


def observed_pad_bias(obs, tcp, rotation, k, d, c2w, limit_mm=3.):
    """2-D measured pad error with calibrated FK depth, in base millimetres."""
    uv = np.asarray(obs['finger_reference_uv'],float)
    local = np.asarray(obs['finger_reference_tcp_mm'],float)
    if uv.shape!=(2,) or local.shape!=(3,) or not np.isfinite(uv).all() or not np.isfinite(local).all():
        raise ValueError('Invalid visual finger reference')
    point = tcp+rotation@local
    cam = c2w[:3,:3].T@(point-c2w[:3,3])
    if cam[2]<=50: raise ValueError('Finger landmark behind camera')
    measured = rays(uv[None],k,d)[0]*cam[2]
    bias = c2w[:3,:3]@(measured-cam)
    if np.linalg.norm(bias)>limit_mm:
        raise ValueError('Visual finger/FK disagreement exceeds 3 mm; check calibration/identity')
    return bias
