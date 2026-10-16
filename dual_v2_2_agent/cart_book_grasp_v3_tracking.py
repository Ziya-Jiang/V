"""Track the V1_3-selected spine and its fixed physical alignment point."""
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError


class LockedBookTracker:
    def __init__(self, rgb, mask, point=None, *, cross_view=False, cross_view_backend='sift', allow_clipped=False):
        rgb = np.asarray(rgb)
        mask = np.asarray(mask, dtype=np.uint8)
        if rgb.ndim != 3 or mask.shape != rgb.shape[:2] or mask.sum() < 100:
            raise ServoError('Invalid selected wrist mask')
        self.shape = rgb.shape
        self.mask = (mask != 0).astype(np.uint8)
        self.allow_clipped = allow_clipped
        self.cross_view = cross_view
        if cross_view_backend not in ('sift', 'superpoint'):
            raise ServoError('Unknown head-to-wrist matching backend')
        self.cross_view_backend = cross_view_backend
        self.match_diagnostics = []
        if cross_view and cross_view_backend == 'superpoint':
            from cart_book_grasp_v3_superpoint import get_backend
            self.backend = get_backend()
            self.sources = self.backend.sources(rgb, mask)
            self.mask = mask.copy()
            self.keys, self.desc = [], None
        else:
            self._initialize_sift(rgb, mask)
        yy, xx = np.nonzero(mask)
        self.center = np.array([np.median(xx), np.median(yy)], np.float32)
        if point is not None:
            point = np.asarray(point, dtype=np.float32)
            if point.shape != (2,) or not np.isfinite(point).all():
                raise ServoError('Invalid fixed alignment point')
            u, v = np.rint(point).astype(int)
            if not (0 <= u < mask.shape[1] and 0 <= v < mask.shape[0] and mask[v, u]):
                raise ServoError('Alignment point is outside the selected spine')
            self.center = point.copy()
        self.corners = np.float32([[xx.min(), yy.min()], [xx.max(), yy.min()],
                                  [xx.max(), yy.max()], [xx.min(), yy.max()]])
        self.size = np.ptp(self.corners, axis=0)

    def _initialize_sift(self, rgb, mask):
        self.detector = cv2.SIFT_create(nfeatures=2500, contrastThreshold=.02)
        # Keep descriptors away from the neighboring books at mask boundaries.
        interior = cv2.erode(mask, np.ones((5, 5), np.uint8))
        self.keys, self.desc = self.detector.detectAndCompute(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), interior*255)
        if self.desc is None or len(self.keys) < 16:
            yy,xx=np.nonzero(mask)
            raise ServoError('Selected book has too little visible texture for V3 tracking: '
                f'features={len(self.keys)}/16, mask_bbox_px={int(np.ptp(xx)+1)}x{int(np.ptp(yy)+1)}, '
                f'mask_pixels={int(np.count_nonzero(mask))}, '
                f'interior_pixels={int(np.count_nonzero(interior))}')

    def observe(self, rgb):
        if self.cross_view and self.cross_view_backend == 'superpoint':
            return self._observe_superpoint(rgb)
        if not self.cross_view and rgb.shape != self.shape:
            raise ServoError('Wrist resolution changed since target selection')
        keys, desc = self.detector.detectAndCompute(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), None)
        if desc is None or len(keys) < 16:
            raise ServoError('Wrist target texture lost')
        matcher = cv2.BFMatcher(cv2.NORM_L2)
        forward = matcher.knnMatch(self.desc, desc, k=2)
        reverse = matcher.match(desc, self.desc)
        matches = [pair[0] for pair in forward if len(pair) == 2 and
                   pair[0].distance < .65*pair[1].distance and
                   reverse[pair[0].trainIdx].trainIdx == pair[0].queryIdx]
        if len(matches) < 12:
            raise ServoError('Cannot reliably reacquire the same selected book')
        src = np.float32([self.keys[m.queryIdx].pt for m in matches])
        dst = np.float32([keys[m.trainIdx].pt for m in matches])
        return self._fit(src, dst, rgb, minimum=12)

    def _observe_superpoint(self, rgb):
        self.match_diagnostics = []
        accepted = []
        for variant, src, dst, count in self.backend.correspondences(self.sources, rgb):
            detail = dict(backend='SuperPoint+LightGlue', variant=variant,
                          source_features=count, matches=len(src), min_inliers=6)
            try:
                result = self._fit(src, dst, rgb, minimum=6)
                detail.update(inliers=result['inliers'], accepted=True)
                accepted.append(result)
            except ServoError as error:
                detail.update(accepted=False, reason=str(error))
            self.match_diagnostics.append(detail)
        if not accepted:
            raise ServoError('SuperPoint+LightGlue head-to-wrist acquisition rejected: ' +
                             '; '.join(f"{d['variant']}: features={d['source_features']}, "
                                       f"matches={d['matches']}, {d.get('reason', '')}"
                                       for d in self.match_diagnostics))
        if len(accepted) == 2 and np.linalg.norm(accepted[0]['uv']-accepted[1]['uv']) > 5:
            raise ServoError('SuperPoint full/ROI disagree on selected book location')
        return max(accepted, key=lambda r: r['inliers'])

    def _fit(self, src, dst, rgb, *, minimum):
        if len(src) < minimum:
            raise ServoError(f'Insufficient independent matches: {len(src)}/{minimum}')
        cv2.setRNGSeed(7)
        matrix, keep = cv2.findHomography(src, dst, cv2.RANSAC, 2.5)
        if matrix is None or keep is None or not np.isfinite(matrix).all():
            raise ServoError('Book tracking transform failed')
        keep = keep.ravel().astype(bool)
        if keep.sum() < minimum or keep.mean() < .65:
            raise ServoError(f'Ambiguous or occluded book texture: {keep.sum()}/{len(keep)} inliers (min={minimum}, ratio>=0.65)')
        if np.any(np.ptp(src[keep], axis=0) < self.size * [.25, .3]):
            raise ServoError('Tracked texture does not span enough of the selected book')
        if cv2.pointPolygonTest(cv2.convexHull(src[keep]), tuple(map(float, self.center)), False) < 0:
            raise ServoError('Alignment point lies outside the matched texture; extrapolation rejected')
        projected = cv2.perspectiveTransform(src[None], matrix)[0]
        if self.cross_view_backend == 'superpoint' and np.sqrt(np.mean(np.sum((projected[keep]-dst[keep])**2, axis=1))) > 1.5:
            raise ServoError('SuperPoint head-to-wrist RMS exceeds 1.5 px')
        if np.percentile(np.linalg.norm(projected[keep]-dst[keep], axis=1), 90) > 2.5:
            raise ServoError('Book tracking reprojection error is too large')
        polygon = cv2.perspectiveTransform(self.corners[None], matrix)[0]
        uv = cv2.perspectiveTransform(self.center.reshape(1, 1, 2), matrix)[0, 0]
        h, w = rgb.shape[:2]
        if (not np.isfinite(polygon).all() or not np.isfinite(uv).all() or
                not cv2.isContourConvex(polygon) or
                np.any(uv < [2,2]) or np.any(uv > [w-3,h-3]) or
                (not self.allow_clipped and (np.any(polygon < [2, 2]) or np.any(polygon > [w-3, h-3])))):
            raise ServoError('Selected book is clipped or tracking is distorted')
        original_area = cv2.contourArea(self.corners, oriented=True)
        ratio = cv2.contourArea(polygon, oriented=True)/original_area
        low, high = (.05, 20.) if self.cross_view else (.4, 2.5)
        if not low <= ratio <= high:
            raise ServoError('Book scale changed beyond the local servo range')
        tracked_mask=cv2.warpPerspective(self.mask,matrix,(w,h),flags=cv2.INTER_NEAREST)
        if tracked_mask.sum() < 100:
            raise ServoError('Locked book has left the wrist image')
        return dict(uv=uv.astype(float), polygon=polygon, mask=tracked_mask.astype(bool),
                    inliers=int(keep.sum()), homography=matrix)


def acquire_selected_spine(head_frame, wrist_rgb, *, diagnostics=None, backend='sift'):
    """Match only V1_3's selected spine, then anchor local tracking in wrist RGB.

    No GPT reselection or nearest-image-center fallback. The selected geometric
    spine point (including V1_3 stable-depth recovery) is transported by a
    checked planar texture match. Lack of common texture aborts acquisition.
    Legacy Y/Z callers retain SIFT; default V3 XYZ explicitly uses SuperPoint.
    """
    if head_frame is None:
        raise ServoError('No current V1_3 selected spine frame')
    selected = head_frame['selected']; source = None
    try:
        source = LockedBookTracker(head_frame['rgb'], selected['mask'],
                                   point=selected['uv'], cross_view=True, cross_view_backend=backend)
        observation = source.observe(wrist_rgb)
    except ServoError as error:
        raise ServoError(f'Head-to-wrist selected-book identity check failed: {error}') from error
    finally:
        if diagnostics is not None and source is not None:
            diagnostics.extend(source.match_diagnostics)
    # Warp the exact spine mask, not its bounding rectangle, which could
    # include neighboring book texture when the spine is tilted.
    mask = cv2.warpPerspective(np.asarray(selected['mask'], np.uint8), observation['homography'],
                               (wrist_rgb.shape[1], wrist_rgb.shape[0]), flags=cv2.INTER_NEAREST)
    try:
        return LockedBookTracker(wrist_rgb, mask, point=observation['uv'])
    except ServoError as error:
        raise ServoError(f'Wrist selected-book tracker initialization failed: {error}') from error


def overlay(rgb, observation, target=None):
    image = rgb.copy()
    center = tuple(np.rint(observation['uv']).astype(int))
    cv2.polylines(image, [np.rint(observation['polygon']).astype(np.int32)], True, (0, 220, 255), 2)
    cv2.circle(image, center, 5, (255, 255, 0), -1)
    if target is not None:
        goal = tuple(np.rint(target).astype(int))
        cv2.drawMarker(image, goal, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.arrowedLine(image, center, goal, (255, 255, 0), 2)
    cv2.rectangle(image, (8, 7), (min(image.shape[1]-1, 685), 40), (25, 25, 25), -1)
    cv2.putText(image, 'V3 locked target / yellow: observed / green: reference',
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 2)
    return image
