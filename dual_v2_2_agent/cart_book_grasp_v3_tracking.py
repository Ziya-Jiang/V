"""Match texture only from the GPT-selected wrist instance; no target reselect."""
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError


class LockedBookTracker:
    def __init__(self, rgb, mask):
        rgb = np.asarray(rgb)
        mask = np.asarray(mask, dtype=np.uint8)
        if rgb.ndim != 3 or mask.shape != rgb.shape[:2] or mask.sum() < 100:
            raise ServoError('Invalid selected wrist mask')
        self.shape = rgb.shape
        self.detector = cv2.SIFT_create(nfeatures=2500, contrastThreshold=.02)
        # Keep descriptors away from the neighboring books at mask boundaries.
        interior = cv2.erode(mask, np.ones((5, 5), np.uint8))
        self.keys, self.desc = self.detector.detectAndCompute(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), interior*255)
        if self.desc is None or len(self.keys) < 16:
            raise ServoError('Selected book has too little visible texture for V3 tracking')
        yy, xx = np.nonzero(mask)
        self.center = np.array([np.median(xx), np.median(yy)], np.float32)
        self.corners = np.float32([[xx.min(), yy.min()], [xx.max(), yy.min()],
                                  [xx.max(), yy.max()], [xx.min(), yy.max()]])
        self.size = np.ptp(self.corners, axis=0)

    def observe(self, rgb):
        if rgb.shape != self.shape:
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
        matrix, keep = cv2.findHomography(src, dst, cv2.RANSAC, 2.5)
        if matrix is None or keep is None or not np.isfinite(matrix).all():
            raise ServoError('Book tracking transform failed')
        keep = keep.ravel().astype(bool)
        if keep.sum() < 12 or keep.mean() < .65:
            raise ServoError('Ambiguous or occluded book texture')
        if np.any(np.ptp(src[keep], axis=0) < self.size * [.25, .3]):
            raise ServoError('Tracked texture does not span enough of the selected book')
        projected = cv2.perspectiveTransform(src[None], matrix)[0]
        if np.percentile(np.linalg.norm(projected[keep]-dst[keep], axis=1), 90) > 2.5:
            raise ServoError('Book tracking reprojection error is too large')
        polygon = cv2.perspectiveTransform(self.corners[None], matrix)[0]
        uv = cv2.perspectiveTransform(self.center.reshape(1, 1, 2), matrix)[0, 0]
        h, w = rgb.shape[:2]
        if (not np.isfinite(polygon).all() or not np.isfinite(uv).all() or
                not cv2.isContourConvex(polygon) or
                np.any(polygon < [2, 2]) or np.any(polygon > [w-3, h-3])):
            raise ServoError('Selected book is clipped or tracking is distorted')
        original_area = cv2.contourArea(self.corners, oriented=True)
        ratio = cv2.contourArea(polygon, oriented=True)/original_area
        if not .4 <= ratio <= 2.5:
            raise ServoError('Book scale changed beyond the local servo range')
        return dict(uv=uv.astype(float), polygon=polygon, inliers=int(keep.sum()))


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
