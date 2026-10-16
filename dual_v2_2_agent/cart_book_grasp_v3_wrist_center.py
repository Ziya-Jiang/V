"""Choose a book in wrist RGB by distance to image center, once per cycle."""
import cv2
import numpy as np
from cart_book_grasp_v2_instances import clean_masks
from cart_book_grasp_v3_servo import ServoError
from cart_book_grasp_v3_tracking import LockedBookTracker


def detector_from_geometry(geometry):
    def detect(rgb, prompt='book'):
        processor, to_numpy, image_cls = geometry._load_sam3_detector()
        return geometry._sam3_raw_books(processor, to_numpy, image_cls.fromarray(rgb), prompt=prompt)
    return detect


def select_center_book(rgb, detections, *, horizontal_only=False, centerline=False):
    height, width = rgb.shape[:2]
    center = np.array([(width-1)/2, (height-1)/2])
    candidates = []
    for score, mask, box in clean_masks(detections, (height, width), dict(
            min_score=.5, min_area_px=max(100, int(height*width*.003)),
            mask_nms_iou=.65, nms_min_area_ratio=.65)):
        ys, xs = np.nonzero(mask)
        centroid = np.array([xs.mean(), ys.mean()])
        if centerline:
            from cart_book_grasp_v3_centerline import visible_centerline
            try: centroid[0]=visible_centerline(mask)[0]
            except ServoError: continue
        elif horizontal_only:
            from cart_book_grasp_v3_column import visible_center_column
            centroid[0]=visible_center_column(mask)
        distance=abs(centroid[0]-center[0]) if horizontal_only or centerline else np.linalg.norm(centroid-center)
        candidates.append(dict(score=score, mask=mask, box=box, uv=centroid.tolist(),distance_px=float(distance)))
    if not candidates:
        raise ServoError('No book detected in the wrist image; center selection cancelled')
    candidates.sort(key=lambda c: (c['distance_px'], -c['score'], c['box']))
    chosen = candidates[0]
    summary = dict(mode='wrist_center', image_center=center.tolist(), min_score=.5,
                   criterion=('Centerline distance at image center row' if centerline else 'Horizontal distance from visible center column' if horizontal_only else
                              'Euclidean pixel distance from mask centroid to image center'),
                   selected_index=0, candidates=[{k:v for k,v in c.items() if k!='mask'} for c in candidates])
    return chosen, summary


def acquire_wrist_center(rgb, detector, *, diagnostics=None):
    chosen, summary = select_center_book(rgb, detector(rgb))
    if diagnostics is not None:
        diagnostics.update(summary)
    # Point must lie within the mask. Use the nearest foreground pixel if the
    # centroid of a concave silhouette lies outside; ranking still uses centroid.
    ys, xs = np.nonzero(chosen['mask'])
    pixels = np.column_stack([xs, ys])
    anchor = pixels[np.argmin(np.sum((pixels-np.asarray(chosen['uv']))**2, axis=1))]
    return LockedBookTracker(rgb, chosen['mask'], point=anchor, allow_clipped=True)


def require_points_in_book(observation, selection):
    mask = selection['mask']
    points=np.asarray(observation['points'])
    if points.shape != (4,2) or not np.isfinite(points).all():
        raise ServoError('Invalid final points for the locked center book')
    pixels = np.rint(points).astype(int)
    if (np.any(pixels < 0) or np.any(pixels >= [mask.shape[1], mask.shape[0]]) or
            not np.all(mask[pixels[:, 1], pixels[:, 0]])):
        raise ServoError('Final taught patch is outside the locked center book; re-teach this book before servo')


def reanchor_wrist_book(rgb, selection):
    """Refresh local texture after a verified frame; never detect/reselect a book."""
    return LockedBookTracker(rgb,selection['mask'],point=selection['uv'],allow_clipped=True)


def draw_selection(rgb, selection):
    image = rgb.copy()
    contours, _ = cv2.findContours(np.uint8(selection['mask']), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (255,210,20), 2)
    h,w = image.shape[:2]
    cv2.drawMarker(image, ((w-1)//2, (h-1)//2), (255,255,255), cv2.MARKER_CROSS, 22, 2)
    cv2.putText(image, 'Wrist center book LOCKED / no head-wrist matching', (12,28),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (255,210,20), 2)
    return image
