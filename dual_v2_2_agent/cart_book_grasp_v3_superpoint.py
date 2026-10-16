"""Local CPU SuperPoint + LightGlue backend; no camera or robot connections.

Unlike the offline comparison entry point, this module never changes CUDA
visibility or imports scene/segmentation scripts into the live robot process.
"""
from functools import lru_cache
from pathlib import Path
import sys
import cv2
import numpy as np
from cart_book_grasp_v3_servo import ServoError

ROOT = Path(__file__).resolve().parent


def in_mask(points, mask):
    pixels = np.rint(points).astype(int)
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < mask.shape[1]) &
              (pixels[:, 1] >= 0) & (pixels[:, 1] < mask.shape[0]))
    result = np.zeros(len(points), bool)
    result[inside] = mask[pixels[inside, 1], pixels[inside, 0]] != 0
    return result


class SuperPointBackend:
    def __init__(self):
        cache = ROOT / 'third_party/superpoint_cache/hub'
        for name in ('superpoint_v1.pth', 'superpoint_lightglue_v0-1_arxiv.pth'):
            if not (cache / 'checkpoints' / name).is_file():
                raise ServoError(f'Missing local SuperPoint/LightGlue weight: {name}; no automatic download')
        for folder in ('LightGlue', 'superpoint_deps'):
            path = str(ROOT / 'third_party' / folder)
            if path not in sys.path:
                sys.path.insert(0, path)
        try:
            import torch
            from lightglue import SuperPoint, LightGlue
            self.torch = torch
            torch.set_num_threads(min(torch.get_num_threads(), 4))
            previous = torch.hub.get_dir()
            try:
                torch.hub.set_dir(str(cache))
                self.extractor = SuperPoint(max_num_keypoints=2048).eval().cpu()
                self.matcher = LightGlue(features='superpoint', depth_confidence=-1,
                                         width_confidence=-1, flash=False).eval().cpu()
            finally:
                torch.hub.set_dir(previous)
        except Exception as error:
            raise ServoError(f'Cannot load local SuperPoint + LightGlue: {error}') from error

    def extract(self, rgb):
        with self.torch.inference_mode():
            image = self.torch.from_numpy(rgb.transpose(2, 0, 1).copy()).float() / 255.
            return self.extractor.extract(image, resize=None)

    def sources(self, rgb, mask):
        ys, xs = np.nonzero(mask)
        x0, x1 = max(0, xs.min()-24), min(rgb.shape[1], xs.max()+25)
        y0, y1 = max(0, ys.min()-24), min(rgb.shape[0], ys.max()+25)
        crop = rgb[y0:y1, x0:x1]
        factor = min(4., max(1., 480. / max(crop.shape[:2])))
        enlarged = cv2.resize(crop, None, fx=factor, fy=factor, interpolation=cv2.INTER_LINEAR)
        scale = np.array([enlarged.shape[1]/crop.shape[1], enlarged.shape[0]/crop.shape[0]])
        result = []
        for name, image, ratio, offset in (
            ('full', rgb, np.ones(2), np.zeros(2)),
            ('roi', enlarged, scale, np.array([x0, y0])),
        ):
            features = self.extract(image)
            points = features['keypoints'][0].numpy()/ratio+offset
            result.append((name, features, points, in_mask(points, mask)))
        return result

    def correspondences(self, sources, rgb):
        destination = self.extract(rgb)
        points = destination['keypoints'][0].numpy()
        results = []
        for name, source, source_points, selected in sources:
            indices = np.empty((0, 2), int)
            if len(source_points) >= 2 and len(points) >= 2:
                with self.torch.inference_mode():
                    indices = self.matcher(dict(image0=source, image1=destination))['matches'][0].numpy()
                indices = indices[selected[indices[:, 0]]]
            # Reject duplicate spatial samples before RANSAC.
            unique = []
            for i, j in indices:
                a, b = source_points[i], points[j]
                if all(np.linalg.norm(a-u) > 1 and np.linalg.norm(b-v) > 2 for u, v in unique):
                    unique.append((a, b))
            a = np.float32([u for u, _ in unique]).reshape(-1, 2)
            b = np.float32([v for _, v in unique]).reshape(-1, 2)
            results.append((name, a, b, int(selected.sum())))
        return results


@lru_cache(maxsize=1)
def get_backend():
    return SuperPointBackend()
