"""A deliberately small baseline: learn per-phase visual goals from annotations.

Coordinates remain in the ORIGINAL wrist image. No action labels are used, but
object boxes and phase keyframe indices are required. This is not a video model.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image

PHASES = ("align", "pregrasp", "grasp")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def box_feature(bbox, size):
    """Pixel xywh -> normalized center xy and log normalized width/height."""
    b = np.asarray(bbox, dtype=float)
    if b.shape != (4,) or not np.all(np.isfinite(b)):
        raise ValueError("bbox must contain four finite numbers: [x, y, width, height]")
    x, y, w, h = b
    width, height = size
    if min(width, height) <= 0 or min(w, h) <= 0 or min(x, y) < 0:
        raise ValueError("Image and bbox sizes must be positive; box origin must be nonnegative")
    if x + w > width or y + h > height:
        raise ValueError("bbox must be inside the original image")
    return np.array([(x + w / 2) / width, (y + h / 2) / height,
                     np.log(w / width), np.log(h / height)])


def feature_box(feature, size):
    u, v, logw, logh = feature
    width, height = size
    w, h = np.exp(logw) * width, np.exp(logh) * height
    return [float(u * width - w / 2), float(v * height - h / 2), float(w), float(h)]


def load_dataset(path):
    path = Path(path).resolve()
    data = read_json(path)
    for key in ("camera_id", "target_id"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"{key} must identify the fixed camera setup and single target object")
    if not data.get("episodes"):
        raise ValueError("Dataset needs at least one episode")
    seen_ids, image_splits = set(), {}
    for episode in data["episodes"]:
        eid = episode["id"]
        if not isinstance(eid, str) or not eid or eid in seen_ids:
            raise ValueError("Episode IDs must be nonempty and unique")
        seen_ids.add(eid)
        split = episode["split"]
        if split not in ("train", "test"):
            raise ValueError(f"{eid}: split must be train or test")
        frames = episode["frames"]
        if not frames:
            raise ValueError(f"{eid}: no frames")
        indices = [episode["keyframes"][phase] for phase in PHASES]
        if any(type(i) is not int or i < 0 or i >= len(frames) for i in indices):
            raise ValueError(f"{eid}: invalid keyframe indices")
        if not indices[0] < indices[1] < indices[2]:
            raise ValueError(f"{eid}: align, pregrasp, grasp must be in chronological order")
        sizes, features, paths = [], [], []
        for frame in frames:
            image_path = (path.parent / frame["image"]).resolve()
            previous_split = image_splits.setdefault(str(image_path), split)
            if previous_split != split:
                raise ValueError(f"Train/test image overlap: {image_path}")
            with Image.open(image_path) as image:
                image.load()
                size = image.size
            sizes.append(size)
            features.append(box_feature(frame["bbox"], size).tolist())
            paths.append(str(image_path))
        if len(set(sizes)) != 1:
            raise ValueError(f"{eid}: image dimensions changed within episode")
        episode["size"] = sizes[0]
        episode["features"] = features
        episode["image_paths"] = paths
    # A fixed camera setup means matching image geometry, not just a name.
    if len({tuple(e["size"]) for e in data["episodes"]}) != 1:
        raise ValueError("First baseline requires the same image dimensions across episodes")
    return data


def fit(data):
    training = [e for e in data["episodes"] if e["split"] == "train"]
    if not training:
        raise ValueError("No training episodes")
    goals = {}
    for phase in PHASES:
        samples = np.array([e["features"][e["keyframes"][phase]] for e in training])
        median = np.median(samples, axis=0)
        # Coordinate-wise medians need not themselves form an in-frame box.
        # Use the observed phase exemplar closest to the median instead.
        distances = np.linalg.norm(samples[:, :2] - median[:2], axis=1)
        distances += 0.25 * np.linalg.norm(samples[:, 2:] - median[2:], axis=1)
        chosen = int(np.argmin(distances))
        source = training[chosen]
        goals[phase] = {
            "feature": samples[chosen].tolist(),
            "source_episode": source["id"],
            "source_image": source["image_paths"][source["keyframes"][phase]],
            "median_absolute_deviation": np.median(abs(samples - median), axis=0).tolist(),
        }
    starts = np.array([e["features"][0] for e in training])
    return {
        "version": 1, "type": "fixed_camera_single_object_phase_exemplars",
        "camera_id": data["camera_id"], "target_id": data["target_id"],
        "size": list(training[0]["size"]),
        "training_episode_ids": [e["id"] for e in training],
        "training_image_paths": [p for e in training for p in e["image_paths"]],
        "initial_feature_min": starts.min(axis=0).tolist(),
        "initial_feature_max": starts.max(axis=0).tolist(), "goals": goals,
    }


def plan(model, initial_bbox, size, camera_id, target_id):
    if camera_id != model["camera_id"] or target_id != model["target_id"]:
        raise ValueError("Camera setup and target object must match the trained baseline")
    if list(size) != model["size"]:
        raise ValueError("Image dimensions must match the trained baseline")
    current = box_feature(initial_bbox, size)
    lower, upper = np.array(model["initial_feature_min"]), np.array(model["initial_feature_max"])
    outside = bool(np.any(current < lower - 1e-8) or np.any(current > upper + 1e-8))
    keyframes = [{"phase": "current", "bbox": list(initial_bbox)}]
    for phase in PHASES:
        goal = model["goals"][phase]
        feature = np.array(goal["feature"])
        # First align laterally while retaining the observed apparent object size.
        if phase == "align":
            feature[2:] = current[2:]
        bbox = feature_box(feature, size)
        try:
            box_feature(bbox, size)
        except ValueError as exc:
            raise ValueError(f"Proposed {phase} box is outside the image; reject this plan") from exc
        keyframes.append({"phase": phase, "bbox": bbox,
                          "reference_image": goal["source_image"]})
    return {
        "type": "visual_bbox_subgoals", "keyframes": keyframes,
        "outside_training_start_range": outside,
        "limitations": [
            "Reference images are retrieved demonstrations, not generated future images.",
            "No depth, rotation, collision, reachability or physical grasp checks.",
            "Grasp is a visual phase label, not proof of contact or successful grasp.",
            "Lift completion requires robot displacement and grasp feedback.",
        ],
    }


def evaluate(model, data):
    testing = [e for e in data["episodes"] if e["split"] == "test"]
    if not testing:
        raise ValueError("Evaluation requires held-out test episodes")
    trained_ids = set(model["training_episode_ids"])
    trained_paths = set(model["training_image_paths"])
    if any(e["id"] in trained_ids or trained_paths.intersection(e["image_paths"]) for e in testing):
        raise ValueError("Evaluation episodes overlap the model's training data")
    records = []
    for e in testing:
        predicted = plan(model, e["frames"][0]["bbox"], e["size"], data["camera_id"], data["target_id"])
        for frame in predicted["keyframes"][1:]:
            phase = frame["phase"]
            truth = np.array(e["features"][e["keyframes"][phase]])
            pred = box_feature(frame["bbox"], e["size"])
            initial = np.array(e["features"][0])
            def errors(value):
                return {"center_error_px": float(np.linalg.norm((value[:2] - truth[:2]) * e["size"])),
                        "mean_abs_log_size_error": float(np.mean(abs(value[2:] - truth[2:])))}
            records.append({"episode": e["id"], "phase": phase,
                            "prediction": errors(pred), "hold_initial_baseline": errors(initial),
                            "outside_training_start_range": predicted["outside_training_start_range"]})
    summary = {}
    for method in ("prediction", "hold_initial_baseline"):
        summary[method] = {metric: float(np.mean([r[method][metric] for r in records]))
                           for metric in ("center_error_px", "mean_abs_log_size_error")}
    return {"test_episodes": len(testing), "summary": summary, "records": records,
            "scope": "Annotation-space agreement only; not grasp success or controller feasibility."}
