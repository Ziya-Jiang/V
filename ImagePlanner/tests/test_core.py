import copy

import numpy as np
import pytest
from PIL import Image

from imageplanner.core import box_feature, evaluate, feature_box, fit, load_dataset, plan, write_json


@pytest.fixture
def dataset(tmp_path):
    data = {"camera_id": "wrist", "target_id": "block", "episodes": []}
    for i, split in enumerate(("train", "train", "test")):
        frames = []
        boxes = [[20 + 10 * i, 30, 20, 20], [150, 110, 20, 20],
                 [140, 100, 40, 40], [130, 90, 60, 60]]
        for j, box in enumerate(boxes):
            # Synthetic fixtures exercise software only, not visual perception.
            name = f"e{i}_{j}.png"
            Image.new("RGB", (320, 240)).save(tmp_path / name)
            frames.append({"image": name, "bbox": box})
        data["episodes"].append({"id": f"e{i}", "split": split, "frames": frames,
                                 "keyframes": {"align": 1, "pregrasp": 2, "grasp": 3}})
    path = tmp_path / "dataset.json"
    write_json(path, data)
    return path, data


def test_roundtrip_preserves_original_image_geometry():
    box = [31, 57, 120, 63]
    assert np.allclose(feature_box(box_feature(box, (640, 480)), (640, 480)), box)
    for invalid in ([0, 0, 0, 2], [-1, 0, 1, 2], [639, 0, 2, 2], [0, 0, np.nan, 2]):
        with pytest.raises(ValueError):
            box_feature(invalid, (640, 480))


def test_fit_and_heldout_evaluation(dataset):
    path, _ = dataset
    data = load_dataset(path)
    model = fit(data)
    report = evaluate(model, data)
    assert report["test_episodes"] == 1
    assert report["summary"]["prediction"]["center_error_px"] < 1e-8
    assert report["summary"]["hold_initial_baseline"]["center_error_px"] > 50
    assert all(g["source_episode"] != "e2" for g in model["goals"].values())
    # Changing test annotations must not change fitting results.
    changed = copy.deepcopy(data)
    changed["episodes"][-1]["features"][2] = [0.1, 0.2, -3, -3]
    assert fit(changed) == model


def test_alignment_preserves_initial_scale_and_checks_setup(dataset):
    path, _ = dataset
    model = fit(load_dataset(path))
    result = plan(model, [10, 10, 24, 30], (320, 240), "wrist", "block")
    assert np.allclose(result["keyframes"][1]["bbox"][2:], [24, 30])
    assert result["outside_training_start_range"]
    with pytest.raises(ValueError, match="Camera"):
        plan(model, [10, 10, 24, 30], (320, 240), "other_camera", "block")
    with pytest.raises(ValueError, match="dimensions"):
        plan(model, [10, 10, 24, 30], (640, 480), "wrist", "block")


def test_dataset_rejects_frame_leakage_and_wrong_phase_order(dataset):
    path, raw = dataset
    raw["episodes"][-1]["frames"][0] = raw["episodes"][0]["frames"][0]
    write_json(path, raw)
    with pytest.raises(ValueError, match="overlap"):
        load_dataset(path)
    raw["episodes"][-1]["frames"][0]["image"] = "e2_0.png"
    raw["episodes"][-1]["keyframes"]["align"] = 3
    write_json(path, raw)
    with pytest.raises(ValueError, match="chronological"):
        load_dataset(path)


def test_evaluation_checks_training_provenance_across_manifests(dataset):
    path, _ = dataset
    data = load_dataset(path)
    model = fit(data)
    data["episodes"][0]["split"] = "test"
    with pytest.raises(ValueError, match="overlap"):
        evaluate(model, data)
