#
# Copyright (C) 2023, Inria
# GRAPHDECO research group, https://team.inria.fr/graphdeco
# All rights reserved.
#
# This software is free for non-commercial, research and evaluation use 
# under the terms of the LICENSE.md file.
#
# For inquiries contact  george.drettakis@inria.fr
#

import numpy as np
import torch
import pickle
import torch.nn.functional as F
from scene import Scene
from PIL import Image, ImageDraw, ImageFont
import matplotlib as mpl
import os
import subprocess
from tqdm import tqdm
from os import makedirs
from gaussian_renderer import render
import torchvision
from utils.general_utils import safe_state
from argparse import ArgumentParser
from arguments import ModelParams, PipelineParams, get_combined_args
from gaussian_renderer import GaussianModel
from utils.camera_utils import get_current_view
from tqdm import tqdm, trange
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
from einops import rearrange, einsum
from scene import skip_feat_decoder
import cv2
import featsplat_editor

def fit_feature_pca(gaussians, max_samples=100_000):
    """Fit one stable PCA color mapping for the complete camera trajectory."""
    features = gaussians.get_distill_features.detach()
    valid = torch.isfinite(features).all(dim=1)
    valid &= torch.linalg.vector_norm(features, dim=1) > 1e-8
    features = features[valid]
    if features.shape[0] < 3:
        raise ValueError("At least three valid Gaussian features are required for PCA")

    if features.shape[0] > max_samples:
        sample_indices = torch.linspace(
            0,
            features.shape[0] - 1,
            max_samples,
            device=features.device,
        ).long()
        features = features[sample_indices]

    mean = features.mean(dim=0)
    centered = features - mean
    _, _, components = torch.pca_lowrank(
        centered, q=3, center=False, niter=4
    )
    projected = centered @ components
    lower = torch.quantile(projected, 0.02, dim=0)
    upper = torch.quantile(projected, 0.98, dim=0)
    scale = torch.clamp(upper - lower, min=1e-6)
    return mean, components, lower, scale


def render_feature_pca(rendered_feat, pca_mapping):
    """Project a rendered CxHxW feature map to a consistent RGB image."""
    mean, components, lower, scale = pca_mapping
    feature_hwc = rendered_feat.permute(1, 2, 0)
    valid = torch.isfinite(feature_hwc).all(dim=2)
    valid &= torch.linalg.vector_norm(feature_hwc, dim=2) > 1e-8
    safe_feature = torch.nan_to_num(feature_hwc)
    feature_rgb = ((safe_feature - mean) @ components - lower) / scale
    feature_rgb = torch.clamp(feature_rgb, 0.0, 1.0)
    feature_rgb[~valid] = 0.0
    return feature_rgb


def tensor_rgb_to_numpy(image):
    return (
        torch.clamp(image, 0.0, 1.0)
        .mul(255)
        .byte()
        .permute(1, 2, 0)
        .contiguous()
        .cpu()
        .numpy()
    )


def estimate_world_up(views):
    """Approximate world up from camera orientations (camera +Y points down)."""
    up = np.zeros(3, dtype=np.float64)
    for view in views:
        up -= np.asarray(view.R, dtype=np.float64)[:, 1]
    norm = float(np.linalg.norm(up))
    if norm < 1e-6:
        return None
    return up / norm


def prepare_prompt_heatmap(args, model_path, gaussians, part_level, views):
    if not args.text_query:
        return None
    if not args.clip_feat:
        raise ValueError("--text_query requires --clip_feat")

    decoder = skip_feat_decoder(
        gaussians.get_distill_features.shape[1],
        part_level=part_level,
    ).cuda()
    decoder_weight_path = os.path.join(model_path, "feat_decoder.pth")
    if not os.path.exists(decoder_weight_path):
        raise ValueError(f"Feature decoder not found: {decoder_weight_path}")
    decoder.load_state_dict(
        torch.load(decoder_weight_path, map_location="cuda"),
        strict=True,
    )
    decoder.eval()

    segmenter = featsplat_editor.clip_segmenter(
        gaussians,
        decoder,
        clip_device="cuda",
    )
    words = (
        segmenter.canonical_words.copy()
        + [args.neg_text_query]
        + [args.text_query]
    )
    text_features = segmenter.get_text_embeddings(words).cuda()
    instances = compute_prompt_obbs(
        args,
        segmenter,
        text_features,
        gaussians,
        estimate_world_up(views),
    )
    for rank, instance in enumerate(instances, start=1):
        print(
            f'Prompt "{args.text_query}" instance {rank}: '
            f"{instance['count']} Gaussians, "
            f"mean similarity {instance['mean_similarity']:.3f}"
        )
    return {
        "decoder": decoder,
        "text_features": text_features,
        "bbox_instances": instances,
    }


def prompt_similarity_per_gaussian(segmenter, text_features, gaussians):
    """Return per-Gaussian prompt similarity normalized to [0, 1]."""
    probabilities = []
    for chunk in gaussians.get_distill_features.split(2**14):
        decoded = segmenter.decoder_infer(chunk, level="object")
        decoded = decoded / (decoded.norm(dim=1, keepdim=True) + 1e-6)
        logits = einsum(
            decoded.float(),
            text_features.float(),
            "n c, m c -> n m",
        )
        probabilities.append((logits / 0.05).softmax(dim=1)[:, -1])
    similarity = torch.cat(probabilities)
    similarity = similarity - similarity.min()
    similarity = similarity / torch.clamp(similarity.max(), min=1e-6)
    return similarity.detach().cpu().numpy()


def orient_axes(axes, world_up):
    """Disambiguate PCA axis signs into a stable right-handed object frame."""
    axes = np.array(axes, dtype=np.float64)
    # The axis closest to world up is flipped to point up, matching how 6D
    # poses are usually drawn for objects resting on a support surface.
    aligned = int(np.argmax(np.abs(world_up @ axes))) if world_up is not None else -1
    for column in range(3):
        if column == aligned:
            reference = float(world_up @ axes[:, column])
        else:
            # eigh signs are arbitrary; pin them down so the pose is stable.
            reference = float(axes[np.argmax(np.abs(axes[:, column])), column])
        if reference < 0:
            axes[:, column] *= -1
    if np.linalg.det(axes) < 0:
        axes[:, 1 if aligned == 2 else 2] *= -1
    return axes


def fit_oriented_bbox(points, quantile, padding, world_up=None):
    """Fit a PCA-oriented 3D box and return its corners and 6D pose."""
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, axes = np.linalg.eigh(np.cov(centered, rowvar=False))
    axes = orient_axes(axes[:, ::-1], world_up)

    local_points = centered @ axes
    lower = np.quantile(local_points, quantile, axis=0)
    upper = np.quantile(local_points, 1.0 - quantile, axis=0)
    margin = np.maximum((upper - lower) * padding, 1e-4)
    lower -= margin
    upper += margin
    local_corners = np.array(
        [
            [x, y, z]
            for x in (lower[0], upper[0])
            for y in (lower[1], upper[1])
            for z in (lower[2], upper[2])
        ],
        dtype=np.float64,
    )
    local_center = (lower + upper) * 0.5
    half_extent = (upper - lower) * 0.5
    return {
        "corners": (centroid + local_corners @ axes.T).astype(np.float32),
        "center": (centroid + local_center @ axes.T).astype(np.float32),
        "axes": axes.astype(np.float32),
        "half_extent": half_extent.astype(np.float32),
    }


def compute_prompt_obbs(args, segmenter, text_features, gaussians, world_up=None):
    """Fit one oriented 3D box per prompt-matching Gaussian cluster."""
    similarity = prompt_similarity_per_gaussian(
        segmenter, text_features, gaussians
    )
    selected = similarity >= args.bbox_threshold
    if np.count_nonzero(selected) < 20:
        raise ValueError(
            f'Prompt "{args.text_query}" selected fewer than 20 Gaussians at '
            f"bbox_threshold={args.bbox_threshold:g}; lower --bbox_threshold"
        )

    xyz = gaussians.get_xyz.detach().cpu().numpy()
    selected_xyz = xyz[selected]
    selected_similarity = similarity[selected]
    labels = DBSCAN(
        eps=args.bbox_cluster_eps, min_samples=20
    ).fit(selected_xyz).labels_

    groups = [labels == label for label in np.unique(labels[labels != -1])]
    if not groups:
        print(
            "[Warning] Prompt selection did not form a DBSCAN cluster; "
            "fitting one box over all thresholded Gaussians"
        )
        groups = [np.ones(len(selected_xyz), dtype=bool)]

    # Rank by total similarity mass so a large weakly-matching blob cannot
    # outrank a compact, strongly-matching instance.
    groups.sort(key=lambda group: -float(selected_similarity[group].sum()))

    instances = []
    for group in groups[: args.bbox_max_instances]:
        points = selected_xyz[group]
        if len(points) < 3:
            continue
        instance = fit_oriented_bbox(
            points, args.bbox_quantile, args.bbox_padding, world_up
        )
        instance["count"] = int(len(points))
        instance["mean_similarity"] = float(selected_similarity[group].mean())
        instances.append(instance)
    if not instances:
        raise ValueError("Not enough prompt-selected points to fit a 6D box")
    return instances


def render_prompt_heatmap(rendered_feat, heatmap_context):
    decoder = heatmap_context["decoder"]
    text_features = heatmap_context["text_features"]
    decoded_features = decoder(rendered_feat.unsqueeze(0))
    rendered_clip_feat = decoded_features[1].squeeze(0)
    rendered_clip_feat = rendered_clip_feat / (
        rendered_clip_feat.norm(dim=0, keepdim=True) + 1e-6
    )
    logits = einsum(
        rendered_clip_feat.float(),
        text_features.float(),
        "c h w, m c -> m h w",
    )
    probability = (logits / 0.01).softmax(dim=0)[-1]
    probability = probability - probability.min()
    probability = probability / torch.clamp(probability.max(), min=1e-6)
    heatmap = mpl.cm.jet(probability.detach().cpu().numpy(), bytes=True)
    return heatmap[..., :3]


def load_overlay_font(image_height, scale=0.04):
    font_size = max(18, round(image_height * scale))
    try:
        return ImageFont.truetype("DejaVuSans.ttf", font_size)
    except OSError:
        return ImageFont.load_default()


def add_panel_title(panel, title):
    image = Image.fromarray(panel, mode="RGB").convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    font = load_overlay_font(image.height, scale=0.032)
    text_box = draw.textbbox((0, 0), title, font=font)
    text_width = text_box[2] - text_box[0]
    text_height = text_box[3] - text_box[1]
    padding_x, padding_y = 14, 7
    left, top = 16, 16
    right = left + text_width + 2 * padding_x
    bottom = top + text_height + 2 * padding_y
    draw.rounded_rectangle(
        (left, top, right, bottom),
        radius=10,
        fill=(12, 16, 24, 175),
        outline=(255, 255, 255, 55),
        width=1,
    )
    draw.text(
        (left + padding_x, top + padding_y - text_box[1]),
        title,
        font=font,
        fill=(255, 255, 255, 235),
    )
    return np.asarray(Image.alpha_composite(image, overlay).convert("RGB"))


def add_prompt_label(panel, prompt):
    """Add a centered translucent prompt label to a panel footer."""
    image = Image.fromarray(panel, mode="RGB").convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    font = load_overlay_font(image.height)
    font_size = getattr(font, "size", max(18, round(image.height * 0.04)))
    normalized_prompt = " ".join(prompt.strip().split())
    if len(normalized_prompt) > 64:
        normalized_prompt = normalized_prompt[:61] + "..."
    text = f'Prompt  ·  "{normalized_prompt}"'
    text_box = draw.textbbox((0, 0), text, font=font)
    text_width = text_box[2] - text_box[0]
    text_height = text_box[3] - text_box[1]
    padding_x = max(16, round(font_size * 0.8))
    padding_y = max(9, round(font_size * 0.45))
    box_width = min(image.width - 32, text_width + 2 * padding_x)
    box_height = text_height + 2 * padding_y
    left = (image.width - box_width) // 2
    top = image.height - box_height - 18
    right = left + box_width
    bottom = top + box_height
    draw.rounded_rectangle(
        (left, top, right, bottom),
        radius=box_height // 2,
        fill=(12, 16, 24, 190),
        outline=(255, 255, 255, 70),
        width=1,
    )
    text_x = (image.width - text_width) // 2
    text_y = top + padding_y - text_box[1]
    draw.text(
        (text_x, text_y),
        text,
        font=font,
        fill=(255, 255, 255, 245),
    )
    return np.asarray(Image.alpha_composite(image, overlay).convert("RGB"))


BBOX_EDGES = (
    (0, 1), (0, 2), (0, 4),
    (1, 3), (1, 5),
    (2, 3), (2, 6),
    (3, 7),
    (4, 5), (4, 6),
    (5, 7),
    (6, 7),
)


def project_points(points, view):
    """Project world points to pixels using the rasterizer's conventions."""
    corners = torch.as_tensor(
        points,
        dtype=view.full_proj_transform.dtype,
        device=view.full_proj_transform.device,
    )
    homogeneous = torch.cat(
        (corners, torch.ones_like(corners[:, :1])),
        dim=1,
    )
    clip = homogeneous @ view.full_proj_transform
    in_front = clip[:, 3] > 1e-5
    ndc = clip[:, :2] / torch.clamp(clip[:, 3:4], min=1e-5)
    # Matches ndc2Pix in the CUDA rasterizer, which does not flip the y axis.
    pixel_x = ((ndc[:, 0] + 1.0) * view.image_width - 1.0) * 0.5
    pixel_y = ((ndc[:, 1] + 1.0) * view.image_height - 1.0) * 0.5
    pixels = torch.stack((pixel_x, pixel_y), dim=1).detach().cpu().numpy()
    depth = clip[:, 3].detach().cpu().numpy()
    return pixels, in_front.detach().cpu().numpy(), depth


# Standard 6D pose convention: X red, Y green, Z blue.
POSE_AXIS_COLORS = ((236, 62, 56), (58, 214, 96), (72, 142, 255))
POSE_AXIS_LABELS = ("X", "Y", "Z")
OUTLINE_COLOR = (8, 18, 24)


def draw_pose_axes(draw, instance, view, font):
    """Draw the object frame at the box center with RGB-coded XYZ axes."""
    center = instance["center"].astype(np.float64)
    axes = instance["axes"].astype(np.float64)
    half_extent = instance["half_extent"].astype(np.float64)
    # Extend slightly past the box faces, and keep thin axes (e.g. a plate's
    # normal) long enough to stay readable.
    lengths = 1.35 * np.maximum(half_extent, 0.5 * float(half_extent.max()))

    endpoints = np.stack(
        [center] + [center + axes[:, i] * lengths[i] for i in range(3)]
    )
    pixels, visible, depth = project_points(endpoints, view)
    if not visible[0]:
        return

    origin = pixels[0]
    # Draw far axes first so nearer ones occlude them correctly.
    order = sorted(range(3), key=lambda i: -depth[i + 1])
    for index in order:
        if not visible[index + 1]:
            continue
        tip = pixels[index + 1]
        direction = tip - origin
        length = float(np.hypot(*direction))
        if length < 4.0:
            continue
        unit = direction / length
        normal = np.array([-unit[1], unit[0]])
        color = POSE_AXIS_COLORS[index]

        head = float(np.clip(length * 0.22, 9.0, 26.0))
        shaft_end = tip - unit * head * 0.75
        draw.line(
            [tuple(origin), tuple(shaft_end)], fill=OUTLINE_COLOR, width=9
        )
        draw.line([tuple(origin), tuple(shaft_end)], fill=color, width=5)
        draw.polygon(
            [
                tuple(tip),
                tuple(tip - unit * head + normal * head * 0.5),
                tuple(tip - unit * head - normal * head * 0.5),
            ],
            fill=color,
            outline=OUTLINE_COLOR,
        )
        label_at = tip + unit * head * 0.9
        draw.text(
            tuple(label_at),
            POSE_AXIS_LABELS[index],
            font=font,
            fill=color,
            anchor="mm",
            stroke_width=3,
            stroke_fill=OUTLINE_COLOR,
        )

    radius = 7
    draw.ellipse(
        (
            origin[0] - radius,
            origin[1] - radius,
            origin[0] + radius,
            origin[1] + radius,
        ),
        fill=(250, 250, 250),
        outline=OUTLINE_COLOR,
        width=2,
    )


def draw_projected_6d_bboxes(rgb, instances, view):
    """Draw each oriented 3D box plus its XYZ pose axes on the RGB frame."""
    image = Image.fromarray(rgb, mode="RGB")
    draw = ImageDraw.Draw(image)
    font = load_overlay_font(image.height, scale=0.035)
    for instance in instances:
        pixels, visible, _ = project_points(instance["corners"], view)
        for first, second in BBOX_EDGES:
            if not (visible[first] and visible[second]):
                continue
            coordinates = [tuple(pixels[first]), tuple(pixels[second])]
            draw.line(coordinates, fill=OUTLINE_COLOR, width=7)
            draw.line(coordinates, fill=(35, 235, 185), width=4)
        for index, point in enumerate(pixels):
            if not visible[index]:
                continue
            x, y = point
            radius = 5
            draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                fill=(255, 245, 165),
                outline=OUTLINE_COLOR,
                width=2,
            )
        draw_pose_axes(draw, instance, view, font)
    return np.asarray(image)


def compose_range_video_frame(
    render_pkg,
    pca_mapping,
    heatmap_context,
    prompt,
    view,
    rotate_ccw_90=False,
):
    rgb = tensor_rgb_to_numpy(render_pkg["render"])
    feature = (
        render_feature_pca(render_pkg["render_feat"], pca_mapping)
        .mul(255)
        .byte()
        .contiguous()
        .cpu()
        .numpy()
    )
    if heatmap_context is None:
        if rotate_ccw_90:
            rgb = np.rot90(rgb, k=1).copy()
            feature = np.rot90(feature, k=1).copy()
        return np.concatenate((rgb, feature), axis=1)

    heatmap = render_prompt_heatmap(
        render_pkg["render_feat"],
        heatmap_context,
    )
    bbox_rgb = draw_projected_6d_bboxes(
        rgb,
        heatmap_context["bbox_instances"],
        view,
    )
    if rotate_ccw_90:
        rgb = np.rot90(rgb, k=1).copy()
        feature = np.rot90(feature, k=1).copy()
        heatmap = np.rot90(heatmap, k=1).copy()
        bbox_rgb = np.rot90(bbox_rgb, k=1).copy()
    rgb_panel = add_panel_title(rgb, "RGB")
    feature_panel = add_panel_title(feature, "PCA FEATURE")
    heatmap_panel = add_prompt_label(
        add_panel_title(heatmap, "PROMPT HEATMAP"),
        prompt,
    )
    bbox_panel = add_prompt_label(
        add_panel_title(bbox_rgb, "6D POSE + BBOX"),
        prompt,
    )
    top_row = np.concatenate((rgb_panel, feature_panel), axis=1)
    bottom_row = np.concatenate((heatmap_panel, bbox_panel), axis=1)
    return np.concatenate((top_row, bottom_row), axis=0)


@torch.no_grad()
def write_camera_range_video(
    args,
    model_path,
    iteration,
    views,
    gaussians,
    pipeline,
    background,
    part_level=False,
):
    """Render every original camera in an inclusive range to one MP4."""
    start_idx, end_idx = args.camera_frame_range
    if start_idx < 0:
        start_idx += len(views)
    if end_idx < 0:
        end_idx += len(views)
    if not (0 <= start_idx < len(views) and 0 <= end_idx < len(views)):
        raise ValueError(
            "camera_frame_range must be inside "
            f"[-{len(views)}, {len(views) - 1}]"
        )
    if end_idx < start_idx:
        raise ValueError("camera_frame_range END must be greater than or equal to START")
    if args.camera_frame_stride < 1:
        raise ValueError("camera_frame_stride must be at least 1")
    if args.video_fps <= 0:
        raise ValueError("video_fps must be positive")
    if args.pca_sample_size < 3:
        raise ValueError("pca_sample_size must be at least 3")
    if not 0.0 <= args.bbox_threshold <= 1.0:
        raise ValueError("bbox_threshold must be between 0 and 1")
    if args.bbox_cluster_eps <= 0:
        raise ValueError("bbox_cluster_eps must be positive")
    if not 0.0 <= args.bbox_quantile < 0.5:
        raise ValueError("bbox_quantile must be in [0, 0.5)")
    if args.bbox_padding < 0:
        raise ValueError("bbox_padding must be non-negative")
    if args.bbox_max_instances < 1:
        raise ValueError("bbox_max_instances must be at least 1")

    camera_indices = list(
        range(start_idx, end_idx + 1, args.camera_frame_stride)
    )
    if camera_indices[-1] != end_idx:
        camera_indices.append(end_idx)

    pca_mapping = fit_feature_pca(gaussians, args.pca_sample_size)
    heatmap_context = prepare_prompt_heatmap(
        args,
        model_path,
        gaussians,
        part_level,
        views,
    )
    first_render = render(
        views[camera_indices[0]],
        gaussians,
        pipeline,
        background,
        render_features=True,
    )
    first_frame = compose_range_video_frame(
        first_render,
        pca_mapping,
        heatmap_context,
        args.text_query,
        views[camera_indices[0]],
        args.rotate_ccw_90,
    )
    height, width = first_frame.shape[:2]

    if args.video_output:
        video_path = os.path.abspath(args.video_output)
    else:
        video_dir = os.path.join(
            model_path,
            "interpolating_camera",
            "ours_{}".format(iteration),
            "videos",
        )
        output_kind = (
            "rgb_feature_heatmap_6dbbox_grid"
            if heatmap_context is not None
            else "rgb_feature"
        )
        video_path = os.path.join(
            video_dir,
            f"{output_kind}_frames_{start_idx:05d}_{end_idx:05d}"
            f"_stride_{args.camera_frame_stride}.mp4",
        )
    os.makedirs(os.path.dirname(video_path), exist_ok=True)

    command = [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(args.video_fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-crf",
        str(args.video_crf),
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        video_path,
    ]
    try:
        encoder = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("ffmpeg is required to write MP4 video") from exc

    try:
        assert encoder.stdin is not None
        encoder.stdin.write(first_frame.tobytes())
        for camera_idx in tqdm(
            camera_indices[1:],
            desc="Rendering RGB + Feature video",
        ):
            render_pkg = render(
                views[camera_idx],
                gaussians,
                pipeline,
                background,
                render_features=True,
            )
            frame = compose_range_video_frame(
                render_pkg,
                pca_mapping,
                heatmap_context,
                args.text_query,
                views[camera_idx],
                args.rotate_ccw_90,
            )
            encoder.stdin.write(frame.tobytes())
        encoder.stdin.close()
        stderr = encoder.stderr.read().decode("utf-8", errors="replace")
        return_code = encoder.wait()
    except Exception:
        encoder.kill()
        encoder.wait()
        if os.path.exists(video_path):
            os.remove(video_path)
        raise
    finally:
        if encoder.stderr is not None:
            encoder.stderr.close()

    if return_code != 0:
        if os.path.exists(video_path):
            os.remove(video_path)
        raise RuntimeError(f"ffmpeg failed with code {return_code}: {stderr.strip()}")

    frame_count = len(camera_indices)
    panel_description = (
        "RGB + Feature + Prompt Heatmap + 6D Bounding Box"
        if heatmap_context is not None
        else "RGB + Feature"
    )
    print(
        f"Saved {frame_count} {panel_description} frames "
        f"at {args.video_fps:g} FPS: "
        f"{video_path}"
    )
    return video_path


@torch.no_grad()
def render_interpolating_trajectory(args, model_path, iteration, views, gaussians, pipeline, background, camera_slerp_list=None,
                                    with_feat=False, clip_feat=False, dino_feat=False, with_editing=True, text_query="",
                                    neg_text_query="", step_size=100, part_feat=False):
    if clip_feat or dino_feat:
        assert with_feat
        my_feat_decoder = skip_feat_decoder(gaussians._distill_features.shape[1], part_level=part_feat).cuda()
        decoder_weight_path = os.path.join(model_path, "feat_decoder.pth")
        assert os.path.exists(decoder_weight_path)
        decoder_weight_dict = torch.load(decoder_weight_path)
        my_feat_decoder.load_state_dict(decoder_weight_dict, strict=True)
        my_feat_decoder.eval()
        if text_query:
            clip_segmeter = featsplat_editor.clip_segmenter(gaussians, my_feat_decoder)
            word_list = clip_segmeter.canonical_words.copy() + [neg_text_query] + [text_query]
            text_features_mc = clip_segmeter.get_text_embeddings(word_list).cuda()
        if args.conditional_text_query:
            assert text_query
            conditional_text_features_mc = clip_segmeter.get_text_embeddings(["object", args.conditional_text_query]).cuda()
        
    render_path = os.path.join(model_path,
                               'interpolating_camera',
                               "ours_{}".format(iteration),
                               "renders")

    makedirs(render_path, exist_ok=True)

    total_step_counter = 0

    if with_editing:
        modifier_dict_pt = os.path.join(model_path,
                                       "point_cloud",
                                       "iteration_" + str(iteration),
                                       "editing_modifier.pkl")
        assert os.path.exists(modifier_dict_pt)
        with open(modifier_dict_pt, "rb") as f:
            editing_modifier_dict = pickle.load(f)
        my_gaussian_editor = featsplat_editor.gaussian_editor(editing_modifier_dict)

    assert len(camera_slerp_list) >= 2
    for idx in range(len(camera_slerp_list) - 1):
        start_idx = camera_slerp_list[idx]
        end_idx = camera_slerp_list[idx + 1]
        for i in trange(step_size):
            if with_editing:
                modified_gaussians = my_gaussian_editor.modify_gaussian(gaussians, total_step_counter)
            else:
                modified_gaussians = gaussians
            t = i / step_size
            view = get_current_view(views, start_idx, end_idx, t)
            render_pkg = render(view, modified_gaussians, pipeline, background, render_features=with_feat)
            rendered_rgb = render_pkg["render"]
            save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}'.format(idx, idx + 1, i) + ".png")
            torchvision.utils.save_image(rendered_rgb, save_path)

            if args.render_depth:
                depth_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_depth'.format(idx, idx + 1, i) + ".png")
                depth_np = render_pkg['render_depth'].cpu().numpy().copy()
                depth_viz = depth_np / depth_np.max()
                Image.fromarray((depth_viz * 255).astype(np.uint8), mode='L').save(depth_save_path)

            if with_feat:
                rendered_feat = render_pkg["render_feat"]
                save_principal_feat = F.interpolate(rendered_feat.unsqueeze(0),
                                                    size=(rendered_feat.shape[1], rendered_feat.shape[2]),
                                                    mode='bilinear',
                                                    align_corners=False).squeeze(0)
                principal_feat_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_principal_feat'.format(idx, idx + 1, i) + ".npy")
                np.save(principal_feat_save_path, save_principal_feat.cpu().numpy())
                if clip_feat or dino_feat:
                    # Downsample to save GPU memory
                    # target_H = rendered_feat.shape[1]
                    # target_W = rendered_feat.shape[2]
                    # rendered_feat = F.interpolate(rendered_feat.unsqueeze(0),
                    #                             size=(target_H, target_W),
                    #                             mode='bilinear',
                    #                             align_corners=False)
                    rendered_feat = rendered_feat.unsqueeze(0)
                    rendered_dino_feat, rendered_clip_feat = my_feat_decoder(rendered_feat)
                    rendered_dino_feat = rendered_dino_feat.squeeze(0)
                    rendered_clip_feat = rendered_clip_feat.squeeze(0)
                    # rendered_part_feat = rendered_part_feat.squeeze(0)
                    dino_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_dino_feat'.format(idx, idx + 1, i) + ".npy")
                    clip_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_clip_feat'.format(idx, idx + 1, i) + ".npy")
                    part_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_part_feat'.format(idx, idx + 1, i) + ".npy")
                    # np.save(dino_save_path, rendered_dino_feat.cpu().numpy())
                    # np.save(clip_save_path, rendered_clip_feat.cpu().numpy())

                    if text_query:
                        rendered_clip_feat = rendered_clip_feat / (rendered_clip_feat.norm(dim=0, keepdim=True) + 1e-6)
                        text_probmap = einsum(rendered_clip_feat.float(), text_features_mc.float(), 'c h w, m c -> m h w')
                        text_probmap = text_probmap / 0.01 # Temperature
                        # plt.imshow(normalized_sim > 0.25); plt.show()

                        normalized_sim = text_probmap.softmax(dim=0)[len(word_list) - 1, :, :].float().cpu().numpy()
                        normalized_sim = normalized_sim - normalized_sim.min()
                        normalized_sim = normalized_sim / normalized_sim.max()
                        obj_level_mask = (normalized_sim > 0.6)
                        # normalized_sim = (normalized_sim > 0.6).astype(np.float32)
                        color_arr = mpl.cm.jet(normalized_sim)
                        # color_arr = normalized_sim
                        color_arr = (color_arr * 255).astype(np.uint8)

                        heatmap_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_heatmap'.format(idx, idx + 1, i) + ".png")
                        Image.fromarray(color_arr).save(heatmap_save_path)

                        if args.conditional_text_query:
                            text_probmap = einsum(rendered_clip_feat.float(), conditional_text_features_mc.float(), 'c h w, m c -> m h w')
                            text_probmap = text_probmap / 0.01 # Temperature

                            normalized_sim = text_probmap.softmax(dim=0)[text_probmap.shape[0] - 1, :, :].float().cpu().numpy()
                            normalized_sim[~obj_level_mask] = 0
                            # normalized_sim = normalized_sim - normalized_sim.min()
                            # normalized_sim = normalized_sim / normalized_sim.max()
                            binary_mask = (normalized_sim > 0.6)
                            heatmap_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_obj_con'.format(idx, idx + 1, i) + ".png")
                            color_arr = (binary_mask * 255).astype(np.uint8)
                            Image.fromarray(color_arr).save(heatmap_save_path)

                        if part_feat:
                            rendered_part_feat = rendered_part_feat / (rendered_part_feat.norm(dim=0, keepdim=True) + 1e-6)
                            text_probmap = einsum(rendered_part_feat.float(), text_features_mc.float(), 'c h w, m c -> m h w')
                            text_probmap = text_probmap / 0.1 # Temperature

                            normalized_sim = text_probmap.softmax(dim=0)[len(word_list) - 1, :, :].float().cpu().numpy()
                            color_arr = mpl.cm.jet(normalized_sim)
                            color_arr = (color_arr * 255).astype(np.uint8)

                            part_obj_mask = (normalized_sim > 0.25)
                            part_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_part'.format(idx, idx + 1, i) + ".png")
                            Image.fromarray((part_obj_mask * 255).astype(np.uint8)).save(part_save_path)

                            heatmap_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_part_heatmap'.format(idx, idx + 1, i) + ".png")
                            Image.fromarray(color_arr).save(heatmap_save_path)

                            if args.conditional_text_query:
                                text_probmap = einsum(rendered_part_feat.float(), conditional_text_features_mc.float(), 'c h w, m c -> m h w')
                                text_probmap = text_probmap / 0.01 # Temperature

                                normalized_sim = text_probmap.softmax(dim=0)[text_probmap.shape[0] - 1, :, :].float().cpu().numpy()
                                normalized_sim[~obj_level_mask] = 0
                                binary_mask = mpl.cm.jet(normalized_sim)
                                heatmap_save_path = os.path.join(render_path, '{0}_{1}_step{2:05d}_part_con'.format(idx, idx + 1, i) + ".png")
                                color_arr = (binary_mask * 255).astype(np.uint8)
                                Image.fromarray(color_arr).save(heatmap_save_path)
            
            total_step_counter += 1

def render_sets(dataset : ModelParams, iteration : int, pipeline : PipelineParams, camera_slerp_list : list,
                with_feat : bool, clip_feat : bool, dino_feat : bool, with_editing : bool, text_query : str,
                neg_text_query : str, step_size : int, args):
    gaussians = GaussianModel(dataset.sh_degree, dataset.distill_feature_dim)
    scene = Scene(dataset, gaussians, load_iteration=iteration, shuffle=False)

    bg_color = [1,1,1] if dataset.white_background else [0, 0, 0]
    background = torch.tensor(bg_color, dtype=torch.float32, device="cuda")

    if args.camera_frame_range is not None:
        write_camera_range_video(
            args,
            dataset.model_path,
            scene.loaded_iter,
            scene.getTrainCameras(),
            gaussians,
            pipeline,
            background,
            part_level=(dataset.feature_type == "clip_part"),
        )
        return

    render_interpolating_trajectory(args, dataset.model_path, scene.loaded_iter,
                                    scene.getTrainCameras(), gaussians, pipeline,
                                    background, camera_slerp_list, with_feat, clip_feat, dino_feat,
                                    with_editing, text_query, neg_text_query, step_size)

if __name__ == "__main__":
    # Set up command line argument parser
    parser = ArgumentParser(description="Testing script parameters")
    model = ModelParams(parser, sentinel=True)
    pipeline = PipelineParams(parser)
    parser.add_argument("--iteration", default=-1, type=int)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--camera_slerp_list", help="List of camera slerp indices", nargs="+", type=str, default=[0, 0])
    parser.add_argument("--with_feat", action="store_true", default=False)
    parser.add_argument("--clip_feat", action="store_true", default=False)
    parser.add_argument("--dino_feat", action="store_true", default=False)
    parser.add_argument("--with_editing", action="store_true", default=False)
    parser.add_argument("--text_query", help="Text query for CLIP", type=str, default="")
    parser.add_argument("--neg_text_query", help="Negative text query for CLIP", type=str, default="")
    parser.add_argument("--conditional_text_query", help="Conditional text query for CLIP", type=str, default="")
    parser.add_argument("--step_size", help="Step size for camera slerp", type=int, default=100)
    parser.add_argument("--render_depth", action="store_true", default=False)
    parser.add_argument("--render_normal", action="store_true", default=False)
    parser.add_argument(
        "--camera_frame_range",
        nargs=2,
        type=int,
        metavar=("START", "END"),
        help=(
            "Render every original camera in the inclusive START..END range "
            "to an RGB/Feature video; use END=-1 for the last camera. "
            "With --text_query, output is a four-panel RGB/PCA/heatmap/6D-box grid"
        ),
    )
    parser.add_argument("--video_output", type=str, help="Output path for range MP4")
    parser.add_argument("--video_fps", type=float, default=30.0)
    parser.add_argument("--video_crf", type=int, default=18)
    parser.add_argument("--pca_sample_size", type=int, default=100_000)
    parser.add_argument("--camera_frame_stride", type=int, default=1)
    parser.add_argument(
        "--rotate_ccw_90",
        "--rotate_ccw",
        action="store_true",
        help=(
            "Rotate every panel 90 degrees counterclockwise before adding "
            "titles, producing a landscape video"
        ),
    )
    parser.add_argument(
        "--bbox_threshold",
        type=float,
        default=0.6,
        help="Prompt similarity threshold for 6D box fitting (default: 0.6)",
    )
    parser.add_argument(
        "--bbox_cluster_eps",
        type=float,
        default=0.1,
        help="DBSCAN radius for selecting one prompt instance (default: 0.1)",
    )
    parser.add_argument(
        "--bbox_quantile",
        type=float,
        default=0.01,
        help="Outlier quantile trimmed on each 6D box side (default: 0.01)",
    )
    parser.add_argument(
        "--bbox_padding",
        type=float,
        default=0.05,
        help="Fractional padding around the fitted 6D box (default: 0.05)",
    )
    parser.add_argument(
        "--bbox_max_instances",
        type=int,
        default=1,
        help=(
            "Number of prompt instances to box, ranked by similarity mass "
            "(default: 1)"
        ),
    )
    args = get_combined_args(parser)
    print("Rendering " + args.model_path)

    if args.text_query:
        assert args.clip_feat

    # Initialize system state (RNG)
    safe_state(args.quiet)

    render_sets(model.extract(args), args.iteration, pipeline.extract(args), args.camera_slerp_list,
                args.with_feat, args.clip_feat, args.dino_feat, args.with_editing, args.text_query,
                args.neg_text_query, args.step_size, args)
