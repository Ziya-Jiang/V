import numpy as np
import matplotlib.pyplot as plt

import torch
import cv2

def normalize_depth(depth_image):
    """
    Normalize the depth image to the range [0, 1]
    :param depth_image: Depth image
    :return: Normalized depth image
    """
    min_depth = np.min(depth_image)
    max_depth = np.max(depth_image)
    
    if max_depth - min_depth == 0:
        print("Warning: Depth image has no variation. Returning zero array.")
        return np.zeros_like(depth_image, dtype=np.float32)
    
    normalized_depth = (depth_image - min_depth) / (max_depth - min_depth)
    return normalized_depth

def viridis_cmap(gray: np.ndarray) -> np.ndarray:
    """
    Visualize a single-channel image using matplotlib's viridis color map
    yellow is high value, blue is low
    :param gray: np.ndarray, (H, W) or (H, W, 1) unscaled
    :return: (H, W, 3) float32 in [0, 1]
    """
    colored = plt.cm.viridis(plt.Normalize()(gray.squeeze()))[..., :-1]
    return colored.astype(np.float32)

def turbo_cmap(gray: np.ndarray) -> np.ndarray:
    """
    Visualize a single-channel image using matplotlib's turbo color map
    yellow is high value, blue is low
    :param gray: np.ndarray, (H, W) or (H, W, 1) unscaled
    :return: (H, W, 3) float32 in [0, 1]
    """
    colored = plt.cm.turbo(plt.Normalize()(gray.squeeze()))[..., :-1]
    return colored.astype(np.float32)

def to8b(img):
    """
    Convert a float image to uint8
    :param img: np.ndarray, (H, W) or (H, W, 3) float32 in [0, 1]
    :return: np.ndarray, (H, W) or (H, W, 3) uint8 in [0, 255]
    """
    # if image range is [0, 1]
    if img.max() > 1.0 or img.min() < 0.0:
        img = (img - img.min()) / (img.max() - img.min())
    img = img * 255.0
    img = img.astype(np.uint8)
    return img


def save_depth_npy(depth_map, filename):
    """
    Save the depth map as a npy file
    :param depth_map: Depth map to save
    :param filename: Filename to save the depth map
    """
    # Save the depth map as a npy file
    np.save(filename, depth_map)


def save_rgb_png(rgb_image, filename):
    """
    Save the RGB image as a png file
    :param rgb_image: RGB image to save
    :param filename: Filename to save the RGB image
    """
    rgb_image = to8b(rgb_image)
    
    # Convert the RGB image to BGR format for OpenCV
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    
    # Save the image using OpenCV
    cv2.imwrite(filename, bgr_image)


def save_depth_png(depth_map, filename, max_depth=2000):
    """
    Save the depth map as a png file, mapping the depth values [0, max_depth] to [0, 2**16 -1]
    :param depth_image: Depth image to save
    :param filename: Filename to save the depth image
    :param max_depth: Maximum depth value for normalization, unit is mm
    """
    # Normalize the depth map to the range [0, 2**16 - 1]
    # depth_image = (depth_map / max_depth) * (2**16 - 1)
    depth_mm = np.rint(depth_map).astype(np.uint16)
    cv2.imwrite(filename, depth_mm, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    
    # # Convert the depth image to uint16
    # depth_image = depth_image.astype(np.uint16)
    
    # # Save the image using OpenCV
    # cv2.imwrite(filename, depth_image)
    

def read_depth_png(filename, max_depth=2000):
    """
    Read the depth image from a png file
    :param filename: Filename to read the depth image
    :return: Depth image
    """
    # Read the image using OpenCV
    depth_image = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    
    # Convert the depth image to float32
    depth_image = depth_image.astype(np.float32)

    # Normalize the depth image to the range [0, max_depth]
    depth_image = (depth_image / (2**16 - 1)) * max_depth
    depth_image[depth_image > max_depth] = 0  # ignore the depth value larger than max depth
    
    return depth_image


def draw_label_at_mask_center(rgb_image, mask, label_text, font_scale=1.0, 
                            font_thickness=2, text_color=(255, 255, 255), 
                            bg_color=None, font=cv2.FONT_HERSHEY_SIMPLEX):
    """
    Draw a text label at the geometric center of a mask in an RGB image.
    
    Args:
        rgb_image (numpy.ndarray): RGB image array (H, W, 3)
        mask (numpy.ndarray): Binary mask array (H, W) where 1 indicates target object
        label_text (str): Text to draw
        font_scale (float): Font scale factor
        font_thickness (int): Font thickness
        text_color (tuple): RGB color for text (R, G, B)
        bg_color (tuple): RGB color for background rectangle, None for no background
        font (int): OpenCV font type
    
    Returns:
        numpy.ndarray: RGB image with label drawn
    """
    # Create a copy of the original image
    result_image = rgb_image.copy()
    
    # Find the geometric center of the mask
    y_coords, x_coords = np.where(mask == 1)
    
    if len(y_coords) == 0:
        print("Warning: No pixels with value 1 found in mask")
        return result_image
    
    # Calculate centroid
    center_x = int(np.mean(x_coords))
    center_y = int(np.mean(y_coords))
    
    # Get text size for positioning
    (text_width, text_height), baseline = cv2.getTextSize(
        label_text, font, font_scale, font_thickness
    )
    
    # Calculate text position (center the text on the centroid)
    text_x = center_x - text_width // 2
    text_y = center_y + text_height // 2
    
    # Draw background rectangle if specified
    if bg_color is not None:
        padding = 2
        cv2.rectangle(
            result_image,
            (text_x - padding, text_y - text_height - padding),
            (text_x + text_width + padding, text_y + baseline + padding),
            bg_color,
            -1
        )
    
    # Draw the text
    cv2.putText(
        result_image,
        label_text,
        (text_x, text_y),
        font,
        font_scale,
        text_color,
        font_thickness
    )
    
    return result_image


def concatenate_images_horizontal(img1_path, img2_path, gap_width=20, output_path=None):
    """
    左右拼接两张图像，中间留有缝隙
    
    Args:
        img1_path: 左侧图像路径
        img2_path: 右侧图像路径
        gap_width: 中间缝隙宽度（像素）
        output_path: 输出图像路径，如果为None则不保存
    
    Returns:
        concatenated_image: 拼接后的图像
    """
    # 读取图像
    img1 = cv2.imread(img1_path)
    img2 = cv2.imread(img2_path)
    
    if img1 is None or img2 is None:
        print(f"Error: Could not load images from {img1_path} or {img2_path}")
        return None
    
    # 获取图像尺寸
    h1, w1, c1 = img1.shape
    h2, w2, c2 = img2.shape
    
    # 统一高度（取最大高度）
    max_height = max(h1, h2)
    
    # 如果图像高度不同，调整尺寸
    if h1 != max_height:
        aspect_ratio = w1 / h1
        new_width = int(max_height * aspect_ratio)
        img1 = cv2.resize(img1, (new_width, max_height))
        w1 = new_width
    
    if h2 != max_height:
        aspect_ratio = w2 / h2
        new_width = int(max_height * aspect_ratio)
        img2 = cv2.resize(img2, (new_width, max_height))
        w2 = new_width
    
    # 创建拼接图像（包含缝隙）
    total_width = w1 + gap_width + w2
    concatenated = np.ones((max_height, total_width, 3), dtype=np.uint8) * 0  # 黑色背景
    
    # 放置图像
    concatenated[:, :w1] = img1
    concatenated[:, w1+gap_width:w1+gap_width+w2] = img2
    
    # 保存图像
    if output_path:
        cv2.imwrite(output_path, concatenated)
        print(f"Concatenated image saved to: {output_path}")
    
    return concatenated


def prepare_third_camera_rgb_for_vggt(rgb, stereo_width, stereo_height):
    """Align the third (diagonal) camera RGB with stereo for VGGT.

    Legacy setup: scale to ``(stereo_width, round(stereo_height * 4/3))`` (e.g. 1280x960)
    then keep the bottom ``stereo_height`` rows (crop 240px from the top at 720p).

    Cameras that already output ``(stereo_height, stereo_width)`` (e.g. Orbbec 720p)
    are returned unchanged to avoid vertical stretching.
    """
    if rgb is None:
        return None
    h, w = rgb.shape[:2]
    if w == stereo_width and h == stereo_height:
        return rgb
    tall_h = int(round(stereo_height * 4.0 / 3.0))
    rgb_scaled = cv2.resize(rgb, (stereo_width, tall_h), interpolation=cv2.INTER_LINEAR)
    crop_top = tall_h - stereo_height
    return rgb_scaled[crop_top:, :, :]