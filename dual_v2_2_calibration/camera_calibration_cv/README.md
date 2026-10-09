# camera_calibration_cv

conda activate calibration

Camera calibration workflow for robot systems that support:

- **Calibration type**
  - `eye_in_hand`: camera is mounted on robot end-effector
  - `eye_to_hand`: camera is fixed in the workspace, board is moved by robot
- **Camera type**: selected by config.yaml
- **Robot type**: selected by config.yaml

This repository focuses on camera capture, image quality check, data logging, and calibration.  
Robot motion control is handled by an external controller.  
Robot state (joint values / pose) is optionally received over UDP.

## Workflow Overview

Calibration is performed in 3 steps:

1. **Pose collection in gravity compensation mode**
   - User physically guides robot to different poses.
   - `detect_chessboard.py` continuously checks if chessboard corners are valid.
   - When image quality is good, user records the robot joint position using external controller.
   - Repeat until enough diverse viewpoints are collected.

2. **Trajectory replay and synchronized data capture**
   - External robot controller replays the collected poses.
   - At each capture point, save:
     - camera image
     - robot state (if enabled in `config.yaml`)

3. **Calibration**
   - Use saved images, robot states and robot model for solving:
     - camera intrinsic matrix
     - distortion parameters
     - extrinsic matrix
       - `eye_in_hand`: `ee_to_camera`
       - `eye_to_hand`: `base_to_camera`

## Configuration

Project settings are stored in `config.yaml`.

Expected fields:

```yaml
calibration_type: eye_in_hand # or eye_to_hand
camera_type: your_camera_type
robot_config_path: /path/to/robot/config.yaml
chessboard_dimension: [rows, cols]
chessboard_square_edge_size_m: 0.01

receive_robot_state:
  enabled: true
  local_ip: 127.0.0.1
  local_port: 5005
  remote_ip: 0.0.0.0
  remote_port: 5005

calibration_path: ./calibration/session_001
calibrate_good_image_only: true
```

Notes:

- `chessboard_dimension` must match the calibration board square count.
- `chessboard_square_edge_size_m` must be in meters.
- If `receive_robot_state.enabled: false`, user need to edit the `records.csv` and provide the `joint_position.csv` and `ee_pose.csv` file before calibration phase.

## Example

### `detect_chessboard.py`

Live camera checker for calibration board quality.

- Continuously reads camera frames.
- Detects chessboard corners.
- Draws detected corners on the image.
- Displays quality status (good / not good) for capture readiness.
- `q` to quit

### `record_data.py`

Interactive capture utility for calibration data.

- Shows same visualization as `detect_chessboard.py`.
- Waits for keyboard input to capture current frame. (`g` to capture and mark `good`, `b` to capture and mark `bad`, `q` to quit)
- Saves image to `calibration_path`.
- Saves robot state snapshot at the same moment if `receive_robot_state` is enabled.

### `calibrate.py`

Runs final calibration from saved data.

- Inputs:
  - images and robot state from `records.py`
  - robot description (robot config)
- Outputs:
  - camera intrinsics + distortion
  - `ee_to_camera` for `eye_in_hand`
  - `base_to_camera` for `eye_to_hand`
  - calibration report (reprojection error and sample filtering info)

## Suggested Data Layout

```text
calibration_path/
    images/
        000001.png
        000002.png
    joint_position.csv
    ee_pose.csv
    records.csv
    results/
        intrinsics.yaml
        extrinsics.yaml
        report.json
```

## Typical Usage

1. Configure `config.yaml`.
2. Run `detect_chessboard.py` to validate board visibility and corner quality.
3. Run `record_data.py` while replaying trajectory and capture enough samples.
4. Run `calibrate.py` to calibrate on saved data.
