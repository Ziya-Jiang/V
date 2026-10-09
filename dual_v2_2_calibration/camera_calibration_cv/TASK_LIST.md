# Task List: Config and Script Implementation

This file defines the implementation scope, expected content, and expected performance for:

- `config.yaml`
- `verify_image.py`
- `record_data.py`
- calibration execution script

## 1) `config.yaml`

### Required content

- `calibration_type`: `eye_in_hand` or `eye_to_hand`
- `camera_type`: camera driver/adapter identifier
- `robot_config_path`: absolute or relative path to robot config file
- `chessboard_dimension`: `[rows, cols]` as inner-corner counts
- `chessboard_square_edge_size_m`: checker square edge size (meters)
- `receive_robot_state`:
  - `enabled`: `true`/`false`
  - `local_ip`:     UDP receive IP
  - `local_port`:   UDP receive port
  - `remote_ip`:    UDP send to IP
  - `remote_port`:  UDP send to port
- `calibration_path`: output root folder for current session (under /calibration)
- `calibrate_good_image_only`: `true`/`false`

## 2) `detect_chessboard.py`

### Required behavior

- Open camera stream based on the configured camera type from depth_sensor repo. Add the camera to depth_sensor if not exist.
- Continuously detect chessboard corners.
- Draw corners and quality label on preview frame.
- Indicate `GOOD` when all expected corners are found with stable detection.
- Provide keyboard quit command (`q`). Print the command usage on terminal to remaind user.

## 3) `record_data.py`

### Required behavior

- Reuse the same live preview and quality feedback as `detect_chessboard.py`.
- Accept keyboard capture input (`g` to capture and mark `good`, `b` to capture and mark `bad`, `q` to quit).
- On capture:
  - save image file with monotonic index/timestamp
  - if robot-state enabled, save synchronized robot state snapshot
  - append one row to record.csv
- Print the command usage on terminal to remaind user.

### File outputs
located inside config.yaml's calibration_path
- `images/<id>.png`
- `joint_position.csv` with first line header (joint id)
- `ee_pose.csv` with first line header (x,y,z,qw,qx,qy,qz)
- `records.csv` with at least:
  - sample id
  - image path
  - robot joint position path
  - robot ee pose path
  - quality flag

## 4) `calibrate.py`

### Required behavior

- Load saved dataset, give warning if dataset dismatch or missing
- IK & FK use func from robot_utilities repo (See robot_utilities\test\mdh_robot_test.py)
- Solve camera intrinsics + distortion.
- Solve hand-eye transform based on mode:
  - `eye_in_hand` -> output `ee_to_camera`
  - `eye_to_hand` -> output `base_to_camera`
- Export machine-readable results and report.

### Suggested outputs
located inside config.yaml's calibration_path
- `results/intrinsics.yaml`
- `results/extrinsics.yaml`
- `results/report.json` with:
  - date and time
  - copy of config.yaml
  - reprojection error (mean/max)
  - used sample IDs

## 5) Delete `old` folder

## 6) Delete `TASK_LIST.md`
