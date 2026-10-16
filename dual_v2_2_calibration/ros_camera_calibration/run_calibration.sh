#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
DEVICE="${DEVICE:-/dev/v4l/by-id/usb-LRCP_G-720P_LRCP_G-720P_SN0001-video-index0}"
BOARD="${BOARD:-11x8}"
SQUARE="${SQUARE:-0.01}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
SESSION="$CALIB_ROOT/sessions/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$SESSION"
touch "$SESSION/started"
publisher_pid=''
cleanup() {
    if [[ -n "$publisher_pid" ]]; then
        kill "$publisher_pid" 2>/dev/null || true
        wait "$publisher_pid" 2>/dev/null || true
    fi
    if [[ /tmp/calibrationdata.tar.gz -nt "$SESSION/started" ]]; then
        cp /tmp/calibrationdata.tar.gz "$SESSION/calibrationdata.tar.gz"
        tar -xOf "$SESSION/calibrationdata.tar.gz" ost.yaml > "$SESSION/ost.yaml"
        echo "Calibration saved to $SESSION"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'device=%s\ninner_corners=%s\nsquare_m=%s\nresolution=%sx%s\n' \
    "$DEVICE" "$BOARD" "$SQUARE" "$WIDTH" "$HEIGHT" > "$SESSION/settings.txt"
"$CALIB_ROOT/.venv/bin/python" -u "$CALIB_ROOT/usb_image_publisher.py" \
    --device "$DEVICE" --width "$WIDTH" --height "$HEIGHT" \
    > "$SESSION/publisher.log" 2>&1 &
publisher_pid=$!
sleep 2
if ! kill -0 "$publisher_pid" 2>/dev/null; then
    cat "$SESSION/publisher.log"
    exit 1
fi
echo "Board: $BOARD inner corners, square: $SQUARE m; session: $SESSION"
echo 'Move board across view and vary distance/tilt. Click CALIBRATE, then SAVE, then press q.'
PYTHONUNBUFFERED=1 ros2 run camera_calibration cameracalibrator \
    --size "$BOARD" --square "$SQUARE" --camera_name wrist_camera \
    --no-service-check --ros-args -r image:=/wrist_camera/image_raw \
    -r camera:=/wrist_camera 2>&1 | tee "$SESSION/calibrator.log"
