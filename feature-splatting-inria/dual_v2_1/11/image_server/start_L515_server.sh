#!/bin/bash
# L515: RGB 1280x720 + Depth 640x480 (separate native streams)
python3 image_server.py \
  --serial f1481047 \
  --width 1280 --height 720 \
  --depth --depth-width 640 --depth-height 480 \
  --fps 30 --no-resize
  
  # ssh hkclr@10.30.3.175, code:hkclr, cd /home/hkclr/wg/image_server, conda activate teleop
