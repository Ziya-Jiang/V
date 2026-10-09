#!/usr/bin/env bash
set -Eeuo pipefail
# This wrapper is copied to vlmdemo's V/dual_v2_2_agent/cart_grasp directory.
exec /mnt/ssd/robotarm/env/robotram_wx/bin/python \
  /mnt/ssd/user/V/dual_v2_2_agent/cart_grasp/run_remote_agent.py "$@"
