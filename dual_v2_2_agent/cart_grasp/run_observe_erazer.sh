#!/usr/bin/env bash
# Run on vlmdemo. Observation only: no arm server, RT, claim, or motion startup.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
control=/tmp/cart-erazer-control
if ! ssh -S "$control" -o BatchMode=yes -o ConnectTimeout=5 erazer@10.30.3.155 true; then
    ssh -M -S "$control" -o ControlPersist=3600 -o ConnectTimeout=5 -fN erazer@10.30.3.155
fi
ssh -S "$control" erazer@10.30.3.155 '
if ! systemctl --user is-active --quiet cart-head-observer; then
  systemd-run --user --unit=cart-head-observer --collect \
    -p CPUQuota=100% -p MemoryMax=1G -p RuntimeMaxSec=620 -p Nice=10 \
    --working-directory=/home/erazer/Seldon/cart_grasp \
    --setenv=PYTHONPATH=/home/erazer/Seldon/cart_grasp:/home/erazer/Seldon/keyboard_cockpit \
    --setenv=OPENBLAS_NUM_THREADS=1 --setenv=OMP_NUM_THREADS=1 \
    /home/erazer/Seldon/keyboard_cockpit/.venv/bin/python tools/head_camera_only.py --seconds 600
  sleep 3
fi'
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 CUDA_VISIBLE_DEVICES=0
exec nice -n 10 /mnt/ssd/robotarm/env/robotram_wx/bin/python tools/probe_remote_agent_camera.py "$@"
