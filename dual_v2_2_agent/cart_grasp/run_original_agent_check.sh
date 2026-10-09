#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export CUDA_VISIBLE_DEVICES=0
export CUARM_CONFIG_ROOT=/mnt/ssd/user/V/dual_v2_2_configuration
exec nice -n 10 /mnt/ssd/robotarm/env/curobo/bin/python \
  tools/cart_agent_check.py --output runtime/original_agent_check.json "$@"
