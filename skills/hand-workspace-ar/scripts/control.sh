#!/usr/bin/env bash
set -euo pipefail
cd /mnt/ssd/user/V/dual_v2_2_agent
if [ "$#" -eq 0 ]; then set -- status; fi
exec /mnt/ssd/robotarm/env/robotram_wx/bin/python -m hand_workspace_ar "$@"
