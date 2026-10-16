#!/usr/bin/env bash
# Captures only local planner state, without opening a control socket.
set -euo pipefail
cd /mnt/ssd/user/V/dual_v2_2_agent
mkdir -p RobotRAM/output/hand_workspace_ar_live
exec 9>RobotRAM/output/hand_workspace_ar_live/state-tap.lock
flock -n 9 || { echo 'A hand state tap is already running.' >&2; exit 1; }
sudo /usr/bin/tcpdump --immediate-mode -i lo -n -s 0 -U -w - 'udp and src host 127.0.0.1 and dst host 127.0.0.1 and dst port 30241' |
  /mnt/ssd/robotarm/env/robotram_wx/bin/python -m hand_workspace_ar.state_relay
