#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/erazer/Seldon/dual_arm_cockpit
MODE=${1:-}
REFERENCE=${2:-strict}

if [[ "$MODE" != "--simulation" && "$MODE" != "--hardware" ]]; then
  echo "用法: $0 --simulation|--hardware [strict|accept-current]" >&2
  exit 2
fi
if [[ "$REFERENCE" != "strict" && "$REFERENCE" != "accept-current" ]]; then
  echo "启动参考必须是 strict 或 accept-current" >&2
  exit 2
fi
if pidof dual_v2_2_rt_control seldon_rt_control dual_v2_2_upper_software \
    >/dev/null 2>&1; then
  echo "已有机器人控制进程运行，拒绝启动第二个硬件驱动。" >&2
  exit 1
fi

BINARY="$ROOT/runtime/build_lingkong/seldon_rt_control"
[[ -x "$BINARY" ]] || {
  echo "缺少 $BINARY，请先运行 rt_lingkong/build_lingkong.py" >&2
  exit 1
}

exec "$BINARY" \
  --config-dir "$ROOT/runtime/config" \
  --encoder-file "$ROOT/runtime/encoder_last_position.bin" \
  "$MODE" \
  --startup-reference "$REFERENCE"
