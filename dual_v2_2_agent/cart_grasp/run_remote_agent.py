#!/usr/bin/env python3
"""Run the original V dual_v2_2 agent on vlmdemo with RGB-D from erazer.

The script is intended to run on 10.30.13.21.  It does not put a password in
the command line or a file: when the erazer SSH control socket is missing it
asks for the password on the controlling terminal.  The original Agent still
owns task selection and its existing safety/execute path.
"""
from __future__ import annotations

import argparse
import getpass
import os
from pathlib import Path
import pexpect
import shlex
import subprocess
import sys
import time


ERAZER = os.environ.get("SELDON_ERAZER_HOST", "erazer@10.30.3.155")
CONTROL = os.environ.get("ROBOTRAM_REMOTE_CAMERA_SSH_CONTROL", "/tmp/cart-erazer-control")
AGENT_ROOT = Path("/mnt/ssd/user/V/dual_v2_2_agent")
PLANNER = Path("/mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_planner/dual_v2_2_upper_planner.py")
PYTHON = "/mnt/ssd/robotarm/env/robotram_wx/bin/python"
CUROBO_PYTHON = "/mnt/ssd/robotarm/env/curobo/bin/python"
REMOTE_RGBD_PACKAGE = AGENT_ROOT / "cart_grasp"
REMOTE_RGBD_DIR = REMOTE_RGBD_PACKAGE / "runtime/erazer_rgbd"
REMOTE_CACHE = "/home/erazer/Seldon/cart_grasp/runtime/rgbd"


def ssh_base():
    return ["ssh", "-4", "-S", CONTROL, "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=15",
            "-o", "ServerAliveCountMax=3", ERAZER]


def control_alive():
    p = subprocess.run(["ssh", "-4", "-S", CONTROL, "-O", "check",
                        "-o", "ConnectTimeout=4", ERAZER],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return p.returncode == 0


def start_control():
    if control_alive():
        print(f"ERAZER_CONTROL_REUSED {CONTROL}", flush=True)
        return
    password = os.environ.get("ERAZER_SSH_PASSWORD")
    if password is None:
        if not sys.stdin.isatty():
            raise RuntimeError(
                "没有 erazer ControlMaster；请在终端运行以输入 erazer SSH 密码，"
                "或临时设置 ERAZER_SSH_PASSWORD 环境变量。"
            )
        password = getpass.getpass("erazer SSH password: ")
    command = ["ssh", "-4", "-M", "-S", CONTROL, "-fN",
               "-o", "ControlPersist=30m", "-o", "ServerAliveInterval=15",
               "-o", "ServerAliveCountMax=3",
               "-o", "StrictHostKeyChecking=accept-new",
               "-o", "PreferredAuthentications=password",
               "-o", "PubkeyAuthentication=no", ERAZER]
    child = pexpect.spawn(command[0], command[1:], encoding="utf-8", timeout=15)
    child.logfile_read = sys.stdout
    while True:
        idx = child.expect([
            r"(?i)are you sure you want to continue connecting.*\?",
            r"(?i)(?:password|密码)\s*:",
            pexpect.EOF,
            pexpect.TIMEOUT,
        ])
        if idx == 0:
            child.sendline("yes")
        elif idx == 1:
            child.sendline(password)
        elif idx == 2:
            break
        else:
            child.close(force=True)
            raise RuntimeError("建立 erazer SSH ControlMaster 超时")
    child.close()
    if not control_alive():
        raise RuntimeError("erazer SSH ControlMaster 建立失败")
    print(f"ERAZER_CONTROL_READY {CONTROL}", flush=True)


def remote(command: str, check=True):
    # ssh concatenates remote command arguments before invoking the remote
    # shell. Pass one shell-quoted command so nested Python quotes survive.
    argv = ssh_base() + [shlex.join(["bash", "-lc", command])]
    return subprocess.run(argv, check=check, text=True)


def ensure_camera(start_if_stale: bool):
    check = (
        "python3 -c "
        + shlex.quote(
            "import json,time; from pathlib import Path; "
            f"d=json.loads((Path({REMOTE_CACHE!r})/'status.json').read_text()); "
            "age=time.time()-float(d['updated_unix_s']); "
            "print('CAMERA_READY' if d.get('state')=='STREAMING' and "
            "d.get('serial')=='CV2TC5100060' and age<=3 else "
            "f'CAMERA_STALE age={age:.1f} state={d.get(\"state\")}'); "
            "raise SystemExit(0 if d.get('state')=='STREAMING' and "
            "d.get('serial')=='CV2TC5100060' and age<=3 else 2)"
        )
    )
    result = remote(check, check=False)
    if result.returncode == 0:
        return
    if not start_if_stale:
        raise RuntimeError(
            "155 上没有新鲜 RGB-D 快照；先运行本脚本的 --ensure-camera，"
            "或在 155 启动 cart_grasp/run_shared_cockpit.sh。"
        )
    print("CAMERA_HANDOVER_START", flush=True)
    command = f"""set -u
systemctl --user stop seldon-camera-web.service 2>/dev/null || true
systemctl --user stop seldon-shared-camera.service 2>/dev/null || true
systemd-run --user --no-block --unit=seldon-shared-camera --collect \\
  --property=Restart=always --property=RestartSec=2 \\
  /home/erazer/Seldon/cart_grasp/run_shared_cockpit.sh \\
  --host 0.0.0.0 --port 8765 --ros --camera orbbec --chassis-camera uvc
for i in $(seq 1 60); do
  if {check}; then exit 0; fi
  sleep 1
done
exit 1
"""
    remote(command, check=True)


def ensure_planner():
    probe = subprocess.run(
        ["bash", "-lc", "pgrep -f '[d]ual_v2_2_upper_planner.py' >/dev/null"],
        check=False,
    )
    if probe.returncode != 0:
        env = os.environ.copy()
        env.update({
            "CUARM_CONFIG_ROOT": "/tmp/dual_v2_2_configuration_remote",
            "CUDA_VISIBLE_DEVICES": "0",
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
        })
        log = Path.home() / ".cache/seldon-startup/planner.log"
        log.parent.mkdir(parents=True, exist_ok=True)
        command = [CUROBO_PYTHON, str(PLANNER)]
        with log.open("ab") as stream:
            subprocess.Popen(command, cwd=str(PLANNER.parent), env=env,
                             stdin=subprocess.DEVNULL, stdout=stream,
                             stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(2)
    if subprocess.run(["bash", "-lc", "pgrep -f '[d]ual_v2_2_upper_planner.py' >/dev/null"],
                      check=False).returncode != 0:
        raise RuntimeError("21 上 planner 未运行；查看 ~/.cache/seldon-startup/planner.log")
    for port in (10166, 30241):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            ready = subprocess.run(
                ["bash", "-lc", f"ss -lunH | awk '{{print $4}}' | grep -q ':{port}$'"],
                check=False,
            ).returncode == 0
            if ready:
                break
            time.sleep(.5)
        else:
            raise RuntimeError(f"21 上 planner UDP {port} 未监听")
    print("PLANNER_READY 10166/30241", flush=True)


def ensure_remote_panel():
    result = remote(
        "ss -lunH | awk '{print $4}' | grep -q ':30240$'",
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "155 的 Planner/Upper Remote UDP 30240 未监听；请先打开双臂页面并连接 Planner。"
        )
    probe = (
        "import json,urllib.request; "
        "w=json.load(open('/home/erazer/Seldon/dual_arm_cockpit/runtime/web.json')); "
        "token=w['url'].split('#token=',1)[1]; "
        "r=urllib.request.Request('http://127.0.0.1:8780/api/state',headers={'X-Arm-Token':token}); "
        "d=json.load(urllib.request.urlopen(r,timeout=3)); "
        "p=d.get('planner',{}); "
        "print(json.dumps({'fresh':d.get('fresh'),'rt_running':d.get('rt_running'),'connected':p.get('connected'),'enabled':p.get('enabled'),'received':p.get('received')})); "
        "raise SystemExit(0 if d.get('fresh') and d.get('rt_running') and p.get('connected') and p.get('enabled') else 2)"
    )
    state = remote("python3 -c " + shlex.quote(probe), check=False)
    if state.returncode != 0:
        raise RuntimeError(
            "155 Planner 尚未允许控制；请在双臂页面点击‘允许 Planner 控制’，"
            "确认 RT 反馈新鲜后再重新运行 Agent。"
        )
    print("REMOTE_PANEL_READY_AND_ENABLED 155:30240", flush=True)


def run_agent(extra):
    env = os.environ.copy()
    env.update({
        "CUARM_CONFIG_ROOT": "/mnt/ssd/user/V/dual_v2_2_configuration",
        "CUDA_VISIBLE_DEVICES": "0",
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "MKL_NUM_THREADS": "1",
        "PYTHONUNBUFFERED": "1",
    })
    command = [PYTHON, str(AGENT_ROOT / "dual_v2_2_agent_pcl.py"),
               "--no-rerun", "--no-rerun-viewer", "--assume-head-camera-calibrated",
               "--remote-rgbd", "--remote-rgbd-package", str(REMOTE_RGBD_PACKAGE),
               "--remote-rgbd-dir", str(REMOTE_RGBD_DIR),
               "--remote-camera-ssh-control", CONTROL]
    command.extend(extra)
    print("AGENT_START", " ".join(shlex.quote(x) for x in command), flush=True)
    return subprocess.call(command, cwd=str(AGENT_ROOT), env=env)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ensure-camera", action="store_true",
                        help="155 快照过期时切换到共享 RGB-D 采集入口")
    parser.add_argument("--check-only", action="store_true",
                        help="只检查 SSH、155 RGB-D、21 planner，不启动 Agent")
    parser.add_argument("agent_args", nargs=argparse.REMAINDER,
                        help="传给原 dual_v2_2_agent_pcl.py 的参数")
    args = parser.parse_args()
    extra = args.agent_args
    if extra and extra[0] == "--":
        extra = extra[1:]
    start_control()
    ensure_camera(args.ensure_camera)
    ensure_planner()
    ensure_remote_panel()
    if args.check_only:
        print("REMOTE_AGENT_PREFLIGHT_OK", flush=True)
        return 0
    return run_agent(extra)


if __name__ == "__main__":
    raise SystemExit(main())
