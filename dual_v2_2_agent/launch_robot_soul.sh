#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEEPSEEK_ROOT="${ROOT_DIR}/deepseek-harness"
ASSISTANT_LAUNCHER="${ROOT_DIR}/launch_robot_assistant.sh"
CONTROL_ROOT="${ROBOT_ASSISTANT_CONTROL_ROOT:-/mnt/ssd/user/V/dual_v2_2_control}"
SOUL_ENTRY="${DEEPSEEK_ROOT}/apps/robot-soul-dsh/src/index.ts"
HARNESS_ENTRY="${DEEPSEEK_ROOT}/apps/cli/src/bin.ts"
HARNESS_PATCH="${DEEPSEEK_ROOT}/examples/robot-approved/cordis.yml"
WEB_DIST="${DEEPSEEK_ROOT}/apps/web/dist/index.html"
VITE_BIN="${DEEPSEEK_ROOT}/apps/web/node_modules/.bin/vite"
PYTHON_BIN="${ROBOT_AGENT_PYTHON:-/mnt/ssd/robotarm/env/robotram_wx/bin/python}"

RUNTIME_DIR="${ROBOT_SOUL_RUNTIME_DIR:-${ROOT_DIR}/.robot_soul_runtime}"
ASSISTANT_RUNTIME_DIR="${ROBOT_ASSISTANT_RUNTIME_DIR:-${ROOT_DIR}/.robot_assistant_runtime}"
PID_DIR="${RUNTIME_DIR}/pids"
LOG_DIR="${RUNTIME_DIR}/logs"
STATUS_DIR="${RUNTIME_DIR}/status"
TOKEN_FILE="${RUNTIME_DIR}/agent.token"
RUNTIME_HARNESS_PATCH="${RUNTIME_DIR}/robot-approved.cordis.yml"
STACK_STATUS_FILE="${STATUS_DIR}/stack.json"
RECOVERY_LOCK_DIR="${STATUS_DIR}/recovery.lock"
SOUL_PID_FILE="${PID_DIR}/soul_gui.pid"
HARNESS_PID_FILE="${PID_DIR}/harness_web.pid"
SUPERVISOR_PID_FILE="${PID_DIR}/supervisor.pid"
SOUL_LOG="${LOG_DIR}/soul_gui.log"
HARNESS_LOG="${LOG_DIR}/harness_web.log"
SUPERVISOR_LOG="${LOG_DIR}/supervisor.log"
RECOVERY_LOG="${LOG_DIR}/hardware_recovery.log"
AGENT_BOOTSTRAP_STATUS_FILE="${ASSISTANT_RUNTIME_DIR}/status/.robot_agent_status.json"

AGENT_HOST="127.0.0.1"
AGENT_PORT="${ROBOT_AGENT_PORT:-15555}"
SOUL_HOST="127.0.0.1"
SOUL_PORT="${ROBOT_SOUL_PORT:-3091}"
HARNESS_HOST="127.0.0.1"
HARNESS_PORT="${ROBOT_SOUL_HARNESS_PORT:-3081}"
OUTPUT_DIR="${ROBOT_AGENT_OUTPUT_DIR:-${ROOT_DIR}/RobotRAM/output/scene_57}"
HARDWARE_CONFIRMATION="ENABLE_DUAL_V2_2_HARDWARE"
# Keep the reviewed startup recovery flags enabled by default. Operators may
# still override either value explicitly for diagnostics.
ALLOW_POSITION_MISMATCH="${CUARM_ALLOW_STARTUP_POSITION_MISMATCH:-1}"
RECOVER_STARTUP_FAULT="${CUARM_RECOVER_STARTUP_FAULT:-1}"
STARTUP_CLEANUP_REQUIRED=0

mkdir -p "${PID_DIR}" "${LOG_DIR}" "${STATUS_DIR}"

log() {
    printf '[robot-soul] %s\n' "$*"
}

die() {
    log "错误: $*" >&2
    return 1
}

assert_command() {
    command -v "$1" >/dev/null 2>&1 || die "缺少命令: $1"
}

assert_file() {
    [[ -f "$1" ]] || die "文件不存在: $1"
}

assert_executable() {
    [[ -x "$1" ]] || die "不可执行: $1"
}

pid_from_file() {
    local file="$1"
    [[ -s "${file}" ]] || return 1
    tr -dc '0-9' < "${file}"
}

pid_is_running() {
    local pid="$1"
    local state=""
    [[ -n "${pid}" ]] || return 1
    state="$(ps -o stat= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ -n "${state}" && "${state}" != Z* ]]
}

process_group_is_running() {
    local pid="$1"
    [[ -n "${pid}" ]] || return 1
    kill -0 -- "-${pid}" 2>/dev/null
}

component_is_running() {
    local pid="$1"
    pid_is_running "${pid}" || process_group_is_running "${pid}"
}

remove_stale_pid_file() {
    local file="$1"
    local pid=""
    pid="$(pid_from_file "${file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && ! component_is_running "${pid}"; then
        rm -f "${file}"
    fi
}

port_is_free() {
    local port="$1"
    ! ss -H -ltn "( sport = :${port} )" | rg -q .
}

assert_port_free() {
    local port="$1"
    port_is_free "${port}" || die "TCP 端口 ${port} 已被占用"
}

ensure_token() {
    local token="${ROBOT_AGENT_TOKEN:-}"
    if [[ -z "${token}" && -s "${TOKEN_FILE}" ]]; then
        token="$(tr -d '\r\n' < "${TOKEN_FILE}")"
    fi
    if [[ -z "${token}" ]]; then
        token="$(
            /mnt/ssd/robotarm/env/robotram_wx/bin/python -c \
                'import secrets; print(secrets.token_urlsafe(32), end="")'
        )"
    fi
    [[ ${#token} -ge 24 ]] || die "ROBOT_AGENT_TOKEN 至少需要 24 个字符"
    umask 077
    printf '%s' "${token}" > "${TOKEN_FILE}"
    chmod 600 "${TOKEN_FILE}"
    printf '%s' "${token}"
}

write_stack_status() {
    local state="$1"
    local reason="$2"
    local detail="$3"
    local recoverable="$4"
    local progress="${5:-}"
    local operation="${6:-}"
    "${PYTHON_BIN}" - \
        "${STACK_STATUS_FILE}" "${state}" "${reason}" "${detail}" \
        "${recoverable}" "${progress}" "${operation}" <<'PY'
from datetime import datetime, timezone
from pathlib import Path
import json
import os
import sys

path = Path(sys.argv[1])
payload = {
    "state": sys.argv[2],
    "reason": sys.argv[3] or None,
    "detail": sys.argv[4] or None,
    "recoverable": sys.argv[5] == "1",
    "updatedAt": datetime.now(timezone.utc).isoformat(),
}
if sys.argv[6]:
    payload["progress"] = max(0, min(100, int(sys.argv[6])))
if sys.argv[7]:
    payload["operation"] = sys.argv[7]
temporary = path.with_suffix(".tmp")
temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
os.chmod(temporary, 0o600)
temporary.replace(path)
PY
}

stack_state() {
    [[ -f "${STACK_STATUS_FILE}" ]] || {
        printf 'healthy'
        return
    }
    "${PYTHON_BIN}" - "${STACK_STATUS_FILE}" <<'PY'
import json
import sys

try:
    value = json.load(open(sys.argv[1], encoding="utf-8")).get("state", "faulted")
except Exception:
    value = "faulted"
print(value, end="")
PY
}

stack_reason() {
    [[ -f "${STACK_STATUS_FILE}" ]] || {
        printf ''
        return 0
    }
    "${PYTHON_BIN}" - "${STACK_STATUS_FILE}" <<'PY'
import json
import sys

try:
    value = json.load(open(sys.argv[1], encoding="utf-8")).get("reason") or ""
except Exception:
    value = ""
print(value, end="")
PY
}

latest_assistant_fault_detail() {
    local assistant_log="${ROOT_DIR}/.robot_assistant_runtime/logs/supervisor.log"
    local control_log="${CONTROL_ROOT}/.dual_v2_2_runtime/logs/rt_control.log"
    local detail=""
    if [[ -f "${control_log}" ]]; then
        detail="$(rg '\[FAULT\]|(^|[^[:alpha:]])ERROR([^[:alpha:]]|$)' \
            "${control_log}" 2>/dev/null | tail -n 1 || true)"
    fi
    if [[ -z "${detail}" && -f "${assistant_log}" ]]; then
        detail="$(tail -n 1 "${assistant_log}" 2>/dev/null || true)"
    fi
    printf '%s' "${detail:-硬件控制栈或 Python Agent 健康检查失败。}"
}

latest_startup_failure_detail() {
    local detail=""
    if [[ -f "${AGENT_BOOTSTRAP_STATUS_FILE}" ]]; then
        detail="$("${PYTHON_BIN}" - "${AGENT_BOOTSTRAP_STATUS_FILE}" <<'PY'
import json
import sys

try:
    payload = json.load(open(sys.argv[1], encoding="utf-8"))
    print(payload.get("error") or "", end="")
except Exception:
    pass
PY
)"
    fi
    if [[ -z "${detail}" && -f "${RECOVERY_LOG}" ]]; then
        detail="$(rg 'Python Agent 启动失败原因:|错误:|sudo:.*(password|not allowed|authentication)' \
            "${RECOVERY_LOG}" 2>/dev/null | tail -n 1 || true)"
    fi
    if [[ -z "${detail}" ]]; then
        detail="$(latest_assistant_fault_detail)"
    fi
    printf '%s' "${detail:-硬件启动失败；未提供具体错误。}"
}

confirm_hardware_interactively() {
    if [[ "${ROBOT_ASSISTANT_HARDWARE_CONFIRM:-}" == "${HARDWARE_CONFIRMATION}" ]]; then
        return
    fi
    [[ -t 0 ]] ||
        die "真实模式需要 export ROBOT_ASSISTANT_HARDWARE_CONFIRM=${HARDWARE_CONFIRMATION}"
    printf '即将初始化真实机器人。请确认工作区清空且物理急停可用。\n'
    printf '输入 %s 继续: ' "${HARDWARE_CONFIRMATION}"
    local confirmation=""
    read -r confirmation
    [[ "${confirmation}" == "${HARDWARE_CONFIRMATION}" ]] ||
        die "硬件确认口令不匹配；未启动任何进程"
    export ROBOT_ASSISTANT_HARDWARE_CONFIRM="${confirmation}"
}

signal_process_group() {
    local pid="$1"
    local signal_name="$2"
    kill "-${signal_name}" -- "-${pid}" 2>/dev/null ||
        kill "-${signal_name}" "${pid}" 2>/dev/null ||
        true
}

stop_component() {
    local name="$1"
    local pid_file="$2"
    local pid=""
    pid="$(pid_from_file "${pid_file}" 2>/dev/null || true)"
    if [[ -z "${pid}" ]] || ! component_is_running "${pid}"; then
        rm -f "${pid_file}"
        log "${name} 已停止。"
        return
    fi
    log "停止 ${name} (PID ${pid})..."
    signal_process_group "${pid}" TERM
    local deadline=$((SECONDS + 15))
    while (( SECONDS < deadline )); do
        if ! component_is_running "${pid}"; then
            rm -f "${pid_file}"
            return
        fi
        sleep 0.2
    done
    signal_process_group "${pid}" KILL
    deadline=$((SECONDS + 5))
    while (( SECONDS < deadline )); do
        if ! component_is_running "${pid}"; then
            rm -f "${pid_file}"
            return
        fi
        sleep 0.2
    done
    die "无法停止 ${name} 进程组 ${pid}"
}

status_component() {
    local name="$1"
    local pid_file="$2"
    local pid=""
    pid="$(pid_from_file "${pid_file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && component_is_running "${pid}"; then
        printf '%-20s RUNNING pid=%s\n' "${name}" "${pid}"
        return 0
    fi
    printf '%-20s STOPPED\n' "${name}"
    return 1
}

preflight() {
    local launch_mode="${1:-hardware}"
    [[ "${launch_mode}" == "hardware" || "${launch_mode}" == "console" ]] ||
        die "未知启动模式: ${launch_mode}"
    assert_command node
    assert_command curl
    assert_command rg
    assert_command ss
    assert_command setsid
    assert_command ps
    assert_executable "${ASSISTANT_LAUNCHER}"
    assert_executable "${VITE_BIN}"
    assert_executable "${PYTHON_BIN}"
    assert_file "${SOUL_ENTRY}"
    assert_file "${HARNESS_ENTRY}"
    assert_file "${HARNESS_PATCH}"
    assert_file "${OUTPUT_DIR}/vlm_categories.json"
    [[ "${ALLOW_POSITION_MISMATCH}" == "0" ||
        "${ALLOW_POSITION_MISMATCH}" == "1" ]] ||
        die "CUARM_ALLOW_STARTUP_POSITION_MISMATCH 只能是 0 或 1"
    [[ "${RECOVER_STARTUP_FAULT}" == "0" ||
        "${RECOVER_STARTUP_FAULT}" == "1" ]] ||
        die "CUARM_RECOVER_STARTUP_FAULT 只能是 0 或 1"
    if [[ "${launch_mode}" == "hardware" ]]; then
        [[ "${ROBOT_ASSISTANT_HARDWARE_CONFIRM:-}" == "${HARDWARE_CONFIRMATION}" ]] ||
            die "真实模式需要 export ROBOT_ASSISTANT_HARDWARE_CONFIRM=${HARDWARE_CONFIRMATION}"
    fi

    remove_stale_pid_file "${SOUL_PID_FILE}"
    remove_stale_pid_file "${HARNESS_PID_FILE}"
    remove_stale_pid_file "${SUPERVISOR_PID_FILE}"
    [[ ! -s "${SOUL_PID_FILE}" ]] || die "Robot Soul GUI 已由本启动器运行"
    [[ ! -s "${HARNESS_PID_FILE}" ]] || die "Harness Web 已由本启动器运行"
    [[ ! -s "${SUPERVISOR_PID_FILE}" ]] || die "Robot Soul Supervisor 已运行"

    if ROBOT_ASSISTANT_HEADLESS=1 \
        "${ASSISTANT_LAUNCHER}" status >/dev/null 2>&1; then
        die "旧 Robot Assistant 栈正在运行；请先执行 ./launch_robot_assistant.sh stop"
    fi
    assert_port_free "${SOUL_PORT}"
    assert_port_free "${HARNESS_PORT}"

    ROBOT_AGENT_TOKEN="${ROBOT_AGENT_TOKEN:-robot-soul-preflight-token-0001}" \
        ROBOT_AGENT_OUTPUT_DIR="${OUTPUT_DIR}" \
        ROBOT_ASSISTANT_HARDWARE_CONFIRM="${HARDWARE_CONFIRMATION}" \
        CUARM_ALLOW_STARTUP_POSITION_MISMATCH="${ALLOW_POSITION_MISMATCH}" \
        CUARM_RECOVER_STARTUP_FAULT="${RECOVER_STARTUP_FAULT}" \
        ROBOT_ASSISTANT_HEADLESS=1 \
        "${ASSISTANT_LAUNCHER}" check
}

build_harness_web() {
    if [[ -f "${WEB_DIST}" ]]; then
        return
    fi
    log "首次构建 DeepSeek Harness Web..."
    (
        cd "${DEEPSEEK_ROOT}/apps/web"
        "${VITE_BIN}" build
    )
    assert_file "${WEB_DIST}"
}

prepare_harness_patch() {
    "${PYTHON_BIN}" - \
        "${HARNESS_PATCH}" "${RUNTIME_HARNESS_PATCH}" "${DEEPSEEK_ROOT}" <<'PY'
from pathlib import Path
import os
import sys

source_path = Path(sys.argv[1])
target_path = Path(sys.argv[2])
root = Path(sys.argv[3]).resolve()
text = source_path.read_text(encoding="utf-8")
replacements = {
    "'../../packages/robot/robot-python-agent/src/index.ts'":
        repr(str(root / "packages/robot/robot-python-agent/src/index.ts")),
    "'../../packages/robot/tool-robot/src/index.ts'":
        repr(str(root / "packages/robot/tool-robot/src/index.ts")),
}
for source, target in replacements.items():
    if text.count(source) != 1:
        raise SystemExit(f"unexpected robot overlay marker: {source}")
    text = text.replace(source, target)
temporary = target_path.with_suffix(".tmp")
temporary.write_text(text, encoding="utf-8")
os.chmod(temporary, 0o600)
temporary.replace(target_path)
PY
}

wait_for_http() {
    local url="$1"
    local pid="$2"
    local timeout_s="$3"
    local deadline=$((SECONDS + timeout_s))
    while (( SECONDS < deadline )); do
        pid_is_running "${pid}" || return 1
        if curl --fail --silent --max-time 2 "${url}" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

start_assistant_stack() {
    local token="$1"
    log "启动硬件控制栈与唯一 Python Agent（位置不匹配=${ALLOW_POSITION_MISMATCH}，启动故障恢复=${RECOVER_STARTUP_FAULT}）..."
    ROBOT_AGENT_TOKEN="${token}" \
        ROBOT_AGENT_OUTPUT_DIR="${OUTPUT_DIR}" \
        ROBOT_ASSISTANT_HARDWARE_CONFIRM="${HARDWARE_CONFIRMATION}" \
        CUARM_ALLOW_STARTUP_POSITION_MISMATCH="${ALLOW_POSITION_MISMATCH}" \
        CUARM_RECOVER_STARTUP_FAULT="${RECOVER_STARTUP_FAULT}" \
        ROBOT_ASSISTANT_HEADLESS=1 \
        "${ASSISTANT_LAUNCHER}" start
}

start_harness() {
    local token="$1"
    : > "${HARNESS_LOG}"
    log "启动 DeepSeek Harness AI..."
    (
        cd "${DEEPSEEK_ROOT}"
        exec setsid env \
            ROBOT_AGENT_HOST="${AGENT_HOST}" \
            ROBOT_AGENT_PORT="${AGENT_PORT}" \
            ROBOT_AGENT_TOKEN="${token}" \
            ROBOT_AGENT_OUTPUT_DIR="${OUTPUT_DIR}" \
            node --import tsx/esm "${HARNESS_ENTRY}" web \
                --patch "${RUNTIME_HARNESS_PATCH}" \
                --host "${HARNESS_HOST}" \
                --port "${HARNESS_PORT}"
    ) >> "${HARNESS_LOG}" 2>&1 < /dev/null &
    local pid=$!
    printf '%s\n' "${pid}" > "${HARNESS_PID_FILE}"
    if ! wait_for_http "http://${HARNESS_HOST}:${HARNESS_PORT}/" "${pid}" 90; then
        log "Harness Web 启动失败，最近日志:"
        tail -n 80 "${HARNESS_LOG}" || true
        return 1
    fi
    log "Harness AI 已就绪。"
}

start_soul() {
    local token="$1"
    : > "${SOUL_LOG}"
    log "启动统一 Robot Soul GUI..."
    (
        cd "${DEEPSEEK_ROOT}"
        exec setsid env \
            ROBOT_SOUL_HARDWARE_READONLY=1 \
            ROBOT_SOUL_APPROVED_EXECUTION="${HARDWARE_CONFIRMATION}" \
            ROBOT_SOUL_AGENT_HOST="${AGENT_HOST}" \
            ROBOT_SOUL_AGENT_PORT="${AGENT_PORT}" \
            ROBOT_SOUL_AGENT_TOKEN="${token}" \
            ROBOT_SOUL_OUTPUT_DIR="${OUTPUT_DIR}" \
            ROBOT_SOUL_HOST="${SOUL_HOST}" \
            ROBOT_SOUL_PORT="${SOUL_PORT}" \
            ROBOT_SOUL_HARNESS_HOST="${HARNESS_HOST}" \
            ROBOT_SOUL_HARNESS_PORT="${HARNESS_PORT}" \
            ROBOT_SOUL_STACK_STATUS_FILE="${STACK_STATUS_FILE}" \
            ROBOT_SOUL_AGENT_STATUS_FILE="${AGENT_BOOTSTRAP_STATUS_FILE}" \
            ROBOT_SOUL_STARTUP_LAUNCHER="${ROOT_DIR}/launch_robot_soul.sh" \
            ROBOT_SOUL_RECOVERY_LAUNCHER="${ROOT_DIR}/launch_robot_soul.sh" \
            node --import tsx/esm "${SOUL_ENTRY}"
    ) >> "${SOUL_LOG}" 2>&1 < /dev/null &
    local pid=$!
    printf '%s\n' "${pid}" > "${SOUL_PID_FILE}"
    if ! wait_for_http \
        "http://${SOUL_HOST}:${SOUL_PORT}/api/ops/status" "${pid}" 30; then
        log "Robot Soul GUI 启动失败，最近日志:"
        tail -n 80 "${SOUL_LOG}" || true
        return 1
    fi
    log "统一 GUI 已就绪。"
}

supervisor_exit() {
    rm -f "${SUPERVISOR_PID_FILE}"
    exit 0
}

supervise_stack() {
    local tier1_failures=0
    local tier2_failures=0
    local tier2_latched=0
    local current_stack_state=""
    trap supervisor_exit INT TERM
    log "统一安全监督器已启动。"
    while true; do
        if curl --fail --silent --max-time 2 \
                "http://${HARNESS_HOST}:${HARNESS_PORT}/" >/dev/null 2>&1 &&
            curl --fail --silent --max-time 2 \
                "http://${SOUL_HOST}:${SOUL_PORT}/" >/dev/null 2>&1; then
            tier1_failures=0
        else
            ((tier1_failures++)) || true
            if (( tier1_failures >= 3 )); then
                log "Harness 或统一网页连续三次健康检查失败，执行完整安全停机。"
                break
            fi
        fi

        current_stack_state="$(stack_state)"
        if [[ "${current_stack_state}" == "offline" ]]; then
            tier2_failures=0
            tier2_latched=0
        elif [[ "${current_stack_state}" == "starting" ||
            "${current_stack_state}" == "recovering" ||
            "${current_stack_state}" == "stopping" ]]; then
            tier2_failures=0
        elif [[ "${current_stack_state}" == "faulted" ]]; then
            tier2_latched=1
        elif (( tier2_latched == 0 )); then
            if ROBOT_ASSISTANT_HEADLESS=1 \
                "${ASSISTANT_LAUNCHER}" status >/dev/null 2>&1; then
                tier2_failures=0
            else
                ((tier2_failures++)) || true
                if (( tier2_failures >= 3 )); then
                    local detail=""
                    detail="$(latest_assistant_fault_detail)"
                    log "硬件面连续三次健康检查失败；锁定并停止硬件面，保留 Harness 与网页。"
                    write_stack_status \
                        "faulted" "hardware_stack_unhealthy" "${detail}" "1"
                    ROBOT_ASSISTANT_HEADLESS=1 \
                        "${ASSISTANT_LAUNCHER}" stop || true
                    tier2_latched=1
                fi
            fi
        elif [[ "${current_stack_state}" == "healthy" ]]; then
            if ROBOT_ASSISTANT_HEADLESS=1 \
                "${ASSISTANT_LAUNCHER}" status >/dev/null 2>&1; then
                log "人工恢复完成；硬件面重新进入健康状态。"
                tier2_latched=0
                tier2_failures=0
            else
                write_stack_status \
                    "faulted" "recovery_lost" \
                    "硬件面恢复后再次离线；请检查硬件连接与恢复日志。" "1"
            fi
        fi
        sleep 1
    done
    stop_component "DeepSeek Harness Web" "${HARNESS_PID_FILE}"
    stop_component "Robot Soul GUI" "${SOUL_PID_FILE}"
    "${ASSISTANT_LAUNCHER}" stop || true
    rm -f "${SUPERVISOR_PID_FILE}"
    return 1
}

start_assistant_from_console() {
    exec >> "${RECOVERY_LOG}" 2>&1
    log "收到现场操作员的一键启动请求。"
    [[ "$(stack_state)" != "faulted" ]] ||
        die "硬件面处于 faulted 状态；请使用恢复操作"
    if ROBOT_ASSISTANT_HEADLESS=1 \
        "${ASSISTANT_LAUNCHER}" status >/dev/null 2>&1; then
        rm -f "${STACK_STATUS_FILE}"
        log "硬件面已经处于健康状态，无需重复启动。"
        return 0
    fi
    if ! mkdir "${RECOVERY_LOCK_DIR}" 2>/dev/null; then
        die "已有硬件生命周期操作正在进行"
    fi
    trap 'status=$?; rmdir "${RECOVERY_LOCK_DIR}" 2>/dev/null || true; if (( status != 0 )) && [[ "$(stack_state)" == "starting" ]]; then write_stack_status "faulted" "startup_interrupted" "硬件启动进程被中断；请检查启动日志后使用恢复操作。" "1"; fi' EXIT

    rm -f "${AGENT_BOOTSTRAP_STATUS_FILE}"
    write_stack_status \
        "starting" "operator_startup" "正在验证硬件启动条件。" "0" \
        "5" "验证硬件状态与生命周期锁"

    write_stack_status \
        "starting" "operator_startup" "正在准备安全凭证与运行目录。" "0" \
        "10" "准备安全凭证"
    local token=""
    token="$(ensure_token)"
    write_stack_status \
        "starting" "operator_startup" "正在启动机器人控制栈。" "0" \
        "15" "启动机器人控制栈"
    if start_assistant_stack "${token}"; then
        rm -f "${STACK_STATUS_FILE}"
        log "硬件控制栈与 Python Agent 一键启动成功。"
        return 0
    fi

    local failure_detail=""
    failure_detail="$(latest_startup_failure_detail)"
    write_stack_status \
        "faulted" "startup_failed" "${failure_detail}" "1"
    die "硬件面一键启动失败"
}

stop_assistant_from_console() {
    exec >> "${RECOVERY_LOG}" 2>&1
    log "收到现场操作员的一键停止请求。"
    if ! mkdir "${RECOVERY_LOCK_DIR}" 2>/dev/null; then
        die "已有硬件生命周期操作正在进行"
    fi
    trap 'status=$?; rmdir "${RECOVERY_LOCK_DIR}" 2>/dev/null || true; if (( status != 0 )) && [[ "$(stack_state)" == "stopping" ]]; then write_stack_status "faulted" "shutdown_interrupted" "硬件停止进程被中断；请检查停止日志和硬件状态。" "1"; fi' EXIT

    write_stack_status \
        "stopping" "operator_shutdown" "正在停止 Python Agent 与硬件控制栈；网页和 Harness 保持在线。" "0"
    if ROBOT_ASSISTANT_HEADLESS=1 "${ASSISTANT_LAUNCHER}" stop; then
        write_stack_status \
            "offline" "operator_startup_available" \
            "The local operator console is ready; the hardware stack is offline." "0"
        log "硬件控制栈与 Python Agent 已停止；网页与 Harness 保持在线。"
        return 0
    fi

    write_stack_status \
        "faulted" "shutdown_failed" \
        "硬件面未能完整停止；请检查停止日志与物理机器人状态。" "1"
    die "硬件面一键停止失败"
}

recover_assistant_stack() {
    exec >> "${RECOVERY_LOG}" 2>&1
    log "收到现场操作员的硬件面恢复请求。"
    [[ "$(stack_state)" == "faulted" ]] ||
        die "硬件面当前不是可恢复的 faulted 状态"
    if ! mkdir "${RECOVERY_LOCK_DIR}" 2>/dev/null; then
        die "已有硬件恢复正在进行"
    fi
    trap 'status=$?; rmdir "${RECOVERY_LOCK_DIR}" 2>/dev/null || true; if (( status != 0 )) && [[ "$(stack_state)" == "recovering" ]]; then write_stack_status "faulted" "recovery_interrupted" "硬件恢复进程被中断；请检查恢复日志后重试。" "1"; fi' EXIT

    write_stack_status \
        "recovering" "operator_recovery" "正在清理旧硬件进程。" "0" \
        "5" "清理旧硬件进程"
    ROBOT_ASSISTANT_HEADLESS=1 "${ASSISTANT_LAUNCHER}" stop || true
    rm -f "${AGENT_BOOTSTRAP_STATUS_FILE}"
    write_stack_status \
        "recovering" "operator_recovery" "正在重新启动机器人控制栈。" "0" \
        "15" "重新启动机器人控制栈"
    local token=""
    token="$(ensure_token)"
    if ROBOT_AGENT_TOKEN="${token}" \
        ROBOT_AGENT_OUTPUT_DIR="${OUTPUT_DIR}" \
        ROBOT_ASSISTANT_HARDWARE_CONFIRM="${HARDWARE_CONFIRMATION}" \
        CUARM_ALLOW_STARTUP_POSITION_MISMATCH="${ALLOW_POSITION_MISMATCH}" \
        CUARM_RECOVER_STARTUP_FAULT="${RECOVER_STARTUP_FAULT}" \
        ROBOT_ASSISTANT_HEADLESS=1 \
        "${ASSISTANT_LAUNCHER}" start; then
        rm -f "${STACK_STATUS_FILE}"
        log "硬件面人工恢复成功；Harness 与网页未重启。"
        return 0
    fi

    local failure_detail=""
    failure_detail="$(latest_startup_failure_detail)"
    write_stack_status \
        "faulted" "recovery_failed" "${failure_detail}" "1"
    die "硬件面重新初始化失败"
}

start_supervisor() {
    : > "${SUPERVISOR_LOG}"
    (
        cd "${ROOT_DIR}"
        exec setsid "${BASH_SOURCE[0]}" __supervise
    ) >> "${SUPERVISOR_LOG}" 2>&1 < /dev/null &
    local pid=$!
    printf '%s\n' "${pid}" > "${SUPERVISOR_PID_FILE}"
    sleep 0.2
    pid_is_running "${pid}" || return 1
}

stop_all() {
    local failed=0
    stop_component "Robot Soul Supervisor" "${SUPERVISOR_PID_FILE}" ||
        failed=1
    stop_component "DeepSeek Harness Web" "${HARNESS_PID_FILE}" ||
        failed=1
    stop_component "Robot Soul GUI" "${SOUL_PID_FILE}" ||
        failed=1
    "${ASSISTANT_LAUNCHER}" stop || failed=1
    return "${failed}"
}

reload_ui() {
    local token=""
    token="$(ensure_token)"
    stop_component "Robot Soul Supervisor" "${SUPERVISOR_PID_FILE}"
    stop_component "Robot Soul GUI" "${SOUL_PID_FILE}"
    if ! start_soul "${token}"; then
        log "网页重载失败；为避免失去外层监督，停止硬件面。"
        ROBOT_ASSISTANT_HEADLESS=1 "${ASSISTANT_LAUNCHER}" stop || true
        return 1
    fi
    start_supervisor
    log "统一网页已重载；Harness 与硬件面未重启。"
    status_all || true
}

startup_exit_cleanup() {
    local status=$?
    if (( STARTUP_CLEANUP_REQUIRED == 1 )); then
        set +e
        log "启动被中断或失败，回收已启动的全部组件..."
        stop_all
    fi
    return "${status}"
}

start_all() {
    confirm_hardware_interactively
    preflight
    build_harness_web
    prepare_harness_patch
    local token=""
    token="$(ensure_token)"
    STARTUP_CLEANUP_REQUIRED=1
    trap startup_exit_cleanup EXIT
    trap 'exit 130' INT TERM
    start_assistant_stack "${token}"
    start_harness "${token}"
    start_soul "${token}"
    rm -f "${STACK_STATUS_FILE}"
    start_supervisor
    STARTUP_CLEANUP_REQUIRED=0
    trap - EXIT INT TERM
    log "Robot Soul 已全部就绪。"
    status_all || true
    printf '\n只需打开一个页面:\n  http://%s:%s/#control\n' \
        "${SOUL_HOST}" "${SOUL_PORT}"
}

start_console() {
    preflight console
    build_harness_web
    prepare_harness_patch
    local token=""
    token="$(ensure_token)"
    STARTUP_CLEANUP_REQUIRED=1
    trap startup_exit_cleanup EXIT
    trap 'exit 130' INT TERM
    local current_stack_state=""
    local current_stack_reason=""
    current_stack_state="$(stack_state)"
    current_stack_reason="$(stack_reason)"
    if [[ "${current_stack_state}" != "faulted" ||
        "${current_stack_reason}" == "sudo_authorization_required" ]]; then
        write_stack_status \
            "offline" "operator_startup_available" \
            "The local operator console is ready; the hardware stack is offline." "0"
    fi
    start_harness "${token}"
    start_soul "${token}"
    start_supervisor
    STARTUP_CLEANUP_REQUIRED=0
    trap - EXIT INT TERM
    log "本地操作台已就绪；真实机器人尚未启动。"
    status_all || true
    printf '\n打开本地页面并选择 Real robot:\n  http://%s:%s/#playground\n' \
        "${SOUL_HOST}" "${SOUL_PORT}"
}

status_all() {
    local ok=0
    local hardware_state=""
    status_component "robot_soul_supervisor" "${SUPERVISOR_PID_FILE}" || ok=1
    status_component "robot_soul_gui" "${SOUL_PID_FILE}" || ok=1
    status_component "deepseek_harness" "${HARNESS_PID_FILE}" || ok=1
    hardware_state="$(stack_state)"
    if [[ "${hardware_state}" == "offline" ]]; then
        printf '%-20s OFFLINE (可从本地网页一键启动)\n' "hardware_plane"
    else
        ROBOT_ASSISTANT_HEADLESS=1 "${ASSISTANT_LAUNCHER}" status || ok=1
    fi
    printf 'unified gui: http://%s:%s/#control\n' "${SOUL_HOST}" "${SOUL_PORT}"
    printf 'soul log:   %s\n' "${SOUL_LOG}"
    printf 'harness log:%s\n' "${HARNESS_LOG}"
    printf 'supervisor: %s\n' "${SUPERVISOR_LOG}"
    return "${ok}"
}

usage() {
    cat <<EOF
用法: $0 [check|start|console|start-all|stop|restart|restart-all|reload-ui|status]

start（默认）与 console 只启动本地网页和 Harness，不初始化机器人。
进入 Real robot 模式后，可在网页完成现场确认并点击按钮启动控制栈与 Python Agent。

start-all 会直接初始化真实机器人，并在终端要求输入一次硬件确认口令；
非交互启动需预先设置:
  export ROBOT_ASSISTANT_HARDWARE_CONFIRM=${HARDWARE_CONFIRMATION}

启动后只需打开:
  http://${SOUL_HOST}:${SOUL_PORT}/#playground
EOF
}

command_name="${1:-start}"
case "${command_name}" in
    __supervise)
        supervise_stack
        ;;
    recover-assistant)
        recover_assistant_stack
        ;;
    start-assistant)
        start_assistant_from_console
        ;;
    stop-assistant)
        stop_assistant_from_console
        ;;
    check)
        preflight console
        prepare_harness_patch
        log "网页操作台启动前检查通过；未启动任何新进程。"
        ;;
    start|console)
        start_console
        ;;
    start-all)
        start_all
        ;;
    stop)
        stop_all
        status_all || true
        ;;
    restart)
        stop_all
        start_console
        ;;
    restart-all)
        stop_all
        start_all
        ;;
    reload-ui)
        reload_ui
        ;;
    status)
        status_all
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
