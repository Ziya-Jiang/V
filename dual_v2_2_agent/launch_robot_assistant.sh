#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTROL_ROOT="${ROBOT_ASSISTANT_CONTROL_ROOT:-/mnt/ssd/user/V/dual_v2_2_control}"
CONTROL_LAUNCHER="${CONTROL_ROOT}/launch_dual_v2_2.sh"
CONTROL_LOG_DIR="${CONTROL_ROOT}/.dual_v2_2_runtime/logs"
PYTHON_BIN="${ROBOT_AGENT_PYTHON:-/mnt/ssd/robotarm/env/robotram_wx/bin/python}"
DEEPSEEK_ROOT="${ROOT_DIR}/deepseek-harness"
GUI_ENTRY="${DEEPSEEK_ROOT}/apps/robot-assistant/src/index.ts"
CALIBRATION_PATH="${ROBOTRAM_ORBBEC_CALIBRATION_PATH:-${ROOT_DIR}/calibration/session_demo/base_to_camera/results/machine-readable results.json}"
OUTPUT_DIR="${ROBOT_AGENT_OUTPUT_DIR:-${ROOT_DIR}/RobotRAM/output/scene_57}"

RUNTIME_DIR="${ROBOT_ASSISTANT_RUNTIME_DIR:-${ROOT_DIR}/.robot_assistant_runtime}"
PID_DIR="${RUNTIME_DIR}/pids"
LOG_DIR="${RUNTIME_DIR}/logs"
STATUS_DIR="${RUNTIME_DIR}/status"
TOKEN_FILE="${RUNTIME_DIR}/agent.token"
AGENT_PID_FILE="${PID_DIR}/python_agent.pid"
GUI_PID_FILE="${PID_DIR}/gui.pid"
SUPERVISOR_PID_FILE="${PID_DIR}/supervisor.pid"
AGENT_LOG="${LOG_DIR}/python_agent.log"
GUI_LOG="${LOG_DIR}/gui.log"
SUPERVISOR_LOG="${LOG_DIR}/supervisor.log"

AGENT_HOST="127.0.0.1"
AGENT_PORT="${ROBOT_AGENT_PORT:-15555}"
GUI_PORT="${ROBOT_ASSISTANT_PORT:-3090}"
AGENT_TIMEOUT_S="${ROBOT_AGENT_STARTUP_TIMEOUT_S:-900}"
GUI_TIMEOUT_S="${ROBOT_ASSISTANT_GUI_TIMEOUT_S:-60}"
HARDWARE_CONFIRMATION="ENABLE_DUAL_V2_2_HARDWARE"
HEADLESS="${ROBOT_ASSISTANT_HEADLESS:-0}"
RUNTIME_ERROR_PATTERN='(^|[^[:alnum:]_])ERROR([^[:alnum:]_]|$)|\[FAULT\]|Traceback|Assertion .* failed|Aborted|SHUTTING DOWN'
BENIGN_RUNTIME_ERROR_PATTERN='^(left|right) Inspire Hand J[0-9]+ before OPEN: error=0x0+, status=[0-9]+$'
STARTUP_CLEANUP_REQUIRED=0
AGENT_START_FAILURE_REASON=""

mkdir -p "${PID_DIR}" "${LOG_DIR}" "${STATUS_DIR}"

log() {
    printf '[robot-assistant] %s\n' "$*"
}

die() {
    log "错误: $*" >&2
    return 1
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
    if [[ -n "${pid}" ]] && ! pid_is_running "${pid}"; then
        rm -f "${file}"
    fi
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

port_is_free() {
    local protocol="$1"
    local port="$2"
    if [[ "${protocol}" == "tcp" ]]; then
        if ss -H -ltn "( sport = :${port} )" | rg -q .; then
            return 1
        fi
    else
        if ss -H -lun "( sport = :${port} )" | rg -q .; then
            return 1
        fi
    fi
    return 0
}

assert_port_free() {
    local protocol="$1"
    local port="$2"
    port_is_free "${protocol}" "${port}" ||
        die "${protocol^^} 端口 ${port} 已被占用"
}

process_is_plan_only_agent() {
    local pid="$1"
    local environment_file="/proc/${pid}/environ"
    [[ -r "${environment_file}" ]] || return 1
    tr '\0' '\n' < "${environment_file}" 2>/dev/null |
        rg -qx 'ROBOT_AGENT_PLAN_ONLY=1'
}

assert_no_conflicting_hardware_owner() {
    local conflicts=""
    local candidate=""
    local pid=""
    while IFS= read -r candidate; do
        read -r pid _ <<< "${candidate}"
        if [[ "${candidate}" == *"-m robot_agent.server"* ]] &&
            process_is_plan_only_agent "${pid}"; then
            log "忽略已验证的 plan-only Agent (PID ${pid})。"
            continue
        fi
        conflicts+="${candidate}"$'\n'
    done < <(
        ps -eo pid=,comm=,args= |
            rg \
                '^\s*[0-9]+\s+(python(3(\.[0-9]+)?)?\s+.*(dual_v2_2_agent\.py|robot_assistant_bridge\.py.*--mode hardware|-m robot_agent\.server)|bash\s+(bash\s+)?(\./|/[^ ]+/)launch_dual_v2_2\.sh\s+(start|restart)(\s|$))' \
            2>/dev/null || true
    )
    conflicts="${conflicts%$'\n'}"
    [[ -z "${conflicts}" ]] ||
        die "检测到其他机器人 Agent/Bridge，请先停止:
${conflicts}"
}

ensure_token() {
    if [[ -n "${ROBOT_AGENT_TOKEN:-}" ]]; then
        [[ ${#ROBOT_AGENT_TOKEN} -ge 24 ]] ||
            die "ROBOT_AGENT_TOKEN 至少需要 24 个字符"
        printf '%s' "${ROBOT_AGENT_TOKEN}"
        return
    fi
    if [[ ! -s "${TOKEN_FILE}" ]]; then
        umask 077
        "${PYTHON_BIN}" -c \
            'import secrets; print(secrets.token_urlsafe(32), end="")' \
            > "${TOKEN_FILE}"
    fi
    chmod 600 "${TOKEN_FILE}"
    tr -d '\r\n' < "${TOKEN_FILE}"
}

control_is_ready() {
    "${CONTROL_LAUNCHER}" status >/dev/null 2>&1
}

component_log_has_error() {
    local log_file="$1"
    local matched_line=""
    [[ -f "${log_file}" ]] || return 1

    while IFS= read -r matched_line; do
        if [[ "${matched_line}" =~ ${BENIGN_RUNTIME_ERROR_PATTERN} ]]; then
            continue
        fi
        return 0
    done < <(rg -i -- "${RUNTIME_ERROR_PATTERN}" "${log_file}" || true)

    return 1
}

control_logs_healthy() {
    local log_file=""
    for log_file in \
        "${CONTROL_LOG_DIR}/rt_control.log" \
        "${CONTROL_LOG_DIR}/upper_software.log" \
        "${CONTROL_LOG_DIR}/upper_planner.log"; do
        if component_log_has_error "${log_file}"; then
            return 1
        fi
    done
    return 0
}

agent_health_check() {
    local token="$1"
    local diagnostic=""
    local status=0
    diagnostic="$(
        ROBOT_AGENT_TOKEN="${token}" ROBOT_AGENT_HOST="${AGENT_HOST}" \
            ROBOT_AGENT_PORT="${AGENT_PORT}" "${PYTHON_BIN}" -c '
import json
import os
import socket
import sys

request = {
    "method": "health",
    "args": {},
    "token": os.environ["ROBOT_AGENT_TOKEN"],
}
try:
    with socket.create_connection(
        (os.environ["ROBOT_AGENT_HOST"], int(os.environ["ROBOT_AGENT_PORT"])),
        timeout=2.0,
    ) as connection:
        connection.sendall((json.dumps(request) + "\n").encode())
        response = b""
        while b"\n" not in response:
            chunk = connection.recv(65536)
            if not chunk:
                break
            response += chunk
    payload = json.loads(response.decode())
except (OSError, ValueError, json.JSONDecodeError):
    raise SystemExit(1)
if not payload.get("ok"):
    print(
        "Python Agent 健康检查失败: "
        + str(payload.get("error") or payload.get("message") or "未知错误")
    )
    raise SystemExit(2)
data = payload.get("data") or {}
state = data.get("bootstrapState", data.get("state"))
if state == "error":
    print(
        "Python Agent 初始化失败: "
        + str(data.get("bootstrapError") or "未提供具体错误")
    )
    raise SystemExit(2)
if state == "ready" or data.get("ready", False):
    raise SystemExit(0)
raise SystemExit(1)
' 2>/dev/null
    )" || status=$?
    if (( status == 2 )); then
        AGENT_START_FAILURE_REASON="${diagnostic:-Python Agent 初始化失败。}"
    fi
    return "${status}"
}

wait_for_agent() {
    local token="$1"
    local pid="$2"
    local health_status=0
    local deadline=$((SECONDS + AGENT_TIMEOUT_S))
    AGENT_START_FAILURE_REASON=""
    while (( SECONDS < deadline )); do
        if ! pid_is_running "${pid}"; then
            AGENT_START_FAILURE_REASON="Python Agent 进程在初始化完成前退出。"
            return 1
        fi
        if ! control_is_ready; then
            AGENT_START_FAILURE_REASON="机器人控制栈在 Python Agent 初始化期间退出。"
            return 1
        fi
        if ! control_logs_healthy; then
            AGENT_START_FAILURE_REASON="机器人控制栈日志出现 ERROR 或 FAULT。"
            return 1
        fi
        health_status=0
        agent_health_check "${token}" || health_status=$?
        case "${health_status}" in
            0)
                return 0
                ;;
            2)
                return 2
                ;;
        esac
        sleep 1
    done
    AGENT_START_FAILURE_REASON="Python Agent 在 ${AGENT_TIMEOUT_S}s 内未进入 ready 状态。"
    return 1
}

wait_for_gui() {
    local pid="$1"
    local deadline=$((SECONDS + GUI_TIMEOUT_S))
    while (( SECONDS < deadline )); do
        pid_is_running "${pid}" || return 1
        control_is_ready || return 1
        control_logs_healthy || return 1
        if curl --fail --silent --max-time 2 \
            "http://127.0.0.1:${GUI_PORT}/api/state" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.5
    done
    return 1
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
    die "无法停止 ${name} 进程组 ${pid}；保留 PID 文件以便继续清理"
}

status_component() {
    local name="$1"
    local pid_file="$2"
    local pid=""
    pid="$(pid_from_file "${pid_file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && pid_is_running "${pid}"; then
        printf '%-20s RUNNING pid=%s\n' "${name}" "${pid}"
        return 0
    fi
    printf '%-20s STOPPED\n' "${name}"
    return 1
}

preflight() {
    assert_command ss
    assert_command rg
    assert_command ps
    assert_command setsid
    assert_command curl
    assert_command node
    assert_executable "${CONTROL_LAUNCHER}"
    assert_executable "${PYTHON_BIN}"
    [[ "${HEADLESS}" == "0" || "${HEADLESS}" == "1" ]] ||
        die "ROBOT_ASSISTANT_HEADLESS 只允许 0 或 1"
    if [[ "${HEADLESS}" != "1" ]]; then
        assert_file "${GUI_ENTRY}"
    fi
    assert_file "${CALIBRATION_PATH}"
    assert_file "${OUTPUT_DIR}/vlm_categories.json"

    [[ "${ROBOT_ASSISTANT_HARDWARE_CONFIRM:-}" == "${HARDWARE_CONFIRMATION}" ]] ||
        die "真实模式需要 export ROBOT_ASSISTANT_HARDWARE_CONFIRM=${HARDWARE_CONFIRMATION}"

    remove_stale_pid_file "${AGENT_PID_FILE}"
    remove_stale_pid_file "${GUI_PID_FILE}"
    remove_stale_pid_file "${SUPERVISOR_PID_FILE}"
    local agent_pid=""
    local gui_pid=""
    local supervisor_pid=""
    agent_pid="$(pid_from_file "${AGENT_PID_FILE}" 2>/dev/null || true)"
    gui_pid="$(pid_from_file "${GUI_PID_FILE}" 2>/dev/null || true)"
    supervisor_pid="$(pid_from_file "${SUPERVISOR_PID_FILE}" 2>/dev/null || true)"
    [[ -z "${agent_pid}" ]] || die "Python Agent 已运行: ${agent_pid}"
    [[ -z "${gui_pid}" ]] || die "GUI 已运行: ${gui_pid}"
    [[ -z "${supervisor_pid}" ]] || die "安全监督器已运行: ${supervisor_pid}"

    assert_no_conflicting_hardware_owner
    assert_port_free tcp "${AGENT_PORT}"
    if [[ "${HEADLESS}" != "1" ]]; then
        assert_port_free tcp "${GUI_PORT}"
    fi
    assert_port_free udp 10165
}

start_control() {
    if control_is_ready && control_logs_healthy; then
        log "机器人控制栈已就绪，继续复用。"
        return
    fi
    log "清理不完整控制栈并重新启动..."
    "${CONTROL_LAUNCHER}" stop >/dev/null 2>&1 || true
    CUARM_MONITOR_AFTER_START=0 \
        CUARM_ALLOW_STARTUP_POSITION_MISMATCH="${CUARM_ALLOW_STARTUP_POSITION_MISMATCH:-1}" \
        CUARM_RECOVER_STARTUP_FAULT="${CUARM_RECOVER_STARTUP_FAULT:-1}" \
        "${CONTROL_LAUNCHER}" start
    if ! control_is_ready || ! control_logs_healthy; then
        die "机器人控制栈未就绪或日志包含运行时错误"
    fi
}

start_agent() {
    local token="$1"
    : > "${AGENT_LOG}"
    log "启动持久化 Python Agent..."
    (
        cd "${ROOT_DIR}"
        exec setsid env \
            PYTHONUNBUFFERED=1 \
            ROBOT_AGENT_TOKEN="${token}" \
            ROBOT_AGENT_STATUS_DIR="${STATUS_DIR}" \
            ROBOT_AGENT_EAGER_BOOTSTRAP=1 \
            ROBOT_AGENT_PORT="${AGENT_PORT}" \
            ROBOTRAM_ROOT="${ROOT_DIR}/RobotRAM" \
            ROBOTRAM_ORBBEC_CALIBRATION_PATH="${CALIBRATION_PATH}" \
            "${PYTHON_BIN}" -m robot_agent.server
    ) >> "${AGENT_LOG}" 2>&1 < /dev/null &
    local pid=$!
    local wait_status=0
    printf '%s\n' "${pid}" > "${AGENT_PID_FILE}"
    wait_for_agent "${token}" "${pid}" || wait_status=$?
    if (( wait_status != 0 )); then
        if [[ -n "${AGENT_START_FAILURE_REASON}" ]]; then
            log "Python Agent 启动失败原因: ${AGENT_START_FAILURE_REASON}"
        fi
        if (( wait_status == 2 )); then
            log "检测到不可恢复的初始化错误，已提前结束等待。"
        fi
        log "Python Agent 启动失败，最近日志:"
        printf '%s\n' "----------------------------------------"
        tail -n 80 "${AGENT_LOG}" || true
        printf '%s\n' "----------------------------------------"
        return 1
    fi
    log "Python Agent 已就绪。"
}

start_gui() {
    local token="$1"
    : > "${GUI_LOG}"
    log "启动 Robot Assistant GUI..."
    (
        cd "${DEEPSEEK_ROOT}"
        exec setsid env \
            ROBOT_ASSISTANT_RUNTIME=python-agent \
            ROBOT_ASSISTANT_PORT="${GUI_PORT}" \
            ROBOT_ASSISTANT_HARDWARE_CONFIRM="${HARDWARE_CONFIRMATION}" \
            ROBOT_AGENT_HOST="${AGENT_HOST}" \
            ROBOT_AGENT_PORT="${AGENT_PORT}" \
            ROBOT_AGENT_TOKEN="${token}" \
            ROBOT_AGENT_OUTPUT_DIR="${OUTPUT_DIR}" \
            node --experimental-strip-types "${GUI_ENTRY}"
    ) >> "${GUI_LOG}" 2>&1 < /dev/null &
    local pid=$!
    printf '%s\n' "${pid}" > "${GUI_PID_FILE}"
    if ! wait_for_gui "${pid}"; then
        log "GUI 启动失败，最近日志:"
        printf '%s\n' "----------------------------------------"
        tail -n 80 "${GUI_LOG}" || true
        printf '%s\n' "----------------------------------------"
        return 1
    fi
    log "GUI 已就绪: http://127.0.0.1:${GUI_PORT}"
}

supervisor_exit() {
    rm -f "${SUPERVISOR_PID_FILE}"
    exit 0
}

supervise_stack() {
    local token=""
    local transient_failures=0
    local gui_ready=0
    trap supervisor_exit INT TERM
    token="${ROBOT_AGENT_TOKEN:-}"
    if [[ -z "${token}" ]]; then
        token="$(tr -d '\r\n' < "${TOKEN_FILE}")"
    fi
    log "安全监督器已启动。"
    while true; do
        if ! control_is_ready || ! control_logs_healthy; then
            log "检测到控制栈退出或 ERROR，立即清理完整 Agent 栈。"
            break
        fi
        gui_ready=0
        if [[ "${HEADLESS}" == "1" ]]; then
            gui_ready=1
        elif status_component "gui" "${GUI_PID_FILE}" >/dev/null &&
            curl --fail --silent --max-time 2 \
                "http://127.0.0.1:${GUI_PORT}/api/state" >/dev/null 2>&1; then
            gui_ready=1
        fi
        if status_component "python_agent" "${AGENT_PID_FILE}" >/dev/null &&
            (( gui_ready == 1 )) &&
            agent_health_check "${token}"; then
            transient_failures=0
        else
            ((transient_failures++)) || true
            if (( transient_failures >= 3 )); then
                log "Agent/GUI 连续三次健康检查失败，立即清理完整栈。"
                break
            fi
        fi
        sleep 1
    done

    stop_component "Robot Assistant GUI" "${GUI_PID_FILE}"
    stop_component "Python Agent" "${AGENT_PID_FILE}"
    "${CONTROL_LAUNCHER}" stop || true
    rm -f "${SUPERVISOR_PID_FILE}"
    return 1
}

start_supervisor() {
    local token="$1"
    : > "${SUPERVISOR_LOG}"
    (
        cd "${ROOT_DIR}"
        exec setsid env ROBOT_AGENT_TOKEN="${token}" \
            "${BASH_SOURCE[0]}" __supervise
    ) >> "${SUPERVISOR_LOG}" 2>&1 < /dev/null &
    local pid=$!
    printf '%s\n' "${pid}" > "${SUPERVISOR_PID_FILE}"
    sleep 0.2
    pid_is_running "${pid}" || return 1
    log "安全监督器已启动 (PID ${pid})。"
}

stop_all() {
    local failed=0
    stop_component "Robot Assistant Supervisor" "${SUPERVISOR_PID_FILE}" ||
        failed=1
    stop_component "Robot Assistant GUI" "${GUI_PID_FILE}" ||
        failed=1
    stop_component "Python Agent" "${AGENT_PID_FILE}" ||
        failed=1
    if [[ "${ROBOT_ASSISTANT_KEEP_CONTROL_STACK:-0}" != "1" ]]; then
        "${CONTROL_LAUNCHER}" stop || failed=1
    fi
    return "${failed}"
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

install_startup_traps() {
    trap startup_exit_cleanup EXIT
    trap 'exit 130' INT TERM
}

clear_startup_traps() {
    trap - EXIT INT TERM
}

start_all() {
    preflight
    local token=""
    token="$(ensure_token)"
    local started_control=0
    local started_agent=0

    if ! control_is_ready || ! control_logs_healthy; then
        started_control=1
    fi
    STARTUP_CLEANUP_REQUIRED=1
    if ! start_control; then
        return 1
    fi
    if ! start_agent "${token}"; then
        stop_component "Python Agent" "${AGENT_PID_FILE}"
        if (( started_control == 1 )); then
            "${CONTROL_LAUNCHER}" stop || true
        fi
        return 1
    fi
    started_agent=1
    if [[ "${HEADLESS}" != "1" ]]; then
        if ! start_gui "${token}"; then
            stop_component "Robot Assistant GUI" "${GUI_PID_FILE}"
            if (( started_agent == 1 )); then
                stop_component "Python Agent" "${AGENT_PID_FILE}"
            fi
            if (( started_control == 1 )); then
                "${CONTROL_LAUNCHER}" stop || true
            fi
            return 1
        fi
    fi
    if ! control_is_ready || ! control_logs_healthy; then
        log "控制栈在 GUI 启动期间出现异常，执行完整回滚。"
        stop_component "Robot Assistant GUI" "${GUI_PID_FILE}"
        stop_component "Python Agent" "${AGENT_PID_FILE}"
        "${CONTROL_LAUNCHER}" stop || true
        return 1
    fi
    if ! start_supervisor "${token}"; then
        log "安全监督器启动失败，执行完整回滚。"
        stop_component "Robot Assistant GUI" "${GUI_PID_FILE}"
        stop_component "Python Agent" "${AGENT_PID_FILE}"
        "${CONTROL_LAUNCHER}" stop || true
        return 1
    fi

    log "完整 Agent 栈已就绪。"
    status_all || true
    STARTUP_CLEANUP_REQUIRED=0
}

status_all() {
    local ok=0
    status_component "supervisor" "${SUPERVISOR_PID_FILE}" || ok=1
    status_component "python_agent" "${AGENT_PID_FILE}" || ok=1
    if [[ "${HEADLESS}" == "1" ]]; then
        printf '%-20s DISABLED (由统一 Robot Soul GUI 替代)\n' "gui"
    else
        status_component "gui" "${GUI_PID_FILE}" || ok=1
    fi
    "${CONTROL_LAUNCHER}" status || ok=1
    printf 'agent log: %s\n' "${AGENT_LOG}"
    printf 'gui log:   %s\n' "${GUI_LOG}"
    printf 'supervisor log: %s\n' "${SUPERVISOR_LOG}"
    return "${ok}"
}

usage() {
    cat <<EOF
用法: $0 [check|start|stop|restart|status]

真实启动前必须设置:
  export ROBOT_ASSISTANT_HARDWARE_CONFIRM=${HARDWARE_CONFIRMATION}

启动完成后打开:
  http://127.0.0.1:${GUI_PORT}
EOF
}

command_name="${1:-start}"
case "${command_name}" in
    __supervise)
        supervise_stack
        ;;
    check)
        preflight
        log "启动前检查通过；未启动任何进程。"
        ;;
    start)
        install_startup_traps
        start_all
        clear_startup_traps
        ;;
    stop)
        stop_all
        status_all || true
        ;;
    restart)
        stop_all
        install_startup_traps
        start_all
        clear_startup_traps
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
