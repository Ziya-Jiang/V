#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RT_DIR="${ROOT_DIR}/cuarm_rt_control"
UPPER_DIR="${ROOT_DIR}/cuarm_upper_software"
PLANNER_DIR="${ROOT_DIR}/cuarm_upper_planner"

RT_BIN="${RT_DIR}/build/dual_v2_2_rt_control"
RT_CONTROL_SOURCE="${RT_DIR}/example/rt_control.cpp"
RT_CANFD_SOURCE="${RT_DIR}/hardware_modules/realman_dual_arm/src/usb_to_can/can_fd.c"
RT_ARM_SOURCE="${RT_DIR}/src/arm/dual_v2_2_arm.cpp"
UPPER_BIN="${UPPER_DIR}/build/dual_v2_2_upper_software"
PLANNER_SCRIPT="${PLANNER_DIR}/dual_v2_2_upper_planner.py"
RT_PROCESS_PATTERN="${RT_BIN}$"
UPPER_PROCESS_PATTERN="${UPPER_BIN}( --headless-auto)?$"
PLANNER_PROCESS_PATTERN="python(3)? ${PLANNER_SCRIPT}$"

RUNTIME_DIR="${CUARM_RUNTIME_DIR:-${ROOT_DIR}/.dual_v2_2_runtime}"
LOG_DIR="${RUNTIME_DIR}/logs"
PID_DIR="${RUNTIME_DIR}/pids"

RT_PID_FILE="${PID_DIR}/rt_control.pid"
RT_WRAPPER_PID_FILE="${PID_DIR}/rt_control_sudo.pid"
UPPER_PID_FILE="${PID_DIR}/upper_software.pid"
PLANNER_PID_FILE="${PID_DIR}/upper_planner.pid"

RT_LOG="${LOG_DIR}/rt_control.log"
UPPER_LOG="${LOG_DIR}/upper_software.log"
PLANNER_LOG="${LOG_DIR}/upper_planner.log"

CONDA_BIN="${CONDA_EXE:-/home/vlmdemo/miniconda3/bin/conda}"
CONDA_ENV="${CUARM_CONDA_ENV:-curobo}"
STARTUP_TIMEOUT_S="${CUARM_STARTUP_TIMEOUT_S:-60}"
PLANNER_TIMEOUT_S="${CUARM_PLANNER_TIMEOUT_S:-240}"
ALLOW_POSITION_MISMATCH="${CUARM_ALLOW_STARTUP_POSITION_MISMATCH:-1}"
RECOVER_STARTUP_FAULT="${CUARM_RECOVER_STARTUP_FAULT:-1}"
STATUS_INTERVAL_S="${CUARM_STATUS_INTERVAL_S:-2}"
MONITOR_AFTER_START="${CUARM_MONITOR_AFTER_START:-1}"
RUNTIME_ERROR_PATTERN='(^|[^[:alnum:]_])ERROR([^[:alnum:]_]|$)|\[FAULT\]|Traceback|Assertion .* failed|Aborted|SHUTTING DOWN'
BENIGN_RUNTIME_ERROR_PATTERN='^(left|right) Inspire Hand J[0-9]+ before OPEN: error=0x0+, status=[0-9]+$'
USBCANFD_VIDPID="${CUARM_USBCANFD_VIDPID:-3068:0009}"
USBCANFD_REQUIRED="${CUARM_USBCANFD_REQUIRED:-2}"
RT_START_RETRIES="${CUARM_RT_START_RETRIES:-3}"
LEFT_ARM_USBCANFD_SN="${CUARM_LEFT_ARM_USBCANFD_SN:-B961A51020E9075418A0}"
RIGHT_ARM_USBCANFD_SN="${CUARM_RIGHT_ARM_USBCANFD_SN:-6472C8BAD0F20BF466B0}"

mkdir -p "${LOG_DIR}" "${PID_DIR}"

normalize_terminal_output() {
    if [[ -t 1 && -e /dev/tty ]]; then
        stty opost onlcr < /dev/tty 2>/dev/null || true
    fi
}

log() {
    normalize_terminal_output
    printf '[dual_v2_2] %s\n' "$*"
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

remove_stale_pid_file() {
    local file="$1"
    local pid=""
    pid="$(pid_from_file "${file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && ! pid_is_running "${pid}"; then
        rm -f "${file}"
    fi
}

wait_for_pid_file() {
    local file="$1"
    local timeout_s="$2"
    local deadline=$((SECONDS + timeout_s))
    while (( SECONDS < deadline )); do
        local pid=""
        pid="$(pid_from_file "${file}" 2>/dev/null || true)"
        if [[ -n "${pid}" ]] && pid_is_running "${pid}"; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

wait_for_log() {
    local file="$1"
    local pattern="$2"
    local pid_file="$3"
    local timeout_s="$4"
    local deadline=$((SECONDS + timeout_s))

    while (( SECONDS < deadline )); do
        if [[ -f "${file}" ]] && rg -q -- "${pattern}" "${file}"; then
            return 0
        fi
        local pid=""
        pid="$(pid_from_file "${pid_file}" 2>/dev/null || true)"
        if [[ -n "${pid}" ]] && ! pid_is_running "${pid}"; then
            return 2
        fi
        sleep 0.2
    done
    return 1
}

show_log_tail() {
    local file="$1"
    if [[ -f "${file}" ]]; then
        printf '%s\n' "----- ${file} -----" >&2
        tail -n 60 "${file}" >&2 || true
    fi
}

assert_command() {
    command -v "$1" >/dev/null 2>&1 || die "缺少命令: $1"
}

assert_file_executable() {
    [[ -x "$1" ]] || die "可执行文件不存在，请先构建: $1"
}

assert_file_readable() {
    [[ -r "$1" ]] || die "文件不存在: $1"
}

assert_no_matching_process() {
    local pattern="$1"
    local label="$2"
    if pgrep -f -- "${pattern}" >/dev/null 2>&1; then
        die "${label} 已在运行；请先执行 $0 stop"
    fi
}

assert_udp_port_free() {
    local port="$1"
    if ss -H -lun | awk '{print $5}' | rg -q -- "[:.]${port}$"; then
        die "UDP 端口 ${port} 已被占用"
    fi
}

wait_until_stopped() {
    local pid="$1"
    local timeout_s="$2"
    local deadline=$((SECONDS + timeout_s))
    while (( SECONDS < deadline )); do
        if ! pid_is_running "${pid}"; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

signal_user_process() {
    local pid="$1"
    local signal="$2"
    local pgid=""
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"
    if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
        kill "-${signal}" -- "-${pgid}" 2>/dev/null || true
    else
        kill "-${signal}" "${pid}" 2>/dev/null || true
    fi
}

signal_root_process() {
    local pid="$1"
    local signal="$2"
    local pgid=""
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"
    if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
        sudo -n kill "-${signal}" -- "-${pgid}" 2>/dev/null || true
    else
        sudo -n kill "-${signal}" "${pid}" 2>/dev/null || true
    fi
}

matching_pids() {
    local pattern="$1"
    pgrep -f -- "${pattern}" 2>/dev/null || true
}

stop_residual_user_processes() {
    local name="$1"
    local pattern="$2"
    local pids=""
    pids="$(matching_pids "${pattern}")"
    [[ -n "${pids}" ]] || return 0

    log "清理残留 ${name} 进程: ${pids//$'\n'/ }"
    local pid=""
    for pid in ${pids}; do
        kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 0.5
    for pid in ${pids}; do
        if pid_is_running "${pid}"; then
            kill -KILL "${pid}" 2>/dev/null || true
        fi
    done
}

stop_residual_root_processes() {
    local name="$1"
    local pattern="$2"
    local pids=""
    pids="$(matching_pids "${pattern}")"
    [[ -n "${pids}" ]] || return 0

    log "清理残留 ${name} 进程: ${pids//$'\n'/ }"
    local pid=""
    for pid in ${pids}; do
        sudo -n kill -INT "${pid}" 2>/dev/null || true
    done
    sleep 1
    for pid in ${pids}; do
        if pid_is_running "${pid}"; then
            sudo -n kill -KILL "${pid}" 2>/dev/null || true
        fi
    done
}

stop_user_process() {
    local name="$1"
    local pid_file="$2"
    local signal="${3:-TERM}"
    local pid=""
    pid="$(pid_from_file "${pid_file}" 2>/dev/null || true)"
    if [[ -z "${pid}" ]] || ! pid_is_running "${pid}"; then
        rm -f "${pid_file}"
        return 0
    fi

    log "停止 ${name} (PID ${pid})..."
    signal_user_process "${pid}" "${signal}"
    if ! wait_until_stopped "${pid}" 10; then
        log "${name} 未及时退出，发送 KILL。"
        signal_user_process "${pid}" KILL
        wait_until_stopped "${pid}" 3 || true
    fi
    rm -f "${pid_file}"
}

stop_rt_process() {
    local rt_pid=""
    local wrapper_pid=""
    rt_pid="$(pid_from_file "${RT_PID_FILE}" 2>/dev/null || true)"
    wrapper_pid="$(pid_from_file "${RT_WRAPPER_PID_FILE}" 2>/dev/null || true)"

    if [[ -n "${rt_pid}" ]] && pid_is_running "${rt_pid}"; then
        log "停止 rt_control (PID ${rt_pid})..."
        signal_root_process "${rt_pid}" INT
        if ! wait_until_stopped "${rt_pid}" 15; then
            log "rt_control 未及时退出，发送 KILL。"
            signal_root_process "${rt_pid}" KILL
            wait_until_stopped "${rt_pid}" 3 || true
        fi
    fi

    if [[ -n "${wrapper_pid}" ]] && pid_is_running "${wrapper_pid}"; then
        kill -TERM "${wrapper_pid}" 2>/dev/null || true
        wait_until_stopped "${wrapper_pid}" 3 || true
    fi
    rm -f "${RT_PID_FILE}" "${RT_WRAPPER_PID_FILE}"
}

stop_all() {
    if [[ -t 0 ]]; then
        sudo -v
    else
        sudo -n true 2>/dev/null || true
    fi
    stop_user_process "upper_planner" "${PLANNER_PID_FILE}"
    stop_residual_user_processes "upper_planner" "${PLANNER_PROCESS_PATTERN}"
    stop_user_process "upper_software" "${UPPER_PID_FILE}"
    stop_residual_user_processes "upper_software" "${UPPER_PROCESS_PATTERN}"
    stop_rt_process
    stop_residual_root_processes "rt_control" "${RT_PROCESS_PATTERN}"
}

status_one() {
    local name="$1"
    local file="$2"
    local pattern="$3"
    local pid=""
    local residual_pids=""
    normalize_terminal_output
    pid="$(pid_from_file "${file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && pid_is_running "${pid}"; then
        printf '%-18s RUNNING pid=%s\n' "${name}" "${pid}"
        return 0
    fi
    residual_pids="$(matching_pids "${pattern}")"
    if [[ -n "${residual_pids}" ]]; then
        printf '%-18s ORPHAN pid=%s\n' \
            "${name}" "${residual_pids//$'\n'/,}"
        return 1
    fi
    printf '%-18s STOPPED\n' "${name}"
    return 1
}

status_all() {
    local result=0
    status_one "rt_control" "${RT_PID_FILE}" "${RT_PROCESS_PATTERN}" || result=1
    status_one "upper_software" "${UPPER_PID_FILE}" "${UPPER_PROCESS_PATTERN}" || result=1
    status_one "upper_planner" "${PLANNER_PID_FILE}" "${PLANNER_PROCESS_PATTERN}" || result=1
    printf 'logs: %s\n' "${LOG_DIR}"
    return "${result}"
}

component_state() {
    local pid_file="$1"
    local pattern="$2"
    local pid=""
    pid="$(pid_from_file "${pid_file}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && pid_is_running "${pid}"; then
        printf 'RUNNING'
    elif [[ -n "$(matching_pids "${pattern}")" ]]; then
        printf 'ORPHAN'
    else
        printf 'ERROR'
    fi
}

component_log_has_error() {
    local log_file="$1"
    local matched_line=""
    [[ -f "${log_file}" ]] || return 1

    while IFS= read -r matched_line; do
        # The hand diagnostic reports its error register even when it is zero.
        # Do not let that successful read trigger the global shutdown monitor.
        if [[ "${matched_line}" =~ ${BENIGN_RUNTIME_ERROR_PATTERN} ]]; then
            continue
        fi
        return 0
    done < <(rg -i -- "${RUNTIME_ERROR_PATTERN}" "${log_file}" || true)

    return 1
}

monitor_interrupt() {
    log "收到中断信号，执行安全退出。"
    stop_all
    exit 130
}

monitor_stack() {
    local key=""
    local rt_state=""
    local upper_state=""
    local planner_state=""

    trap monitor_interrupt INT TERM
    log "进入持续监控；任一组件异常将自动清理退出，按 s 或 Ctrl+C 可主动安全停止。"

    while true; do
        # Keep the credential used for the root RT process valid so an
        # emergency/manual shutdown never gets stuck at a password prompt.
        sudo -n -v >/dev/null 2>&1 || true

        rt_state="$(component_state "${RT_PID_FILE}" "${RT_PROCESS_PATTERN}")"
        upper_state="$(component_state "${UPPER_PID_FILE}" "${UPPER_PROCESS_PATTERN}")"
        planner_state="$(component_state "${PLANNER_PID_FILE}" "${PLANNER_PROCESS_PATTERN}")"
        if component_log_has_error "${RT_LOG}"; then
            rt_state="LOG_ERROR"
        fi
        if component_log_has_error "${UPPER_LOG}"; then
            upper_state="LOG_ERROR"
        fi
        if component_log_has_error "${PLANNER_LOG}"; then
            planner_state="LOG_ERROR"
        fi
        normalize_terminal_output
        printf '[%s] RT=%s | Upper=%s | Planner=%s | 异常自动退出\n' \
            "$(date '+%H:%M:%S')" \
            "${rt_state}" "${upper_state}" "${planner_state}"

        if [[ "${rt_state}" != "RUNNING" ||
              "${upper_state}" != "RUNNING" ||
              "${planner_state}" != "RUNNING" ]]; then
            log "检测到组件异常，立即按 Planner → Upper → RT 顺序清理。"
            log "日志目录: ${LOG_DIR}"
            stop_all
            status_all || true
            trap - INT TERM
            return 1
        fi

        key=""
        if [[ -t 0 ]]; then
            if IFS= read -r -s -n 1 -t "${STATUS_INTERVAL_S}" key; then
                if [[ "${key}" == "s" || "${key}" == "S" ]]; then
                    log "收到 s，按 Planner → Upper → RT 顺序安全退出。"
                    stop_all
                    status_all || true
                    trap - INT TERM
                    return 0
                fi
            fi
        else
            sleep "${STATUS_INTERVAL_S}"
        fi
    done
}

list_usbcanfd_busdev() {
    lsusb | awk -v id="${USBCANFD_VIDPID}" '
        index($0, "ID " id) {
            bus=$2
            dev=$4
            gsub(":", "", dev)
            printf "%s/%s\n", bus, dev
        }
    '
}

count_usbcanfd() {
    local n=0
    n="$(list_usbcanfd_busdev | wc -l)"
    printf '%s' "${n//[[:space:]]/}"
}

wait_for_usbcanfd() {
    local timeout_s="${1:-8}"
    local deadline=$((SECONDS + timeout_s))
    local n=0
    while (( SECONDS < deadline )); do
        n="$(count_usbcanfd)"
        if (( n >= USBCANFD_REQUIRED )); then
            return 0
        fi
        sleep 0.3
    done
    return 1
}

reset_usbcanfd() {
    local busdev=""
    local devices=""
    devices="$(list_usbcanfd_busdev || true)"
    if [[ -z "${devices}" ]]; then
        log "未检测到 USBCANFD (${USBCANFD_VIDPID})，跳过复位。"
        return 0
    fi
    log "复位 USBCANFD: ${devices//$'\n'/ }"
    while read -r busdev; do
        [[ -n "${busdev}" ]] || continue
        sudo -n usbreset "${busdev}" >/dev/null 2>&1 || true
    done <<< "${devices}"
    sleep 1
    wait_for_usbcanfd 8 || true
}

assert_usbcanfd_ready() {
    local n=0
    n="$(count_usbcanfd)"
    if (( n < USBCANFD_REQUIRED )); then
        log "USBCANFD 数量不足 (${n}/${USBCANFD_REQUIRED})，尝试复位后重检。"
        reset_usbcanfd
        n="$(count_usbcanfd)"
    fi
    if (( n < USBCANFD_REQUIRED )); then
        die "需要 ${USBCANFD_REQUIRED} 个 USBCANFD-100U-mini (${USBCANFD_VIDPID})，当前 ${n} 个。请拔插适配器后重试。"
        return 1
    fi
    log "已检测到 ${n} 个 USBCANFD (${USBCANFD_VIDPID})。"
}

assert_rt_can_mapping() {
    local left_pattern=""
    local right_pattern=""
    left_pattern="\\[CANMAP\\] logical_device=1 physical_device=[0-9]+ SN=${LEFT_ARM_USBCANFD_SN}"
    right_pattern="\\[CANMAP\\] logical_device=0 physical_device=[0-9]+ SN=${RIGHT_ARM_USBCANFD_SN}"
    if ! rg -q -- "${left_pattern}" "${RT_LOG}" ||
        ! rg -q -- "${right_pattern}" "${RT_LOG}"; then
        log "未获得可信的左右臂 CAN 序列号绑定:"
        log "  logical 1 左臂期望 ${LEFT_ARM_USBCANFD_SN}"
        log "  logical 0 右臂期望 ${RIGHT_ARM_USBCANFD_SN}"
        die "USBCANFD 固定映射校验失败；已禁止 Upper、Planner 和 Agent 启动。"
        return 1
    fi
    log "USBCANFD 固定映射校验通过: 左臂=${LEFT_ARM_USBCANFD_SN}，右臂=${RIGHT_ARM_USBCANFD_SN}。"
}

start_rt_with_retry() {
    local attempt=1
    local max_attempts="${RT_START_RETRIES}"
    while (( attempt <= max_attempts )); do
        if start_rt; then
            return 0
        fi
        if [[ -f "${RT_LOG}" ]] && rg -q -- 'libusb_submit_transfer' "${RT_LOG}"; then
            log "rt_control USB-CAN 句柄失效 (第 ${attempt}/${max_attempts} 次)，复位适配器后重试。"
            stop_rt_process
            reset_usbcanfd
            ((attempt++)) || true
            continue
        fi
        return 1
    done
    die "rt_control 在 USB 复位重试后仍失败"
    return 1
}

start_rt() {
    : > "${RT_LOG}"
    rm -f "${RT_PID_FILE}" "${RT_WRAPPER_PID_FILE}"

    sudo -n setsid bash -c '
        set -Eeuo pipefail
        rt_dir="$1"
        rt_bin="$2"
        pid_file="$3"
        log_file="$4"
        recover_fault="$5"
        allow_mismatch="$6"
        cd "${rt_dir}"
        printf "%s\n" "$$" > "${pid_file}"
        exec env \
            CUARM_RECOVER_STARTUP_FAULT="${recover_fault}" \
            CUARM_ALLOW_STARTUP_POSITION_MISMATCH="${allow_mismatch}" \
            CUARM_USBCANFD_SN_0="$7" \
            CUARM_USBCANFD_SN_1="$8" \
            stdbuf -oL -eL "${rt_bin}" >> "${log_file}" 2>&1
    ' _ \
        "${RT_DIR}" "${RT_BIN}" "${RT_PID_FILE}" "${RT_LOG}" \
        "${RECOVER_STARTUP_FAULT}" "${ALLOW_POSITION_MISMATCH}" \
        "${RIGHT_ARM_USBCANFD_SN}" "${LEFT_ARM_USBCANFD_SN}" \
        < /dev/null &
    printf '%s\n' "$!" > "${RT_WRAPPER_PID_FILE}"

    if ! wait_for_pid_file "${RT_PID_FILE}" 5; then
        die "rt_control 未创建 PID 文件"
        return 1
    fi
    local rc=0
    wait_for_log "${RT_LOG}" 'Task Start!' "${RT_PID_FILE}" "${STARTUP_TIMEOUT_S}" || rc=$?
    if (( rc != 0 )); then
        show_log_tail "${RT_LOG}"
        die "rt_control 启动失败或等待超时"
        return 1
    fi
    if ! assert_rt_can_mapping; then
        stop_rt_process
        return 1
    fi
    log "rt_control 已就绪。"
}

start_upper() {
    : > "${UPPER_LOG}"
    (
        cd "${UPPER_DIR}"
        exec setsid env \
            QT_QPA_PLATFORM=offscreen \
            QT_QUICK_BACKEND=software \
            LIBGL_ALWAYS_SOFTWARE=1 \
            stdbuf -oL -eL "${UPPER_BIN}" --headless-auto \
            >> "${UPPER_LOG}" 2>&1
    ) < /dev/null &
    printf '%s\n' "$!" > "${UPPER_PID_FILE}"

    local rc=0
    wait_for_log \
        "${UPPER_LOG}" '\[AUTO_READY\]' "${UPPER_PID_FILE}" \
        "${STARTUP_TIMEOUT_S}" || rc=$?
    if (( rc != 0 )); then
        show_log_tail "${UPPER_LOG}"
        die "upper_software 无界面自动连接失败或等待超时"
        return 1
    fi
    log "upper_software 硬件模式与 Remote Connect/Start 已就绪。"
}

start_planner() {
    : > "${PLANNER_LOG}"
    (
        cd "${PLANNER_DIR}"
        exec setsid env PYTHONUNBUFFERED=1 \
            "${CONDA_BIN}" run -n "${CONDA_ENV}" --no-capture-output \
            python "${PLANNER_SCRIPT}" \
            >> "${PLANNER_LOG}" 2>&1
    ) < /dev/null &
    printf '%s\n' "$!" > "${PLANNER_PID_FILE}"

    local rc=0
    wait_for_log \
        "${PLANNER_LOG}" 'Curobo is ready\.' "${PLANNER_PID_FILE}" \
        "${PLANNER_TIMEOUT_S}" || rc=$?
    if (( rc != 0 )); then
        show_log_tail "${PLANNER_LOG}"
        die "upper_planner 启动失败或等待超时"
        return 1
    fi
    log "upper_planner 已就绪。"
}

build_rt_if_stale() {
    if [[ ! -x "${RT_BIN}" ||
        "${RT_CONTROL_SOURCE}" -nt "${RT_BIN}" ||
        "${RT_CANFD_SOURCE}" -nt "${RT_BIN}" ||
        "${RT_ARM_SOURCE}" -nt "${RT_BIN}" ]]; then
        log "检测到 rt_control 安全源码有更新，正在单线程重建。"
        cmake --build "${RT_DIR}/build" \
            --target dual_v2_2_rt_control --parallel 1 ||
            die "rt_control 重建失败；未启动任何机器人硬件。"
    fi
}

start_all() {
    assert_command rg
    assert_command ss
    assert_command awk
    assert_command cmake
    assert_command stdbuf
    assert_command setsid
    assert_command sudo
    assert_command lsusb
    assert_command usbreset
    assert_file_readable "${RT_CONTROL_SOURCE}"
    assert_file_readable "${RT_CANFD_SOURCE}"
    assert_file_readable "${RT_ARM_SOURCE}"
    build_rt_if_stale
    assert_file_executable "${RT_BIN}"
    assert_file_executable "${UPPER_BIN}"
    assert_file_readable "${PLANNER_SCRIPT}"
    assert_file_executable "${CONDA_BIN}"

    remove_stale_pid_file "${RT_PID_FILE}"
    remove_stale_pid_file "${RT_WRAPPER_PID_FILE}"
    remove_stale_pid_file "${UPPER_PID_FILE}"
    remove_stale_pid_file "${PLANNER_PID_FILE}"

    assert_no_matching_process 'dual_v2_2_rt_control$' "rt_control"
    assert_no_matching_process 'dual_v2_2_upper_software( --headless-auto)?$' "upper_software"
    assert_no_matching_process 'python(3)? .*dual_v2_2_upper_planner\.py$' "upper_planner"

    for port in 21201 21101 30240 30241 10166; do
        assert_udp_port_free "${port}"
    done

    "${CONDA_BIN}" run -n "${CONDA_ENV}" python -c 'import sys; print(sys.executable)' \
        >/dev/null 2>&1 ||
        die "Conda 环境不可用: ${CONDA_ENV}"

    true
    if [[ "${ALLOW_POSITION_MISMATCH}" == "1" ]]; then
        log "安全提示: 已启用 CUARM_ALLOW_STARTUP_POSITION_MISMATCH=1。"
    fi

    reset_usbcanfd
    assert_usbcanfd_ready

    local start_failed=0
    start_rt_with_retry || start_failed=1
    if (( start_failed == 0 )); then
        start_upper || start_failed=1
    fi
    if (( start_failed == 0 )); then
        start_planner || start_failed=1
    fi

    if (( start_failed != 0 )); then
        log "启动未完成，回滚已启动的进程。"
        stop_all
        return 1
    fi

    log "全部组件已就绪。"
    status_all || true
    if [[ "${MONITOR_AFTER_START}" == "1" ]]; then
        monitor_stack
    fi
}

usage() {
    cat <<EOF
用法: $0 [start|stop|restart|status]

默认命令为 start。
日志目录: ${LOG_DIR}

可选环境变量:
  CUARM_ALLOW_STARTUP_POSITION_MISMATCH=0|1
  CUARM_RECOVER_STARTUP_FAULT=0|1
  CUARM_LEFT_ARM_USBCANFD_SN=${LEFT_ARM_USBCANFD_SN}
  CUARM_RIGHT_ARM_USBCANFD_SN=${RIGHT_ARM_USBCANFD_SN}
  CUARM_CONDA_ENV=${CONDA_ENV}
  CUARM_STARTUP_TIMEOUT_S=${STARTUP_TIMEOUT_S}
  CUARM_PLANNER_TIMEOUT_S=${PLANNER_TIMEOUT_S}
  CUARM_STATUS_INTERVAL_S=${STATUS_INTERVAL_S}
  CUARM_MONITOR_AFTER_START=0|1
EOF
}

command_name="${1:-start}"
case "${command_name}" in
    start)
        start_all
        ;;
    stop)
        stop_all
        status_all || true
        ;;
    restart)
        stop_all
        start_all
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
