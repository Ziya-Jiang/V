#!/usr/bin/env python3
"""Apply a bounded receiver-watchdog fix, retaining all existing source edits."""
import argparse
import hashlib
import shutil
from pathlib import Path
import time


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise RuntimeError('Source changed or already patched: '+before[:60])
    return text.replace(before, after, 1)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cpp-sha256', required=True)
    p.add_argument('--header-sha256', required=True)
    args = p.parse_args()
    root = Path('/mnt/ssd/user/V/dual_v2_2_agent/cart_grasp')
    panel = Path('/mnt/ssd/user/V/dual_v2_2_control/cuarm_upper_software/src/panel')
    cpp_path, header_path = panel/'remote_page_widget.cpp', panel/'remote_page_widget.h'
    for path, sha in ((cpp_path, args.cpp_sha256), (header_path, args.header_sha256)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != sha:
            raise RuntimeError('Concurrent source change; refusing overwrite: '+str(path))
    cpp, header = cpp_path.read_text(), header_path.read_text()
    cpp = replace_once(cpp, 'switch (last_set_target_type_){', 'switch (command.target_type){')
    header = replace_once(header, '#include <memory>', '#include <memory>\n#include "remote_command_lease.h"')
    header = replace_once(header, '        std::mutex received_mutex_;',
        '        std::mutex received_mutex_;\n        double received_remote_at_ = 0.;\n        RemoteCommandLease command_lease_;')
    cpp = replace_once(cpp, '    //Send PanelState\n', '''    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (is_running_ && command_lease_.expired(now)) {
        std::cerr << "[REMOTE WATCHDOG] Planner command stream lost; holding measured joints. Press Start to resume." << std::endl;
        slotStartButtonClicked(false);
        changed = true;
    }

    //Send PanelState
''')
    cpp = replace_once(cpp, '        //Get lastest RemotePanelCommand\n',
        '        double received_at = 0.;\n        //Get lastest RemotePanelCommand\n')
    cpp = replace_once(cpp, '            remote_panel_command_unique_ptr_ = std::move(received_remote_panel_command_unique_ptr_);',
        '            remote_panel_command_unique_ptr_ = std::move(received_remote_panel_command_unique_ptr_);\n            received_at = received_remote_at_;')
    cpp = replace_once(cpp, '        RemotePanelCommand remote_cmd = *remote_panel_command_unique_ptr_.get();', '''        RemotePanelCommand remote_cmd = *remote_panel_command_unique_ptr_.get();
        if (!isValidRemoteCommand(remote_cmd)) return changed;
        const double consumed_at = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!command_lease_.accept(received_at, consumed_at)) {
            std::cerr << "[REMOTE WATCHDOG] Expired queued command rejected; manual Start required." << std::endl;
            slotStartButtonClicked(false);
            return true;
        }''')
    cpp = replace_once(cpp, '    is_running_ = true;\n    panel_state_.state = PanelRemoteState::kStart;\n    received_remote_panel_command_unique_ptr_.reset(nullptr);',
        '    is_running_ = true;\n    command_lease_.start();\n    clearJointAndGripperCommand();\n    panel_state_.state = PanelRemoteState::kStart;\n    { std::lock_guard<std::mutex> lock(received_mutex_);\n      received_remote_panel_command_unique_ptr_.reset(nullptr); received_remote_at_ = 0.; }')
    cpp = replace_once(cpp, '    is_running_ = false;\n    panel_state_.state = PanelRemoteState::kStop;\n    received_remote_panel_command_unique_ptr_.reset(nullptr);',
        '    is_running_ = false;\n    command_lease_.stop();\n    panel_state_.state = PanelRemoteState::kStop;\n    { std::lock_guard<std::mutex> lock(received_mutex_);\n      received_remote_panel_command_unique_ptr_.reset(nullptr); received_remote_at_ = 0.; }')
    cpp = replace_once(cpp, '                received_remote_panel_command_unique_ptr_ = std::move(temp_ptr);',
        '                received_remote_panel_command_unique_ptr_ = std::move(temp_ptr);\n                received_remote_at_ = std::chrono::duration<double>(\n                    std::chrono::steady_clock::now().time_since_epoch()).count();')
    backup = root/'runtime/backups'/('native_upper_'+time.strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(parents=True, exist_ok=False)
    for path in (cpp_path, header_path):
        shutil.copy2(path, backup/path.name)
    binary = panel.parents[1]/'build/dual_v2_2_upper_software'
    if binary.exists(): shutil.copy2(binary, backup/binary.name)
    shutil.copy2(root/'hardware_patch/remote_command_lease.h', panel/'remote_command_lease.h')
    cpp_path.write_text(cpp)
    header_path.write_text(header)
    print('Backups: '+str(backup))


if __name__ == '__main__': main()
