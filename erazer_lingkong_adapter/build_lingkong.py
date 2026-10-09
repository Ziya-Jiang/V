#!/usr/bin/env python3
"""Build an isolated rt_control entry point using the installed driver's CMake objects.
Never edit the original source/config/binary. Fail closed when source anchors change.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--legacy', default='/home/erazer/cuarm_panel_control')
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
legacy = Path(a.legacy).resolve()
source = legacy/'cuarm_rt_control/example/rt_control.cpp'
build = legacy/'cuarm_rt_control/build'
out = root/'runtime/build_lingkong'
out.mkdir(parents=True, exist_ok=True)
s = source.read_text()
def replace(old, new):
    global s
    if s.count(old) != 1:
        raise RuntimeError('Legacy RT source changed; review anchor: '+old[:90])
    s = s.replace(old, new)
# Add the existing V2 hand transport through the same AbstractGripper interface.
hand_root = root/'config_sources/V2/dual_v2_2_control/cuarm_rt_control/hardware_modules/eyou-main'
replace('#include "src/gripper/uart_servo_two_finger_gripper.h"',
        '#include "src/gripper/uart_servo_two_finger_gripper.h"\n#include "inspire_gripper.h"\n#include "lingkong_can_gripper.h"')
replace('''            if (gripper_type == "UartServoTwoFingerGripper"){
                int id = config["robot"]["gripper"][gri_i]["hardware"]["servo_id"].as<int>();''',
'''            if (gripper_type == "UartServoTwoFingerGripper" && gri_i == 1){
                const YAML::Node hardware = YAML::LoadFile(
                    "/home/erazer/Seldon/dual_arm_cockpit/rt_lingkong/right_gripper.yaml"
                )["hardware"];
                LingkongCanGripperConfig lingkong;
                lingkong.interface_name = hardware["interface_name"].as<std::string>();
                lingkong.node_id = hardware["node_id"].as<int>();
                lingkong.open_angle_deg = hardware["open_angle_deg"].as<float>();
                lingkong.closed_angle_deg = hardware["closed_angle_deg"].as<float>();
                lingkong.calibrated = hardware["calibrated"].as<bool>();
                lingkong.speed_dps = hardware["speed_dps"].as<int>();
                lingkong.torque_limit_raw = hardware["torque_limit_raw"].as<int>();
                lingkong.max_speed_dps = hardware["max_speed_dps"].as<int>();
                lingkong.max_torque_limit_raw = hardware["max_torque_limit_raw"].as<int>();
                lingkong.min_angle_deg = hardware["min_angle_deg"].as<float>();
                lingkong.max_angle_deg = hardware["max_angle_deg"].as<float>();
                lingkong.max_temperature_c = hardware["max_temperature_c"].as<int>();
                lingkong.min_voltage_v = hardware["min_voltage_v"].as<float>();
                lingkong.max_voltage_v = hardware["max_voltage_v"].as<float>();
                lingkong.response_timeout_ms = hardware["response_timeout_ms"].as<int>();
                lingkong.telemetry_period_ms = hardware["telemetry_period_ms"].as<int>();
                lingkong.keepalive_ms = hardware["keepalive_ms"].as<int>();
                lingkong.command_watchdog_ms = hardware["command_watchdog_ms"].as<int>();
                gripper_ptr_vec.push_back(
                    std::make_unique<LingkongCanGripper>(gripper_name, lingkong)
                );
            }else if (gripper_type == "UartServoTwoFingerGripper"){
                int id = config["robot"]["gripper"][gri_i]["hardware"]["servo_id"].as<int>();''')
replace('            #ifdef DUAL_V1\n                gripper_ptr_vec.push_back(std::make_unique<InspireHandWrapper>(gripper_name, gri_i));\n            #else', '            #ifdef DUAL_V1\n                gripper_ptr_vec.push_back(std::make_unique<InspireHandWrapper>(gripper_name, gri_i));\n            #elif defined(DUAL_V2_2)\n                if (gri_i != 0) throw std::invalid_argument("Left hand must be gripper 0");\n                gripper_ptr_vec.push_back(std::make_unique<SeldonDisabledHand>(gripper_name));\n            #else')
# No write to the hand on startup. The existing UART driver opens the right
# gripper during explicit hardware initialization and reports its last command.
replace('            //Set gripper to open position\n            std::vector<float> cmd(gripper_ptr_vec[gri_i]->getJointSize(), 1.0f);\n            gripper_ptr_vec[gri_i]->setNormalizedPosition(cmd);', '            if (shutdown_all_threads) throw std::runtime_error("Gripper initialization failed");\n            auto* gripper = gripper_ptr_vec[gri_i].get();\n            if (gripper_type == "UartServoTwoFingerGripper") {\n                if (gripper->setNormalizedPosition({1.0f}) != RetCode::kSuccess)\n                    throw std::runtime_error("Right gripper initialization command failed");\n            }\n            std::vector<float> measured(gripper->getJointSize());\n            if (gripper->getPanelPos(measured) != RetCode::kSuccess)\n                throw std::runtime_error("Cannot read initial gripper state");\n            std::copy(measured.begin(), measured.end(), robot_state.GripperJointPos[gri_i]);')
# A settings-only heartbeat must not actuate end effectors; propagate I/O faults.
replace('    gripper_ptr->setPanelCommand(normalized_cmd);', '    if (!panel_command_ptr->need_setting_update &&\n        gripper_ptr->setPanelCommand(normalized_cmd) != RetCode::kSuccess) {\n        planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultGripperNotFound;\n        shutdown_all_threads = true;\n        return;\n    }')
replace('        std::copy(read_normalized_cmd.begin(), read_normalized_cmd.end(), robot_state.GripperJointPos[gripper_i]);\n    }\n}', '        std::copy(read_normalized_cmd.begin(), read_normalized_cmd.end(), robot_state.GripperJointPos[gripper_i]);\n    } else {\n        planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultGripperNotFound;\n        shutdown_all_threads = true;\n    }\n}')
replace('#include <thread>\n// #define SIMULATION', '''#include <thread>
#include <atomic>
#include <chrono>
#include "startup_reference.h"
static bool seldon_accept_current = false;
static bool seldon_simulation = true;
static bool seldon_init_failed = false;
static std::atomic<long long> seldon_last_packet{0};
static long long seldon_now() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
''')
replace('    std::signal(SIGINT, shutdown_signal_handler);', '''    std::string seldon_config = CONFIG_PREFIX_PATH;
    std::string seldon_encoder;
    for (int i=1; i<argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            std::cout << "Seldon rt_control --config-dir DIR --encoder-file FILE "
                "[--simulation|--hardware] [--startup-reference strict|accept-current]\\n";
            return 0;
        } else if (arg == "--hardware") seldon_simulation = false;
        else if (arg == "--simulation") seldon_simulation = true;
        else if (arg == "--config-dir" && i+1<argc) seldon_config = argv[++i];
        else if (arg == "--encoder-file" && i+1<argc) seldon_encoder = argv[++i];
        else if (arg == "--startup-reference" && i+1<argc) {
            const std::string mode = argv[++i];
            if (mode != "strict" && mode != "accept-current") return 2;
            seldon_accept_current = mode == "accept-current";
        } else { std::cerr << "Invalid argument: " << arg << std::endl; return 2; }
    }
    if (seldon_encoder.empty()) { std::cerr << "--encoder-file is required\\n"; return 2; }
    std::signal(SIGINT, shutdown_signal_handler);''')
replace('std::string config_prefix_path = CONFIG_PREFIX_PATH;', 'std::string config_prefix_path = seldon_config;')
replace('std::string encoder_last_position_filename = config_prefix_path + "/rt_control/encoder_last_position.bin";', 'std::string encoder_last_position_filename = seldon_encoder;')
replace('        read_config(config);', '''        config["rt_control"]["hardware_simulation"] = seldon_simulation;
        read_config(config);
        if (!seldon_simulation && !seldon_accept_current &&
            (!std::filesystem::exists(seldon_encoder) || std::filesystem::file_size(seldon_encoder) != 14*sizeof(float)))
            throw std::runtime_error("Missing/incomplete startup reference; select accept-current explicitly");
        if (!seldon_simulation && std::filesystem::exists(seldon_encoder))
            std::filesystem::copy_file(seldon_encoder, seldon_encoder + ".backup-" + std::to_string(seldon_now()));''')
start = s.index('void arm_update_encode_zero_offset(int arm_i){')
end = s.index('\nvoid read_config(', start)
s = s[:start]+'''void arm_update_encode_zero_offset(int arm_i){
    auto* arm = arm_ptr_vec[arm_i].get();
    const int n = arm->getJointSize();
    std::vector<float> position(n), velocity(n), first(n);
    // The driver opens its hardware here. Read repeatedly before accepting a reference.
    for (int sample=0; sample<20; ++sample) {
        if (arm->getState(&position, &velocity, nullptr) != RetCode::kSuccess)
            throw std::runtime_error("Cannot read startup encoder state");
        if (sample==0) first = position;
        for (int j=0; j<n; ++j) {
            if (!std::isfinite(position[j]) || !std::isfinite(velocity[j]) ||
                std::abs(velocity[j]) > 0.02 || std::abs(position[j]-first[j]) > 0.01)
                throw std::runtime_error("Startup reference requires finite, stationary joints");
        }
        sleep_period(10000);
    }
    for (int j=0; j<n; ++j) {
        robot_state.MotorPos[arm_i][j] = position[j];
        if (seldon_accept_current) {
            rt_config.encoder_last_position[arm_i][j] = position[j];
            std::cout << "[Seldon accept-current] arm=" << arm_i << " joint=" << j
                      << " reference=" << position[j] << " (encoder zero unchanged)\\n";
            continue;
        }
        auto delta = seldon::startup_delta(position[j], rt_config.encoder_last_position[arm_i][j]);
        if (!delta.matches)
            throw std::runtime_error("Startup Position Mismatch on arm " + std::to_string(arm_i)
                + " joint " + std::to_string(j) + "; use accept-current only after verifying physical pose");
        if (delta.laps && arm->updateEncoderZeroOffset(j, -2*PI*delta.laps) != RetCode::kSuccess)
            throw std::runtime_error("Failed to restore encoder turn offset");
    }
}
''' + s[end:]
replace('            arm_update_encode_zero_offset(arm_i);', '            if (shutdown_all_threads) throw std::runtime_error("Arm initialization failed");\n            arm_update_encode_zero_offset(arm_i);')
replace('            transmit_panel_command_ptr = std::move(temp_ptr);', '            transmit_panel_command_ptr = std::move(temp_ptr);\n            seldon_last_packet.store(seldon_now());')
# No driver command may continue indefinitely after the upper process or network disappears.
replace('    long t0 = get_time_now();\n    planner_command_ptr->connection_state', '    long t0 = get_time_now();\n    const auto seldon_started = seldon_now();\n    planner_command_ptr->connection_state')
replace('        //Update panel command\n', '''        const auto last_packet = seldon_last_packet.load();
        if ((last_packet && seldon_now()-last_packet > 500) ||
            (!last_packet && seldon_now()-seldon_started > 10000)) {
            std::cerr << "[Seldon] Panel watchdog expired; closing RT hardware\\n";
            shutdown_all_threads = true;
            break;
        }
        //Update panel command
''')
# The original helper reads shared unique_ptr outside its mutex; remove that race in the overlay.
replace('''        if (transmit_panel_command_ptr){
            std::lock_guard<std::mutex> lock(planel_command_mutex);
            panel_command_ptr = std::move(transmit_panel_command_ptr);
        }''', '''        {
            std::lock_guard<std::mutex> lock(planel_command_mutex);
            if (transmit_panel_command_ptr) panel_command_ptr = std::move(transmit_panel_command_ptr);
        }''')
replace('        planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultUnknown;', '        seldon_init_failed = true;\n        if (planner_state_ptr) planner_state_ptr->system_diagnostic_flags |= DiagnosticFlags::kFaultUnknown;')
replace('    printf("Terminated all threads, exit program.\\n");\n    return 0;', '''    for (auto& gripper : gripper_ptr_vec) {
        if (gripper && dynamic_cast<LingkongCanGripper*>(gripper.get()))
            gripper.release();
    }
    printf("Terminated all threads, exit program.\\n");
    return seldon_init_failed ? 1 : 0;''')
replace('    udp_ptr->close();', '    if (udp_ptr) udp_ptr->close();')
replace('        initialize_gripper(config, project_base_path);', '        initialize_gripper(config, project_base_path);\n        planner_ptr->updatePlannerState(robot_state);\n        for (int g=0; g<robot_state.GripperSize; ++g)\n            std::copy(robot_state.GripperJointPos[g], robot_state.GripperJointPos[g]+robot_state.GripperJointSize[g], planner_state_ptr->GripperJointPos[g]);')
# Redirect logs and encoder state into Seldon, preserving the legacy hardware configuration path.
replace('initialize_log_file(project_base_path+"/build/");', 'initialize_log_file(std::filesystem::path(seldon_encoder).parent_path().string()+"/");')
# Guard null panel during a startup/watchdog exit (set_and_read_arm already has this guard).
(out/'rt_control.cpp').write_text(s)
flags = (build/'CMakeFiles/dual_v2_2_rt_control.dir/flags.make').read_text()
values = dict(line.split(' = ',1) for line in flags.splitlines() if ' = ' in line)
cmd = ['/usr/bin/c++']
for key in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS'):
    cmd += shlex.split(values[key])
cmd += ['-I'+str(root/'rt'), '-I'+str(root/'rt_lingkong'),
        '-c', str(out/'rt_control.cpp'), '-o', str(out/'rt_control.o')]
cmd += ['-I'+str(root/'config_sources/V2/dual_v2_2_control/cuarm_rt_control/hardware_modules/eyou-main/include'),
        '-I'+str(root/'config_sources/V2/dual_v2_2_control/cuarm_rt_control/hardware_modules/eyou-main/include/x86')]
subprocess.run(cmd, check=True, cwd=build)
hand_objects = []
for name in ('inspire_hand', 'hand_can_transport'):
    obj = out/(name+'.o')
    hand_cmd = cmd[:]
    hand_cmd[hand_cmd.index('-c')+1] = str(hand_root/'src/motor_sensor'/(name+'.cpp'))
    hand_cmd[hand_cmd.index('-o')+1] = str(obj)
    subprocess.run(hand_cmd, check=True, cwd=build)
    hand_objects.append(str(obj))
lingkong_object = out/'lingkong_can_gripper.o'
lingkong_cmd = cmd[:]
lingkong_cmd[lingkong_cmd.index('-c')+1] = str(root/'rt_lingkong'/'lingkong_can_gripper.cpp')
lingkong_cmd[lingkong_cmd.index('-o')+1] = str(lingkong_object)
subprocess.run(lingkong_cmd, check=True, cwd=build)
link = shlex.split((build/'CMakeFiles/dual_v2_2_rt_control.dir/link.txt').read_text())
link = [str(out/'rt_control.o') if x.endswith('/example/rt_control.cpp.o') else x for x in link]
link[1:1] = hand_objects + [str(lingkong_object)]
link[link.index('-o')+1] = str(out/'seldon_rt_control.new')
subprocess.run(link, check=True, cwd=build)
(out/'seldon_rt_control.new').replace(out/'seldon_rt_control')
(out/'manifest.json').write_text(json.dumps({
    'legacy_source': str(source),
    'legacy_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'adapter_config': str(root/'rt_lingkong'/'right_gripper.yaml'),
}, indent=2))
print(out/'seldon_rt_control')
