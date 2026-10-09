import os
import platform
import sys
import time
import traceback
from importlib.machinery import EXTENSION_SUFFIXES
from pathlib import Path

LOCALHOST = "127.0.0.1"

PLANNER_ROOT = Path(__file__).resolve().parent
SCRIPT_PATH = PLANNER_ROOT / "scripts"
CONFIG_ROOT = Path(
    os.environ.get(
        "CUARM_CONFIG_ROOT",
        "/mnt/ssd/user/V/dual_v2_2_configuration",
    )
).expanduser().resolve()
CONFIG_PATH = CONFIG_ROOT / "config.yaml"
UDP_REPO_PATH = CONFIG_ROOT / "curi_udp" / "python"
ROBOT_PLATFORM_UTILS_PATH = (
    CONFIG_ROOT / "robot_platform_utils" / "python" / "scripts"
)

machine = platform.machine().lower()
if machine in ("x86_64", "amd64"):
    CONFIG_LOADER_PATH = (
        CONFIG_ROOT / "robot_platform_utils" / "python" / "lib" / "x86"
    )
elif machine in ("aarch64", "arm64"):
    CONFIG_LOADER_PATH = (
        CONFIG_ROOT / "robot_platform_utils" / "python" / "lib" / "arm64"
    )
else:
    raise RuntimeError(f"Unsupported architecture: {machine}")

required_paths = {
    "CuRobo planner": SCRIPT_PATH / "curobo_planner.py",
    "CuRobo configuration": SCRIPT_PATH / "curobo_config.py",
    "CuRobo IK solver": SCRIPT_PATH / "curobo_ik_solver.py",
    "configuration": CONFIG_PATH,
    "UDP module": UDP_REPO_PATH / "udp_socket.py",
    "robot platform utilities": ROBOT_PLATFORM_UTILS_PATH / "cuarm_state.py",
}
missing_paths = [
    f"{name}: {path}" for name, path in required_paths.items() if not path.exists()
]
config_loader_module = next(
    (
        CONFIG_LOADER_PATH / f"config_loader{suffix}"
        for suffix in EXTENSION_SUFFIXES
        if (CONFIG_LOADER_PATH / f"config_loader{suffix}").is_file()
    ),
    None,
)
if config_loader_module is None:
    missing_paths.append(
        "config loader ABI: "
        f"{CONFIG_LOADER_PATH}/config_loader{EXTENSION_SUFFIXES[0]}"
    )
if missing_paths:
    raise RuntimeError(
        "Missing planner dependencies:\n  " + "\n  ".join(missing_paths)
    )

# Keep the planner self-contained while sharing the same configuration and
# protocol implementation as Upper Software and RT Control.
os.environ.setdefault("CUARM_CONFIG_ROOT", str(CONFIG_ROOT))
sys.path[:0] = [
    str(SCRIPT_PATH),
    str(UDP_REPO_PATH),
    str(ROBOT_PLATFORM_UTILS_PATH),
    str(CONFIG_LOADER_PATH),
]


def read_positive_float_env(name, default):
    raw = os.environ.get(name, str(default))
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a number, got {raw!r}") from exc
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")
    return value


def validate_udp_endpoint(ip, port, label):
    if not isinstance(ip, str) or not ip.strip():
        raise ValueError(f"{label} IP must be a non-empty string")
    try:
        port = int(port)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} port must be an integer, got {port!r}") from exc
    if not 1 <= port <= 65535:
        raise ValueError(f"{label} port out of range: {port}")
    return ip.strip(), port


def remote_page_endpoints(config):
    try:
        remote_page = config.config["panel"]["remote_page"]
        upper_local = validate_udp_endpoint(
            remote_page["default_local_ip"],
            remote_page["default_local_port"],
            "Upper Remote local",
        )
        upper_remote = validate_udp_endpoint(
            remote_page["default_remote_ip"],
            remote_page["default_remote_port"],
            "Upper Remote peer",
        )
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "Missing panel.remote_page UDP settings in configuration"
        ) from exc
    return upper_local, upper_remote

from curobo_planner import CuroboPlanner
from curobo_config import CuroboConfig

#Add from cuarm_configuration repo
from cuarm_state import PanelTargetMode, PanelState, RemotePanelCommand, RobotState, \
    ControlType, PanelRemoteState, TargetCommandPlannerMode, TargetCommandArmCommandType, \
    PlannerConnectionMethod, PlannerCuroboSolverType
from cuarm_udp import CuarmUdp, CuarmUdpThread
from cuarm_state_initializers import get_initialized_remote_panel_command
from cuarm_utils import get_arm_target_mode, \
    convert_panel_state_to_robot_state, convert_curobo_command_to_remote_panel_command

def create_curobo_planner(config, solver_type):
    target_cartesian_position_precision = 0.005
    solver_position_threshold = 0.001
    target_cartesian_orientation_precision = solver_rotation_threshold = 0.1
    if solver_type == PlannerCuroboSolverType.IK_SOLVER:
        return CuroboPlanner(
            config,
            connection_method=PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT,
            solver_type=PlannerCuroboSolverType.IK_SOLVER,
            catch_SIGINT=False,
            solver_position_threshold=solver_position_threshold,
            solver_rotation_threshold=solver_rotation_threshold,
            target_cartesian_position_precision=target_cartesian_position_precision,
            target_cartesian_orientation_precision=target_cartesian_orientation_precision,
            soft_limit_k=0.99,
            gripper_output_urdf_order=True,
            gripper_output_normalized=True,
            gripper_output_zero_open=True,
            num_ik_seeds=1,
            self_collision_check=True,
            self_collision_opt=True,
            ik_particle_opt=False,
            use_cuda_graph=True,
        )
    if solver_type == PlannerCuroboSolverType.MOTION_GEN:
        trim_steps = [2, -2]
        trajopt_tsteps = 10
        time_dilation_factor = 0.4
        return CuroboPlanner(
            config,
            connection_method=PlannerConnectionMethod.REMOTE_TARGET_AND_LOCAL_ROBOT,
            solver_type=PlannerCuroboSolverType.MOTION_GEN,
            catch_SIGINT=False,
            time_dilation_factor=time_dilation_factor,
            trajopt_tsteps=trajopt_tsteps,
            trim_steps=trim_steps,
            solver_position_threshold=solver_position_threshold,
            solver_rotation_threshold=solver_rotation_threshold,
            target_cartesian_position_precision=target_cartesian_position_precision,
            target_cartesian_orientation_precision=target_cartesian_orientation_precision,
            soft_limit_k=0.99,
            gripper_output_urdf_order=True,
            gripper_output_normalized=True,
            gripper_output_zero_open=True,
            use_cuda_graph=True,
        )
    raise ValueError(f"Unsupported CuRobo solver type: {solver_type}")


def get_dual_v2_2_setup():
    print("Load config from: ", CONFIG_PATH)
    config = CuroboConfig.load_from_yaml_path(str(CONFIG_PATH))

    target_type = ControlType.POSITION
    actuator_mode = ControlType.POSITION
    upper_local, upper_remote = remote_page_endpoints(config)
    planner_local = upper_remote
    planner_remote = upper_local
    print(
        f"Remote panel config: upper local {upper_local[0]}:{upper_local[1]}, "
        f"upper remote {upper_remote[0]}:{upper_remote[1]}"
    )
    print(
        f"Planner panel UDP: local {planner_local[0]}:{planner_local[1]}, "
        f"remote {planner_remote[0]}:{planner_remote[1]}"
    )
    return config, target_type, actuator_mode, planner_local, planner_remote


def validate_control_mode(remote_panel_cmd, planner):
    if remote_panel_cmd.target_type == ControlType.POSITION:
        return
    if planner.target_arm_type == TargetCommandArmCommandType.CARTESIAN:
        raise ValueError(
            "Cartesian commands are not supported for non-position control"
        )
    unsupported_modes = {
        TargetCommandPlannerMode.PLANNING,
        TargetCommandPlannerMode.NO_PLANNING,
        TargetCommandPlannerMode.COLLISION_ONLY,
    }
    if planner.planner_mode in unsupported_modes:
        raise ValueError(
            "Planning and collision checking require position control"
        )


def terminate_safely(component, name):
    if component is None:
        return
    try:
        component.terminate()
    except Exception as exc:
        print(f"Warning: failed to terminate {name}: {exc}", file=sys.stderr)


def main():
    robot_recv_delay_ms = 10
    target_recv_delay_ms = 30
    delay_ms = 50
    panel_state_timeout_s = read_positive_float_env(
        "CUARM_PANEL_STATE_TIMEOUT_S", 0.5
    )
    target_local = validate_udp_endpoint(
        os.environ.get("CUARM_TARGET_LOCAL_IP", LOCALHOST),
        os.environ.get("CUARM_TARGET_LOCAL_PORT", "10166"),
        "Planner target local",
    )
    target_remote = validate_udp_endpoint(
        os.environ.get("CUARM_TARGET_REMOTE_IP", LOCALHOST),
        os.environ.get("CUARM_TARGET_REMOTE_PORT", "10165"),
        "Planner target peer",
    )

    planner = None
    panel_udp = None
    try:
        (
            config,
            target_type,
            actuator_mode,
            panel_local,
            panel_remote,
        ) = get_dual_v2_2_setup()

        solver_type = PlannerCuroboSolverType.MOTION_GEN
        planner = create_curobo_planner(config, solver_type)

        print(
            f"Planner target UDP: local {target_local[0]}:{target_local[1]}, "
            f"remote {target_remote[0]}:{target_remote[1]}"
        )
        planner.connect_udp_to_target(
            target_local[0],
            target_local[1],
            target_remote[0],
            target_remote[1],
            recv_delay_ms=target_recv_delay_ms,
        )

        panel_udp = CuarmUdpThread(
            panel_local[0],
            panel_local[1],
            panel_remote[0],
            panel_remote[1],
            CuarmUdp.unpack_panel_state,
            CuarmUdp.pack_remote_panel_command,
            recv_delay_ms=robot_recv_delay_ms,
        )
        remote_panel_cmd: RemotePanelCommand = (
            get_initialized_remote_panel_command(
                target_type=target_type,
                actuator_mode=actuator_mode,
                arm_size=config.arm_size,
                arm_joint_size=config.arm_joint_size,
                gripper_size=config.gripper_size,
                gripper_joint_size=config.gripper_joint_size,
            )
        )

        zero_vel_threshold = 0.005
        pos_threshold = 0.00174533
        robot_state = RobotState()
        panel_state_seen = False
        panel_timeout_latched = False
        last_panel_wait_log = 0.0
        print(
            "Curobo is ready. PanelState timeout: "
            f"{panel_state_timeout_s:.3f}s"
        )

        while True:
            start_time = time.perf_counter()
            panel_state: PanelState = panel_udp.receive(
                max_age_s=panel_state_timeout_s
            )
            if panel_state is None:
                now = time.monotonic()
                if now - last_panel_wait_log > 2.0:
                    print(
                        "Waiting for fresh PanelState on UDP "
                        f"{panel_local[1]}<-{panel_remote[1]}. "
                        "Check Upper Remote Connect/Start."
                    )
                    last_panel_wait_log = now
                if panel_state_seen and not panel_timeout_latched:
                    print(
                        "PanelState timed out; clearing the active target and "
                        "stopping panel command output.",
                        file=sys.stderr,
                    )
                    if planner.curobo_cmd is not None:
                        planner.clear_curobo_command()
                    panel_timeout_latched = True
            else:
                if panel_timeout_latched:
                    print("Fresh PanelState restored.")
                panel_state_seen = True
                panel_timeout_latched = False
                convert_panel_state_to_robot_state(
                    robot_state,
                    panel_state,
                    remote_panel_cmd,
                    zero_vel_threshold=zero_vel_threshold,
                    pos_threshold=pos_threshold,
                )

                if panel_state.state == PanelRemoteState.STOP:
                    planner.receive(robot_state=robot_state)
                    planner.clear_curobo_command()
                else:
                    curobo_cmd, _ = planner.receive_and_compute(
                        robot_state=robot_state
                    )
                    if planner.last_received_target_command is not None:
                        validate_control_mode(remote_panel_cmd, planner)
                        arm_target_mode = get_arm_target_mode(
                            solver_type, planner.planner_mode
                        )
                        is_preview = str(
                            getattr(
                                planner.last_received_target_command,
                                "description",
                                "",
                            )
                        ).startswith("__preview__:")
                        if not is_preview and curobo_cmd is not None:
                            convert_curobo_command_to_remote_panel_command(
                                remote_panel_cmd,
                                curobo_cmd,
                                planner.last_received_target_command,
                                arm_target_mode,
                            )
                            panel_udp.send(remote_panel_cmd)

            process_time = time.perf_counter() - start_time
            sleep_time = delay_ms / 1000 - process_time
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("Ctrl+C detected. Exiting...")
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        terminate_safely(panel_udp, "panel UDP")
        terminate_safely(planner, "CuRobo planner")


if __name__ == "__main__":
    sys.exit(main())
