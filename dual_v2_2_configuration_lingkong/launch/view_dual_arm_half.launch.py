from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CURRENT_DIR = Path(__file__).resolve().parent
CONFIG_DIR = CURRENT_DIR.parent
DEFAULT_URDF = CONFIG_DIR / "share" / "left_hand_right_gripper_half" / "dual_arm_half.urdf"
DEFAULT_RVIZ_CONFIG = CONFIG_DIR / "rviz" / "tf_model.rviz"


def resolve_mesh_uris(urdf_text: str, urdf_file: Path) -> str:
    """Rewrite relative ../meshes/ paths to file:// URIs for RViz2.

    RViz treats bare relative paths as network URIs (host='..'), which fails with
    'Could not resolve host: ..'. CuRobo/MuJoCo can keep using relative paths in
    the on-disk URDF; only the published robot_description is rewritten.
    """
    meshes_dir = (urdf_file.parent / "../meshes").resolve()
    return urdf_text.replace("../meshes/", f"{meshes_dir.as_uri()}/")


def launch_setup(context, *args, **kwargs):
    urdf_file = Path(LaunchConfiguration("urdf_file").perform(context)).resolve()
    rviz_config = Path(LaunchConfiguration("rviz_config").perform(context)).resolve()

    robot_description = resolve_mesh_uris(urdf_file.read_text(), urdf_file)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "publish_frequency": 50.0,
                    "use_sim_time": False,
                }
            ],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            arguments=[str(urdf_file)],
            condition=IfCondition(LaunchConfiguration("use_gui")),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            arguments=[str(urdf_file)],
            condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(rviz_config)],
            cwd=str(urdf_file.parent),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_file",
                default_value=str(DEFAULT_URDF),
                description="Absolute path to the URDF file to visualize.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(DEFAULT_RVIZ_CONFIG),
                description="RViz config with RobotModel and TF displays enabled.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="false",
                description="Start joint_state_publisher_gui for moving non-fixed joints.",
            ),
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="true",
                description="Start non-GUI joint_state_publisher instead of the GUI.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
