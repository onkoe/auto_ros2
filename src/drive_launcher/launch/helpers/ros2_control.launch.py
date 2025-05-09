"""
This file starts everything needed for `ros2_control`.

See its source here:
https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/controller_manager/spawner.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_conf: PathJoinSubstitution = _get_controllers_conf()

    # unfortunately, this dumb Node has to exist until we upgrade to Jazzy or
    # Kilted due to limitations in the Humble version of the
    # `controller_manager::spawner` Node.
    #
    # that's because the `--controller-ros-args` flag on that Node won't exist
    # until Jazzy! D:
    cmd_vel_remapping_relay = Node(
        name="cmdvel_relay",
        executable="relay",
        package="topic_tools",
        arguments=[
            "/cmd_vel",
            "/diff_drive_controller/cmd_vel_unstamped",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # spawn the ros2_control... controllers:
    #
    # see: https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html#helper-scripts
    spawn_joint_state_broadcaster: Node = Node(
        name="joint_state_broadcaster_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
    spawn_diff_drive: Node = Node(
        package="controller_manager",
        name="diff_drive_controller_spawner",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            controllers_conf,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
            ("/diff_drive_controller/odom", "/odom"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            spawn_joint_state_broadcaster,
            #
            # very important! only spawn controllers when the broadcast guy is up
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_joint_state_broadcaster,
                    on_exit=[cmd_vel_remapping_relay],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=spawn_joint_state_broadcaster,
                    on_start=[spawn_diff_drive],
                )
            ),
        ]
    )


def _get_controllers_conf() -> PathJoinSubstitution:
    return PathJoinSubstitution(
        [
            FindPackageShare("drive_launcher"),
            "params",
            "ros2_control",
            "controllers.yaml",
        ]
    )
