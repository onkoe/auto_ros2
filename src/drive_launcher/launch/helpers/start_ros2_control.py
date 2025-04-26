"""
This file starts everything needed for `ros2_control`.

See its source here:
https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/controller_manager/spawner.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    controller_manager: Node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "ros2_control",
                    "controllers.yaml",
                ]
            ),
        ],
        output="screen",
    )

    # spawn all the ros2_control... controllers:
    #
    # see: https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html#helper-scripts
    spawn_joint_state_broadcaster: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    spawn_left: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_wheels_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    spawn_right: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_wheels_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            controller_manager,
            spawn_joint_state_broadcaster,
            spawn_left,
            spawn_right,
        ]
    )
