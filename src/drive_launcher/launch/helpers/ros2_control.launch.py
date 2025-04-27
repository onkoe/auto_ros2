"""
This file starts everything needed for `ros2_control`.

See its source here:
https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/controller_manager/spawner.py
"""

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    controllers_conf: PathJoinSubstitution = _get_controllers_conf()

    controller_manager: Node = Node(
        name="controller_manager",
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_conf],
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
        ],
    )
    spawn_left: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_wheels_velocity_controller",
            "--param-file",
            controllers_conf,
        ],
    )
    spawn_right: Node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_wheels_velocity_controller",
            "--param-file",
            controllers_conf,
        ],
    )

    return LaunchDescription(
        [
            controller_manager,
            spawn_joint_state_broadcaster,
            #
            # very important! only spawn controllers when the broadcast guy is up
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_joint_state_broadcaster,
                    on_exit=[spawn_left, spawn_right],
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
