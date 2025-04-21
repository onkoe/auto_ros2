from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from navigator_node.types import NavigationMode


def generate_launch_description():
    pkg_simulator: str = get_package_share_directory("simulator")

    # launch all the gazebo stuff
    gazebo_launch_file: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_simulator,
                        "launch",
                        "gazebo_only.launch.py",
                    ]
                )
            ],
        )
    )

    # add some coordinate to go to...
    latitude = 0.0002
    longitude = 0.0002

    mode_int: int = NavigationMode.ARUCO.value

    # make an instance of the navigator node!
    navigator: Node = Node(
        executable="navigator_node",
        package="navigator_node",
        name="navigator",
        parameters=[{"longitude": longitude, "latitude": latitude, "mode": mode_int}],
    )

    # turn on debug logs
    log_setting: DeclareLaunchArgument = DeclareLaunchArgument(
        name="log_level",
        default_value="debug",
        description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
    )

    return LaunchDescription(
        [
            gazebo_launch_file,
            navigator,
            log_setting,
        ],
    )
