import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
)
from navigator_node.types import NavigationMode


def generate_launch_description():
    pkg_simulator: str = get_package_share_directory("simulator")

    # generate the rover model from its macro file
    rover_state_publisher_node: Node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            "xacro ",
                            PathJoinSubstitution(
                                [
                                    pkg_simulator,
                                    "resource",
                                    "rover.urdf.xacro.xml",
                                ]
                            ),
                        ]
                    ),
                    value_type=str,
                )
            }
        ],
    )

    # spawn the rover model
    rover_model_spawner: Node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "remi",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    # bridge for ros 2 <---> gazebo topics, minus cameras
    bridge_params: str = os.path.join(
        pkg_simulator,
        "params",
        "bridge.yaml",
    )
    bridge: Node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # a custom bridge to use our message types.
    #
    # yea... it's goin through three layers lol
    soro_bridge: Node = Node(package="simulator", executable="simulator")

    # spawn the gazebo server
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )
    gz_server: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution(
                    [pkg_simulator, "resource", "world.sdf.xml"]
                ),
                " -r",
            ],
            "on_exit_shutdown": "True",
        }.items(),
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
        parameters=[
            {"longitude": longitude, "latitude": latitude, "mode": mode_int}
        ],
    )

    # turn on debug logs
    log_setting: DeclareLaunchArgument = DeclareLaunchArgument(
        name="log_level",
        default_value="warn",
        description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
    )

    return LaunchDescription(
        [
            bridge,
            navigator,
            gz_server,
            soro_bridge,
            rover_state_publisher_node,
            rover_model_spawner,
            log_setting,
        ],
    )
