# this launch file handles launching Nav2 in the right order, with support for
# GPS coordinate mapping.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    pkg_simulator: str = get_package_share_directory("simulator")

    # first up, and most importantly, we MUST have navsat_transform.
    #
    # this node actually comes from `robot_localization`, but we use it to work
    # with GPS coordinates without wanting to scrape our eyes of their sockets...
    navsat_transform_node: Node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        parameters=[
            {
                "from_file": [
                    PathJoinSubstitution(
                        [
                            pkg_drive_launcher,
                            "params",
                            "navsat_transform.yaml",
                        ]
                    )
                ]
            }
        ],
        respawn=True,
    )

    # now we can launch the local odometry ekf filter (for imu).
    #
    # (odom -> base_link) tf
    local_ekf_node: Node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        parameters=[
            PathJoinSubstitution([pkg_drive_launcher, "params", "local_odom.yaml"]),
        ],
        respawn=True,
    )

    # launch the global odometry ekf filter (for gps).
    #
    # (map -> odom) tf
    global_ekf_node: Node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        parameters=[
            PathJoinSubstitution([pkg_drive_launcher, "params", "global_odom.yaml"]),
        ],
        respawn=True,
    )

    # start the `slam_toolbox` to get a `map` :)
    slam_toolbox: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "start_slam_toolbox.py",
                    ]
                )
            ]
        ),
    )

    # and the `depthimage_to_laserscan_node` provides the map with a pointcloud
    # to work on
    depthimage_to_laserscan: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "start_depth_cam_laser_mapping.py",
                    ]
                )
            ]
        ),
    )

    # we'll also want the `robot_state_publisher`.
    #
    # this node says where things are on the rover in relation to one another,
    # which is required for consistent mapping, navigation and object
    # avoidance.
    robot_state_publisher: IncludeLaunchDescription = _robot_state_publisher()

    # use node composition on Nav2 to speed things up!
    #
    # see: https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html#component-container-types
    nav2_container: Node = Node(
        package="rclcpp_components",
        executable="component_container",
        name="nav2_container",
        namespace="",
        parameters=[{"use_sim_time": False}],
    )
    # launch the rest of Nav2 using our helper script
    nav2_helper: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "start_nav2.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": PathJoinSubstitution(
                [
                    pkg_simulator,
                    "params",
                    "nav2.yaml",
                ]
            ),
            "namespace": "",
            "container_name": "nav2_container",
        }.items(),
    )

    # provides ways to control the rover
    ros2_control: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "start_ros2_control.py",
                    ]
                )
            ]
        ),
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            depthimage_to_laserscan,
            slam_toolbox,
            navsat_transform_node,
            local_ekf_node,
            global_ekf_node,
            nav2_container,
            nav2_helper,
            ros2_control,
        ]
    )


def _robot_state_publisher() -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "robot_state_publisher.launch.py",
                    ]
                )
            ]
        ),
    )
