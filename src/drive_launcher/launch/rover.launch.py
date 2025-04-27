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
    # this collection of nodes is from the `robot_localization` package. these
    # provide essential functionality for the Rover.
    #
    # together, they work to translate known positions from our (hopefully
    # accurate) URDF model of the Rover into offsets.
    #
    # `robot_localization::navsat_transform_node` offsets the recv'd GPS
    # coordinates such that they are local to the Rover's origin.
    #
    # next up, we have two `robot_localization::ekf_node` instances...
    #
    # the first is `ekf_filter_node_odom`, which reads from the IMU and
    # converts its data into something usable on the frame (in other words,
    # local "odometry" data). other nodes use that info for state estimation!
    #
    # on the other hand, we've got `ekf_filter_node_map`. it uses info from the
    # `navsat_transform_node` and `ekf_filter_node_odom` to make a "global"
    # representation of the map. it's used by the planners (and, as a result,
    # directly by our Navigator).
    (navsat_transform_node, ekf_filter_node_odom, ekf_filter_node_map) = (
        _robot_localization()
    )

    # the `depthimage_to_laserscan::depthimage_to_laserscan_node` gives us
    # another source of `/tf:map` data.
    #
    # it converts our depth camera data (from the Zed 2i) into usable point
    # clouds, which you'd otherwise have to get from a laser scanner.
    #
    # our Rover doesn't have one of those..! so, for object avoidance, we use
    # this node to add boundaries to our map.
    depthimage_to_laserscan: IncludeLaunchDescription = _depthimage_to_laserscan()

    # the `slam_toolbox::async_slam_toolbox_node` fills the `/tf:map` and
    # `tf:odom` frames with our translated laserscan data.
    slam_toolbox: IncludeLaunchDescription = _slam_toolbox()

    # we'll also want the `robot_state_publisher::robot_state_publisher` node.
    #
    # it says where things are on the Rover in relation to one another, which
    # is required for consistent mapping, good navigation, and object
    # avoidance.
    robot_state_publisher: IncludeLaunchDescription = _robot_state_publisher()

    # Nav2 provides navigation utilties to the SoRo Navigator and beyond.
    #
    # it's primarily helpful for its provided algorithms that are really
    # difficult (and verbose) to implement by hand. you control it through the
    # various actions its plugins provide during navigation.
    (nav2_container, nav2_launch) = _nav2()

    # `ros2_control` is a collection of nodes that we use to control the Rover.
    #
    # it recvs instructions from any node, but in practice, it'll always come
    # directly from the Nav2 stack directing the robot where to go (and how not
    # to get stuck on giant cliff boulders).
    #
    # the nodes from this package are particularly useful in simulating and
    # visualizing the Rover, as they provide simple `tf2` transforms on the
    # wheels as they spin.
    #
    # we currently do not use this package for directly manipulating the
    # wheels, as doing so would require writing a `ros2_control` hardware
    # plugin.
    #
    # that'd be an extremely painful task with our microcontroller setup, as
    # there are no cross-platform networking APIs in the C++ stdlib. note that
    # `boost` could be an option in the future if future Autonomous members
    # would like to move toward a `ros2_control`-based approach.
    ros2_control: IncludeLaunchDescription = _ros2_control()

    return LaunchDescription(
        [
            navsat_transform_node,
            robot_state_publisher,
            ekf_filter_node_odom,
            ekf_filter_node_map,
            depthimage_to_laserscan,
            slam_toolbox,
            nav2_container,
            nav2_launch,
            ros2_control,
        ]
    )


def _robot_localization() -> tuple[Node, Node, Node]:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    # offset gps coordinates by correct amount
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
                            "robot_localization",
                            "navsat_transform.yaml",
                        ]
                    )
                ]
            }
        ],
        respawn=True,
    )

    # local odometry ekf filter (for imu)...
    #
    # (odom -> base_link) tf
    ekf_filter_node_odom: Node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        parameters=[
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "robot_localization",
                    "local_odom.yaml",
                ]
            ),
        ],
        respawn=True,
    )

    # global odometry ekf filter (for gps)...
    #
    # (map -> odom) tf
    ekf_filter_node_map: Node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        parameters=[
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "robot_localization",
                    "global_odom.yaml",
                ]
            ),
        ],
        respawn=True,
    )

    return (navsat_transform_node, ekf_filter_node_odom, ekf_filter_node_map)


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


def _slam_toolbox() -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "slam_toolbox.launch.py",
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


def _nav2() -> tuple[Node, IncludeLaunchDescription]:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

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
    nav2_launch: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "nav2.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "nav2.yaml",
                ]
            ),
            "namespace": "",
            "container_name": "nav2_container",
        }.items(),
    )

    return (nav2_container, nav2_launch)


def _depthimage_to_laserscan() -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "depthimage_to_laserscan.launch.py",
                    ]
                )
            ]
        ),
    )


def _ros2_control() -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "ros2_control.launch.py",
                    ]
                )
            ]
        ),
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
