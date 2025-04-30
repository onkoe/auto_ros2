from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    starts the `slam_toolbox` mapping node (to get the `/map` topic)
    """
    pkg_slam_toolbox: FindPackageShare = FindPackageShare("slam_toolbox")

    use_sim_time: LaunchConfiguration = LaunchConfiguration(
        "use_sim_time", default="false"
    )

    # we need this little node to make a link between `camera_link` and
    # `camera_depth_frame`
    camera_chain: Node = _make_camera_depth_frame_link_node(use_sim_time)

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_slam_toolbox,
                        "launch",
                        "online_async_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("slam_params_file", _get_conf()),
            ("use_lifecycle_manager", "true"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            camera_chain,
            slam_toolbox,
        ]
    )


def _get_conf() -> PathJoinSubstitution:
    return PathJoinSubstitution(
        [
            FindPackageShare("drive_launcher"),
            "params",
            "slam_toolbox.yaml",
        ]
    )


def _make_camera_depth_frame_link_node(
    use_sim_time: LaunchConfiguration,
) -> Node:
    return Node(
        name="tf2_publish_camera_depth_frame_link_node",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",  # x
            "0",  # y
            "0",  # z
            "0",  # rx
            "0",  # ry
            "0",  # rz
            "camera_link",  # parent frame
            "camera_depth_frame",  # subframe
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
